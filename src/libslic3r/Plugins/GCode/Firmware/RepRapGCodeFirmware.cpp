///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "RepRapGCodeFirmware.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

/*
RepRapFirmware implementation
=============================

`RepRapGCodeFirmwareSession` keeps the common PrintingPlan traversal and
machine-state bookkeeping, then translates the resulting state changes to
RepRapFirmware's command vocabulary.

The specialization flow is:

    resolve_empty_script()
    |-- emit `M600` for a color-change event
    |-- emit `M226` for a pause-print event
    `-- delegate other script types to the common session

    encode_machine_envelope()
    |-- emit acceleration limits with `M201`
    |-- convert feedrates and minimum extruding feedrate to mm/min for `M203`
    |-- emit print/travel acceleration with `M204`
    `-- emit jerk limits with `M566`, also in mm/min

    encode fan and pressure-advance changes
    `-- use RepRapFirmware's tool-oriented command parameters

The neutral `MachineEnvelope` remains in mm/sec-based units until this class
performs the dialect-boundary conversion. State transitions, extrusion values,
and script processing stay in the inherited session. This class only supplies
RepRapFirmware encodings and is instantiated by the common firmware provider.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Firmware {

namespace {

// Convert a Reader scalar back to the integer register used by Gantry. RRF
// accepts decimal input, while generated acceleration commands are integral;
// round exactly as the firmware encoder does and reject impossible values.
std::optional<uint32_t> acceleration_register(const std::optional<double> &value)
{
    if (!value)
        return std::nullopt;
    if (!std::isfinite(*value) || *value < 0.0 ||
        *value > double(std::numeric_limits<uint32_t>::max()))
        throw std::invalid_argument("RepRapFirmware returned an invalid acceleration register.");
    return static_cast<uint32_t>(std::lround(*value));
}

} // namespace

void RepRapGCodeFirmwareSession::setup_firmware(const Config &config)
{
    DefaultGCodeFirmwareSession::setup_firmware(config);
    m_reader_state = {};
}

void RepRapGCodeFirmwareSession::append_reader_private_state(
    std::vector<raw_gcode_reader_event_item> &items) const
{
    // Until M563 supplies explicit relationships, built-in exports use the
    // conventional one-to-one tool/drive/heater/fan mapping. The snapshot is
    // copied back into `items`; no pointer owned by the temporary tracker escapes.
    for (const DefaultExtruder &extruder : extruders()) {
        Reader::RepRapToolState &tool = m_reader_state.tools[extruder.id()];
        if (tool.drives.empty())
            tool.drives.push_back(extruder.id());
        if (tool.heaters.empty())
            tool.heaters.push_back(extruder.id());
        if (tool.fans.empty())
            tool.fans.push_back(extruder.id());
    }

    raw_gcode_reader_state raw = {};
    raw.items = items.empty() ? nullptr : items.data();
    raw.item_count = static_cast<uint32_t>(items.size());
    GCodeStateTracker tracker(raw);
    m_reader_state.write_to(tracker, reader_state_id());
    const raw_gcode_reader_state &complete = tracker.state();
    items.assign(complete.items, complete.items + complete.item_count);
}

void RepRapGCodeFirmwareSession::synchronize_reader_private_state(
    const GCodeStateTracker &state)
{
    // Decode to a temporary first. A malformed private snapshot therefore
    // leaves both the cached RRF model and all encoded gantry registers intact.
    const Reader::RepRapGCodeReaderState decoded =
        Reader::RepRapGCodeReaderState::from_tracker(state, reader_state_id());
    gantry().synchronize_encoded_print_acceleration(
        acceleration_register(decoded.print_acceleration));
    gantry().synchronize_encoded_travel_acceleration(
        acceleration_register(decoded.travel_acceleration));
    for (DefaultExtruder &extruder : extruders()) {
        const std::map<uint32_t, Reader::RepRapToolState>::const_iterator tool =
            decoded.tools.find(extruder.id());
        if (tool == decoded.tools.end()) {
            extruder.pressure_advance().synchronize_encoded(std::nullopt);
            extruder.fan().synchronize_encoded_after_external_gcode(std::nullopt);
            continue;
        }

        std::optional<double> pressure;
        bool pressure_consistent = !tool->second.drives.empty();
        for (uint32_t drive : tool->second.drives) {
            const std::map<uint32_t, double>::const_iterator found =
                decoded.pressure_advance.find(drive);
            if (found == decoded.pressure_advance.end() ||
                (pressure && *pressure != found->second)) {
                pressure_consistent = false;
                break;
            }
            pressure = found->second;
        }
        extruder.pressure_advance().synchronize_encoded(
            pressure_consistent ? pressure : std::optional<double>());

        std::optional<double> fan;
        bool fan_consistent = !tool->second.fans.empty();
        for (uint32_t fan_id : tool->second.fans) {
            const std::optional<double> physical = state.fan_speed(fan_id);
            if (!physical || (fan && *fan != *physical)) {
                fan_consistent = false;
                break;
            }
            fan = physical;
        }
        extruder.fan().synchronize_encoded_after_external_gcode(
            fan_consistent ? fan : std::optional<double>());
        if (tool->second.retracted)
            synchronize_native_retraction(extruder.id(), *tool->second.retracted);
    }
    m_reader_state = decoded;
}

std::string RepRapGCodeFirmwareSession::resolve_empty_script(
    gcode_script_type script_type,
    const Config *producer_config) const
{
    if (script_type == GCODE_SCRIPT_TYPE_COLOR_CHANGE_GCODE)
        return "M600\n";
    if (script_type == GCODE_SCRIPT_TYPE_PAUSE_PRINT_GCODE)
        return "M226\n";
    return DefaultGCodeFirmwareSession::resolve_empty_script(script_type, producer_config);
}

std::string RepRapGCodeFirmwareSession::encode_machine_envelope(
    const MachineEnvelope &envelope) const
{
    // RepRapFirmware expresses feedrate and jerk envelope values in mm/min,
    // while acceleration values remain in mm/sec^2.
    const double unit_factor = 60.0;
    std::string output;
    GCodeFormatter formatter(2, 2);

    m_reader_state.max_acceleration = {
        double(envelope.max_acceleration_x), double(envelope.max_acceleration_y),
        double(envelope.max_acceleration_z), double(envelope.max_acceleration_e)};
    m_reader_state.max_feedrate = {
        envelope.max_feedrate_x, envelope.max_feedrate_y,
        envelope.max_feedrate_z, envelope.max_feedrate_e};
    m_reader_state.min_extruding_feedrate = envelope.min_extruding_feedrate;
    m_reader_state.print_acceleration = envelope.max_print_acceleration;
    m_reader_state.travel_acceleration = envelope.max_travel_acceleration;
    m_reader_state.machine_jerk = {
        envelope.max_jerk_x, envelope.max_jerk_y,
        envelope.max_jerk_z, envelope.max_jerk_e};

    formatter.emit_string("M201");
    formatter.emit_integer_parameter('X', envelope.max_acceleration_x);
    formatter.emit_integer_parameter('Y', envelope.max_acceleration_y);
    formatter.emit_integer_parameter('Z', envelope.max_acceleration_z);
    formatter.emit_integer_parameter('E', envelope.max_acceleration_e);
    formatter.emit_comment(true, "sets maximum accelerations, mm/sec^2");
    output += formatter.string();
    formatter.clear();

    // Convert feedrates only at the dialect boundary; MachineEnvelope remains
    // expressed in its neutral mm/sec units.
    formatter.emit_string("M203");
    formatter.emit_integer_parameter('X', envelope.max_feedrate_x * unit_factor);
    formatter.emit_integer_parameter('Y', envelope.max_feedrate_y * unit_factor);
    formatter.emit_integer_parameter('Z', envelope.max_feedrate_z * unit_factor);
    formatter.emit_integer_parameter('E', envelope.max_feedrate_e * unit_factor);
    formatter.emit_integer_parameter('I', envelope.min_extruding_feedrate * unit_factor);
    formatter.emit_comment(true, "sets maximum feedrates, mm/min");
    output += formatter.string();
    formatter.clear();

    formatter.emit_string("M204");
    formatter.emit_integer_parameter('P', envelope.max_print_acceleration);
    formatter.emit_integer_parameter('T', envelope.max_travel_acceleration);
    formatter.emit_comment(true, "sets print and travel acceleration, mm/sec^2");
    output += formatter.string();
    formatter.clear();

    formatter.emit_string("M566");
    formatter.emit_fixed_parameter('X', envelope.max_jerk_x * unit_factor, 2);
    formatter.emit_fixed_parameter('Y', envelope.max_jerk_y * unit_factor, 2);
    formatter.emit_fixed_parameter('Z', envelope.max_jerk_z * unit_factor, 2);
    formatter.emit_fixed_parameter('E', envelope.max_jerk_e * unit_factor, 2);
    formatter.emit_comment(true, "sets the jerk limits, mm/min");
    output += formatter.string();
    return output;
}

std::string RepRapGCodeFirmwareSession::encode_tool_temperature(
    extruder_index_t tool_id,
    int16_t temperature,
    bool wait) const
{
    // [2026-08-25] RepRapFirmware uses G10 to set a tool temperature and M116
    // to wait: https://docs.duet3d.com/User_manual/Reference/Gcodes
    std::string output;
    const std::optional<int16_t> encoded = tool_id < extruders().size() ?
        extruders()[tool_id].heater().encoded_temperature() : std::optional<int16_t>();
    if (!wait || !encoded || *encoded != temperature)
        output = "G10 P" + std::to_string(tool_id) + " S" + std::to_string(temperature) + "\n";
    if (wait)
        output += "M116 P" + std::to_string(tool_id) + "\n";
    return output;
}

// Clamp the requested fan percentage and return mapped physical fan commands;
// SEMM uses tool 0's fan mapping (fan 0 without a mapping), without state mutation.
std::string RepRapGCodeFirmwareSession::encode_fan(extruder_index_t tool_id,
                                                    double speed_percent) const
{
    const double normalized = std::clamp(speed_percent, 0.0, 100.0) / 100.0;
    const extruder_index_t fan_tool = single_extruder_multi_material() ? 0 : tool_id;
    const std::map<uint32_t, Reader::RepRapToolState>::const_iterator tool =
        m_reader_state.tools.find(fan_tool);
    const std::vector<uint32_t> fallback{static_cast<uint32_t>(fan_tool)};
    const std::vector<uint32_t> &fans = tool != m_reader_state.tools.end() &&
                                               !tool->second.fans.empty() ?
        tool->second.fans : fallback;
    std::string output;
    for (uint32_t fan : fans)
        output += "M106 P" + std::to_string(fan) +
                  " S" + format_number(normalized) + "\n";
    return output;
}

// Return pressure-advance commands and record their physical drive values in the
// owned Reader cache; SEMM caches its unaddressed M572 on drive 0, while other tools
// use their declared mappings or a same-ID fallback. Inputs are resolved upstream.
std::string RepRapGCodeFirmwareSession::encode_pressure_advance(
    extruder_index_t tool_id,
    double pressure_advance) const
{
    if (single_extruder_multi_material()) {
        m_reader_state.pressure_advance[0] = pressure_advance;
        return "M572 S" + format_number(pressure_advance) + "\n";
    }
    // [2026-08-25] RepRapFirmware assigns pressure advance per drive with
    // M572: https://docs.duet3d.com/User_manual/Reference/Gcodes
    const std::map<uint32_t, Reader::RepRapToolState>::const_iterator tool =
        m_reader_state.tools.find(tool_id);
    const std::vector<uint32_t> fallback{static_cast<uint32_t>(tool_id)};
    const std::vector<uint32_t> &drives = tool != m_reader_state.tools.end() &&
                                                 !tool->second.drives.empty() ?
        tool->second.drives : fallback;
    std::string output;
    for (uint32_t drive : drives) {
        m_reader_state.pressure_advance[drive] = pressure_advance;
        output += "M572 D" + std::to_string(drive) +
                  " S" + format_number(pressure_advance) + "\n";
    }
    return output;
}

std::string RepRapGCodeFirmwareSession::encode_acceleration(
    uint32_t acceleration, bool travel) const
{
    if (travel)
        m_reader_state.travel_acceleration = acceleration;
    else
        m_reader_state.print_acceleration = acceleration;
    return DefaultGCodeFirmwareSession::encode_acceleration(acceleration, travel);
}

std::string RepRapGCodeFirmwareSession::encode_firmware_retraction(bool retract) const
{
    if (const std::optional<extruder_index_t> tool_id = current_extruder_id())
        m_reader_state.tools[*tool_id].retracted = retract;
    return DefaultGCodeFirmwareSession::encode_firmware_retraction(retract);
}

std::string RepRapGCodeFirmwareSession::encode_object_definitions(
    const std::vector<ObjectLabel> &objects) const
{
    return "M486 T" + std::to_string(objects.size()) + "\n";
}

std::string RepRapGCodeFirmwareSession::encode_object_start(const ObjectLabel &object) const
{
    return "M486 S" + std::to_string(object.id) + " A\"" + object.name + "\"\n";
}

std::string RepRapGCodeFirmwareSession::encode_object_end(const ObjectLabel &) const
{
    return "M486 S-1\n";
}

}}} // namespace slic3r_api::GCodeGeneration::Firmware
