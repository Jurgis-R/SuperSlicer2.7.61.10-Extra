///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "RepRapGCodeReaderState.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

/*
RepRapFirmware private state serialization
==========================================

Decoding first discovers every logical tool mentioned by a scalar or mapping
count. It then validates each indexed mapping as one complete list. Encoding
removes the known RRF ranges before writing the replacement, preventing stale
slots from surviving when a later command shortens a mapping.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Reader {

namespace {

// Return an exact uint32 value from an already finite transport scalar.
uint32_t checked_uint32(double value, const char *description);
// Decode one count+slot mapping and reject incomplete or duplicate lists.
std::vector<uint32_t> read_mapping(const GCodeStateTracker &tracker,
                                   uint32_t tool, uint32_t count_type,
                                   uint32_t slot_begin, const char *description);
// Append every target which uses a known tool-scoped scalar key.
void discover_tool_targets(const raw_gcode_reader_state &raw,
                           std::set<uint32_t> &tools);
// Report whether a private type belongs to this implementation's fixed ranges.
bool is_known_rrf_type(uint32_t type);
// Remove all known RRF keys while preserving future private extensions.
void erase_known_rrf_values(GCodeStateTracker &tracker);
// Write one optional scalar without manufacturing an unknown value.
void write_optional(GCodeStateTracker &tracker, uint32_t type, uint32_t target,
                    const std::optional<double> &value);
// Write one deterministic count+slot mapping.
void write_mapping(GCodeStateTracker &tracker, uint32_t tool, uint32_t count_type,
                   uint32_t slot_begin, const std::vector<uint32_t> &devices);
// Check one optional physical register shared by decoding and encoding.
void validate_nonnegative(const std::optional<double> &value, const char *description);
void validate_finite(const std::optional<double> &value, const char *description);

uint32_t checked_uint32(double value, const char *description)
{
    if (value < 0.0 || value > double(std::numeric_limits<uint32_t>::max()) ||
        std::trunc(value) != value)
        throw std::invalid_argument(std::string(description) + " is not an exact uint32 value.");
    return static_cast<uint32_t>(value);
}

std::vector<uint32_t> read_mapping(const GCodeStateTracker &tracker,
                                   uint32_t tool, uint32_t count_type,
                                   uint32_t slot_begin, const char *description)
{
    const std::optional<double> raw_count = tracker.value(count_type, tool);
    if (!raw_count)
        return {};
    const uint32_t count = checked_uint32(*raw_count, description);
    if (count > REPRAP_MAPPING_SLOT_COUNT)
        throw std::invalid_argument(std::string(description) + " exceeds the private slot range.");

    std::vector<uint32_t> devices;
    std::set<uint32_t> unique;
    devices.reserve(count);
    for (uint32_t slot = 0; slot < count; ++slot) {
        const std::optional<double> raw_device = tracker.value(slot_begin + slot, tool);
        if (!raw_device)
            throw std::invalid_argument(std::string(description) + " has a missing indexed slot.");
        const uint32_t device = checked_uint32(*raw_device, description);
        if (!unique.insert(device).second)
            throw std::invalid_argument(std::string(description) + " contains a duplicate device.");
        devices.push_back(device);
    }

    // A slot outside the declared half-open list is stale or malformed. Its
    // presence must not silently change meaning when the count later grows.
    const raw_gcode_reader_state &raw = tracker.state();
    for (uint32_t index = 0; index < raw.item_count; ++index) {
        const raw_gcode_reader_event_item &item = raw.items[index];
        if (item.target == tool && item.type >= slot_begin &&
            item.type < slot_begin + REPRAP_MAPPING_SLOT_COUNT &&
            item.type - slot_begin >= count)
            throw std::invalid_argument(std::string(description) + " has a slot beyond its count.");
    }
    return devices;
}

void discover_tool_targets(const raw_gcode_reader_state &raw,
                           std::set<uint32_t> &tools)
{
    for (uint32_t index = 0; index < raw.item_count; ++index) {
        const uint32_t type = raw.items[index].type;
        const bool tool_scalar = type >= RRF_ITEM_TOOL_STANDBY_TEMPERATURE &&
                                 type <= RRF_ITEM_TOOL_FAN_COUNT;
        const bool tool_slot = (type >= RRF_ITEM_TOOL_DRIVE_SLOT_BEGIN &&
                                type < RRF_ITEM_TOOL_DRIVE_SLOT_BEGIN + REPRAP_MAPPING_SLOT_COUNT) ||
                               (type >= RRF_ITEM_TOOL_HEATER_SLOT_BEGIN &&
                                type < RRF_ITEM_TOOL_HEATER_SLOT_BEGIN + REPRAP_MAPPING_SLOT_COUNT) ||
                               (type >= RRF_ITEM_TOOL_FAN_SLOT_BEGIN &&
                                type < RRF_ITEM_TOOL_FAN_SLOT_BEGIN + REPRAP_MAPPING_SLOT_COUNT);
        if (tool_scalar || tool_slot)
            tools.insert(raw.items[index].target);
    }
}

bool is_known_rrf_type(uint32_t type)
{
    return (type >= RRF_ITEM_PRINT_ACCELERATION && type <= RRF_ITEM_TOOL_FAN_COUNT) ||
           (type >= RRF_ITEM_TOOL_DRIVE_SLOT_BEGIN &&
            type < RRF_ITEM_TOOL_DRIVE_SLOT_BEGIN + REPRAP_MAPPING_SLOT_COUNT) ||
           (type >= RRF_ITEM_TOOL_HEATER_SLOT_BEGIN &&
            type < RRF_ITEM_TOOL_HEATER_SLOT_BEGIN + REPRAP_MAPPING_SLOT_COUNT) ||
           (type >= RRF_ITEM_TOOL_FAN_SLOT_BEGIN &&
            type < RRF_ITEM_TOOL_FAN_SLOT_BEGIN + REPRAP_MAPPING_SLOT_COUNT);
}

void erase_known_rrf_values(GCodeStateTracker &tracker)
{
    std::vector<std::pair<uint32_t, uint32_t>> keys;
    const raw_gcode_reader_state &raw = tracker.state();
    for (uint32_t index = 0; index < raw.item_count; ++index)
        if (is_known_rrf_type(raw.items[index].type))
            keys.emplace_back(raw.items[index].type, raw.items[index].target);
    for (const std::pair<uint32_t, uint32_t> &key : keys)
        tracker.erase_custom_value(key.first, key.second);
}

void write_optional(GCodeStateTracker &tracker, uint32_t type, uint32_t target,
                    const std::optional<double> &value)
{
    if (value)
        tracker.set_custom_value(type, target, *value);
}

void write_mapping(GCodeStateTracker &tracker, uint32_t tool, uint32_t count_type,
                   uint32_t slot_begin, const std::vector<uint32_t> &devices)
{
    if (devices.size() > REPRAP_MAPPING_SLOT_COUNT)
        throw std::invalid_argument("A RepRapFirmware tool mapping exceeds its private slot range.");
    tracker.set_custom_value(count_type, tool, double(devices.size()));
    for (size_t slot = 0; slot < devices.size(); ++slot)
        tracker.set_custom_value(slot_begin + static_cast<uint32_t>(slot), tool,
                                 double(devices[slot]));
}

void validate_nonnegative(const std::optional<double> &value, const char *description)
{
    if (value && (!std::isfinite(*value) || *value < 0.0))
        throw std::invalid_argument(std::string(description) + " must be finite and non-negative.");
}

void validate_finite(const std::optional<double> &value, const char *description)
{
    if (value && !std::isfinite(*value))
        throw std::invalid_argument(std::string(description) + " must be finite.");
}

} // namespace

RepRapGCodeReaderState RepRapGCodeReaderState::from_tracker(
    const GCodeStateTracker &tracker, uint32_t reader_state_id)
{
    if (tracker.reader_id() != reader_state_id)
        throw std::invalid_argument("The G-code Reader state does not belong to RepRapFirmware.");

    RepRapGCodeReaderState result;
    result.print_acceleration = tracker.value(RRF_ITEM_PRINT_ACCELERATION);
    result.travel_acceleration = tracker.value(RRF_ITEM_TRAVEL_ACCELERATION);
    for (uint32_t axis = 0; axis < 4; ++axis) {
        result.max_acceleration[axis] = tracker.value(RRF_ITEM_MAX_ACCELERATION_X + axis);
        result.max_feedrate[axis] = tracker.value(RRF_ITEM_MAX_FEEDRATE_X + axis);
        result.machine_jerk[axis] = tracker.value(RRF_ITEM_MACHINE_JERK_X + axis);
        result.job_jerk[axis] = tracker.value(RRF_ITEM_JOB_JERK_X + axis);
    }
    result.min_extruding_feedrate = tracker.value(RRF_ITEM_MIN_EXTRUDING_FEEDRATE);

    const raw_gcode_reader_state &raw = tracker.state();
    std::set<uint32_t> tool_ids;
    discover_tool_targets(raw, tool_ids);
    for (uint32_t index = 0; index < raw.item_count; ++index) {
        const raw_gcode_reader_event_item &item = raw.items[index];
        if (item.type == RRF_ITEM_PRESSURE_ADVANCE)
            result.pressure_advance[item.target] = item.value;
    }

    for (uint32_t tool : tool_ids) {
        RepRapToolState &value = result.tools[tool];
        value.drives = read_mapping(tracker, tool, RRF_ITEM_TOOL_DRIVE_COUNT,
                                    RRF_ITEM_TOOL_DRIVE_SLOT_BEGIN, "RepRapFirmware drive mapping");
        value.heaters = read_mapping(tracker, tool, RRF_ITEM_TOOL_HEATER_COUNT,
                                     RRF_ITEM_TOOL_HEATER_SLOT_BEGIN, "RepRapFirmware heater mapping");
        value.fans = read_mapping(tracker, tool, RRF_ITEM_TOOL_FAN_COUNT,
                                  RRF_ITEM_TOOL_FAN_SLOT_BEGIN, "RepRapFirmware fan mapping");
        value.standby_temperature = tracker.value(RRF_ITEM_TOOL_STANDBY_TEMPERATURE, tool);
        value.offset[0] = tracker.value(RRF_ITEM_TOOL_OFFSET_X, tool);
        value.offset[1] = tracker.value(RRF_ITEM_TOOL_OFFSET_Y, tool);
        value.offset[2] = tracker.value(RRF_ITEM_TOOL_OFFSET_Z, tool);
        const std::optional<double> retracted = tracker.value(RRF_ITEM_TOOL_RETRACTED, tool);
        if (retracted && *retracted != 0.0 && *retracted != 1.0)
            throw std::invalid_argument("RepRapFirmware retraction state must be zero or one.");
        if (retracted)
            value.retracted = *retracted != 0.0;
    }
    result.validate();
    return result;
}

void RepRapGCodeReaderState::validate() const
{
    validate_nonnegative(print_acceleration, "RepRapFirmware print acceleration");
    validate_nonnegative(travel_acceleration, "RepRapFirmware travel acceleration");
    validate_nonnegative(min_extruding_feedrate, "RepRapFirmware minimum extrusion feedrate");
    for (uint32_t axis = 0; axis < 4; ++axis) {
        validate_nonnegative(max_acceleration[axis], "RepRapFirmware maximum acceleration");
        validate_nonnegative(max_feedrate[axis], "RepRapFirmware maximum feedrate");
        validate_nonnegative(machine_jerk[axis], "RepRapFirmware machine jerk");
        validate_nonnegative(job_jerk[axis], "RepRapFirmware job jerk");
    }
    for (const std::pair<const uint32_t, double> &entry : pressure_advance)
        if (!std::isfinite(entry.second) || entry.second < 0.0)
            throw std::invalid_argument("RepRapFirmware pressure advance must be finite and non-negative.");
    for (const std::pair<const uint32_t, RepRapToolState> &entry : tools) {
        validate_finite(entry.second.standby_temperature,
                        "RepRapFirmware standby temperature");
        for (const std::optional<double> &offset : entry.second.offset)
            if (offset && !std::isfinite(*offset))
                throw std::invalid_argument("RepRapFirmware tool offset must be finite.");
        const std::vector<uint32_t> *mappings[] = {
            &entry.second.drives, &entry.second.heaters, &entry.second.fans
        };
        for (const std::vector<uint32_t> *mapping : mappings) {
            if (mapping->size() > REPRAP_MAPPING_SLOT_COUNT)
                throw std::invalid_argument("A RepRapFirmware tool mapping exceeds its private slot range.");
            const std::set<uint32_t> unique(mapping->begin(), mapping->end());
            if (unique.size() != mapping->size())
                throw std::invalid_argument("A RepRapFirmware tool mapping contains duplicate devices.");
        }
    }
}

void RepRapGCodeReaderState::write_to(
    GCodeStateTracker &tracker, uint32_t reader_state_id) const
{
    if (tracker.reader_id() != reader_state_id)
        throw std::invalid_argument("Cannot write RepRapFirmware state for another Reader identity.");

    validate();

    erase_known_rrf_values(tracker);
    write_optional(tracker, RRF_ITEM_PRINT_ACCELERATION, 0, print_acceleration);
    write_optional(tracker, RRF_ITEM_TRAVEL_ACCELERATION, 0, travel_acceleration);
    for (uint32_t axis = 0; axis < 4; ++axis) {
        write_optional(tracker, RRF_ITEM_MAX_ACCELERATION_X + axis, 0, max_acceleration[axis]);
        write_optional(tracker, RRF_ITEM_MAX_FEEDRATE_X + axis, 0, max_feedrate[axis]);
        write_optional(tracker, RRF_ITEM_MACHINE_JERK_X + axis, 0, machine_jerk[axis]);
        write_optional(tracker, RRF_ITEM_JOB_JERK_X + axis, 0, job_jerk[axis]);
    }
    write_optional(tracker, RRF_ITEM_MIN_EXTRUDING_FEEDRATE, 0, min_extruding_feedrate);
    for (const std::pair<const uint32_t, double> &entry : pressure_advance)
        tracker.set_custom_value(RRF_ITEM_PRESSURE_ADVANCE, entry.first, entry.second);

    for (const std::pair<const uint32_t, RepRapToolState> &entry : tools) {
        const uint32_t tool = entry.first;
        const RepRapToolState &value = entry.second;
        write_mapping(tracker, tool, RRF_ITEM_TOOL_DRIVE_COUNT,
                      RRF_ITEM_TOOL_DRIVE_SLOT_BEGIN, value.drives);
        write_mapping(tracker, tool, RRF_ITEM_TOOL_HEATER_COUNT,
                      RRF_ITEM_TOOL_HEATER_SLOT_BEGIN, value.heaters);
        write_mapping(tracker, tool, RRF_ITEM_TOOL_FAN_COUNT,
                      RRF_ITEM_TOOL_FAN_SLOT_BEGIN, value.fans);
        write_optional(tracker, RRF_ITEM_TOOL_STANDBY_TEMPERATURE, tool,
                       value.standby_temperature);
        write_optional(tracker, RRF_ITEM_TOOL_OFFSET_X, tool, value.offset[0]);
        write_optional(tracker, RRF_ITEM_TOOL_OFFSET_Y, tool, value.offset[1]);
        write_optional(tracker, RRF_ITEM_TOOL_OFFSET_Z, tool, value.offset[2]);
        if (value.retracted)
            tracker.set_custom_value(RRF_ITEM_TOOL_RETRACTED, tool,
                                     *value.retracted ? 1.0 : 0.0);
    }
}

}}} // namespace slic3r_api::GCodeGeneration::Reader
