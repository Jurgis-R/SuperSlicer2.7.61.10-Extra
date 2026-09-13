///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "RepRapGCodeReader.hpp"
#include "RepRapGCodeReaderState.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

/*
RepRapFirmware command extensions
=================================

Each handler first updates RepRapGCodeReaderState, whose custom scalar keys are
opaque to the host, then publishes any portable consequence. The final state
therefore preserves persistent RRF registers while ordinary consumers continue
to see only the normalized current machine state.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Reader {

namespace {

// Extract one colon-separated non-negative device list. RRF's -1 sentinel
// means an explicitly empty mapping. Dotted remote-board ids and expressions
// remain unsupported because they cannot be represented by uint32 device ids.
bool parse_device_list(std::string_view parameter, std::vector<uint32_t> &devices);
// Distinguish bare G10 firmware retraction from parameterized G10 forms. A
// trailing comment is not a command argument.
bool has_command_arguments(const GCodeLineParser::GCodeLine &line);
// Resolve an explicit P tool or fall back to the currently active tool.
std::optional<uint32_t> command_tool(const GCodeLineParser::GCodeLine &line,
                                     const GCodeStateTracker &state);
// Apply X/Y/Z/E values to one four-axis register with a unit conversion.
bool update_axes(const GCodeLineParser::GCodeLine &line,
                 std::array<std::optional<double>, 4> &values,
                 double factor);
// Store private state before publishing the portable command event.
void commit_private_state(RepRapGCodeReaderState &private_state,
                          GCodeStateTracker &state);

bool parse_device_list(std::string_view parameter, std::vector<uint32_t> &devices)
{
    devices.clear();
    if (parameter.empty())
        return false;
    parameter.remove_prefix(1);
    while (!parameter.empty() && (parameter.front() == ' ' || parameter.front() == '\t'))
        parameter.remove_prefix(1);
    const size_t end = parameter.find_first_of(" \t;\r\n");
    parameter = parameter.substr(0, end);
    if (parameter == "-1")
        return true;
    if (parameter.empty())
        return false;

    while (!parameter.empty()) {
        const size_t separator = parameter.find(':');
        const std::string_view token = parameter.substr(0, separator);
        uint32_t device = 0;
        const std::from_chars_result parsed = std::from_chars(
            token.data(), token.data() + token.size(), device);
        if (token.empty() || parsed.ec != std::errc() ||
            parsed.ptr != token.data() + token.size() ||
            std::find(devices.begin(), devices.end(), device) != devices.end())
            return false;
        devices.push_back(device);
        if (separator == std::string_view::npos)
            break;
        parameter.remove_prefix(separator + 1);
        if (parameter.empty())
            return false;
    }
    return true;
}

bool has_command_arguments(const GCodeLineParser::GCodeLine &line)
{
    std::string_view tail = line.raw().substr(line.command().size());
    while (!tail.empty() && (tail.front() == ' ' || tail.front() == '\t'))
        tail.remove_prefix(1);
    return !tail.empty() && tail.front() != ';';
}

std::optional<uint32_t> command_tool(const GCodeLineParser::GCodeLine &line,
                                     const GCodeStateTracker &state)
{
    int32_t tool = -1;
    if (line.has_parameter('P')) {
        if (!line.parameter_value('P', tool) || tool < 0)
            return std::nullopt;
        return static_cast<uint32_t>(tool);
    }
    return state.active_tool();
}

bool update_axes(const GCodeLineParser::GCodeLine &line,
                 std::array<std::optional<double>, 4> &values,
                 double factor)
{
    const char axes[] = {'X', 'Y', 'Z', 'E'};
    bool changed = false;
    for (uint32_t axis = 0; axis < 4; ++axis) {
        if (const std::optional<float> value = line.parameter_value(axes[axis])) {
            values[axis] = double(*value) * factor;
            changed = true;
        }
    }
    return changed;
}

void commit_private_state(RepRapGCodeReaderState &private_state,
                          GCodeStateTracker &state)
{
    private_state.write_to(state);
}

} // namespace

RepRapGCodeReader::RepRapGCodeReader(orchestrator_handle *orchestrator) :
    BuiltinGCodeReaderBase(orchestrator)
{}

const char *RepRapGCodeReader::id_impl() const noexcept
{
    return "gcode.reader.reprap";
}

const char *RepRapGCodeReader::name_impl() const noexcept
{
    return "RepRapFirmware G-code reader";
}

const char *RepRapGCodeReader::description_impl() const noexcept
{
    return "Reads RepRapFirmware G-code into normalized machine-state events.";
}

uint32_t RepRapGCodeReader::probe_impl(std::string_view prefix) const
{
    return probe_marker(prefix, "reprap");
}

bool RepRapGCodeReader::consume_text_command(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &line,
    raw_gcode_reader_source source) const
{
    if (line.command() != "T-1")
        return false;

    const raw_gcode_reader_event_item active_tool = {
        RAW_GCODE_READER_ITEM_ACTIVE_TOOL, 0, 0.0
    };
    publish_items(run_context, context, state, RAW_GCODE_READER_EVENT_INVALIDATE,
                  source, {active_tool});
    return true;
}

bool RepRapGCodeReader::consume_numeric_command(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &line,
    char letter,
    uint32_t number,
    raw_gcode_reader_source source) const
{
    RepRapGCodeReaderState private_state;
    try {
        private_state = RepRapGCodeReaderState::from_tracker(state);
    } catch (const std::exception &) {
        publish_unknown(run_context, context, state, source);
        return true;
    }

    if (letter == 'G' && number == 10) {
        const std::optional<uint32_t> tool = command_tool(line, state);
        if (!tool) {
            publish_unknown(run_context, context, state, source);
            return true;
        }

        RepRapToolState &tool_state = private_state.tools[*tool];
        const std::optional<float> active_temperature = line.parameter_value('S');
        const std::optional<float> standby_temperature = line.parameter_value('R');
        const bool has_offset = line.has_parameter('X') || line.has_parameter('Y') ||
                                line.has_parameter('Z');
        if (!active_temperature && !standby_temperature && !has_offset) {
            // Bare G10 is firmware retraction. Parameterized but unsupported
            // forms may alter coordinate systems, so only a truly bare command
            // receives this interpretation.
            if (has_command_arguments(line)) {
                publish_unknown(run_context, context, state, source);
                return true;
            }
            tool_state.retracted = true;
            commit_private_state(private_state, state);
            publish_state_update(run_context, context, state, source);
            return true;
        }

        if (standby_temperature)
            tool_state.standby_temperature = double(*standby_temperature);
        if (const std::optional<float> value = line.parameter_value('X'))
            tool_state.offset[0] = *value;
        if (const std::optional<float> value = line.parameter_value('Y'))
            tool_state.offset[1] = *value;
        if (const std::optional<float> value = line.parameter_value('Z'))
            tool_state.offset[2] = *value;
        commit_private_state(private_state, state);
        if (active_temperature) {
            publish_items(run_context, context, state,
                          RAW_GCODE_READER_EVENT_SET_TEMPERATURE, source,
                          {{RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE,
                            *tool, double(*active_temperature)}});
        } else {
            publish_state_update(run_context, context, state, source);
        }
        return true;
    }

    if (letter == 'G' && number == 11) {
        const std::optional<uint32_t> tool = state.active_tool();
        if (!tool) {
            publish_unknown(run_context, context, state, source);
            return true;
        }
        private_state.tools[*tool].retracted = false;
        commit_private_state(private_state, state);
        publish_state_update(run_context, context, state, source);
        return true;
    }

    if (letter == 'M' && number == 116) {
        std::vector<raw_gcode_reader_event_item> reached;
        if (line.has_parameter('P')) {
            const std::optional<uint32_t> requested_tool = command_tool(line, state);
            if (!requested_tool) {
                publish_unknown(run_context, context, state, source);
                return true;
            }
            const std::optional<double> target = state.tool_temperature(
                *requested_tool);
            if (target)
                reached.push_back({RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE,
                                   *requested_tool, *target});
        } else {
            const raw_gcode_reader_state &raw = state.state();
            for (uint32_t index = 0; index < raw.item_count; ++index)
                if (raw.items[index].type == RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE)
                    reached.push_back(raw.items[index]);
        }
        publish_items(run_context, context, state,
                      RAW_GCODE_READER_EVENT_WAIT_TEMPERATURE, source, reached);
        return true;
    }

    if (letter == 'M' && (number == 106 || number == 107)) {
        std::vector<uint32_t> fans;
        int32_t explicit_fan = -1;
        if (line.has_parameter('P')) {
            if (!line.parameter_value('P', explicit_fan) || explicit_fan < 0) {
                publish_unknown(run_context, context, state, source);
                return true;
            }
            fans.push_back(static_cast<uint32_t>(explicit_fan));
        } else if (const std::optional<uint32_t> tool = state.active_tool()) {
            const std::map<uint32_t, RepRapToolState>::const_iterator found =
                private_state.tools.find(*tool);
            if (found != private_state.tools.end())
                fans = found->second.fans;
        }
        if (fans.empty()) {
            publish_unknown(run_context, context, state, source);
            return true;
        }
        std::vector<raw_gcode_reader_event_item> items;
        const std::optional<float> speed = number == 107 ?
            std::optional<float>(0.0f) : line.parameter_value('S');
        if (!speed) {
            publish_unknown(run_context, context, state, source);
            return true;
        }
        for (uint32_t fan : fans)
            items.push_back({RAW_GCODE_READER_ITEM_FAN_SPEED, fan,
                             std::clamp(double(*speed) * 100.0, 0.0, 100.0)});
        publish_items(run_context, context, state,
                      RAW_GCODE_READER_EVENT_STATE_UPDATE, source, items);
        return true;
    }

    if (letter == 'M' && number == 201) {
        if (!update_axes(line, private_state.max_acceleration, 1.0)) {
            publish_unknown(run_context, context, state, source);
            return true;
        }
        commit_private_state(private_state, state);
        publish_state_update(run_context, context, state, source);
        return true;
    }

    if (letter == 'M' && number == 203) {
        bool changed = update_axes(line, private_state.max_feedrate, 1.0 / 60.0);
        if (const std::optional<float> minimum = line.parameter_value('I')) {
            private_state.min_extruding_feedrate = double(*minimum) / 60.0;
            changed = true;
        }
        if (!changed) {
            publish_unknown(run_context, context, state, source);
            return true;
        }
        commit_private_state(private_state, state);
        publish_state_update(run_context, context, state, source);
        return true;
    }

    if (letter == 'M' && number == 205) {
        if (!update_axes(line, private_state.job_jerk, 1.0)) {
            publish_unknown(run_context, context, state, source);
            return true;
        }
        commit_private_state(private_state, state);
        publish_state_update(run_context, context, state, source);
        return true;
    }

    if (letter == 'M' && number == 563) {
        int32_t requested_tool = -1;
        if (!line.parameter_value('P', requested_tool) || requested_tool < 0) {
            publish_unknown(run_context, context, state, source);
            return true;
        }
        const uint32_t tool = static_cast<uint32_t>(requested_tool);
        RepRapToolState &tool_state = private_state.tools[tool];
        bool changed = false;
        const std::pair<char, std::vector<uint32_t> *> mappings[] = {
            {'D', &tool_state.drives}, {'H', &tool_state.heaters}, {'F', &tool_state.fans}
        };
        for (const std::pair<char, std::vector<uint32_t> *> &mapping : mappings) {
            if (!line.has_parameter(mapping.first))
                continue;
            if (!parse_device_list(line.parameter_position(mapping.first), *mapping.second)) {
                publish_unknown(run_context, context, state, source);
                return true;
            }
            changed = true;
        }
        if (!changed) {
            publish_unknown(run_context, context, state, source);
            return true;
        }
        commit_private_state(private_state, state);
        publish_state_update(run_context, context, state, source);
        return true;
    }

    if (letter == 'M' && number == 566) {
        if (!update_axes(line, private_state.machine_jerk, 1.0 / 60.0)) {
            publish_unknown(run_context, context, state, source);
            return true;
        }
        commit_private_state(private_state, state);
        publish_state_update(run_context, context, state, source);
        return true;
    }

    if (letter == 'M' && number == 568) {
        const std::optional<uint32_t> tool = command_tool(line, state);
        if (!tool) {
            publish_unknown(run_context, context, state, source);
            return true;
        }
        const std::optional<float> active_temperature = line.parameter_value('S');
        const std::optional<float> standby_temperature = line.parameter_value('R');
        if (!active_temperature && !standby_temperature) {
            publish_unknown(run_context, context, state, source);
            return true;
        }
        if (standby_temperature)
            private_state.tools[*tool].standby_temperature = *standby_temperature;
        commit_private_state(private_state, state);
        if (active_temperature)
            publish_items(run_context, context, state,
                          RAW_GCODE_READER_EVENT_SET_TEMPERATURE, source,
                          {{RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE, *tool,
                            double(*active_temperature)}});
        else
            publish_state_update(run_context, context, state, source);
        return true;
    }

    if (letter == 'M' && number == 572) {
        int32_t drive = -1;
        const std::optional<float> pressure = line.parameter_value('S');
        if (!pressure || !line.parameter_value('D', drive) || drive < 0) {
            publish_unknown(run_context, context, state, source);
            return true;
        }
        private_state.pressure_advance[static_cast<uint32_t>(drive)] = *pressure;
        commit_private_state(private_state, state);
        publish_state_update(run_context, context, state, source);
        return true;
    }

    if (letter != 'M' || number != 204)
        return false;
    std::optional<float> acceleration = line.parameter_value('S');
    const std::optional<float> print = line.parameter_value('P');
    const std::optional<float> travel = line.parameter_value('T');
    if (acceleration) {
        private_state.print_acceleration = *acceleration;
        private_state.travel_acceleration = *acceleration;
    } else {
        if (print)
            private_state.print_acceleration = *print;
        if (travel)
            private_state.travel_acceleration = *travel;
        if (print && !travel)
            acceleration = print;
        else if (travel && !print)
            acceleration = travel;
    }
    if (!acceleration && !print && !travel) {
        publish_unknown(run_context, context, state, source);
        return true;
    }
    commit_private_state(private_state, state);
    std::vector<raw_gcode_reader_event_item> items;
    if (acceleration)
        items.push_back({RAW_GCODE_READER_ITEM_ACCELERATION, 0, double(*acceleration)});
    publish_items(run_context, context, state,
                  RAW_GCODE_READER_EVENT_STATE_UPDATE, source, items);
    return true;
}

bool RepRapGCodeReader::is_neutral_command(char letter, uint32_t number) const
{
    return (letter == 'G' && number == 4) ||
           (letter == 'M' && (number == 900 || number == 906));
}

bool RepRapGCodeReader::is_marker_command(char letter, uint32_t number) const
{
    return letter == 'M' &&
           (number == 0 || number == 600 || number == 601 || number == 226);
}

void RepRapGCodeReader::validate_private_state(const GCodeStateTracker &state) const
{
    // Decoding performs identity, scalar-domain and complete mapping checks.
    // The resulting value is intentionally discarded at this boundary.
    RepRapGCodeReaderState::from_tracker(state);
}

}}} // namespace slic3r_api::GCodeGeneration::Reader
