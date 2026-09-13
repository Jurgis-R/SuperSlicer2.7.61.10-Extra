///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "RepRapGCodeReader.hpp"

#include <algorithm>
#include <optional>
#include <vector>

/*
RepRapFirmware command extensions
=================================

G10 and M116 model tool temperatures, M106 uses a normalized fan value, and
M204 uses RepRapFirmware's acceleration form. These commands are intercepted
before the common parser because their parameter semantics are dialect-owned.
Malformed G10 variants invalidate state because they may alter offsets or tool
configuration which the portable Reader state cannot represent.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Reader {

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

bool RepRapGCodeReader::consume_numeric_command(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &line,
    char letter,
    uint32_t number,
    raw_gcode_reader_source source) const
{
    if (letter == 'G' && number == 10) {
        int32_t requested_tool = -1;
        const std::optional<float> temperature = line.parameter_value('S');
        if (temperature && line.parameter_value('P', requested_tool) && requested_tool >= 0) {
            publish_items(run_context, context, state,
                          RAW_GCODE_READER_EVENT_SET_TEMPERATURE, source,
                          {{RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE,
                            uint32_t(requested_tool), double(*temperature)}});
        } else {
            publish_unknown(run_context, context, state, source);
        }
        return true;
    }

    if (letter == 'M' && number == 116) {
        int32_t requested_tool = int32_t(active_tool(state));
        line.parameter_value('P', requested_tool);
        std::vector<raw_gcode_reader_event_item> reached;
        if (requested_tool >= 0) {
            const std::optional<double> target = state.tool_temperature(
                static_cast<uint32_t>(requested_tool));
            if (target)
                reached.push_back({RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE,
                                   uint32_t(requested_tool), *target});
        }
        publish_items(run_context, context, state,
                      RAW_GCODE_READER_EVENT_WAIT_TEMPERATURE, source, reached);
        return true;
    }

    if (letter == 'M' && (number == 106 || number == 107)) {
        int32_t fan = 0;
        line.parameter_value('P', fan);
        std::vector<raw_gcode_reader_event_item> items;
        if (fan >= 0) {
            const std::optional<float> speed = number == 107 ?
                std::optional<float>(0.0f) : line.parameter_value('S');
            if (speed)
                items.push_back({RAW_GCODE_READER_ITEM_FAN_SPEED, uint32_t(fan),
                                 std::clamp(double(*speed) * 100.0, 0.0, 100.0)});
        }
        publish_items(run_context, context, state,
                      RAW_GCODE_READER_EVENT_STATE_UPDATE, source, items);
        return true;
    }

    if (letter != 'M' || number != 204)
        return false;
    std::optional<float> acceleration = line.parameter_value('S');
    if (!acceleration) {
        const std::optional<float> print = line.parameter_value('P');
        const std::optional<float> travel = line.parameter_value('T');
        if (print && !travel)
            acceleration = print;
        else if (travel && !print)
            acceleration = travel;
    }
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
           (letter == 'M' && (number == 201 || number == 203 || number == 205 ||
                              number == 566 || number == 572 || number == 900 ||
                              number == 906));
}

bool RepRapGCodeReader::is_marker_command(char letter, uint32_t number) const
{
    return letter == 'M' &&
           (number == 0 || number == 600 || number == 601 || number == 226);
}

}}} // namespace slic3r_api::GCodeGeneration::Reader
