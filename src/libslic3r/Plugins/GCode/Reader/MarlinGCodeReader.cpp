///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "MarlinGCodeReader.hpp"

#include <optional>
#include <vector>

/*
Marlin command extensions
=========================

Marlin mostly follows the common numeric G-code syntax. Its dispatcher asks the
common base first, adds M204 only when the base declines the command, and
customizes the already-common M109 through a virtual handler override. This
keeps one dispatch path while still accepting Marlin's cooling target syntax.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Reader {

MarlinGCodeReader::MarlinGCodeReader(orchestrator_handle *orchestrator) :
    BuiltinGCodeReaderBase(orchestrator)
{}

const char *MarlinGCodeReader::id_impl() const noexcept
{
    return "gcode.reader.marlin";
}

const char *MarlinGCodeReader::name_impl() const noexcept
{
    return "Marlin G-code reader";
}

const char *MarlinGCodeReader::description_impl() const noexcept
{
    return "Reads Marlin, Prusa and Sprinter G-code into normalized machine-state events.";
}

uint32_t MarlinGCodeReader::probe_impl(std::string_view prefix) const
{
    return probe_marker(prefix, "marlin");
}

bool MarlinGCodeReader::consume_numeric_command(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &line,
    char letter,
    uint32_t number,
    raw_gcode_reader_source source) const
{
    if (BuiltinGCodeReaderBase::consume_numeric_command(
            run_context, context, state, line, letter, number, source))
        return true;
    if (letter != 'M' || number != 204)
        return false;
    process_M204(run_context, context, state, line, source);
    return true;
}

void MarlinGCodeReader::process_M104_M109(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &line,
    uint32_t number,
    raw_gcode_reader_source source) const
{
    // R on M109 selects the same target as S but also waits while cooling.
    if (number == 109 && !line.parameter_value('S')) {
        const std::optional<float> temperature = line.parameter_value('R');
        if (temperature) {
            int32_t requested_tool = int32_t(active_tool(state));
            line.parameter_value('T', requested_tool);
            std::vector<raw_gcode_reader_event_item> items;
            if (requested_tool >= 0)
                items.push_back({RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE,
                                 uint32_t(requested_tool), double(*temperature)});
            publish_items(run_context, context, state,
                          RAW_GCODE_READER_EVENT_WAIT_TEMPERATURE, source, items);
            return;
        }
    }
    BuiltinGCodeReaderBase::process_M104_M109(
        run_context, context, state, line, number, source);
}

void MarlinGCodeReader::process_M204(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &line,
    raw_gcode_reader_source source) const
{
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
}

bool MarlinGCodeReader::is_neutral_command(char letter, uint32_t number) const
{
    return (letter == 'G' && (number == 4 || number == 10 || number == 11)) ||
           (letter == 'M' && (number == 201 || number == 203 || number == 205 ||
                              number == 566 || number == 572 || number == 900 ||
                              number == 906));
}

bool MarlinGCodeReader::is_marker_command(char letter, uint32_t number) const
{
    return letter == 'M' &&
           (number == 0 || number == 600 || number == 601 || number == 226);
}

}}} // namespace slic3r_api::GCodeGeneration::Reader
