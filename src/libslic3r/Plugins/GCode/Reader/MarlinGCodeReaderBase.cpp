///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "MarlinGCodeReaderBase.hpp"

#include <optional>
#include <vector>

/*
Common Marlin-family command handling
=====================================

The shared dispatcher lets the portable Reader consume standard commands
first. Only M204 reaches a dialect-specific virtual handler. Temperature,
neutral-command and marker handling are identical for the four built-in
Marlin-family Readers and therefore stay here.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Reader {

MarlinGCodeReaderBase::MarlinGCodeReaderBase(orchestrator_handle *orchestrator) :
    BuiltinGCodeReaderBase(orchestrator)
{}

bool MarlinGCodeReaderBase::consume_numeric_command(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &line,
    char letter,
    uint32_t number) const
{
    if (BuiltinGCodeReaderBase::consume_numeric_command(
            run_context, context, state, line, letter, number))
        return true;
    if (letter != 'M' || number != 204)
        return false;
    process_M204(run_context, context, state, line);
    return true;
}

void MarlinGCodeReaderBase::process_M104_M109(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &line,
    uint32_t number) const
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
                          RAW_GCODE_READER_EVENT_WAIT_TEMPERATURE, items);
            return;
        }
    }
    BuiltinGCodeReaderBase::process_M104_M109(
        run_context, context, state, line, number);
}

bool MarlinGCodeReaderBase::is_neutral_command(char letter, uint32_t number) const
{
    return (letter == 'G' && (number == 4 || number == 10 || number == 11)) ||
           (letter == 'M' && (number == 201 || number == 203 || number == 205 ||
                              number == 566 || number == 572 || number == 900 ||
                              number == 906));
}

bool MarlinGCodeReaderBase::is_marker_command(char letter, uint32_t number) const
{
    return letter == 'M' &&
           (number == 0 || number == 600 || number == 601 || number == 226);
}

}}} // namespace slic3r_api::GCodeGeneration::Reader
