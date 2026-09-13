///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "Marlin1GCodeReader.hpp"

#include <optional>
#include <vector>

/*
Marlin 1 Reader provider
========================

The provider owns no stream state. It identifies Marlin 1 input during probing
and specializes only the shared M204 S acceleration command; all other parsing
is inherited from the Marlin-family and portable bases.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Reader {

Marlin1GCodeReader::Marlin1GCodeReader(orchestrator_handle *orchestrator) :
    MarlinGCodeReaderBase(orchestrator)
{}

const char *Marlin1GCodeReader::id_impl() const noexcept
{
    return "gcode.reader.marlin1";
}

const char *Marlin1GCodeReader::name_impl() const noexcept
{
    return "Marlin 1 G-code reader";
}

const char *Marlin1GCodeReader::description_impl() const noexcept
{
    return "Reads Marlin 1 G-code into normalized machine-state events.";
}

uint32_t Marlin1GCodeReader::probe_impl(std::string_view prefix) const
{
    if (probe_marker(prefix, "marlin 1") || probe_marker(prefix, "marlin1"))
        return 90;
    return probe_marker(prefix, "marlin") ? 70u : 0u;
}

void Marlin1GCodeReader::process_M204(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &line) const
{
    std::vector<raw_gcode_reader_event_item> items;
    if (const std::optional<float> acceleration = line.parameter_value('S'))
        items.push_back({RAW_GCODE_READER_ITEM_ACCELERATION, 0, double(*acceleration)});
    publish_items(run_context, context, state,
                  RAW_GCODE_READER_EVENT_STATE_UPDATE, items);
}

}}} // namespace slic3r_api::GCodeGeneration::Reader
