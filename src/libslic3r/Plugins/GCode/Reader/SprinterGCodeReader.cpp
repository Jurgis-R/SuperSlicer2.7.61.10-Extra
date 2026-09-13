///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "SprinterGCodeReader.hpp"

#include <optional>
#include <vector>

/*
Sprinter Reader provider
========================

The provider reuses portable and Marlin-family parsing, recognizes Sprinter
files during probing and maps the dialect's M204 P command to the portable
current acceleration.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Reader {

SprinterGCodeReader::SprinterGCodeReader(orchestrator_handle *orchestrator) :
    MarlinGCodeReaderBase(orchestrator)
{}

const char *SprinterGCodeReader::id_impl() const noexcept
{
    return "gcode.reader.sprinter";
}

const char *SprinterGCodeReader::name_impl() const noexcept
{
    return "Sprinter G-code reader";
}

const char *SprinterGCodeReader::description_impl() const noexcept
{
    return "Reads Sprinter G-code into normalized machine-state events.";
}

uint32_t SprinterGCodeReader::probe_impl(std::string_view prefix) const
{
    return probe_marker(prefix, "sprinter");
}

void SprinterGCodeReader::process_M204(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &line) const
{
    std::vector<raw_gcode_reader_event_item> items;
    if (const std::optional<float> acceleration = line.parameter_value('P'))
        items.push_back({RAW_GCODE_READER_ITEM_ACCELERATION, 0, double(*acceleration)});
    publish_items(run_context, context, state,
                  RAW_GCODE_READER_EVENT_STATE_UPDATE, items);
}

}}} // namespace slic3r_api::GCodeGeneration::Reader
