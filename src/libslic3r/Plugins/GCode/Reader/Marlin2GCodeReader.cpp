///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "Marlin2GCodeReader.hpp"

#include <optional>
#include <vector>

/*
Marlin 2 Reader provider
========================

This concrete provider supplies the default Marlin probe and interprets the
P/T forms emitted by the Marlin 2 firmware. Prusa derives from it because its
movement and acceleration vocabulary follows the same rules.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Reader {

Marlin2GCodeReader::Marlin2GCodeReader(orchestrator_handle *orchestrator) :
    MarlinGCodeReaderBase(orchestrator)
{}

const char *Marlin2GCodeReader::id_impl() const noexcept
{
    return "gcode.reader.marlin2";
}

const char *Marlin2GCodeReader::name_impl() const noexcept
{
    return "Marlin 2 G-code reader";
}

const char *Marlin2GCodeReader::description_impl() const noexcept
{
    return "Reads Marlin 2 G-code into normalized machine-state events.";
}

uint32_t Marlin2GCodeReader::probe_impl(std::string_view prefix) const
{
    if (probe_marker(prefix, "marlin 2") || probe_marker(prefix, "marlin2"))
        return 90;
    return probe_marker(prefix, "marlin") ? 80u : 0u;
}

void Marlin2GCodeReader::process_M204(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &line) const
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
                  RAW_GCODE_READER_EVENT_STATE_UPDATE, items);
}

}}} // namespace slic3r_api::GCodeGeneration::Reader
