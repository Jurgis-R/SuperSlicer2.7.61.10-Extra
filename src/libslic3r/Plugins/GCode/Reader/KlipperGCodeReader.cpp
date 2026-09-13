///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "KlipperGCodeReader.hpp"

#include <algorithm>
#include <charconv>
#include <optional>
#include <string_view>
#include <vector>

#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"

/*
Klipper command extensions
==========================

Text commands are handled before the base attempts to parse a G/M number.
ACTIVATE_EXTRUDER maps Klipper section names to the portable numeric tool. A
slicing Print makes configured tool_name values authoritative; an external file
without a Print accepts only the conventional extruder/extruderN spelling.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Reader {
namespace {

// Resolve one Klipper extruder section name without inventing a tool when a
// custom or duplicate configuration makes the mapping ambiguous.
std::optional<uint32_t> klipper_tool_id(const run_ctx_gcode_reader &context,
                                        std::string_view name);

std::optional<uint32_t> klipper_tool_id(const run_ctx_gcode_reader &context,
                                        std::string_view name)
{
    std::vector<uint32_t> matches;
    if (context.print != nullptr) {
        const Config config = Print(context.print).config();
        if (config.has("tool_name")) {
            const ConfigOption names = config.get("tool_name");
            for (uint32_t index = 0; index < names.size(); ++index)
                if (names.is_enabled(index) && names.get_string(index) == name)
                    matches.push_back(index);
        }
        return matches.size() == 1 ?
            std::optional<uint32_t>(matches.front()) : std::nullopt;
    }

    if (name == "extruder") {
        matches.push_back(0);
    } else if (name.size() > 8 && name.substr(0, 8) == "extruder") {
        uint32_t index = 0;
        const std::from_chars_result parsed = std::from_chars(
            name.data() + 8, name.data() + name.size(), index);
        if (parsed.ec == std::errc() && parsed.ptr == name.data() + name.size())
            matches.push_back(index);
    }
    return matches.size() == 1 ? std::optional<uint32_t>(matches.front()) : std::nullopt;
}

} // namespace

KlipperGCodeReader::KlipperGCodeReader(orchestrator_handle *orchestrator) :
    BuiltinGCodeReaderBase(orchestrator)
{}

const char *KlipperGCodeReader::id_impl() const noexcept
{
    return "gcode.reader.klipper";
}

const char *KlipperGCodeReader::name_impl() const noexcept
{
    return "Klipper G-code reader";
}

const char *KlipperGCodeReader::description_impl() const noexcept
{
    return "Reads Klipper G-code into normalized machine-state events.";
}

uint32_t KlipperGCodeReader::probe_impl(std::string_view prefix) const
{
    return probe_marker(prefix, "klipper");
}

bool KlipperGCodeReader::consume_text_command(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &line,
    raw_gcode_reader_source source) const
{
    const std::string_view command = line.command();
    if (command == "ACTIVATE_EXTRUDER") {
        const std::optional<std::string_view> name = named_parameter(line.raw(), "EXTRUDER");
        if (!name)
            return false;
        const std::optional<uint32_t> tool = klipper_tool_id(context, *name);
        if (tool) {
            publish_items(run_context, context, state,
                          RAW_GCODE_READER_EVENT_STATE_UPDATE, source,
                          {{RAW_GCODE_READER_ITEM_ACTIVE_TOOL, 0, double(*tool)}});
        } else {
            publish_items(run_context, context, state,
                          RAW_GCODE_READER_EVENT_INVALIDATE, source,
                          {{RAW_GCODE_READER_ITEM_ACTIVE_TOOL, 0, 0.0}});
        }
        return true;
    }
    if (command == "SET_PRESSURE_ADVANCE" && named_parameter(line.raw(), "ADVANCE")) {
        publish_state_update(run_context, context, state, source);
        return true;
    }
    if (command == "PAUSE") {
        publish_items(run_context, context, state,
                      RAW_GCODE_READER_EVENT_MARKER, source, {});
        return true;
    }
    return false;
}

bool KlipperGCodeReader::consume_numeric_command(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &line,
    char letter,
    uint32_t number,
    raw_gcode_reader_source source) const
{
    if (letter != 'M' || number != 204)
        return false;
    std::optional<float> acceleration = line.parameter_value('S');
    if (!acceleration) {
        const std::optional<float> print = line.parameter_value('P');
        const std::optional<float> travel = line.parameter_value('T');
        if (print && travel)
            acceleration = std::min(*print, *travel);
        else if (print)
            acceleration = print;
        else if (travel)
            acceleration = travel;
    }
    std::vector<raw_gcode_reader_event_item> items;
    if (acceleration)
        items.push_back({RAW_GCODE_READER_ITEM_ACCELERATION, 0, double(*acceleration)});
    publish_items(run_context, context, state,
                  RAW_GCODE_READER_EVENT_STATE_UPDATE, source, items);
    return true;
}

bool KlipperGCodeReader::is_neutral_command(char letter, uint32_t number) const
{
    return (letter == 'G' && (number == 4 || number == 10 || number == 11)) ||
           (letter == 'M' && (number == 201 || number == 203 || number == 205 ||
                              number == 566 || number == 572 || number == 900 ||
                              number == 906));
}

bool KlipperGCodeReader::is_marker_command(char letter, uint32_t number) const
{
    return letter == 'M' &&
           (number == 0 || number == 600 || number == 601 || number == 226);
}

}}} // namespace slic3r_api::GCodeGeneration::Reader
