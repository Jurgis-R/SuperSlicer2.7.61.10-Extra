///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "KlipperGCodeReader.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
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

EXCLUDE_OBJECT commands name objects with strings. The Reader assigns their
portable numeric IDs in a per-run registry instead of storing parser bookkeeping
inside the caller's custom state. This preserves opaque custom items exactly and
compares complete names rather than collision-prone numeric hashes.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Reader {
namespace {

// Resolve one Klipper extruder section name without inventing a tool when a
// custom or duplicate configuration makes the mapping ambiguous.
struct KlipperToolResolution
{
    enum class Status { Found, Unknown, Ambiguous };
    Status status = Status::Unknown;
    uint32_t tool = 0;
};
KlipperToolResolution klipper_tool_id(const run_ctx_gcode_reader &context,
                                      std::string_view name);
std::optional<std::string_view> klipper_object_name(
    const GCodeLineParser::GCodeLine &line);

KlipperToolResolution klipper_tool_id(const run_ctx_gcode_reader &context,
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
        if (matches.size() == 1)
            return {KlipperToolResolution::Status::Found, matches.front()};
        return {matches.empty() ? KlipperToolResolution::Status::Unknown :
                                  KlipperToolResolution::Status::Ambiguous, 0};
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
    if (matches.size() == 1)
        return {KlipperToolResolution::Status::Found, matches.front()};
    return {KlipperToolResolution::Status::Unknown, 0};
}

std::optional<std::string_view> klipper_object_name(
    const GCodeLineParser::GCodeLine &line)
{
    const std::string_view raw = line.raw();
    const size_t marker = raw.find("NAME=");
    if (marker == std::string_view::npos)
        return std::nullopt;
    const size_t value_begin = marker + 5;
    if (value_begin >= raw.size())
        return std::nullopt;

    const char quote = raw[value_begin];
    std::string_view name;
    if (quote == '\'' || quote == '"') {
        const size_t value_end = raw.find(quote, value_begin + 1);
        if (value_end == std::string_view::npos)
            return std::nullopt;
        name = raw.substr(value_begin + 1, value_end - value_begin - 1);
    } else {
        const size_t value_end = raw.find_first_of(" \t;", value_begin);
        name = raw.substr(value_begin, value_end == std::string_view::npos ?
            raw.size() - value_begin : value_end - value_begin);
    }
    return name.empty() ? std::nullopt : std::optional<std::string_view>(name);
}

} // namespace

KlipperGCodeReader::KlipperGCodeReader(orchestrator_handle *orchestrator) :
    BuiltinGCodeReaderBase(orchestrator)
{}

void KlipperGCodeReader::read_impl(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state) const
{
    {
        std::lock_guard<std::mutex> lock(m_object_registries_mutex);
        const std::pair<decltype(m_object_registries)::iterator, bool> inserted =
            m_object_registries.emplace(&state, ObjectRegistry{});
        if (!inserted.second)
            throw std::logic_error("A Klipper Reader state is already being parsed.");
    }

    try {
        BuiltinGCodeReaderBase::read_impl(run_context, context, state);
    } catch (...) {
        erase_object_registry(state);
        throw;
    }
    erase_object_registry(state);
}

uint32_t KlipperGCodeReader::find_or_add_object_id(
    const GCodeStateTracker &state, const std::string_view name) const
{
    std::lock_guard<std::mutex> lock(m_object_registries_mutex);
    const decltype(m_object_registries)::iterator run =
        m_object_registries.find(&state);
    if (run == m_object_registries.end())
        throw std::logic_error("Klipper object lookup occurred outside a Reader run.");

    ObjectRegistry &registry = run->second;
    const std::string owned_name(name);
    const std::unordered_map<std::string, uint32_t>::const_iterator existing =
        registry.ids.find(owned_name);
    if (existing != registry.ids.end())
        return existing->second;
    if (registry.next_id == std::numeric_limits<uint32_t>::max())
        throw std::overflow_error("Klipper object count exceeds uint32_t.");

    const uint32_t id = registry.next_id++;
    registry.ids.emplace(owned_name, id);
    return id;
}

void KlipperGCodeReader::erase_object_registry(
    const GCodeStateTracker &state) const noexcept
{
    std::lock_guard<std::mutex> lock(m_object_registries_mutex);
    m_object_registries.erase(&state);
}

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
    const GCodeLineParser::GCodeLine &line) const
{
    if (BuiltinGCodeReaderBase::consume_text_command(
            run_context, context, state, line))
        return true;
    const std::string_view command = line.command();
    if (command == "ACTIVATE_EXTRUDER") {
        process_ACTIVATE_EXTRUDER(run_context, context, state, line);
        return true;
    }
    if (command == "SET_PRESSURE_ADVANCE") {
        process_SET_PRESSURE_ADVANCE(run_context, context, state, line);
        return true;
    }
    if (command == "PAUSE") {
        process_PAUSE(run_context, context, state);
        return true;
    }
    if (command == "EXCLUDE_OBJECT_DEFINE") {
        process_EXCLUDE_OBJECT_DEFINE(run_context, context, state, line);
        return true;
    }
    if (command == "EXCLUDE_OBJECT_START") {
        process_EXCLUDE_OBJECT_START(run_context, context, state, line);
        return true;
    }
    if (command == "EXCLUDE_OBJECT_END") {
        process_EXCLUDE_OBJECT_END(run_context, context, state, line);
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

void KlipperGCodeReader::process_ACTIVATE_EXTRUDER(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &line) const
{
    const std::optional<std::string_view> name = named_parameter(line.raw(), "EXTRUDER");
    if (!name) {
        publish_unknown(run_context, context, state);
        return;
    }
    const KlipperToolResolution tool = klipper_tool_id(context, *name);
    if (tool.status == KlipperToolResolution::Status::Found) {
        publish_items(run_context, context, state,
                      RAW_GCODE_READER_EVENT_STATE_UPDATE,
                      {{RAW_GCODE_READER_ITEM_ACTIVE_TOOL, 0, double(tool.tool)}});
    } else {
        publish_diagnostic(
            run_context, context, state,
            tool.status == KlipperToolResolution::Status::Ambiguous ?
                RAW_GCODE_READER_DIAGNOSTIC_AMBIGUOUS_TOOL :
                RAW_GCODE_READER_DIAGNOSTIC_UNKNOWN_TOOL);
        publish_items(run_context, context, state,
                      RAW_GCODE_READER_EVENT_INVALIDATE,
                      {{RAW_GCODE_READER_ITEM_ACTIVE_TOOL, 0, 0.0}});
    }
}

void KlipperGCodeReader::process_SET_PRESSURE_ADVANCE(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &line) const
{
    const std::optional<std::string_view> advance_text =
        named_parameter(line.raw(), "ADVANCE");
    double advance = 0.0;
    if (!advance_text || advance_text->empty()) {
        publish_unknown(run_context, context, state);
        return;
    }
    const std::from_chars_result parsed = std::from_chars(
        advance_text->data(), advance_text->data() + advance_text->size(),
        advance);
    if (parsed.ec != std::errc() ||
        parsed.ptr != advance_text->data() + advance_text->size() ||
        !std::isfinite(advance)) {
        publish_unknown(run_context, context, state);
        return;
    }

    const std::optional<std::string_view> name =
        named_parameter(line.raw(), "EXTRUDER");
    if (name) {
        const KlipperToolResolution tool = klipper_tool_id(context, *name);
        if (tool.status != KlipperToolResolution::Status::Found) {
            publish_diagnostic(
                run_context, context, state,
                tool.status == KlipperToolResolution::Status::Ambiguous ?
                    RAW_GCODE_READER_DIAGNOSTIC_AMBIGUOUS_TOOL :
                    RAW_GCODE_READER_DIAGNOSTIC_UNKNOWN_TOOL);
            return;
        }
    }
    publish_diagnostic(
        run_context, context, state,
        RAW_GCODE_READER_DIAGNOSTIC_UNSUPPORTED_STATE_EFFECT);
}

void KlipperGCodeReader::process_PAUSE(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state) const
{
    publish_items(run_context, context, state,
                  RAW_GCODE_READER_EVENT_MARKER,
                  {{RAW_GCODE_READER_ITEM_MARKER, 0,
                    double(RAW_GCODE_READER_MARKER_PAUSE_PRINT)}});
}

void KlipperGCodeReader::process_EXCLUDE_OBJECT_DEFINE(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &line) const
{
    const std::optional<std::string_view> name = klipper_object_name(line);
    if (!name) {
        publish_unknown(run_context, context, state);
        return;
    }
    find_or_add_object_id(state, *name);
    publish_items(run_context, context, state,
                  RAW_GCODE_READER_EVENT_STATE_UPDATE, {});
}

void KlipperGCodeReader::process_EXCLUDE_OBJECT_START(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &line) const
{
    const std::optional<std::string_view> name = klipper_object_name(line);
    if (!name) {
        publish_unknown(run_context, context, state);
        return;
    }
    const uint32_t id = find_or_add_object_id(state, *name);
    const std::optional<double> current = state.value(RAW_GCODE_READER_ITEM_OBJECT);
    if (current && static_cast<uint32_t>(*current) == id)
        return;
    publish_items(run_context, context, state,
                  RAW_GCODE_READER_EVENT_STATE_UPDATE,
                  {{RAW_GCODE_READER_ITEM_OBJECT, 0, double(id)}});
}

void KlipperGCodeReader::process_EXCLUDE_OBJECT_END(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &) const
{
    const std::optional<double> current = state.value(RAW_GCODE_READER_ITEM_OBJECT);
    if (current && static_cast<uint32_t>(*current) == UINT32_MAX)
        return;
    publish_items(run_context, context, state,
                  RAW_GCODE_READER_EVENT_STATE_UPDATE,
                  {{RAW_GCODE_READER_ITEM_OBJECT, 0, double(UINT32_MAX)}});
}

void KlipperGCodeReader::process_M204(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    const GCodeLineParser::GCodeLine &line) const
{
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
                  RAW_GCODE_READER_EVENT_STATE_UPDATE, items);
}

bool KlipperGCodeReader::is_neutral_command(char letter, uint32_t number) const
{
    return letter == 'G' && number == 4;
}

bool KlipperGCodeReader::has_unsupported_state_effect(
    char letter, uint32_t number) const
{
    return (letter == 'G' && (number == 10 || number == 11)) ||
           (letter == 'M' && (number == 201 || number == 203 || number == 205 ||
                              number == 566 || number == 572 || number == 900 ||
                              number == 906));
}

std::optional<raw_gcode_reader_marker_type>
KlipperGCodeReader::marker_type(char letter, uint32_t number) const
{
    if (letter != 'M')
        return std::nullopt;
    if (number == 600)
        return RAW_GCODE_READER_MARKER_COLOR_CHANGE;
    if (number == 0 || number == 601 || number == 226)
        return RAW_GCODE_READER_MARKER_PAUSE_PRINT;
    return std::nullopt;
}

}}} // namespace slic3r_api::GCodeGeneration::Reader
