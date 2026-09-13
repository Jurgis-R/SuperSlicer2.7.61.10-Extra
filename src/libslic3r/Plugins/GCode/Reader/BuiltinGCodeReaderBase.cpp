///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "BuiltinGCodeReaderBase.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_gcode_reader.h"

/*
Built-in G-code reader base
===========================

This file translates the common modal G-code vocabulary into normalized Reader
events. Concrete providers first receive a chance to handle dialect syntax,
then this parser handles portable movement, temperature, fan and mode commands.

One run owns one local GCodeStateTracker, so the same provider can serve a
visible file or a firmware script without retaining input or parser state in
the plugin singleton.

Input may arrive in arbitrary chunks. The parser retains only an unfinished
last line during run(), assigns byte ranges while complete lines are consumed,
then emits a normalized event for each command. Unknown commands invalidate
portable and private machine state instead of letting a later host-generated
command reuse a value which arbitrary dialect G-code may have changed.

    arbitrary chunks
          |
          v
    pending complete lines -- incomplete tail retained across chunks
          |
          v
    GCodeLineParser
          |
          +-- supported command -> semantic event + state delta
          `-- unknown command   -> INVALIDATE currently known portable state

Each implementation intentionally recognizes a conservative subset. A command
is only treated as harmless when the dialect guarantees that it cannot alter a
portable state item; otherwise an unsupported command invalidates known state
so the firmware writer cannot continue from guessed coordinates, modes or tools.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Reader {
namespace {

const char *const k_no_dependencies[] = {nullptr};

// Return an unsigned command suffix only when every character after the command
// letter is decimal. Rejecting signs and suffixes prevents partial parses such
// as "G1foo" from being mistaken for a supported command.
std::optional<uint32_t> command_number(std::string_view command);
// Convert values expressed in the current input units to the millimetres used
// by the normalized Reader state.
double unit_scale(const GCodeStateTracker &state);
// Read a modal boolean, using the command family's standard default when the
// initial state did not establish the mode.
bool modal_bool(const GCodeStateTracker &state, uint32_t type, bool fallback);
// Return the selected logical tool, or conventional tool zero when selection
// is absent from an incomplete initial state.
uint32_t active_tool_value(const GCodeStateTracker &state);
// Resolve one parsed coordinate to its final absolute value. Relative values
// require a known previous coordinate; an empty result tells consume_line() to
// invalidate that axis rather than inventing an origin.
std::optional<double> final_axis_value(const GCodeStateTracker &state,
                                       uint32_t type,
                                       uint32_t target,
                                       std::optional<float> parsed,
                                       bool relative);

std::optional<uint32_t> command_number(std::string_view command)
{
    if (command.size() < 2)
        return std::nullopt;
    uint32_t number = 0;
    const std::from_chars_result result = std::from_chars(
        command.data() + 1, command.data() + command.size(), number);
    if (result.ec != std::errc() || result.ptr != command.data() + command.size())
        return std::nullopt;
    return number;
}

// Normalize textual coordinates to millimetres. Unknown units use the usual
// G-code default of millimetres, which keeps an incomplete initial snapshot
// useful without claiming it was explicitly configured.
double unit_scale(const GCodeStateTracker &state)
{
    const std::optional<double> millimetres = state.value(RAW_GCODE_READER_ITEM_UNITS_MM);
    return !millimetres || *millimetres != 0.0 ? 1.0 : 25.4;
}

// Read a modal boolean while preserving the command-specific default when the
// input state did not establish that mode.
bool modal_bool(const GCodeStateTracker &state, uint32_t type, bool fallback)
{
    const std::optional<double> value = state.value(type);
    return value ? *value != 0.0 : fallback;
}

// E coordinates and tool temperatures need a target. Unknown selection uses
// tool zero, matching the conventional single-tool G-code default.
uint32_t active_tool_value(const GCodeStateTracker &state)
{
    const std::optional<double> tool = state.value(RAW_GCODE_READER_ITEM_ACTIVE_TOOL);
    return tool ? static_cast<uint32_t>(*tool) : 0;
}

std::optional<double> final_axis_value(const GCodeStateTracker &state,
                                       uint32_t type,
                                       uint32_t target,
                                       std::optional<float> parsed,
                                       bool relative)
{
    // Absolute parameters directly replace a coordinate. Relative parameters
    // need a known origin; absence means the reader must later invalidate that
    // axis instead of inventing a destination.
    if (!parsed)
        return std::nullopt;
    const double value = double(*parsed) * unit_scale(state);
    if (!relative)
        return value;
    const std::optional<double> previous = state.value(type, target);
    return previous ? std::optional<double>(*previous + value) : std::nullopt;
}

} // namespace

uint32_t BuiltinGCodeReaderBase::active_tool(const GCodeStateTracker &state)
{
    return active_tool_value(state);
}

uint32_t BuiltinGCodeReaderBase::probe_marker(std::string_view text, std::string_view needle)
{
    if (needle.empty() || needle.size() > text.size())
        return 0;
    return std::search(text.begin(), text.end(), needle.begin(), needle.end(),
        [](char left, char right) {
            return std::tolower(static_cast<unsigned char>(left)) ==
                   std::tolower(static_cast<unsigned char>(right));
        }) != text.end() ? 100u : 0u;
}

std::optional<std::string_view> BuiltinGCodeReaderBase::named_parameter(
    std::string_view line, std::string_view name)
{
    size_t cursor = line.find_first_of(" \t");
    while (cursor != std::string_view::npos) {
        cursor = line.find_first_not_of(" \t", cursor);
        if (cursor == std::string_view::npos || line[cursor] == ';')
            break;
        const size_t end = line.find_first_of(" \t;", cursor);
        const std::string_view word = line.substr(
            cursor, end == std::string_view::npos ? line.size() - cursor : end - cursor);
        if (word.size() > name.size() && word.substr(0, name.size()) == name &&
            word[name.size()] == '=')
            return word.substr(name.size() + 1);
        cursor = end;
    }
    return std::nullopt;
}

BuiltinGCodeReaderBase::BuiltinGCodeReaderBase(orchestrator_handle *orchestrator) :
    GCodeReaderPluginBase(orchestrator)
{}

const char *const *BuiltinGCodeReaderBase::dependencies_impl() const noexcept
{
    return k_no_dependencies;
}

int32_t BuiltinGCodeReaderBase::priority_impl() const noexcept
{
    return 0;
}

bool BuiltinGCodeReaderBase::consume_text_command(
    const plugin_run_context &, const run_ctx_gcode_reader &, GCodeStateTracker &,
    const GCodeLineParser::GCodeLine &, raw_gcode_reader_source) const
{
    return false;
}

bool BuiltinGCodeReaderBase::consume_numeric_command(
    const plugin_run_context &, const run_ctx_gcode_reader &, GCodeStateTracker &,
    const GCodeLineParser::GCodeLine &, char, uint32_t, raw_gcode_reader_source) const
{
    return false;
}

bool BuiltinGCodeReaderBase::is_neutral_command(char, uint32_t) const
{
    return false;
}

bool BuiltinGCodeReaderBase::is_marker_command(char, uint32_t) const
{
    return false;
}

void BuiltinGCodeReaderBase::validate_private_state(const GCodeStateTracker &) const
{}

void BuiltinGCodeReaderBase::read_impl(const plugin_run_context &run_context,
                                       const run_ctx_gcode_reader &context,
                                       GCodeStateTracker &state) const
{
    validate_private_state(state);
    // Chunks may split anywhere, including in CRLF and numeric parameters.
    // Retain only bytes after the last complete newline between pulls.
    std::string pending;
    uint64_t pending_begin = 0;
    uint64_t supplied_bytes = 0;
    std::string_view chunk;
    while (next_chunk(run_context, context, chunk)) {
        if (chunk.size() > std::numeric_limits<uint64_t>::max() - supplied_bytes)
            throw std::overflow_error("G-code reader input exceeds the uint64_t source range.");
        supplied_bytes += static_cast<uint64_t>(chunk.size());
        pending.append(chunk.data(), chunk.size());
        size_t consumed = 0;
        size_t line_end = 0;
        while ((line_end = pending.find('\n', consumed)) != std::string::npos) {
            const size_t line_begin = consumed;
            const size_t text_size = line_end > 0 && pending[line_end - 1] == '\r' ? line_end - 1 : line_end;
            const size_t line_size = text_size - line_begin;
            consume_line(run_context, context, state,
                         std::string_view(pending.data() + line_begin, line_size),
                         pending_begin + line_begin, pending_begin + text_size);
            consumed = line_end + 1;
            state.advance_line_number();
        }

        // Compact once per host chunk, not once per line. Repeatedly erasing
        // the front would move all remaining bytes for every physical line
        // and make a large input chunk quadratic to consume.
        if (consumed != 0) {
            pending.erase(0, consumed);
            pending_begin += static_cast<uint64_t>(consumed);
        }
    }
    if (!pending.empty())
        consume_line(run_context, context, state, pending, pending_begin, pending_begin + pending.size());
    validate_private_state(state);
}

void BuiltinGCodeReaderBase::publish_items(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    raw_gcode_reader_event_kind kind,
    raw_gcode_reader_source source,
    const std::vector<raw_gcode_reader_event_item> &items) const
{
    // LINE_NUMBER is mandatory in every event and belongs to this session's
    // cursor, not to a parser branch. Copy the supplied delta before adding it
    // so the caller's temporary vector remains untouched.
    std::vector<raw_gcode_reader_event_item> event_items;
    event_items.reserve(items.size() + 1);
    for (const raw_gcode_reader_event_item &item : items) {
        if (item.type == RAW_GCODE_READER_ITEM_LINE_NUMBER)
            throw std::logic_error("Built-in G-code reader must own line-number event items.");
        event_items.push_back(item);
    }
    event_items.push_back({RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, double(state.line_number())});

    raw_gcode_reader_event event = {};
    event.kind = kind;
    event.source = source;
    event.items = event_items.data();
    event.item_count = static_cast<uint32_t>(event_items.size());
    publish(run_context, context, state, event);
}

void BuiltinGCodeReaderBase::publish_state_update(
    const plugin_run_context &run_context,
    const run_ctx_gcode_reader &context,
    GCodeStateTracker &state,
    raw_gcode_reader_source source) const
{
    publish_items(run_context, context, state,
                  RAW_GCODE_READER_EVENT_STATE_UPDATE, source, {});
}

void BuiltinGCodeReaderBase::publish_unknown(const plugin_run_context &run_context,
                                             const run_ctx_gcode_reader &context,
                                             GCodeStateTracker &state,
                                             raw_gcode_reader_source source) const
{
    // One unsupported command may have changed any portable or private value. Send
    // an INVALIDATE item for every currently known value so downstream writers
    // re-establish state explicitly instead of continuing from stale data.
    // Use the exact current keys, including dynamically discovered tool and
    // fan targets. A future extension is therefore invalidated safely too.
    const raw_gcode_reader_state &snapshot = state.state();
    std::vector<raw_gcode_reader_event_item> items;
    for (uint32_t index = 0; index < snapshot.item_count; ++index) {
        const raw_gcode_reader_event_item &item = snapshot.items[index];
        if (item.type != RAW_GCODE_READER_ITEM_LINE_NUMBER &&
            item.type != RAW_GCODE_READER_ITEM_READER_ID)
            items.push_back(item);
    }
    publish_items(run_context, context, state, RAW_GCODE_READER_EVENT_INVALIDATE, source, items);
}

void BuiltinGCodeReaderBase::consume_line(const plugin_run_context &run_context,
                                         const run_ctx_gcode_reader &context,
                                         GCodeStateTracker &state,
                                         std::string_view text,
                                         uint64_t begin,
                                         uint64_t end) const
{
    // Convert one parsed command into the shared event vocabulary. Commands
    // outside the common subset and the derived dialect hooks invalidate
    // portable state so later output never relies on guessed machine values.
    GCodeLineParser parser;
    parser.parse_line(std::string(text),
                      [this, &run_context, &context, &state, begin, end](
                          GCodeLineParser &, const GCodeLineParser::GCodeLine &line) {
        const std::string_view command = line.command();
        if (command.empty())
            return;
        raw_gcode_reader_source source = {begin, end};

        if (consume_text_command(run_context, context, state, line, source))
            return;

        const char letter = command.front();
        const std::optional<uint32_t> number = command_number(command);
        if (!number)
            return publish_unknown(run_context, context, state, source);
        if (consume_numeric_command(run_context, context, state, line,
                                    letter, *number, source))
            return;

        std::vector<raw_gcode_reader_event_item> items;
        if (letter == 'G' && (*number == 0 || *number == 1 || *number == 2 || *number == 3)) {
            const bool xyz_relative = modal_bool(state, RAW_GCODE_READER_ITEM_XYZ_ABSOLUTE, true) == false;
            const bool e_relative = modal_bool(state, RAW_GCODE_READER_ITEM_E_ABSOLUTE, true) == false;
            const std::pair<char, uint32_t> axes[] = {
                {'X', RAW_GCODE_READER_ITEM_X}, {'Y', RAW_GCODE_READER_ITEM_Y},
                {'Z', RAW_GCODE_READER_ITEM_Z}, {'E', RAW_GCODE_READER_ITEM_E}
            };
            std::vector<raw_gcode_reader_event_item> invalidated;
            for (const std::pair<char, uint32_t> &axis : axes) {
                const uint32_t target = axis.first == 'E' ? active_tool(state) : 0;
                const std::optional<double> value = final_axis_value(
                    state, axis.second, target, line.parameter_value(axis.first),
                    axis.first == 'E' ? e_relative : xyz_relative);
                if (value)
                    items.push_back({axis.second, target, *value});
                else if (line.has_parameter(axis.first))
                    invalidated.push_back({axis.second, target, 0.0});
            }
            if (const std::optional<float> speed = line.parameter_value('F'))
                items.push_back({RAW_GCODE_READER_ITEM_FEEDRATE, 0, double(*speed) * unit_scale(state)});
            if (*number == 2 || *number == 3) {
                const std::pair<char, uint32_t> centers[] = {
                    {'I', RAW_GCODE_READER_ITEM_ARC_CENTER_X},
                    {'J', RAW_GCODE_READER_ITEM_ARC_CENTER_Y},
                    {'K', RAW_GCODE_READER_ITEM_ARC_CENTER_Z}
                };
                for (const std::pair<char, uint32_t> &center : centers)
                    if (const std::optional<float> value = line.parameter_value(center.first))
                        items.push_back({center.second, 0, double(*value) * unit_scale(state)});
                items.push_back({RAW_GCODE_READER_ITEM_ARC_CLOCKWISE, 0, *number == 2 ? 1.0 : 0.0});
            }
            // A relative move needs a known origin. Keep the structural move
            // event for downstream timeline consumers, but explicitly discard
            // each axis whose final coordinate cannot be derived from the
            // previous state.
            if (!invalidated.empty())
                publish_items(run_context, context, state,
                              RAW_GCODE_READER_EVENT_INVALIDATE, source, invalidated);
            publish_items(run_context, context, state,
                          *number <= 1 ? RAW_GCODE_READER_EVENT_LINEAR_MOVE : RAW_GCODE_READER_EVENT_ARC_MOVE,
                          source, items);
            return;
        }
        if (letter == 'G' && *number == 20)
            items.push_back({RAW_GCODE_READER_ITEM_UNITS_MM, 0, 0.0});
        else if (letter == 'G' && *number == 21)
            items.push_back({RAW_GCODE_READER_ITEM_UNITS_MM, 0, 1.0});
        else if (letter == 'G' && *number == 90)
            items.push_back({RAW_GCODE_READER_ITEM_XYZ_ABSOLUTE, 0, 1.0});
        else if (letter == 'G' && *number == 91)
            items.push_back({RAW_GCODE_READER_ITEM_XYZ_ABSOLUTE, 0, 0.0});
        else if (letter == 'G' && *number == 92) {
            const std::pair<char, uint32_t> axes[] = {
                {'X', RAW_GCODE_READER_ITEM_X}, {'Y', RAW_GCODE_READER_ITEM_Y},
                {'Z', RAW_GCODE_READER_ITEM_Z}, {'E', RAW_GCODE_READER_ITEM_E}
            };
            for (const std::pair<char, uint32_t> &axis : axes)
                if (const std::optional<float> value = line.parameter_value(axis.first))
                    items.push_back({axis.second, axis.first == 'E' ? active_tool(state) : 0,
                                     double(*value) * unit_scale(state)});
            publish_items(run_context, context, state,
                          RAW_GCODE_READER_EVENT_SET_POSITION, source, items);
            return;
        } else if (letter == 'M' && *number == 82)
            items.push_back({RAW_GCODE_READER_ITEM_E_ABSOLUTE, 0, 1.0});
        else if (letter == 'M' && *number == 83)
            items.push_back({RAW_GCODE_READER_ITEM_E_ABSOLUTE, 0, 0.0});
        else if (letter == 'T')
            items.push_back({RAW_GCODE_READER_ITEM_ACTIVE_TOOL, 0, double(*number)});
        else if (letter == 'M' && (*number == 104 || *number == 109)) {
            std::optional<float> temperature = line.parameter_value('S');
            if (temperature) {
                int32_t requested_tool = int32_t(active_tool(state));
                line.parameter_value('T', requested_tool);
                if (requested_tool >= 0)
                    items.push_back({RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE,
                                     uint32_t(requested_tool), double(*temperature)});
                publish_items(run_context, context, state,
                              *number == 109 ? RAW_GCODE_READER_EVENT_WAIT_TEMPERATURE :
                                               RAW_GCODE_READER_EVENT_SET_TEMPERATURE,
                              source, items);
                return;
            }
        } else if (letter == 'M' && (*number == 140 || *number == 190)) {
            if (const std::optional<float> temperature = line.parameter_value('S')) {
                items.push_back({RAW_GCODE_READER_ITEM_BED_TEMPERATURE, 0, double(*temperature)});
                publish_items(run_context, context, state,
                              *number == 190 ? RAW_GCODE_READER_EVENT_WAIT_TEMPERATURE :
                                               RAW_GCODE_READER_EVENT_SET_TEMPERATURE,
                              source, items);
                return;
            }
        } else if (letter == 'M' && (*number == 141 || *number == 191)) {
            if (const std::optional<float> temperature = line.parameter_value('S')) {
                items.push_back({RAW_GCODE_READER_ITEM_CHAMBER_TEMPERATURE, 0, double(*temperature)});
                publish_items(run_context, context, state,
                              *number == 191 ? RAW_GCODE_READER_EVENT_WAIT_TEMPERATURE :
                                               RAW_GCODE_READER_EVENT_SET_TEMPERATURE,
                              source, items);
                return;
            }
        } else if (letter == 'M' && (*number == 106 || *number == 107)) {
            int32_t fan = 0;
            line.parameter_value('P', fan);
            if (fan >= 0) {
                const std::optional<float> raw_speed = *number == 107 ?
                    std::optional<float>(0.0f) : line.parameter_value('S');
                if (raw_speed) {
                    items.push_back({RAW_GCODE_READER_ITEM_FAN_SPEED, uint32_t(fan),
                                     std::clamp(double(*raw_speed) * 100.0 / 255.0, 0.0, 100.0)});
                }
            }
        } else if (letter == 'M' && *number == 220) {
            if (const std::optional<float> factor = line.parameter_value('S'))
                items.push_back({RAW_GCODE_READER_ITEM_SPEED_FACTOR, 0, double(*factor)});
        } else if (letter == 'M' && *number == 221) {
            if (const std::optional<float> factor = line.parameter_value('S'))
                items.push_back({RAW_GCODE_READER_ITEM_EXTRUSION_FACTOR, 0, double(*factor)});
        } else if (letter == 'M' && *number == 117) {
            // M117 displays text on the printer and does not change portable
            // motion state. Publish its source for viewers without invalidating
            // coordinates, modes or the active tool needed by the next move.
            publish_items(run_context, context, state,
                          RAW_GCODE_READER_EVENT_DIAGNOSTIC, source, {});
            return;
        } else if (is_neutral_command(letter, *number)) {
            // These built-in commands affect timing, machine limits, pressure
            // advance, current or firmware retraction, none of which has a
            // portable scalar in raw_gcode_reader_state. Recognizing them as
            // neutral preserves the state values they cannot change.
        } else if (is_marker_command(letter, *number)) {
            publish_items(run_context, context, state,
                          RAW_GCODE_READER_EVENT_MARKER, source, {});
            return;
        } else {
            publish_unknown(run_context, context, state, source);
            return;
        }
        publish_items(run_context, context, state,
                      RAW_GCODE_READER_EVENT_STATE_UPDATE, source, items);
    });
}

}}} // namespace slic3r_api::GCodeGeneration::Reader
