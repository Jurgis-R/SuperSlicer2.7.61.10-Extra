///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Plugins_GCode_Reader_BuiltinGCodeReaderBase_hpp_
#define slic3r_Plugins_GCode_Reader_BuiltinGCodeReaderBase_hpp_

#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

#include "libslic3r/Api/plugin/cpp/gcode/GCodeLineParser.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeReaderPlugin.hpp"

/*
Built-in G-code reader base
===========================

BuiltinGCodeReaderBase implements the stream and state machinery shared by the
built-in Readers. A concrete dialect only describes its metadata and overrides
the smallest hook matching a syntax difference:

    chunks -> complete physical line -> textual dialect hook
                                     -> numeric dialect hook
                                     -> common modal parser
                                     -> marker/neutral classification
                                     -> conservative state invalidation

The textual hook is for commands such as Klipper's NAME=value syntax. The
numeric hook is for dialect-specific G/M commands or variants of a common
command. The two predicates classify commands which either produce a timeline
marker or are understood but have no portable Reader state item.

Returning true from a command hook means that the derived Reader has completely
handled the command and published the required event. Returning false delegates
to the next stage. Unknown commands ultimately invalidate portable state so a
later writer never relies on values an unsupported command may have changed.

Plugin instances contain no stream state. Each read creates its own tracker and
line buffer, so immutable provider instances may be executed concurrently.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Reader {

class BuiltinGCodeReaderBase : public GCodeReaderPluginBase
{
public:
    explicit BuiltinGCodeReaderBase(orchestrator_handle *orchestrator);

protected:
    // Handle a non-numeric dialect command before command-number parsing.
    // A true result means the hook has synchronously published its event.
    virtual bool consume_text_command(const plugin_run_context &run_context,
                                      const run_ctx_gcode_reader &context,
                                      GCodeStateTracker &state,
                                      const GCodeLineParser::GCodeLine &line,
                                      raw_gcode_reader_source source) const;
    // Handle a numeric dialect command before the common modal parser. This is
    // also where a dialect overrides a shared command's parameter semantics.
    virtual bool consume_numeric_command(const plugin_run_context &run_context,
                                         const run_ctx_gcode_reader &context,
                                         GCodeStateTracker &state,
                                         const GCodeLineParser::GCodeLine &line,
                                         char letter,
                                         uint32_t number,
                                         raw_gcode_reader_source source) const;
    // Return true for a command understood by the dialect which cannot alter
    // any value represented by raw_gcode_reader_state.
    virtual bool is_neutral_command(char letter, uint32_t number) const;
    // Return true for a pause or filament-change command represented as a
    // timeline marker rather than a portable machine-state update.
    virtual bool is_marker_command(char letter, uint32_t number) const;

    // Publish a caller-owned delta. The event and its temporary item storage
    // need only remain alive for this synchronous call.
    void publish_items(const plugin_run_context &run_context,
                       const run_ctx_gcode_reader &context,
                       GCodeStateTracker &state,
                       raw_gcode_reader_event_kind kind,
                       raw_gcode_reader_source source,
                       const std::vector<raw_gcode_reader_event_item> &items) const;
    // Publish a recognized command which changes no portable state.
    void publish_state_update(const plugin_run_context &run_context,
                              const run_ctx_gcode_reader &context,
                              GCodeStateTracker &state,
                              raw_gcode_reader_source source) const;
    // Discard every currently known portable value after an unsupported
    // command, while preserving the session's line cursor.
    void publish_unknown(const plugin_run_context &run_context,
                         const run_ctx_gcode_reader &context,
                         GCodeStateTracker &state,
                         raw_gcode_reader_source source) const;

    // Shared parsing helpers are protected so a dialect can use exactly the
    // same strict command and parameter rules as the common parser.
    static uint32_t probe_marker(std::string_view prefix, std::string_view marker);
    static std::optional<std::string_view> named_parameter(std::string_view line,
                                                           std::string_view name);
    static uint32_t active_tool(const GCodeStateTracker &state);

private:
    const char *const *dependencies_impl() const noexcept override;
    int32_t priority_impl() const noexcept override;
    // Reassemble physical lines from arbitrary chunks and dispatch each
    // complete line in source order. All parser state remains local to this run.
    void read_impl(const plugin_run_context &run_context,
                   const run_ctx_gcode_reader &context,
                   GCodeStateTracker &state) const override;
    // Parse one physical line. begin/end are absolute half-open offsets which
    // exclude CR/LF terminators but include the command's original bytes.
    void consume_line(const plugin_run_context &run_context,
                      const run_ctx_gcode_reader &context,
                      GCodeStateTracker &state,
                      std::string_view text,
                      uint64_t begin,
                      uint64_t end) const;
};

}}} // namespace slic3r_api::GCodeGeneration::Reader

#endif // slic3r_Plugins_GCode_Reader_BuiltinGCodeReaderBase_hpp_
