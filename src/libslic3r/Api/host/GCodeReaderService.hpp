///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_host_GCodeReaderService_hpp_
#define slic3r_Api_host_GCodeReaderService_hpp_

#include <string>
#include <string_view>

#include "libslic3r/Api/plugin/cpp/gcode/GCodeReaderService.hpp"

/*
Host G-code reader service
==========================

This host-only adapter executes one GCODE_READER provider selected by its
stable id. For imported text, detect_gcode_reader() separately probes every
active provider against the same bounded prefix. No PrintConfig option or
exclusive group participates in either operation.

The plugin-side slic3r_api::GCodeReaderService owns the common execution and
validation path. This host facade adds only automatic detection, which needs
access to all active Reader providers and their priorities.

    firmware/viewer -> explicit id or detect_gcode_reader()
                              |
                              v
    host caller -> GCodeReaderService -> plugin-side Reader helper
                         |                         |
                         +-- detect()              +-- exact Reader execution

A chunk source returns a non-empty view while input remains and an empty view
at end-of-stream. Its bytes must remain valid until the source is called again.
The string overload below is a convenience for small generated scripts; file
and viewer callers can instead supply a fixed-size reusable input buffer.
*/

namespace Slic3r {

class Orchestrator;
class Print;
class Plugin;

// Return the next non-empty borrowed chunk, or an empty view to mark the source
// exhausted. The service never calls the source again after that marker. The
// returned storage must survive until the next call.
using GCodeReaderChunkSource = slic3r_api::GCodeReaderChunkSource;

inline constexpr const char *DEFAULT_GCODE_READER_PLUGIN_ID =
    slic3r_api::DEFAULT_GCODE_READER_PLUGIN_ID;
inline constexpr size_t GCODE_READER_PROBE_PREFIX_SIZE = 256u * 1024u;

// Host facade for Reader execution and automatic dialect detection. Exact
// execution delegates to the same helper available to C++ plugins.
class GCodeReaderService
{
public:
    // Borrow the optional Print for the lifetime of this service. It lets a
    // Reader resolve slicer configuration such as Klipper tool names; callers
    // parsing an external file may pass null. The Print must outlive every
    // synchronous read or probe made through this facade.
    GCodeReaderService(Orchestrator &orchestrator, Print *print);
    slic3r_api::GCodeStateTracker read(std::string_view reader_plugin_id,
                                       const raw_gcode_reader_state &initial_state,
                                       GCodeReaderChunkSource source) const;
    slic3r_api::GCodeStateTracker read(std::string_view reader_plugin_id,
                                       const raw_gcode_reader_state &initial_state,
                                       std::string_view text) const;
    std::string detect(std::string_view prefix) const;
    std::string validated_reader_id(std::string_view reader_plugin_id) const;

private:
    Orchestrator &m_orchestrator;
    Print *m_print;
    slic3r_api::GCodeReaderService m_reader;
};

// Resolve exactly reader_plugin_id, consume the source synchronously and return
// an owned normalized final state. An empty id explicitly requests the default
// Reader. print may be null when the Reader is used outside slicing. The
// initial state and source remain caller-owned; every event, chunk and callback
// result is validated before this function returns.
slic3r_api::GCodeStateTracker read_gcode(
    Orchestrator &orchestrator,
    Print *print,
    std::string_view reader_plugin_id,
    const raw_gcode_reader_state &initial_state,
    GCodeReaderChunkSource source);

// Present one in-memory text as a single chunk. The string remains borrowed
// only for this call; large inputs should use the chunk-source overload.
slic3r_api::GCodeStateTracker read_gcode(
    Orchestrator &orchestrator,
    Print *print,
    std::string_view reader_plugin_id,
    const raw_gcode_reader_state &initial_state,
    std::string_view text);

// Probe every active Reader against the same bounded prefix and return the
// winning stable id. Probe failures do not hide successful candidates.
std::string detect_gcode_reader(Orchestrator &orchestrator, Print *print,
                                std::string_view prefix);

} // namespace Slic3r

#endif // slic3r_Api_host_GCodeReaderService_hpp_
