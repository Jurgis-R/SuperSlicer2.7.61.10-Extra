///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_host_GCodeReaderService_hpp_
#define slic3r_Api_host_GCodeReaderService_hpp_

#include <functional>
#include <mutex>
#include <string>
#include <string_view>

#include "libslic3r/Api/plugin/c/slic3r_gcode_reader.h"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeStateTracker.hpp"

/*
Host G-code reader service
==========================

This host-only adapter executes one GCODE_READER provider selected by its
stable id. For imported text, detect_gcode_reader() separately probes every
active provider against the same bounded prefix. No PrintConfig option or
exclusive group participates in either operation.

The selected provider pulls bounded chunks from a caller-owned source,
publishes action events and lends one final state to the service callback. The
service copies that state before the callback returns, so no provider-owned
pointer or session survives execute_plugin().

    firmware/viewer -> explicit id or detect_gcode_reader()
                              |
                              v
    caller -> read_gcode() -> Orchestrator -> selected Reader plugin
       ^             |                              |
       |             +-- ReaderInvocation <--------+
       |                    | chunks, events, final state
       +------ owned GCodeStateTracker

ReaderInvocation is an implementation detail in the .cpp file. It turns the
C++ chunk source into C callbacks, enforces callback order and translates all
callback failures back into one C++ exception from read_gcode().

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
using GCodeReaderChunkSource = std::function<std::string_view()>;

inline constexpr const char *DEFAULT_GCODE_READER_PLUGIN_ID = "gcode.reader.default";
inline constexpr size_t GCODE_READER_PROBE_PREFIX_SIZE = 256u * 1024u;

// Host-owned facade lent to one firmware session. It exposes only synchronous
// execution by stable plugin id; plugin discovery and validation stay inside
// the host rather than crossing the ABI.
class GCodeReaderService
{
public:
    // Borrow the optional Print for the lifetime of this service. It lets a
    // Reader resolve slicer configuration such as Klipper tool names; callers
    // parsing an external file may pass null. The Print must outlive every
    // synchronous read or probe made through this facade.
    GCodeReaderService(Orchestrator &orchestrator, Print *print);
    const raw_gcode_reader_service *c_service() const { return &m_service; }
    slic3r_api::GCodeStateTracker read(std::string_view reader_plugin_id,
                                       const raw_gcode_reader_state &initial_state,
                                       GCodeReaderChunkSource source) const;
    slic3r_api::GCodeStateTracker read(std::string_view reader_plugin_id,
                                       const raw_gcode_reader_state &initial_state,
                                       std::string_view text) const;
    std::string detect(std::string_view prefix) const;
    std::string validated_reader_id(std::string_view reader_plugin_id) const;

private:
    static void read_callback(void *service_context,
                              const char *reader_plugin_id,
                              const raw_gcode_reader_state *initial_state,
                              void *source_context,
                              gcode_reader_get_next_chunk_fn get_next_chunk,
                              void *final_state_context,
                              gcode_reader_emit_final_state_fn receive_final_state,
                              raw_gcode_reader_result *result) noexcept;
    void read_from_callbacks(const char *reader_plugin_id,
                             const raw_gcode_reader_state &initial_state,
                             void *source_context,
                             gcode_reader_get_next_chunk_fn get_next_chunk,
                             void *final_state_context,
                             gcode_reader_emit_final_state_fn receive_final_state);

    Orchestrator &m_orchestrator;
    Print *m_print;
    raw_gcode_reader_service m_service;
    // The result diagnostic is borrowed by C until the next service call.
    // Serializing calls both protects that buffer and makes this lifetime
    // promise unambiguous when a firmware invokes the service concurrently.
    std::mutex m_mutex;
    std::string m_error;
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
