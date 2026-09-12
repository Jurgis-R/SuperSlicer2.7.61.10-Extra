///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_host_GCodeReaderService_hpp_
#define slic3r_Api_host_GCodeReaderService_hpp_

#include <functional>
#include <string_view>

#include "libslic3r/Api/plugin/c/slic3r_gcode_reader.h"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeStateTracker.hpp"

/*
Host G-code reader service
==========================

This host-only adapter selects one active GCODE_READER provider and executes it
synchronously. The provider pulls bounded chunks from a caller-owned source,
publishes action events and lends one final state to the service callback. The
service copies that state before the callback returns, so no provider-owned
pointer or session survives execute_plugin().

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

// Return the next non-empty borrowed chunk, or an empty view to mark the source
// exhausted. The service never calls the source again after that marker. The
// returned storage must survive until the next call.
using GCodeReaderChunkSource = std::function<std::string_view()>;

// Select the Reader provider from the print configuration, consume the source
// synchronously and return an owned normalized final state. print may be null
// when no printer-specific provider selection is available. The initial state
// and source remain caller-owned; every event, chunk and callback result is
// validated before this function returns.
slic3r_api::GCodeStateTracker read_gcode(
    Orchestrator &orchestrator,
    Print *print,
    const raw_gcode_reader_state &initial_state,
    GCodeReaderChunkSource source);

// Present one in-memory text as a single chunk. The string remains borrowed
// only for this call; large inputs should use the chunk-source overload.
slic3r_api::GCodeStateTracker read_gcode(
    Orchestrator &orchestrator,
    Print *print,
    const raw_gcode_reader_state &initial_state,
    std::string_view text);

} // namespace Slic3r

#endif // slic3r_Api_host_GCodeReaderService_hpp_
