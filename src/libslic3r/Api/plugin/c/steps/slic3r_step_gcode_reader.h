///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_step_gcode_reader_h_
#define slic3r_step_gcode_reader_h_

#define SLIC3R_PLUGIN_API_STEP_GCODE_READER_MAJOR 1u
#define SLIC3R_PLUGIN_API_STEP_GCODE_READER_MINOR 0u

#include "slic3r_step_common.h"
#include "../slic3r_gcode_reader.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
Synchronous payload for a GCODE_READER service plugin.

The host owns every pointer in this structure for the complete run() call. The
provider repeatedly calls get_next_chunk() until it receives the null end
marker, emits zero or more action events, then emits exactly one final state.
The provider must not retain the context, callbacks, chunks or states after
run() returns.

The generic plugin_run_context remains responsible for cancellation, errors
and progress. This typed payload contains only Reader-specific data. A C++
provider normally derives from GCodeReaderPluginBase instead of invoking these
callbacks directly.
*/
typedef struct run_ctx_gcode_reader {
    const raw_gcode_reader_state *initial_state;
    void *callback_context;
    gcode_reader_get_next_chunk_fn get_next_chunk;
    gcode_reader_emit_event_fn emit_event;
    gcode_reader_emit_final_state_fn emit_final_state;
} run_ctx_gcode_reader;

static inline run_ctx_gcode_reader *
plugin_ctx_as_gcode_reader(const plugin_run_context *ctx)
{
    /* Reject a payload from another service before interpreting ctx->data.
       The returned pointer remains owned by the host and is valid only during
       the current plugin callback. */
    if (!ctx || ctx->step != GCODE_READER)
        return NULL;
    return (run_ctx_gcode_reader *)ctx->data;
}

#ifdef __cplusplus
}
#endif

#endif // slic3r_step_gcode_reader_h_
