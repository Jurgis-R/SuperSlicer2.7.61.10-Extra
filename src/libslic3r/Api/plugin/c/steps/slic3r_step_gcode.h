///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_step_gcode_h_
#define slic3r_step_gcode_h_

#define SLIC3R_PLUGIN_API_STEP_GCODE_MAJOR 6u
#define SLIC3R_PLUGIN_API_STEP_GCODE_MINOR 0u

#include "slic3r_step_common.h"
#include "../slic3r_gcode_firmware.h"
#include "../slic3r_printing_plan.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
Payload for STEP_GCODE.

STEP_GCODE plugins are responsible for turning the already ordered
PrintingPlan into an output file. The host computes the final path from the
print settings before the plugin runs, then passes it here as a borrowed UTF-8
string. The string is valid only during the current plugin run.

The final plan is read-only at this boundary. All ordering and plan editing
finishes before the writer runs. The source Print remains the context for
settings, statistics and status; plugins should not mutate unrelated Print state
directly. The firmware instance is borrowed for this run. Its returned text must
be consumed before invoking another firmware callback.

The output artifact and the text consumed by GCodeReader may differ. For
example, a writer may send a binary format or a pipe while keeping a text file
for the viewer. After that text file is complete and readable, call
publish_reader_source_path() exactly once. The host copies the path during the
callback but does not own or remove the file. The producer must keep it
available while the resulting G-code preview is in use.
*/
typedef int32_t (*gcode_publish_reader_source_path_fn)(void *context,
                                                        const char *path);

typedef struct run_ctx_generate_gcode {
    const print_handle *print;
    const printing_plan_handle *plan;
    const raw_gcode_firmware_instance *firmware;
    const char *output_path;
    void *reader_source_context;
    gcode_publish_reader_source_path_fn publish_reader_source_path;
} run_ctx_generate_gcode;

static inline const run_ctx_generate_gcode *
plugin_ctx_as_generate_gcode(const plugin_run_context *ctx)
{
    if (!ctx || ctx->step != STEP_GCODE)
        return NULL;
    return (const run_ctx_generate_gcode *)ctx->data;
}

#ifdef __cplusplus
}
#endif

#endif // slic3r_step_gcode_h_
