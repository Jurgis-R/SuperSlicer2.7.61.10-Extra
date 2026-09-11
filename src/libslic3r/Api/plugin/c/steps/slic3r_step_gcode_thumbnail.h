///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_step_gcode_thumbnail_h_
#define slic3r_step_gcode_thumbnail_h_

#define SLIC3R_PLUGIN_API_STEP_GCODE_THUMBNAIL_MAJOR 1u
#define SLIC3R_PLUGIN_API_STEP_GCODE_THUMBNAIL_MINOR 0u

#include "slic3r_step_common.h"
#include "../slic3r_gcode_thumbnail.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Creation payload for one GCODE_THUMBNAIL encoder session. The Print and
   renderer are borrowed until the returned instance is destroyed. Ownership
   of the instance moves to the caller when the provider returns successfully.
   The caller must also destroy any published instance on an execution error. */
typedef struct run_ctx_gcode_thumbnail {
    const print_handle *print;
    const raw_gcode_thumbnail_renderer *renderer;
    raw_gcode_thumbnail_encoder_instance instance;
} run_ctx_gcode_thumbnail;

static inline run_ctx_gcode_thumbnail *
plugin_ctx_as_gcode_thumbnail(const plugin_run_context *ctx)
{
    if (!ctx || ctx->step != GCODE_THUMBNAIL)
        return NULL;
    return (run_ctx_gcode_thumbnail *)ctx->data;
}

#ifdef __cplusplus
}
#endif

#endif // slic3r_step_gcode_thumbnail_h_
