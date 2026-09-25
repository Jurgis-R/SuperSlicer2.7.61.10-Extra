///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_step_finalize_slicing_h_
#define slic3r_step_finalize_slicing_h_

#define SLIC3R_PLUGIN_API_STEP_FINALIZE_SLICING_MAJOR 1u
#define SLIC3R_PLUGIN_API_STEP_FINALIZE_SLICING_MINOR 0u

#include "slic3r_step_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
Payload for STEP_FINALIZE_SLICING.

Last step that may edit the Print data tree (objects, layers, regions and
islands). It runs after skirt/brim generation and before STEP_ORDERING builds
the PrintingPlan. Once it completes, the data tree is frozen: later steps only
read it and write their results into the PrintingPlan.
*/
typedef struct run_ctx_finalize_slicing {
    const print_handle *print;
    const object_handle *object;
} run_ctx_finalize_slicing;

static inline const run_ctx_finalize_slicing *
plugin_ctx_as_finalize_slicing(const plugin_run_context *ctx)
{
    if (!ctx || ctx->step != STEP_FINALIZE_SLICING)
        return NULL;
    return (const run_ctx_finalize_slicing *)ctx->data;
}

#ifdef __cplusplus
}
#endif

#endif // slic3r_step_finalize_slicing_h_
