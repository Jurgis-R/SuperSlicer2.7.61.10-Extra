///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_step_seam_placer_h_
#define slic3r_step_seam_placer_h_

#define SLIC3R_PLUGIN_API_STEP_SEAM_PLACER_MAJOR 2u
#define SLIC3R_PLUGIN_API_STEP_SEAM_PLACER_MINOR 0u

#include <stdint.h>

#include "../slic3r_extrusion_entity.h"
#include "../slic3r_printing_plan.h"
#include "slic3r_step_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
Seam placer service contract
============================

A SEAM_PLACER call follows one ownership-preserving sequence:

1. The consumer puts a borrowed PrintingPlan in run_ctx_seam_placer.
2. The orchestrator executes the selected provider to create a session for that
   plan.
3. The provider initializes that session once and publishes it in instance.
4. The consumer calls place_seam for loops from that plan, then calls destroy
   exactly once when the plan no longer needs the service.

Publishing instance transfers ownership of session to the consumer. It does
not transfer ownership of the plugin, plan, loops, or layer regions.

The session callback may be called concurrently. Result storage remains owned
by the caller; only error_message is borrowed until the next callback on the
same thread or until the session is destroyed. Providers must not allow C++
exceptions to cross this interface.
*/

typedef enum raw_seam_placer_status {
    /* The callback did not publish a result. */
    RAW_SEAM_PLACER_STATUS_UNSET = 0,
    /* point contains a valid proposed seam. */
    RAW_SEAM_PLACER_STATUS_SUCCESS,
    /* The instance, loop, layer region, or output structure was invalid. */
    RAW_SEAM_PLACER_STATUS_INVALID_ARGUMENT,
    /* The provider failed; error_message may describe the failure. */
    RAW_SEAM_PLACER_STATUS_ERROR
} raw_seam_placer_status;

/*
Output storage supplied by the caller for one place_seam invocation.

The caller zero-initializes the structure and sets struct_size. A provider that
can write the complete declared structure resets the remaining fields, then
publishes one terminal status. point is meaningful only for SUCCESS and
error_message is optional for ERROR.
*/
typedef struct raw_seam_placer_result {
    /* Set to sizeof(raw_seam_placer_result) by the caller. */
    uint32_t struct_size;
    raw_seam_placer_status status;
    c_point point;
    /* Borrowed diagnostic; see the lifetime rule in the file introduction. */
    const char *error_message;
} raw_seam_placer_result;

/* Release one published session; the consumer calls this exactly once. */
typedef void (*seam_placer_destroy_fn)(void *session);

/*
Ask one initialized session to choose a point on loop for layer_region.

loop is geometry from the initialized plan. layer_region is the source region
chosen for the surrounding PrintingExtrusion and supplies its effective
per-region configuration. Both handles remain host-owned and are valid for the
duration of the callback; the provider must neither free nor retain them past
their owner lifetimes. start_position is the current tool position in scaled
platter coordinates and may be ignored by a position-independent policy.
*/
typedef void (*seam_placer_place_fn)(
    const void *session,
    const extrusion_entity_handle *loop,
    const layer_region_handle *layer_region,
    c_point start_position,
    raw_seam_placer_result *result);

typedef struct raw_seam_placer_vtable {
    /* Set to sizeof(raw_seam_placer_vtable) by the provider. */
    uint32_t struct_size;
    seam_placer_destroy_fn destroy;
    seam_placer_place_fn place_seam;
} raw_seam_placer_vtable;

/* Move-only service value published by a provider and consumed by one owner. */
typedef struct raw_seam_placer_instance {
    /* Set to sizeof(raw_seam_placer_instance) by the provider. */
    uint32_t struct_size;
    /* Provider-owned state transferred to the consumer. */
    void *session;
    /* Static callback table that must outlive session. */
    const raw_seam_placer_vtable *vtable;
} raw_seam_placer_instance;

/*
Single payload used to create a seam placer session.

plan is borrowed and remains owned by the caller. A successful provider writes
a complete instance into an initially zeroed instance during run(). Ownership
then transfers to the consumer, which must clear or otherwise invalidate the
source value and eventually invoke instance.vtable->destroy(instance.session).
*/
typedef struct run_ctx_seam_placer {
    const printing_plan_handle *plan;
    raw_seam_placer_instance instance;
} run_ctx_seam_placer;

/*
Return the borrowed service payload only while a SEAM_PLACER plugin is running.

The returned pointer aliases plugin_run_context::data. It must not be freed or
retained after the run callback and is NULL for every other step.
*/
static inline run_ctx_seam_placer *
plugin_ctx_as_seam_placer(const plugin_run_context *ctx)
{
    if (ctx == NULL || ctx->step != SEAM_PLACER)
        return NULL;
    return (run_ctx_seam_placer *)ctx->data;
}

#ifdef __cplusplus
}
#endif

#endif // slic3r_step_seam_placer_h_
