///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_step_seam_placer_h_
#define slic3r_step_seam_placer_h_

#define SLIC3R_PLUGIN_API_STEP_SEAM_PLACER_MAJOR 3u
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

1. The consumer puts a borrowed PrintingPlan and the PrintRegions assigned to
   this provider in run_ctx_seam_placer.
2. The orchestrator executes the selected provider to create a session for that
   plan.
3. The provider initializes that session once and publishes it in instance.
4. The consumer calls place_seam for loops from that plan, then calls destroy
   exactly once when the plan no longer needs the service.

Publishing instance transfers ownership of session to the consumer. It does
not transfer ownership of the plugin, plan, loops, or layer regions.

After initialization, get_priority and place_seam may be called concurrently.
Result storage remains owned by the caller; only error_message is borrowed
until the next callback of either kind on the same thread or until the session
is destroyed. Providers must not allow C++ exceptions to cross this interface.
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

/*
Output storage supplied by the caller for one get_priority invocation.

The caller zero-initializes the structure and sets struct_size. priority is
meaningful only for SUCCESS and must be finite. Larger values win; exact ties
are resolved by the consumer's stable LayerRegion order. error_message follows
the same borrowed lifetime as raw_seam_placer_result::error_message.
*/
typedef struct raw_seam_placer_priority_result {
    uint32_t struct_size;
    raw_seam_placer_status status;
    float priority;
    const char *error_message;
} raw_seam_placer_priority_result;

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

/*
Rank one candidate LayerRegion for loop when several regions own its geometry.

The loop and region are borrowed from the initialized plan. The callback must
not mutate either object. It may inspect regional configuration and immutable
session caches. Consumers do not call this callback when only one candidate
region exists.
*/
typedef void (*seam_placer_priority_fn)(
    const void *session,
    const extrusion_entity_handle *loop,
    const layer_region_handle *layer_region,
    raw_seam_placer_priority_result *result);

typedef struct raw_seam_placer_vtable {
    /* Set to sizeof(raw_seam_placer_vtable) by the provider. */
    uint32_t struct_size;
    seam_placer_destroy_fn destroy;
    seam_placer_priority_fn get_priority;
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

plan and regions are borrowed and remain owned by the caller. regions contains
the distinct PrintRegions whose regional selector chose this provider, in
stable first-encounter order. The array is valid only for run(), while the
pointed-to PrintRegions remain valid with the Print. A successful provider
writes a complete instance into an initially zeroed instance during run().
Ownership then transfers to the consumer, which must clear or otherwise
invalidate the source value and eventually invoke destroy exactly once.
*/
typedef struct run_ctx_seam_placer {
    const printing_plan_handle *plan;
    const print_region_handle *const *regions;
    uint32_t region_count;
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
