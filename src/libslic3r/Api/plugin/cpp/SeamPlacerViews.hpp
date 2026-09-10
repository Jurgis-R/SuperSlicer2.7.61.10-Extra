///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_plugin_cpp_SeamPlacerViews_hpp_
#define slic3r_Api_plugin_cpp_SeamPlacerViews_hpp_

#include <memory>
#include <vector>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_seam_placer.h"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"

/*
Seam placer C++ service adapter
===============================

A provider derives SeamPlacerSession. During its SEAM_PLACER run(), it calls
make_seam_placer_instance() with the plan from run_ctx_seam_placer and stores
the returned C instance in that context. Initialization therefore finishes
before any seam request can reach the session.

A consumer selects and executes the provider through OrchestratorView, then
constructs SeamPlacer from the published instance. This move-like construction
clears the payload instance. SeamPlacer then owns only the created session; the
orchestrator continues to own the registered plugin and the host continues to
own every geometry and data-tree view passed to the session.
*/

namespace slic3r_api {

class SeamPlacerSession
{
public:
    /* Allow a session implementation to release its provider-owned caches. */
    virtual ~SeamPlacerSession() = default;

    /*
    Prepare every immutable cache needed by seam placement.

    This method is called exactly once by make_seam_placer_instance(), before
    ownership crosses the C ABI. It may perform expensive work and organize
    its own parallelism. The plan remains host-owned and must outlive the
    resulting session; implementations must not free it.
    */
    virtual void initialize(const PrintingPlan &plan,
                            const std::vector<PrintRegion> &print_regions) = 0;

    /*
    Rank one candidate region when a loop belongs to several LayerRegions.

    Larger finite values have precedence. Calls may run concurrently and must
    not mutate the loop, region, or shared session state without synchronization.
    Consumers deliberately skip this callback for a single candidate.
    */
    virtual float get_priority(const ExtrusionEntity &loop,
                               const LayerRegion &layer_region) const = 0;

    /*
    Select one point on loop in layer_region when approaching it from
    start_position.

    Calls may execute concurrently after initialize() returns, so providers
    must not mutate shared state without synchronization. Repeating a call with
    the same loop, region state and position must return the same point for this
    session. loop is plan geometry, while layer_region is source data-tree
    metadata selected for the surrounding PrintingExtrusion. Both views are
    borrowed from the host and must not be freed or retained beyond their owner
    lifetimes.
    */
    virtual c_point place_seam(const ExtrusionEntity &loop,
                               const LayerRegion &layer_region,
                               c_point start_position) const = 0;
};

/*
Initialize a provider session and transfer it to the C service instance.

The unique_ptr is consumed by this call. If initialization throws, it destroys
the session and no instance is published. On success the returned C instance
owns the session until its destroy callback is invoked, normally by SeamPlacer.
*/
raw_seam_placer_instance make_seam_placer_instance(
    std::unique_ptr<SeamPlacerSession> session,
    const PrintingPlan &plan,
    const std::vector<PrintRegion> &print_regions);

class SeamPlacer
{
public:
    /* Construct an invalid wrapper that owns no session. */
    SeamPlacer() = default;

    /*
    Take the instance published in run_ctx_seam_placer.

    The source is cleared after a successful transfer, preventing the context
    and this wrapper from destroying the same session. An incomplete instance
    is rejected without changing the source.
    */
    explicit SeamPlacer(raw_seam_placer_instance &instance);

    /* A session has unique ownership and therefore cannot be copied. */
    SeamPlacer(const SeamPlacer &) = delete;
    SeamPlacer &operator=(const SeamPlacer &) = delete;

    /* Transfer session ownership without invoking the provider destructor. */
    SeamPlacer(SeamPlacer &&other) noexcept;
    SeamPlacer &operator=(SeamPlacer &&other) noexcept;

    /* Destroy the owned provider session, when present. */
    ~SeamPlacer();

    /* Return whether this wrapper owns a complete callable session. */
    bool valid() const noexcept;

    /*
    Ask the provider for a seam and validate its service contract.

    loop must be a non-empty, fixed, continuous closed path and layer_region
    must be the borrowed source region selected for that plan extrusion. The
    returned point must lie on one of the loop's straight or arc segments within
    SCALED_EPSILON. A provider error or invalid point raises an exception; no
    projection is made. The loop's membership in the initialized plan is a
    caller precondition. start_position and the returned point are both in
    scaled platter coordinates.
    */
    c_point place_seam(const ExtrusionEntity &loop,
                       const LayerRegion &layer_region,
                       c_point start_position) const;

    /* Return the provider's finite ranking for one loop/region candidate. */
    float get_priority(const ExtrusionEntity &loop,
                       const LayerRegion &layer_region) const;

private:
    /* Destroy the owned session and restore the invalid state. */
    void reset() noexcept;

    raw_seam_placer_instance m_instance = {};
};

} // namespace slic3r_api

#endif // slic3r_Api_plugin_cpp_SeamPlacerViews_hpp_
