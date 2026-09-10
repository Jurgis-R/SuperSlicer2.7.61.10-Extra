///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
SeamPlacementConstraints: shared geometry for seam policies
=========================================================
This session-owned helper classifies loop points and portions as Enforced, Free
or Blocked. It does not choose a seam direction or a final candidate: the provider
uses these classifications to implement its own placement policy.

Call initialize(plan, storage) before publishing the seam session. Initialization
copies projected paint and sliced volume polygons into private memory and records
which source layer and instance each plan extrusion belongs to. Storage is borrowed
only for that call; temporary StoredPolygonCollection owners free their allocations,
including during exception unwinding. The helper owns its copied caches through a
unique_ptr and releases them on destruction. It never owns or frees host objects.

The plan, source objects and existing loop handles must outlive the helper. After
initialization, queries are const and may run concurrently provided the borrowed
geometry and configuration are not being modified concurrently. Reordering or
rotating existing loops between queries is supported; new loop identities or changed
source constraints require a new session. Returned Result and Curve objects contain
value snapshots; modifying them does not modify the caches or host extrusions.

XY positions, arc radii and paint margins use scaled coordinate units. Curve
parameters are dimensionless and angles are in radians. Cached zones use centered
object coordinates: object transformation minus center_offset. Query points and
returned curves use the plan's platter frame; the explicit source instance shift
connects the two frames, never the incoming tool position.

Paint is projected through layer slabs with an extrusion-width XY margin; this is
not the legacy 3D triangle-distance test. Only INSIDE volumes enforce seams, without
that margin; attractors are ignored. Blockers override enforcers, including their
boundaries. Candidate coordinates rounded to integers can be checked again with
classify_point() before the provider returns a seam to its consumer.
*/

#ifndef slic3r_SeamPlacementConstraints_hpp_
#define slic3r_SeamPlacementConstraints_hpp_

#include <memory>
#include <vector>
#include "libslic3r/Api/plugin/cpp/SeamPlacerViews.hpp"

namespace slic3r_api { namespace SeamPlacement {


class SeamPlacementConstraints
{
public:
    enum class Status { Blocked, Free, Enforced };
    struct Curve {
        c_extrusion_segment segment{};
        double cx = 0, cy = 0, radius = 0, start = 0, sweep = 0;
        /* Parameters are in [0,1]; returned points lie on the original line/arc,
           rounded to scaled integer XY. No host geometry is changed. */
        c_point point(double t) const;
        double parameter(double angle) const;
    };
    struct Portion {
        Curve curve;
        double begin = 0, end = 1;
        Status status = Status::Free;
        bool include_begin = true, include_end = true;
        double paint_margin = 0;
    };
    struct Result {
        bool relevant = false;
        std::vector<Portion> portions;
    };

    SeamPlacementConstraints();
    ~SeamPlacementConstraints();
    SeamPlacementConstraints(const SeamPlacementConstraints &) = delete;
    SeamPlacementConstraints &operator=(const SeamPlacementConstraints &) = delete;
    /* Called once, before publication. storage is borrowed only for this call. */
    void initialize(const PrintingPlan &plan, storage_handle *storage);
    /* Classify true segments in tree order, inheriting extrusion width/role.
       Blockers override enforcers; boundaries belong to the closed zone. */
    Result portions(const ExtrusionEntity &loop, const LayerRegion &region) const;
    /* Recheck integer-rounded candidates against the same closed zones. */
    Status classify_point(const ExtrusionEntity &loop, c_point point, double paint_margin) const;
private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

}} // namespace slic3r_api::SeamPlacement
#endif
