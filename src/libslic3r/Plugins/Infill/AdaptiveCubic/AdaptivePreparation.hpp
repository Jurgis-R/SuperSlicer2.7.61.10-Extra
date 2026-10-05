///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#ifndef plugins_infill_adaptivecubic_adaptivepreparation_hpp_
#define plugins_infill_adaptivecubic_adaptivepreparation_hpp_

// Setup owns the octree snapshot; API geometry views are borrowed only while mesh and bridge inputs are copied.
#include <memory>

#include "AdaptiveOctree.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_infill.h"

namespace slic3r_api { class Object; class Print; class ExPolygon; }

namespace slic3r_api { namespace Infill { namespace AdaptiveCubic {

struct AdaptiveCache
{
    std::unique_ptr<Octree> octree;
    double line_spacing = 0.;
    CubicMode mode = CubicMode::Adaptive;
};

// Bridge polygons, including holes, refine the octree at their projected lower face.
std::vector<Triangle> triangulate_boundary(const ExPolygon &boundary, double bottom_z);

// Adaptive and Support snapshots never share spacing or octree ownership.
std::unique_ptr<AdaptiveCache> prepare_cache(
    const Print &print, const Object &object,
    CubicMode mode,
    const plugin_run_context *run_ctx);

}}}

#endif
