///|/ Copyright (c) 2026 Jurgis-R
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
#ifndef slic3r_HybridNonplanarExtrusion_hpp_
#define slic3r_HybridNonplanarExtrusion_hpp_

#include "ExtrusionEntity.hpp"
#include "HybridNonplanarSurfacePlanner.hpp"

#include <cmath>
#include <limits>
#include <vector>

namespace Slic3r {

// Convert planner coordinates (millimetres, absolute Z) to SuperSlicer's
// scaled XY coordinates plus Z offsets relative to the G-code writer's nominal
// Z at the start of this pass. base_z_mm MUST be that nominal writer Z, not
// necessarily the ideal H boundary if the final planar layer crossed it.
//
// This is deliberately only a conversion step: callers must not invoke it on
// unvalidated candidates or emit its result until a real swept-tool collision
// validator has approved the complete path and its approach/departure moves.
inline bool make_hybrid_surface_extrusion_paths(
    const std::vector<HybridSurfacePath> &candidates,
    double base_z_mm,
    const ExtrusionAttributes &attributes,
    std::vector<ExtrusionPath3D> &output)
{
    output.clear();
    if (candidates.empty() || !std::isfinite(base_z_mm) ||
        !std::isfinite(attributes.mm3_per_mm) || attributes.mm3_per_mm <= 0.0 ||
        !std::isfinite(attributes.width) || attributes.width <= 0.f ||
        !std::isfinite(attributes.height) || attributes.height <= 0.f)
        return false;

    const double max_scaled_mm =
        static_cast<double>(std::numeric_limits<coord_t>::max()) * SCALING_FACTOR;
    std::vector<ExtrusionPath3D> converted;
    converted.reserve(candidates.size());

    for (const HybridSurfacePath &candidate : candidates) {
        if (candidate.points.size() < 2)
            return false;

        ExtrusionPath3D path(attributes, false);
        Vec3d previous_scaled;
        bool have_previous = false;
        for (const Vec3d &point : candidate.points) {
            if (!point.allFinite())
                return false;
            const double z_offset_mm = point.z() - base_z_mm;
            if (std::abs(point.x()) > max_scaled_mm ||
                std::abs(point.y()) > max_scaled_mm ||
                std::abs(z_offset_mm) > max_scaled_mm)
                return false;

            const Point xy = Point::new_scale(point.x(), point.y());
            const coord_t z_offset = scale_t(z_offset_mm);
            const Vec3d scaled_point(static_cast<double>(xy.x()),
                                     static_cast<double>(xy.y()),
                                     static_cast<double>(z_offset));
            if (have_previous && (scaled_point - previous_scaled).squaredNorm() <= 0.0)
                return false;

            path.push_back(xy, z_offset);
            previous_scaled = scaled_point;
            have_previous = true;
        }

        if (path.polyline.size() < 2 || path.z_offsets.size() != path.polyline.size())
            return false;
        converted.emplace_back(std::move(path));
    }

    output.swap(converted);
    return true;
}


 // Run candidate generation, mandatory segment validation, and coordinate/extrusion
 // conversion as one fail-closed operation. The supplied validator must check the
 // actual swept nozzle/tool envelope against relevant model geometry; a permissive
 // placeholder callback is not a collision validator. Approach and departure
 // moves must also be validated by the caller before G-code emission.
inline bool plan_hybrid_surface_extrusion_paths(
    const indexed_triangle_set &mesh,
    const HybridSurfacePlannerOptions &options,
    double base_z_mm,
    const ExtrusionAttributes &attributes,
    const std::function<bool(const Vec3d &, const Vec3d &)> &swept_tool_segment_is_clear,
    std::vector<ExtrusionPath3D> &output)
{
    output.clear();
    std::vector<HybridSurfacePath> candidates;
    if (!plan_hybrid_surface_paths(mesh, options, swept_tool_segment_is_clear, candidates))
        return false;
    return make_hybrid_surface_extrusion_paths(candidates, base_z_mm, attributes, output);
}

} // namespace Slic3r

#endif // slic3r_HybridNonplanarExtrusion_hpp_
