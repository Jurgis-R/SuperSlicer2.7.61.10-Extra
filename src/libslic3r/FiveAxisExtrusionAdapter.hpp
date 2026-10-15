/// Copyright (c) 2026 Jurgis-R
/// SuperSlicer is released under the terms of the AGPLv3 or higher.
#ifndef slic3r_FiveAxisExtrusionAdapter_hpp_
#define slic3r_FiveAxisExtrusionAdapter_hpp_

#include "ExtrusionEntity.hpp"
#include "FiveAxisMotionPlanner.hpp"

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace Slic3r {

// Convert an existing SuperSlicer 3D extrusion path into planner poses.
// This deliberately requires explicit per-vertex tool-axis directions: it
// never guesses a five-axis orientation from an XYZ path or layer normal.
// Coordinates in ExtrusionPath3D are scaled millimetres; base_z_mm is the
// nominal layer Z in millimetres. E is emitted as a cumulative absolute value
// beginning at initial_e, using the caller's already-computed E/mm factor.
inline FiveAxisValidationResult make_five_axis_poses_from_extrusion_path(
    const ExtrusionPath3D &path,
    double base_z_mm,
    double initial_e,
    double e_per_mm,
    double feed_mm_min,
    std::vector<FiveAxisToolPose> &poses)
{
    poses.clear();
    const std::size_t count = path.polyline.size();
    if (count < 2)
        return { false, "A five-axis extrusion path requires at least two vertices." };
    if (path.polyline.has_arc())
        return { false, "Five-axis extrusion adapter requires linearized path segments." };
    if (!path.has_tool_axis_normals())
        return { false, "Extrusion path has no complete per-vertex tool-axis directions." };
    if (!std::isfinite(base_z_mm) || !std::isfinite(initial_e) ||
        !std::isfinite(e_per_mm) || e_per_mm < 0.0 ||
        !std::isfinite(feed_mm_min) || feed_mm_min <= 0.0)
        return { false, "Five-axis extrusion conversion received invalid Z, E, or feed settings." };

    std::vector<FiveAxisToolPose> candidate;
    candidate.reserve(count);
    double cumulative_e = initial_e;
    FiveAxisVec3 previous_point {};
    for (std::size_t i = 0; i < count; ++i) {
        const Point &xy = path.polyline.get_point(i);
        const FiveAxisVec3 point {
            double(xy.x()) * SCALING_FACTOR,
            double(xy.y()) * SCALING_FACTOR,
            base_z_mm + (i < path.z_offsets.size()
                ? double(path.z_offsets[i]) * SCALING_FACTOR : 0.0)
        };
        const auto &raw_normal = path.tool_axis_normals[i];
        FiveAxisVec3 normal { raw_normal[0], raw_normal[1], raw_normal[2] };
        if (!five_axis_finite(point) || !five_axis_normalize(normal))
            return { false, "Five-axis extrusion path contains an invalid point or zero/non-finite tool direction." };

        if (i > 0) {
            const FiveAxisVec3 delta = point - previous_point;
            const double segment_length = five_axis_norm(delta);
            if (!std::isfinite(segment_length))
                return { false, "Five-axis extrusion segment length is non-finite." };
            cumulative_e += segment_length * e_per_mm;
        }
        candidate.push_back({ point, normal, cumulative_e, feed_mm_min });
        previous_point = point;
    }

    poses.swap(candidate);
    return { true, {} };
}

} // namespace Slic3r

#endif // slic3r_FiveAxisExtrusionAdapter_hpp_
