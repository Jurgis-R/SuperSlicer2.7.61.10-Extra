///|/ Copyright (c) 2026 Jurgis-R
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
#ifndef slic3r_HybridNonplanarExtrusion_hpp_
#define slic3r_HybridNonplanarExtrusion_hpp_

#include "ExtrusionEntity.hpp"
#include "HybridNonplanarSurfacePlanner.hpp"

#include <algorithm>

#include <cmath>
#include <functional>
#include <limits>
#include <utility>
#include <vector>

namespace Slic3r {

// Convert absolute print-space XYZ (millimetres) to the writer's coordinate
// frame. XY includes the same origin and active-tool offset as point_to_gcode();
// Z is absolute print Z because the writer applies its configured Z offset when
// formatting G-code. This is a coordinate conversion only, not a lift-state fix.
inline bool hybrid_print_to_writer_xyz(
    const Vec3d &print_point,
    const Vec2d &origin,
    const Vec2d &tool_offset,
    Vec3d &writer_point)
{
    if (!print_point.allFinite() || !origin.allFinite() || !tool_offset.allFinite())
        return false;
    const Vec3d converted(print_point.x() + origin.x() - tool_offset.x(),
                          print_point.y() + origin.y() - tool_offset.y(),
                          print_point.z());
    if (!converted.allFinite())
        return false;
    writer_point = converted;
    return true;
}

// Inverse of hybrid_print_to_writer_xyz(), used to interpret the live writer
// position in the same print-space frame as collision-preflight routes.
inline bool hybrid_writer_to_print_xyz(
    const Vec3d &writer_point,
    const Vec2d &origin,
    const Vec2d &tool_offset,
    Vec3d &print_point)
{
    if (!writer_point.allFinite() || !origin.allFinite() || !tool_offset.allFinite())
        return false;
    const Vec3d converted(writer_point.x() - origin.x() + tool_offset.x(),
                          writer_point.y() - origin.y() + tool_offset.y(),
                          writer_point.z());
    if (!converted.allFinite())
        return false;
    print_point = converted;
    return true;
}

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


 // Convenience entry point for the real object-mesh path: apply the
 // PrintObject/model transform first, then run candidate generation, required
 // swept-segment validation, and ExtrusionPath3D conversion. The caller still
 // must validate the non-extruding approach/departure moves before emission.
inline bool plan_hybrid_surface_extrusion_paths(
    const indexed_triangle_set &source_mesh,
    const Transform3d &mesh_to_print,
    const HybridSurfacePlannerOptions &options,
    double base_z_mm,
    const ExtrusionAttributes &attributes,
    const std::function<bool(const Vec3d &, const Vec3d &)> &swept_tool_segment_is_clear,
    std::vector<ExtrusionPath3D> &output)
{
    output.clear();
    indexed_triangle_set print_mesh;
    if (!transform_hybrid_surface_mesh(source_mesh, mesh_to_print, print_mesh))
        return false;
    return plan_hybrid_surface_extrusion_paths(
        print_mesh, options, base_z_mm, attributes, swept_tool_segment_is_clear, output);
}


 
// A non-planar approach is an explicit XYZ polyline from the writer's live
// position, through a configured high-clearance waypoint, to the first
// extrusion vertex. Coordinates are absolute millimetres in print space.
// This helper validates the route; it does not itself emit G-code.
struct HybridNonplanarApproach
{
    std::vector<Vec3d> points;
    Vec3d expected_start { Vec3d::Zero() };
    Vec3d extrusion_start { Vec3d::Zero() };
    double clearance_z_mm { 0.0 };
};

inline bool validate_hybrid_nonplanar_approach(
    const HybridNonplanarApproach &approach,
    const Vec3d &tracked_writer_position,
    const Vec3d &path_start,
    const std::function<bool(const Vec3d &, const Vec3d &)> &swept_tool_segment_is_clear,
    double position_tolerance_mm = 1e-4)
{
    if (!swept_tool_segment_is_clear || approach.points.size() < 3 ||
        !tracked_writer_position.allFinite() || !path_start.allFinite() ||
        !approach.expected_start.allFinite() || !approach.extrusion_start.allFinite() ||
        !std::isfinite(approach.clearance_z_mm) ||
        !std::isfinite(position_tolerance_mm) || position_tolerance_mm < 0.0)
        return false;

    const double tolerance_sq = position_tolerance_mm * position_tolerance_mm;
    if ((approach.expected_start - tracked_writer_position).squaredNorm() > tolerance_sq ||
        (approach.extrusion_start - path_start).squaredNorm() > tolerance_sq ||
        (approach.points.front() - tracked_writer_position).squaredNorm() > tolerance_sq ||
        (approach.points.back() - path_start).squaredNorm() > tolerance_sq)
        return false;

    double max_z = -std::numeric_limits<double>::infinity();
    for (const Vec3d &point : approach.points) {
        if (!point.allFinite())
            return false;
        max_z = std::max(max_z, point.z());
    }
    if (max_z + position_tolerance_mm < approach.clearance_z_mm)
        return false;

    // Validate the exact segments the emitter must write. Coincident
    // waypoints are omitted by the emitter and therefore need no sweep check.
    for (size_t i = 1; i < approach.points.size(); ++i) {
        if ((approach.points[i] - approach.points[i - 1]).squaredNorm() > 0.0 &&
            !swept_tool_segment_is_clear(approach.points[i - 1], approach.points[i]))
            return false;
    }
    return true;
}


// Explicit writer-state contract for an experimental surface-route lifecycle.
// Positions are absolute writer-frame XYZ in millimetres; lift is the writer's
// tracked lift amount, not an inferred Z difference. This describes state only;
// it does not move the writer or emit G-code.
struct HybridPreExtrusionTravelContract
{
    Vec3d expected_start_position { Vec3d::Zero() };
    double expected_start_lift_mm { 0.0 };
    Vec3d expected_end_position { Vec3d::Zero() };
    double expected_end_lift_mm { 0.0 };
    Vec3d expected_return_position { Vec3d::Zero() };
    double expected_return_lift_mm { 0.0 };
    double position_tolerance_mm { 1e-4 };
    double lift_tolerance_mm { 1e-4 };
};

// Check either side of the pre-extrusion contract against live writer state. The
// extrusion end is explicit because a surface-following route need not end at the
// nominal planar layer Z. Callers must abort export on failure after any route move.
inline bool validate_hybrid_pre_extrusion_state(
    const HybridPreExtrusionTravelContract &contract,
    const Vec3d &live_writer_position,
    double live_lift_mm,
    bool validate_end_state)
{
    if (!contract.expected_start_position.allFinite() ||
        !contract.expected_end_position.allFinite() ||
        !live_writer_position.allFinite() ||
        !std::isfinite(contract.expected_start_lift_mm) ||
        !std::isfinite(contract.expected_end_lift_mm) ||
        !std::isfinite(live_lift_mm) ||
        !std::isfinite(contract.position_tolerance_mm) ||
        contract.position_tolerance_mm < 0.0 ||
        !std::isfinite(contract.lift_tolerance_mm) ||
        contract.lift_tolerance_mm < 0.0)
        return false;

    const Vec3d &expected_position = validate_end_state ?
        contract.expected_end_position : contract.expected_start_position;
    const double expected_lift = validate_end_state ?
        contract.expected_end_lift_mm : contract.expected_start_lift_mm;
    return (live_writer_position - expected_position).norm() <=
               contract.position_tolerance_mm &&
           std::abs(live_lift_mm - expected_lift) <= contract.lift_tolerance_mm;
}

// Validate the post-route state separately from the extrusion-start state. The
// route emitter normalizes lift to zero during travel, so callers restore the saved
// lift only after the physical return coordinate has been reached.
inline bool validate_hybrid_return_state(
    const HybridPreExtrusionTravelContract &contract,
    const Vec3d &live_writer_position,
    double live_lift_mm)
{
    return contract.expected_return_position.allFinite() &&
           live_writer_position.allFinite() &&
           std::isfinite(contract.expected_return_lift_mm) &&
           std::isfinite(live_lift_mm) &&
           std::isfinite(contract.position_tolerance_mm) &&
           contract.position_tolerance_mm >= 0.0 &&
           std::isfinite(contract.lift_tolerance_mm) &&
           contract.lift_tolerance_mm >= 0.0 &&
           (live_writer_position - contract.expected_return_position).norm() <=
               contract.position_tolerance_mm &&
           std::abs(live_lift_mm - contract.expected_return_lift_mm) <=
               contract.lift_tolerance_mm;
}

// Complete, pre-emission route contract. The extrusion points are the exact
// absolute XYZ vertices that will be emitted; departure and return are explicit
// travel polylines. This validates geometry only and never mutates GCodeWriter.
struct HybridNonplanarPassRoute
{
    HybridNonplanarApproach approach;
    std::vector<Vec3d> extrusion_points;
    std::vector<Vec3d> departure_points;
    std::vector<Vec3d> return_points;
};

// Transform every absolute XYZ vertex of a validated print-space route into
// writer coordinates before passing it to GCodeWriter. Keep collision checks in
// print space; this conversion must not be mistaken for a clearance validation.
inline bool hybrid_print_to_writer_route(
    const HybridNonplanarPassRoute &print_route,
    const Vec2d &origin,
    const Vec2d &tool_offset,
    HybridNonplanarPassRoute &writer_route)
{
    HybridNonplanarPassRoute converted = print_route;
    auto convert_points = [&](std::vector<Vec3d> &points) {
        for (Vec3d &point : points) {
            Vec3d writer_point = Vec3d::Zero();
            if (!hybrid_print_to_writer_xyz(point, origin, tool_offset, writer_point))
                return false;
            point = writer_point;
        }
        return true;
    };
    if (!hybrid_print_to_writer_xyz(print_route.approach.expected_start, origin, tool_offset,
                                    converted.approach.expected_start) ||
        !hybrid_print_to_writer_xyz(print_route.approach.extrusion_start, origin, tool_offset,
                                    converted.approach.extrusion_start) ||
        !convert_points(converted.approach.points) ||
        !convert_points(converted.extrusion_points) ||
        !convert_points(converted.departure_points) ||
        !convert_points(converted.return_points))
        return false;
    writer_route = std::move(converted);
    return true;
}

inline bool validate_hybrid_nonplanar_pass_route(
    const HybridNonplanarPassRoute &route,
    const Vec3d &tracked_writer_position,
    const std::function<bool(const Vec3d &, const Vec3d &)> &swept_tool_segment_is_clear,
    double position_tolerance_mm = 1e-4)
{
    if (!swept_tool_segment_is_clear || route.extrusion_points.size() < 2 ||
        route.departure_points.size() < 2 || route.return_points.size() < 2 ||
        !std::isfinite(position_tolerance_mm) || position_tolerance_mm < 0.0)
        return false;

    const double tolerance_sq = position_tolerance_mm * position_tolerance_mm;
    auto coincides = [tolerance_sq](const Vec3d &a, const Vec3d &b) {
        return a.allFinite() && b.allFinite() && (a - b).squaredNorm() <= tolerance_sq;
    };
    if (!coincides(route.approach.extrusion_start, route.extrusion_points.front()) ||
        !coincides(route.departure_points.front(), route.extrusion_points.back()) ||
        !coincides(route.return_points.front(), route.departure_points.back()))
        return false;

    if (!validate_hybrid_nonplanar_approach(
            route.approach, tracked_writer_position, route.extrusion_points.front(),
            swept_tool_segment_is_clear, position_tolerance_mm))
        return false;

    auto validate_segments = [&swept_tool_segment_is_clear](
        const std::vector<Vec3d> &points) {
        for (const Vec3d &point : points)
            if (!point.allFinite())
                return false;
        for (size_t i = 1; i < points.size(); ++i) {
            if ((points[i] - points[i - 1]).squaredNorm() > 0.0 &&
                !swept_tool_segment_is_clear(points[i - 1], points[i]))
                return false;
        }
        return true;
    };
    return validate_segments(route.extrusion_points) &&
           validate_segments(route.departure_points) &&
           validate_segments(route.return_points);
}

} // namespace Slic3r

#endif // slic3r_HybridNonplanarExtrusion_hpp_
