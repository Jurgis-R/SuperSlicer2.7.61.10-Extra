///|/ Copyright (c) 2026 Jurgis-R
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
#ifndef slic3r_HybridNonplanarSurfacePlanner_hpp_
#define slic3r_HybridNonplanarSurfacePlanner_hpp_

#include "TriangleMesh.hpp"
#include "HybridNonplanarSectionPlanner.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <vector>

namespace Slic3r {

struct HybridSurfacePath {
    std::vector<Vec3d> points;
};

struct HybridSurfacePlannerOptions {
    double sample_spacing_mm { 0.4 };
    double max_slope_angle_degrees { 35.0 };
    double max_z_offset_mm { 0.8 };
    double max_vertical_rate_mm_s { 5.0 };
    double max_slope_rate_degrees_per_mm { 30.0 };
    double travel_speed_mm_s { 10.0 };
    double section_start_z { 0.0 };
    double section_top_z { 2.0 };
    std::size_t max_grid_samples { 1000000 };
};

// Generates candidate finishing paths directly from upward-facing mesh facets.
// A caller must supply a swept-tool collision validator. Missing validation,
// invalid limits, excessive grid size, or any unsafe segment returns false and
// leaves output empty: candidates are never silently treated as print-safe.
inline bool plan_hybrid_surface_paths(
    const indexed_triangle_set &mesh,
    const HybridSurfacePlannerOptions &options,
    const std::function<bool(const Vec3d &, const Vec3d &)> &swept_tool_segment_is_clear,
    std::vector<HybridSurfacePath> &output)
{
    output.clear();
    if (mesh.indices.empty() || mesh.vertices.empty() ||
        !swept_tool_segment_is_clear ||
        !std::isfinite(options.sample_spacing_mm) || options.sample_spacing_mm <= 0.0 ||
        !std::isfinite(options.max_slope_angle_degrees) ||
        options.max_slope_angle_degrees <= 0.0 || options.max_slope_angle_degrees >= 90.0 ||
        !std::isfinite(options.max_z_offset_mm) || options.max_z_offset_mm <= 0.0 ||
        !std::isfinite(options.max_vertical_rate_mm_s) || options.max_vertical_rate_mm_s <= 0.0 ||
        !std::isfinite(options.max_slope_rate_degrees_per_mm) ||
        options.max_slope_rate_degrees_per_mm <= 0.0 ||
        !std::isfinite(options.travel_speed_mm_s) || options.travel_speed_mm_s <= 0.0 ||
        !std::isfinite(options.section_start_z) || !std::isfinite(options.section_top_z) ||
        options.section_start_z < 0.0 || options.section_top_z <= options.section_start_z ||
        options.max_grid_samples < 4)
        return false;

    // Validate every face index before any indexed vertex access. Meshes may
    // come from imported files, so malformed topology must fail closed.
    for (const Vec3i32 &face : mesh.indices) {
        for (int corner = 0; corner < 3; ++corner) {
            if (face(corner) < 0 ||
                static_cast<std::size_t>(face(corner)) >= mesh.vertices.size())
                return false;
        }
    }

    Vec3d lo = mesh.vertices.front().cast<double>();
    Vec3d hi = lo;
    for (const Vec3f &v : mesh.vertices) {
        if (!v.allFinite()) return false;
        lo = lo.cwiseMin(v.cast<double>());
        hi = hi.cwiseMax(v.cast<double>());
    }
    const double spacing = options.sample_spacing_mm;
    const double width = hi.x() - lo.x(), depth = hi.y() - lo.y();
    if (!(width > 0.0) || !(depth > 0.0)) return false;
    const double nx_d = std::floor(width / spacing) + 1.0;
    const double ny_d = std::floor(depth / spacing) + 1.0;
    if (!std::isfinite(nx_d) || !std::isfinite(ny_d) || nx_d < 2.0 || ny_d < 2.0 ||
        nx_d * ny_d > static_cast<double>(options.max_grid_samples))
        return false;
    // This deliberately simple first planner is bounded against pathological
    // mesh/grid combinations; a spatial acceleration structure is required
    // before raising this budget for production-size models.
    if (nx_d * ny_d * static_cast<double>(mesh.indices.size()) > 50000000.0)
        return false;
    const std::size_t nx = static_cast<std::size_t>(nx_d);
    const std::size_t ny = static_cast<std::size_t>(ny_d);
    const double max_slope = std::tan(options.max_slope_angle_degrees * PI / 180.0);
    const double z_low = options.section_start_z - options.max_z_offset_mm;
    const double z_high = options.section_top_z + options.max_z_offset_mm;

    // Each grid sample selects the highest eligible facet at XY. Facets are
    // accepted only when their upward normal meets the slope limit.
    std::vector<HybridSurfacePath> candidates;
    for (std::size_t iy = 0; iy < ny; ++iy) {
        const double y = lo.y() + iy * spacing;
        HybridSurfacePath current;
        auto flush = [&]() {
            if (current.points.size() >= 2)
                candidates.emplace_back(std::move(current));
            current = HybridSurfacePath{};
        };
        for (std::size_t ix = 0; ix < nx; ++ix) {
            const double x = lo.x() + ix * spacing;
            double best_z = -std::numeric_limits<double>::infinity();
            for (const Vec3i32 &face : mesh.indices) {
                const Vec3d a = mesh.vertices[face(0)].cast<double>();
                const Vec3d b = mesh.vertices[face(1)].cast<double>();
                const Vec3d c = mesh.vertices[face(2)].cast<double>();
                const Vec3d normal = (b - a).cross(c - a);
                if (!normal.allFinite() || normal.norm() <= 1e-12 || normal.z() <= 0.0)
                    continue;
                const double nz = normal.z() / normal.norm();
                if (nz <= 0.0 || std::sqrt(std::max(0.0, 1.0 - nz * nz)) / nz > max_slope)
                    continue;
                const double denom = (b.y() - c.y()) * (a.x() - c.x()) +
                                     (c.x() - b.x()) * (a.y() - c.y());
                if (std::abs(denom) <= 1e-12) continue;
                const double wa = ((b.y() - c.y()) * (x - c.x()) +
                                   (c.x() - b.x()) * (y - c.y())) / denom;
                const double wb = ((c.y() - a.y()) * (x - c.x()) +
                                   (a.x() - c.x()) * (y - c.y())) / denom;
                const double wc = 1.0 - wa - wb;
                if (wa < -1e-8 || wb < -1e-8 || wc < -1e-8) continue;
                const double z = wa * a.z() + wb * b.z() + wc * c.z();
                if (z < z_low || z > z_high ||
                    std::abs(z - options.section_top_z) > options.max_z_offset_mm)
                    continue;
                best_z = std::max(best_z, z);
            }
            if (!std::isfinite(best_z)) {
                flush();
                continue;
            }
            const Vec3d point(x, y, best_z);
            if (!current.points.empty()) {
                const Vec3d prev = current.points.back();
                const double dx = point.x() - prev.x(), dy = point.y() - prev.y();
                const double ds = std::hypot(dx, dy);
                if (ds <= 1e-9 || std::abs(point.z() - prev.z()) / ds >
                    max_slope + 1e-8) {
                    flush();
                }
            }
            current.points.emplace_back(point);
        }
        flush();
    }

    // Validate every segment after generation. Also enforce vertical velocity
    // and rate-of-change of path slope before any candidate can be returned.
    for (const HybridSurfacePath &path : candidates) {
        double previous_slope = 0.0;
        bool have_previous_slope = false;
        for (std::size_t i = 1; i < path.points.size(); ++i) {
            const Vec3d &a = path.points[i - 1], &b = path.points[i];
            const double ds = std::hypot(b.x() - a.x(), b.y() - a.y());
            if (!(ds > 1e-9) || !a.allFinite() || !b.allFinite()) {
                output.clear();
                return false;
            }
            const double slope = (b.z() - a.z()) / ds;
            if (std::abs(slope) * options.travel_speed_mm_s >
                    options.max_vertical_rate_mm_s + 1e-8 ||
                (have_previous_slope &&
                 std::abs(std::atan(slope) - std::atan(previous_slope)) *
                     180.0 / PI / ds > options.max_slope_rate_degrees_per_mm + 1e-8) ||
                !swept_tool_segment_is_clear(a, b)) {
                output.clear();
                return false;
            }
            previous_slope = slope;
            have_previous_slope = true;
        }
    }
    output.swap(candidates);
    return true;
}

} // namespace Slic3r

#endif // slic3r_HybridNonplanarSurfacePlanner_hpp_
