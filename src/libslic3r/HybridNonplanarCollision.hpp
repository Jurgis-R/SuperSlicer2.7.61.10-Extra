///|/ Copyright (c) 2026 Jurgis-R
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
#ifndef slic3r_HybridNonplanarCollision_hpp_
#define slic3r_HybridNonplanarCollision_hpp_

#include "AABBMesh.hpp"
#include "TriangleMesh.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace Slic3r {

// Conservative sampled clearance checker for a closed, consistently oriented
// triangle mesh. This checks a cylindrical nozzle shaft and a cylindrical
// toolhead envelope above a separately modelled tip-contact region. It is not
// suitable for open/non-manifold meshes and deliberately rejects those meshes.
// The sampling margin is derived from the maximum distance between a point on
// the swept centerline and its nearest sample; the distance-to-mesh function
// is 1-Lipschitz, so passing samples with this extra margin bounds the entire
// sampled centerline between them.
struct HybridSweptToolOptions {
    double path_sample_step_mm { 0.2 };
    double nozzle_tip_exclusion_mm { 0.6 };
    double nozzle_shaft_radius_mm { 0.4 };
    double nozzle_shaft_top_mm { 8.0 };
    double toolhead_radius_mm { 8.0 };
    double toolhead_start_mm { 8.0 };
    double toolhead_top_mm { 25.0 };
    double required_clearance_mm { 0.2 };
};

inline bool hybrid_mesh_is_closed_oriented(const indexed_triangle_set &mesh)
{
    // Keep topology validation bounded: this first implementation intentionally
    // rejects very large meshes rather than allocating unbounded edge storage.
    if (mesh.vertices.empty() || mesh.indices.empty() || mesh.indices.size() > 2000000)
        return false;

    struct EdgeUse { int low; int high; int orientation; };
    std::vector<EdgeUse> edges;
    edges.reserve(mesh.indices.size() * 3);
    double signed_volume_six = 0.0;
    for (const Vec3i32 &face : mesh.indices) {
        Vec3d p[3];
        for (int corner = 0; corner < 3; ++corner) {
            const int index = face(corner);
            if (index < 0 || static_cast<std::size_t>(index) >= mesh.vertices.size())
                return false;
            p[corner] = mesh.vertices[index].cast<double>();
            if (!p[corner].allFinite())
                return false;
        }
        const Vec3d area_vector = (p[1] - p[0]).cross(p[2] - p[0]);
        if (!area_vector.allFinite() || area_vector.squaredNorm() <= 1e-20)
            return false;
        signed_volume_six += p[0].dot(p[1].cross(p[2]));
        for (int corner = 0; corner < 3; ++corner) {
            const int a = face(corner);
            const int b = face((corner + 1) % 3);
            if (a == b)
                return false;
            edges.push_back({ std::min(a, b), std::max(a, b), a < b ? 1 : -1 });
        }
    }

    // AABBMesh::hit_result::is_inside() relies on outward-facing normals.
    // Reject meshes whose global signed volume does not establish that winding.
    if (!std::isfinite(signed_volume_six) || signed_volume_six <= 1e-12)
        return false;

    std::sort(edges.begin(), edges.end(), [](const EdgeUse &a, const EdgeUse &b) {
        return a.low < b.low || (a.low == b.low && a.high < b.high);
    });
    for (std::size_t i = 0; i < edges.size();) {
        std::size_t j = i + 1;
        int orientation_sum = edges[i].orientation;
        while (j < edges.size() && edges[j].low == edges[i].low &&
               edges[j].high == edges[i].high) {
            orientation_sum += edges[j].orientation;
            ++j;
        }
        if (j - i != 2 || orientation_sum != 0)
            return false;
        i = j;
    }
    return true;
}

class HybridSweptToolCollisionValidator {
public:
    HybridSweptToolCollisionValidator(const indexed_triangle_set &mesh,
                                      const AABBMesh &aabb,
                                      const HybridSweptToolOptions &options)
        : m_aabb(aabb), m_options(options),
          m_valid(hybrid_mesh_is_closed_oriented(mesh) && valid_options(options))
    {}

    bool valid() const { return m_valid; }

    // from/to are nozzle-tip coordinates in millimetres. This verifies both
    // the nozzle shaft and upper toolhead envelope along the whole segment.
    // The tip-contact region below nozzle_tip_exclusion_mm is intentionally
    // exempt; callers must separately verify approach/departure and tip contact.
    bool segment_is_clear(const Vec3d &from, const Vec3d &to) const
    {
        if (!m_valid || !from.allFinite() || !to.allFinite())
            return false;
        const double path_length = (to - from).norm();
        if (!std::isfinite(path_length))
            return false;
        const double step = m_options.path_sample_step_mm;
        const double path_intervals_d = std::ceil(path_length / step);
        if (!std::isfinite(path_intervals_d) || path_intervals_d > 1000000.0)
            return false;
        const std::size_t path_intervals =
            std::max<std::size_t>(1, static_cast<std::size_t>(path_intervals_d));
        const double sampling_margin = step * std::sqrt(2.0) * 0.5;
        const double nozzle_clearance =
            m_options.nozzle_shaft_radius_mm + m_options.required_clearance_mm + sampling_margin;
        const double head_clearance =
            m_options.toolhead_radius_mm + m_options.required_clearance_mm + sampling_margin;

        for (std::size_t i = 0; i <= path_intervals; ++i) {
            const double t = static_cast<double>(i) / path_intervals;
            const Vec3d tip = from + (to - from) * t;
            if (!check_vertical_envelope(tip, m_options.nozzle_tip_exclusion_mm,
                                         m_options.nozzle_shaft_top_mm,
                                         nozzle_clearance, step))
                return false;
            if (!check_vertical_envelope(tip, m_options.toolhead_start_mm,
                                         m_options.toolhead_top_mm,
                                         head_clearance, step))
                return false;
        }
        return true;
    }

private:
    static bool valid_options(const HybridSweptToolOptions &o)
    {
        return std::isfinite(o.path_sample_step_mm) && o.path_sample_step_mm > 0.0 &&
            std::isfinite(o.nozzle_tip_exclusion_mm) && o.nozzle_tip_exclusion_mm >= 0.0 &&
            std::isfinite(o.nozzle_shaft_radius_mm) && o.nozzle_shaft_radius_mm > 0.0 &&
            std::isfinite(o.nozzle_shaft_top_mm) &&
            o.nozzle_shaft_top_mm > o.nozzle_tip_exclusion_mm &&
            std::isfinite(o.toolhead_radius_mm) && o.toolhead_radius_mm > 0.0 &&
            std::isfinite(o.toolhead_start_mm) &&
            o.toolhead_start_mm >= o.nozzle_shaft_top_mm &&
            std::isfinite(o.toolhead_top_mm) && o.toolhead_top_mm > o.toolhead_start_mm &&
            std::isfinite(o.required_clearance_mm) && o.required_clearance_mm >= 0.0 &&
            o.path_sample_step_mm <= 2.0;
    }

    bool check_vertical_envelope(const Vec3d &tip, double start_z, double top_z,
                                 double required_distance, double step) const
    {
        const double height = top_z - start_z;
        const double intervals_d = std::ceil(height / step);
        if (!std::isfinite(intervals_d) || intervals_d > 100000.0)
            return false;
        const std::size_t intervals =
            std::max<std::size_t>(1, static_cast<std::size_t>(intervals_d));
        const double required_squared = required_distance * required_distance;
        for (std::size_t i = 0; i <= intervals; ++i) {
            const double h = start_z + height * static_cast<double>(i) / intervals;
            const Vec3d sample = tip + Vec3d(0.0, 0.0, h);
            const double d2 = m_aabb.squared_distance(sample);
            if (!std::isfinite(d2) || d2 < required_squared)
                return false;

            // Distance-to-surface alone cannot detect a point deep inside a
            // closed solid. Ray orientation is used as a second rejection gate.
            const AABBMesh::hit_result hit = m_aabb.query_ray_hit(sample, Vec3d::UnitX());
            if (!hit.is_valid())
                return false;
            if (hit.is_hit() && hit.is_inside())
                return false;
        }
        return true;
    }

    const AABBMesh &m_aabb;
    HybridSweptToolOptions m_options;
    bool m_valid { false };
};

} // namespace Slic3r

#endif // slic3r_HybridNonplanarCollision_hpp_
