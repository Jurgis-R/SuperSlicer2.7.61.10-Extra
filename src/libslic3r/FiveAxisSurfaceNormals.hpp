/// Copyright (c) 2026 Jurgis-R
/// SuperSlicer is released under the terms of the AGPLv3 or higher.
#ifndef slic3r_FiveAxisSurfaceNormals_hpp_
#define slic3r_FiveAxisSurfaceNormals_hpp_

#include "AABBMesh.hpp"
#include "ExtrusionEntity.hpp"
#include "FiveAxisKinematics.hpp"

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace Slic3r {

// Accelerated nearest-surface normal lookup for an indexed model mesh.
// The mesh must outlive this sampler. Query points and mesh vertices must be
// in the SAME coordinate frame and in millimetres. No instance/object transform
// is guessed or applied here.
class FiveAxisSurfaceNormalSampler {
public:
    explicit FiveAxisSurfaceNormalSampler(const indexed_triangle_set &mesh)
        : m_mesh(&mesh), m_index(mesh)
    {}

    FiveAxisValidationResult sample(
        const FiveAxisVec3 &point_mm,
        FiveAxisVec3 &normal_out,
        double *distance_mm = nullptr,
        double max_distance_mm = std::numeric_limits<double>::infinity()) const
    {
        normal_out = {};
        if (!five_axis_finite(point_mm) || !std::isfinite(max_distance_mm) &&
            max_distance_mm != std::numeric_limits<double>::infinity())
            return { false, "Surface-normal query point or distance limit is invalid." };
        if (max_distance_mm < 0..0)
            return { false, "Surface-normal maximum distance must be non-negative." };
        if (m_mesh->indices.empty())
            return { false, "Cannot sample tool directions from an empty triangle mesh." };

        int face_id = -1;
        Vec3d closest;
        const double squared_distance = m_index.squared_distance(
            Vec3d(point_mm.x, point_mm.y, point_mm.z), face_id, closest);
        if (!std::isfinite(squared_distance) || squared_distance < 0.0 || face_id < 0)
            return { false, "Nearest-surface query failed to identify a triangle." };
        const double distance = std::sqrt(squared_distance);
        if (!std::isfinite(distance) || distance > max_distance_mm)
            return { false, "Toolpath vertex is farther from the source mesh than the configured limit." };

        const Vec3d n = m_index.normal_by_face_id(face_id);
        normal_out = { n.x(), n.y(), n.z() };
        if (!five_axis_finite(normal_out) || !five_axis_normalize(normal_out))
            return { false, "Nearest triangle has a degenerate or invalid normal." };
        if (distance_mm != nullptr)
            *distance_mm = distance;
        return { true, {} };
    }

    // Attach outward/inward (as wound in the mesh) face normals to each
    // ExtrusionPath3D vertex. base_z_mm + z_offsets and XY are interpreted in
    // the same frame as mesh coordinates. Existing metadata is replaced only
    // after every sample succeeds, so failure leaves the path unchanged.
    FiveAxisValidationResult assign_path_normals(
        ExtrusionPath3D &path,
        double base_z_mm,
        double max_distance_mm = std::numeric_limits<double>::infinity()) const
    {
        if (path.polyline.size() < 2)
            return { false, "A surface-oriented extrusion path requires at least two vertices." };
        if (!std::isfinite(base_z_mm) ||
            (!std::isfinite(max_distance_mm) &&
             max_distance_mm != std::numeric_limits<double>::infinity()) ||
            max_distance_mm < 0.0)
            return { false, "Surface-normal path sampling received invalid settings." };

        std::vector<std::array<double, 3>> normals;
        normals.reserve(path.polyline.size());
        for (std::size_t i = 0; i < path.polyline.size(); ++i) {
            const Point &xy = path.polyline.get_point(i);
            const double z = base_z_mm + (i < path.z_offsets.size()
                ? double(path.z_offsets[i]) * SCALING_FACTOR : 0.0);
            FiveAxisVec3 normal;
            const auto result = sample({
                double(xy.x()) * SCALING_FACTOR,
                double(xy.y()) * SCALING_FACTOR,
                z
            }, normal, nullptr, max_distance_mm);
            if (!result.valid)
                return { false, "Could not derive tool direction at path vertex " +
                    std::to_string(i) + ": " + result.error };
            normals.push_back({ { normal.x, normal.y, normal.z } });
        }
        path.tool_axis_normals.swap(normals);
        return { true, {} };
    }

private:
    const indexed_triangle_set *m_mesh;
    AABBMesh m_index;
};

} // namespace Slic3r

#endif // slic3r_FiveAxisSurfaceNormals_hpp_
