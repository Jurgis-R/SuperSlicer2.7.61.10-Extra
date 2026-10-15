/// Copyright (c) 2026 Jurgis-R
/// SuperSlicer is released under the terms of the AGPLv3 or higher.
#ifndef slic3r_FiveAxisSurfaceNormals_hpp_
#define slic3r_FiveAxisSurfaceNormals_hpp_

#include "AABBMesh.hpp"
#include "ExtrusionEntity.hpp"
#include "FiveAxisEngine.hpp"
#include "FiveAxisKinematics.hpp"
#include "Point.hpp"

#include <Eigen/Geometry>
#include <Eigen/LU>

#include <cmath>
#include <array>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace Slic3r {

class PrintObject;
class ModelVolume;

// Build the surface mesh in the same coordinate frame used by SuperSlicer's
// volume slicing: PrintObject::trafo_centered() * ModelVolume::get_matrix().
FiveAxisValidationResult transform_five_axis_print_volume_mesh(
    const PrintObject &print_object,
    const ModelVolume &volume,
    indexed_triangle_set &output);



 
// Transform a source mesh into the same millimetre coordinate frame used by
// print-space toolpaths. The caller must supply the correct composed object,
// volume, and instance transform; this function does not guess which instance
// is being sliced. Face winding is reversed for mirrored transforms so the
// resulting triangle normals retain their intended orientation.
inline FiveAxisValidationResult transform_five_axis_surface_mesh(
    const indexed_triangle_set &source,
    const Transform3d &source_to_print_mm,
    indexed_triangle_set &output)
{
    if (source.vertices.empty() || source.indices.empty())
        return { false, "Cannot transform an empty surface mesh." };

    const Eigen::Matrix4d matrix = source_to_print_mm.matrix();
    if (!matrix.allFinite())
        return { false, "Surface mesh transform contains non-finite values." };
    const double determinant = source_to_print_mm.linear().determinant();
    if (!std::isfinite(determinant) || std::abs(determinant) <= 1e-12)
        return { false, "Surface mesh transform must have a non-singular linear part." };

    for (const Vec3i32 &face : source.indices) {
        for (int corner = 0; corner < 3; ++corner) {
            if (face[corner] < 0 ||
                static_cast<std::size_t>(face[corner]) >= source.vertices.size())
                return { false, "Surface mesh contains a face with an invalid vertex index." };
        }
    }

    indexed_triangle_set transformed;
    transformed.vertices.reserve(source.vertices.size());
    transformed.indices = source.indices;
    for (const Vec3f &vertex : source.vertices) {
        const Vec3d point = source_to_print_mm * Vec3d(
            double(vertex.x()), double(vertex.y()), double(vertex.z()));
        if (!point.allFinite())
            return { false, "Surface mesh transform produced a non-finite vertex." };
        const Vec3f transformed_vertex(
            float(point.x()), float(point.y()), float(point.z()));
        if (!std::isfinite(transformed_vertex.x()) ||
            !std::isfinite(transformed_vertex.y()) ||
            !std::isfinite(transformed_vertex.z()))
            return { false, "Surface mesh transform exceeds the supported vertex coordinate range." };
        transformed.vertices.emplace_back(transformed_vertex);
    }

    if (determinant < 0.0) {
        for (Vec3i32 &face : transformed.indices)
            std::swap(face[1], face[2]);
    }
    output = std::move(transformed);
    return { true, {} };
}


 
// SuperSlicer slices a model volume using the composition
// PrintObject::trafo_centered() * ModelVolume::get_matrix(). Keep this
// composition explicit here so callers do not accidentally omit the volume
// transform or apply the centering transform twice. The input mesh is the
// volume's untransformed indexed mesh; output is in PrintObject slicing space.
inline FiveAxisValidationResult transform_five_axis_volume_mesh(
    const indexed_triangle_set &volume_mesh,
    const Transform3d &print_object_centered_transform,
    const Transform3d &volume_transform,
    indexed_triangle_set &output)
{
    if (!print_object_centered_transform.matrix().allFinite() ||
        !volume_transform.matrix().allFinite())
        return { false, "Print-object or volume transform contains non-finite values." };
    return transform_five_axis_surface_mesh(
        volume_mesh, print_object_centered_transform * volume_transform, output);
}

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
        double max_distance_mm = std::numeric_limits<double>::infinity(),
        double competing_face_distance_mm = 0.25) const
    {
        normal_out = {};
        if (!five_axis_finite(point_mm) ||
            (!std::isfinite(max_distance_mm) &&
             max_distance_mm != std::numeric_limits<double>::infinity()))
            return { false, "Surface-normal query point or distance limit is invalid." };
        if (max_distance_mm < 0.0 ||
            !std::isfinite(competing_face_distance_mm) ||
            competing_face_distance_mm < 0.0)
            return { false, "Surface-normal distance limits must be finite and non-negative." };
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

        // A nearest-face query cannot distinguish the two sides of a thin
        // wall. Probe through the nearest surface along its normal and reject
        // a nearby second face whose normal substantially opposes the first.
        // This is intentionally conservative and can be disabled with a zero
        // threshold by callers that have an independently verified source face.
        if (competing_face_distance_mm > 0.0) {
            const Vec3d surface = closest;
            const Vec3d normal(normal_out.x, normal_out.y, normal_out.z);
            const double epsilon = 1e-4;
            for (const double sign : { -1.0, 1.0 }) {
                const Vec3d ray_origin = surface + normal * (sign * epsilon);
                const Vec3d ray_direction = normal * (-sign);
                const auto hits = m_index.query_ray_hits(ray_origin, ray_direction);
                for (const auto &hit : hits) {
                    if (!hit.is_hit() || hit.face() == face_id)
                        continue;
                    const double hit_distance = hit.distance() - epsilon;
                    if (hit_distance <= 1e-3 ||
                        hit_distance > competing_face_distance_mm)
                        continue;
                    const Vec3d other_normal = m_index.normal_by_face_id(hit.face());
                    if (other_normal.allFinite() &&
                        other_normal.norm() > 1e-12 &&
                        (other_normal.normalized().dot(normal) < -0.5))
                        return { false, "Surface normal is ambiguous: an opposing face lies within the thin-wall ambiguity distance." };
                }
            }
        }
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

 
// Resolves a complete extrusion path against a set of candidate source-volume
// meshes. A single source must explain every vertex; near-equal candidates are
// rejected instead of silently switching between overlapping volumes. This
// does not infer provenance from the slicer's merged region paths: callers must
// supply the candidate volumes in the same print-space coordinate frame.
class FiveAxisSurfaceCorrespondence {
public:
    FiveAxisValidationResult add_source(
        int volume_id,
        const indexed_triangle_set &print_space_mesh)
    {
        if (print_space_mesh.vertices.empty() || print_space_mesh.indices.empty())
            return { false, "Cannot register an empty source-volume mesh." };
        for (const auto &source : m_sources)
            if (source->volume_id == volume_id)
                return { false, "A source-volume ID may only be registered once." };
        m_sources.emplace_back(std::make_unique<SourceSampler>(volume_id, print_space_mesh));
        return { true, {} };
    }

    std::size_t source_count() const { return m_sources.size(); }

    FiveAxisValidationResult assign_path_normals(
        ExtrusionPath3D &path,
        double base_z_mm,
        double max_distance_mm,
        double ambiguity_margin_mm = 0.05,
        int *matched_volume_id = nullptr) const
    {
        if (path.polyline.size() < 2)
            return { false, "A surface-oriented extrusion path requires at least two vertices." };
        if (!std::isfinite(base_z_mm) ||
            (!std::isfinite(max_distance_mm) &&
             max_distance_mm != std::numeric_limits<double>::infinity()) ||
            max_distance_mm < 0.0 || !std::isfinite(ambiguity_margin_mm) ||
            ambiguity_margin_mm < 0.0)
            return { false, "Surface-correspondence settings are invalid." };
        if (m_sources.empty())
            return { false, "No candidate source volumes have been registered." };

        struct Candidate {
            int volume_id;
            double mean_distance;
            std::vector<std::array<double, 3>> normals;
        };
        std::vector<Candidate> viable;
        viable.reserve(m_sources.size());

        for (const auto &source : m_sources) {
            Candidate candidate { source->volume_id, 0.0, {} };
            candidate.normals.reserve(path.polyline.size());
            bool complete = true;
            double distance_sum = 0.0;
            for (std::size_t i = 0; i < path.polyline.size(); ++i) {
                const Point &xy = path.polyline.get_point(i);
                const double z = base_z_mm + (i < path.z_offsets.size()
                    ? double(path.z_offsets[i]) * SCALING_FACTOR : 0.0);
                FiveAxisVec3 normal;
                double distance = 0.0;
                const auto result = source->sampler.sample({
                    double(xy.x()) * SCALING_FACTOR,
                    double(xy.y()) * SCALING_FACTOR,
                    z
                }, normal, &distance, max_distance_mm);
                if (!result.valid) {
                    complete = false;
                    break;
                }
                distance_sum += distance;
                candidate.normals.push_back({ { normal.x, normal.y, normal.z } });
            }
            if (complete) {
                candidate.mean_distance = distance_sum / double(path.polyline.size());
                viable.emplace_back(std::move(candidate));
            }
        }

        if (viable.empty())
            return { false, "No single source-volume mesh matches every path vertex within the distance limit." };

        std::sort(viable.begin(), viable.end(), [](const Candidate &a, const Candidate &b) {
            return a.mean_distance < b.mean_distance;
        });
        if (viable.size() > 1 &&
            viable[1].mean_distance - viable[0].mean_distance <= ambiguity_margin_mm)
            return { false, "Path-to-volume correspondence is ambiguous; two source meshes are equally plausible." };

        path.tool_axis_normals.swap(viable[0].normals);
        if (matched_volume_id != nullptr)
            *matched_volume_id = viable[0].volume_id;
        return { true, {} };
    }

private:
    struct SourceSampler {
        int volume_id;
        FiveAxisSurfaceNormalSampler sampler;
        SourceSampler(int id, const indexed_triangle_set &mesh)
            : volume_id(id), sampler(mesh) {}
    };
    std::vector<std::unique_ptr<SourceSampler>> m_sources;
};

} // namespace Slic3r

#endif // slic3r_FiveAxisSurfaceNormals_hpp_
