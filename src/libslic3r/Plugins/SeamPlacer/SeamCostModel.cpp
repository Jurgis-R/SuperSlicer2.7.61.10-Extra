///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "SeamCostModel.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <vector>

#include <boost/log/trivial.hpp>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include "libslic3r/AABBTreeIndirect.hpp"
#include "libslic3r/Api/plugin/cpp/VolumeViews.hpp"
#include "libslic3r/KDTreeIndirect.hpp"
#include "libslic3r/PointUtils.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/ShortEdgeCollapse.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/TriangleSetSampling.hpp"
#include "libslic3r/Utils.hpp"

namespace Slic3r {
namespace SeamCostModel {

// These values match the legacy cost path so moving the model does not change
// the density of visibility samples, mesh decimation, or hemispherical rays.
constexpr size_t kVisibilitySamplesCount = 30000;
constexpr size_t kDecimationTriangleCountTarget = 16000;
constexpr size_t kRaysPerSampleAxis = 5;

/* Evaluate the legacy normalized falloff used for travel and angle costs. */
float gauss(float value, float mean_x_coord, float mean_value, float falloff_speed);

/* Convert a signed corner angle into the legacy asymmetric corner penalty. */
float compute_angle_penalty(float ccw_angle);

/* Reproduce the legacy arm-length angle sampling for Cost candidates. */
std::vector<float> calculate_cost_angles_internal(const Points &points, float min_arm_length);

/* Compute Cost visibility with an object-local mesh and the legacy ray model. */
std::vector<float> calculate_cost_visibility(const slic3r_api::Object &object,
                                             const std::vector<c_vec3f> &points);

namespace {

struct CoordinateFunctor {
    const std::vector<Vec3f> *coordinates = nullptr;

    const float &operator()(size_t index, size_t dimension) const
    {
        return (*coordinates)[index][dimension];
    }
};

/* Keep only the visibility state needed by Cost; painted seam trees belong to
   the legacy placer and are intentionally not rebuilt here. */
struct VisibilityModel {
    TriangleSetSamples mesh_samples;
    std::vector<float> mesh_samples_visibility;
    CoordinateFunctor mesh_samples_coordinate_functor;
    KDTreeIndirect<3, float, CoordinateFunctor> mesh_samples_tree { CoordinateFunctor{} };
    float mesh_samples_radius = 0.0f;
};

/* Return one deterministic hemisphere direction for the ray grid. */
Vec3f sample_hemisphere_uniform(const Vec2f &samples);

/* Transform a local +Z hemisphere direction into the frame of a surface normal. */
Vec3f direction_in_normal_frame(const Vec3f &normal, const Vec3f &local_direction);

/* Raycast the sampled mesh while preserving negative-volume parity handling. */
std::vector<float> raycast_visibility(const AABBTreeIndirect::Tree<3, float> &raycasting_tree,
                                      const indexed_triangle_set &triangles,
                                      const TriangleSetSamples &samples,
                                      size_t negative_volumes_start_index);

/* Build the object-local visibility samples and their spatial lookup tree. */
void build_visibility_model(const slic3r_api::Object &object, VisibilityModel &model);

/* Convert plugin ABI mesh/matrix values to the private geometry algorithm types. */
Transform3d to_transform(const c_matrix4d &matrix);
indexed_triangle_set copy_triangle_mesh(const slic3r_api::TriangleMesh &mesh);

/* Interpolate visibility from nearby surface samples at one object-local point. */
float calculate_point_visibility(const VisibilityModel &model, const Vec3f &position);

/* Calculate the signed corner angle using the same arm-window walk as legacy. */
std::vector<float> calculate_polyline_angles_at_vertices(const Points &polyline,
                                                         const std::vector<float> &lengths,
                                                         float min_arm_length);

Vec3f sample_hemisphere_uniform(const Vec2f &samples)
{
    const float term1 = 2.0f * float(PI) * samples.x();
    const float term2 = 2.0f * std::sqrt(samples.y() - samples.y() * samples.y());
    return {std::cos(term1) * term2, std::sin(term1) * term2,
        std::abs(1.0f - 2.0f * samples.y())};
}

Vec3f direction_in_normal_frame(const Vec3f &normal, const Vec3f &local_direction)
{
    const Vec3f z_axis = normal.normalized();
    const Vec3f x_seed = std::abs(z_axis.x()) > 0.99f ? Vec3f(0, 1, 0) : Vec3f(1, 0, 0);
    const Vec3f y_axis = z_axis.cross(x_seed).normalized();
    const Vec3f x_axis = y_axis.cross(z_axis);
    return local_direction.x() * x_axis + local_direction.y() * y_axis + local_direction.z() * z_axis;
}

Transform3d to_transform(const c_matrix4d &matrix)
{
    Transform3d result = Transform3d::Identity();
    for (size_t row = 0; row < 4; ++row)
        for (size_t column = 0; column < 4; ++column)
            result.matrix()(int(row), int(column)) = matrix.value[row * 4 + column];
    return result;
}

indexed_triangle_set copy_triangle_mesh(const slic3r_api::TriangleMesh &mesh)
{
    indexed_triangle_set result;
    result.vertices.reserve(mesh.vertex_count());
    for (uint32_t vertex_index = 0; vertex_index < mesh.vertex_count(); ++vertex_index) {
        const c_vec3f vertex = mesh.vertex(vertex_index);
        result.vertices.emplace_back(vertex.x, vertex.y, vertex.z);
    }
    result.indices.reserve(mesh.triangle_count());
    for (uint32_t triangle_index = 0; triangle_index < mesh.triangle_count(); ++triangle_index) {
        const c_triangle_indices triangle = mesh.triangle(triangle_index);
        result.indices.emplace_back(triangle.a, triangle.b, triangle.c);
    }
    return result;
}

std::vector<float> raycast_visibility(const AABBTreeIndirect::Tree<3, float> &raycasting_tree,
                                      const indexed_triangle_set &triangles,
                                      const TriangleSetSamples &samples,
                                      size_t negative_volumes_start_index)
{
    BOOST_LOG_TRIVIAL(debug)
        << "SeamCostModel: raycast visibility of " << samples.positions.size()
        << " samples over " << triangles.indices.size() << " triangles: start";

    const float step_size = 1.0f / float(kRaysPerSampleAxis);
    std::vector<Vec3f> precomputed_sample_directions(kRaysPerSampleAxis * kRaysPerSampleAxis);
    for (size_t x_index = 0; x_index < kRaysPerSampleAxis; ++x_index) {
        const float sample_x = float(x_index) * step_size + step_size / 2.0f;
        for (size_t y_index = 0; y_index < kRaysPerSampleAxis; ++y_index) {
            const size_t direction_index = x_index * kRaysPerSampleAxis + y_index;
            const float sample_y = float(y_index) * step_size + step_size / 2.0f;
            precomputed_sample_directions[direction_index] = sample_hemisphere_uniform({sample_x, sample_y});
        }
    }

    const bool model_contains_negative_parts = negative_volumes_start_index < triangles.indices.size();
    std::vector<float> result(samples.positions.size());
    tbb::parallel_for(tbb::blocked_range<size_t>(0, result.size()),
        [&triangles, &precomputed_sample_directions, model_contains_negative_parts,
         negative_volumes_start_index, &raycasting_tree, &result, &samples](tbb::blocked_range<size_t> range) {
            // Reuse hit storage for all rays handled by one worker.
            std::vector<igl::Hit> hits;
            for (size_t sample_index = range.begin(); sample_index < range.end(); ++sample_index) {
                result[sample_index] = 1.0f;
                constexpr float decrease_step = 1.0f /
                    float(kRaysPerSampleAxis * kRaysPerSampleAxis);
                const Vec3f &center = samples.positions[sample_index];
                const Vec3f &normal = samples.normals[sample_index];
                for (const Vec3f &local_direction : precomputed_sample_directions) {
                    Vec3f final_ray_direction = direction_in_normal_frame(normal, local_direction);
                    Vec3d ray_origin = (center + normal * 0.01f).cast<double>();
                    if (!model_contains_negative_parts) {
                        igl::Hit hitpoint;
                        const Vec3d ray_direction = final_ray_direction.cast<double>();
                        const bool hit = AABBTreeIndirect::intersect_ray_first_hit(
                            triangles.vertices, triangles.indices, raycasting_tree,
                            ray_origin, ray_direction, hitpoint);
                        if (hit && its_face_normal(triangles, hitpoint.id).dot(final_ray_direction) <= 0)
                            result[sample_index] -= decrease_step;
                        continue;
                    }

                    const bool casting_from_negative_volume = samples.triangle_indices[sample_index]
                        >= negative_volumes_start_index;
                    if (casting_from_negative_volume) {
                        final_ray_direction = -final_ray_direction;
                        ray_origin = (center - normal * 0.01f).cast<double>();
                    }
                    const Vec3d ray_direction = final_ray_direction.cast<double>();
                    const bool some_hit = AABBTreeIndirect::intersect_ray_all_hits(
                        triangles.vertices, triangles.indices, raycasting_tree,
                        ray_origin, ray_direction, hits);
                    if (!some_hit) continue;

                    int counter = 0;
                    // Reverse hit analysis starts outside every solid and tracks
                    // ordinary and negative-volume faces with opposite parity.
                    for (int hit_index = int(hits.size()) - 1; hit_index >= 0; --hit_index) {
                        const Vec3f face_normal = its_face_normal(triangles, hits[hit_index].id);
                        const float orientation = face_normal.dot(final_ray_direction);
                        const int sign = int(orientation > 0.0f) - int(orientation < 0.0f);
                        if (hits[hit_index].id >= int(negative_volumes_start_index))
                            counter -= sign;
                        else
                            counter += sign;
                    }
                    if (counter == 0) result[sample_index] -= decrease_step;
                }
            }
        });

    BOOST_LOG_TRIVIAL(debug)
        << "SeamCostModel: raycast visibility of " << samples.positions.size()
        << " samples over " << triangles.indices.size() << " triangles: end";
    return result;
}

void build_visibility_model(const slic3r_api::Object &object, VisibilityModel &model)
{
    BOOST_LOG_TRIVIAL(debug) << "SeamCostModel: gather occlusion meshes: start";
    const Transform3d object_transform = to_transform(object.transform_centered());
    indexed_triangle_set triangle_set;
    indexed_triangle_set negative_volumes_set;
    for (uint32_t volume_index = 0; volume_index < object.volume_count(); ++volume_index) {
        const slic3r_api::Volume volume = object.volume(volume_index);
        if (!volume.is_model_part() && !volume.is_negative())
            continue;
        const Transform3d model_transformation = to_transform(volume.matrix());
        const indexed_triangle_set model_its = copy_triangle_mesh(volume.mesh());
        indexed_triangle_set transformed_its = model_its;
        its_transform(transformed_its, model_transformation);
        if (volume.is_model_part())
            its_merge(triangle_set, transformed_its);
        else
            its_merge(negative_volumes_set, transformed_its);
    }
    BOOST_LOG_TRIVIAL(debug) << "SeamCostModel: gather occlusion meshes: end";

    its_short_edge_collpase(triangle_set, kDecimationTriangleCountTarget);
    its_short_edge_collpase(negative_volumes_set, kDecimationTriangleCountTarget);
    const size_t negative_volumes_start_index = triangle_set.indices.size();
    its_merge(triangle_set, negative_volumes_set);
    its_transform(triangle_set, object_transform);

    model.mesh_samples = sample_its_uniform_parallel(kVisibilitySamplesCount, triangle_set);
    model.mesh_samples_coordinate_functor = CoordinateFunctor{&model.mesh_samples.positions};
    model.mesh_samples_tree = KDTreeIndirect<3, float, CoordinateFunctor>(
        model.mesh_samples_coordinate_functor, model.mesh_samples.positions.size());

    // The radius is chosen so a neighborhood contains more than four samples
    // with approximately 90% probability, matching the legacy cost path.
    const float probability = 0.9f;
    const float samples = 4.0f;
    const float density = float(kVisibilitySamplesCount) / model.mesh_samples.total_area;
    const float search_area = samples / (-std::log(probability) * density);
    model.mesh_samples_radius = std::sqrt(search_area / float(PI));

    const AABBTreeIndirect::Tree<3, float> raycasting_tree =
        AABBTreeIndirect::build_aabb_tree_over_indexed_triangle_set(
            triangle_set.vertices, triangle_set.indices);
    model.mesh_samples_visibility = raycast_visibility(
        raycasting_tree, triangle_set, model.mesh_samples, negative_volumes_start_index);
}

float calculate_point_visibility(const VisibilityModel &model, const Vec3f &position)
{
    const std::vector<size_t> nearby_points = find_nearby_points(
        model.mesh_samples_tree, position, model.mesh_samples_radius);
    if (nearby_points.empty()) return 1.0f;

    const auto distance_to_plane = [](const Vec3f &point, const Vec3f &plane_origin,
                                      const Vec3f &plane_normal) {
        return std::abs((point - plane_origin).dot(plane_normal));
    };
    float total_weight = 0.0f;
    float total_visibility = 0.0f;
    for (const size_t sample_index : nearby_points) {
        const Vec3f &sample_point = model.mesh_samples.positions[sample_index];
        const Vec3f &sample_normal = model.mesh_samples.normals[sample_index];
        const float weight = model.mesh_samples_radius -
            distance_to_plane(position, sample_point, sample_normal) +
            model.mesh_samples_radius - (position - sample_point).norm();
        total_visibility += weight * model.mesh_samples_visibility[sample_index];
        total_weight += weight;
    }
    return total_visibility / total_weight;
}

std::vector<float> calculate_polyline_angles_at_vertices(const Points &polyline,
                                                         const std::vector<float> &lengths,
                                                         float min_arm_length)
{
    std::vector<float> result(polyline.size());
    assert(polyline.size() - 1 == lengths.size());
    if (polyline.size() == 1) {
        result[0] = 0.0f;
        return result;
    }

    size_t index_previous = 0;
    size_t index_current = 0;
    size_t index_next = 0;
    size_t index_start = 0;
    size_t index_end = 0;
    float distance_to_previous = 0.0f;
    float distance_to_next = 0.0f;
    const bool is_polygon = polyline.front() == polyline.back();
    if (is_polygon) {
        index_end = polyline.size() - 1;
        while (distance_to_previous < min_arm_length) {
            index_previous = prev_idx_modulo(index_previous, lengths.size());
            distance_to_previous += lengths[index_previous];
        }
    } else {
        index_start = 1;
        index_end = polyline.size() - 1;
        result.front() = -float(PI) / 2.0f;
        result.back() = -float(PI) / 2.0f;
    }

    for (size_t index = index_start; index < index_end; ++index) {
        while (index_previous < lengths.size() &&
               distance_to_previous - lengths[index_previous] > min_arm_length) {
            distance_to_previous -= lengths[index_previous];
            index_previous = is_polygon ? next_idx_modulo(index_previous, lengths.size()) : index_previous + 1;
        }
        while (distance_to_next < min_arm_length && index_next < lengths.size()) {
            distance_to_next += lengths[index_next];
            index_next = is_polygon ? next_idx_modulo(index_next, lengths.size()) : index_next + 1;
        }

        const Point &point_previous = polyline[index_previous];
        const Point &point_current = polyline[index_current];
        const Point &point_next = polyline[index_next];
        result[index_current] = float(angle_ccw(
            point_current - point_previous, point_next - point_current));
        const float current_distance = lengths[index_current];
        ++index_current;
        distance_to_previous += current_distance;
        distance_to_next -= current_distance;
    }
    if (is_polygon) result.back() = result.front();
    return result;
}

} // namespace

float gauss(float value, float mean_x_coord, float mean_value, float falloff_speed)
{
    // base function: ((e^((1/(x^2+1)))-1)/(e-1))
    const float shifted = value - mean_x_coord;
    const float denominator = falloff_speed * shifted * shifted + 1.0f;
    const float exponent = 1.0f / denominator;
    return mean_value * (std::exp(exponent) - 1.0f) / (std::exp(1.0f) - 1.0f);
}

float compute_angle_penalty(float ccw_angle)
{
    // The Gaussian and sigmoid terms deliberately make concave corners less
    // attractive than convex corners while retaining a smooth score.
    return gauss(ccw_angle, 0.0f, 1.0f, 3.0f) +
           1.0f / (2.0f + std::exp(-ccw_angle));
}

std::vector<float> calculate_cost_angles_internal(const Points &points, float min_arm_length)
{
    if (points.size() < 2) return std::vector<float>(points.size(), 0.0f);

    Points ordered_points = points;
    const bool reverse = ordered_points.front() == ordered_points.back() &&
                         Polygon(ordered_points).is_clockwise();
    if (reverse) std::reverse(ordered_points.begin(), ordered_points.end());

    std::vector<float> lengths;
    lengths.reserve(ordered_points.size() - 1);
    for (size_t index = 0; index + 1 < ordered_points.size(); ++index)
        lengths.push_back(float((unscale_p(ordered_points[index]) -
                                 unscale_p(ordered_points[index + 1])).norm()));
    if (std::none_of(lengths.begin(), lengths.end(), [](float length) { return length > 0.0f; }))
        return std::vector<float>(points.size(), 0.0f);

    std::vector<float> angles = calculate_polyline_angles_at_vertices(
        ordered_points, lengths, min_arm_length);
    if (reverse) std::reverse(angles.begin(), angles.end());
    return angles;
}

std::vector<float> calculate_cost_angles(const std::vector<c_point> &points, float min_arm_length)
{
    Points internal_points;
    internal_points.reserve(points.size());
    for (const c_point point : points)
        internal_points.emplace_back(point.x, point.y);
    return calculate_cost_angles_internal(internal_points, min_arm_length);
}

std::vector<float> calculate_cost_visibility(const slic3r_api::Object &object,
                                             const std::vector<c_vec3f> &points)
{
    VisibilityModel model;
    build_visibility_model(object, model);
    std::vector<float> visibility;
    visibility.reserve(points.size());
    for (const c_vec3f point : points)
        visibility.push_back(calculate_point_visibility(model, Vec3f(point.x, point.y, point.z)));
    return visibility;
}

} // namespace SeamCostModel
} // namespace Slic3r
