///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
// Native geometry stays behind the C ABI. Operations build private results and
// publish only after validation and cancellation checks. Scenes own all data used
// by their AABB; no Volume, Print, or scratch-storage pointer is retained.
#include "libslic3r/Api/plugin/c/slic3r_mesh_queries.h"
#include "libslic3r/AABBTreeIndirect.hpp"
#include "libslic3r/ShortEdgeCollapse.hpp"
#include "libslic3r/TriangleSetSampling.hpp"
#include <algorithm>
#include <cmath>
#include <climits>
#include <cstdio>
#include <limits>
#include <memory>
#include <set>
#include <stdexcept>
#include <vector>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>

struct mesh_query_mesh
{
    indexed_triangle_set triangles;
};
struct mesh_query_scene
{
    indexed_triangle_set triangles;
    std::vector<uint64_t> labels;
    Slic3r::AABBTreeIndirect::Tree<3, float> tree;
};
struct mesh_query_samples
{
    std::vector<mesh_query_sample> values;
    double area = 0;
};
struct mesh_query_hits
{
    std::vector<std::vector<mesh_query_hit>> rays;
};
namespace {
using namespace Slic3r;
struct Cancelled
{};
// Check cancellation only on the caller thread, outside parallel native kernels.
void checkpoint(const mesh_query_cancel *cancel);
// Reject matrices that cannot represent a finite affine transform.
Transform3d matrix(const c_matrix4d &input);
// Check native integer limits and geometry before invoking native algorithms.
void validate(const indexed_triangle_set &mesh);
// Catch every failure before it crosses C and copy the diagnostic to the caller.
template<class F> mesh_query_status guarded(mesh_query_diagnostic *d, F fn);
// Closed slab intervals handle parallel rays on a box face without 0 * infinity.
bool box_intersects(const Eigen::AlignedBox<float, 3> &box, const Vec3d &origin,
                    const Vec3d &direction, double near, double far);
// Traverse the native AABB with an explicit distance interval. First-hit mode
// prunes beyond the current best distance and keeps deterministic triangle ties.
std::vector<mesh_query_hit> cast_ray(const mesh_query_scene &scene, const mesh_query_ray &ray, bool all);
// Copy one bounded range without allocating or throwing across C.
template<class T> uint64_t copy_range(const std::vector<T> &values, uint64_t offset, T *out, uint64_t capacity);
void checkpoint(const mesh_query_cancel *cancel) {
    if (cancel && cancel->is_cancelled && cancel->is_cancelled(cancel->context))
        throw Cancelled{};
}
Transform3d matrix(const c_matrix4d &input) {
    Transform3d result = Transform3d::Identity();
    for (int row = 0; row < 4; ++row)
        for (int col = 0; col < 4; ++col) {
            const double value = input.value[row * 4 + col];
            if (!std::isfinite(value))
                throw std::invalid_argument("Non-finite mesh transform.");
            result.matrix()(row, col) = value;
        }
    if (input.value[12] != 0 || input.value[13] != 0 || input.value[14] != 0 || input.value[15] != 1)
        throw std::invalid_argument("Mesh transform must be affine.");
    return result;
}
void validate(const indexed_triangle_set &mesh) {
    if (mesh.vertices.size() > size_t(INT_MAX) || mesh.indices.size() > size_t(INT_MAX))
        throw std::invalid_argument("Mesh exceeds native index limits.");
    for (const Vec3f &v : mesh.vertices)
        if (!v.allFinite())
            throw std::invalid_argument("Non-finite mesh vertex.");
    for (const Vec3i32 &face : mesh.indices)
        for (int axis = 0; axis < 3; ++axis)
            if (face[axis] < 0 || size_t(face[axis]) >= mesh.vertices.size())
                throw std::invalid_argument("Invalid mesh triangle index.");
}
template<class F> mesh_query_status guarded(mesh_query_diagnostic *d, F fn) {
    if (d)
        d->message[0] = 0;
    try {
        fn();
        return MESH_QUERY_OK;
    } catch (const Cancelled &) {
        if (d)
            std::snprintf(d->message, sizeof(d->message), "Mesh query cancelled.");
        return MESH_QUERY_CANCELLED;
    } catch (const std::invalid_argument &e) {
        if (d)
            std::snprintf(d->message, sizeof(d->message), "%s", e.what());
        return MESH_QUERY_INVALID;
    } catch (const std::exception &e) {
        if (d)
            std::snprintf(d->message, sizeof(d->message), "%s", e.what());
        return MESH_QUERY_ERROR;
    } catch (...) {
        if (d)
            std::snprintf(d->message, sizeof(d->message), "Unknown mesh query failure.");
        return MESH_QUERY_ERROR;
    }
}
bool box_intersects(const Eigen::AlignedBox<float, 3> &box, const Vec3d &origin,
                    const Vec3d &direction, double near, double far)
{
    for (int axis = 0; axis < 3; ++axis) {
        if (direction[axis] == 0) {
            if (origin[axis] < box.min()[axis] || origin[axis] > box.max()[axis]) return false;
        } else {
            double entry = (double(box.min()[axis]) - origin[axis]) / direction[axis];
            double exit = (double(box.max()[axis]) - origin[axis]) / direction[axis];
            if (entry > exit) std::swap(entry, exit);
            near = std::max(near, entry); far = std::min(far, exit);
            if (near > far) return false;
        }
    }
    return true;
}
std::vector<mesh_query_hit> cast_ray(const mesh_query_scene &scene, const mesh_query_ray &ray, bool all) {
    const Vec3d origin(ray.origin.x, ray.origin.y, ray.origin.z);
    Vec3d direction(ray.direction.x, ray.direction.y, ray.direction.z);
    const double length = direction.stableNorm();
    if (!origin.allFinite() || !direction.allFinite() || !std::isfinite(length) || length == 0 ||
        !std::isfinite(ray.min_distance) || ray.min_distance < 0 || std::isnan(ray.max_distance) ||
        ray.max_distance < ray.min_distance)
        throw std::invalid_argument("Invalid ray geometry or interval.");
    direction /= length;
    std::vector<mesh_query_hit> result;
    std::vector<size_t> pending;
    if (!scene.tree.empty())
        pending.push_back(0);
    double far = ray.max_distance;
    while (!pending.empty()) {
        const size_t index = pending.back();
        pending.pop_back();
        const AABBTreeIndirect::Tree<3, float>::Node &node = scene.tree.node(index);
        if (!node.is_valid() || !box_intersects(node.bbox, origin, direction, ray.min_distance, far))
            continue;
        if (!node.is_leaf()) {
            pending.push_back(index * 2 + 2);
            pending.push_back(index * 2 + 1);
            continue;
        }
        const Vec3i32 &face = scene.triangles.indices[node.idx];
        double distance, u, v;
        if (!AABBTreeIndirect::detail::intersect_triangle(origin, direction, scene.triangles.vertices[face[0]],
                                                          scene.triangles.vertices[face[1]],
                                                          scene.triangles.vertices[face[2]], distance, u, v, 0.000001))
            continue;
        if (!std::isfinite(distance))
            throw std::invalid_argument("Intersection distance overflow.");
        if (distance < ray.min_distance || distance > far)
            continue;
        const Vec3f normal = its_face_normal(scene.triangles, node.idx);
        const mesh_query_hit hit{distance, {normal.x(), normal.y(), normal.z()}, scene.labels[node.idx], node.idx};
        if (all)
            result.push_back(hit);
        else if (result.empty() || distance < result.front().distance ||
                 (distance == result.front().distance && node.idx < result.front().triangle_id)) {
            result.assign(1, hit);
            far = distance;
        }
    }
    std::sort(result.begin(), result.end(), [](const mesh_query_hit &a, const mesh_query_hit &b) {
        return a.distance < b.distance || (a.distance == b.distance && a.triangle_id < b.triangle_id);
    });
    return result;
}
template<class T> uint64_t copy_range(const std::vector<T> &values, uint64_t offset, T *out, uint64_t capacity) {
    if (!out || offset >= values.size())
        return 0;
    const size_t count = size_t(std::min<uint64_t>(capacity, values.size() - size_t(offset)));
    std::copy_n(values.data() + size_t(offset), count, out);
    return count;
}
} // namespace

extern "C" {
mesh_query_status mesh_query_mesh_create(mesh_query_mesh **out, mesh_query_diagnostic *d) {
    if (out)
        *out = nullptr;
    return guarded(d, [&] {
        if (!out)
            throw std::invalid_argument("Missing mesh output.");
        *out = new mesh_query_mesh;
    });
}
void mesh_query_mesh_destroy(mesh_query_mesh *mesh) { delete mesh; }
mesh_query_status mesh_query_mesh_append(mesh_query_mesh *mesh,
                                         const triangle_mesh_handle *source,
                                         c_matrix4d transform,
                                         const mesh_query_cancel *cancel,
                                         mesh_query_diagnostic *d) {
    return guarded(d, [&] {
        if (!mesh || !source)
            throw std::invalid_argument("Missing mesh.");
        checkpoint(cancel);
        indexed_triangle_set copy = reinterpret_cast<const Slic3r::TriangleMesh *>(source)->its;
        validate(copy);
        its_transform(copy, matrix(transform));
        validate(copy);
        if (copy.vertices.size() + mesh->triangles.vertices.size() > size_t(INT_MAX) ||
            copy.indices.size() + mesh->triangles.indices.size() > size_t(INT_MAX))
            throw std::invalid_argument("Merged mesh exceeds native index limits.");
        indexed_triangle_set merged = mesh->triangles;
        its_merge(merged, copy);
        checkpoint(cancel);
        mesh->triangles = std::move(merged);
    });
}
mesh_query_status mesh_query_mesh_decimate(mesh_query_mesh *mesh,
                                           uint64_t target,
                                           const mesh_query_cancel *cancel,
                                           mesh_query_diagnostic *d) {
    return guarded(d, [&] {
        if (!mesh || !target || target > size_t(INT_MAX))
            throw std::invalid_argument("Invalid mesh or triangle target.");
        checkpoint(cancel);
        indexed_triangle_set copy = mesh->triangles;
        if (copy.indices.size() > target)
            its_short_edge_collpase(copy, size_t(target));
        validate(copy);
        checkpoint(cancel);
        mesh->triangles = std::move(copy);
    });
}
mesh_query_status mesh_query_scene_create(const mesh_query_part *parts,
                                          uint64_t count,
                                          const mesh_query_cancel *cancel,
                                          mesh_query_scene **out,
                                          mesh_query_diagnostic *d) {
    if (out)
        *out = nullptr;
    return guarded(d, [&] {
        if (!out || (count && !parts) || count > size_t(INT_MAX))
            throw std::invalid_argument("Invalid scene input.");
        std::unique_ptr<mesh_query_scene> scene(new mesh_query_scene);
        std::set<uint64_t> labels;
        checkpoint(cancel);
        for (uint64_t i = 0; i < count; ++i) {
            if (!parts[i].mesh || !labels.insert(parts[i].geometry_id).second)
                throw std::invalid_argument("Invalid or duplicate scene geometry.");
            indexed_triangle_set copy = parts[i].mesh->triangles;
            its_transform(copy, matrix(parts[i].transform));
            validate(copy);
            if (copy.vertices.size() + scene->triangles.vertices.size() > size_t(INT_MAX) ||
                copy.indices.size() + scene->triangles.indices.size() > size_t(INT_MAX))
                throw std::invalid_argument("Scene exceeds native index limits.");
            // Degenerate faces have no surface normal and cannot be sampled.
            copy.indices.erase(std::remove_if(copy.indices.begin(), copy.indices.end(),
                                              [&](const Vec3i32 &f) {
                                                  const Vec3f cross = (copy.vertices[f[1]] - copy.vertices[f[0]])
                                                                          .cross(copy.vertices[f[2]] -
                                                                                 copy.vertices[f[0]]);
                                                  if (!cross.allFinite() || !std::isfinite(cross.norm()))
                                                      throw std::invalid_argument("Triangle area overflow.");
                                                  return cross.squaredNorm() == 0;
                                              }),
                               copy.indices.end());
            scene->labels.insert(scene->labels.end(), copy.indices.size(), parts[i].geometry_id);
            its_merge(scene->triangles, copy);
            checkpoint(cancel);
        }
        scene->tree = AABBTreeIndirect::build_aabb_tree_over_indexed_triangle_set(scene->triangles.vertices,
                                                                                  scene->triangles.indices);
        checkpoint(cancel);
        *out = scene.release();
    });
}
void mesh_query_scene_destroy(mesh_query_scene *scene) { delete scene; }
uint64_t mesh_query_scene_triangle_count(const mesh_query_scene *scene, uint64_t id) {
    return scene ? uint64_t(std::count(scene->labels.begin(), scene->labels.end(), id)) : 0;
}
mesh_query_status mesh_query_scene_sample(const mesh_query_scene *scene,
                                          uint64_t count,
                                          const mesh_query_cancel *cancel,
                                          mesh_query_samples **out,
                                          mesh_query_diagnostic *d) {
    if (out)
        *out = nullptr;
    return guarded(d, [&] {
        if (!scene || !out || count > size_t(INT_MAX))
            throw std::invalid_argument("Invalid sampling input.");
        checkpoint(cancel);
        std::unique_ptr<mesh_query_samples> result(new mesh_query_samples);
        if (!scene->triangles.indices.empty()) {
            // Match native float accumulation, but reject overflow before sampling.
            float area = 0;
            for (const Vec3i32 &f : scene->triangles.indices)
                area += .5 *
                    double((scene->triangles.vertices[f[1]] - scene->triangles.vertices[f[0]])
                               .cross(scene->triangles.vertices[f[2]] - scene->triangles.vertices[f[0]])
                               .norm());
            if (!std::isfinite(area))
                throw std::invalid_argument("Surface area overflow.");
            result->area = area;
            if (area > 0 && count) {
                const TriangleSetSamples samples = sample_its_uniform_parallel(size_t(count), scene->triangles);
                result->values.reserve(size_t(count));
                for (size_t i = 0; i < samples.positions.size(); ++i) {
                    const Vec3f &p = samples.positions[i], &n = samples.normals[i];
                    result->values.push_back({{p.x(), p.y(), p.z()},
                                              {n.x(), n.y(), n.z()},
                                              scene->labels[samples.triangle_indices[i]],
                                              samples.triangle_indices[i]});
                }
            }
        }
        checkpoint(cancel);
        *out = result.release();
    });
}
void mesh_query_samples_destroy(mesh_query_samples *samples) { delete samples; }
uint64_t mesh_query_samples_count(const mesh_query_samples *samples) { return samples ? samples->values.size() : 0; }
double mesh_query_samples_area(const mesh_query_samples *samples) { return samples ? samples->area : 0; }
uint64_t mesh_query_samples_copy(const mesh_query_samples *samples,
                                 uint64_t offset,
                                 mesh_query_sample *out,
                                 uint64_t capacity) {
    return samples ? copy_range(samples->values, offset, out, capacity) : 0;
}
mesh_query_status mesh_query_scene_cast(const mesh_query_scene *scene,
                                        const mesh_query_ray *rays,
                                        uint64_t count,
                                        int32_t all_hits,
                                        const mesh_query_cancel *cancel,
                                        mesh_query_hits **out,
                                        mesh_query_diagnostic *d) {
    if (out)
        *out = nullptr;
    return guarded(d, [&] {
        if (!scene || !out || (count && !rays) || count > size_t(INT_MAX) || (all_hits != 0 && all_hits != 1))
            throw std::invalid_argument("Invalid ray batch.");
        checkpoint(cancel);
        std::unique_ptr<mesh_query_hits> result(new mesh_query_hits);
        result->rays.resize(size_t(count));
        for (size_t begin = 0; begin < count; begin += 256) {
            const size_t end = std::min(size_t(count), begin + 256);
            tbb::parallel_for(tbb::blocked_range<size_t>(begin, end), [&](const tbb::blocked_range<size_t> &range) {
                for (size_t i = range.begin(); i < range.end(); ++i)
                    result->rays[i] = cast_ray(*scene, rays[i], all_hits != 0);
            });
            checkpoint(cancel);
        }
        *out = result.release();
    });
}
void mesh_query_hits_destroy(mesh_query_hits *hits) { delete hits; }
uint64_t mesh_query_hits_ray_count(const mesh_query_hits *hits) { return hits ? hits->rays.size() : 0; }
uint64_t mesh_query_hits_count(const mesh_query_hits *hits, uint64_t ray) {
    return hits && ray < hits->rays.size() ? hits->rays[size_t(ray)].size() : 0;
}
uint64_t mesh_query_hits_copy(
    const mesh_query_hits *hits, uint64_t ray, uint64_t offset, mesh_query_hit *out, uint64_t capacity) {
    return hits && ray < hits->rays.size() ? copy_range(hits->rays[size_t(ray)], offset, out, capacity) : 0;
}
}
