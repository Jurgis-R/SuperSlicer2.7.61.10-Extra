///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
/*
MeshQueryViews.hpp
==================

Small RAII wrappers over the mesh-query C ABI from slic3r_mesh_queries.h. They
hide the double-output C pattern and turn every non-OK status into
MeshQueryError, while preserving the exact same ownership and units.

Typical C++ raytracing workflow:

    WorkingMesh mesh;
    mesh.append(volume.mesh().handle(), object.transform_centered());
    mesh.decimate(16000); // optional

    MeshQueryScene scene({{mesh.handle(), matrix4d_identity(), 0}});
    const mesh_query_ray ray{{0, 0, 10}, {0, 0, -1}, 0, 100};
    const MeshRayHits hits = scene.cast({ray}, false);
    const std::vector<mesh_query_hit> first_hits = hits.ray(0);

Use all_hits=false for a nearest-hit visibility query. Use all_hits=true when
the caller needs to count crossings or inspect multiple overlapping/touching
surfaces. Returned hit ranges are already sorted by distance, then triangle id.

All coordinates and distances are unscaled millimetres, matching TriangleMesh
vertices. None of these wrappers accepts coord_t / scaled slicer XY values
unless the caller converts them first.

Lifetime model:
- WorkingMesh owns a mutable host work mesh. Appending/decimating requires
  exclusive access to that object.
- MeshQueryScene owns an immutable copied scene and acceleration structure. It
  does not borrow the WorkingMesh after construction.
- MeshSurfaceSamples and MeshRayHits own immutable result handles. Their values()
  and ray() methods return std::vector copies, so the returned vectors may outlive
  the wrapper objects.
*/
#ifndef slic3r_api_MeshQueryViews_hpp_
#define slic3r_api_MeshQueryViews_hpp_
#include "libslic3r/Api/plugin/c/slic3r_mesh_queries.h"
#include <memory>
#include <stdexcept>
#include <vector>
namespace slic3r_api {

/* Exception raised by the RAII wrappers when the C ABI reports a failure.
   status keeps the machine-readable reason; what() contains the host diagnostic
   copied before the C call returned. */
class MeshQueryError : public std::runtime_error
{
public:
    MeshQueryError(mesh_query_status status, const char *message) : std::runtime_error(message), status(status) {}
    mesh_query_status status;
};
inline void check_mesh_query(mesh_query_status status, const mesh_query_diagnostic &diagnostic) {
    if (status != MESH_QUERY_OK)
        throw MeshQueryError(status, diagnostic.message);
}

/*
Mutable host-side triangle buffer.

append() copies the TriangleMesh behind `source` immediately and applies the
provided affine transform in millimetres. The source mesh may be destroyed or
mutated after append() returns. decimate() simplifies the owned copy in place;
it is approximate and may keep more triangles than requested. A WorkingMesh can
be reused in several scenes because scenes copy from it during construction.
*/
class WorkingMesh
{
public:
    WorkingMesh() : m_mesh(nullptr, mesh_query_mesh_destroy) {
        mesh_query_mesh *out = nullptr;
        mesh_query_diagnostic d{};
        const mesh_query_status s = mesh_query_mesh_create(&out, &d);
        m_mesh.reset(out);
        check_mesh_query(s, d);
    }
    void append(const triangle_mesh_handle *source, c_matrix4d transform, const mesh_query_cancel *cancel = nullptr) {
        mesh_query_diagnostic d{};
        const mesh_query_status s = mesh_query_mesh_append(m_mesh.get(), source, transform, cancel, &d);
        check_mesh_query(s, d);
    }
    void decimate(uint64_t target, const mesh_query_cancel *cancel = nullptr) {
        mesh_query_diagnostic d{};
        const mesh_query_status s = mesh_query_mesh_decimate(m_mesh.get(), target, cancel, &d);
        check_mesh_query(s, d);
    }
    const mesh_query_mesh *handle() const { return m_mesh.get(); }

private:
    std::unique_ptr<mesh_query_mesh, void (*)(mesh_query_mesh *)> m_mesh;
};

/* Owned result of scene.sample(). values() copies every record into a vector.
   The copied samples include position, normal, geometry_id and triangle_id in
   scene coordinates. area() is the total sampled surface area in mm^2. */
class MeshSurfaceSamples
{
public:
    explicit MeshSurfaceSamples(mesh_query_samples *owned) : m_samples(owned, mesh_query_samples_destroy) {}
    double area() const { return mesh_query_samples_area(m_samples.get()); }
    std::vector<mesh_query_sample> values() const {
        std::vector<mesh_query_sample> out(size_t(mesh_query_samples_count(m_samples.get())));
        mesh_query_samples_copy(m_samples.get(), 0, out.data(), out.size());
        return out;
    }

private:
    std::unique_ptr<mesh_query_samples, void (*)(mesh_query_samples *)> m_samples;
};

/*
Owned result of scene.cast().

ray(index) returns the sorted hit list for one input ray. It returns an empty
vector for a miss and throws std::out_of_range only when the requested ray index
does not exist. The copied vector is independent of this MeshRayHits object.
*/
class MeshRayHits
{
public:
    explicit MeshRayHits(mesh_query_hits *owned) : m_hits(owned, mesh_query_hits_destroy) {}
    uint64_t ray_count() const { return mesh_query_hits_ray_count(m_hits.get()); }
    std::vector<mesh_query_hit> ray(uint64_t index) const {
        if (index >= ray_count())
            throw std::out_of_range("Ray index outside result.");
        std::vector<mesh_query_hit> out(size_t(mesh_query_hits_count(m_hits.get(), index)));
        mesh_query_hits_copy(m_hits.get(), index, 0, out.data(), out.size());
        return out;
    }

private:
    std::unique_ptr<mesh_query_hits, void (*)(mesh_query_hits *)> m_hits;
};

/*
Immutable ray/sampling scene.

The constructor copies every mesh_query_part into a single transformed scene and
builds an acceleration structure. Every geometry_id must be unique within the
scene because ray hits and surface samples use it as the caller-visible part
label. The input vector is borrowed only during construction, and the WorkingMesh
objects may be destroyed afterwards.

cast() keeps the input ray order: result.ray(i) belongs to rays[i]. Use a
nonzero min_distance to ignore self-intersections near the ray origin when the
origin lies on or very close to a surface.
*/
class MeshQueryScene
{
public:
    explicit MeshQueryScene(const std::vector<mesh_query_part> &parts, const mesh_query_cancel *cancel = nullptr)
        : m_scene(nullptr, mesh_query_scene_destroy) {
        mesh_query_scene *out = nullptr;
        mesh_query_diagnostic d{};
        const mesh_query_status s = mesh_query_scene_create(parts.data(), parts.size(), cancel, &out, &d);
        m_scene.reset(out);
        check_mesh_query(s, d);
    }
    uint64_t triangle_count(uint64_t geometry_id) const {
        return mesh_query_scene_triangle_count(m_scene.get(), geometry_id);
    }
    MeshSurfaceSamples sample(uint64_t count, const mesh_query_cancel *cancel = nullptr) const {
        mesh_query_samples *out = nullptr;
        mesh_query_diagnostic d{};
        const mesh_query_status s = mesh_query_scene_sample(m_scene.get(), count, cancel, &out, &d);
        MeshSurfaceSamples result(out);
        check_mesh_query(s, d);
        return result;
    }
    MeshRayHits cast(const std::vector<mesh_query_ray> &rays,
                     bool all_hits,
                     const mesh_query_cancel *cancel = nullptr) const {
        mesh_query_hits *out = nullptr;
        mesh_query_diagnostic d{};
        const mesh_query_status s = mesh_query_scene_cast(m_scene.get(), rays.data(), rays.size(), all_hits ? 1 : 0,
                                                          cancel, &out, &d);
        MeshRayHits result(out);
        check_mesh_query(s, d);
        return result;
    }

private:
    std::unique_ptr<mesh_query_scene, void (*)(mesh_query_scene *)> m_scene;
};
} // namespace slic3r_api
#endif
