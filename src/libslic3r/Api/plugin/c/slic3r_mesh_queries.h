///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_mesh_queries_h_
#define slic3r_mesh_queries_h_
#define SLIC3R_PLUGIN_API_MESH_QUERIES_MAJOR 1u
#define SLIC3R_PLUGIN_API_MESH_QUERIES_MINOR 0u
#include "slic3r_geometry_types.h"

/*
Mesh query C ABI
================

This API lets plugins ask the host to perform mesh operations that would be
expensive or unsafe to duplicate outside libslic3r: copying TriangleMesh data,
building an AABB scene, sampling surface points, and raycasting.

Typical raytracing workflow:
  1. Create one or more mesh_query_mesh objects with mesh_query_mesh_create().
  2. Append TriangleMesh handles with mesh_query_mesh_append(). The host copies
     the triangles immediately, so the source mesh is not retained.
  3. Optionally simplify a work mesh with mesh_query_mesh_decimate().
  4. Build an immutable mesh_query_scene from mesh_query_part records. Each part
     supplies a copied work mesh, a finite affine transform and a geometry_id.
  5. Build one or more mesh_query_ray records in the same unscaled millimetre
     coordinate system as the scene.
  6. Call mesh_query_scene_cast(). Use all_hits=0 for the nearest intersection
     per ray, or all_hits=1 when inside/outside parity, occlusion stacks, or
     negative volumes need every hit.
  7. Read counts with mesh_query_hits_ray_count()/mesh_query_hits_count(), copy
     records with mesh_query_hits_copy(), then destroy the result.
  8. Destroy scenes and work meshes when no later query needs them.

Ownership and lifetime:
  - mesh_query_mesh, mesh_query_scene, mesh_query_samples and mesh_query_hits are
    owned handles returned by the host. Destroy each successful output exactly
    once with its matching destroy function. Destroying NULL is allowed.
  - Input arrays, rays, parts and diagnostics are borrowed only for the duration
    of the call. Output handles own their copied results and may outlive the
    input arrays and the scene they were derived from, unless stated otherwise.
  - Work meshes are mutable and require exclusive access while appending or
    decimating. Scenes and query results are immutable after creation and allow
    concurrent read-only calls when each call uses its own output slot.
  - Do not destroy a handle while another call is using it.

Units and geometry:
  - Coordinates, distances and transforms are unscaled millimetres, matching
    TriangleMesh vertices. They are not coord_t / scaled 2D slicer units.
  - Scene creation removes degenerate triangles because they have no usable
    surface normal and cannot be sampled or raycast reliably.
  - Ray directions only select the direction; the host normalizes them before
    testing. Hit distances are measured along that normalized direction.

No C++ or third-party types cross this boundary.
*/
#ifdef __cplusplus
extern "C" {
#endif
typedef struct triangle_mesh_handle triangle_mesh_handle;
typedef struct mesh_query_mesh mesh_query_mesh;
typedef struct mesh_query_scene mesh_query_scene;
typedef struct mesh_query_samples mesh_query_samples;
typedef struct mesh_query_hits mesh_query_hits;
typedef enum mesh_query_status {
    MESH_QUERY_OK = 1,
    MESH_QUERY_INVALID = -1,
    MESH_QUERY_ERROR = -2,
    MESH_QUERY_CANCELLED = -3
} mesh_query_status;
/* Caller-owned optional diagnostic; overwritten by each status-returning call.
   Its message is technical and intended for logs or exception text. */
typedef struct mesh_query_diagnostic
{
    char message[512];
} mesh_query_diagnostic;
/* Called serially on the calling thread between native stages/batches. Must not
   throw or reenter the current operation. Nonzero requests cancellation. Native
   decimation and sampling stages finish before observing a pending request. */
typedef int32_t (*mesh_query_cancel_callback)(void *context);
typedef struct mesh_query_cancel
{
    void *context;
    mesh_query_cancel_callback is_cancelled;
} mesh_query_cancel;
typedef struct mesh_query_part
{
    const mesh_query_mesh *mesh;
    c_matrix4d transform; /* finite affine matrix, row-major, applied while building the scene */
    uint64_t geometry_id; /* opaque label; must be unique within the scene */
} mesh_query_part;
typedef struct mesh_query_sample
{
    c_vec3f position;
    c_vec3f normal; /* unit oriented normal of the transformed triangle */
    uint64_t geometry_id;
    uint64_t triangle_id; /* global scene index, stable for the scene lifetime */
} mesh_query_sample;
typedef struct mesh_query_ray
{
    c_vec3d origin;
    c_vec3d direction;   /* finite, nonzero; normalized by the host before intersection tests */
    double min_distance; /* inclusive, finite and >= 0, in millimetres */
    double max_distance; /* inclusive, >= min_distance; +infinity permitted */
} mesh_query_ray;
typedef struct mesh_query_hit
{
    double distance;     /* millimetres along the normalized ray direction */
    c_vec3f normal;      /* oriented normal of the triangle that was hit */
    uint64_t geometry_id; /* copied from the mesh_query_part containing the triangle */
    uint64_t triangle_id; /* global scene triangle index, stable for this scene lifetime */
} mesh_query_hit;

/* Create an empty mutable work mesh. The output slot must not already own a
   mesh_query_mesh. On failure the output is NULL. */
SLIC3R_HOST_API mesh_query_status mesh_query_mesh_create(mesh_query_mesh **out, mesh_query_diagnostic *diagnostic);
SLIC3R_HOST_API void mesh_query_mesh_destroy(mesh_query_mesh *mesh);
/* Copy one TriangleMesh into a mutable work mesh after applying transform.
   The source handle is borrowed only for this call; the work mesh keeps its own
   transformed copy. On failure the existing work mesh content is unchanged. */
SLIC3R_HOST_API mesh_query_status mesh_query_mesh_append(mesh_query_mesh *mesh,
                                                         const triangle_mesh_handle *source,
                                                         c_matrix4d transform,
                                                         const mesh_query_cancel *cancel,
                                                         mesh_query_diagnostic *diagnostic);
/* Approximate short-edge collapse; may undershoot the target and does not
   guarantee topology preservation. Zero target is invalid. */
SLIC3R_HOST_API mesh_query_status mesh_query_mesh_decimate(mesh_query_mesh *mesh,
                                                           uint64_t target,
                                                           const mesh_query_cancel *cancel,
                                                           mesh_query_diagnostic *diagnostic);
/* Build an immutable query scene from copied work meshes. The scene owns a
   merged, transformed triangle set and an acceleration structure; it does not
   retain pointers to parts or work meshes. geometry_id values must be unique so
   hit records can unambiguously report which part was intersected. */
SLIC3R_HOST_API mesh_query_status mesh_query_scene_create(const mesh_query_part *parts,
                                                          uint64_t count,
                                                          const mesh_query_cancel *cancel,
                                                          mesh_query_scene **out,
                                                          mesh_query_diagnostic *diagnostic);
SLIC3R_HOST_API void mesh_query_scene_destroy(mesh_query_scene *scene);
/* Number of nondegenerate triangles for an opaque geometry label; zero if absent. */
SLIC3R_HOST_API uint64_t mesh_query_scene_triangle_count(const mesh_query_scene *scene, uint64_t geometry_id);
/* Area-weighted sampling with the native deterministic seed 27644437. Empty or
   zero-area scenes yield no samples and total_area=0, regardless of count. */
SLIC3R_HOST_API mesh_query_status mesh_query_scene_sample(const mesh_query_scene *scene,
                                                          uint64_t count,
                                                          const mesh_query_cancel *cancel,
                                                          mesh_query_samples **out,
                                                          mesh_query_diagnostic *diagnostic);
SLIC3R_HOST_API void mesh_query_samples_destroy(mesh_query_samples *samples);
SLIC3R_HOST_API uint64_t mesh_query_samples_count(const mesh_query_samples *samples);
SLIC3R_HOST_API double mesh_query_samples_area(const mesh_query_samples *samples);
/* Copy up to capacity records starting at offset; return the number copied.
   Null handles/buffers and out-of-range offsets copy zero. */
SLIC3R_HOST_API uint64_t mesh_query_samples_copy(const mesh_query_samples *samples,
                                                 uint64_t offset,
                                                 mesh_query_sample *out,
                                                 uint64_t capacity);
/* Cast a batch of rays against an immutable scene.

   all_hits must be 0 or 1. Results have one range per input ray, including
   empty ranges for misses. Each range is sorted by distance, then triangle ID.
   First-hit queries return the first record of that ordering. Surface hits are
   not deduplicated: overlapping triangles, touching geometry and front/back
   intersections are all reported as the ray traversal finds them, then sorted.

   For a simple visibility check, use all_hits=0 and test whether the ray range
   is empty. For parity checks, negative-volume handling, or "how many surfaces
   did this ray cross?", use all_hits=1 and interpret the returned sorted list
   yourself. */
SLIC3R_HOST_API mesh_query_status mesh_query_scene_cast(const mesh_query_scene *scene,
                                                        const mesh_query_ray *rays,
                                                        uint64_t count,
                                                        int32_t all_hits,
                                                        const mesh_query_cancel *cancel,
                                                        mesh_query_hits **out,
                                                        mesh_query_diagnostic *diagnostic);
SLIC3R_HOST_API void mesh_query_hits_destroy(mesh_query_hits *hits);
SLIC3R_HOST_API uint64_t mesh_query_hits_ray_count(const mesh_query_hits *hits);
SLIC3R_HOST_API uint64_t mesh_query_hits_count(const mesh_query_hits *hits, uint64_t ray);
SLIC3R_HOST_API uint64_t mesh_query_hits_copy(
    const mesh_query_hits *hits, uint64_t ray, uint64_t offset, mesh_query_hit *out, uint64_t capacity);
#ifdef __cplusplus
}
#endif
#endif
