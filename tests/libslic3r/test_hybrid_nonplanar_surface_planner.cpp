#include "libslic3r/HybridNonplanarSurfacePlanner.hpp"
#include "libslic3r/HybridNonplanarExtrusion.hpp"
#include "libslic3r/HybridNonplanarCollision.hpp"

#include <catch2/catch.hpp>

using namespace Slic3r;


static indexed_triangle_set make_closed_cube()
{
    indexed_triangle_set mesh;
    mesh.vertices = {
        Vec3f(0.f, 0.f, 0.f), Vec3f(1.f, 0.f, 0.f),
        Vec3f(1.f, 1.f, 0.f), Vec3f(0.f, 1.f, 0.f),
        Vec3f(0.f, 0.f, 1.f), Vec3f(1.f, 0.f, 1.f),
        Vec3f(1.f, 1.f, 1.f), Vec3f(0.f, 1.f, 1.f)
    };
    mesh.indices = {
        Vec3i32(0, 2, 1), Vec3i32(0, 3, 2),
        Vec3i32(4, 5, 6), Vec3i32(4, 6, 7),
        Vec3i32(0, 1, 5), Vec3i32(0, 5, 4),
        Vec3i32(1, 2, 6), Vec3i32(1, 6, 5),
        Vec3i32(2, 3, 7), Vec3i32(2, 7, 6),
        Vec3i32(3, 0, 4), Vec3i32(3, 4, 7)
    };
    return mesh;
}

static indexed_triangle_set make_flat_surface()
{
    indexed_triangle_set mesh;
    mesh.vertices = {
        Vec3f(0.f, 0.f, 2.f), Vec3f(2.f, 0.f, 2.f),
        Vec3f(2.f, 2.f, 2.f), Vec3f(0.f, 2.f, 2.f)
    };
    mesh.indices = { Vec3i32(0, 1, 2), Vec3i32(0, 2, 3) };
    return mesh;
}

TEST_CASE("Hybrid surface planner generates new paths from upward mesh facets",
          "[HybridNonplanarSurfacePlanner]")
{
    const indexed_triangle_set mesh = make_flat_surface();
    HybridSurfacePlannerOptions options;
    options.sample_spacing_mm = 0.5;
    options.section_start_z = 1.5;
    options.section_top_z = 2.0;
    options.max_z_offset_mm = 0.2;

    std::vector<HybridSurfacePath> paths;
    const auto clear = [](const Vec3d &, const Vec3d &) { return true; };
    REQUIRE(plan_hybrid_surface_paths(mesh, options, clear, paths));
    REQUIRE_FALSE(paths.empty());
    for (const HybridSurfacePath &path : paths) {
        REQUIRE(path.points.size() >= 2);
        for (const Vec3d &point : path.points)
            REQUIRE(point.z() == Approx(2.0));
    }
}

TEST_CASE("Hybrid surface planner fails closed without swept-tool collision validation",
          "[HybridNonplanarSurfacePlanner]")
{
    const indexed_triangle_set mesh = make_flat_surface();
    HybridSurfacePlannerOptions options;
    std::vector<HybridSurfacePath> paths(1);
    paths.front().points.emplace_back(1.0, 1.0, 2.0);

    REQUIRE_FALSE(plan_hybrid_surface_paths(mesh, options, {}, paths));
    REQUIRE(paths.empty());
}

TEST_CASE("Hybrid surface planner rejects a candidate when any tool sweep collides",
          "[HybridNonplanarSurfacePlanner]")
{
    const indexed_triangle_set mesh = make_flat_surface();
    HybridSurfacePlannerOptions options;
    options.sample_spacing_mm = 0.5;
    options.section_start_z = 1.5;
    options.section_top_z = 2.0;
    options.max_z_offset_mm = 0.2;

    std::vector<HybridSurfacePath> paths;
    const auto colliding = [](const Vec3d &, const Vec3d &) { return false; };
    REQUIRE_FALSE(plan_hybrid_surface_paths(mesh, options, colliding, paths));
    REQUIRE(paths.empty());
}

TEST_CASE("Hybrid surface planner rejects malformed mesh indices",
          "[HybridNonplanarSurfacePlanner]")
{
    indexed_triangle_set mesh = make_flat_surface();
    mesh.indices.front()(2) = 999999;
    HybridSurfacePlannerOptions options;
    std::vector<HybridSurfacePath> paths;
    const auto clear = [](const Vec3d &, const Vec3d &) { return true; };

    REQUIRE_FALSE(plan_hybrid_surface_paths(mesh, options, clear, paths));
    REQUIRE(paths.empty());
}

TEST_CASE("Hybrid surface planner rejects invalid limits and pathological sample grids",
          "[HybridNonplanarSurfacePlanner]")
{
    const indexed_triangle_set mesh = make_flat_surface();
    HybridSurfacePlannerOptions options;
    std::vector<HybridSurfacePath> paths;
    const auto clear = [](const Vec3d &, const Vec3d &) { return true; };

    options.max_slope_angle_degrees = 90.0;
    REQUIRE_FALSE(plan_hybrid_surface_paths(mesh, options, clear, paths));
    options.max_slope_angle_degrees = 35.0;
    options.sample_spacing_mm = 1e-9;
    REQUIRE_FALSE(plan_hybrid_surface_paths(mesh, options, clear, paths));
    REQUIRE(paths.empty());
}

TEST_CASE("Hybrid surface planner rejects paths above the configured vertical-rate limit",
          "[HybridNonplanarSurfacePlanner]")
{
    indexed_triangle_set mesh;
    mesh.vertices = {
        Vec3f(0.f, 0.f, 1.9f), Vec3f(2.f, 0.f, 2.1f),
        Vec3f(2.f, 2.f, 2.1f), Vec3f(0.f, 2.f, 1.9f)
    };
    mesh.indices = { Vec3i32(0, 1, 2), Vec3i32(0, 2, 3) };

    HybridSurfacePlannerOptions options;
    options.sample_spacing_mm = 0.5;
    options.section_start_z = 1.5;
    options.section_top_z = 2.0;
    options.max_z_offset_mm = 0.2;
    options.travel_speed_mm_s = 10.0;
    options.max_vertical_rate_mm_s = 0.1;

    std::vector<HybridSurfacePath> paths;
    const auto clear = [](const Vec3d &, const Vec3d &) { return true; };
    REQUIRE_FALSE(plan_hybrid_surface_paths(mesh, options, clear, paths));
    REQUIRE(paths.empty());
}

TEST_CASE("Hybrid surface planner rejects paths above the configured slope-rate limit",
          "[HybridNonplanarSurfacePlanner]")
{
    indexed_triangle_set mesh;
    // Two connected planar facets with a slope change along X.
    mesh.vertices = {
        Vec3f(0.f, 0.f, 2.f), Vec3f(1.f, 0.f, 2.f),
        Vec3f(1.f, 2.f, 2.f), Vec3f(2.f, 0.f, 2.2f),
        Vec3f(2.f, 2.f, 2.2f)
    };
    mesh.indices = {
        Vec3i32(0, 1, 2), Vec3i32(1, 3, 4), Vec3i32(1, 4, 2)
    };

    HybridSurfacePlannerOptions options;
    options.sample_spacing_mm = 0.25;
    options.section_start_z = 1.5;
    options.section_top_z = 2.0;
    options.max_z_offset_mm = 0.25;
    options.travel_speed_mm_s = 1.0;
    options.max_vertical_rate_mm_s = 10.0;
    options.max_slope_rate_degrees_per_mm = 0.01;

    std::vector<HybridSurfacePath> paths;
    const auto clear = [](const Vec3d &, const Vec3d &) { return true; };
    REQUIRE_FALSE(plan_hybrid_surface_paths(mesh, options, clear, paths));
    REQUIRE(paths.empty());
}

TEST_CASE("Hybrid surface planner rejects grids too narrow to sample a path",
          "[HybridNonplanarSurfacePlanner]")
{
    indexed_triangle_set mesh;
    mesh.vertices = {
        Vec3f(0.f, 0.f, 2.f), Vec3f(2.f, 0.f, 2.f),
        Vec3f(2.f, 0.1f, 2.f), Vec3f(0.f, 0.1f, 2.f)
    };
    mesh.indices = { Vec3i32(0, 1, 2), Vec3i32(0, 2, 3) };

    HybridSurfacePlannerOptions options;
    options.sample_spacing_mm = 0.4;
    options.section_start_z = 1.5;
    options.section_top_z = 2.0;
    options.max_z_offset_mm = 0.2;

    std::vector<HybridSurfacePath> paths;
    const auto clear = [](const Vec3d &, const Vec3d &) { return true; };
    REQUIRE_FALSE(plan_hybrid_surface_paths(mesh, options, clear, paths));
    REQUIRE(paths.empty());
}


TEST_CASE("Hybrid surface paths convert to scaled 3D extrusion paths relative to writer Z",
          "[HybridNonplanarSurfacePlanner][HybridNonplanarExtrusion]")
{
    HybridSurfacePath candidate;
    candidate.points = { Vec3d(1.0, 2.0, 2.1), Vec3d(1.5, 2.5, 2.2) };
    const std::vector<HybridSurfacePath> candidates{ candidate };
    ExtrusionAttributes attributes(ExtrusionRole::ExternalPerimeter,
                                   ExtrusionFlow(0.08, 0.4f, 0.2f));
    std::vector<ExtrusionPath3D> paths;

    REQUIRE(make_hybrid_surface_extrusion_paths(candidates, 2.0, attributes, paths));
    REQUIRE(paths.size() == 1);
    REQUIRE(paths.front().polyline.size() == 2);
    REQUIRE(paths.front().z_offsets.size() == 2);
    CHECK(unscale(paths.front().polyline.get_point(0).x()) == Approx(1.0));
    CHECK(unscale(paths.front().polyline.get_point(0).y()) == Approx(2.0));
    CHECK(unscale(paths.front().z_offsets[0]) == Approx(0.1));
    CHECK(unscale(paths.front().z_offsets[1]) == Approx(0.2));
    CHECK(paths.front().mm3_per_mm() == Approx(0.08));
    CHECK_FALSE(paths.front().can_reverse());
}

TEST_CASE("Hybrid surface extrusion conversion rejects invalid input without partial output",
          "[HybridNonplanarExtrusion]")
{
    ExtrusionAttributes attributes(ExtrusionRole::ExternalPerimeter,
                                   ExtrusionFlow(0.08, 0.4f, 0.2f));
    std::vector<ExtrusionPath3D> paths;
    paths.emplace_back(attributes, false);
    paths.back().push_back(Point::new_scale(0.0, 0.0), 0);
    HybridSurfacePath too_short;
    too_short.points = { Vec3d(0.0, 0.0, 2.0) };
    REQUIRE_FALSE(make_hybrid_surface_extrusion_paths({ too_short }, 2.0, attributes, paths));
    REQUIRE(paths.empty());

    HybridSurfacePath non_finite;
    non_finite.points = { Vec3d(0.0, 0.0, 2.0),
                          Vec3d(std::numeric_limits<double>::quiet_NaN(), 1.0, 2.0) };
    REQUIRE_FALSE(make_hybrid_surface_extrusion_paths({ non_finite }, 2.0, attributes, paths));
    REQUIRE(paths.empty());

    attributes.mm3_per_mm = 0.0;
    HybridSurfacePath valid;
    valid.points = { Vec3d(0.0, 0.0, 2.0), Vec3d(1.0, 0.0, 2.0) };
    REQUIRE_FALSE(make_hybrid_surface_extrusion_paths({ valid }, 2.0, attributes, paths));
    REQUIRE(paths.empty());
}


TEST_CASE("Hybrid surface planning and extrusion conversion fail closed on a colliding segment",
          "[HybridNonplanarExtrusion]")
{
    const indexed_triangle_set mesh = make_flat_surface();
    HybridSurfacePlannerOptions options;
    options.sample_spacing_mm = 0.5;
    options.section_start_z = 1.5;
    options.section_top_z = 2.0;
    options.max_z_offset_mm = 0.2;
    ExtrusionAttributes attributes(ExtrusionRole::ExternalPerimeter,
                                   ExtrusionFlow(0.08, 0.4f, 0.2f));
    std::vector<ExtrusionPath3D> paths;
    paths.emplace_back(attributes, false);
    paths.back().push_back(Point::new_scale(0.0, 0.0), 0);

    const auto colliding = [](const Vec3d &, const Vec3d &) { return false; };
    REQUIRE_FALSE(plan_hybrid_surface_extrusion_paths(
        mesh, options, 2.0, attributes, colliding, paths));
    REQUIRE(paths.empty());
}

TEST_CASE("Hybrid surface planning can produce converted paths only after segment validation",
          "[HybridNonplanarExtrusion]")
{
    const indexed_triangle_set mesh = make_flat_surface();
    HybridSurfacePlannerOptions options;
    options.sample_spacing_mm = 0.5;
    options.section_start_z = 1.5;
    options.section_top_z = 2.0;
    options.max_z_offset_mm = 0.2;
    ExtrusionAttributes attributes(ExtrusionRole::ExternalPerimeter,
                                   ExtrusionFlow(0.08, 0.4f, 0.2f));
    std::vector<ExtrusionPath3D> paths;
    const auto clear = [](const Vec3d &, const Vec3d &) { return true; };

    REQUIRE(plan_hybrid_surface_extrusion_paths(
        mesh, options, 2.0, attributes, clear, paths));
    REQUIRE_FALSE(paths.empty());
    for (const ExtrusionPath3D &path : paths) {
        REQUIRE(path.polyline.size() >= 2);
        REQUIRE(path.z_offsets.size() == path.polyline.size());
        REQUIRE(path.mm3_per_mm() == Approx(0.08));
    }
}


TEST_CASE("Hybrid surface mesh transform handles translation and mirrored winding",
          "[HybridNonplanarSurfacePlanner]")
{
    const indexed_triangle_set source = make_flat_surface();
    Transform3d transform = Transform3d::Identity();
    transform.translate(Vec3d(10.0, -3.0, 1.0));
    transform.scale(Vec3d(-1.0, 1.0, 1.0));

    indexed_triangle_set transformed;
    REQUIRE(transform_hybrid_surface_mesh(source, transform, transformed));
    REQUIRE(transformed.vertices.size() == source.vertices.size());
    REQUIRE(transformed.indices.size() == source.indices.size());
    CHECK(transformed.vertices[0].x() == Approx(10.0f));
    CHECK(transformed.vertices[0].y() == Approx(-3.0f));
    CHECK(transformed.vertices[0].z() == Approx(3.0f));

    const Vec3i32 &face = transformed.indices.front();
    const Vec3d a = transformed.vertices[face(0)].cast<double>();
    const Vec3d b = transformed.vertices[face(1)].cast<double>();
    const Vec3d c = transformed.vertices[face(2)].cast<double>();
    CHECK((b - a).cross(c - a).z() > 0.0);
}

TEST_CASE("Hybrid surface mesh transform rejects invalid topology and singular transforms",
          "[HybridNonplanarSurfacePlanner]")
{
    indexed_triangle_set source = make_flat_surface();
    indexed_triangle_set transformed = make_flat_surface();
    source.indices.front()(1) = 123456;
    REQUIRE_FALSE(transform_hybrid_surface_mesh(source, Transform3d::Identity(), transformed));
    REQUIRE(transformed.vertices.empty());
    REQUIRE(transformed.indices.empty());

    source = make_flat_surface();
    Transform3d singular = Transform3d::Identity();
    singular.scale(Vec3d(1.0, 0.0, 1.0));
    REQUIRE_FALSE(transform_hybrid_surface_mesh(source, singular, transformed));
    REQUIRE(transformed.vertices.empty());
    REQUIRE(transformed.indices.empty());
}


TEST_CASE("Hybrid surface pipeline transforms the source mesh before planning",
          "[HybridNonplanarExtrusion]")
{
    const indexed_triangle_set source = make_flat_surface();
    Transform3d transform = Transform3d::Identity();
    transform.translate(Vec3d(10.0, -3.0, 1.0));
    ExtrusionAttributes attributes(ExtrusionRole::ExternalPerimeter,
                                   ExtrusionFlow(0.08, 0.4f, 0.2f));
    HybridSurfacePlannerOptions options;
    options.sample_spacing_mm = 0.5;
    options.section_start_z = 2.5;
    options.section_top_z = 3.0;
    options.max_z_offset_mm = 0.2;
    std::vector<ExtrusionPath3D> paths;
    const auto clear = [](const Vec3d &, const Vec3d &) { return true; };

    REQUIRE(plan_hybrid_surface_extrusion_paths(
        source, transform, options, 3.0, attributes, clear, paths));
    REQUIRE_FALSE(paths.empty());
    for (const ExtrusionPath3D &path : paths) {
        REQUIRE(path.polyline.size() >= 2);
        CHECK(unscale(path.polyline.get_point(0).x()) >= 10.0);
        CHECK(unscale(path.polyline.get_point(0).y()) >= -3.0);
        CHECK(unscale(path.z_offsets.front()) == Approx(0.0));
    }
}


TEST_CASE("Hybrid swept-tool validator rejects open meshes and invalid envelopes",
          "[HybridNonplanarCollision]")
{
    indexed_triangle_set open_mesh = make_flat_surface();
    AABBMesh open_aabb(open_mesh);
    HybridSweptToolOptions options;
    HybridSweptToolCollisionValidator open_validator(open_mesh, open_aabb, options);
    REQUIRE_FALSE(open_validator.valid());

    const indexed_triangle_set closed_mesh = make_closed_cube();
    AABBMesh closed_aabb(closed_mesh);
    options.toolhead_start_mm = options.nozzle_shaft_top_mm - 1.0;
    HybridSweptToolCollisionValidator invalid_validator(closed_mesh, closed_aabb, options);
    REQUIRE_FALSE(invalid_validator.valid());
}

TEST_CASE("Hybrid swept-tool validator rejects a path whose tool envelope approaches a closed model",
          "[HybridNonplanarCollision]")
{
    const indexed_triangle_set closed_mesh = make_closed_cube();
    AABBMesh aabb(closed_mesh);
    HybridSweptToolOptions options;
    options.path_sample_step_mm = 0.1;
    options.nozzle_tip_exclusion_mm = 0.2;
    options.nozzle_shaft_top_mm = 2.0;
    options.toolhead_start_mm = 2.0;
    options.toolhead_top_mm = 4.0;
    options.nozzle_shaft_radius_mm = 0.4;
    options.toolhead_radius_mm = 1.0;
    options.required_clearance_mm = 0.1;
    HybridSweptToolCollisionValidator validator(closed_mesh, aabb, options);
    REQUIRE(validator.valid());
    REQUIRE_FALSE(validator.segment_is_clear(Vec3d(0.5, 0.5, 1.0),
                                             Vec3d(0.6, 0.5, 1.0)));
}


TEST_CASE("Hybrid swept-tool validator accepts a closed outward-oriented mesh",
          "[HybridNonplanarCollision]")
{
    const indexed_triangle_set mesh = make_closed_cube();
    AABBMesh aabb(mesh);
    HybridSweptToolOptions options;
    HybridSweptToolCollisionValidator validator(mesh, aabb, options);

    REQUIRE(validator.valid());
}

TEST_CASE("Hybrid swept-tool validator rejects open meshes and invalid sampling step",
          "[HybridNonplanarCollision]")
{
    const indexed_triangle_set open_mesh = make_flat_surface();
    AABBMesh open_aabb(open_mesh);
    HybridSweptToolOptions options;
    HybridSweptToolCollisionValidator open_validator(open_mesh, open_aabb, options);
    REQUIRE_FALSE(open_validator.valid());

    const indexed_triangle_set closed_mesh = make_closed_cube();
    AABBMesh closed_aabb(closed_mesh);
    options.path_sample_step_mm = 0.0;
    HybridSweptToolCollisionValidator invalid_options_validator(closed_mesh, closed_aabb, options);
    REQUIRE_FALSE(invalid_options_validator.valid());
}

TEST_CASE("Hybrid swept-tool validator rejects a sampled shaft sweep through the model",
          "[HybridNonplanarCollision]")
{
    const indexed_triangle_set mesh = make_closed_cube();
    AABBMesh aabb(mesh);
    HybridSweptToolOptions options;
    HybridSweptToolCollisionValidator validator(mesh, aabb, options);
    REQUIRE(validator.valid());

    CHECK_FALSE(validator.segment_is_clear(Vec3d(0.25, 0.5, -0.5),
                                           Vec3d(0.75, 0.5, -0.5)));
    CHECK(validator.segment_is_clear(Vec3d(0.25, 0.5, 2.0),
                                     Vec3d(0.75, 0.5, 2.0)));
}


TEST_CASE("Hybrid swept-tool validator rejects overlapping envelope height definitions",
          "[HybridNonplanarCollision]")
{
    const indexed_triangle_set mesh = make_closed_cube();
    AABBMesh aabb(mesh);
    HybridSweptToolOptions options;
    options.nozzle_shaft_top_mm = 10.0;
    options.toolhead_start_mm = 9.0;
    HybridSweptToolCollisionValidator validator(mesh, aabb, options);
    REQUIRE_FALSE(validator.valid());

    options.nozzle_shaft_top_mm = 8.0;
    options.toolhead_start_mm = 8.0;
    options.path_sample_step_mm = 2.1;
    HybridSweptToolCollisionValidator excessive_step_validator(mesh, aabb, options);
    REQUIRE_FALSE(excessive_step_validator.valid());
}


TEST_CASE("Hybrid swept-tool validator rejects inward-oriented closed meshes",
          "[HybridNonplanarCollision]")
{
    indexed_triangle_set mesh = make_closed_cube();
    for (Vec3i32 &face : mesh.indices)
        std::swap(face(1), face(2));
    AABBMesh aabb(mesh);
    HybridSweptToolOptions options;
    HybridSweptToolCollisionValidator validator(mesh, aabb, options);
    REQUIRE_FALSE(validator.valid());
}

TEST_CASE("Hybrid swept-tool validator rejects non-finite and excessively long segments",
          "[HybridNonplanarCollision]")
{
    const indexed_triangle_set mesh = make_closed_cube();
    AABBMesh aabb(mesh);
    HybridSweptToolOptions options;
    HybridSweptToolCollisionValidator validator(mesh, aabb, options);
    REQUIRE(validator.valid());

    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(validator.segment_is_clear(Vec3d(nan, 0.5, 2.0),
                                           Vec3d(0.5, 0.5, 2.0)));
    CHECK_FALSE(validator.segment_is_clear(Vec3d(-1.0e7, 0.5, 2.0),
                                           Vec3d(1.0e7, 0.5, 2.0)));
}

TEST_CASE("Hybrid non-planar approach validates exact XYZ route from live writer position",
          "[HybridNonplanarEmission]")
{
    HybridNonplanarApproach approach;
    approach.expected_start = Vec3d(10.0, 10.0, 2.0);
    approach.extrusion_start = Vec3d(20.0, 10.0, 2.4);
    approach.clearance_z_mm = 8.0;
    approach.points = {
        Vec3d(10.0, 10.0, 2.0),
        Vec3d(10.0, 10.0, 8.0),
        Vec3d(20.0, 10.0, 8.0),
        Vec3d(20.0, 10.0, 2.4)
    };
    std::vector<std::pair<Vec3d, Vec3d>> checked;
    const auto clear = [&checked](const Vec3d &a, const Vec3d &b) {
        checked.emplace_back(a, b);
        return true;
    };

    REQUIRE(validate_hybrid_nonplanar_approach(
        approach, Vec3d(10.0, 10.0, 2.0), Vec3d(20.0, 10.0, 2.4), clear));
    REQUIRE(checked.size() == approach.points.size() - 1);
    CHECK(checked.front().first == approach.points.front());
    CHECK(checked.back().second == approach.points.back());
}

TEST_CASE("Hybrid non-planar approach rejects writer-position drift and unsafe segments",
          "[HybridNonplanarEmission]")
{
    HybridNonplanarApproach approach;
    approach.expected_start = Vec3d(1.0, 2.0, 3.0);
    approach.extrusion_start = Vec3d(5.0, 2.0, 3.5);
    approach.clearance_z_mm = 7.0;
    approach.points = {
        Vec3d(1.0, 2.0, 3.0), Vec3d(1.0, 2.0, 7.0),
        Vec3d(5.0, 2.0, 7.0), Vec3d(5.0, 2.0, 3.5)
    };
    const auto clear = [](const Vec3d &, const Vec3d &) { return true; };
    CHECK_FALSE(validate_hybrid_nonplanar_approach(
        approach, Vec3d(1.0, 2.0, 3.2), Vec3d(5.0, 2.0, 3.5), clear));

    const auto colliding = [](const Vec3d &a, const Vec3d &b) {
        return !(a.z() == 7.0 && b.z() == 7.0);
    };
    CHECK_FALSE(validate_hybrid_nonplanar_approach(
        approach, Vec3d(1.0, 2.0, 3.0), Vec3d(5.0, 2.0, 3.5), colliding));
}

TEST_CASE("Hybrid non-planar approach rejects missing clearance and malformed endpoints",
          "[HybridNonplanarEmission]")
{
    HybridNonplanarApproach approach;
    approach.expected_start = Vec3d(0.0, 0.0, 1.0);
    approach.extrusion_start = Vec3d(2.0, 0.0, 1.5);
    approach.clearance_z_mm = 5.0;
    approach.points = {
        Vec3d(0.0, 0.0, 1.0), Vec3d(1.0, 0.0, 2.0),
        Vec3d(2.0, 0.0, 1.5)
    };
    const auto clear = [](const Vec3d &, const Vec3d &) { return true; };
    CHECK_FALSE(validate_hybrid_nonplanar_approach(
        approach, Vec3d(0.0, 0.0, 1.0), Vec3d(2.0, 0.0, 1.5), clear));
    approach.points.back().x() = 3.0;
    approach.points[1].z() = 5.0;
    CHECK_FALSE(validate_hybrid_nonplanar_approach(
        approach, Vec3d(0.0, 0.0, 1.0), Vec3d(2.0, 0.0, 1.5), clear));
}


TEST_CASE("Hybrid non-planar complete pass route is validated before emission",
          "[HybridNonplanarEmission]")
{
    HybridNonplanarPassRoute route;
    route.approach.expected_start = Vec3d(0.0, 0.0, 2.0);
    route.approach.extrusion_start = Vec3d(4.0, 0.0, 2.5);
    route.approach.clearance_z_mm = 8.0;
    route.approach.points = {
        Vec3d(0.0, 0.0, 2.0), Vec3d(0.0, 0.0, 8.0),
        Vec3d(4.0, 0.0, 8.0), Vec3d(4.0, 0.0, 2.5)
    };
    route.extrusion_points = {
        Vec3d(4.0, 0.0, 2.5), Vec3d(6.0, 0.0, 2.8),
        Vec3d(8.0, 0.0, 2.5)
    };
    route.departure_points = {
        Vec3d(8.0, 0.0, 2.5), Vec3d(8.0, 0.0, 8.0)
    };
    route.return_points = {
        Vec3d(8.0, 0.0, 8.0), Vec3d(0.0, 0.0, 8.0),
        Vec3d(0.0, 0.0, 2.0)
    };

    std::vector<std::pair<Vec3d, Vec3d>> checked;
    const auto clear = [&checked](const Vec3d &a, const Vec3d &b) {
        checked.emplace_back(a, b);
        return true;
    };
    REQUIRE(validate_hybrid_nonplanar_pass_route(
        route, Vec3d(0.0, 0.0, 2.0), clear));
    CHECK(checked.size() == 4 + 2 + 1 + 2);
    CHECK(checked.front().first == route.approach.points.front());
    CHECK(checked.back().second == route.return_points.back());
}

TEST_CASE("Hybrid non-planar complete pass route rejects unsafe departure and return",
          "[HybridNonplanarEmission]")
{
    HybridNonplanarPassRoute route;
    route.approach.expected_start = Vec3d(0.0, 0.0, 2.0);
    route.approach.extrusion_start = Vec3d(4.0, 0.0, 2.5);
    route.approach.clearance_z_mm = 8.0;
    route.approach.points = {
        Vec3d(0.0, 0.0, 2.0), Vec3d(0.0, 0.0, 8.0),
        Vec3d(4.0, 0.0, 8.0), Vec3d(4.0, 0.0, 2.5)
    };
    route.extrusion_points = {
        Vec3d(4.0, 0.0, 2.5), Vec3d(6.0, 0.0, 2.8)
    };
    route.departure_points = {
        Vec3d(6.0, 0.0, 2.8), Vec3d(6.0, 0.0, 8.0)
    };
    route.return_points = {
        Vec3d(6.0, 0.0, 8.0), Vec3d(0.0, 0.0, 8.0),
        Vec3d(0.0, 0.0, 2.0)
    };

    const auto colliding_departure = [](const Vec3d &a, const Vec3d &b) {
        return !(a.z() < 8.0 && b.z() == 8.0 && a.x() == 6.0);
    };
    CHECK_FALSE(validate_hybrid_nonplanar_pass_route(
        route, Vec3d(0.0, 0.0, 2.0), colliding_departure));

    const auto colliding_return = [](const Vec3d &a, const Vec3d &b) {
        return !(a.x() == 0.0 && b.x() == 0.0 && a.z() == 8.0 && b.z() == 2.0);
    };
    CHECK_FALSE(validate_hybrid_nonplanar_pass_route(
        route, Vec3d(0.0, 0.0, 2.0), colliding_return));

    route.return_points.front().x() = 7.0;
    const auto clear = [](const Vec3d &, const Vec3d &) { return true; };
    CHECK_FALSE(validate_hybrid_nonplanar_pass_route(
        route, Vec3d(0.0, 0.0, 2.0), clear));
}
