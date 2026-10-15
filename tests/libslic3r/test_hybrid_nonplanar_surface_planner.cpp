#include "libslic3r/HybridNonplanarSurfacePlanner.hpp"
#include "libslic3r/HybridNonplanarExtrusion.hpp"

#include <catch2/catch.hpp>

using namespace Slic3r;

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
