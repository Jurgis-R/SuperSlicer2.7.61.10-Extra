#include "libslic3r/HybridNonplanarSurfacePlanner.hpp"

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
