#include "libslic3r/FiveAxisBuildDirectionPlanner.hpp"

#include <catch2/catch.hpp>

using namespace Slic3r;

static indexed_triangle_set make_direction_test_mesh()
{
    indexed_triangle_set mesh;
    mesh.vertices = {
        Vec3f(0.f, 0.f, 0.f), Vec3f(10.f, 0.f, 0.f),
        Vec3f(0.f, 10.f, 0.f), Vec3f(0.f, 0.f, 10.f)
    };
    // A small tetrahedron, with valid indices and non-zero face areas.
    mesh.indices = { Vec3i(0, 2, 1), Vec3i(0, 1, 3),
                     Vec3i(1, 2, 3), Vec3i(2, 0, 3) };
    return mesh;
}

TEST_CASE("Five-axis build-direction scorer validates input and returns finite metrics", "[FiveAxisBuildDirection]")
{
    const indexed_triangle_set mesh = make_direction_test_mesh();
    const auto result = score_five_axis_build_direction(mesh, {0.0, 0.0, 1.0});
    REQUIRE(result.valid);
    CHECK(result.unsupported_area_ratio >= 0.0);
    CHECK(result.unsupported_area_ratio <= 1.0);
    CHECK(result.projected_height_mm > 0.0);
    CHECK(std::isfinite(result.score));
    CHECK_FALSE(score_five_axis_build_direction(mesh, {0.0, 0.0, 0.0}).valid);
}

TEST_CASE("Five-axis build-direction chooser selects a valid candidate", "[FiveAxisBuildDirection]")
{
    const indexed_triangle_set mesh = make_direction_test_mesh();
    FiveAxisBuildDirectionConfig config;
    config.height_penalty_weight = 0.0;
    const auto result = choose_five_axis_build_direction(
        mesh, {{0.0, 0.0, 1.0}, {0.0, 0.0, -1.0}}, config);
    REQUIRE(result.valid);
    CHECK(result.score >= 0.0);
}

TEST_CASE("Five-axis build-direction scorer rejects invalid mesh indices", "[FiveAxisBuildDirection]")
{
    indexed_triangle_set mesh = make_direction_test_mesh();
    mesh.indices[0].x() = 999;
    CHECK_FALSE(score_five_axis_build_direction(mesh, {0.0, 0.0, 1.0}).valid);
}
