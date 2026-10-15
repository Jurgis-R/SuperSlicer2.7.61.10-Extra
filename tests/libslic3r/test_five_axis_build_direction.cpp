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

TEST_CASE("Five-axis build-direction candidate generation is deterministic and covers cardinal axes", "[FiveAxisBuildDirection]")
{
    const auto candidates = generate_five_axis_build_direction_candidates(18);
    REQUIRE(candidates.size() == 18);
    CHECK(candidates[0].x == 1.0);
    CHECK(candidates[1].x == -1.0);
    CHECK(candidates[2].y == 1.0);
    CHECK(candidates[3].y == -1.0);
    CHECK(candidates[4].z == 1.0);
    CHECK(candidates[5].z == -1.0);

    for (const FiveAxisVec3 &candidate : candidates) {
        CHECK(five_axis_finite(candidate));
        const double length = std::sqrt(candidate.x * candidate.x +
                                        candidate.y * candidate.y +
                                        candidate.z * candidate.z);
        CHECK(length == Approx(1.0).margin(1e-9));
    }
    CHECK(generate_five_axis_build_direction_candidates(18).size() == candidates.size());
}

TEST_CASE("Automatic build-direction candidates can be scored without a manually supplied list", "[FiveAxisBuildDirection]")
{
    const indexed_triangle_set mesh = make_direction_test_mesh();
    const auto candidates = generate_five_axis_build_direction_candidates();
    REQUIRE(candidates.size() >= 6);
    FiveAxisBuildDirectionConfig config;
    config.height_penalty_weight = 0.0;
    const auto result = choose_five_axis_build_direction(mesh, candidates, config);
    REQUIRE(result.valid);
    CHECK(std::isfinite(result.score));
}

TEST_CASE("Five-axis build-direction planner generates and scores candidates in one call", "[FiveAxisBuildDirection]")
{
    const indexed_triangle_set mesh = make_direction_test_mesh();
    FiveAxisBuildDirectionConfig config;
    config.height_penalty_weight = 0.0;
    const auto result = choose_five_axis_build_direction(mesh, config, 24);
    REQUIRE(result.valid);
    CHECK(std::isfinite(result.score));
    CHECK(result.projected_height_mm > 0.0);
}

 
TEST_CASE("Machine-aware orientation filter rejects unreachable rotary poses", "[FiveAxisBuildDirection]")
{
    FiveAxisMachineProfile profile;
    profile.enabled = true;
    profile.postprocessor_id = "test-post";
    profile.kinematics.rotary_1_min_deg = -10.0;
    profile.kinematics.rotary_1_max_deg = 10.0;
    profile.kinematics.rotary_2_min_deg = -10.0;
    profile.kinematics.rotary_2_max_deg = 10.0;
    profile.output.rotary_1_min_deg = -10.0;
    profile.output.rotary_1_max_deg = 10.0;
    profile.output.rotary_2_min_deg = -10.0;
    profile.output.rotary_2_max_deg = 10.0;

    std::vector<FiveAxisBuildDirectionCandidate> candidates(2);
    candidates[0].valid = true;
    candidates[0].build_up = {0.0, 0.0, 1.0};
    candidates[0].score = 0.1;
    candidates[1].valid = true;
    candidates[1].build_up = {0.0, 1.0, 0.0};
    candidates[1].score = 0.2;

    const std::vector<std::vector<FiveAxisToolPose>> poses = {
        { {{10.0, 10.0, 10.0}, {0.0, 0.0, 1.0}, 0.0, 600.0},
          {{11.0, 10.0, 10.0}, {0.0, 0.0, 1.0}, 0.1, 600.0} },
        { {{10.0, 10.0, 10.0}, {0.0, 1.0, 0.0}, 0.0, 600.0},
          {{11.0, 10.0, 10.0}, {0.0, 1.0, 0.0}, 0.1, 600.0} }
    };
    const auto filtered = filter_five_axis_build_direction_candidates(candidates, poses, profile);
    REQUIRE(filtered.size() == 2);
    CHECK(filtered[0].valid);
    CHECK_FALSE(filtered[1].valid);
    CHECK(filtered[1].error.find("Machine feasibility rejected") != std::string::npos);
}

TEST_CASE("Machine-aware orientation filter rejects candidates outside XYZ envelope", "[FiveAxisBuildDirection]")
{
    FiveAxisMachineProfile profile;
    profile.enabled = true;
    profile.postprocessor_id = "test-post";
    std::vector<FiveAxisBuildDirectionCandidate> candidates(1);
    candidates[0].valid = true;
    candidates[0].build_up = {0.0, 0.0, 1.0};
    const std::vector<std::vector<FiveAxisToolPose>> poses = {
        { {{300.0, 10.0, 10.0}, {0.0, 0.0, 1.0}, 0.0, 600.0},
          {{301.0, 10.0, 10.0}, {0.0, 0.0, 1.0}, 0.1, 600.0} }
    };
    const auto filtered = filter_five_axis_build_direction_candidates(candidates, poses, profile);
    REQUIRE(filtered.size() == 1);
    CHECK_FALSE(filtered[0].valid);
    CHECK(filtered[0].error.find("XYZ envelope rejected") != std::string::npos);
}

TEST_CASE("Machine-aware orientation filter fails closed for mismatched candidate pose sets", "[FiveAxisBuildDirection]")
{
    FiveAxisMachineProfile profile;
    profile.enabled = true;
    profile.postprocessor_id = "test-post";
    std::vector<FiveAxisBuildDirectionCandidate> candidates(1);
    candidates[0].valid = true;
    const auto filtered = filter_five_axis_build_direction_candidates(candidates, {}, profile);
    REQUIRE(filtered.size() == 1);
    CHECK_FALSE(filtered[0].valid);
    CHECK(filtered[0].error.find("corresponding candidate-specific") != std::string::npos);
}

TEST_CASE("Machine-aware orientation filter applies configured obstacle clearance", "[FiveAxisBuildDirection]")
{
    FiveAxisMachineProfile profile;
    profile.enabled = true;
    profile.postprocessor_id = "test-post";
    profile.collision.obstacles.push_back({ {10.0, 9.0, 9.0}, {12.0, 11.0, 11.0} });
    std::vector<FiveAxisBuildDirectionCandidate> candidates(1);
    candidates[0].valid = true;
    candidates[0].build_up = {0.0, 0.0, 1.0};
    const std::vector<std::vector<FiveAxisToolPose>> poses = {
        { {{9.0, 10.0, 10.0}, {0.0, 0.0, 1.0}, 0.0, 600.0},
          {{13.0, 10.0, 10.0}, {0.0, 0.0, 1.0}, 0.1, 600.0} }
    };
    const auto filtered = filter_five_axis_build_direction_candidates(candidates, poses, profile);
    REQUIRE(filtered.size() == 1);
    CHECK_FALSE(filtered[0].valid);
    CHECK(filtered[0].error.find("Machine feasibility rejected") != std::string::npos);
}

TEST_CASE("Machine-aware orientation filter refuses a disabled profile", "[FiveAxisBuildDirection]")
{
    FiveAxisMachineProfile profile;
    std::vector<FiveAxisBuildDirectionCandidate> candidates(1);
    candidates[0].valid = true;
    const std::vector<std::vector<FiveAxisToolPose>> poses = {
        { {{1.0, 1.0, 10.0}, {0.0, 0.0, 1.0}, 0.0, 600.0},
          {{2.0, 1.0, 10.0}, {0.0, 0.0, 1.0}, 0.1, 600.0} }
    };
    const auto filtered = filter_five_axis_build_direction_candidates(candidates, poses, profile);
    REQUIRE(filtered.size() == 1);
    CHECK_FALSE(filtered[0].valid);
    CHECK(filtered[0].error.find("enabled five-axis profile") != std::string::npos);
}
