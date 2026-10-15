#include "libslic3r/FiveAxisExtrusionAdapter.hpp"

#include <catch2/catch.hpp>

using namespace Slic3r;

static ExtrusionPath3D make_oriented_test_path()
{
    ExtrusionPath3D path(ExtrusionAttributes{}, true);
    path.push_back(Point(0, 0), 0);
    path.push_back(Point(1000, 0), 500);
    path.push_back(Point(2000, 0), 500);
    path.tool_axis_normals = {
        { { 0.0, 0.0, 2.0 } },
        { { 0.0, 1.0, 0.0 } },
        { { 0.0, 1.0, 0.0 } }
    };
    return path;
}

TEST_CASE("Five-axis adapter maps real 3D extrusion vertices to normalized poses", "[FiveAxisExtrusionAdapter]")
{
    const ExtrusionPath3D path = make_oriented_test_path();
    std::vector<FiveAxisToolPose> poses;
    const auto result = make_five_axis_poses_from_extrusion_path(
        path, 10.0, 5.0, 0.5, 600.0, poses);

    REQUIRE(result.valid);
    REQUIRE(poses.size() == 3);
    CHECK(poses[0].point.x == Approx(0.0));
    CHECK(poses[0].point.z == Approx(10.0));
    CHECK(poses[1].point.x == Approx(1.0));
    CHECK(poses[1].point.z == Approx(10.5));
    CHECK(poses[2].point.x == Approx(2.0));
    CHECK(poses[0].normal.z == Approx(1.0));
    CHECK(poses[1].normal.y == Approx(1.0));
    CHECK(poses[0].extrusion == Approx(5.0));
    CHECK(poses[1].extrusion == Approx(5.0 + std::sqrt(1.25) * 0.5));
    CHECK(poses[2].extrusion == Approx(poses[1].extrusion + 1.0 * 0.5));
    CHECK(poses[2].feed_mm_min == Approx(600.0));
}

TEST_CASE("Five-axis adapter rejects missing or misaligned tool directions", "[FiveAxisExtrusionAdapter]")
{
    ExtrusionPath3D path = make_oriented_test_path();
    std::vector<FiveAxisToolPose> poses;
    path.tool_axis_normals.clear();
    auto result = make_five_axis_poses_from_extrusion_path(path, 0.0, 0.0, 1.0, 600.0, poses);
    CHECK_FALSE(result.valid);
    CHECK(poses.empty());

    path = make_oriented_test_path();
    path.tool_axis_normals.pop_back();
    result = make_five_axis_poses_from_extrusion_path(path, 0.0, 0.0, 1.0, 600.0, poses);
    CHECK_FALSE(result.valid);
    CHECK(poses.empty());
}

TEST_CASE("Five-axis adapter rejects zero normals and invalid motion settings", "[FiveAxisExtrusionAdapter]")
{
    ExtrusionPath3D path = make_oriented_test_path();
    std::vector<FiveAxisToolPose> poses;
    path.tool_axis_normals[1] = { { 0.0, 0.0, 0.0 } };
    auto result = make_five_axis_poses_from_extrusion_path(path, 0.0, 0.0, 1.0, 600.0, poses);
    CHECK_FALSE(result.valid);
    CHECK(poses.empty());

    path = make_oriented_test_path();
    result = make_five_axis_poses_from_extrusion_path(path, 0.0, 0.0, 1.0, 0.0, poses);
    CHECK_FALSE(result.valid);
    CHECK(poses.empty());
}

TEST_CASE("Five-axis extrusion pipeline plans an oriented sliced path through its machine profile", "[FiveAxisExtrusionAdapter]")
{
    FiveAxisMachineProfile profile;
    profile.enabled = true;
    profile.postprocessor_id = "test-controller";
    ExtrusionPath3D path(ExtrusionAttributes{}, true);
    path.push_back(Point(0, 0), 0);
    path.push_back(Point(1000, 0), 0);
    path.push_back(Point(2000, 0), 0);
    path.tool_axis_normals = {
        { { 0.0, 0.0, 1.0 } },
        { { 0.0, 0.0, 1.0 } },
        { { 0.0, 0.0, 1.0 } }
    };
    std::vector<FiveAxisCommand> commands;
    const auto result = plan_five_axis_extrusion_path(
        profile, path, 10.0, 0.0, 0.1, 600.0, commands);

    REQUIRE(result.valid);
    REQUIRE(commands.size() == 3);
    CHECK(commands[0].x == Approx(0.0));
    CHECK(commands[1].x == Approx(1.0));
    CHECK(commands[2].x == Approx(2.0));
    CHECK(commands[2].e == Approx(0.2));
    CHECK(commands[2].rotary_1_deg == Approx(0.0));
    CHECK(commands[2].rotary_2_deg == Approx(0.0));
}

TEST_CASE("Five-axis extrusion pipeline fails closed when profile is disabled", "[FiveAxisExtrusionAdapter]")
{
    FiveAxisMachineProfile profile;
    ExtrusionPath3D path(ExtrusionAttributes{}, true);
    path.push_back(Point(0, 0), 0);
    path.push_back(Point(1000, 0), 0);
    path.tool_axis_normals = {
        { { 0.0, 0.0, 1.0 } },
        { { 0.0, 0.0, 1.0 } }
    };
    std::vector<FiveAxisCommand> commands = {
        { 1.0, 2.0, 3.0, 4.0, 0.0, 0.0, 600.0 }
    };
    const auto result = plan_five_axis_extrusion_path(
        profile, path, 0.0, 0.0, 0.1, 600.0, commands);
    CHECK_FALSE(result.valid);
    CHECK(commands.empty());
}

TEST_CASE("Five-axis extrusion pipeline rejects points outside the configured machine envelope", "[FiveAxisExtrusionAdapter]")
{
    FiveAxisMachineProfile profile;
    profile.enabled = true;
    profile.postprocessor_id = "test-controller";
    profile.machine_min = { 0.0, 0.0, 0.0 };
    profile.machine_max = { 10.0, 10.0, 10.0 };

    ExtrusionPath3D path(ExtrusionAttributes{}, true);
    path.push_back(Point(0, 0), 0);
    path.push_back(Point(20000, 0), 0); // 20 mm, beyond configured X maximum.
    path.tool_axis_normals = {
        { { 0.0, 0.0, 1.0 } },
        { { 0.0, 0.0, 1.0 } }
    };
    std::vector<FiveAxisCommand> commands;
    const auto result = plan_five_axis_extrusion_path(
        profile, path, 5.0, 0.0, 0.1, 600.0, commands);
    CHECK_FALSE(result.valid);
    CHECK(result.error.find("machine XYZ envelope") != std::string::npos);
    CHECK(commands.empty());
}
