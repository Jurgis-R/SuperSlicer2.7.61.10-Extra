#include "libslic3r/FiveAxisMotionPlanner.hpp"

#include <catch2/catch.hpp>

using namespace Slic3r;

static std::vector<FiveAxisToolPose> simple_poses()
{
    return {
        { {0.0, 0.0, 10.0}, {0.0, 0.0, 1.0}, 0.0, 600.0 },
        { {1.0, 0.0, 10.0}, {0.0, 0.0, 1.0}, 0.1, 600.0 },
        { {2.0, 0.0, 10.0}, {0.0, 0.0, 1.0}, 0.2, 600.0 }
    };
}

TEST_CASE("Five-axis output validates non-extruding rotary-only moves", "[FiveAxisMotionPlanner]")
{
    FiveAxisOutputConfig config;
    std::vector<FiveAxisCommand> path = {
        { 10.0, 10.0, 10.0, 0.0, 0.0, 0.0, 600.0 },
        { 10.0, 10.0, 10.0, 0.0, 15.0, 0.0, 600.0 }
    };

    CHECK(validate_five_axis_path(config, path).valid);
    path[1].e = 0.2;
    CHECK_FALSE(validate_five_axis_path(config, path).valid);
    path[1].e = 0.0;
    path[1].rotary_1_deg = 0.0;
    path[1].e = 0.2;
    CHECK_FALSE(validate_five_axis_path(config, path).valid);
    path[1].e = 0.0;
    path[1].rotary_1_deg = 15.0;

    config.max_rotary_rate_deg_s = 5.0;
    CHECK_FALSE(validate_five_axis_path(config, path).valid);
    config.max_rotary_rate_deg_s = 30.0;
    config.supports_simultaneous_rotary = false;
    path[1].rotary_1_deg = 10.0;
    path[1].rotary_2_deg = 10.0;
    CHECK_FALSE(validate_five_axis_path(config, path).valid);
    std::vector<FiveAxisCommand> sequential = {
        { 10.0, 10.0, 10.0, 0.0, 0.0, 0.0, 600.0 },
        { 10.0, 10.0, 10.0, 0.0, 10.0, 0.0, 600.0 },
        { 10.0, 10.0, 10.0, 0.0, 10.0, 10.0, 600.0 }
    };
    CHECK(validate_five_axis_path(config, sequential).valid);
}

TEST_CASE("Five-axis output limits feed to respect rotary speed", "[FiveAxisMotionPlanner]")
{
    FiveAxisOutputConfig config;
    std::vector<FiveAxisCommand> path = {
        { 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 6000.0 },
        { 1.0, 0.0, 10.0, 0.1, 10.0, 0.0, 6000.0 },
        { 2.0, 0.0, 10.0, 0.2, 20.0, 0.0, 6000.0 }
    };

    REQUIRE(limit_five_axis_rotary_feedrates(config, path).valid);
    CHECK(path[0].feed_mm_min == Approx(180.0));
    CHECK(path[1].feed_mm_min == Approx(180.0));
    CHECK(path[2].feed_mm_min == Approx(180.0));
    CHECK(validate_five_axis_path(config, path).valid);

    std::vector<FiveAxisCommand> rotary_only = {
        { 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 6000.0 },
        { 0.0, 0.0, 10.0, 0.0, 15.0, 0.0, 6000.0 }
    };
    REQUIRE(limit_five_axis_rotary_feedrates(config, rotary_only).valid);
    CHECK(rotary_only[0].feed_mm_min == Approx(1800.0));
    CHECK(rotary_only[1].feed_mm_min == Approx(1800.0));
    CHECK(validate_five_axis_path(config, rotary_only).valid);
}

TEST_CASE("Five-axis planner maps work poses to machine commands", "[FiveAxisMotionPlanner]")
{
    FiveAxisKinematicsConfig kin;
    FiveAxisOutputConfig output;
    FiveAxisCollisionConfig collision;
    std::vector<FiveAxisCommand> commands;

    const auto result = plan_five_axis_path(kin, output, collision, simple_poses(), commands);
    REQUIRE(result.valid);
    REQUIRE(commands.size() == 3);
    REQUIRE(commands[0].x == Approx(0.0));
    REQUIRE(commands[2].x == Approx(2.0));
    REQUIRE(commands[2].e == Approx(0.2));
    REQUIRE(commands[2].feed_mm_min == Approx(600.0));
}

TEST_CASE("Five-axis planner applies active tool offsets after rotary transforms", "[FiveAxisMotionPlanner]")
{
    FiveAxisKinematicsConfig kin_with_offset;
    kin_with_offset.machine_frame_tool_offset = { -2.0, 3.0, 0.0 };
    FiveAxisKinematicsConfig kin_without_offset;
    FiveAxisOutputConfig output;
    FiveAxisCollisionConfig collision;
    auto poses = simple_poses();
    for (auto &pose : poses)
        pose.normal = { 0.0, 1.0, 0.0 };

    std::vector<FiveAxisCommand> with_offset;
    std::vector<FiveAxisCommand> without_offset;
    const auto offset_result =
        plan_five_axis_path(kin_with_offset, output, collision, poses, with_offset);
    const auto base_result =
        plan_five_axis_path(kin_without_offset, output, collision, poses, without_offset);

    REQUIRE(offset_result.valid);
    REQUIRE(base_result.valid);
    REQUIRE(with_offset.size() == poses.size());
    REQUIRE(without_offset.size() == poses.size());
    CHECK(with_offset.front().rotary_1_deg == Approx(90.0).margin(0.1));
    for (std::size_t i = 0; i < poses.size(); ++i) {
        CHECK(with_offset[i].x == Approx(without_offset[i].x - 2.0));
        CHECK(with_offset[i].y == Approx(without_offset[i].y + 3.0));
        CHECK(with_offset[i].z == Approx(without_offset[i].z));
        CHECK(with_offset[i].rotary_1_deg == Approx(without_offset[i].rotary_1_deg));
        CHECK(with_offset[i].rotary_2_deg == Approx(without_offset[i].rotary_2_deg));
    }
}

TEST_CASE("Five-axis planner compensates rotary-bed pivots", "[FiveAxisMotionPlanner]")
{
    FiveAxisKinematicsConfig kin;
    kin.mount = FiveAxisRotaryMount::RotaryBed;
    kin.rotary_1_pivot = { 10.0, 20.0, 30.0 };
    FiveAxisOutputConfig output;
    FiveAxisCollisionConfig collision;
    auto poses = simple_poses();
    for (auto &pose : poses)
        pose.point = { 1.0, 2.0, 3.0 };
    std::vector<FiveAxisCommand> commands;

    const auto result = plan_five_axis_path(kin, output, collision, poses, commands);
    REQUIRE(result.valid);
    REQUIRE(commands.front().x == Approx(11.0));
    REQUIRE(commands.front().y == Approx(22.0));
    REQUIRE(commands.front().z == Approx(33.0));
}

TEST_CASE("Five-axis planner fails closed when any pose is unreachable", "[FiveAxisMotionPlanner]")
{
    FiveAxisKinematicsConfig kin;
    kin.rotary_1_min_deg = -10.0;
    kin.rotary_1_max_deg = 10.0;
    kin.rotary_2_min_deg = -10.0;
    kin.rotary_2_max_deg = 10.0;
    FiveAxisOutputConfig output;
    FiveAxisCollisionConfig collision;
    std::vector<FiveAxisCommand> commands = {
        { 99.0, 99.0, 99.0, 0.0, 0.0, 0.0, 100.0 }
    };
    auto poses = simple_poses();
    for (auto &pose : poses)
        pose.normal = { 0.0, 1.0, 0.0 };

    const auto result = plan_five_axis_path(kin, output, collision, poses, commands);
    REQUIRE_FALSE(result.valid);
    REQUIRE(commands.empty());
}

TEST_CASE("Five-axis planner rejects paths intersecting configured obstacles", "[FiveAxisMotionPlanner]")
{
    FiveAxisKinematicsConfig kin;
    FiveAxisOutputConfig output;
    output.max_rotary_delta_deg_per_mm = 100.0;
    FiveAxisCollisionConfig collision;
    collision.max_sample_spacing_mm = 0.25;
    collision.obstacles.push_back({ { 4.0, -1.0, 9.0 }, { 6.0, 1.0, 11.0 } });

    auto poses = simple_poses();
    poses[0].point = { 0.0, 0.0, 10.0 };
    poses[1].point = { 5.0, 0.0, 10.0 };
    poses[2].point = { 10.0, 0.0, 10.0 };
    std::vector<FiveAxisCommand> commands;

    const auto result = plan_five_axis_path(kin, output, collision, poses, commands);
    REQUIRE_FALSE(result.valid);
    REQUIRE(commands.empty());
}

TEST_CASE("Five-axis planner validates collision-box and sampling configuration", "[FiveAxisMotionPlanner]")
{
    FiveAxisCollisionConfig collision;
    collision.max_sample_spacing_mm = 0.0;
    REQUIRE_FALSE(validate_five_axis_collision_config(collision).valid);

    collision.max_sample_spacing_mm = 1.0;
    collision.obstacles.push_back({ { 2.0, 0.0, 0.0 }, { 1.0, 1.0, 1.0 } });
    REQUIRE_FALSE(validate_five_axis_collision_config(collision).valid);
}

TEST_CASE("Five-axis planner supports rotary-head tool-tip compensation", "[FiveAxisMotionPlanner]")
{
    FiveAxisKinematicsConfig kin;
    kin.mount = FiveAxisRotaryMount::RotaryHead;
    kin.rotary_1_pivot = { 10.0, 0.0, 0.0 };
    kin.rotary_2_pivot = { 0.0, 0.0, 5.0 };
    kin.tool_tip_offset = { 0.0, 0.0, 10.0 };
    FiveAxisOutputConfig output;
    FiveAxisCollisionConfig collision;
    auto poses = simple_poses();
    for (auto &pose : poses)
        pose.point = { 100.0, 100.0, 100.0 };
    std::vector<FiveAxisCommand> commands;

    const auto result = plan_five_axis_path(kin, output, collision, poses, commands);
    REQUIRE(result.valid);
    REQUIRE(commands.front().x == Approx(90.0));
    REQUIRE(commands.front().y == Approx(100.0));
    REQUIRE(commands.front().z == Approx(90.0));
}

TEST_CASE("Five-axis collision sampling rejects excessive sampling work", "[FiveAxisMotionPlanner]")
{
    FiveAxisCollisionConfig collision;
    collision.max_sample_spacing_mm = 1e-6;
    collision.max_samples_per_segment = 10;
    std::vector<FiveAxisCommand> path = {
        { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 600.0 },
        { 1.0, 0.0, 1.0, 0.1, 0.0, 0.0, 600.0 }
    };
    REQUIRE_FALSE(validate_five_axis_path_clearance(collision, path).valid);
}
