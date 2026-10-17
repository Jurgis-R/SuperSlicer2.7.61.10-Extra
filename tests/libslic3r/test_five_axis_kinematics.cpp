#include "libslic3r/FiveAxisKinematics.hpp"

#include <catch2/catch.hpp>

using namespace Slic3r;

TEST_CASE("Five-axis orientation IK solves rotary-bed and rotary-head normals",
          "[FiveAxisKinematics]")
{
    FiveAxisKinematicsConfig bed;
    bed.mount = FiveAxisRotaryMount::RotaryBed;
    const FiveAxisJointSolution bed_solution =
        solve_five_axis_orientation(bed, FiveAxisVec3{0.0, 1.0, 0.0});
    REQUIRE(bed_solution.valid);
    REQUIRE(bed_solution.rotary_1_deg == Approx(90.0).margin(0.1));
    REQUIRE(bed_solution.orientation_error <= bed.orientation_tolerance);

    FiveAxisKinematicsConfig head;
    head.mount = FiveAxisRotaryMount::RotaryHead;
    const FiveAxisJointSolution head_solution =
        solve_five_axis_orientation(head, FiveAxisVec3{0.0, 1.0, 0.0});
    REQUIRE(head_solution.valid);
    REQUIRE(head_solution.rotary_1_deg == Approx(-90.0).margin(0.1));
    REQUIRE(head_solution.orientation_error <= head.orientation_tolerance);
}

TEST_CASE("Five-axis kinematics rejects unreachable normals and invalid axes",
          "[FiveAxisKinematics]")
{
    FiveAxisKinematicsConfig config;
    config.rotary_1_axis = FiveAxisVec3{0.0, 0.0, 0.0};
    REQUIRE_FALSE(validate_five_axis_kinematics_config(config));

    config = FiveAxisKinematicsConfig{};
    config.rotary_1_min_deg = 0.0;
    config.rotary_1_max_deg = 0.0;
    REQUIRE_FALSE(validate_five_axis_kinematics_config(config));

    config = FiveAxisKinematicsConfig{};
    config.rotary_1_min_deg = -10.0;
    config.rotary_1_max_deg = 10.0;
    config.rotary_2_min_deg = -10.0;
    config.rotary_2_max_deg = 10.0;
    const FiveAxisJointSolution solution =
        solve_five_axis_orientation(config, FiveAxisVec3{0.0, 1.0, 0.0});
    REQUIRE_FALSE(solution.valid);
    REQUIRE_FALSE(solution.error.empty());
}

TEST_CASE("Five-axis kinematics transforms bed points and compensates rotary-head pivots",
          "[FiveAxisKinematics]")
{
    FiveAxisKinematicsConfig bed;
    bed.rotary_1_pivot = FiveAxisVec3{10.0, 20.0, 30.0};
    const FiveAxisVec3 point{1.0, 2.0, 3.0};
    const FiveAxisVec3 unchanged = five_axis_transform_bed_point(
        bed, point, 0.0, 0.0);
    REQUIRE(unchanged.x == Approx(point.x));
    REQUIRE(unchanged.y == Approx(point.y));
    REQUIRE(unchanged.z == Approx(point.z));

    // A quarter turn about the machine X axis rotates the point around the
    // configured pivot, rather than adding the pivot as a translation.
    const FiveAxisVec3 transformed = five_axis_transform_bed_point(
        bed, FiveAxisVec3{10.0, 21.0, 30.0}, 90.0, 0.0);
    REQUIRE(transformed.x == Approx(10.0).margin(1e-6));
    REQUIRE(transformed.y == Approx(20.0).margin(1e-6));
    REQUIRE(transformed.z == Approx(31.0).margin(1e-6));

    FiveAxisKinematicsConfig head;
    head.mount = FiveAxisRotaryMount::RotaryHead;
    head.rotary_1_pivot = FiveAxisVec3{10.0, 0.0, 0.0};
    head.rotary_2_pivot = FiveAxisVec3{0.0, 0.0, 5.0};
    head.tool_tip_offset = FiveAxisVec3{0.0, 0.0, 10.0};
    const FiveAxisVec3 carriage = five_axis_compensate_head_position(
        head, FiveAxisVec3{100.0, 100.0, 100.0}, 0.0, 0.0);
    REQUIRE(carriage.x == Approx(90.0));
    REQUIRE(carriage.y == Approx(100.0));
    REQUIRE(carriage.z == Approx(90.0));
}

TEST_CASE("Five-axis kinematics normalizes non-unit rotary axes",
          "[FiveAxisKinematics]")
{
    FiveAxisKinematicsConfig config;
    config.rotary_1_axis = FiveAxisVec3{2.0, 0.0, 0.0};
    config.rotary_2_axis = FiveAxisVec3{0.0, 3.0, 0.0};
    REQUIRE(validate_five_axis_kinematics_config(config));
    const FiveAxisJointSolution solution =
        solve_five_axis_orientation(config, FiveAxisVec3{0.0, 1.0, 0.0});
    REQUIRE(solution.valid);
}
