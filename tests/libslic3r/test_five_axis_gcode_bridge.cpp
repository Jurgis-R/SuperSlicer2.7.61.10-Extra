#include "libslic3r/FiveAxisGCodeBridge.hpp"

#include <catch2/catch.hpp>

using namespace Slic3r;

static ExtrusionPath3D make_bridge_test_path()
{
    ExtrusionPath3D path(ExtrusionAttributes{}, true);
    path.push_back(Point(0, 0), 0);
    path.push_back(Point(1000, 0), 0);
    path.push_back(Point(2000, 0), 0);
    path.tool_axis_normals = {
        { { 0.0, 0.0, 1.0 } },
        { { 0.0, 0.0, 1.0 } },
        { { 0.0, 0.0, 1.0 } }
    };
    return path;
}

TEST_CASE("Five-axis G-code bridge plans and emits atomically for the selected Klipper profile",
          "[FiveAxisGCodeBridge]")
{
    FiveAxisMachineProfile profile;
    profile.enabled = true;
    profile.postprocessor_id = FIVE_AXIS_KLIPPER_POSTPROCESSOR_ID;
    const ExtrusionPath3D path = make_bridge_test_path();
    std::string output = "stale";

    const auto result = plan_and_emit_five_axis_extrusion(
        profile, path, 10.0, 3.0, 0.1, 600.0, output);

    REQUIRE(result.validation.valid);
    CHECK(result.command_count == 3);
    CHECK(result.final_e == Approx(3.2));
    CHECK(output.find("G1 X2.000 Y0.000 Z10.000") != std::string::npos);
    CHECK(output.find("E3.200") != std::string::npos);
}

TEST_CASE("Five-axis G-code bridge leaves no partial output when oriented geometry is absent",
          "[FiveAxisGCodeBridge]")
{
    FiveAxisMachineProfile profile;
    profile.enabled = true;
    profile.postprocessor_id = FIVE_AXIS_KLIPPER_POSTPROCESSOR_ID;
    ExtrusionPath3D path(ExtrusionAttributes{}, true);
    path.push_back(Point(0, 0), 0);
    path.push_back(Point(1000, 0), 0);
    std::string output = "stale";

    const auto result = plan_and_emit_five_axis_extrusion(
        profile, path, 10.0, 3.0, 0.1, 600.0, output);

    CHECK_FALSE(result.validation.valid);
    CHECK(output.empty());
    CHECK(result.final_e == Approx(3.0));
    CHECK(result.command_count == 0);
}


TEST_CASE("Five-axis G-code bridge rejects incomplete per-vertex Z offsets",
          "[FiveAxisGCodeBridge]")
{
    FiveAxisMachineProfile profile;
    profile.enabled = true;
    profile.postprocessor_id = FIVE_AXIS_KLIPPER_POSTPROCESSOR_ID;
    ExtrusionPath3D path = make_bridge_test_path();
    path.z_offsets.pop_back();
    std::string output = "stale";

    const auto result = plan_and_emit_five_axis_extrusion(
        profile, path, 10.0, 3.0, 0.1, 600.0, output);

    CHECK_FALSE(result.validation.valid);
    CHECK(result.validation.message.find("Z offset") != std::string::npos);
    CHECK(output.empty());
    CHECK(result.command_count == 0);
    CHECK(result.final_e == Approx(3.0));
}
