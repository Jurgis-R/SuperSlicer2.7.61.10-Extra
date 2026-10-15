#include "libslic3r/FiveAxisPostprocessor.hpp"

#include <catch2/catch.hpp>

using namespace Slic3r;

TEST_CASE("Five-axis postprocessor dispatcher requires an explicit registered identifier",
          "[FiveAxisPostprocessor]")
{
    FiveAxisMachineProfile profile;
    profile.enabled = true;
    std::vector<FiveAxisCommand> commands = {
        { 10.0, 20.0, 5.0, 1.0, 0.0, 0.0, 600.0 },
        { 11.0, 20.0, 5.0, 1.1, 1.0, 2.0, 600.0 }
    };
    std::string output = "stale";

    profile.postprocessor_id = "unknown_controller";
    auto result = emit_five_axis_gcode_for_profile(profile, commands, output);
    CHECK_FALSE(result.valid);
    CHECK(output.empty());
    CHECK_FALSE(is_five_axis_postprocessor_registered("unknown_controller"));

    profile.postprocessor_id = FIVE_AXIS_KLIPPER_POSTPROCESSOR_ID;
    result = emit_five_axis_gcode_for_profile(profile, commands, output);
    REQUIRE(result.valid);
    CHECK(output.find("G1 X11.000 Y20.000 Z5.000 A1.000 B2.000") != std::string::npos);
    CHECK(is_five_axis_postprocessor_registered(FIVE_AXIS_KLIPPER_POSTPROCESSOR_ID));
}

TEST_CASE("Five-axis postprocessor dispatcher rejects disabled profiles",
          "[FiveAxisPostprocessor]")
{
    FiveAxisMachineProfile profile;
    profile.postprocessor_id = FIVE_AXIS_KLIPPER_POSTPROCESSOR_ID;
    std::vector<FiveAxisCommand> commands = {
        { 10.0, 20.0, 5.0, 1.0, 0.0, 0.0, 600.0 },
        { 11.0, 20.0, 5.0, 1.1, 1.0, 2.0, 600.0 }
    };
    std::string output = "stale";
    const auto result = emit_five_axis_gcode_for_profile(profile, commands, output);
    CHECK_FALSE(result.valid);
    CHECK(output.empty());
}
