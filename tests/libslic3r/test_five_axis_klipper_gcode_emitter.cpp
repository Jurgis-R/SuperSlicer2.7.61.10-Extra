#include "libslic3r/FiveAxisKlipperGCodeEmitter.hpp"

#include <catch2/catch.hpp>

using namespace Slic3r;

static FiveAxisMachineProfile klipper_test_profile()
{
    FiveAxisMachineProfile profile;
    profile.enabled = true;
    profile.postprocessor_id = FIVE_AXIS_KLIPPER_POSTPROCESSOR_ID;
    return profile;
}

TEST_CASE("Klipper five-axis emitter writes configured GCODE_AXIS words with extrusion", "[FiveAxisKlipperGCodeEmitter]")
{
    FiveAxisMachineProfile profile = klipper_test_profile();
    std::vector<FiveAxisCommand> commands = {
        { 10.0, 20.0, 5.0, 1.0, 0.0, 0.0, 600.0 },
        { 11.0, 20.0, 5.2, 1.2, 2.0, -1.0, 540.0 }
    };
    std::string output;
    const auto result = emit_klipper_five_axis_gcode(profile, commands, output);

    REQUIRE(result.valid);
    CHECK(output.find("G1 X10.000 Y20.000 Z5.000 A0.000 B0.000 E1.000 F600.000") != std::string::npos);
    CHECK(output.find("G1 X11.000 Y20.000 Z5.200 A2.000 B-1.000 E1.200 F540.000") != std::string::npos);
}

TEST_CASE("Klipper five-axis emitter honors configured rotary axis letters", "[FiveAxisKlipperGCodeEmitter]")
{
    FiveAxisMachineProfile profile = klipper_test_profile();
    profile.output.rotary_1_word = 'U';
    profile.output.rotary_2_word = 'V';
    std::vector<FiveAxisCommand> commands = {
        { 10.0, 20.0, 5.0, 1.0, 0.0, 0.0, 600.0 },
        { 11.0, 20.0, 5.0, 1.1, 1.0, 2.0, 600.0 }
    };
    std::string output;
    const auto result = emit_klipper_five_axis_gcode(profile, commands, output);

    REQUIRE(result.valid);
    CHECK(output.find(" U1.000 V2.000 ") != std::string::npos);
    CHECK(output.find(" A1.000 B2.000 ") == std::string::npos);
}

TEST_CASE("Klipper five-axis emitter fails closed and clears output on invalid profile or path", "[FiveAxisKlipperGCodeEmitter]")
{
    std::string output = "stale";
    FiveAxisMachineProfile profile = klipper_test_profile();
    profile.postprocessor_id = "other-controller";
    std::vector<FiveAxisCommand> commands = {
        { 10.0, 20.0, 5.0, 1.0, 0.0, 0.0, 600.0 },
        { 11.0, 20.0, 5.0, 1.1, 1.0, 2.0, 600.0 }
    };
    auto result = emit_klipper_five_axis_gcode(profile, commands, output);
    CHECK_FALSE(result.valid);
    CHECK(output.empty());

    profile = klipper_test_profile();
    commands[1].x = 500.0;
    output = "stale";
    result = emit_klipper_five_axis_gcode(profile, commands, output);
    CHECK_FALSE(result.valid);
    CHECK(output.empty());
}

TEST_CASE("Klipper five-axis emitter rejects disabled mode", "[FiveAxisKlipperGCodeEmitter]")
{
    FiveAxisMachineProfile profile;
    std::vector<FiveAxisCommand> commands = {
        { 10.0, 20.0, 5.0, 1.0, 0.0, 0.0, 600.0 },
        { 11.0, 20.0, 5.0, 1.1, 1.0, 2.0, 600.0 }
    };
    std::string output = "stale";
    const auto result = emit_klipper_five_axis_gcode(profile, commands, output);
    CHECK_FALSE(result.valid);
    CHECK(output.empty());
}
