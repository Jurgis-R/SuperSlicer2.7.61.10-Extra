#include "libslic3r/FiveAxisMachineProfile.hpp"
#include "libslic3r/FiveAxisConfig.hpp"

#include <catch2/catch.hpp>

using namespace Slic3r;

TEST_CASE("Five-axis machine profile defaults to disabled and validates", "[FiveAxisMachineProfile]")
{
    FiveAxisMachineProfile profile;
    CHECK_FALSE(profile.enabled);
    CHECK(validate_five_axis_machine_profile(profile).valid);
}

TEST_CASE("Enabled five-axis machine profile requires an explicit postprocessor", "[FiveAxisMachineProfile]")
{
    FiveAxisMachineProfile profile;
    profile.enabled = true;
    CHECK_FALSE(validate_five_axis_machine_profile(profile).valid);

    profile.postprocessor_id = "test-controller";
    CHECK(validate_five_axis_machine_profile(profile).valid);
}

TEST_CASE("Five-axis machine profile rejects inconsistent axis limits and envelopes", "[FiveAxisMachineProfile]")
{
    FiveAxisMachineProfile profile;
    profile.output.rotary_1_max_deg = 120.0;
    CHECK_FALSE(validate_five_axis_machine_profile(profile).valid);

    profile.output.rotary_1_max_deg = profile.kinematics.rotary_1_max_deg;
    profile.machine_max.x = profile.machine_min.x;
    CHECK_FALSE(validate_five_axis_machine_profile(profile).valid);
}



TEST_CASE("Persisted five-axis printer settings load into the validated profile", "[FiveAxisMachineProfile]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    FiveAxisMachineProfile profile;

    const FiveAxisValidationResult default_result = load_five_axis_machine_profile(config, profile);
    REQUIRE(default_result.valid);
    CHECK_FALSE(profile.enabled);

    config.option<ConfigOptionBool>("five_axis_enabled")->value = true;
    config.opt_string("five_axis_postprocessor_id") = "test-controller";
    config.opt_string("five_axis_mount") = "rotary_head";
    config.opt_float("five_axis_machine_max_x") = 320.0;

    const FiveAxisValidationResult enabled_result = load_five_axis_machine_profile(config, profile);
    REQUIRE(enabled_result.valid);
    CHECK(profile.enabled);
    CHECK(profile.postprocessor_id == "test-controller");
    CHECK(profile.kinematics.mount == FiveAxisRotaryMount::RotaryHead);
    CHECK(profile.machine_max.x == Approx(320.0));
}

TEST_CASE("Five-axis export gate preserves disabled mode and rejects missing controller writer", "[FiveAxisMachineProfile]")
{
    FiveAxisMachineProfile profile;
    CHECK(validate_five_axis_export_gate(profile, "").valid);

    profile.enabled = true;
    profile.postprocessor_id = "example-controller";
    CHECK_FALSE(validate_five_axis_export_gate(profile, "").valid);
    CHECK_FALSE(validate_five_axis_export_gate(profile, "different-controller").valid);
    CHECK(validate_five_axis_export_gate(profile, "example-controller").valid);
}
