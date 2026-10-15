#include "libslic3r/FiveAxisMachineProfile.hpp"

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
