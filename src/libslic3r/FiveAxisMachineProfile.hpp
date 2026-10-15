/// Copyright (c) 2026 Jurgis-R
/// SuperSlicer is released under the terms of the AGPLv3 or higher.
#ifndef slic3r_FiveAxisMachineProfile_hpp_
#define slic3r_FiveAxisMachineProfile_hpp_

#include "FiveAxisMotionPlanner.hpp"

#include <cmath>
#include <string>

namespace Slic3r {

// Typed, opt-in machine profile for the experimental five-axis pipeline.
// This is the validated engine-side schema; persistence and UI registration
// in PrintConfig are a separate integration step.
struct FiveAxisMachineProfile {
    bool enabled { false };
    std::string name { "Unconfigured five-axis machine" };
    // Stable identifier selected by the exporter; no generic postprocessor
    // is assumed safe for every controller.
    std::string postprocessor_id;

    FiveAxisKinematicsConfig kinematics;
    FiveAxisOutputConfig output;
    FiveAxisCollisionConfig collision;

    // Conservative bounds for the XYZ carriage/tool reference in machine
    // coordinates. The current planner does not yet enforce these bounds.
    FiveAxisVec3 machine_min { 0.0, 0.0, 0.0 };
    FiveAxisVec3 machine_max { 250.0, 250.0, 250.0 };
};

inline FiveAxisValidationResult validate_five_axis_machine_profile(
    const FiveAxisMachineProfile &profile)
{
    if (profile.name.empty())
        return { false, "Five-axis machine profile needs a name." };
    if (profile.enabled && profile.postprocessor_id.empty())
        return { false, "Enabled five-axis profiles require an explicit postprocessor identifier." };

    std::string error;
    if (!validate_five_axis_kinematics_config(profile.kinematics, &error))
        return { false, "Invalid five-axis kinematics: " + error };
    const auto output_validation = validate_five_axis_output_config(profile.output);
    if (!output_validation.valid)
        return output_validation;
    const auto collision_validation = validate_five_axis_collision_config(profile.collision);
    if (!collision_validation.valid)
        return collision_validation;

    if (profile.output.rotary_1_min_deg != profile.kinematics.rotary_1_min_deg ||
        profile.output.rotary_1_max_deg != profile.kinematics.rotary_1_max_deg ||
        profile.output.rotary_2_min_deg != profile.kinematics.rotary_2_min_deg ||
        profile.output.rotary_2_max_deg != profile.kinematics.rotary_2_max_deg)
        return { false, "Kinematics and output rotary-axis limits must match exactly." };

    if (!five_axis_finite(profile.machine_min) || !five_axis_finite(profile.machine_max) ||
        profile.machine_min.x >= profile.machine_max.x ||
        profile.machine_min.y >= profile.machine_max.y ||
        profile.machine_min.z >= profile.machine_max.z)
        return { false, "Machine XYZ envelope must contain finite increasing bounds." };

    return { true, {} };
}

} // namespace Slic3r

#endif // slic3r_FiveAxisMachineProfile_hpp_
