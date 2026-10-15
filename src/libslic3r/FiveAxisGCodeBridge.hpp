/// Copyright (c) 2026 Jurgis-R
/// SuperSlicer is released under the terms of the AGPLv3 or higher.
#ifndef slic3r_FiveAxisGCodeBridge_hpp_
#define slic3r_FiveAxisGCodeBridge_hpp_

#include "FiveAxisExtrusionAdapter.hpp"
#include "FiveAxisPostprocessor.hpp"

#include <string>
#include <vector>

namespace Slic3r {

// Atomic bridge for the eventual GCodeGenerator extrusion hook. The caller
// must pass the E value after its normal retract/unretract decision and the
// same E/mm and feed values used by SuperSlicer's extrusion calculation.
// This helper does not mutate GCodeWriter or the generator's retract state.
struct FiveAxisGCodeBridgeResult {
    FiveAxisValidationResult validation;
    double final_e { 0.0 };
    std::size_t command_count { 0 };
};

inline FiveAxisGCodeBridgeResult plan_and_emit_five_axis_extrusion(
    const FiveAxisMachineProfile &profile,
    const ExtrusionPath3D &path,
    double base_z_mm,
    double initial_e,
    double e_per_mm,
    double feed_mm_min,
    std::string &output,
    double initial_rotary_1_deg = 0.0,
    double initial_rotary_2_deg = 0.0)
{
    output.clear();
    FiveAxisGCodeBridgeResult result;
    result.final_e = initial_e;

    if (!profile.enabled) {
        result.validation = { false, "Five-axis bridge requires an enabled profile." };
        return result;
    }

    std::vector<FiveAxisCommand> commands;
    result.validation = plan_five_axis_extrusion_path(
        profile, path, base_z_mm, initial_e, e_per_mm, feed_mm_min,
        commands, initial_rotary_1_deg, initial_rotary_2_deg);
    if (!result.validation.valid)
        return result;

    std::string candidate;
    result.validation = emit_five_axis_gcode_for_profile(profile, commands, candidate);
    if (!result.validation.valid)
        return result;

    // Commit output/state only after planning and emission both succeed.
    output.swap(candidate);
    result.command_count = commands.size();
    result.final_e = commands.back().e;
    return result;
}

} // namespace Slic3r

#endif // slic3r_FiveAxisGCodeBridge_hpp_
