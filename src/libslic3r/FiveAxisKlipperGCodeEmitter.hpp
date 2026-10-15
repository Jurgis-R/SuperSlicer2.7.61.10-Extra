/// Copyright (c) 2026 Jurgis-R
/// SuperSlicer is released under the terms of the AGPLv3 or higher.
#ifndef slic3r_FiveAxisKlipperGCodeEmitter_hpp_
#define slic3r_FiveAxisKlipperGCodeEmitter_hpp_

#include "FiveAxisMachineProfile.hpp"

#include <string>
#include <vector>

namespace Slic3r {

// Klipper supports extra G1 axes when each rotary stepper is configured with
// MANUAL_STEPPER ... GCODE_AXIS=<letter>. This emitter only formats motion;
// it does not configure steppers, home rotary axes, or infer machine geometry.
static constexpr const char *FIVE_AXIS_KLIPPER_POSTPROCESSOR_ID = "klipper_gcode_axis";

inline FiveAxisValidationResult emit_klipper_five_axis_gcode(
    const FiveAxisMachineProfile &profile,
    const std::vector<FiveAxisCommand> &commands,
    std::string &output)
{
    output.clear();
    if (!profile.enabled)
        return { false, "Klipper five-axis output requires an enabled machine profile." };
    if (profile.postprocessor_id != FIVE_AXIS_KLIPPER_POSTPROCESSOR_ID)
        return { false, "Five-axis profile does not select the Klipper GCODE_AXIS postprocessor." };

    const FiveAxisValidationResult profile_result = validate_five_axis_machine_profile(profile);
    if (!profile_result.valid)
        return profile_result;

    const FiveAxisValidationResult envelope_result =
        validate_five_axis_machine_envelope(profile, commands);
    if (!envelope_result.valid)
        return envelope_result;

    std::string candidate;
    std::string error;
    if (!emit_five_axis_linear_gcode(profile.output, commands, candidate, &error))
        return { false, error };

    output.swap(candidate);
    return { true, {} };
}

} // namespace Slic3r

#endif // slic3r_FiveAxisKlipperGCodeEmitter_hpp_
