/// Copyright (c) 2026 Jurgis-R
/// SuperSlicer is released under the terms of the AGPLv3 or higher.
#ifndef slic3r_FiveAxisPostprocessor_hpp_
#define slic3r_FiveAxisPostprocessor_hpp_

#include "FiveAxisKlipperGCodeEmitter.hpp"

#include <string>
#include <vector>

namespace Slic3r {

// Deliberately explicit allow-list: a profile never falls back to a generic
// emitter. Add a controller here only after its output contract has tests.
inline bool is_five_axis_postprocessor_registered(const std::string &identifier)
{
    return identifier == FIVE_AXIS_KLIPPER_POSTPROCESSOR_ID;
}

// Dispatches only from the postprocessor identifier stored in the selected
// profile. GCodeGenerator must still provide a fully planned, preflighted path
// and preserve its writer/retraction state before this function can be wired
// into live export.
inline FiveAxisValidationResult emit_five_axis_gcode_for_profile(
    const FiveAxisMachineProfile &profile,
    const std::vector<FiveAxisCommand> &commands,
    std::string &output)
{
    output.clear();
    if (!profile.enabled)
        return { false, "Five-axis output requires an enabled machine profile." };
    if (!is_five_axis_postprocessor_registered(profile.postprocessor_id))
        return { false, "The selected five-axis postprocessor is not registered." };

    if (profile.postprocessor_id == FIVE_AXIS_KLIPPER_POSTPROCESSOR_ID)
        return emit_klipper_five_axis_gcode(profile, commands, output);

    // Keep this defensive return if the registry grows in the future.
    return { false, "No emitter is registered for the selected five-axis postprocessor." };
}

} // namespace Slic3r

#endif // slic3r_FiveAxisPostprocessor_hpp_
