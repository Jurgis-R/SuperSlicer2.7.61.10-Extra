///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Plugins_SeamPlacer_FixedDirectionSeamPlacer_hpp_
#define slic3r_Plugins_SeamPlacer_FixedDirectionSeamPlacer_hpp_

#include "libslic3r/Api/plugin/c/slic3r_plugin_types.h"

namespace slic3r_api { namespace SeamPlacement { namespace FixedDirectionPlugin {

/*
Register the process-wide fixed-direction provider with the orchestrator.

Built-in discovery calls this once before activation is restored. Registration
publishes plugin metadata and callbacks; it does not create a per-plan session.
*/
void register_fixed_direction_seam_placer_plugin(orchestrator_handle *orchestrator);

}}} // namespace slic3r_api::SeamPlacement::FixedDirectionPlugin

#endif // slic3r_Plugins_SeamPlacer_FixedDirectionSeamPlacer_hpp_
