///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef plugins_infill_concentricpattern_hpp_
#define plugins_infill_concentricpattern_hpp_

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_infill.h"
#include "libslic3r/Arachne/WallToolPaths.hpp"

/*
Concentric pattern plugin
=========================

Registration entry point of the `concentric` INFILL_PATTERN plugin, and the
surface filler it runs. The filler takes the Arachne settings as a parameter so
the parity tests can run it without a print region; they are only used by
Arachne recipes on full, adjustable surfaces.
*/

namespace slic3r_api { namespace Infill {

void register_concentric_pattern_plugin(orchestrator_handle *orch);
// Fill one surface with outside-in loops of each component, classic or Arachne walls.
void fill_concentric_surface(const run_ctx_infill_pattern &ctx, storage_handle *storage, const plugin_run_context *run_ctx,
                             const Slic3r::Arachne::WallToolPathsSettings &arachne_settings);

}} // namespace slic3r_api::Infill

#endif // plugins_infill_concentricpattern_hpp_
