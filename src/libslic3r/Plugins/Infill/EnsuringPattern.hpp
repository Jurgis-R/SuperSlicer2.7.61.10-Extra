///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef plugins_infill_ensuringpattern_hpp_
#define plugins_infill_ensuringpattern_hpp_

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_infill.h"
#include "libslic3r/Arachne/WallToolPaths.hpp"

/*
Ensuring pattern plugin
=======================

Registration entry point of the `ensuring` INFILL_PATTERN plugin, and the
surface filler it runs. The filler takes the Arachne settings as a parameter so
the parity tests can run it without a print region.
*/

namespace slic3r_api { namespace Infill {

void register_ensuring_pattern_plugin(orchestrator_handle *orch);
// Fill one surface with straight lines and Arachne walls in the gaps they leave.
void fill_ensuring_surface(const run_ctx_infill_pattern &ctx, storage_handle *storage, const plugin_run_context *run_ctx,
                           const Slic3r::Arachne::WallToolPathsSettings &arachne_settings);

}} // namespace slic3r_api::Infill

#endif // plugins_infill_ensuringpattern_hpp_
