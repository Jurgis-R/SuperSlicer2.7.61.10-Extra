///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef plugins_infill_smoothpattern_hpp_
#define plugins_infill_smoothpattern_hpp_

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_infill.h"

/*
Smooth (Ironing) pattern plugin
===============================

Registration entry point of the `smooth` INFILL_PATTERN plugin, and the surface
filler it runs with the region settings. The filler takes the settings as a
parameter so the parity tests can run it without a print region.
*/

namespace slic3r_api { namespace Infill {

// Region settings of the smooth pattern, with the option defaults.
struct SmoothSettings
{
    // fill_smooth_width: ironing line spacing as a fraction of the recipe spacing.
    double width = 0.5;
    // fill_smooth_distribution: share of the flow given to the ironing pass.
    double distribution = 0.1;
};

void register_smooth_pattern_plugin(orchestrator_handle *orch);
// Fill one surface with the deposit and ironing passes; exposed for the parity tests.
void fill_smooth_surface(const run_ctx_infill_pattern &ctx, storage_handle *storage, const plugin_run_context *run_ctx,
                         const SmoothSettings &settings);

}} // namespace slic3r_api::Infill

#endif // plugins_infill_smoothpattern_hpp_
