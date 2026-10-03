///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef plugins_infill_simpleinfillpatterns_hpp_
#define plugins_infill_simpleinfillpatterns_hpp_

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_infill.h"

/*
Simple infill pattern plugins
=============================

Registration entry points for the infill patterns that were ported from the
legacy Fill classes to standalone plugins. Each implementation lives in its own
.cpp file and only uses the public plugin API plus PatternPluginHelpers.hpp.
*/

namespace slic3r_api { namespace Infill {

void register_concentric_pattern_plugin(orchestrator_handle *orch);
// Classic concentric filling preserves contour nesting and group order through the public geometry API.
void fill_concentric_surface(const run_ctx_infill_pattern &ctx, storage_handle *storage, const plugin_run_context *run_ctx);
void register_line_pattern_plugin(orchestrator_handle *orch);
// Fill one surface with the line pattern; exposed for the parity tests.
void fill_line_surface(const run_ctx_infill_pattern &ctx, storage_handle *storage, const plugin_run_context *run_ctx);
void register_honeycomb_pattern_plugin(orchestrator_handle *orch);
void register_honeycomb_3d_pattern_plugin(orchestrator_handle *orch);
void register_hilbert_curve_pattern_plugin(orchestrator_handle *orch);
void register_archimedean_chords_pattern_plugin(orchestrator_handle *orch);
void register_octagram_spiral_pattern_plugin(orchestrator_handle *orch);

}} // namespace slic3r_api::Infill

#endif // plugins_infill_simpleinfillpatterns_hpp_
