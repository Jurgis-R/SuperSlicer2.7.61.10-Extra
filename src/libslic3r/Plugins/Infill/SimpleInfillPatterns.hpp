///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef plugins_infill_simpleinfillpatterns_hpp_
#define plugins_infill_simpleinfillpatterns_hpp_

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"

/*
Simple infill pattern plugins
=============================

Registration entry points for the infill patterns that were ported from the
legacy Fill classes to standalone plugins. Each implementation lives in its own
.cpp file and only uses the public plugin API plus PatternPluginHelpers.hpp.
*/

namespace slic3r_api { namespace Infill {

void register_honeycomb_pattern_plugin(orchestrator_handle *orch);
void register_honeycomb_3d_pattern_plugin(orchestrator_handle *orch);
void register_hilbert_curve_pattern_plugin(orchestrator_handle *orch);
void register_archimedean_chords_pattern_plugin(orchestrator_handle *orch);
void register_octagram_spiral_pattern_plugin(orchestrator_handle *orch);

}} // namespace slic3r_api::Infill

#endif // plugins_infill_simpleinfillpatterns_hpp_
