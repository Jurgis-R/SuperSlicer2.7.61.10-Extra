///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef plugins_infill_rectilinear_patterns_hpp_
#define plugins_infill_rectilinear_patterns_hpp_

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"

/*
Rectilinear infill pattern plugins
==================================

Registration entry point for the nine rectilinear patterns ported from
FillRectilinear.cpp: rectilinear, alignedrectilinear, monotonic, monotoniclines,
grid, triangles, stars, cubic and scatteredrectilinear. The algorithms live in
RectilinearFills.cpp; this registration only wraps them with the INFILL_PATTERN
plugin API.
*/

namespace slic3r_api { namespace Infill {

void register_rectilinear_pattern_plugins(orchestrator_handle *orch);

}} // namespace slic3r_api::Infill

#endif // plugins_infill_rectilinear_patterns_hpp_
