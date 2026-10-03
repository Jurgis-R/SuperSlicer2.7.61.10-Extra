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

Registration entry point for the eleven rectilinear patterns ported from
FillRectilinear.cpp: rectilinear, alignedrectilinear, monotonic, monotoniclines,
grid, triangles, stars, cubic, scatteredrectilinear, sawtooth and
rectiwithperimeter. Sawtooth creates a Z-profiled extrusion tree and is
published separately. The other algorithms live in RectilinearFills.cpp and
RectilinearWithPerimeter.cpp; this registration wraps them with the
INFILL_PATTERN plugin API.
*/

namespace slic3r_api { namespace Infill {

void register_rectilinear_pattern_plugins(orchestrator_handle *orch);

}} // namespace slic3r_api::Infill

#endif // plugins_infill_rectilinear_patterns_hpp_
