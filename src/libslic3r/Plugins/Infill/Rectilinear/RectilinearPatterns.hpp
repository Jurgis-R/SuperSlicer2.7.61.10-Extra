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

Registration entry point for the twelve rectilinear pattern ids ported from the
core family: rectilinear, alignedrectilinear, monotonic, monotoniclines, grid,
triangles, stars, cubic, scatteredrectilinear, sawtooth, rectiwithperimeter and
supportbase. The latter two are registered for id-based selection only and do
not join user option lists. Sawtooth creates a Z-profiled extrusion tree and
uses the host-safe random helper for z-hop offsets; the generator sets
C_INFILL_FLAG_DONT_ADJUST for the sawtooth recipe so later flow normalization
does not alter the generated profile. The other algorithms live in
RectilinearFills.cpp and RectilinearWithPerimeter.cpp; this registration wraps
them with the INFILL_PATTERN plugin API.
*/

namespace slic3r_api { namespace Infill {

void register_rectilinear_pattern_plugins(orchestrator_handle *orch);

}} // namespace slic3r_api::Infill

#endif // plugins_infill_rectilinear_patterns_hpp_
