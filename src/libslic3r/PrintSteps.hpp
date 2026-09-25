///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_PrintSteps_hpp_
#define slic3r_PrintSteps_hpp_

#include <cstdint>

#include "Api/plugin/c/slic3r_slicing_step.h"

namespace Slic3r {

using PrintStep = slicing_step_t;
using PrintObjectStep = slicing_step_t;

// Compatibility aliases for the historical FFF print-level steps.
static constexpr PrintStep psAlertWhenSupportsNeeded = STEP_ALERT_SUPPORTS_NEEDED;
static constexpr PrintStep psSkirtBrim               = STEP_SKIRT_BRIM;
static constexpr PrintStep psCheckConflict           = STEP_CHECK_CONFLICT;
static constexpr PrintStep psSlicingFinished         = STEP_CHECK_CONFLICT;
static constexpr PrintStep psToolOrdering            = STEP_ORDERING;
static constexpr PrintStep psWipeTower               = STEP_WIPETOWER;
static constexpr PrintStep psGCodeExport             = STEP_GCODE;
static constexpr PrintStep psCount                   = static_cast<PrintStep>(STEP_GCODE + 1);

// Compatibility aliases for the historical FFF object-level steps.
static constexpr PrintObjectStep posSlice                          = STEP_SLICING;
static constexpr PrintObjectStep posPerimeters                     = STEP_PERIMETER;
static constexpr PrintObjectStep posPrepareInfill                  = STEP_PRE_INFILL;
static constexpr PrintObjectStep posInfill                         = STEP_INFILL;
static constexpr PrintObjectStep posIroning                        = STEP_POST_INFILL;
static constexpr PrintObjectStep posSupportSpotsSearch             = STEP_SUPPORT_SPOT;
static constexpr PrintObjectStep posSupportMaterial                = STEP_SUPPORT;
static constexpr PrintObjectStep posEstimateCurledExtrusions       = STEP_LAYER_EXTRUSION_EDIT;
static constexpr PrintObjectStep posCalculateOverhangingPerimeters = STEP_EXTRUSION_EDIT;
static constexpr PrintObjectStep posSimplifyPath                   = STEP_EXTRUSION_SIMPLIFICATION;
static constexpr PrintObjectStep posCount                          = static_cast<PrintObjectStep>(STEP_EXTRUSION_SIMPLIFICATION + 1);

int printstep_percent(slicing_step_t step);
int objectstep_percent(slicing_step_t step);
bool is_print_step(slicing_step_t step);
bool is_print_object_step(slicing_step_t step);

} // namespace Slic3r

#endif // slic3r_PrintSteps_hpp_
