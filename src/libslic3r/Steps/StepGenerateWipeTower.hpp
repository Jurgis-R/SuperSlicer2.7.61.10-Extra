///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef steps_stepgeneratewipetower_hpp_
#define steps_stepgeneratewipetower_hpp_

// The wipe-tower step dispatches whole-plan managers and validates their synthetic extrusion roots.

#include <string>

#include "libslic3r/Api/plugin/c/slic3r_plugin_types.h"

namespace Slic3r {
class Orchestrator;
class Print;

namespace Steps::StepGenerateWipeTower {

void clear(Print &print);
void prepare(Print &print);
bool validate_pre(const Print &print, std::string *error = nullptr);
// Reject wipe roots without valid synthetic provenance or fixed execution order.
bool validate_post(const Print &print, std::string *error = nullptr);
// Execute selected managers once on the whole mutable plan, then validate output.
void run_step(Orchestrator &orchestrator, Print &print);

} // namespace Steps::StepGenerateWipeTower
} // namespace Slic3r

#endif // steps_stepgeneratewipetower_hpp_
