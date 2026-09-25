///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef steps_stepfinalizeslicing_hpp_
#define steps_stepfinalizeslicing_hpp_

#include <string>

#include "libslic3r/Api/plugin/c/slic3r_plugin_types.h"

namespace Slic3r {
class Orchestrator;
class Print;

namespace Steps::StepFinalizeSlicing {

void clean_and_prepare(Print &print);
bool validate_pre(const Print &print, std::string *error = nullptr);
bool validate_post(const Print &print, std::string *error = nullptr);
void run_step(Orchestrator &orchestrator, Print &print);

} // namespace Steps::StepFinalizeSlicing
} // namespace Slic3r

#endif // steps_stepfinalizeslicing_hpp_
