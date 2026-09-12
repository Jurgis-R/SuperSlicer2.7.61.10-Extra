///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef steps_stepgenerategcode_hpp_
#define steps_stepgenerategcode_hpp_

#include "libslic3r/Api/plugin/c/slic3r_gcode_thumbnail.h"
#include <string>

#include "libslic3r/Api/plugin/c/slic3r_plugin_types.h"

namespace Slic3r {
class Orchestrator;
class Print;

namespace Steps::StepGenerateGcode {

void clean_and_prepare(Print &print);
bool validate_pre(const Print &print, std::string *error = nullptr);
bool validate_post(const Print &print, std::string *error = nullptr);
std::string run_step(Orchestrator &orchestrator, Print &print, const std::string &path,
        const raw_gcode_thumbnail_renderer *thumbnail_renderer = nullptr);

} // namespace Steps::StepGenerateGcode
} // namespace Slic3r

#endif // steps_stepgenerategcode_hpp_
