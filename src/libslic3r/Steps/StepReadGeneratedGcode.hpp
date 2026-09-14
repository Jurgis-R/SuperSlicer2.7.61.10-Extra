///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef steps_stepreadgeneratedgcode_hpp_
#define steps_stepreadgeneratedgcode_hpp_

#include <string_view>

namespace Slic3r {

struct GCodeProcessorResult;
class Orchestrator;
class Print;

namespace Steps::StepReadGeneratedGcode {

/*
Generated G-code preview phase
==============================

This host-only phase follows STEP_GCODE but is not a selectable slicing step.
The writer has already completed its artifact and stored the path of a readable
text representation in Print. This phase streams that text through the Reader
selected by the firmware and builds the legacy GCodeProcessorResult expected
by GCodeViewer.

The caller decides whether a Reader failure is fatal. run() reports failures as
exceptions and never removes or modifies the producer-owned source file.
*/
void run(Orchestrator &orchestrator,
         Print &print,
         std::string_view reader_plugin_id,
         GCodeProcessorResult &result);

} // namespace Steps::StepReadGeneratedGcode
} // namespace Slic3r

#endif // steps_stepreadgeneratedgcode_hpp_
