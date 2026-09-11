///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_BuiltinGCodeThumbnails_hpp_
#define slic3r_BuiltinGCodeThumbnails_hpp_

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"

/* Registers the four built-in encoders in one exclusive service group. Each
   export selects one provider and may ask that session to encode many sizes. */
namespace slic3r_api { namespace GCodeGeneration { namespace Thumbnail {

void register_builtin_gcode_thumbnail_plugins(orchestrator_handle *orchestrator);

}}} // namespace slic3r_api::GCodeGeneration::Thumbnail

#endif // slic3r_BuiltinGCodeThumbnails_hpp_
