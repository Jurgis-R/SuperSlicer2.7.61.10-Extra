///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Seam initialization cancellation
================================
Seam providers build immutable sessions before publishing them. These helpers
keep the run context borrowed by that build phase and provide inexpensive,
consistent checkpoints for outer work units and long scalar loops.
*/

#ifndef slic3r_Plugins_SeamPlacer_SeamInitializationCancellation_hpp_
#define slic3r_Plugins_SeamPlacer_SeamInitializationCancellation_hpp_

#include <cstddef>

#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"

namespace slic3r_api { namespace SeamPlacement { namespace InitializationCancellation {

inline void check(const plugin_run_context *run_context)
{
    throw_if_cancelled(run_context);
}

inline void check_periodic(const plugin_run_context *run_context,
                           size_t index, size_t stride = 64)
{
    if (index % stride == 0)
        check(run_context);
}

}}} // namespace slic3r_api::SeamPlacement::InitializationCancellation

#endif
