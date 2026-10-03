///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Adaptive Cubic filling interface
================================

The filler turns prepared octree lines into clipped, flow-attributed infill
paths for one surface.
*/

#ifndef plugins_infill_adaptivecubic_adaptivefill_hpp_
#define plugins_infill_adaptivecubic_adaptivefill_hpp_

#include "AdaptivePreparation.hpp"

namespace slic3r_api { namespace Infill { namespace AdaptiveCubic {

/* Generate one surface from an already prepared object-level octree. */
void fill_surface(const run_ctx_infill_pattern &ctx,
                  const AdaptiveCache &cache,
                  storage_handle *storage,
                  const plugin_run_context *run_ctx);

}}} // namespace slic3r_api::Infill::AdaptiveCubic

#endif // plugins_infill_adaptivecubic_adaptivefill_hpp_
