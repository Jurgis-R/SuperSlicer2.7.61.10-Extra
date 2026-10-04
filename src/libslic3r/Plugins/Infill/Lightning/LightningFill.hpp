///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#ifndef plugins_lightning_fill_hpp_
#define plugins_lightning_fill_hpp_
#include "LightningGenerator.hpp"

namespace slic3r_api { namespace Infill { namespace Lightning {
// Conversion consumes the host random sequence without mutating cached trees.
void fill_surface(const run_ctx_infill_pattern &ctx, const Cache &cache,
                   storage_handle *storage, const plugin_run_context *run_ctx);
}}}
#endif
