///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#ifndef plugins_lightning_generator_hpp_
#define plugins_lightning_generator_hpp_
#include <map>
#include "LightningLayer.hpp"

namespace slic3r_api { namespace Infill { namespace Lightning {
struct Cache
{
    std::vector<LayerTree> layers;
    std::map<const layer_handle *, size_t> layer_indices;
};
// Build top-down with private scratch, then discard all input geometry and mutable query grids.
std::unique_ptr<Cache> build_cache(const Inputs &inputs, storage_handle *scratch,
                                  const plugin_run_context *run_ctx);
}}}
#endif
