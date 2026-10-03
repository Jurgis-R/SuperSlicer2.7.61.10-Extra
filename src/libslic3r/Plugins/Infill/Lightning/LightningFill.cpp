///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

// Immutable layer branches are clipped and connected per surface using call-local buffers and storage.
#include "LightningFill.hpp"
#include <stdexcept>
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/Plugins/Infill/InfillPaths.hpp"
#include "libslic3r/Plugins/Infill/PatternPluginHelpers.hpp"
#include "libslic3r/Plugins/Infill/Rectilinear/RectilinearConnect.hpp"

namespace slic3r_api { namespace Infill { namespace Lightning {

void fill_surface(const run_ctx_infill_pattern &ctx, const Cache &cache,
                   storage_handle *storage, const plugin_run_context *run_ctx)
{
    const std::map<const layer_handle *, size_t>::const_iterator layer = cache.layer_indices.find(ctx.layer);
    if (layer == cache.layer_indices.end())
        throw std::runtime_error("Lightning layer was not prepared during setup.");
    const raw_infill_pattern_params &params = *ctx.params;
    const ExPolygon surface = Surface(ctx.surface).expolygon();
    const StoredExPolygonCollection inset = PatternHelpers::shrink_for_fill(storage, surface, params.spacing);
    StoredExtrusionEntity group(storage);
    for (const ExPolygon component : inset) {
        throw_if_cancelled(run_ctx);
        const coord_t overlap = scale_i(0.5 * params.spacing - params.overlap);
        const work::Polylines branches = cache.layers[layer->second].paths(overlap, run_ctx);
        const StoredPolylineCollection subjects = Paths::store_polylines(storage, branches, run_ctx);
        StoredExPolygonCollection mask(storage);
        mask.push_back(component);
        throw_if_cancelled(run_ctx);
        const StoredPolylineCollection pieces = clipper_intersection_polylines_expolygons(storage, subjects, mask);
        work::Polylines clipped = Paths::read_polylines(pieces, run_ctx);
        work::Polylines output;
        if (params.connection == C_INFILL_CONNECTION_NOT_CONNECTED || clipped.size() <= 1)
            output = Paths::chain_polylines(std::move(clipped), run_ctx);
        else
            RectilinearPatternsPlugin::connect::connect_infill(storage, std::move(clipped), component,
                output, scale_i(params.spacing), params);
        work::ensure_valid(output, params.fill_resolution);
        for (const work::Polyline &path : output) {
            StoredExtrusionEntity extrusion = PatternHelpers::make_path(storage, path.points, params);
            group.append_child_move(extrusion.mutable_view());
        }
    }
    if (!group.empty()) {
        group.set_flags(RAW_EXTRUSION_FLAG_SORTABLE | RAW_EXTRUSION_FLAG_REVERSIBLE);
        MutableExtrusionEntity(ctx.output).append_child_move(group.mutable_view());
    }
}
}}}
