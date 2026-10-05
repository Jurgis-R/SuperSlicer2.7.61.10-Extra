///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Adaptive Cubic filling implementation
======================================

Surface filling uses only the immutable object cache and publishes paths
through the common infill helpers.
*/

#include "AdaptiveFill.hpp"
#include "AdaptivePaths.hpp"

#include <algorithm>
#include <limits>
#include <vector>

#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/Plugins/Infill/PatternPluginHelpers.hpp"
#include "libslic3r/Plugins/Infill/InfillPaths.hpp"
#include "libslic3r/Plugins/Infill/Rectilinear/RectilinearConnect.hpp"

/*
Surface filling is intentionally separate from preparation. The immutable
octree is shared by every surface of one object, while clipping and extrusion
publication remain local to the current pattern call.
*/

namespace slic3r_api { namespace Infill { namespace AdaptiveCubic {

void fill_surface(const run_ctx_infill_pattern &ctx,
                  const AdaptiveCache &cache,
                  storage_handle *storage,
                  const plugin_run_context *run_ctx)
{
    const raw_infill_pattern_params &params = *ctx.params;
    if (cache.octree == nullptr || params.density <= 0.f || params.spacing <= 0.)
        return;

    const ExPolygon surface = Surface(ctx.surface).expolygon();
    StoredExPolygonCollection clip = PatternHelpers::shrink_for_fill(storage, surface, params.spacing);
    if (clip.empty())
        return;

    const std::vector<std::pair<c_point, c_point>> lines = cache.octree->generate_lines(params.z, run_ctx);
    StoredPolylineCollection subjects(storage);
    for (const std::pair<c_point, c_point> &line : lines) {
        throw_if_cancelled(run_ctx);
        StoredPolyline path(storage);
        path.push_back(line.first);
        path.push_back(line.second);
        subjects.push_back_move(std::move(path));
    }
    StoredExtrusionEntity group(storage);
    for (const ExPolygon boundary : clip) {
        throw_if_cancelled(run_ctx);
        StoredExPolygonCollection mask(storage);
        mask.push_back(boundary);
        throw_if_cancelled(run_ctx);
        const StoredPolylineCollection clipped = clipper_intersection_polylines_expolygons(storage, subjects, mask);
        work::Polylines pieces = Paths::read_polylines(clipped, run_ctx);
        for (work::Polyline &piece : pieces)
            if (piece.size() > 2)
                piece.points.erase(piece.points.begin() + 1, piece.points.end() - 1);
        if (pieces.size() > 1)
            pieces = connect_hooks(std::move(pieces), boundary, params.spacing,
                std::min(double(std::numeric_limits<coord_t>::max()), scale_d(params.anchor_length)),
                std::min(double(std::numeric_limits<coord_t>::max()), scale_d(params.anchor_length_max)), run_ctx);
        work::Polylines connected;
        if (params.connection == C_INFILL_CONNECTION_NOT_CONNECTED || pieces.size() <= 1)
            connected = Paths::chain_polylines(std::move(pieces), run_ctx);
        else
            RectilinearPatternsPlugin::connect::connect_infill(storage, std::move(pieces), boundary,
                connected, scale_i(params.spacing), params);
        work::ensure_valid(connected, params.fill_resolution);
        for (const work::Polyline &line : connected) {
            StoredExtrusionEntity path = PatternHelpers::make_path(storage, line.points, params);
            group.append_child_move(path.mutable_view());
        }
    }
    if (!group.empty()) {
        group.set_flags(RAW_EXTRUSION_FLAG_SORTABLE | RAW_EXTRUSION_FLAG_REVERSIBLE);
        MutableExtrusionEntity(ctx.output).append_child_move(group.mutable_view());
    }
}

}}} // namespace slic3r_api::Infill::AdaptiveCubic
