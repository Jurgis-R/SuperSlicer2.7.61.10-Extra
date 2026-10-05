///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "SimpleInfillPatterns.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_def.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_infill.h"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/Plugins/Infill/PatternPluginHelpers.hpp"

/*
Honeycomb infill pattern
========================

Standalone port of the former FillHoneycomb core algorithm. The hexagon math is
kept as-is; the frame handling, clipping and publication come from
PatternPluginHelpers.hpp. The legacy layer rotation (a third of a turn every
three layers) is applied on top of the shared infill angle.
*/

namespace slic3r_api { namespace Infill {
namespace {

using namespace slic3r_api::Infill::PatternHelpers;

const char *k_id = "honeycomb";
const char *k_no_dependencies[] = { nullptr };

void generate_honeycomb_surface(const run_ctx_infill_pattern &ctx, storage_handle *storage)
{
    const raw_infill_pattern_params &params = *ctx.params;
    const double my_spacing = params.max_sparse_infill_spacing > 0.f ?
        double(params.max_sparse_infill_spacing) : params.spacing;
    if (my_spacing <= 0. || params.density <= 0.f)
        return;

    // Cached hexagon math from the legacy implementation.
    const coord_t min_spacing = scale_i(my_spacing);
    const coord_t distance = coord_t(double(min_spacing) / double(params.density));
    if (distance <= 0)
        return;
    const coord_t hex_side = coord_t(double(distance) / (std::sqrt(3.) / 2.));
    const coord_t hex_width = distance * 2;
    const coord_t pattern_height = hex_side * 2 + hex_side;
    const coord_t y_short = coord_t(double(distance) * std::sqrt(3.) / 3.);
    const coord_t x_offset = min_spacing / 2;
    const coord_t y_offset = coord_t(double(x_offset) * std::sqrt(3.) / 3.);
    const c_point hex_center { hex_width / 2, hex_side };

    // FillHoneycomb::_layer_angle() rotates by a third of a turn every three
    // layers; it replaces the default cross-layer alternation.
    const double angle = infill_angle(params) + k_pi / 3. * double(params.layer_id % 3u);

    StoredExPolygonCollection clip = shrink_for_fill(storage, Surface(ctx.surface).expolygon(), params.spacing);
    if (clip.empty())
        return;

    StoredPolylineCollection pieces(storage);
    for (const ExPolygon &expolygon : clip) {
        const c_bounding_box contour_box = expolygon.contour().bounding_box();

        // Rotate the bounding box around the hex center and use the extents of
        // the rotated box as the generation frame. The minimum corner is then
        // snapped to the honeycomb lattice so the pattern stays aligned across
        // layers, exactly like the legacy FillHoneycomb.
        c_bounding_box frame = bounding_box_rotate(contour_box, angle, hex_center);
        bounding_box_merge_aligned(frame, c_point{hex_width, pattern_height});
        if (!bounding_box_valid(frame))
            continue;
        const coord_t bbox_min_x = frame.min.x;
        const coord_t bbox_min_y = frame.min.y;
        const coord_t bbox_max_x = frame.max.x;
        const coord_t bbox_max_y = frame.max.y;

        std::vector<std::vector<c_point>> polylines;
        coord_t x = bbox_min_x;
        while (x <= bbox_max_x) {
            std::vector<c_point> polyline;
            coord_t ax[2] = { x + x_offset, x + distance - x_offset };
            for (size_t half = 0; half < 2; ++half) {
                std::reverse(polyline.begin(), polyline.end());
                for (coord_t y = bbox_min_y; y <= bbox_max_y;
                     y += y_short + hex_side + y_short + hex_side) {
                    polyline.push_back({ ax[1], y + y_offset });
                    polyline.push_back({ ax[0], y + y_short - y_offset });
                    polyline.push_back({ ax[0], y + y_short + hex_side + y_offset });
                    polyline.push_back({ ax[1], y + y_short + hex_side + y_short - y_offset });
                    polyline.push_back({ ax[1], y + y_short + hex_side + y_short + hex_side + y_offset });
                }
                ax[0] += distance;
                ax[1] += distance;
                std::swap(ax[0], ax[1]);
                x += distance;
            }
            for (c_point &point : polyline)
                point = rotate_around(point, -angle, hex_center);
            if (polyline.size() >= 2)
                polylines.emplace_back(std::move(polyline));
        }

        StoredExPolygonCollection single = ClipperOperand(storage, expolygon).to_expolygon_collection();
        for (const std::vector<c_point> &polyline : polylines)
            append_clipped_pieces(storage, polyline, single, 0., pieces);
    }

    if (pieces.empty())
        return;

    order_pieces(pieces);
    publish_infill_paths(ctx, storage, pieces, params);
}

class HoneycombPattern : public PluginBase
{
public:
    explicit HoneycombPattern(orchestrator_handle *orch) : PluginBase(orch) {}

private:
    const char *id_impl() const noexcept override { return k_id; }
    const char *name_impl() const noexcept override { return "Honeycomb"; }
    const char *description_impl() const noexcept override
    {
        return "Honeycomb infill pattern implemented with the public plugin API.";
    }
    slicing_step_t step_impl() const noexcept override { return INFILL_PATTERN; }
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    int32_t priority_impl() const noexcept override { return 10; }
    const char *progress_message_format_impl() const noexcept override
    {
        return "Generating honeycomb pattern: %u / %u surfaces";
    }

    void inilialize_impl(storage_handle *) const override
    {
        add_self_to_pattern_list(m_orchestrator, k_id, "Honeycomb", { "fill_pattern" });
    }

    void run_impl(const plugin_run_context *run_ctx) const override
    {
        throw_if_cancelled(run_ctx);
        const run_ctx_infill_pattern *ctx = plugin_ctx_as_infill_pattern(run_ctx);
        if (ctx == nullptr || ctx->surface == nullptr || ctx->params == nullptr || ctx->output == nullptr ||
            run_ctx == nullptr || run_ctx->plugin_storage == nullptr)
            return;

        generate_honeycomb_surface(*ctx, run_ctx->plugin_storage);
        progress().increment();
    }
};

HoneycombPattern &instance(orchestrator_handle *orch)
{
    static HoneycombPattern s_instance(orch);
    return s_instance;
}

} // namespace

void register_honeycomb_pattern_plugin(orchestrator_handle *orch)
{
    orchestrator_register_plugin(orch, instance(orch).c_instance());
}

}} // namespace slic3r_api::Infill
