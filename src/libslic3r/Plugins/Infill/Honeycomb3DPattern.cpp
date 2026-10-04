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
3D honeycomb infill pattern
===========================

Standalone port of the former Fill3DHoneycomb core algorithm. It creates a
contiguous sequence of points at a given height that makes up a horizontal
slice of the edges of a space filling truncated octahedron tesselation, as
described by David Eccles (gringer). No pattern rotation is supported.
*/

namespace slic3r_api { namespace Infill {
namespace {

using namespace slic3r_api::Infill::PatternHelpers;

const char *k_id = "3dhoneycomb";
const char *k_no_dependencies[] = { nullptr };

// Generate an array of points that are in the same direction as the basic
// printing line (i.e. Y points for columns, X points for rows). A negative
// offset only causes a change in the perpendicular direction.
std::vector<double> colinear_points(double offset, size_t base_location, size_t grid_length)
{
    const double offset2 = std::abs(offset / 2.);
    std::vector<double> points;
    points.push_back(double(base_location) - offset2);
    for (size_t i = 0; i < grid_length; ++i) {
        points.push_back(double(base_location) + double(i) + offset2);
        points.push_back(double(base_location) + double(i) + 1. - offset2);
    }
    points.push_back(double(base_location) + double(grid_length) + offset2);
    return points;
}

// Generate an array of points for the dimension that is perpendicular to the
// basic printing line (i.e. X points for columns, Y points for rows).
std::vector<double> perpend_points(double offset, size_t base_location, size_t grid_length)
{
    const double offset2 = offset / 2.;
    coord_t side = 2 * (base_location & 1) - 1;
    std::vector<double> points;
    points.push_back(double(base_location) - offset2 * double(side));
    for (size_t i = 0; i < grid_length; ++i) {
        side = 2 * ((i + base_location) & 1) - 1;
        points.push_back(double(base_location) + offset2 * double(side));
        points.push_back(double(base_location) + offset2 * double(side));
    }
    points.push_back(double(base_location) - offset2 * double(side));
    return points;
}

void trim(std::vector<GridPoint> &points, double min_x, double min_y, double max_x, double max_y)
{
    for (GridPoint &point : points) {
        point.x = std::clamp(point.x, min_x, max_x);
        point.y = std::clamp(point.y, min_y, max_y);
    }
}

std::vector<GridPoint> zip(const std::vector<double> &x, const std::vector<double> &y)
{
    std::vector<GridPoint> out;
    out.reserve(x.size());
    for (size_t i = 0; i < x.size(); ++i)
        out.push_back({ x[i], y[i] });
    return out;
}

// Generate a set of curves that describe a horizontal slice of a truncated
// regular octahedron with edge length 1. curve_type selects the lines to
// print: 1 for vertical lines (columns), 2 for horizontal lines (rows), and 3
// for both.
std::vector<std::vector<GridPoint>> make_normalised_grid(double z, size_t grid_width, size_t grid_height, size_t curve_type)
{
    // Offset required to create a regular octagram.
    const double octagram_gap = 0.5;
    // Sawtooth wave function for range f(z) = [-octagram_gap .. octagram_gap].
    const double a = std::sqrt(2.);
    const double wave = std::abs(std::fmod(z, a) - a / 2.) / a * 4. - 1.;
    const double offset = wave * octagram_gap;

    std::vector<std::vector<GridPoint>> points;
    if ((curve_type & 1) != 0) {
        for (size_t x = 0; x <= grid_width; ++x) {
            std::vector<GridPoint> new_points = zip(perpend_points(offset, x, grid_height),
                                                    colinear_points(offset, 0, grid_height));
            trim(new_points, 0., 0., double(grid_width), double(grid_height));
            if (x & 1)
                std::reverse(new_points.begin(), new_points.end());
            points.emplace_back(std::move(new_points));
        }
    }
    if ((curve_type & 2) != 0) {
        for (size_t y = 0; y <= grid_height; ++y) {
            std::vector<GridPoint> new_points = zip(colinear_points(offset, 0, grid_width),
                                                    perpend_points(offset, y, grid_width));
            trim(new_points, 0., 0., double(grid_width), double(grid_height));
            if (y & 1)
                std::reverse(new_points.begin(), new_points.end());
            points.emplace_back(std::move(new_points));
        }
    }
    return points;
}

std::vector<std::vector<c_point>> make_grid(coord_t z, coord_t grid_size, size_t grid_width, size_t grid_height,
                                            size_t curve_type)
{
    const double normalised_z = double(z) / double(grid_size);
    const std::vector<std::vector<GridPoint>> polylines =
        make_normalised_grid(normalised_z, grid_width, grid_height, curve_type);

    std::vector<std::vector<c_point>> result;
    result.reserve(polylines.size());
    for (const std::vector<GridPoint> &points : polylines) {
        std::vector<c_point> polyline;
        polyline.reserve(points.size());
        for (const GridPoint &point : points)
            polyline.push_back({ coord_t(point.x * double(grid_size)), coord_t(point.y * double(grid_size)) });
        result.emplace_back(std::move(polyline));
    }
    return result;
}

void generate_honeycomb_3d_surface(const run_ctx_infill_pattern &ctx, storage_handle *storage)
{
    const raw_infill_pattern_params &params = *ctx.params;
    const coord_t distance = line_spacing_for_density(params);
    if (distance <= 0)
        return;

    StoredExPolygonCollection clip = shrink_for_fill(storage, Surface(ctx.surface).expolygon(), params.spacing);
    if (clip.empty())
        return;

    StoredPolylineCollection pieces(storage);
    for (const ExPolygon &expolygon : clip) {
        // A module is 2*distance: one half-module is growing while the other
        // half-module is shrinking. Snap the minimum corner to that module so
        // the tesselation stays aligned across layers, exactly like the legacy
        // Fill3DHoneycomb.
        c_bounding_box bb = expolygon.contour().bounding_box();
        bounding_box_merge_aligned(bb, c_point{2 * distance, 2 * distance});
        const coord_t width = bb.max.x - bb.min.x;
        const coord_t height = bb.max.y - bb.min.y;
        if (width <= 0 || height <= 0)
            continue;

        const size_t grid_width = size_t(std::ceil(double(width) / double(distance))) + 1;
        const size_t grid_height = size_t(std::ceil(double(height) / double(distance))) + 1;
        const size_t curve_type = size_t(params.layer_id % 2u) + 1;

        std::vector<std::vector<c_point>> polylines =
            make_grid(scale_i(params.z), distance, grid_width, grid_height, curve_type);
        for (std::vector<c_point> &polyline : polylines)
            for (c_point &point : polyline) {
                point.x += bb.min.x;
                point.y += bb.min.y;
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

class Honeycomb3DPattern : public PluginBase
{
public:
    explicit Honeycomb3DPattern(orchestrator_handle *orch) : PluginBase(orch) {}

private:
    const char *id_impl() const noexcept override { return k_id; }
    const char *name_impl() const noexcept override { return "3D Honeycomb"; }
    const char *description_impl() const noexcept override
    {
        return "3D honeycomb infill pattern implemented with the public plugin API.";
    }
    slicing_step_t step_impl() const noexcept override { return INFILL_PATTERN; }
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    int32_t priority_impl() const noexcept override { return 11; }
    const char *progress_message_format_impl() const noexcept override
    {
        return "Generating 3D honeycomb pattern: %u / %u surfaces";
    }

    void inilialize_impl(storage_handle *) const override
    {
        add_self_to_pattern_list(m_orchestrator, k_id, "3D Honeycomb", { "fill_pattern" });
    }

    void run_impl(const plugin_run_context *run_ctx) const override
    {
        throw_if_cancelled(run_ctx);
        const run_ctx_infill_pattern *ctx = plugin_ctx_as_infill_pattern(run_ctx);
        if (ctx == nullptr || ctx->surface == nullptr || ctx->params == nullptr || ctx->output == nullptr ||
            run_ctx == nullptr || run_ctx->plugin_storage == nullptr)
            return;

        generate_honeycomb_3d_surface(*ctx, run_ctx->plugin_storage);
        progress().increment();
    }
};

Honeycomb3DPattern &instance(orchestrator_handle *orch)
{
    static Honeycomb3DPattern s_instance(orch);
    return s_instance;
}

} // namespace

void register_honeycomb_3d_pattern_plugin(orchestrator_handle *orch)
{
    orchestrator_register_plugin(orch, instance(orch).c_instance());
}

}} // namespace slic3r_api::Infill
