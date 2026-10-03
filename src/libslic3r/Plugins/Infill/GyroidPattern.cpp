///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "GyroidPattern.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_def.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_infill.h"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/Plugins/Infill/PatternPluginHelpers.hpp"

/*
Gyroid infill pattern
=====================

This plugin is the standalone port of the former FillGyroid core algorithm. It
only uses the public plugin API:

    run_ctx_infill_pattern
    |-- surface expolygon through SurfaceViews
    |-- recipe through raw_infill_pattern_params
    |-- clipping through ClipperViews
    `-- extrusion publication through ExtrusionViews

The wave math is a direct port of the legacy implementation, and the shared
frame/angle/ordering helpers keep it aligned with the legacy Fill pipeline. The
clipped waves stay separate sortable paths, ordered by the shared connect_infill
helper and finally by STEP_ORDERING, which also inserts the travels.
*/

namespace slic3r_api { namespace Infill { namespace GyroidPatternPlugin {
namespace {

using namespace slic3r_api::Infill::PatternHelpers;

const char *k_gyroid_id = "gyroid";
const char *k_fill_pattern_key = "fill_pattern";
const char *k_no_dependencies[] = { nullptr };

// Density adjustment from the legacy gyroid implementation.
constexpr double k_density_adjust = 2.44;

struct Vec2d
{
    double x = 0.;
    double y = 0.;

    Vec2d() = default;
    Vec2d(double x_value, double y_value) : x(x_value), y(y_value) {}
};

double gyroid_wave(double x, double z_sin, double z_cos, bool vertical, bool flip)
{
    if (vertical) {
        const double phase_offset = (z_cos < 0 ? k_pi : 0) + k_pi;
        const double a   = std::sin(x + phase_offset);
        const double b   = -z_cos;
        const double res = z_sin * std::cos(x + phase_offset + (flip ? k_pi : 0.));
        const double r   = std::sqrt(a * a + b * b);
        return std::asin(a / r) + std::asin(res / r) + k_pi;
    } else {
        const double phase_offset = z_sin < 0 ? k_pi : 0.;
        const double a   = std::cos(x + phase_offset);
        const double b   = -z_sin;
        const double res = z_cos * std::sin(x + phase_offset + (flip ? 0 : k_pi));
        const double r   = std::sqrt(a * a + b * b);
        return std::asin(a / r) + std::asin(res / r) + 0.5 * k_pi;
    }
}

// One period of the wave in dimensionless units. The piecewise subdivision
// keeps the chord deviation below the requested tolerance.
std::vector<Vec2d> make_one_period(double width, double z_cos, double z_sin, bool vertical, bool flip, double tolerance)
{
    std::vector<Vec2d> points;
    const double dx = k_half_pi;
    const double limit = std::min(2. * k_pi, width);
    points.reserve(size_t(std::ceil(limit / tolerance / 3.)));

    for (double x = 0.; x < limit - EPSILON; x += dx)
        points.push_back({ x, gyroid_wave(x, z_sin, z_cos, vertical, flip) });
    points.push_back({ limit, gyroid_wave(limit, z_sin, z_cos, vertical, flip) });

    for (;;) {
        const size_t size = points.size();
        for (size_t i = 1; i < size; ++i) {
            const Vec2d &lp = points[i - 1];
            const Vec2d &rp = points[i];
            const double x = lp.x + (rp.x - lp.x) / 2.;
            const Vec2d ip = { x, gyroid_wave(x, z_sin, z_cos, vertical, flip) };
            const double cross = (ip.x - lp.x) * (ip.y - rp.y) - (ip.y - lp.y) * (ip.x - rp.x);
            if (std::abs(cross) > tolerance * tolerance)
                points.push_back(ip);
        }

        if (size == points.size())
            break;
        std::sort(points.begin(), points.end(), [](const Vec2d &lhs, const Vec2d &rhs) { return lhs.x < rhs.x; });
    }

    return points;
}

// Extend one period to the requested width and scale it to coord_t.
std::vector<c_point> make_wave(const std::vector<Vec2d> &one_period, double width, double height, double offset,
                               double scale_factor, double z_cos, double z_sin, bool vertical, bool flip)
{
    std::vector<Vec2d> points = one_period;
    const double period = points.back().x;
    if (width != period) {
        points.reserve(one_period.size() * size_t(std::floor(width / period)));
        points.pop_back();

        const size_t n = points.size();
        do {
            points.emplace_back(points[points.size() - n].x + period, points[points.size() - n].y);
        } while (points.back().x < width - EPSILON);

        points.emplace_back(width, gyroid_wave(width, z_sin, z_cos, vertical, flip));
    }

    std::vector<c_point> out;
    out.reserve(points.size());
    for (Vec2d &point : points) {
        point.y += offset;
        point.y = std::clamp(point.y, 0., height);
        if (vertical)
            std::swap(point.x, point.y);
        out.push_back({ coord_t(std::llround(point.x * scale_factor)),
                        coord_t(std::llround(point.y * scale_factor)) });
    }
    return out;
}

std::vector<std::vector<c_point>> make_gyroid_waves(double grid_z, double scale_factor, double width, double height,
                                                    double tolerance)
{
    const double z = grid_z / scale_factor;
    const double z_sin = std::sin(z);
    const double z_cos = std::cos(z);

    bool vertical = std::abs(z_sin) <= std::abs(z_cos);
    double lower_bound = 0.;
    double upper_bound = height;
    bool flip = true;
    if (vertical) {
        flip = false;
        lower_bound = -k_pi;
        upper_bound = width - k_half_pi;
        std::swap(width, height);
    }

    const std::vector<Vec2d> one_period_odd = make_one_period(width, z_cos, z_sin, vertical, flip, tolerance);
    flip = !flip;
    const std::vector<Vec2d> one_period_even = make_one_period(width, z_cos, z_sin, vertical, flip, tolerance);

    std::vector<std::vector<c_point>> result;
    for (double y0 = lower_bound; y0 < upper_bound + EPSILON; y0 += k_pi) {
        result.emplace_back(make_wave(one_period_odd, width, height, y0, scale_factor, z_cos, z_sin, vertical, flip));
        y0 += k_pi;
        if (y0 < upper_bound + EPSILON)
            result.emplace_back(make_wave(one_period_even, width, height, y0, scale_factor, z_cos, z_sin, vertical, flip));
    }
    return result;
}

void generate_gyroid_surface(const run_ctx_infill_pattern &ctx, storage_handle *storage)
{
    const raw_infill_pattern_params &params = *ctx.params;
    if (params.spacing <= 0. || params.density <= 0.f)
        return;

    const ExPolygon surface_expolygon = Surface(ctx.surface).expolygon();

    // Match Fill::fill_surface(): shrink the surface by half the line spacing.
    StoredExPolygonCollection clip = shrink_for_fill(storage, surface_expolygon, params.spacing);
    if (clip.empty())
        return;

    const double infill_angle = PatternHelpers::infill_angle(params) + gyroid_layer_angle(params);

    const coord_t base_spacing = line_spacing_for_density(params);
    const coord_t line_spacing = coord_t(double(base_spacing) / k_density_adjust);
    if (line_spacing <= 0)
        return;

    // Bounding box of the clip area in the rotated pattern frame. Every point is
    // rotated and rounded like the legacy Polygon::rotate(), then the minimum
    // corner is snapped to the wave lattice so the waves stay aligned across
    // layers.
    c_bounding_box bounds = bounding_box_empty();
    for (const ExPolygon &expolygon : clip)
        bounding_box_merge_rotated(bounds, expolygon, -infill_angle, c_point{0, 0});
    if (!bounding_box_valid(bounds))
        return;

    const coord_t grid = coord_t(2. * k_pi * double(line_spacing));
    if (grid <= 0)
        return;
    bounding_box_merge_aligned(bounds, c_point{grid, grid});
    const coord_t origin_x = bounds.min.x;
    const coord_t origin_y = bounds.min.y;
    const double width = double((bounds.max.x - origin_x) / line_spacing) + 1.;
    const double height = double((bounds.max.y - origin_y) / line_spacing) + 1.;
    if (width <= 0. || height <= 0.)
        return;

    const double tolerance = unscaled(params.fill_resolution) / unscaled(line_spacing);
    std::vector<std::vector<c_point>> waves = make_gyroid_waves(
        scale_d(params.z), double(line_spacing), width, height, tolerance);

    const double cos_angle = std::cos(infill_angle);
    const double sin_angle = std::sin(infill_angle);
    const double min_length = double(scale_i(0.8 * params.spacing));

    StoredPolylineCollection pieces(storage);
    for (std::vector<c_point> &wave : waves) {
        // Place the wave on the rotated grid origin, then rotate it back into
        // the surface frame so it can be clipped against the original area.
        for (c_point &point : wave) {
            const double x = double(point.x + origin_x);
            const double y = double(point.y + origin_y);
            point.x = coord_t(std::llround(x * cos_angle - y * sin_angle));
            point.y = coord_t(std::llround(x * sin_angle + y * cos_angle));
        }

        append_clipped_pieces(storage, wave, clip, min_length, pieces);
    }

    if (pieces.empty())
        return;

    order_pieces(pieces);
    publish_infill_paths(ctx, storage, pieces, params);
}

} // namespace

GyroidPattern &GyroidPattern::instance(orchestrator_handle *orch)
{
    static GyroidPattern s_instance(orch);
    return s_instance;
}

const char *GyroidPattern::id_impl() const noexcept
{
    return k_gyroid_id;
}

const char *GyroidPattern::name_impl() const noexcept
{
    return "Gyroid";
}

const char *GyroidPattern::description_impl() const noexcept
{
    return "Gyroid sparse infill pattern implemented with the public plugin API.";
}

slicing_step_t GyroidPattern::step_impl() const noexcept
{
    return INFILL_PATTERN;
}

const char *const *GyroidPattern::dependencies_impl() const noexcept
{
    return k_no_dependencies;
}

int32_t GyroidPattern::priority_impl() const noexcept
{
    // Keep the former declaration position so pattern runtime ids stay stable.
    return 12;
}

void GyroidPattern::inilialize_impl(storage_handle *) const
{
    add_self_to_pattern_list(m_orchestrator, k_gyroid_id, "Gyroid", { k_fill_pattern_key });
}

const char *GyroidPattern::progress_message_format_impl() const noexcept
{
    return "Generating gyroid pattern: %u / %u surfaces";
}

void GyroidPattern::run_impl(const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    const run_ctx_infill_pattern *ctx = plugin_ctx_as_infill_pattern(run_ctx);
    if (ctx == nullptr || ctx->surface == nullptr || ctx->params == nullptr || ctx->output == nullptr ||
        run_ctx == nullptr || run_ctx->plugin_storage == nullptr)
        return;

    generate_gyroid_surface(*ctx, run_ctx->plugin_storage);
    progress().increment();
}

void register_gyroid_pattern_plugin(orchestrator_handle *orch)
{
    orchestrator_register_plugin(orch, GyroidPattern::instance(orch).c_instance());
}

}}} // namespace slic3r_api::Infill::GyroidPatternPlugin
