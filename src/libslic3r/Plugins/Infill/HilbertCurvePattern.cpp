///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "SimpleInfillPatterns.hpp"

#include <cmath>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_def.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_infill.h"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/Plugins/Infill/PatternPluginHelpers.hpp"

/*
Hilbert curve infill pattern
============================

Standalone port of the former FillHilbertCurve plane path, adapted from
Math::PlanePath::HilbertCurve. The curve covers a power-of-two square and is
clipped to the shrunk surface, then chained.
*/

namespace slic3r_api { namespace Infill {
namespace {

using namespace slic3r_api::Infill::PatternHelpers;

const char *k_id = "hilbertcurve";
const char *k_no_dependencies[] = { nullptr };

// state=0    3--2   plain
//               |
//            0--1
//
// state=4    1--2  transpose
//            |  |
//            0  3
//
// state=8
//
// state=12   3  0  rot180 + transpose
//            |  |
//            2--1
//
c_point hilbert_n_to_xy(size_t n)
{
    static constexpr const int next_state[16] { 4,0,0,12, 0,4,4,8, 12,8,8,4, 8,12,12,0 };
    static constexpr const int digit_to_x[16] { 0,1,1,0, 0,0,1,1, 1,0,0,1, 1,1,0,0 };
    static constexpr const int digit_to_y[16] { 0,0,1,1, 0,1,1,0, 1,1,0,0, 1,0,0,1 };

    // Number of 2 bit digits.
    size_t ndigits = 0;
    {
        size_t nc = n;
        while (nc > 0) {
            nc >>= 2;
            ++ndigits;
        }
    }
    int state = (ndigits & 1) ? 4 : 0;
    coord_t x = 0;
    coord_t y = 0;
    for (int i = int(ndigits) - 1; i >= 0; --i) {
        const int digit = (n >> (i * 2)) & 3;
        state += digit;
        x |= coord_t(digit_to_x[state]) << i;
        y |= coord_t(digit_to_y[state]) << i;
        state = next_state[state];
    }
    return { x, y };
}

void generate_hilbert_curve(coord_t min_x, coord_t min_y, coord_t max_x, coord_t max_y, double,
                            std::vector<GridPoint> &out)
{
    if (max_x < min_x || max_y < min_y)
        return;

    // Minimum power of two square to fit the domain.
    size_t sz = 2;
    const size_t sz0 = size_t(std::max(max_x + 1 - min_x, max_y + 1 - min_y));
    while (sz < sz0)
        sz <<= 1;

    const size_t sz2 = sz * sz;
    out.reserve(sz2);
    for (size_t i = 0; i < sz2; ++i) {
        const c_point point = hilbert_n_to_xy(i);
        out.push_back({ double(point.x + min_x), double(point.y + min_y) });
    }
}

class HilbertCurvePattern : public PluginBase
{
public:
    explicit HilbertCurvePattern(orchestrator_handle *orch) : PluginBase(orch) {}

private:
    const char *id_impl() const noexcept override { return k_id; }
    const char *name_impl() const noexcept override { return "Hilbert Curve"; }
    const char *description_impl() const noexcept override
    {
        return "Hilbert curve infill pattern implemented with the public plugin API.";
    }
    slicing_step_t step_impl() const noexcept override { return INFILL_PATTERN; }
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    int32_t priority_impl() const noexcept override { return 12; }
    const char *progress_message_format_impl() const noexcept override
    {
        return "Generating hilbert curve pattern: %u / %u surfaces";
    }

    void inilialize_impl(storage_handle *) const override
    {
        add_self_to_pattern_list(m_orchestrator, k_id, "Hilbert Curve",
                                 { "fill_pattern", "top_fill_pattern", "bottom_fill_pattern", "solid_fill_pattern" });
    }

    void run_impl(const plugin_run_context *run_ctx) const override
    {
        throw_if_cancelled(run_ctx);
        const run_ctx_infill_pattern *ctx = plugin_ctx_as_infill_pattern(run_ctx);
        if (ctx == nullptr || ctx->surface == nullptr || ctx->params == nullptr || ctx->output == nullptr ||
            run_ctx == nullptr || run_ctx->plugin_storage == nullptr)
            return;

        fill_plane_path_surface(*ctx, run_ctx->plugin_storage, Surface(ctx->surface).expolygon(), *ctx->params,
                                false, generate_hilbert_curve);
        progress().increment();
    }
};

HilbertCurvePattern &instance(orchestrator_handle *orch)
{
    static HilbertCurvePattern s_instance(orch);
    return s_instance;
}

} // namespace

void register_hilbert_curve_pattern_plugin(orchestrator_handle *orch)
{
    orchestrator_register_plugin(orch, instance(orch).c_instance());
}

}} // namespace slic3r_api::Infill
