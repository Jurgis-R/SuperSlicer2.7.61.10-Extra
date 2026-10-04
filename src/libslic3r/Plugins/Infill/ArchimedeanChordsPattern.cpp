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
Archimedean chords infill pattern
=================================

Standalone port of the former FillArchimedeanChords plane path. It follows an
Archimedean spiral in polar coordinates: r = a + b*theta, centered on the
pattern frame.
*/

namespace slic3r_api { namespace Infill {
namespace {

using namespace slic3r_api::Infill::PatternHelpers;

const char *k_id = "archimedeanchords";
const char *k_no_dependencies[] = { nullptr };

void generate_archimedean_chords(coord_t min_x, coord_t min_y, coord_t max_x, coord_t max_y, double resolution,
                                 std::vector<GridPoint> &out)
{
    // Radius to achieve.
    const double rmax = std::sqrt(double(max_x) * double(max_x) + double(max_y) * double(max_y)) * std::sqrt(2.) + 1.5;
    // Now unwind the spiral.
    const double a = 1.;
    const double b = 1. / (2. * k_pi);
    double theta = 0.;
    double r = 1.;
    // If used as a solid infill, there is a gap left at the center.
    out.push_back({ 0., 0. });
    out.push_back({ 1., 0. });
    while (r < rmax) {
        // Discretization angle to achieve a discretization error lower than resolution.
        theta += 2. * std::acos(1. - resolution / r);
        r = a + b * theta;
        out.push_back({ r * std::cos(theta), r * std::sin(theta) });
    }
}

class ArchimedeanChordsPattern : public PluginBase
{
public:
    explicit ArchimedeanChordsPattern(orchestrator_handle *orch) : PluginBase(orch) {}

private:
    const char *id_impl() const noexcept override { return k_id; }
    const char *name_impl() const noexcept override { return "Archimedean Chords"; }
    const char *description_impl() const noexcept override
    {
        return "Archimedean chords infill pattern implemented with the public plugin API.";
    }
    slicing_step_t step_impl() const noexcept override { return INFILL_PATTERN; }
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    int32_t priority_impl() const noexcept override { return 13; }
    const char *progress_message_format_impl() const noexcept override
    {
        return "Generating archimedean chords pattern: %u / %u surfaces";
    }

    void inilialize_impl(storage_handle *) const override
    {
        add_self_to_pattern_list(m_orchestrator, k_id, "Archimedean Chords",
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
                                true, generate_archimedean_chords);
        progress().increment();
    }
};

ArchimedeanChordsPattern &instance(orchestrator_handle *orch)
{
    static ArchimedeanChordsPattern s_instance(orch);
    return s_instance;
}

} // namespace

void register_archimedean_chords_pattern_plugin(orchestrator_handle *orch)
{
    orchestrator_register_plugin(orch, instance(orch).c_instance());
}

}} // namespace slic3r_api::Infill
