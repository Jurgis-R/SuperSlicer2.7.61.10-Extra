#include <catch2/catch.hpp>

#include <initializer_list>
#include <set>
#include <string>
#include <vector>

#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_infill.h"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepGenerateInfill.hpp"
#include "libslic3r/Steps/StepGeneratePerimeter.hpp"
#include "libslic3r/Steps/StepSurfaceGeneration.hpp"

/*
Infill pattern discovery and initialization
===========================================

The default STEP_INFILL generator performs a discovery pass during its
setup_run() phase: it resolves every fill surface recipe, groups the surfaces
that share one recipe and reports the groups through
run_ctx_generate_infill::report_pattern_assignments(). The host turns that into
one run_ctx_infill_pattern_init payload per pattern plugin.

These tests drive the generator directly with recording callbacks so the
discovery contract can be observed without a real pattern plugin: grouping,
per-surface pattern calls, and the cached replay in run().
*/

using namespace Slic3r;
using namespace Slic3r::Test::PerimeterPluginTests;

namespace {

constexpr infill_pattern_runtime_id k_gyroid_runtime_id = 7;

// Local copy of the perimeter test scope helper so the infill test can restrict
// the active plugin set without touching the shared helper.
class ScopedActivePlugins
{
public:
    explicit ScopedActivePlugins(std::initializer_list<const char *> plugin_ids) :
        m_orchestrator(Orchestrator::instance())
    {
        m_previous_active_plugins.reserve(m_orchestrator.active_plugins().size());
        for (Plugin *plugin : m_orchestrator.active_plugins())
            m_previous_active_plugins.push_back(plugin);

        m_orchestrator.clear_active_plugins();
        for (const char *plugin_id : plugin_ids)
            REQUIRE(m_orchestrator.set_plugin_active(plugin_id, true));
    }

    ~ScopedActivePlugins()
    {
        m_orchestrator.clear_active_plugins();
        for (Plugin *plugin : m_previous_active_plugins)
            m_orchestrator.set_plugin_active(plugin, true);
    }

private:
    Orchestrator &m_orchestrator;
    std::vector<Plugin *> m_previous_active_plugins;
};

DynamicPrintConfig gyroid_test_config()
{
    return perimeter_config({
        { "fill_pattern", "gyroid" },
        { "fill_density", "20%" },
        { "top_solid_layers", "0" },
        { "bottom_solid_layers", "0" },
        { "solid_infill_every_layers", "0" },
        { "infill_every_layers", "1" }
    });
}

struct DiscoveryRecorder
{
    infill_pattern_runtime_id reported_pattern_id = INFILL_PATTERN_RUNTIME_ID_INVALID;
    size_t report_calls = 0;
    size_t reported_groups = 0;
    size_t reported_surfaces = 0;
    bool invalid_assignment = false;
    bool all_reported_params_used = true;
    std::set<const raw_infill_pattern_params *> reported_params;
    size_t generate_calls = 0;
    size_t append_calls = 0;
};

int32_t record_report(const run_ctx_generate_infill *ctx,
                      infill_pattern_runtime_id pattern_id,
                      const raw_infill_pattern_assignment *assignments,
                      uint32_t assignment_count)
{
    DiscoveryRecorder *recorder = static_cast<DiscoveryRecorder *>(ctx->host_context);
    recorder->reported_pattern_id = pattern_id;
    ++recorder->report_calls;
    for (uint32_t index = 0; index < assignment_count; ++index) {
        const raw_infill_pattern_assignment &assignment = assignments[index];
        if (assignment.layer_island == nullptr || assignment.region_island == nullptr ||
            assignment.params == nullptr || assignment.surfaces == nullptr || assignment.surface_count == 0) {
            recorder->invalid_assignment = true;
            continue;
        }
        if (assignment.params->pattern_id != pattern_id || assignment.params->density <= 0.f)
            recorder->invalid_assignment = true;
        for (uint32_t surface = 0; surface < assignment.surface_count; ++surface)
            if (assignment.surfaces[surface] == nullptr)
                recorder->invalid_assignment = true;
        ++recorder->reported_groups;
        recorder->reported_surfaces += assignment.surface_count;
        recorder->reported_params.insert(assignment.params);
    }
    return 1;
}

infill_pattern_runtime_id record_resolve_pattern_id(const run_ctx_generate_infill *, const char *plugin_id)
{
    return plugin_id != nullptr && std::string(plugin_id) == "gyroid" ? k_gyroid_runtime_id :
                                                                       INFILL_PATTERN_RUNTIME_ID_INVALID;
}

const char *record_pattern_plugin_id(const run_ctx_generate_infill *, infill_pattern_runtime_id pattern_id)
{
    return pattern_id == k_gyroid_runtime_id ? "gyroid" : "";
}

int32_t record_modify_surface_recipe(const run_ctx_generate_infill *,
                                     const layer_handle *,
                                     const layer_island_handle *,
                                     const layer_region_island_handle *,
                                     const layer_region_handle *,
                                     const surface_handle *,
                                     const expolygon_collection_handle *,
                                     raw_infill_pattern_params *)
{
    return 1;
}

int32_t record_generate_pattern(const run_ctx_generate_infill *ctx,
                                infill_pattern_runtime_id pattern_id,
                                const layer_handle *,
                                const layer_island_handle *,
                                const layer_region_island_handle *,
                                const layer_region_handle *,
                                const surface_handle *,
                                const expolygon_collection_handle *,
                                const raw_infill_pattern_params *params,
                                extrusion_entity_handle *)
{
    DiscoveryRecorder *recorder = static_cast<DiscoveryRecorder *>(ctx->host_context);
    ++recorder->generate_calls;
    if (pattern_id != k_gyroid_runtime_id || params == nullptr ||
        recorder->reported_params.find(params) == recorder->reported_params.end())
        recorder->all_reported_params_used = false;
    // Return without extrusion: the test only observes the call protocol.
    return 0;
}

int32_t record_append_region_island_extrusion(layer_region_island_handle *, raw_extrusion_role, extrusion_entity_handle *)
{
    return 1;
}

} // namespace

TEST_CASE("STEP_INFILL discovery groups surfaces and replays them from the cache",
          "[plugins][infill][init]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, gyroid_test_config());

    ScopedActivePlugins active({
        SIMPLE_PERIMETER_GENERATOR,
        INITIAL_TYPED_SURFACE_BUILDER,
        "infill.generator.default",
        "gyroid"
    });

    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepGeneratePerimeter::clean_and_prepare(prepared.print);
    Steps::StepGeneratePerimeter::run_step(orchestrator, prepared.print);
    Steps::StepSurfaceGeneration::clean_and_prepare(prepared.print);
    Steps::StepSurfaceGeneration::run_step(orchestrator, prepared.print);

    Plugin *generator = orchestrator.get_plugin("infill.generator.default");
    REQUIRE(generator != nullptr);

    DiscoveryRecorder recorder;
    plugin_host_context host;
    host.orchestrator = &orchestrator;
    host.print = &prepared.print;
    host.plugin = generator;
    host.step = STEP_INFILL;
    plugin_run_context run_ctx = orchestrator.prepare_plugin_run_context(STEP_INFILL, generator, &host);

    run_ctx_generate_infill payload = {};
    payload.print = reinterpret_cast<const print_handle *>(&prepared.print);
    payload.object = reinterpret_cast<const object_handle *>(&prepared.print.object(0));
    payload.host_context = &recorder;
    payload.resolve_pattern_id = &record_resolve_pattern_id;
    payload.pattern_plugin_id = &record_pattern_plugin_id;
    payload.generate_pattern = &record_generate_pattern;
    payload.modify_surface_recipe = &record_modify_surface_recipe;
    payload.append_region_island_extrusion = &record_append_region_island_extrusion;
    payload.report_pattern_assignments = &record_report;
    run_ctx.data = &payload;

    generator->setup(run_ctx, 1);
    generator->setup_run(run_ctx);

    // Discovery must report at least one complete group for the used pattern.
    CHECK_FALSE(recorder.invalid_assignment);
    CHECK(recorder.report_calls > 0);
    CHECK(recorder.reported_groups > 0);
    CHECK(recorder.reported_surfaces > 0);
    CHECK(recorder.reported_pattern_id == k_gyroid_runtime_id);

    // The run phase replays the discovery result: one pattern call per reported
    // surface, with the exact params pointer that was reported.
    const size_t discovered_surfaces = recorder.reported_surfaces;
    generator->run(run_ctx);
    CHECK(recorder.generate_calls == discovered_surfaces);
    CHECK(recorder.all_reported_params_used);
    CHECK(recorder.append_calls == 0);

    // The cache is consumed by the run. A later run falls back to rebuilding
    // the recipes and still produces the same pattern calls.
    recorder.generate_calls = 0;
    generator->run(run_ctx);
    CHECK(recorder.generate_calls == discovered_surfaces);
}
