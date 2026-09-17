
///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "StepSkirtBrim.hpp"

#include <atomic>
#include <cassert>
#include <memory>
#include <vector>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/internal/PrintAccess.hpp"
#include "libslic3r/Api/internal/PrintObjectAccess.hpp"
#include "libslic3r/Api/plugin/c/slic3r_data_tree.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_skirt_brim.h"
#include "libslic3r/Exception.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "libslic3r/Steps/StepRunner.hpp"

namespace Slic3r::Steps::StepSkirtBrim {
namespace {

run_ctx_skirt_brim payload_for_print(Print &print)
{
    run_ctx_skirt_brim payload = {};
    payload.print = reinterpret_cast<print_handle *>(&print);
    return payload;
}

} // namespace

void clean_and_prepare(Print &print)
{
    /*
    STEP_SKIRT_BRIM is append-oriented: each active plugin publishes the brim
    pieces it owns. Clear previous brim output once before the chain starts so
    disabled plugins cannot leave stale adhesion geometry behind.
    */
    ApiInternal::PrintAccess::clear_brim(print);
    ApiInternal::PrintAccess::clear_skirt(print);
}

bool validate_pre(const Print &, std::string *)
{
    return true;
}

bool validate_post(const Print &, std::string *)
{
    return true;
}

void run_step(Orchestrator &orchestrator, Print &print)
{
    std::vector<Plugin *> plugins =
        selected_or_active_plugins_for_step(orchestrator, STEP_SKIRT_BRIM, &print.full_print_config());
    run_ctx_skirt_brim payload = payload_for_print(print);

    for (Plugin *plugin : plugins) {
        if (plugin == nullptr)
            continue;

        /*
        Each skirt/brim plugin runs once for the whole print. A plugin that
        needs per-object behavior can iterate print.objects() itself; this
        keeps global-vs-object brim decisions inside the generator.
        */
        std::atomic_bool execution_error { false };
        plugin_host_context host_context =
            Detail::prepare_guarded_plugin_host_context(
                orchestrator, STEP_SKIRT_BRIM, *plugin, print, execution_error);
        host_context.object_count = print.objects().size();
        plugin_run_context run_context =
            orchestrator.prepare_plugin_run_context(STEP_SKIRT_BRIM, plugin, &host_context);
        run_context.data = &payload;

        plugin->setup(run_context, 1);
        Detail::throw_if_plugin_stopped(orchestrator, STEP_SKIRT_BRIM, *plugin, execution_error, "setup");
        plugin->setup_run(run_context);
        Detail::throw_if_plugin_stopped(orchestrator, STEP_SKIRT_BRIM, *plugin, execution_error, "setup_run");
        plugin->run(run_context);
        Detail::throw_if_plugin_stopped(orchestrator, STEP_SKIRT_BRIM, *plugin, execution_error, "run");
    }

    ApiInternal::PrintAccess::normalize_skirt_brim_direction(print);
    ApiInternal::PrintAccess::rebuild_first_layer_convex_hull_after_skirt_brim(print);
}

} // namespace Slic3r::Steps::StepSkirtBrim
