///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

// Setup publishes complete object caches before surface workers read their immutable layer trees.
#include "LightningPattern.hpp"
#include "LightningFill.hpp"
#include <set>
#include <stdexcept>
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/ParallelFor.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Plugins/Infill/PatternPluginHelpers.hpp"

namespace slic3r_api { namespace Infill { namespace Lightning {

LightningPattern &LightningPattern::instance(orchestrator_handle *orchestrator)
{
    static LightningPattern pattern(orchestrator);
    return pattern;
}
const char *LightningPattern::id_impl() const noexcept { return "lightning"; }
const char *LightningPattern::name_impl() const noexcept { return "Lightning"; }
const char *LightningPattern::description_impl() const noexcept { return "Lightning infill with setup-owned layer trees."; }
slicing_step_t LightningPattern::step_impl() const noexcept { return INFILL_PATTERN; }
const char *const *LightningPattern::dependencies_impl() const noexcept
{
    static const char *none[] = {nullptr};
    return none;
}
int32_t LightningPattern::priority_impl() const noexcept { return 23; }
const char *LightningPattern::progress_message_format_impl() const noexcept { return "Generating lightning infill: %u / %u surfaces"; }

void LightningPattern::inilialize_impl(storage_handle *) const
{
    PatternHelpers::add_self_to_pattern_list(m_orchestrator, "lightning", "Lightning", {"fill_pattern"});
}

void LightningPattern::setup_impl(const plugin_run_context *run_ctx, uint32_t count) const
{
    m_caches.clear();
    const run_ctx_infill_pattern_init *init = plugin_ctx_as_infill_pattern_init(run_ctx);
    if (init == nullptr || init->print == nullptr || init->objects == nullptr)
        return;
    struct Job { const object_handle *object; std::unique_ptr<Cache> cache; };
    std::vector<Job> jobs;
    std::set<const object_handle *> assigned;
    for (uint32_t index = 0; index < init->object_count; ++index) {
        const raw_infill_pattern_object_init &object = init->objects[index];
        if (object.object != nullptr && object.assignment_count > 0 && assigned.insert(object.object).second)
            jobs.push_back({object.object, {}});
    }
    const Print print(init->print);
    // Workers fill distinct slots using isolated scratch; persistent nodes own ordinary C++ memory.
    parallel_for_storage_with_progress(0, uint32_t(jobs.size()), run_ctx, nullptr,
        [&](uint32_t index, storage_handle *scratch) {
            const Inputs inputs = read_inputs(print, Object(jobs[index].object), scratch, run_ctx);
            jobs[index].cache = build_cache(inputs, scratch, run_ctx);
        });
    throw_if_cancelled(run_ctx);
    std::map<const object_handle *, std::unique_ptr<Cache>> pending;
    for (Job &job : jobs)
        pending.emplace(job.object, std::move(job.cache));
    m_caches = std::move(pending);
    progress().add_max(count);
}

void LightningPattern::run_impl(const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    const run_ctx_infill_pattern *ctx = plugin_ctx_as_infill_pattern(run_ctx);
    if (ctx == nullptr || ctx->object == nullptr || ctx->layer == nullptr || ctx->surface == nullptr ||
        ctx->params == nullptr || ctx->output == nullptr || run_ctx->plugin_storage == nullptr)
        return;
    const std::map<const object_handle *, std::unique_ptr<Cache>>::const_iterator cache = m_caches.find(ctx->object);
    if (cache == m_caches.end())
        throw std::runtime_error("Lightning object cache was not prepared during setup.");
    fill_surface(*ctx, *cache->second, run_ctx->plugin_storage, run_ctx);
    progress().increment();
}

void register_lightning_pattern_plugin(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(orchestrator, LightningPattern::instance(orchestrator).c_instance());
}
}}}
