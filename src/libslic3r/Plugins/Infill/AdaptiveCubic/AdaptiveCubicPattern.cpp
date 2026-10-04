///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Adaptive Cubic pattern plugin implementation
=============================================

setup() owns preparation and run() owns only per-surface clipping and
publication. The cache never stores borrowed surface handles.
*/

#include "AdaptiveCubicPattern.hpp"

#include <cassert>
#include <stdexcept>
#include <string>

#include "AdaptiveFill.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_infill.h"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Plugins/Infill/PatternPluginHelpers.hpp"

/*
Adaptive Cubic INFILL_PATTERN plugin
====================================

setup() builds one owned object cache for every object reported by the default
infill generator. run() only reads that immutable cache and publishes the
surface-local paths. Preparation never modifies the source surfaces.
*/

namespace slic3r_api { namespace Infill { namespace AdaptiveCubicPatternPlugin {
namespace {

const char *k_adaptive_id = "adaptivecubic";
const char *k_adaptive_name = "Adaptive Cubic";
const char *k_support_id = "supportcubic";
const char *k_support_name = "Support Cubic";
const char *k_no_dependencies[] = { nullptr };
const char *k_fill_pattern_key = "fill_pattern";

} // namespace

AdaptiveCubicPattern &AdaptiveCubicPattern::instance(orchestrator_handle *orch,
                                                    const AdaptiveCubic::CubicMode mode)
{
    static AdaptiveCubicPattern adaptive(orch,
                                        k_adaptive_id,
                                        k_adaptive_name,
                                        "Adaptive cubic infill pattern using an object-level octree.",
                                        15,
                                        AdaptiveCubic::CubicMode::Adaptive);
    static AdaptiveCubicPattern support(orch,
                                       k_support_id,
                                       k_support_name,
                                       "Support cubic infill pattern using upward-facing model facets.",
                                       16,
                                       AdaptiveCubic::CubicMode::Support);
    return mode == AdaptiveCubic::CubicMode::Adaptive ? adaptive : support;
}

const char *AdaptiveCubicPattern::id_impl() const noexcept
{
    return m_id;
}

const char *AdaptiveCubicPattern::name_impl() const noexcept
{
    return m_name;
}

const char *AdaptiveCubicPattern::description_impl() const noexcept
{
    return m_description;
}

slicing_step_t AdaptiveCubicPattern::step_impl() const noexcept
{
    return INFILL_PATTERN;
}

const char *const *AdaptiveCubicPattern::dependencies_impl() const noexcept
{
    return k_no_dependencies;
}

int32_t AdaptiveCubicPattern::priority_impl() const noexcept
{
    return m_priority;
}

void AdaptiveCubicPattern::inilialize_impl(storage_handle *) const
{
    PatternHelpers::add_self_to_pattern_list(m_orchestrator, m_id, m_name, { k_fill_pattern_key });
}

const char *AdaptiveCubicPattern::progress_message_format_impl() const noexcept
{
    return m_mode == AdaptiveCubic::CubicMode::Adaptive ?
        "Generating adaptive cubic infill: %u / %u surfaces" :
        "Generating support cubic infill: %u / %u surfaces";
}

void AdaptiveCubicPattern::setup_impl(const plugin_run_context *run_ctx, uint32_t run_count) const
{
    m_caches.clear();
    const run_ctx_infill_pattern_init *ctx = plugin_ctx_as_infill_pattern_init(run_ctx);
    if (ctx == nullptr || ctx->print == nullptr)
        return;

    throw_if_cancelled(run_ctx);
    const Print print(ctx->print);
    if (ctx->objects == nullptr)
        return;

    std::map<const object_handle *, std::unique_ptr<AdaptiveCubic::AdaptiveCache>> pending;
    for (uint32_t object_index = 0; object_index < ctx->object_count; ++object_index) {
        throw_if_cancelled(run_ctx);
        const raw_infill_pattern_object_init &object_init = ctx->objects[object_index];
        if (object_init.object == nullptr || object_init.assignment_count == 0 ||
            pending.find(object_init.object) != pending.end())
            continue;
        const Object object(object_init.object);
        pending.emplace(object_init.object,
                        AdaptiveCubic::prepare_cache(print, object, m_mode, run_ctx));
    }
    throw_if_cancelled(run_ctx);
    m_caches = std::move(pending);
    progress().add_max(run_count);
}

void AdaptiveCubicPattern::setup_run_impl(const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
}

void AdaptiveCubicPattern::run_impl(const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    const run_ctx_infill_pattern *ctx = plugin_ctx_as_infill_pattern(run_ctx);
    if (ctx == nullptr || ctx->object == nullptr || ctx->surface == nullptr ||
        ctx->params == nullptr || ctx->output == nullptr || run_ctx->plugin_storage == nullptr)
        return;

    const std::map<const object_handle *, std::unique_ptr<AdaptiveCubic::AdaptiveCache>>::const_iterator cache_it =
        m_caches.find(ctx->object);
    if (cache_it == m_caches.end() || cache_it->second == nullptr)
        throw std::runtime_error(std::string(m_name) + " object cache was not prepared before generation.");

    AdaptiveCubic::fill_surface(*ctx, *cache_it->second, run_ctx->plugin_storage, run_ctx);
    progress().increment();
}

void register_adaptive_cubic_pattern_plugin(orchestrator_handle *orch)
{
    orchestrator_register_plugin(
        orch, AdaptiveCubicPattern::instance(orch, AdaptiveCubic::CubicMode::Adaptive).c_instance());
}

void register_support_cubic_pattern_plugin(orchestrator_handle *orch)
{
    orchestrator_register_plugin(
        orch, AdaptiveCubicPattern::instance(orch, AdaptiveCubic::CubicMode::Support).c_instance());
}

}}} // namespace slic3r_api::Infill::AdaptiveCubicPatternPlugin
