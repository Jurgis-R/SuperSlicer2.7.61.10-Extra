///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Adaptive Cubic pattern plugin
=============================

The plugin prepares one object-scoped cache during setup() and reuses it for
each surface dispatched by the default infill generator.
*/

#ifndef plugins_infill_adaptivecubic_adaptivecubicpattern_hpp_
#define plugins_infill_adaptivecubic_adaptivecubicpattern_hpp_

#include <map>
#include <memory>

#include "AdaptivePreparation.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

namespace slic3r_api { namespace Infill { namespace AdaptiveCubicPatternPlugin {

class AdaptiveCubicPattern : public PluginBase
{
public:
    static AdaptiveCubicPattern &instance(orchestrator_handle *orch, AdaptiveCubic::CubicMode mode);

private:
    AdaptiveCubicPattern(orchestrator_handle *orch,
                         const char *id,
                         const char *name,
                         const char *description,
                         int32_t priority,
                         AdaptiveCubic::CubicMode mode)
        : PluginBase(orch), m_id(id), m_name(name), m_description(description), m_priority(priority), m_mode(mode) {}

    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    slicing_step_t step_impl() const noexcept override;
    const char *const *dependencies_impl() const noexcept override;
    int32_t priority_impl() const noexcept override;
    void inilialize_impl(storage_handle *storage) const override;
    const char *progress_message_format_impl() const noexcept override;
    void setup_impl(const plugin_run_context *run_ctx, uint32_t run_count) const override;
    void setup_run_impl(const plugin_run_context *run_ctx) const override;
    void run_impl(const plugin_run_context *run_ctx) const override;

    const char *const m_id;
    const char *const m_name;
    const char *const m_description;
    const int32_t m_priority;
    const AdaptiveCubic::CubicMode m_mode;
    mutable std::map<const object_handle *, std::unique_ptr<AdaptiveCubic::AdaptiveCache>> m_caches;
};

void register_adaptive_cubic_pattern_plugin(orchestrator_handle *orch);
void register_support_cubic_pattern_plugin(orchestrator_handle *orch);

}}} // namespace slic3r_api::Infill::AdaptiveCubicPatternPlugin

#endif // plugins_infill_adaptivecubic_adaptivecubicpattern_hpp_
