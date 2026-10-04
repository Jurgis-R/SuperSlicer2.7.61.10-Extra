///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#ifndef plugins_lightning_pattern_hpp_
#define plugins_lightning_pattern_hpp_
#include "LightningGenerator.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

namespace slic3r_api { namespace Infill { namespace Lightning {
// One setup generation owns all immutable object caches; there is no pipeline preparation service.
class LightningPattern : public PluginBase
{
public:
    explicit LightningPattern(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}
    static LightningPattern &instance(orchestrator_handle *orchestrator);

private:
    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    slicing_step_t step_impl() const noexcept override;
    const char *const *dependencies_impl() const noexcept override;
    int32_t priority_impl() const noexcept override;
    const char *progress_message_format_impl() const noexcept override;
    void inilialize_impl(storage_handle *storage) const override;
    void setup_impl(const plugin_run_context *run_ctx, uint32_t count) const override;
    void run_impl(const plugin_run_context *run_ctx) const override;
    mutable std::map<const object_handle *, std::unique_ptr<Cache>> m_caches;
};
void register_lightning_pattern_plugin(orchestrator_handle *orchestrator);
}}}
#endif
