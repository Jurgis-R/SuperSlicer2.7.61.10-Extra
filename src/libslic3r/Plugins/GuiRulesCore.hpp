///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef plugins_guirulescore_hpp_
#define plugins_guirulescore_hpp_

#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

namespace slic3r_api { namespace GuiRulesCorePlugin {

// Built-in plugin that owns the GUI activation rules of the built-in print
// settings. It does not run in the slicing pipeline: registering the rules is
// its only job. Rules of settings implemented by a feature plugin are meant to
// move to that plugin; this one keeps the remaining built-in logic until it is
// distributed.
// Tower controls distinguish absolute global positions from offsets shared by
// every instance of an object; sequential printing permits instance towers.
class GuiRulesCore : public PluginBase
{
public:
    static GuiRulesCore &instance(orchestrator_handle *orch);

private:
    GuiRulesCore(orchestrator_handle *orch) : PluginBase(orch) {}

    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    slicing_step_t step_impl() const noexcept override;
    const char *const *dependencies_impl() const noexcept override;
    int32_t priority_impl() const noexcept override;
    // Register activation rules for built-in FFF settings, including the per-instance wipe tower controls.
    void inilialize_impl(storage_handle *storage) const override;
    void run_impl(const plugin_run_context *run_ctx) const override;
};

// Register the singleton plugin in an orchestrator or external plugin loader.
void register_gui_rules_core_plugin(orchestrator_handle *orch);

}} // namespace slic3r_api::GuiRulesCorePlugin

#endif // plugins_guirulescore_hpp_
