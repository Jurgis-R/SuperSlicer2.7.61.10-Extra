///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
OnlyOnePerimeterOnTop.hpp
=========================

Declares the only one perimeter on top module (id
"perimeter.module.only_one_perimeter_on_top").

While a STEP_PERIMETER generator builds its node tree, the module detects
top-facing parts of the current island and asks the generator to stop the
matching branches after one perimeter. The tree split stays owned by the
generator through the perimeter-node callback. The plugin owns
min_width_top_surface and only_one_perimeter_top_other_algo and defines them,
with their activation rules, in the .cpp.
*/

#ifndef plugins_perimeter_onlyoneperimeterontop_hpp_
#define plugins_perimeter_onlyoneperimeterontop_hpp_

#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

namespace slic3r_api { namespace Perimeter { namespace OnlyOnePerimeterOnTopPlugin {

// PerimeterGenerationModule that stops top-facing branches after one perimeter.
//
// The module detects top-facing parts of the current island and asks the active
// perimeter generator to stop the matching branches after one perimeter. The
// actual tree split remains owned by the generator through the perimeter-node
// split callback.
class OnlyOnePerimeterOnTop : public PluginBase
{
public:
    static OnlyOnePerimeterOnTop &instance(orchestrator_handle *orch);

private:
    OnlyOnePerimeterOnTop(orchestrator_handle *orch) : PluginBase(orch) {}

    const char *id_impl() const noexcept override;
    slicing_step_t step_impl() const noexcept override;
    const char *const *dependencies_impl() const noexcept override;
    int32_t priority_impl() const noexcept override;
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    int32_t defined_config_keys(const char **keys) const noexcept override;
    void inilialize_impl(storage_handle *) const override;
    const char *progress_message_format_impl() const noexcept override;
    void run_impl(const plugin_run_context *run_ctx) const override;
};

void register_only_one_perimeter_on_top_plugin(orchestrator_handle *orch);

}}} // namespace slic3r_api::Perimeter::OnlyOnePerimeterOnTopPlugin

#endif // plugins_perimeter_onlyoneperimeterontop_hpp_
