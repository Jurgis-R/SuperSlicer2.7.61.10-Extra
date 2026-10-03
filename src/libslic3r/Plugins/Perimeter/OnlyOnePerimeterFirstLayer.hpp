///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
OnlyOnePerimeterFirstLayer.hpp
==============================

Declares the only one perimeter first layer module (id
"perimeter.module.only_one_perimeter_first_layer").

The module limits the first printed layer to a single perimeter. It runs after
the first perimeter ring exists and evaluates RegionSettings, so only the parts
of an island where a region or modifier enables the setting are collapsed, while
the generator keeps ownership of the tree split. The plugin owns
only_one_perimeter_first_layer and defines it, with its activation rule, in the
.cpp.
*/

#ifndef plugins_perimeter_onlyoneperimeterfirstlayer_hpp_
#define plugins_perimeter_onlyoneperimeterfirstlayer_hpp_

#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

namespace slic3r_api { namespace Perimeter { namespace OnlyOnePerimeterFirstLayerPlugin {

// PerimeterGenerationModule that limits the first layer to one perimeter.
//
// Applies "one perimeter on the first layer" after the first perimeter ring
// exists. Evaluating RegionSettings at that point splits only the parts of an
// island where a region or modifier enables the setting, instead of limiting
// the whole island from the start.
class OnlyOnePerimeterFirstLayer : public PluginBase
{
public:
    static OnlyOnePerimeterFirstLayer &instance(orchestrator_handle *orch);

private:
    OnlyOnePerimeterFirstLayer(orchestrator_handle *orch) : PluginBase(orch) {}

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

void register_only_one_perimeter_first_layer_plugin(orchestrator_handle *orch);

}}} // namespace slic3r_api::Perimeter::OnlyOnePerimeterFirstLayerPlugin

#endif // plugins_perimeter_onlyoneperimeterfirstlayer_hpp_
