///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
ExtraPerimeterOddLayer.hpp
==========================

Declares the extra perimeter odd layer module (id
"perimeter.module.extra_perimeter_odd_layer").

While a STEP_PERIMETER generator builds its node tree, the module raises the
requested perimeter count by one on odd layer numbers. It only adjusts the
requested count; the generator still creates the extrusion loops. The plugin
owns extra_perimeters_odd_layers and defines it, with its activation rule, in
the .cpp.
*/

#ifndef plugins_perimeter_extraperimeteroddlayer_hpp_
#define plugins_perimeter_extraperimeteroddlayer_hpp_

#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

namespace slic3r_api { namespace Perimeter { namespace ExtraPerimeterOddLayerPlugin {

// PerimeterGenerationModule that adds one perimeter on odd layer numbers.
//
// The module is called by a STEP_PERIMETER generator while its perimeter-node
// tree is being processed. It only adjusts the requested perimeter count; the
// generator remains responsible for creating the actual extrusion loops.
class ExtraPerimeterOddLayer : public PluginBase
{
public:
    static ExtraPerimeterOddLayer &instance(orchestrator_handle *orch);

private:
    ExtraPerimeterOddLayer(orchestrator_handle *orch) : PluginBase(orch) {}

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

void register_extra_perimeter_odd_layer_plugin(orchestrator_handle *orch);

}}} // namespace slic3r_api::Perimeter::ExtraPerimeterOddLayerPlugin

#endif // plugins_perimeter_extraperimeteroddlayer_hpp_
