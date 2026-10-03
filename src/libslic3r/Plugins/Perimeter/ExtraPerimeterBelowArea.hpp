///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
ExtraPerimeterBelowArea.hpp
===========================

Declares the extra perimeter below area module (id
"perimeter.module.extra_perimeter_below_area").

PerimeterGenerationModule plugins run inside a STEP_PERIMETER generator while it
builds the node tree. This module watches the child areas the generator creates
for the next inner region and asks it to keep producing perimeters whenever an
area is smaller than the configured threshold. The plugin owns
extra_perimeters_below_area and defines it, with its activation rule, in the
.cpp.
*/

#ifndef plugins_perimeter_extraperimeterbelowarea_hpp_
#define plugins_perimeter_extraperimeterbelowarea_hpp_

#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

namespace slic3r_api { namespace Perimeter { namespace ExtraPerimeterBelowAreaPlugin {

// PerimeterGenerationModule that keeps generating perimeters for small child areas.
//
// The module runs after a generator has created children for the next inner
// area. When a child area is smaller than the configured threshold, it
// asks the generator to keep producing many more perimeters for that branch.
class ExtraPerimeterBelowArea : public PluginBase
{
public:
    static ExtraPerimeterBelowArea &instance(orchestrator_handle *orch);

private:
    ExtraPerimeterBelowArea(orchestrator_handle *orch) : PluginBase(orch) {}

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

void register_extra_perimeter_below_area_plugin(orchestrator_handle *orch);

}}} // namespace slic3r_api::Perimeter::ExtraPerimeterBelowAreaPlugin

#endif // plugins_perimeter_extraperimeterbelowarea_hpp_
