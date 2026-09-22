///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
ExtraPerimeterCount.hpp
=======================

Declares the extra perimeter count module (id
"perimeter.module.extra_perimeter_count").

The module raises the requested perimeter count while a STEP_PERIMETER
generator builds its node tree. A uniform value increases the root count
directly; when the value differs between regions or modifiers, the module asks
the generator to split the matching child areas and adds one pass per child
until every branch reaches its configured count. The plugin owns
extra_perimeters_count and defines it, with its activation rule, in the .cpp.
*/

#ifndef plugins_perimeter_extraperimetercount_hpp_
#define plugins_perimeter_extraperimetercount_hpp_

#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

namespace slic3r_api { namespace Perimeter { namespace ExtraPerimeterCountPlugin {

// PerimeterGenerationModule that raises the requested perimeter count.
//
// In the uniform case it simply increases the root perimeter count. When the
// value differs between regions/modifiers, it asks the generator to split child
// nodes by the active areas and requests one additional perimeter pass for the
// matching children until each branch reaches the configured count.
class ExtraPerimeterCount : public PluginBase
{
public:
    static ExtraPerimeterCount &instance(orchestrator_handle *orch);

private:
    ExtraPerimeterCount(orchestrator_handle *orch) : PluginBase(orch) {}

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

void register_extra_perimeter_count_plugin(orchestrator_handle *orch);

}}} // namespace slic3r_api::Perimeter::ExtraPerimeterCountPlugin

#endif // plugins_perimeter_extraperimetercount_hpp_
