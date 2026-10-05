///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef plugins_infill_gyroidpattern_hpp_
#define plugins_infill_gyroidpattern_hpp_

#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

namespace slic3r_api { namespace Infill { namespace GyroidPatternPlugin {

// Standalone gyroid INFILL_PATTERN plugin.
//
// It is the reference migration of a legacy Fill algorithm to the public plugin
// API: the algorithm only reads the per-surface recipe and geometry through the
// C ABI and the C++ view helpers, then publishes extrusion leaves into the host
// output collection. It does not include any internal slicer header.
class GyroidPattern : public PluginBase
{
public:
    static GyroidPattern &instance(orchestrator_handle *orch);

private:
    GyroidPattern(orchestrator_handle *orch) : PluginBase(orch) {}

    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    slicing_step_t step_impl() const noexcept override;
    const char *const *dependencies_impl() const noexcept override;
    int32_t priority_impl() const noexcept override;
    void inilialize_impl(storage_handle *storage) const override;
    const char *progress_message_format_impl() const noexcept override;
    void run_impl(const plugin_run_context *run_ctx) const override;
};

void register_gyroid_pattern_plugin(orchestrator_handle *orch);

}}} // namespace slic3r_api::Infill::GyroidPatternPlugin

#endif // plugins_infill_gyroidpattern_hpp_
