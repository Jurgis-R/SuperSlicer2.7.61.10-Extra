///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef plugins_infill_postinfillgapfill_hpp_
#define plugins_infill_postinfillgapfill_hpp_

#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

namespace slic3r_api { namespace Infill { namespace PostInfillGapFillPlugin {

// Placeholder STEP_POST_INFILL plugin for the future gap-fill pass.
//
// Gap fill used to be requested directly from some Fill implementations. The
// new pipeline gives it a dedicated post-infill slot so it can inspect already
// generated infill and create residual narrow extrusion as a separate pass.
//
// FIXME: publish future gap fill as a child of the source surface's infill
// subtree so it inherits c_extrusion_property_infill, instead of the separate
// GAP_FILL bucket. infill.post_process.full_fill_volume then measures fill and
// gap fill together, like the legacy Fill accounting did.
class PostInfillGapFill : public PluginBase
{
public:
    static PostInfillGapFill &instance(orchestrator_handle *orch);

private:
    PostInfillGapFill(orchestrator_handle *orch) : PluginBase(orch) {}

    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    slicing_step_t step_impl() const noexcept override;
    const char *const *dependencies_impl() const noexcept override;
    int32_t priority_impl() const noexcept override;
    void run_impl(const plugin_run_context *run_ctx) const override;
};

void register_post_infill_gap_fill_plugin(orchestrator_handle *orch);

}}} // namespace slic3r_api::Infill::PostInfillGapFillPlugin

#endif // plugins_infill_postinfillgapfill_hpp_
