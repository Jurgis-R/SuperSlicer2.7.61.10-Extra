///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef plugins_infill_postfullfillvolume_hpp_
#define plugins_infill_postfullfillvolume_hpp_

#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"

namespace slic3r_api { namespace Infill { namespace PostFullFillVolumePlugin {

// STEP_POST_INFILL plugin that corrects the flow of 100% infill surfaces.
//
// The legacy Fill implementations adjusted the flow of an enforced full fill
// volume themselves. The new pipeline keeps pattern plugins purely geometric:
// STEP_INFILL stamps the raw fill recipe and the area to fill on the subtree
// root of every surface, and this pass turns them into one flow multiplier per
// subtree, exactly like the legacy Fill::fill_surface_extrusion() did.
class PostFullFillVolume : public PluginBase
{
public:
    static PostFullFillVolume &instance(orchestrator_handle *orch);

private:
    PostFullFillVolume(orchestrator_handle *orch) : PluginBase(orch) {}

    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    slicing_step_t step_impl() const noexcept override;
    const char *const *dependencies_impl() const noexcept override;
    int32_t priority_impl() const noexcept override;
    const char *progress_message_format_impl() const noexcept override;
    void setup_run_impl(const plugin_run_context *run_ctx) const override;
    void run_impl(const plugin_run_context *run_ctx) const override;
};

void register_post_full_fill_volume_plugin(orchestrator_handle *orch);

}}} // namespace slic3r_api::Infill::PostFullFillVolumePlugin

#endif // plugins_infill_postfullfillvolume_hpp_
