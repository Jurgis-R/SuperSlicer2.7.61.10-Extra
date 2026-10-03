///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef plugins_nozzle_cleaning_hpp_
#define plugins_nozzle_cleaning_hpp_

/* STEP_EXTRUSION_EDIT plugin that tracks per-tool nozzle-cleaning intervals
   and inserts the configured script at an allowed ordered print transition. */

#include <cstdint>

#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PluginPropertyKey.hpp"

namespace slic3r_api { namespace NozzleCleaningPlugin {

struct NozzleCleaningGeneratedProperty
{
    uint8_t generated = 1;
};

class NozzleCleaning : public PluginBase
{
public:
    static NozzleCleaning &instance(orchestrator_handle *orchestrator);
    static const char *print_ui_fragment() noexcept;

private:
    explicit NozzleCleaning(orchestrator_handle *orchestrator);

    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    slicing_step_t step_impl() const noexcept override;
    const char *const *dependencies_impl() const noexcept override;
    int32_t priority_impl() const noexcept override;
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    int32_t defined_config_keys(const char **keys) const noexcept override;
    const char *progress_message_format_impl() const noexcept override;
    void inilialize_impl(storage_handle *storage) const override;
    void run_impl(const plugin_run_context *run_ctx) const override;

    PluginPropertyKey<NozzleCleaningGeneratedProperty> m_generated_property;
};

void register_nozzle_cleaning_plugin(orchestrator_handle *orchestrator);

}} // namespace slic3r_api::NozzleCleaningPlugin

#endif // plugins_nozzle_cleaning_hpp_
