
#include "StepExtrusionOrdering.hpp"

#include <algorithm>
#include <sstream>
#include <string>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_ordering.h"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Steps/StepRunner.hpp"

namespace Slic3r::Steps::StepExtrusionOrdering {

namespace {

enum class SourceObjectStatus {
    NoSource,
    Valid,
    Invalid
};

struct SourceObjectResult {
    SourceObjectStatus status;
    const PrintObject *object;
};

/* Distinguish an absent source from an island whose source context is corrupt. */
SourceObjectResult source_object(const LayerRegionIsland *region_island)
{
    if (region_island == nullptr)
        return { SourceObjectStatus::NoSource, nullptr };
    if (region_island->regions().empty())
        return { SourceObjectStatus::Invalid, nullptr };

    const PrintObject *object = nullptr;
    for (const LayerRegion *region : region_island->regions()) {
        if (region == nullptr || region->layer() == nullptr || region->layer()->object() == nullptr)
            return { SourceObjectStatus::Invalid, nullptr };
        const PrintObject *candidate = region->layer()->object();
        if (object != nullptr && object != candidate)
            return { SourceObjectStatus::Invalid, nullptr };
        object = candidate;
    }
    return object == nullptr ?
        SourceObjectResult{ SourceObjectStatus::Invalid, nullptr } :
        SourceObjectResult{ SourceObjectStatus::Valid, object };
}

/* Format one post-ordering contract violation without hiding its tree path. */
bool validation_failure(std::string *error,
                        const std::string &path,
                        const std::string &cause)
{
    if (error != nullptr)
        *error = path + ": " + cause;
    return false;
}

/* Describe the catalog state used by an extrusion in a diagnostic. */
const char *instance_kind(const Printing::PrintingObjectInstance *instance,
                          const uint32_t printing_catalog_id)
{
    if (printing_catalog_id == Printing::PRINTING_OBJECT_INSTANCE_ID_NONE)
        return "NONE";
    return instance != nullptr && instance->object != nullptr ? "real" : "synthetic";
}

} // namespace

void clean_and_prepare(Print &print)
{
    print.reset_printing_plan();
}

bool validate_pre(const Print &, std::string *)
{
    return true;
}

bool validate_post(const Print &print, std::string *error)
{
    const Printing::PrintingPlan *plan = print.printing_plan();
    if (plan == nullptr)
        return validation_failure(error, "plan", "the PrintingPlan is missing");

    for (size_t object_instance_idx = 0; object_instance_idx < plan->object_instances.size(); ++object_instance_idx) {
        const Printing::PrintingObjectInstance &instance = plan->object_instances[object_instance_idx];
        const std::string path = "plan.catalog[" + std::to_string(object_instance_idx) + "]";
        if (instance.object != nullptr) {
            if (instance.object->print() != &print)
                return validation_failure(error, path, "real catalog object belongs to another Print");
            if (instance.object_instance_idx >= instance.object->instances().size())
                return validation_failure(error, path, "real catalog instance index is out of range");
        } else {
            if (instance.object_instance_idx != size_t(-1))
                return validation_failure(error, path, "synthetic catalog entry has a real instance index");
            if (instance.name.empty() || instance.projected_outline.empty() ||
                instance.projected_outline.area() == 0.0)
                return validation_failure(error, path, "synthetic catalog metadata is incomplete");
        }
    }

    for (size_t group_idx = 0; group_idx < plan->groups.size(); ++group_idx) {
        const Printing::PrintingGroup &group = plan->groups[group_idx];
        const std::string group_path = "group[" + std::to_string(group_idx) + "]";
        if (group.plan_identity != plan->identity_token())
            return validation_failure(error, group_path, "group belongs to another PrintingPlan");
        for (const Printing::PrintingObjectInstanceId printing_catalog_id : group.printing_catalog_ids) {
            if (printing_catalog_id == Printing::PRINTING_OBJECT_INSTANCE_ID_NONE ||
                printing_catalog_id >= plan->object_instances.size())
                return validation_failure(error, group_path,
                                          "printing catalog ID is not a valid catalog entry");
        }

        for (size_t layer_idx = 0; layer_idx < group.layers.size(); ++layer_idx) {
            const Printing::PrintingLayerGroup &layer = group.layers[layer_idx];
            const std::string layer_path = group_path + ".layer[" + std::to_string(layer_idx) + "]";
            if (layer.plan_identity != plan->identity_token())
                return validation_failure(error, layer_path, "layer group belongs to another PrintingPlan");

            for (size_t tool_idx = 0; tool_idx < layer.tool_groups.size(); ++tool_idx) {
                const Printing::PrintingToolGroup &tool = layer.tool_groups[tool_idx];
                const std::string tool_path = layer_path + ".tool[" + std::to_string(tool_idx) + "]";
                if (tool.plan_identity != plan->identity_token())
                    return validation_failure(error, tool_path, "tool group belongs to another PrintingPlan");

                for (size_t extrusion_idx = 0; extrusion_idx < tool.extrusions.size(); ++extrusion_idx) {
                    const Printing::PrintingExtrusion &extrusion = tool.extrusions[extrusion_idx];
                    const std::string extrusion_path =
                        tool_path + ".extrusion[" + std::to_string(extrusion_idx) + "]";
                    if (extrusion.root == nullptr)
                        return validation_failure(error, extrusion_path, "root is not owned by the plan");

                    const uint32_t printing_catalog_id = extrusion.printing_catalog_id;
                    const Printing::PrintingObjectInstance *instance =
                        printing_catalog_id == Printing::PRINTING_OBJECT_INSTANCE_ID_NONE ?
                            nullptr : plan->object_instance(printing_catalog_id);
                    std::ostringstream details;
                    details << extrusion_path << " (instance="
                            << instance_kind(instance, printing_catalog_id)
                            << ", id=" << printing_catalog_id << ")";
                    if (printing_catalog_id != Printing::PRINTING_OBJECT_INSTANCE_ID_NONE && instance == nullptr)
                        return validation_failure(error, details.str(), "extrusion ID is not in the plan catalog");

                    const SourceObjectResult source = source_object(extrusion.region_island);
                    if (source.status == SourceObjectStatus::Invalid)
                        return validation_failure(error, details.str(), "extrusion source region island is invalid");
                    if (source.status == SourceObjectStatus::Valid && source.object->print() != &print)
                        return validation_failure(error, details.str(), "extrusion source belongs to another Print");
                    if (instance != nullptr && instance->object != nullptr &&
                        (source.status != SourceObjectStatus::Valid || source.object != instance->object))
                        return validation_failure(error, details.str(),
                                                  "real instance does not match the extrusion source PrintObject");
                    if (instance != nullptr && instance->object == nullptr && source.status == SourceObjectStatus::Valid &&
                        source.object != print.auxiliary_object())
                        return validation_failure(error, details.str(),
                                                  "synthetic instance is associated with a real object region");
                    if (printing_catalog_id == Printing::PRINTING_OBJECT_INSTANCE_ID_NONE &&
                        source.status == SourceObjectStatus::Valid && source.object != print.auxiliary_object())
                        return validation_failure(error, details.str(),
                                                  "NONE instance is associated with a real object region");

                    if (printing_catalog_id != Printing::PRINTING_OBJECT_INSTANCE_ID_NONE &&
                        std::find(group.printing_catalog_ids.begin(), group.printing_catalog_ids.end(), printing_catalog_id) ==
                            group.printing_catalog_ids.end())
                        return validation_failure(error, details.str(),
                                                  "extrusion ID is not listed by its enclosing group");
                }
            }
        }
    }
    return true;
}

void run_step(Orchestrator &orchestrator, Print &print)
{
    // STEP_ORDERING is a small sequential pipeline. The first plugin normally
    // builds the PrintingPlan, and later plugins refine that same mutable plan.
    // Running these plugins in parallel would make the plan order depend on
    // scheduler timing, so every selected plugin receives the same payload and
    // runs to completion before the next plugin starts.
    Printing::PrintingPlan &plan = print.mutable_printing_plan();
    run_ctx_extrusion_ordering payload = {};
    payload.print = reinterpret_cast<const print_handle *>(&print);
    payload.plan = reinterpret_cast<printing_plan_handle *>(&plan);

    std::vector<Plugin *> plugins =
        selected_or_active_plugins_for_step(orchestrator, STEP_ORDERING, &print.full_print_config());
    std::string last_plugin_id = "<plugin pipeline>";
    for (Plugin *plugin : plugins) {
        if (plugin == nullptr)
            continue;
        last_plugin_id = plugin->get_id();

        plugin_host_context host_context = orchestrator.prepare_plugin_host_context(STEP_ORDERING, plugin, &print);
        plugin_run_context run_context = orchestrator.prepare_plugin_run_context(STEP_ORDERING, plugin, &host_context);
        run_context.data = &payload;

        plugin->setup(run_context, 1);
        plugin->setup_run(run_context);
        plugin->run(run_context);
    }

    std::string validation_error;
    if (!validate_post(print, &validation_error))
        throw RuntimeError("STEP_ORDERING post-validation failed after the plugin pipeline "
                           "(last executed plugin: '" + last_plugin_id + "'): " + validation_error);
}

} // namespace Slic3r::Steps::StepExtrusionOrdering
