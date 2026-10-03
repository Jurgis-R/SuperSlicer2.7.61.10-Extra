///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Nozzle cleaning plan-editor tests
=================================

These focused tests build small ordered PrintingPlans and run the production
STEP_EXTRUSION_EDIT plugin. They inspect typed script properties rather than
firmware-specific output, covering per-tool state, allowed transition timing,
retraction-cycle coalescing, and cleanup before a repeated plugin run.
*/

#include <catch2/catch.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "layer_extrusion_edit_transition_test_helpers.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Api/plugin/cpp/properties/ExtrusionProperties.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/ExtrusionProperty.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Steps/StepExtrusionEdition.hpp"
#include "plugins_cpp/NozzleCleaning/NozzleCleaning.hpp"

namespace {

using namespace Slic3r;
using namespace Slic3r::Printing;
using namespace Slic3r::Test::TransitionPipeline;

constexpr const char *NOZZLE_CLEANING_PLUGIN = "nozzle_cleaning";
constexpr const char *NOZZLE_CLEANING_MARKER =
    "superslicer.nozzle_cleaning.generated";

struct GeneratedScript
{
    extruder_index_t extruder_id = EXTRUDER_ID_UNKNOWN;
    bool is_unretract = false;
    std::string text;
};

/* Run only nozzle cleaning so each test controls the precise plan chronology. */
void run_nozzle_cleaning(Print &print, const uint32_t run_count = 1)
{
    ScopedActivePlugins active({NOZZLE_CLEANING_PLUGIN});
    Orchestrator &orchestrator = Orchestrator::instance();
    for (uint32_t run_idx = 0; run_idx < run_count; ++run_idx) {
        orchestrator.reset_plugin_cancel();
        Steps::StepExtrusionEdition::clean_and_prepare(print);
        Steps::StepExtrusionEdition::run_step(orchestrator, print);
        REQUIRE_FALSE(orchestrator.is_plugin_cancelled());
    }
}

/* Collect only scripts bearing the plugin's private generated marker. */
void collect_generated_scripts(
    const slic3r_api::ExtrusionEntity &entity,
    const slic3r_api::PluginPropertyKey<
        slic3r_api::NozzleCleaningPlugin::NozzleCleaningGeneratedProperty> &key,
    std::vector<GeneratedScript> &scripts)
{
    if (key.has(entity)) {
        const slic3r_api::EPropertyCustomGcode *script =
            entity.get(slic3r_api::EPropertyCustomGcode::key);
        REQUIRE(script != nullptr);
        if (script != nullptr) {
            const slic3r_api::EPropertyExtrusionAxis *axis =
                entity.get(slic3r_api::EPropertyExtrusionAxis::key);
            GeneratedScript generated;
            generated.extruder_id = script->processing_extruder_id;
            generated.is_unretract = axis != nullptr &&
                axis->operation == C_EXTRUSION_AXIS_OPERATION_UNRETRACT;
            generated.text = entity.stored_string(script->text_id);
            scripts.push_back(std::move(generated));
        }
    }

    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx)
        collect_generated_scripts(entity.child(child_idx), key, scripts);
}

/* Return generated nozzle scripts in complete plan serialization order. */
std::vector<GeneratedScript> generated_scripts(Print &print)
{
    Orchestrator &orchestrator = Orchestrator::instance();
    const slic3r_api::PluginPropertyKey<
        slic3r_api::NozzleCleaningPlugin::NozzleCleaningGeneratedProperty> key =
        slic3r_api::PluginPropertyKey<
            slic3r_api::NozzleCleaningPlugin::NozzleCleaningGeneratedProperty>::register_dynamic(
                reinterpret_cast<orchestrator_handle *>(&orchestrator),
                NOZZLE_CLEANING_MARKER);
    const PrintingPlan &core_plan = *print.printing_plan();
    const slic3r_api::PrintingPlan plan(
        reinterpret_cast<const printing_plan_handle *>(&core_plan));
    std::vector<GeneratedScript> scripts;
    for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
        const slic3r_api::PrintingGroup group = plan.group(group_idx);
        for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx) {
            const slic3r_api::PrintingLayerGroup layer = group.layer_group(layer_idx);
            for (uint32_t tool_idx = 0; tool_idx < layer.tool_group_count(); ++tool_idx) {
                const slic3r_api::PrintingToolGroup tool = layer.tool_group(tool_idx);
                const slic3r_api::PrintingScopeEvents events = tool.events();
                if (events.has_before())
                    collect_generated_scripts(events.before(), key, scripts);
                if (events.has_after())
                    collect_generated_scripts(events.after(), key, scripts);
                for (uint32_t extrusion_idx = 0;
                     extrusion_idx < tool.extrusion_count(); ++extrusion_idx) {
                    const slic3r_api::PrintingExtrusion extrusion = tool.extrusion(extrusion_idx);
                    collect_generated_scripts(extrusion.root(), key, scripts);
                }
            }
        }
    }
    return scripts;
}

/* Configure one printer's nozzle-cleaning script and interval options. */
void configure_nozzle_cleaning(Print &print,
                               const char *script,
                               const char *layers,
                               const char *volume,
                               const char *retractions,
                               const char *toolchanges,
                               const char *when)
{
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.set_deserialize_strict("nozzle_cleaning_gcode", script);
    config.set_deserialize_strict("nozzle_cleaning_interval_layers", layers);
    config.set_deserialize_strict("nozzle_cleaning_interval_volume", volume);
    config.set_deserialize_strict("nozzle_cleaning_interval_retractions", retractions);
    config.set_deserialize_strict("nozzle_cleaning_interval_toolchanges", toolchanges);
    config.set_deserialize_strict("nozzle_cleaning_when", when);
}

/* Add a tool visit to the test layer with one independent printable path. */
PrintingToolGroup &append_tool_visit(PrintingLayerGroup &layer,
                                     const extruder_index_t extruder_id,
                                     const double start_x)
{
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool = layer.tool_groups.back();
    tool.extruder_id = extruder_id;
    append_printing_extrusion(tool, make_print_path(start_x, start_x + 10.0));
    return tool;
}

/* Build a distinct valid synthetic catalog identity for object-boundary tests. */
uint32_t add_test_instance(const slic3r_api::PrintingPlan &plan,
                           const char *name,
                           const double x_offset)
{
    const Polygon outline({
        Point(scale_i(x_offset), 0),
        Point(scale_i(x_offset + 1.0), 0),
        Point(scale_i(x_offset + 1.0), scale_i(1.0)),
        Point(scale_i(x_offset), scale_i(1.0))
    });
    const slic3r_api::Polygon outline_view(
        reinterpret_cast<const polygon_handle *>(&outline));
    return plan.add_synthetic_object_instance(name, outline_view);
}

/* Add one semantic extrusion-axis event to a test tree. */
void append_axis_event(ExtrusionEntityCollection &sequence,
                       const c_extrusion_axis_operation operation,
                       const double value)
{
    sequence.append(std::make_unique<ExtrusionNop>(
        ExtrusionPropertyExtrusionAxis(operation, value)));
}

} // namespace

TEST_CASE("Nozzle cleaning counters are independent per tool and idempotent",
          "[plugins][nozzle-cleaning][toolchange][idempotence]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_print(print, model);
    configure_nozzle_cleaning(print, "; NOZZLE CLEAN", "0", "0", "0", "1", "toolchange");

    PrintingLayerGroup &layer = print.mutable_printing_plan().groups.front().layers.front();
    append_tool_visit(layer, 1, 40.0);
    append_tool_visit(layer, 0, 60.0);

    run_nozzle_cleaning(print, 2);

    const std::vector<GeneratedScript> scripts = generated_scripts(print);
    REQUIRE(scripts.size() == 1);
    CHECK(scripts[0].extruder_id == 0);
    CHECK(scripts[0].text == "; NOZZLE CLEAN");
}

TEST_CASE("Nozzle cleaning can use a tool change with an empty tool visit",
          "[plugins][nozzle-cleaning][toolchange][empty-visit]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_print(print, model);
    configure_nozzle_cleaning(print, "; EMPTY VISIT CLEAN", "0", "0", "0", "1", "toolchange");

    PrintingLayerGroup &layer = print.mutable_printing_plan().groups.front().layers.front();
    layer.tool_groups.emplace_back();
    layer.tool_groups.back().extruder_id = 1;
    layer.tool_groups.emplace_back();
    layer.tool_groups.back().extruder_id = 0;

    run_nozzle_cleaning(print, 2);

    const std::vector<GeneratedScript> scripts = generated_scripts(print);
    REQUIRE(scripts.size() == 1);
    CHECK(scripts[0].extruder_id == 0);
    CHECK(scripts[0].text == "; EMPTY VISIT CLEAN");
}

TEST_CASE("Nozzle cleaning waits for its configured object-level opportunity",
          "[plugins][nozzle-cleaning][object-transition]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_print(print, model);
    configure_nozzle_cleaning(print, "; OBJECT CLEAN", "0", "0.01", "0", "0", "object");

    PrintingPlan &core_plan = print.mutable_printing_plan();
    slic3r_api::PrintingPlan plan(
        reinterpret_cast<printing_plan_handle *>(&core_plan));
    const uint32_t first_object = add_test_instance(plan, "first instance", 100.0);
    const uint32_t second_object = add_test_instance(plan, "second instance", 110.0);
    PrintingToolGroup &tool = core_plan.groups.front().layers.front().tool_groups.front();
    REQUIRE(tool.extrusions.size() >= 2);
    tool.extrusions[0].printing_catalog_id = first_object;
    tool.extrusions[1].printing_catalog_id = second_object;

    run_nozzle_cleaning(print);

    const std::vector<GeneratedScript> scripts = generated_scripts(print);
    REQUIRE(scripts.size() == 1);
    CHECK(scripts[0].extruder_id == 0);
    CHECK(scripts[0].text == "; OBJECT CLEAN");
}

TEST_CASE("Nozzle cleaning layers become due only after an active layer",
          "[plugins][nozzle-cleaning][layers]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_print(print, model);
    configure_nozzle_cleaning(print, "; LAYER CLEAN", "1", "0", "0", "0", "object");

    PrintingPlan &core_plan = print.mutable_printing_plan();
    slic3r_api::PrintingPlan plan(
        reinterpret_cast<printing_plan_handle *>(&core_plan));
    const uint32_t first_object = add_test_instance(plan, "first layer object", 120.0);
    const uint32_t second_object = add_test_instance(plan, "second layer object", 130.0);
    PrintingToolGroup &first_tool = core_plan.groups.front().layers.front()
        .tool_groups.front();
    REQUIRE(first_tool.extrusions.size() >= 2);
    first_tool.extrusions[0].printing_catalog_id = first_object;
    first_tool.extrusions[1].printing_catalog_id = first_object;

    core_plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &second_layer = core_plan.groups.front().layers.back();
    second_layer.print_z = scale_i(0.4);
    append_tool_visit(second_layer, 0, 80.0);
    second_layer.tool_groups.front().extrusions.front().printing_catalog_id = second_object;

    run_nozzle_cleaning(print);

    const std::vector<GeneratedScript> scripts = generated_scripts(print);
    REQUIRE(scripts.size() == 1);
    CHECK(scripts[0].extruder_id == 0);
    CHECK(scripts[0].text == "; LAYER CLEAN");
}

TEST_CASE("Nozzle cleaning is disabled when every interval is zero",
          "[plugins][nozzle-cleaning][disabled]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_print(print, model);
    configure_nozzle_cleaning(print, "; DISABLED CLEAN", "0", "0", "0", "0", "retraction");

    run_nozzle_cleaning(print);

    CHECK(generated_scripts(print).empty());
}

TEST_CASE("Nozzle cleaning counts retract cycles and coalesces due triggers",
          "[plugins][nozzle-cleaning][retraction]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_print(print, model);
    configure_nozzle_cleaning(print, "; RETRACTION CLEAN", "0", "0.01", "1", "0", "retraction");

    PrintingExtrusion &extrusion = print.mutable_printing_plan().groups.front().layers.front()
        .tool_groups.front().extrusions.front();
    std::unique_ptr<ExtrusionEntityCollection> sequence =
        std::make_unique<ExtrusionEntityCollection>(false, false);
    append_axis_event(*sequence, C_EXTRUSION_AXIS_OPERATION_RETRACT_TO, 2.0);
    append_axis_event(*sequence, C_EXTRUSION_AXIS_OPERATION_RETRACT_TO, 3.0);
    sequence->append(make_print_path(0.0, 10.0));
    append_axis_event(*sequence, C_EXTRUSION_AXIS_OPERATION_UNRETRACT, 0.0);
    append_axis_event(*sequence, C_EXTRUSION_AXIS_OPERATION_RETRACT_TO, 2.0);
    append_axis_event(*sequence, C_EXTRUSION_AXIS_OPERATION_UNRETRACT, 0.0);
    extrusion.root = std::move(sequence);

    run_nozzle_cleaning(print, 2);

    const std::vector<GeneratedScript> scripts = generated_scripts(print);
    REQUIRE(scripts.size() == 2);
    for (const GeneratedScript &script : scripts) {
        CHECK(script.extruder_id == 0);
        CHECK(script.is_unretract);
        CHECK(script.text == "; RETRACTION CLEAN");
    }
}

TEST_CASE("Nozzle cleaning coalesces an object entry with its retraction phase",
          "[plugins][nozzle-cleaning][transition-coalescing]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    Model model;
    prepare_print(print, model);
    configure_nozzle_cleaning(print, "; TRANSITION CLEAN", "0", "0.01", "0", "0", "retraction");

    PrintingPlan &core_plan = print.mutable_printing_plan();
    slic3r_api::PrintingPlan plan(
        reinterpret_cast<printing_plan_handle *>(&core_plan));
    const uint32_t first_object = add_test_instance(plan, "source object", 140.0);
    const uint32_t second_object = add_test_instance(plan, "target object", 150.0);
    PrintingToolGroup &tool = core_plan.groups.front().layers.front().tool_groups.front();
    REQUIRE(tool.extrusions.size() >= 2);
    tool.extrusions[0].printing_catalog_id = first_object;
    tool.extrusions[1].printing_catalog_id = second_object;

    std::unique_ptr<ExtrusionEntityCollection> transition =
        std::make_unique<ExtrusionEntityCollection>(false, false);
    append_axis_event(*transition, C_EXTRUSION_AXIS_OPERATION_RETRACT_TO, 2.0);
    append_axis_event(*transition, C_EXTRUSION_AXIS_OPERATION_UNRETRACT, 0.0);
    transition->append(make_print_path(20.0, 30.0));
    tool.extrusions[1].root = std::move(transition);

    run_nozzle_cleaning(print);

    const std::vector<GeneratedScript> scripts = generated_scripts(print);
    REQUIRE(scripts.size() == 1);
    CHECK(scripts[0].extruder_id == 0);
    CHECK(scripts[0].is_unretract);
    CHECK(scripts[0].text == "; TRANSITION CLEAN");
}
