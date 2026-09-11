#include <catch2/catch.hpp>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include <boost/filesystem.hpp>

#include "plugin_test_helpers.hpp"
#include "gcode_test_helpers.hpp"
#include "perimeter_test_helpers.hpp"

#include "libslic3r/Api/host/ApiHostUtils.hpp"
#include "libslic3r/Api/host/GCodeScriptProcessor.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_gcode.h"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/properties/ExtrusionProperties.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/DefaultGCodeFirmwareSession.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeFirmwareViews.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeScriptProcessorViews.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/MachineEnvelope.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Config/ConfigSnapshotSerialization.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityVisitors.hpp"
#include "libslic3r/GCode.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Geometry/ArcWelder.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Plugins/GCode/Firmware/BuiltinGCodeFirmwares.hpp"
#include "libslic3r/Plugins/GCode/Firmware/KlipperGCodeFirmware.hpp"
#include "libslic3r/Plugins/GCode/Firmware/Marlin1GCodeFirmware.hpp"
#include "libslic3r/Plugins/GCode/Firmware/Marlin2GCodeFirmware.hpp"
#include "libslic3r/Plugins/GCode/Firmware/PrusaGCodeFirmware.hpp"
#include "libslic3r/Plugins/GCode/Firmware/RepRapGCodeFirmware.hpp"
#include "libslic3r/Plugins/GCode/Firmware/SprinterGCodeFirmware.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Steps/StepExtrusionEdition.hpp"
#include "libslic3r/Steps/StepGenerateGcode.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "test_data.hpp"

/*
Default firmware-session tests verify neutral machine-state interpretation, extrusion traversal and extension hooks.
Shared plan construction and output helpers live in gcode_test_helpers so this file keeps only component-specific behavior.
*/

namespace {
using namespace Slic3r;
using namespace Slic3r::Printing;
using namespace Slic3r::Test::GCode;

class CustomFanFirmwareSession final :
    public slic3r_api::GCodeGeneration::DefaultGCodeFirmwareSession
{
protected:
    std::string encode_fan(uint16_t tool_id, double speed_percent) const override
    {
        return "CUSTOM_FAN T" + std::to_string(tool_id) +
               " S" + std::to_string(int32_t(std::lround(speed_percent))) + "\n";
    }
};

// Exposes the protected syntax encoder so kind dispatch can be tested without
// involving extrusion traversal or filesystem publication.
class CustomGCodeFirmwareProbe final :
    public slic3r_api::GCodeGeneration::DefaultGCodeFirmwareSession
{
public:
    using DefaultGCodeFirmwareSession::encode_custom_gcode;
};

// Exposes the parent begin-print orchestration without adding any dialect
// behavior beyond a visible marker for the machine-envelope hook.
class MachineEnvelopeFirmwareSession final :
    public slic3r_api::GCodeGeneration::DefaultGCodeFirmwareSession
{
public:
    uint32_t encode_count() const { return m_encode_count; }
    bool initialized_during_encoding() const { return m_initialized_during_encoding; }
    const std::optional<slic3r_api::GCodeGeneration::MachineEnvelope> &encoded_envelope() const
    {
        return m_encoded_envelope;
    }
    bool formatter_initialized() const
    {
        (void)gcode_formatter();
        return true;
    }

protected:
    std::string encode_machine_envelope(
        const slic3r_api::GCodeGeneration::MachineEnvelope &envelope) const override
    {
        // Accessing the formatter proves that setup() completed before the
        // virtual dialect hook was entered.
        (void)gcode_formatter();
        m_initialized_during_encoding = true;
        m_encoded_envelope = envelope;
        ++m_encode_count;
        return "machine-envelope\n";
    }

private:
    mutable uint32_t m_encode_count = 0;
    mutable bool m_initialized_during_encoding = false;
    mutable std::optional<slic3r_api::GCodeGeneration::MachineEnvelope> m_encoded_envelope;
};

class PreviewInspectingFirmwareSession final :
    public slic3r_api::GCodeGeneration::DefaultGCodeFirmwareSession
{
public:
    std::string end_print() override
    {
        return printer().preview_enabled() ? "PREVIEW_ON\n" : "PREVIEW_OFF\n";
    }
};

class TraversalHookFirmwareSession final :
    public slic3r_api::GCodeGeneration::DefaultGCodeFirmwareSession
{
public:
    explicit TraversalHookFirmwareSession(std::vector<std::string> &calls) : m_calls(calls) {}

protected:
    std::string enter_extrusion_node(const slic3r_api::ExtrusionEntity &) override
    {
        m_calls.emplace_back("enter");
        return {};
    }

    std::string visit_extrusion_leaf(const slic3r_api::ExtrusionEntity &) override
    {
        m_calls.emplace_back("leaf");
        return {};
    }

    std::string leave_extrusion_node(const slic3r_api::ExtrusionEntity &) override
    {
        m_calls.emplace_back("leave");
        return {};
    }

private:
    std::vector<std::string> &m_calls;
};

/* Add the minimum plan hierarchy needed by STEP_GCODE validation tests. */
PrintingToolGroup &append_speed_validation_tool_group(Print &print,
                                                      coord_t print_z = scale_i(0.42),
                                                      uint16_t extruder_id = 3)
{
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.back().layers.emplace_back();
    PrintingLayerGroup &layer_group = plan.groups.back().layers.back();
    layer_group.print_z = print_z;
    layer_group.tool_groups.emplace_back();
    layer_group.tool_groups.back().extruder_id = extruder_id;
    return layer_group.tool_groups.back();
}

/* Build one two-point geometric leaf with an optional direct speed property. */
std::unique_ptr<ExtrusionEntity> speed_validation_path(
    std::optional<float> speed,
    double start_x = 1.25,
    double end_x = 3.75)
{
    std::unique_ptr<ExtrusionPath> path(new ExtrusionPath(
        ArcPolyline(Points{
            Point(scale_i(start_x), scale_i(2.5)),
            Point(scale_i(end_x), scale_i(2.5))
        }),
        ExtrusionAttributes(ExtrusionRole::Perimeter, ExtrusionFlow(0.08, 0.4f, 0.2f)),
        nullptr,
        true));
    if (speed)
        path->add_property(ExtrusionPropertySpeed(*speed));
    return path;
}

/* Append one independently owned validation root to a tool group. */
PrintingExtrusion &append_speed_validation_extrusion(
    PrintingToolGroup &tool_group,
    std::unique_ptr<ExtrusionEntity> root,
    const LayerRegionIsland *region_island = nullptr,
    uint16_t object_instance_idx = 0)
{
    PrintingExtrusion extrusion;
    extrusion.region_island = region_island;
    extrusion.sregion_island_role = ExtrusionRole::Perimeter;
    extrusion.root = std::move(root);
    extrusion.object_instance_idx = object_instance_idx;
    tool_group.extrusions.push_back(std::move(extrusion));
    return tool_group.extrusions.back();
}

/* Add one geometric event below a scope's fixed before or after root. */
void append_speed_validation_event(PrintingScopeEvents &events,
                                   const bool before,
                                   std::optional<float> speed)
{
    const std::unique_ptr<ExtrusionEntity> event = speed_validation_path(speed);
    if (before)
        events.append_before(*event);
    else
        events.append_after(*event);
}

/* Add one E-only command whose firmware path still emits a movement line. */
void append_special_extrusion_event(PrintingScopeEvents &events,
                                    const bool before,
                                    std::optional<float> speed)
{
    ExtrusionNop event;
    event.add_property(ExtrusionPropertySpecialCommand(
        ExtrusionPropertySpecialCommand::Code::EXTRUSION, 0.5));
    if (speed)
        event.add_property(ExtrusionPropertySpeed(*speed));
    if (before)
        events.append_before(event);
    else
        events.append_after(event);
}

} // namespace

TEST_CASE("STEP_GCODE pre-validation resolves inherited geometric speeds",
          "[plugins][gcode][validation][speed]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    SECTION("a local positive speed is accepted") {
        Print print;
        PrintingToolGroup &tool_group = append_speed_validation_tool_group(print);
        append_speed_validation_extrusion(tool_group, speed_validation_path(22.f));

        std::string error = "stale";
        CHECK(Steps::StepGenerateGcode::validate_pre(print, &error));
        CHECK(error.empty());
    }

    SECTION("a child inherits a positive parent speed") {
        Print print;
        PrintingToolGroup &tool_group = append_speed_validation_tool_group(print);
        ExtrusionEntity::Children children;
        children.push_back(speed_validation_path(std::nullopt));
        std::unique_ptr<ExtrusionEntity> root(new ExtrusionEntity(
            std::move(children), false, false, true));
        root->add_property(ExtrusionPropertySpeed(24.f));
        append_speed_validation_extrusion(tool_group, std::move(root));

        CHECK(Steps::StepGenerateGcode::validate_pre(print));
    }

    SECTION("a non-positive child declaration keeps the inherited parent speed") {
        Print print;
        PrintingToolGroup &tool_group = append_speed_validation_tool_group(print);
        ExtrusionEntity::Children children;
        children.push_back(speed_validation_path(0.f));
        children.push_back(speed_validation_path(-1.f));
        std::unique_ptr<ExtrusionEntity> root(new ExtrusionEntity(
            std::move(children), false, false, true));
        root->add_property(ExtrusionPropertySpeed(26.f));
        append_speed_validation_extrusion(tool_group, std::move(root));

        CHECK(Steps::StepGenerateGcode::validate_pre(print));
    }

    SECTION("an event leaf without geometry does not require speed") {
        Print print;
        PrintingToolGroup &tool_group = append_speed_validation_tool_group(print);
        append_speed_validation_extrusion(
            tool_group, std::unique_ptr<ExtrusionEntity>(new ExtrusionNop()));

        CHECK(Steps::StepGenerateGcode::validate_pre(print));
    }
}

TEST_CASE("STEP_GCODE pre-validation reports missing and malformed speeds",
          "[plugins][gcode][validation][speed]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    SECTION("a PrintingPlan is required") {
        Print print;
        std::string error = "stale";
        CHECK_FALSE(Steps::StepGenerateGcode::validate_pre(print, &error));
        CHECK(error.find("Print has no plan") != std::string::npos);
    }

    SECTION("a geometric leaf without any declaration has a complete diagnostic") {
        Print print;
        PrintingToolGroup &tool_group = append_speed_validation_tool_group(print);
        ExtrusionEntity::Children inner_children;
        inner_children.push_back(speed_validation_path(std::nullopt));
        ExtrusionEntity::Children root_children;
        root_children.push_back(std::unique_ptr<ExtrusionEntity>(new ExtrusionEntity(
            std::move(inner_children), false, false, true)));
        append_speed_validation_extrusion(tool_group, std::unique_ptr<ExtrusionEntity>(
            new ExtrusionEntity(std::move(root_children), false, false, true)));

        std::string error;
        CHECK_FALSE(Steps::StepGenerateGcode::validate_pre(print, &error));
        CHECK(error.find("1 geometric extrusion leaf has no valid effective speed") != std::string::npos);
        CHECK(error.find(
            "groups[0].layers[0].tool_groups[0].extrusions[0].root.children[0].children[0]") !=
            std::string::npos);
        CHECK(error.find("print_z=0.42 mm") != std::string::npos);
        CHECK(error.find("extruder_id=3") != std::string::npos);
        CHECK(error.find("source_role=IPeri") != std::string::npos);
        CHECK(error.find("region_island=unavailable") != std::string::npos);
        CHECK(error.find("object_instance_idx=0") != std::string::npos);
        CHECK(error.find("type=path") != std::string::npos);
        CHECK(error.find("points=2, segments=1") != std::string::npos);
        CHECK(error.find("length=2.5 mm") != std::string::npos);
        CHECK(error.find("start=(1.25, 2.5)") != std::string::npos);
        CHECK(error.find("end=(3.75, 2.5)") != std::string::npos);
        CHECK(error.find("contains_arcs=no") != std::string::npos);
        CHECK(error.find("Speed declarations: none on the leaf or its parents") != std::string::npos);
        CHECK(error.find("neither this geometric leaf nor any parent declares a speed") != std::string::npos);
    }

    SECTION("inherit sentinels without a positive ancestor are listed") {
        Print print;
        PrintingToolGroup &tool_group = append_speed_validation_tool_group(print);
        ExtrusionEntity::Children children;
        children.push_back(speed_validation_path(0.f));
        std::unique_ptr<ExtrusionEntity> root(new ExtrusionEntity(
            std::move(children), false, false, true));
        root->add_property(ExtrusionPropertySpeed(-1.f));
        append_speed_validation_extrusion(tool_group, std::move(root));

        std::string error;
        CHECK_FALSE(Steps::StepGenerateGcode::validate_pre(print, &error));
        CHECK(error.find(".root: speed_mm_per_s=-1 (inherit)") != std::string::npos);
        CHECK(error.find(".root.children[0]: speed_mm_per_s=0 (inherit)") != std::string::npos);
        CHECK(error.find("every speed declaration on this path requests inheritance") != std::string::npos);
    }

    SECTION("non-finite declarations remain errors despite a valid ancestor") {
        const std::vector<std::pair<float, const char *>> invalid_values{
            {std::numeric_limits<float>::quiet_NaN(), "NaN"},
            {std::numeric_limits<float>::infinity(), "+infinity"},
            {-std::numeric_limits<float>::infinity(), "-infinity"}
        };
        for (const std::pair<float, const char *> &invalid : invalid_values) {
            CAPTURE(invalid.second);
            Print print;
            PrintingToolGroup &tool_group = append_speed_validation_tool_group(print);
            ExtrusionEntity::Children children;
            children.push_back(speed_validation_path(invalid.first));
            std::unique_ptr<ExtrusionEntity> root(new ExtrusionEntity(
                std::move(children), false, false, true));
            root->add_property(ExtrusionPropertySpeed(28.f));
            append_speed_validation_extrusion(tool_group, std::move(root));

            std::string error;
            CHECK_FALSE(Steps::StepGenerateGcode::validate_pre(print, &error));
            CHECK(error.find(std::string("speed_mm_per_s=") + invalid.second) != std::string::npos);
            CHECK(error.find("Every explicit speed must be finite") != std::string::npos);
        }
    }

    SECTION("a missing extrusion root is rejected at its plan location") {
        Print print;
        PrintingToolGroup &tool_group = append_speed_validation_tool_group(print);
        append_speed_validation_extrusion(tool_group, nullptr);

        std::string error;
        CHECK_FALSE(Steps::StepGenerateGcode::validate_pre(print, &error));
        CHECK(error.find("extrusions[0].root") != std::string::npos);
        CHECK(error.find("print_z=0.42 mm") != std::string::npos);
        CHECK(error.find("extruder_id=3") != std::string::npos);
        CHECK(error.find("Entity: unavailable") != std::string::npos);
        CHECK(error.find("Speed declarations: unavailable") != std::string::npos);
        CHECK(error.find("has no root extrusion tree") != std::string::npos);
    }
}

TEST_CASE("STEP_GCODE pre-validation covers every before and after event scope",
          "[plugins][gcode][validation][speed][events]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    PrintingToolGroup &tool_group = append_speed_validation_tool_group(print);
    PrintingPlan &plan = print.mutable_printing_plan();
    PrintingGroup &group = plan.groups.front();
    PrintingLayerGroup &layer_group = group.layers.front();

    append_speed_validation_event(plan.events, true, std::nullopt);
    append_speed_validation_event(plan.events, false, std::nullopt);
    append_speed_validation_event(group.events, true, std::nullopt);
    append_speed_validation_event(group.events, false, std::nullopt);
    append_speed_validation_event(layer_group.events, true, std::nullopt);
    append_speed_validation_event(layer_group.events, false, std::nullopt);
    append_speed_validation_event(tool_group.events, true, std::nullopt);
    append_speed_validation_event(tool_group.events, false, std::nullopt);

    std::string error;
    CHECK_FALSE(Steps::StepGenerateGcode::validate_pre(print, &error));
    CHECK(error.find("8 movement-producing before/after event entities have") != std::string::npos);
    CHECK(error.find("plan.events.before.children[0]") != std::string::npos);
    CHECK(error.find("plan.events.after.children[0]") != std::string::npos);
    CHECK(error.find("groups[0].events.before.children[0]") != std::string::npos);
    CHECK(error.find("groups[0].events.after.children[0]") != std::string::npos);
    CHECK(error.find("groups[0].layers[0].events.before.children[0]") != std::string::npos);
    CHECK(error.find("groups[0].layers[0].events.after.children[0]") != std::string::npos);
    CHECK(error.find(
        "groups[0].layers[0].tool_groups[0].events.before.children[0]") !=
        std::string::npos);
    CHECK(error.find(
        "groups[0].layers[0].tool_groups[0].events.after.children[0]") !=
        std::string::npos);
    CHECK(error.find("source_role=unavailable (scope event)") != std::string::npos);
    CHECK(error.find("Source context: unavailable for a before/after scope event") !=
          std::string::npos);
}

TEST_CASE("STEP_GCODE pre-validation recognizes event movement kinds",
          "[plugins][gcode][validation][speed][events]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    SECTION("geometric and E-only events accept their own speed") {
        Print print;
        PrintingToolGroup &tool_group = append_speed_validation_tool_group(print);
        append_speed_validation_event(tool_group.events, true, 18.f);
        append_special_extrusion_event(tool_group.events, false, 12.f);

        CHECK(Steps::StepGenerateGcode::validate_pre(print));
    }

    SECTION("event geometry inherits speed inside its own event root") {
        Print print;
        PrintingToolGroup &tool_group = append_speed_validation_tool_group(print);
        ExtrusionEntity::Children children;
        children.push_back(speed_validation_path(std::nullopt));
        ExtrusionEntity event(std::move(children), false, false, true);
        event.add_property(ExtrusionPropertySpeed(19.f));
        tool_group.events.append_before(event);

        CHECK(Steps::StepGenerateGcode::validate_pre(print));
    }

    SECTION("an E-only special extrusion without speed is diagnosed") {
        Print print;
        PrintingToolGroup &tool_group = append_speed_validation_tool_group(print);
        append_special_extrusion_event(tool_group.events, false, std::nullopt);

        std::string error;
        CHECK_FALSE(Steps::StepGenerateGcode::validate_pre(print, &error));
        CHECK(error.find("1 movement-producing before/after event entity has") !=
              std::string::npos);
        CHECK(error.find("movement=E-only special extrusion") != std::string::npos);
        CHECK(error.find("neither this E-only event nor any parent declares a speed") !=
              std::string::npos);
    }

    SECTION("a non-movement scope event needs no speed") {
        Print print;
        PrintingToolGroup &tool_group = append_speed_validation_tool_group(print);
        tool_group.events.append_before(ExtrusionNop());

        CHECK(Steps::StepGenerateGcode::validate_pre(print));
    }

    SECTION("an E-only command in a PrintingExtrusion is also rejected early") {
        Print print;
        PrintingToolGroup &tool_group = append_speed_validation_tool_group(print);
        std::unique_ptr<ExtrusionNop> event(new ExtrusionNop());
        event->add_property(ExtrusionPropertySpecialCommand(
            ExtrusionPropertySpecialCommand::Code::EXTRUSION, -15.0));
        append_speed_validation_extrusion(tool_group, std::move(event));

        std::string error;
        CHECK_FALSE(Steps::StepGenerateGcode::validate_pre(print, &error));
        CHECK(error.find("1 E-only special extrusion event has") != std::string::npos);
        CHECK(error.find("extrusions[0].root") != std::string::npos);
    }
}

TEST_CASE("STEP_GCODE speed diagnostics retain source object context",
          "[plugins][gcode][validation][speed]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    using namespace Slic3r::Test::PerimeterPluginTests;

    PreparedPerimeterPrint prepared;
    const DynamicPrintConfig config = perimeter_config({});
    prepare_cube_print(prepared, config);
    REQUIRE(prepared.print.objects().size() == 1);
    prepared.print.object(0).model_object()->name = "speed_validation_cube.stl";
    REQUIRE(prepared.print.object(0).layer_count() > 0);
    const Layer &source_layer = prepared.print.object(0).layer(0);
    REQUIRE(source_layer.region_count() > 0);
    const LayerRegionIsland source_island({&source_layer.region(0)}, 0);

    PrintingToolGroup &tool_group = append_speed_validation_tool_group(
        prepared.print, source_layer.scaled_print_z(), source_island.extruder_id());
    append_speed_validation_extrusion(
        tool_group, speed_validation_path(std::nullopt), &source_island, 0);

    std::string error;
    CHECK_FALSE(Steps::StepGenerateGcode::validate_pre(prepared.print, &error));
    CHECK(error.find("region_count=1") != std::string::npos);
    CHECK(error.find("layer_id=0") != std::string::npos);
    CHECK(error.find("print_object_index=0") != std::string::npos);
    CHECK(error.find("object_name=\"speed_validation_cube.stl\"") != std::string::npos);
    CHECK(error.find("object_instance_idx=0") != std::string::npos);
}

TEST_CASE("STEP_GCODE speed diagnostics are bounded without hiding the total",
          "[plugins][gcode][validation][speed]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    PrintingToolGroup &tool_group = append_speed_validation_tool_group(print);
    for (size_t extrusion_idx = 0; extrusion_idx < 12; ++extrusion_idx)
        append_speed_validation_extrusion(
            tool_group, speed_validation_path(std::nullopt, double(extrusion_idx), double(extrusion_idx + 1)));

    std::string error;
    CHECK_FALSE(Steps::StepGenerateGcode::validate_pre(print, &error));
    CHECK(error.find("12 geometric extrusion leaves") != std::string::npos);
    CHECK(error.find("[10/12]") != std::string::npos);
    CHECK(error.find("[11/12]") == std::string::npos);
    CHECK(error.find("2 additional validation errors were omitted") != std::string::npos);
}

TEST_CASE("STEP_GCODE rejects an invalid speed before touching output",
          "[plugins][gcode][validation][speed]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    PrintingToolGroup &tool_group = append_speed_validation_tool_group(print);
    append_speed_validation_extrusion(tool_group, speed_validation_path(std::nullopt));

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    write_text_file(output_path, "previous output\n");
    try {
        Steps::StepGenerateGcode::run_step(
            Orchestrator::instance(), print, output_path.string());
        FAIL("STEP_GCODE accepted a geometric leaf without speed");
    } catch (const RuntimeError &error) {
        CHECK(std::string(error.what()).find("PrintingPlan location") != std::string::npos);
    }
    CHECK(read_text_file(output_path) == "previous output\n");
    CHECK_FALSE(boost::filesystem::exists(output_path.string() + ".tmp"));
    remove_output_pair(output_path);
}

TEST_CASE("G-code formatter builds checked command parameters",
          "[plugins][gcode][formatter]")
{
    SECTION("integer and fixed parameters preserve their requested representation") {
        slic3r_api::GCodeGeneration::GCodeFormatter formatter(3, 5);
        formatter.emit_string("M204");
        formatter.emit_integer_parameter('P', 499.5);
        formatter.emit_fixed_parameter('T', 8.0, 2);
        formatter.emit_comment(true, "test command");
        CHECK(formatter.string() == "M204 P500 T8.00 ; test command\n");
    }

    SECTION("invalid values are rejected before encoding") {
        slic3r_api::GCodeGeneration::GCodeFormatter formatter(3, 5);
        CHECK_THROWS_AS(formatter.emit_integer_parameter('X', -1.0), std::invalid_argument);
        CHECK_THROWS_AS(
            formatter.emit_integer_parameter('X', std::numeric_limits<double>::quiet_NaN()),
            std::invalid_argument);
        CHECK_THROWS_AS(
            formatter.emit_fixed_parameter('X', std::numeric_limits<double>::infinity(), 2),
            std::invalid_argument);
        CHECK_THROWS_AS(
            formatter.emit_integer_parameter('X', std::numeric_limits<double>::max()),
            std::invalid_argument);
        CHECK_THROWS_AS(formatter.emit_fixed_parameter('X', 1.0, 10), std::invalid_argument);
    }
}
TEST_CASE("Default firmware owns machine envelope initialization",
          "[plugins][gcode][firmware][machine-envelope]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    SECTION("disabled limits initialize the session without calling the dialect hook") {
        Print print;
        configure_standard_firmware(print);
        const slic3r_api::Print print_view(
            reinterpret_cast<const print_handle *>(&print));
        MachineEnvelopeFirmwareSession session;
        CHECK(session.begin_print(print_view).empty());
        CHECK(session.formatter_initialized());
        CHECK(session.encode_count() == 0);
    }

    SECTION("enabled limits reach the dialect hook after setup") {
        Print print;
        configure_standard_firmware(print);
        configure_machine_envelope(print);
        const slic3r_api::Print print_view(
            reinterpret_cast<const print_handle *>(&print));
        MachineEnvelopeFirmwareSession session;
        CHECK(session.begin_print(print_view) == "machine-envelope\n");
        CHECK(session.encode_count() == 1);
        CHECK(session.initialized_during_encoding());
        REQUIRE(session.encoded_envelope());
        CHECK(session.encoded_envelope()->max_acceleration_x == 1000.0);

        // The neutral base remains directly usable and deliberately has no
        // firmware-specific envelope syntax.
        slic3r_api::GCodeGeneration::DefaultGCodeFirmwareSession neutral_session;
        CHECK(neutral_session.begin_print(print_view).empty());
    }

    SECTION("invalid limits fail before entering the dialect hook") {
        Print print;
        configure_standard_firmware(print);
        configure_machine_envelope(print);
        PrintConfig &config = const_cast<PrintConfig &>(print.config());
        config.machine_max_feedrate_x.set(std::vector<double>{-1.0});
        const slic3r_api::Print print_view(
            reinterpret_cast<const print_handle *>(&print));
        MachineEnvelopeFirmwareSession session;
        CHECK_THROWS_AS(
            session.begin_print(print_view), std::invalid_argument);
        CHECK(session.formatter_initialized());
        CHECK(session.encode_count() == 0);
    }
}
TEST_CASE("Machine envelope reads and validates neutral printer limits",
          "[plugins][gcode][firmware][dialects][machine-envelope]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    // Presets may retain limits for estimates without asking firmware sessions
    // to publish those limits in the generated file.
    Print disabled_print;
    configure_standard_firmware(disabled_print);
    const slic3r_api::Print disabled_view(
        reinterpret_cast<const print_handle *>(&disabled_print));
    CHECK_FALSE(slic3r_api::GCodeGeneration::configured_machine_envelope(
        disabled_view.config()));

    // The public reader copies printer data without applying any dialect unit
    // conversion, leaving that decision to the selected firmware session.
    Print enabled_print;
    configure_standard_firmware(enabled_print);
    configure_machine_envelope(enabled_print);
    const slic3r_api::Print enabled_view(
        reinterpret_cast<const print_handle *>(&enabled_print));
    const std::optional<slic3r_api::GCodeGeneration::MachineEnvelope> envelope =
        slic3r_api::GCodeGeneration::configured_machine_envelope(enabled_view.config());
    REQUIRE(envelope);
    CHECK(envelope->max_acceleration_x == 1000.0);
    CHECK(envelope->max_acceleration_y == 1100.0);
    CHECK(envelope->max_acceleration_z == 120.0);
    CHECK(envelope->max_acceleration_e == 1300.0);
    CHECK(envelope->max_feedrate_x == 200.0);
    CHECK(envelope->max_feedrate_y == 210.0);
    CHECK(envelope->max_feedrate_z == 12.0);
    CHECK(envelope->max_feedrate_e == 25.0);
    CHECK(envelope->max_print_acceleration == 500.0);
    CHECK(envelope->max_retract_acceleration == 800.0);
    CHECK(envelope->max_travel_acceleration == 900.0);
    CHECK(envelope->max_jerk_x == 8.0);
    CHECK(envelope->max_jerk_y == 9.0);
    CHECK(envelope->max_jerk_z == 0.4);
    CHECK(envelope->max_jerk_e == 2.5);
    CHECK(envelope->min_extruding_feedrate == 3.0);
    CHECK(envelope->min_travel_feedrate == 4.0);

    SECTION("negative limits are rejected") {
        Print invalid_print;
        configure_standard_firmware(invalid_print);
        configure_machine_envelope(invalid_print);
        PrintConfig &config = const_cast<PrintConfig &>(invalid_print.config());
        config.machine_max_feedrate_x.set(std::vector<double>{-1.0});
        const slic3r_api::Print invalid_view(
            reinterpret_cast<const print_handle *>(&invalid_print));
        CHECK_THROWS_AS(
            slic3r_api::GCodeGeneration::configured_machine_envelope(invalid_view.config()),
            std::invalid_argument);
    }

    SECTION("non-finite limits are rejected") {
        Print invalid_print;
        configure_standard_firmware(invalid_print);
        configure_machine_envelope(invalid_print);
        PrintConfig &config = const_cast<PrintConfig &>(invalid_print.config());
        config.machine_max_feedrate_x.set(
            std::vector<double>{std::numeric_limits<double>::quiet_NaN()});
        const slic3r_api::Print invalid_view(
            reinterpret_cast<const print_handle *>(&invalid_print));
        CHECK_THROWS_AS(
            slic3r_api::GCodeGeneration::configured_machine_envelope(invalid_view.config()),
            std::invalid_argument);
    }
}
TEST_CASE("Default firmware interprets scope events with its extrusion visitor",
          "[plugins][gcode][firmware][printing][plan][events]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    select_printing_plan_writer(print);
    PrintingPlan &plan = print.mutable_printing_plan();
    ExtrusionNop event;
    event.add_property(ExtrusionPropertyCustomGcodeText("M117 scope event"));
    plan.events.append_before(event);

    FirmwareInstanceOwner firmware(slic3r_api::make_gcode_firmware_instance(
        std::unique_ptr<slic3r_api::GCodeFirmwareSession>(
            new slic3r_api::GCodeGeneration::DefaultGCodeFirmwareSession())));
    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        run_file_writer_with_firmware(print, output_path, firmware.instance());
        CHECK(read_text_file(output_path) == "M117 scope event\n");
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}
TEST_CASE("Default firmware emits inherited process state and ordered linear moves",
          "[plugins][gcode][firmware]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    select_printing_plan_writer(print);
    configure_standard_firmware(print, 2);
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.front().layers.front();
    layer.print_z = scale_i(0.2);
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool_group = layer.tool_groups.front();
    tool_group.extruder_id = 0;

    ArcPolyline polyline(Points{
        Point(scale_i(0.0), scale_i(0.0)),
        Point(scale_i(10.0), scale_i(0.0)),
        Point(scale_i(20.0), scale_i(0.0))
    });
    polyline.set_z_offset(0, scale_i(0.1));
    polyline.set_z_offset(1, scale_i(0.1));
    polyline.set_z_offset(2, scale_i(0.2));
    append_path_extrusion(tool_group, make_firmware_path(polyline, 20.f, 500.f, 0.04f, 50.f, 210.f));

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, output_path.string());
        const std::string output = read_text_file(output_path);
        INFO(output);

        const size_t temperature_pos = output.find("M104 S210 T0\n");
        const size_t fan_pos = output.find("M106 S128\n");
        const size_t pressure_pos = output.find("M900 K0.04\n");
        const size_t travel_pos = output.find("G0 X0 Y0 Z.3 F6000\n");
        const size_t print_acceleration_pos = output.find("M204 P500\n");
        const size_t first_extrusion_pos = output.find("G1 X10 E");
        const size_t second_extrusion_pos = output.find("G1 X20 Z.4 E");
        REQUIRE(temperature_pos != std::string::npos);
        REQUIRE(fan_pos != std::string::npos);
        REQUIRE(pressure_pos != std::string::npos);
        REQUIRE(travel_pos != std::string::npos);
        REQUIRE(print_acceleration_pos != std::string::npos);
        REQUIRE(first_extrusion_pos != std::string::npos);
        REQUIRE(second_extrusion_pos != std::string::npos);
        CHECK(temperature_pos < fan_pos);
        CHECK(fan_pos < pressure_pos);
        CHECK(pressure_pos < travel_pos);
        CHECK(travel_pos < print_acceleration_pos);
        CHECK(print_acceleration_pos < first_extrusion_pos);
        CHECK(first_extrusion_pos < second_extrusion_pos);

        // State remains emitted across both segments. Speed is attached only
        // to the first movement that needs the new value.
        CHECK(count_occurrences(output, "M104 S210 T0\n") == 1);
        CHECK(count_occurrences(output, "M106 S128\n") == 1);
        CHECK(count_occurrences(output, "M900 K0.04\n") == 1);
        CHECK(count_occurrences(output, "M204 P500\n") == 1);
        CHECK(output.find("M204 T") == std::string::npos);
        CHECK(count_occurrences(output, " F1200") == 1);
        CHECK_FALSE(boost::filesystem::exists(output_path.string() + ".tmp"));
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}
TEST_CASE("Default firmware preserves arcs and computes their extrusion length",
          "[plugins][gcode][firmware]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    select_printing_plan_writer(print);
    configure_standard_firmware(print);
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.front().layers.front();
    layer.print_z = scale_i(0.2);
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool_group = layer.tool_groups.front();
    tool_group.extruder_id = 0;

    Geometry::ArcWelder::Path arc_path;
    arc_path.emplace_back(Point(scale_i(0.0), scale_i(0.0)));
    arc_path.emplace_back(
        Point(scale_i(10.0), scale_i(0.0)),
        float(scale_i(5.0)),
        Geometry::ArcWelder::Orientation::CCW);
    append_path_extrusion(
        tool_group,
        make_firmware_path(ArcPolyline(arc_path), 25.f, 600.f));

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, output_path.string());
        const std::string output = read_text_file(output_path);
        const size_t arc_pos = output.find("G3 X10 R5 E");
        REQUIRE(arc_pos != std::string::npos);
        CHECK(output.find("G1 X10") == std::string::npos);

        // A 5 mm radius semicircle is longer than the 10 mm chord, so its E
        // output must exceed the straight-path value of about 0.333 mm.
        const size_t e_pos = output.find(" E", arc_pos);
        REQUIRE(e_pos != std::string::npos);
        CHECK(std::stod(output.substr(e_pos + 2)) > 0.5);
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}
TEST_CASE("Default firmware interprets one acceleration field from the movement role",
          "[plugins][gcode][firmware]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    select_printing_plan_writer(print);
    configure_standard_firmware(print);
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.front().layers.front();
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool_group = layer.tool_groups.front();
    tool_group.extruder_id = 0;

    // The travel and the following print path carry the same property type.
    // Their effective roles decide whether Marlin receives M204 T or M204 P.
    append_path_extrusion(
        tool_group,
        make_firmware_travel(
            ArcPolyline(Points{Point(scale_i(0.0), scale_i(0.0)), Point(scale_i(10.0), scale_i(0.0))}),
            80.f,
            1200.f));
    append_path_extrusion(
        tool_group,
        make_firmware_path(
            ArcPolyline(Points{Point(scale_i(10.0), scale_i(0.0)), Point(scale_i(11.0), scale_i(0.0))}),
            20.f,
            400.f));

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, output_path.string());
        const std::string output = read_text_file(output_path);
        INFO(output);
        CHECK(count_occurrences(output, "M204 T1200\n") == 1);
        CHECK(count_occurrences(output, "M204 P400\n") == 1);
        CHECK(output.find("G0 X10 F4800\n") != std::string::npos);
        CHECK(output.find("G0 X10 E") == std::string::npos);
        CHECK(output.find("G1 X11 E") != std::string::npos);
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}
TEST_CASE("Default firmware writes the E value prepared by the extruder state",
          "[plugins][gcode][firmware]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    select_printing_plan_writer(print);
    configure_standard_firmware(print);
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.front().layers.front();
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool_group = layer.tool_groups.front();
    tool_group.extruder_id = 0;

    // Two invisible deltas cross the output precision together. The last
    // command then returns the absolute coordinate to zero, which must remain
    // distinguishable from an absent E word.
    ExtrusionEntity::Children children;
    children.push_back(special_command_entity(ExtrusionPropertySpecialCommand::Code::EXTRUSION, 0.000004));
    children.push_back(special_command_entity(ExtrusionPropertySpecialCommand::Code::EXTRUSION, 0.000004));
    children.push_back(special_command_entity(ExtrusionPropertySpecialCommand::Code::EXTRUSION, -0.000008));
    std::unique_ptr<ExtrusionEntity> root(
        new ExtrusionEntity(std::move(children), false, true, true));
    root->add_property(ExtrusionPropertySpeed(12.f, 300.f));
    append_path_extrusion(tool_group, std::move(root));

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, output_path.string());
        const std::string output = read_text_file(output_path);
        INFO(output);
        CHECK(output.find("G1 E.00001") != std::string::npos);
        CHECK(output.find("G1 E0") != std::string::npos);
        CHECK(count_occurrences(output, "G1 E") == 2);
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}

TEST_CASE("E-only unretraction resolves zero speed per extruder without inherited speed",
          "[plugins][gcode][firmware][retraction][deretract-zero]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const uint16_t tool_id = GENERATE(uint16_t(0), uint16_t(1));
    const bool explicit_speed = GENERATE(false, true);
    Print print;
    select_printing_plan_writer(print);
    configure_standard_firmware(print, 2);
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.use_firmware_retraction.value = false;
    config.use_relative_e_distances.value = false;
    config.retract_speed.set(std::vector<double>{40.0, 55.0});
    config.deretract_speed.set(std::vector<double>{explicit_speed ? 18.0 : 0.0,
                                                 explicit_speed ? 23.0 : 0.0});
    PrintingToolGroup &tool = append_speed_validation_tool_group(print, scale_i(0.2), tool_id);

    // Neither event nor parent declares speed: both E-only moves must resolve
    // their speed from the selected tool, independently of previous XY moves.
    ExtrusionEntity::Children children;
    for (const bool is_retract : {true, false}) {
        std::unique_ptr<ExtrusionNop> event(new ExtrusionNop());
        slic3r_api::MutableExtrusionEntity view(
            reinterpret_cast<extrusion_entity_handle *>(event.get()));
        view.get_or_add(slic3r_api::EPropertyAttributes::key)
            .extrusion_role(is_retract ? RAW_EXTRUSION_ROLE_RETRACT : RAW_EXTRUSION_ROLE_UNRETRACT)
            .mm3_per_mm(0.0);
        if (is_retract)
            view.get_or_add(slic3r_api::EPropertyExtrusionAxis::key).retract_to(2.0);
        else
            view.get_or_add(slic3r_api::EPropertyExtrusionAxis::key).unretract(0.0);
        children.push_back(std::move(event));
    }
    append_path_extrusion(tool, std::unique_ptr<ExtrusionEntity>(
        new ExtrusionEntity(std::move(children), false, true, true)));

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, output_path.string());
        const std::string output = read_text_file(output_path);
        INFO(output);
        const std::string retract_feed = tool_id == 0 ? "2400" : "3300";
        CHECK(output.find("G1 E-2 F" + retract_feed + "\n") != std::string::npos);
        if (explicit_speed) {
            const std::string unretract_feed = tool_id == 0 ? "1080" : "1380";
            CHECK(output.find("G1 E0 F" + unretract_feed + "\n") != std::string::npos);
        } else {
            // The same feed is already encoded by the retract command.
            CHECK(output.find("G1 E0\n") != std::string::npos);
        }
        CHECK(count_occurrences(output, "G1 E") == 2);
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}

TEST_CASE("Default firmware encodes semantic retract wipe and unretract leaves",
          "[plugins][gcode][firmware][retraction][wipe]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    select_printing_plan_writer(print);
    configure_standard_firmware(print);
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.front().layers.front();
    layer.print_z = scale_i(0.2);
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool_group = layer.tool_groups.front();
    tool_group.extruder_id = 0;

    ExtrusionEntity::Children children;
    std::unique_ptr<ExtrusionNop> retract(new ExtrusionNop());
    slic3r_api::MutableExtrusionEntity retract_view(
        reinterpret_cast<extrusion_entity_handle *>(retract.get()));
    retract_view.get_or_add(slic3r_api::EPropertyAttributes::key)
        .extrusion_role(RAW_EXTRUSION_ROLE_RETRACT).mm3_per_mm(0.0);
    retract_view.get_or_add(slic3r_api::EPropertyExtrusionAxis::key)
        .retract_to(0.2);
    children.push_back(std::move(retract));

    ArcPolyline wipe_polyline(Points{Point(scale_i(0.0), scale_i(0.0)),
                                     Point(scale_i(2.5), scale_i(0.0)),
                                     Point(scale_i(5.0), scale_i(0.0))});
    wipe_polyline.set_z_offset(0, 0);
    wipe_polyline.set_z_offset(1, scale_i(10.0));
    wipe_polyline.set_z_offset(2, 0);
    std::unique_ptr<ExtrusionPath> wipe = make_firmware_travel(
        wipe_polyline, 40.f, 800.f);
    slic3r_api::MutableExtrusionEntity wipe_view(
        reinterpret_cast<extrusion_entity_handle *>(wipe.get()));
    wipe_view.get_mutable(slic3r_api::EPropertyAttributes::key)->extrusion_role(
        RAW_EXTRUSION_ROLE_TRAVEL | RAW_EXTRUSION_ROLE_WIPE |
        RAW_EXTRUSION_ROLE_RETRACT);
    wipe_view.get_or_add(slic3r_api::EPropertyExtrusionAxis::key).retract_to(0.8);
    children.push_back(std::move(wipe));

    std::unique_ptr<ExtrusionPath> unretract = make_firmware_travel(
        ArcPolyline(Points{Point(scale_i(5.0), scale_i(0.0)),
                           Point(scale_i(6.0), scale_i(0.0))}),
        40.f,
        800.f);
    slic3r_api::MutableExtrusionEntity unretract_view(
        reinterpret_cast<extrusion_entity_handle *>(unretract.get()));
    unretract_view.get_or_add(slic3r_api::EPropertyAttributes::key)
        .extrusion_role(RAW_EXTRUSION_ROLE_TRAVEL | RAW_EXTRUSION_ROLE_WIPE |
                        RAW_EXTRUSION_ROLE_UNRETRACT).mm3_per_mm(0.0);
    unretract_view.get_or_add(slic3r_api::EPropertyExtrusionAxis::key)
        .unretract(-0.05);
    children.push_back(std::move(unretract));

    append_path_extrusion(tool_group, std::unique_ptr<ExtrusionEntity>(
        new ExtrusionEntity(std::move(children), false, true, true)));

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, output_path.string());
        const std::string output = read_text_file(output_path);
        INFO(output);
        const size_t retract_pos = output.find("G1 E-.2");
        const size_t wipe_start_pos = output.find(";WIPE_START\n");
        const size_t first_wipe_move_pos = output.find("G1 X2.5 Z10.2 E-.5");
        const size_t wipe_move_pos = output.find("G1 X5 Z.2 E-.8");
        const size_t wipe_end_pos = output.find(";WIPE_END\n");
        const size_t unretract_pos = output.find("G1 X6 E-.05");
        REQUIRE(retract_pos != std::string::npos);
        REQUIRE(wipe_start_pos != std::string::npos);
        REQUIRE(first_wipe_move_pos != std::string::npos);
        REQUIRE(wipe_move_pos != std::string::npos);
        REQUIRE(wipe_end_pos != std::string::npos);
        REQUIRE(unretract_pos != std::string::npos);
        CHECK(retract_pos < wipe_start_pos);
        CHECK(wipe_start_pos < first_wipe_move_pos);
        CHECK(first_wipe_move_pos < wipe_move_pos);
        CHECK(wipe_move_pos < unretract_pos);
        CHECK(unretract_pos < wipe_end_pos);
        CHECK(output.find("G0 X5 E") == std::string::npos);
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}

TEST_CASE("Default firmware accepts a motion-only wipe without an E request",
          "[plugins][gcode][firmware][wipe]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    select_printing_plan_writer(print);
    configure_standard_firmware(print);
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.front().layers.front();
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool_group = layer.tool_groups.front();
    tool_group.extruder_id = 0;

    std::unique_ptr<ExtrusionPath> wipe = make_firmware_travel(
        ArcPolyline(Points{Point(scale_i(0.0), scale_i(0.0)),
                           Point(scale_i(5.0), scale_i(0.0))}),
        40.f,
        800.f);
    slic3r_api::MutableExtrusionEntity wipe_view(
        reinterpret_cast<extrusion_entity_handle *>(wipe.get()));
    wipe_view.get_mutable(slic3r_api::EPropertyAttributes::key)->extrusion_role(
        RAW_EXTRUSION_ROLE_TRAVEL | RAW_EXTRUSION_ROLE_WIPE);
    append_path_extrusion(tool_group, std::move(wipe));

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, output_path.string());
        const std::string output = read_text_file(output_path);
        INFO(output);
        CHECK(output.find(";WIPE_START\n") != std::string::npos);
        CHECK(output.find("G1 X5") != std::string::npos);
        CHECK(output.find(" E") == std::string::npos);
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}

TEST_CASE("Default firmware supports native semantic retraction",
          "[plugins][gcode][firmware][retraction]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    select_printing_plan_writer(print);
    configure_standard_firmware(print);
    const_cast<PrintConfig &>(print.config()).use_firmware_retraction.value = true;
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.front().layers.front();
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool_group = layer.tool_groups.front();
    tool_group.extruder_id = 0;

    ExtrusionEntity::Children children;
    std::unique_ptr<ExtrusionNop> retract(new ExtrusionNop());
    slic3r_api::MutableExtrusionEntity retract_view(
        reinterpret_cast<extrusion_entity_handle *>(retract.get()));
    retract_view.get_or_add(slic3r_api::EPropertyAttributes::key)
        .extrusion_role(RAW_EXTRUSION_ROLE_RETRACT).mm3_per_mm(0.0);
    retract_view.get_or_add(slic3r_api::EPropertyExtrusionAxis::key).retract_to(0.8);
    children.push_back(std::move(retract));
    std::unique_ptr<ExtrusionPath> wipe = make_firmware_travel(
        ArcPolyline(Points{Point(scale_i(0.0), scale_i(0.0)),
                           Point(scale_i(5.0), scale_i(0.0))}),
        40.f,
        800.f);
    slic3r_api::MutableExtrusionEntity wipe_view(
        reinterpret_cast<extrusion_entity_handle *>(wipe.get()));
    wipe_view.get_mutable(slic3r_api::EPropertyAttributes::key)->extrusion_role(
        RAW_EXTRUSION_ROLE_TRAVEL | RAW_EXTRUSION_ROLE_WIPE |
        RAW_EXTRUSION_ROLE_RETRACT);
    wipe_view.get_or_add(slic3r_api::EPropertyExtrusionAxis::key).retract_to(0.8);
    children.push_back(std::move(wipe));
    std::unique_ptr<ExtrusionPath> unretract = make_firmware_travel(
        ArcPolyline(Points{Point(scale_i(5.0), scale_i(0.0)),
                           Point(scale_i(6.0), scale_i(0.0))}),
        40.f,
        800.f);
    slic3r_api::MutableExtrusionEntity unretract_view(
        reinterpret_cast<extrusion_entity_handle *>(unretract.get()));
    unretract_view.get_or_add(slic3r_api::EPropertyAttributes::key)
        .extrusion_role(RAW_EXTRUSION_ROLE_TRAVEL | RAW_EXTRUSION_ROLE_WIPE |
                        RAW_EXTRUSION_ROLE_UNRETRACT).mm3_per_mm(0.0);
    unretract_view.get_or_add(slic3r_api::EPropertyExtrusionAxis::key).unretract();
    children.push_back(std::move(unretract));
    append_path_extrusion(tool_group, std::unique_ptr<ExtrusionEntity>(
        new ExtrusionEntity(std::move(children), false, true, true)));

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, output_path.string());
        const std::string output = read_text_file(output_path);
        INFO(output);
        CHECK(count_occurrences(output, "G10\n") == 1);
        CHECK(count_occurrences(output, "G11\n") == 1);
        CHECK(output.find("G1 E") == std::string::npos);
        const size_t wipe_start = output.find(";WIPE_START\n");
        const size_t wipe_move = output.find("G1 X5");
        const size_t approach_move = output.find("G1 X6");
        const size_t unretract = output.find("G11\n");
        REQUIRE(wipe_start != std::string::npos);
        REQUIRE(wipe_move != std::string::npos);
        REQUIRE(approach_move != std::string::npos);
        REQUIRE(unretract != std::string::npos);
        CHECK(wipe_start < wipe_move);
        CHECK(wipe_move < approach_move);
        CHECK(approach_move < unretract);
        CHECK(output.find("G0 X5") == std::string::npos);
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}
TEST_CASE("A derived firmware may replace only the fan encoder",
          "[plugins][gcode][firmware]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    configure_standard_firmware(print);
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.front().layers.front();
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool_group = layer.tool_groups.front();
    tool_group.extruder_id = 0;
    append_path_extrusion(
        tool_group,
        make_firmware_path(
            ArcPolyline(Points{Point(scale_i(0.0), scale_i(0.0)), Point(scale_i(1.0), scale_i(0.0))}),
            15.f,
            400.f,
            -1.f,
            40.f));

    FirmwareInstanceOwner firmware(slic3r_api::make_gcode_firmware_instance(
        std::unique_ptr<slic3r_api::GCodeFirmwareSession>(new CustomFanFirmwareSession())));
    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        run_file_writer_with_firmware(print, output_path, firmware.instance());
        const std::string output = read_text_file(output_path);
        CHECK(output.find("CUSTOM_FAN T0 S40\n") != std::string::npos);
        CHECK(output.find("M106") == std::string::npos);
        CHECK(output.find("M204 P400\n") != std::string::npos);
        CHECK(output.find("G1 X1 E") != std::string::npos);
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}
TEST_CASE("A derived firmware may extend extrusion traversal without replacing it",
          "[plugins][gcode][firmware]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    configure_standard_firmware(print);
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.front().layers.front();
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool_group = layer.tool_groups.front();
    tool_group.extruder_id = 0;

    // The collection and its path form two nested nodes. The derived session
    // observes both scopes and the leaf while the base visitor still performs
    // all standard state resolution and geometry serialization.
    ExtrusionEntity::Children children;
    children.push_back(make_firmware_path(
        ArcPolyline(Points{Point(scale_i(0.0), scale_i(0.0)), Point(scale_i(1.0), scale_i(0.0))}),
        15.f,
        400.f));
    append_path_extrusion(
        tool_group,
        std::unique_ptr<ExtrusionEntity>(new ExtrusionEntity(std::move(children), false, true, true)));

    std::vector<std::string> calls;
    FirmwareInstanceOwner firmware(slic3r_api::make_gcode_firmware_instance(
        std::unique_ptr<slic3r_api::GCodeFirmwareSession>(new TraversalHookFirmwareSession(calls))));
    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        run_file_writer_with_firmware(print, output_path, firmware.instance());
        const std::vector<std::string> expected{"enter", "enter", "leaf", "leave", "leave"};
        CHECK(calls == expected);
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}
TEST_CASE("Default firmware restores inherited requests between sibling nodes",
          "[plugins][gcode][firmware]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    select_printing_plan_writer(print);
    configure_standard_firmware(print);
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.front().layers.front();
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool_group = layer.tool_groups.front();
    tool_group.extruder_id = 0;

    ExtrusionEntity::Children children;
    children.push_back(make_firmware_path(
        ArcPolyline(Points{Point(scale_i(0.0), scale_i(0.0)), Point(scale_i(1.0), scale_i(0.0))}),
        -1.f,
        -1.f));
    children.push_back(make_firmware_path(
        ArcPolyline(Points{Point(scale_i(1.0), scale_i(0.0)), Point(scale_i(2.0), scale_i(0.0))}),
        30.f,
        700.f,
        0.08f,
        70.f,
        220.f));
    children.push_back(make_firmware_path(
        ArcPolyline(Points{Point(scale_i(2.0), scale_i(0.0)), Point(scale_i(3.0), scale_i(0.0))}),
        -1.f,
        -1.f));
    std::unique_ptr<ExtrusionEntity> root(
        new ExtrusionEntity(std::move(children), false, true, true));
    root->add_property(ExtrusionPropertySpeed(20.f, 500.f, 0.04f, 50.f, 210.f));
    append_path_extrusion(tool_group, std::move(root));

    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, output_path.string());
        const std::string output = read_text_file(output_path);
        INFO(output);

        const size_t first_temperature = output.find("M104 S210 T0\n");
        const size_t child_temperature = output.find("M104 S220 T0\n");
        const size_t restored_temperature = output.find("M104 S210 T0\n", first_temperature + 1);
        REQUIRE(first_temperature != std::string::npos);
        REQUIRE(child_temperature != std::string::npos);
        REQUIRE(restored_temperature != std::string::npos);
        CHECK(first_temperature < child_temperature);
        CHECK(child_temperature < restored_temperature);
        CHECK(count_occurrences(output, "M204 P500\n") == 2);
        CHECK(count_occurrences(output, "M204 P700\n") == 1);
        CHECK(count_occurrences(output, " F1200") == 2);
        CHECK(count_occurrences(output, " F1800") == 1);
        CHECK(count_occurrences(output, "M106 S128\n") == 2);
        CHECK(count_occurrences(output, "M106 S179\n") == 1);
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}
TEST_CASE("Default firmware handles ordered special commands and custom G-code",
          "[plugins][gcode][firmware]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    configure_standard_firmware(print, 2);
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.front().layers.front();
    layer.tool_groups.emplace_back();
    PrintingToolGroup &tool_group = layer.tool_groups.front();
    tool_group.extruder_id = 0;

    ExtrusionEntity::Children children;
    children.push_back(special_command_entity(ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 1.0));
    children.push_back(special_command_entity(
        ExtrusionPropertySpecialCommand::Code::SAVE_AND_RESET_SPEED_RATIO, 0.8));
    children.push_back(special_command_entity(
        ExtrusionPropertySpecialCommand::Code::RESTORE_SPEED_RATIO));
    children.push_back(special_command_entity(
        ExtrusionPropertySpecialCommand::Code::FLUSH_PLANNER_QUEUE));
    children.push_back(special_command_entity(
        ExtrusionPropertySpecialCommand::Code::EXTRUSION, 0.5));
    children.push_back(special_command_entity(
        ExtrusionPropertySpecialCommand::Code::RETRACT, -0.2));
    children.push_back(special_command_entity(
        ExtrusionPropertySpecialCommand::Code::PAUSE, 25.0));
    children.push_back(special_command_entity(
        ExtrusionPropertySpecialCommand::Code::WAIT_FOR_TEMP));
    children.push_back(special_command_entity(
        ExtrusionPropertySpecialCommand::Code::WAIT_FOR_TEMP));
    children.push_back(special_command_entity(
        ExtrusionPropertySpecialCommand::Code::DISABLE_PREVIEW));
    children.push_back(special_command_entity(
        ExtrusionPropertySpecialCommand::Code::ENABLE_PREVIEW));
    children.push_back(special_command_entity(
        ExtrusionPropertySpecialCommand::Code::EXTRUDER_CURRENT, 650.0));
    std::unique_ptr<ExtrusionNop> custom(new ExtrusionNop());
    custom->add_property(ExtrusionPropertyCustomGcodeText("M117 ready"));
    children.push_back(std::move(custom));

    std::unique_ptr<ExtrusionEntity> root(
        new ExtrusionEntity(std::move(children), false, true, true));
    root->add_property(ExtrusionPropertySpeed(12.f, 300.f, 0.02f, 30.f, 205.f));
    append_path_extrusion(tool_group, std::move(root));

    FirmwareInstanceOwner firmware(slic3r_api::make_gcode_firmware_instance(
        std::unique_ptr<slic3r_api::GCodeFirmwareSession>(new PreviewInspectingFirmwareSession())));
    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        run_file_writer_with_firmware(print, output_path, firmware.instance());
        const std::string output = read_text_file(output_path);
        INFO(output);
        CHECK(output.find("T0\nT1\n") != std::string::npos);
        CHECK(output.find("M220 B\nM220 S80\n") != std::string::npos);
        CHECK(output.find("M220 R\n") != std::string::npos);
        CHECK(output.find("G4 S0\n") != std::string::npos);
        CHECK(output.find("G1 E") != std::string::npos);
        CHECK(output.find(" F720\n") != std::string::npos);
        CHECK(output.find(" F2400\n") != std::string::npos);
        CHECK(output.find("G4 P25\n") != std::string::npos);
        CHECK(output.find("M109 S205 T1\n") != std::string::npos);
        CHECK(count_occurrences(output, "M109 S205 T1\n") == 1);
        CHECK(output.find("M906 T1 E650\n") != std::string::npos);
        CHECK(output.find("M117 ready\n") != std::string::npos);
        CHECK(output.find("PREVIEW_ON\n") != std::string::npos);
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}

TEST_CASE("Tool-group semantic selection suppresses the automatic firmware command",
          "[plugins][gcode][firmware][toolchange]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    configure_standard_firmware(print, 2);
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    PrintingLayerGroup &layer = plan.groups.front().layers.front();
    layer.tool_groups.emplace_back();
    layer.tool_groups.back().extruder_id = 0;
    layer.tool_groups.emplace_back();
    PrintingToolGroup &target = layer.tool_groups.back();
    target.extruder_id = 1;

    // CreateToolChange stores the semantic command in the target tool-group's
    // before events. begin_tool_group() must update no physical selection of
    // its own; write_event() then emits the sole T1 command in sequence.
    std::unique_ptr<ExtrusionEntity> command = special_command_entity(
        ExtrusionPropertySpecialCommand::Code::TOOLCHANGE, 1.0);
    target.events.append_before(std::move(*command));

    const std::string output = export_with_firmware(
        print, "gcode.firmware.marlin2");
    INFO(output);
    CHECK(count_command_lines(output, "T0") == 1);
    CHECK(count_command_lines(output, "T1") == 1);
    CHECK(output.find("T0\nT1\n") != std::string::npos);
}
TEST_CASE("Default firmware distinguishes raw G-code, comments and scripts",
          "[plugins][gcode][firmware][custom-gcode]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    CustomGCodeFirmwareProbe firmware;

    CHECK(firmware.encode_custom_gcode(
              C_EXTRUSION_CUSTOM_GCODE_GCODE, "M117 raw") == "M117 raw\n");
    CHECK(firmware.encode_custom_gcode(
              C_EXTRUSION_CUSTOM_GCODE_COMMENT, "first\nsecond") ==
          "; first\n; second\n");
    CHECK_THROWS_AS(
        firmware.encode_custom_gcode(C_EXTRUSION_CUSTOM_GCODE_SCRIPT, "M117 {layer_num}"),
        std::invalid_argument);
    CHECK_THROWS_AS(
        firmware.encode_custom_gcode(
            static_cast<c_extrusion_custom_gcode_kind>(99), "M117 unknown"),
        std::invalid_argument);

    Print print;
    configure_standard_firmware(print);
    const slic3r_api::Print print_view(reinterpret_cast<const print_handle *>(&print));
    CHECK(firmware.begin_print(print_view).empty());

    ExtrusionNop typed_raw(ExtrusionPropertyCustomGcodeText("M117 raw"));
    ExtrusionPropertyCustomGcode *typed_raw_property = typed_raw.get_property<ExtrusionPropertyCustomGcode>();
    REQUIRE(typed_raw_property != nullptr);
    typed_raw_property->script_type = GCODE_SCRIPT_TYPE_START_GCODE;
    CHECK_THROWS_AS(firmware.write_event(
                        slic3r_api::ExtrusionEntity(reinterpret_cast<const extrusion_entity_handle *>(&typed_raw))),
                    std::invalid_argument);

    ExtrusionNop untyped_script(
        ExtrusionPropertyCustomGcodeText(ExtrusionPropertyCustomGcodeText::Code::SCRIPT, "M117 script"));
    ExtrusionPropertyCustomGcode *untyped_script_property = untyped_script.get_property<ExtrusionPropertyCustomGcode>();
    REQUIRE(untyped_script_property != nullptr);
    untyped_script_property->script_type = GCODE_SCRIPT_TYPE_INVALID;
    CHECK_THROWS_AS(firmware.write_event(slic3r_api::ExtrusionEntity(
                        reinterpret_cast<const extrusion_entity_handle *>(&untyped_script))),
                    std::invalid_argument);
}
TEST_CASE("The generic firmware session keeps every tool environment independent",
          "[plugins][gcode][firmware]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Print print;
    configure_standard_firmware(print, 2);
    PrintingPlan &plan = print.mutable_printing_plan();
    plan.groups.emplace_back();
    plan.groups.front().layers.emplace_back();
    append_multi_tool_process_sequence(plan.groups.front().layers.front());

    FirmwareInstanceOwner firmware(slic3r_api::make_gcode_firmware_instance(
        std::unique_ptr<slic3r_api::GCodeFirmwareSession>(
            new slic3r_api::GCodeGeneration::DefaultGCodeFirmwareSession())));
    const boost::filesystem::path output_path = temporary_gcode_path();
    remove_output_pair(output_path);
    try {
        run_file_writer_with_firmware(print, output_path, firmware.instance());
        const std::string output = read_text_file(output_path);
        INFO(output);
        CHECK(count_occurrences(output, "M104 S200 T0\n") == 1);
        CHECK(count_occurrences(output, "M104 S210 T1\n") == 1);
        CHECK(count_occurrences(output, "M106 S51\n") == 1);
        CHECK(count_occurrences(output, "M106 S77\n") == 1);
    } catch (...) {
        remove_output_pair(output_path);
        throw;
    }
    remove_output_pair(output_path);
}
TEST_CASE("Default firmware rejects missing process properties and unknown tools",
          "[plugins][gcode][firmware]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    SECTION("printable leaf without valid flow attributes") {
        Print print;
        select_printing_plan_writer(print);
        configure_standard_firmware(print);
        PrintingPlan &plan = print.mutable_printing_plan();
        plan.groups.emplace_back();
        plan.groups.front().layers.emplace_back();
        PrintingLayerGroup &layer = plan.groups.front().layers.front();
        layer.tool_groups.emplace_back();
        PrintingToolGroup &tool_group = layer.tool_groups.front();
        tool_group.extruder_id = 0;

        // A printable role with default attributes has no usable mm3_per_mm,
        // even though its process speed and acceleration are otherwise valid.
        std::unique_ptr<ExtrusionPath> path(new ExtrusionPath(
            ArcPolyline(Points{Point(scale_i(0.0), scale_i(0.0)), Point(scale_i(1.0), scale_i(0.0))}),
            ExtrusionAttributes(ExtrusionRole::Perimeter),
            nullptr,
            true));
        path->add_property(ExtrusionPropertySpeed(20.f, 500.f));
        append_path_extrusion(tool_group, std::move(path));

        const boost::filesystem::path output_path = temporary_gcode_path();
        remove_output_pair(output_path);
        CHECK_THROWS_AS(
            Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, output_path.string()),
            RuntimeError);
        Orchestrator::instance().reset_plugin_cancel();
        CHECK_FALSE(boost::filesystem::exists(output_path));
        CHECK_FALSE(boost::filesystem::exists(output_path.string() + ".tmp"));
    }

    SECTION("printable leaf without speed") {
        Print print;
        select_printing_plan_writer(print);
        configure_standard_firmware(print);
        PrintingPlan &plan = print.mutable_printing_plan();
        plan.groups.emplace_back();
        plan.groups.front().layers.emplace_back();
        PrintingLayerGroup &layer = plan.groups.front().layers.front();
        layer.tool_groups.emplace_back();
        PrintingToolGroup &tool_group = layer.tool_groups.front();
        tool_group.extruder_id = 0;
        append_path_extrusion(
            tool_group,
            make_firmware_path(
                ArcPolyline(Points{Point(scale_i(0.0), scale_i(0.0)), Point(scale_i(1.0), scale_i(0.0))}),
                -1.f,
                500.f));

        const boost::filesystem::path output_path = temporary_gcode_path();
        remove_output_pair(output_path);
        write_text_file(output_path, "previous output\n");
        CHECK_THROWS_AS(
            Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, output_path.string()),
            RuntimeError);
        Orchestrator::instance().reset_plugin_cancel();
        CHECK(read_text_file(output_path) == "previous output\n");
        CHECK_FALSE(boost::filesystem::exists(output_path.string() + ".tmp"));
        remove_output_pair(output_path);
    }

    SECTION("printable leaf without acceleration") {
        Print print;
        select_printing_plan_writer(print);
        configure_standard_firmware(print);
        PrintingPlan &plan = print.mutable_printing_plan();
        plan.groups.emplace_back();
        plan.groups.front().layers.emplace_back();
        PrintingLayerGroup &layer = plan.groups.front().layers.front();
        layer.tool_groups.emplace_back();
        PrintingToolGroup &tool_group = layer.tool_groups.front();
        tool_group.extruder_id = 0;
        append_path_extrusion(
            tool_group,
            make_firmware_path(
                ArcPolyline(Points{Point(scale_i(0.0), scale_i(0.0)), Point(scale_i(1.0), scale_i(0.0))}),
                20.f,
                -1.f));

        const boost::filesystem::path output_path = temporary_gcode_path();
        remove_output_pair(output_path);
        Steps::StepGenerateGcode::run_step(
            Orchestrator::instance(), print, output_path.string());
        const std::string output = read_text_file(output_path);
        CHECK(output.find("M204") == std::string::npos);
        CHECK(output.find("G1 X1 E") != std::string::npos);
        remove_output_pair(output_path);
    }

    SECTION("tool group outside the configured range") {
        Print print;
        select_printing_plan_writer(print);
        configure_standard_firmware(print);
        PrintingPlan &plan = print.mutable_printing_plan();
        plan.groups.emplace_back();
        plan.groups.front().layers.emplace_back();
        PrintingLayerGroup &layer = plan.groups.front().layers.front();
        layer.tool_groups.emplace_back();
        PrintingToolGroup &tool_group = layer.tool_groups.front();
        tool_group.extruder_id = 4;
        append_empty_extrusion(tool_group);

        const boost::filesystem::path output_path = temporary_gcode_path();
        remove_output_pair(output_path);
        CHECK_THROWS_AS(
            Steps::StepGenerateGcode::run_step(Orchestrator::instance(), print, output_path.string()),
            RuntimeError);
        Orchestrator::instance().reset_plugin_cancel();
        CHECK_FALSE(boost::filesystem::exists(output_path));
        CHECK_FALSE(boost::filesystem::exists(output_path.string() + ".tmp"));
    }
}
