#include "plugin_test_helpers.hpp"

#include <cassert>
#include <cstdlib>
#include <limits>
#include <stdexcept>

#include <boost/log/trivial.hpp>

#ifdef SLIC3R_TEST_PYTHON_PLUGINS
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#else
#include <dlfcn.h>
#include <limits.h>
#include <unistd.h>
#endif

#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#endif

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/Plugins/GCode/FeatureGCode.hpp"
#include "libslic3r/Plugins/GCode/CustomGCodePerPrintZ.hpp"
#include "libslic3r/Plugins/GCode/Firmware/BuiltinGCodeFirmwares.hpp"
#include "libslic3r/Plugins/GCode/Thumbnail/BuiltinGCodeThumbnails.hpp"
#include "libslic3r/Plugins/GCode/LegacyGCodeGenerator.hpp"
#include "libslic3r/Plugins/GCode/PrintingPlanFileWriter.hpp"
#include "libslic3r/Plugins/GCode/Reader/DefaultGCodeReader.hpp"
#include "libslic3r/Plugins/GCode/SettingsGCodeScripts.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/AvoidCrossingPerimetersTravel.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/CreateRetraction.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/CreateRetractionLift.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/CreateRetractionWipe.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/CreateToolChange.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/CreateTransitionScope.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/DefaultAcceleration.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/DefaultFan.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/DefaultLayerEntryState.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/DefaultSpeed.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/DefaultTravel.hpp"
#include "libslic3r/Plugins/LayerExtrusionEdit/DefaultTravelDeceleration.hpp"
#include "libslic3r/Plugins/Ordering/DefaultOrdering.hpp"
#include "libslic3r/Plugins/PluginLoader.hpp"
#include "libslic3r/Plugins/SeamPlacer/FixedDirectionSeamPlacer.hpp"
#include "libslic3r/Plugins/SeamPlacer/CostSeamPlacer.hpp"
#include "libslic3r/Plugins/SeamPlacer/RandomSeamPlacer.hpp"
#include "libslic3r/Plugins/SkirtBrim/DefaultBrimSkirtTrim.hpp"
#include "libslic3r/Plugins/SkirtBrim/DefaultBrimGenerator.hpp"
#include "libslic3r/Plugins/SkirtBrim/DefaultSkirtGenerator.hpp"
#include "libslic3r/Plugins/Perimeter/ArachnePerimeterGenerator.hpp"
#include "libslic3r/Plugins/Perimeter/ClassicPerimeterGenerator.hpp"
#include "libslic3r/Plugins/Perimeter/ExtraPerimeterBelowArea.hpp"
#include "libslic3r/Plugins/Perimeter/ExtraPerimeterCount.hpp"
#include "libslic3r/Plugins/Perimeter/ExtraPerimeterOddLayer.hpp"
#include "libslic3r/Plugins/Perimeter/DetectOverhang.hpp"
#include "libslic3r/Plugins/Perimeter/ExtraPerimetersOnOverhangs.hpp"
#include "libslic3r/Plugins/Perimeter/FuzzySkin.hpp"
#include "libslic3r/Plugins/Perimeter/MarkFirstLoop.hpp"
#include "libslic3r/Plugins/Perimeter/OnlyOnePerimeterFirstLayer.hpp"
#include "libslic3r/Plugins/Perimeter/OnlyOnePerimeterOnTop.hpp"
#include "libslic3r/Plugins/Perimeter/RemoveGapFillOnOverhangs.hpp"
#include "libslic3r/Plugins/Perimeter/SeparateHoleContour.hpp"
#include "libslic3r/Plugins/Perimeter/SimplePerimeterGenerator.hpp"
#include "libslic3r/Plugins/SliceVolume.hpp"
#include "libslic3r/Plugins/StandardLayerHeightGenerator.hpp"
#include "libslic3r/Plugins/Surface/CleanInfillSurfaces.hpp"
#include "libslic3r/Plugins/Surface/InitialTypedSurfaceBuilder.hpp"
#include "libslic3r/Plugins/Surface/InfillRegionCompatibilitySplitter.hpp"
#include "libslic3r/Plugins/Surface/SolidShells.hpp"
#include "libslic3r/Plugins/Surface/TopSurfaceExpansion.hpp"
#include "libslic3r/Plugins/Support/SupportDemandBridgeRemoval.hpp"
#include "libslic3r/Plugins/Support/SupportDemandModifiers.hpp"
#include "libslic3r/Plugins/Support/SupportDemandOverhangs.hpp"
#include "libslic3r/Plugins/Support/SupportDemandPainting.hpp"
#include "libslic3r/Plugins/VaseMultiIslandConnector.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/SLA/SLAPrintConfig.hpp"
#include "plugins_cpp/FlatAreaLayerHeight/FlatAreaLayerHeight.hpp"
#include "plugins_cpp/DenseInfill/DenseInfill.hpp"
#include "plugins_cpp/ExtraPerimeterOverhangWave/ExtraPerimeterOverhangWave.hpp"
#include "plugins_cpp/Polyholes/Polyholes.hpp"

namespace {

// Register one deliberately rich option before PrintConfigDef is finalized so
// introspection tests can verify every pointer-backed and presence-tracked field.
void register_config_introspection_fixture(orchestrator_handle *orchestrator);
void register_config_option_or_fail(orchestrator_handle *orchestrator,
                                    const raw_config_option_def &definition);

#ifdef SLIC3R_TEST_PYTHON_PLUGINS

bool g_python_plugins_loaded = false;

boost::filesystem::path current_executable_dir()
{
#ifdef _WIN32
    std::vector<wchar_t> buffer(MAX_PATH);
    DWORD length = 0;
    for (;;) {
        length = GetModuleFileNameW(NULL, buffer.data(), DWORD(buffer.size()));
        if (length == 0)
            return {};
        if (length < buffer.size() - 1)
            break;
        buffer.resize(buffer.size() * 2);
    }
    return boost::filesystem::path(std::wstring(buffer.data(), length)).parent_path();
#else
    std::vector<char> buffer(PATH_MAX);
    for (;;) {
        const ssize_t length = readlink("/proc/self/exe", buffer.data(), buffer.size());
        if (length < 0)
            return boost::filesystem::current_path();
        if (size_t(length) < buffer.size())
            return boost::filesystem::path(std::string(buffer.data(), size_t(length))).parent_path();
        buffer.resize(buffer.size() * 2);
    }
#endif
}

bool load_python_plugins_for_tests(orchestrator_handle *orchestrator)
{
#ifdef _WIN32
    const boost::filesystem::path loader_path = current_executable_dir() / "plugins" / "python" / "plugin.dll";
    if (!boost::filesystem::exists(loader_path)) {
        BOOST_LOG_TRIVIAL(warning) << "Python plugin loader not found for plugin tests: " << loader_path.string();
        return false;
    }

    static std::vector<HMODULE> loaded_modules;
    HMODULE module = LoadLibraryW(loader_path.wstring().c_str());
    if (module == NULL) {
        BOOST_LOG_TRIVIAL(error) << "Cannot load Python plugin loader '" << loader_path.string()
                                 << "': error " << GetLastError();
        return false;
    }
    loaded_modules.push_back(module);

    FARPROC proc = GetProcAddress(module, "register_plugin");
    if (proc == NULL) {
        BOOST_LOG_TRIVIAL(error) << "Python plugin loader '" << loader_path.string()
                                 << "' does not export register_plugin.";
        return false;
    }
#else
    const boost::filesystem::path loader_path = current_executable_dir() / "plugins" / "python" / "plugin.so";
    if (!boost::filesystem::exists(loader_path)) {
        BOOST_LOG_TRIVIAL(warning) << "Python plugin loader not found for plugin tests: " << loader_path.string();
        return false;
    }

    static std::vector<void *> loaded_modules;
    void *module = dlopen(loader_path.string().c_str(), RTLD_NOW | RTLD_GLOBAL);
    if (module == nullptr) {
        BOOST_LOG_TRIVIAL(error) << "Cannot load Python plugin loader '" << loader_path.string()
                                 << "': " << dlerror();
        return false;
    }
    loaded_modules.push_back(module);

    void *proc = dlsym(module, "register_plugin");
    if (proc == nullptr) {
        BOOST_LOG_TRIVIAL(error) << "Python plugin loader '" << loader_path.string()
                                 << "' does not export register_plugin: " << dlerror();
        return false;
    }
#endif

    using RegisterPluginFn = void (*)(orchestrator_handle *);
    reinterpret_cast<RegisterPluginFn>(proc)(orchestrator);
    return true;
}

#endif // SLIC3R_TEST_PYTHON_PLUGINS

void register_config_option_or_fail(orchestrator_handle *orchestrator,
                                    const raw_config_option_def &definition)
{
    const option_def_error_code result = orchestrator_create_option_def(orchestrator, &definition);
    if (result != OPTION_DEF_ERROR_OK) {
        const std::string key = definition.opt_key == nullptr ? "<null>" : definition.opt_key;
        throw std::runtime_error(
            "Failed to register configuration introspection fixture '" + key +
            "' (error " + std::to_string(int(result)) + ").");
    }
}

void register_config_introspection_fixture(orchestrator_handle *orchestrator)
{
    static const char *aliases[] = {"test_introspection_old", "test_introspection_legacy"};
    static const char *shortcuts[] = {"perimeter_speed", "infill_speed"};
    static const char *dependencies[] = {"layer_height#0", "nozzle_diameter#0"};
    static const key_value_string_pair_t enum_entries[] = {
        {"0.5", "Half"},
        {"1.25", "Nominal"},
    };
    static const raw_graph_type graph_types[] = {
        RAW_GRAPH_TYPE_LINEAR,
        RAW_GRAPH_TYPE_SPLINE,
    };
    static const c_vec2d graph_reset_points[] = {
        {0.0, 10.0},
        {100.0, 20.0},
    };
    static const c_vec2d graph_enforced_values[] = {
        {std::numeric_limits<double>::quiet_NaN(), 10.0},
        {100.0, 20.0},
    };
    static const c_vec2d small_graph_points[] = {
        {0.0, 0.0},
        {100.0, 0.001},
    };

    raw_config_option_def definition = raw_config_option_def_init();
    definition.opt_key = "test_config_definition_introspection";
    definition.type = RAW_CO_FLOAT;
    definition.category = RAW_OPTION_CATEGORY_ADVANCED;
    definition.level = RAW_OPTION_LEVEL_EXPERT;
    definition.container_type = RAW_CONTAINER_TYPE_PROJECT;
    definition.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    definition.gui_type = RAW_GUI_TYPE_F_ENUM_OPEN;
    definition.printer_technology = RAW_PT_FFF;
    definition.invalidates_step = STEP_INFILL;
    definition.is_optional = 1;
    definition.gui_flags = "serialized,show_value";
    definition.label = "Introspection value";
    definition.full_label = "Plugin configuration introspection value";
    definition.tooltip = "Exercises the complete normalized definition snapshot.";
    definition.sidetext = "mm";
    definition.cli = "test-config-definition-introspection=f";
    definition.ratio_over = "nozzle_diameter";
    definition.multiline = 1;
    definition.full_width = 1;
    definition.is_code = 1;
    definition.is_vector_extruder = 0;
    definition.readonly = 1;
    definition.can_phony = 1;
    definition.can_be_disabled = 1;
    definition.height = 7;
    definition.width = 19;
    definition.label_width = 23;
    definition.aligned_label_left = 1;
    definition.sidetext_width = 11;
    definition.min_value = -2.5;
    definition.max_value = 8.5;
    definition.has_min = 1;
    definition.has_max = 1;
    definition.max_literal_value = 4.5;
    definition.max_literal_is_percent = 1;
    definition.has_max_literal = 1;
    definition.precision = 4;
    definition.mode = RAW_CONFIG_OPTION_MODE_EXPERT;
    definition.flags = uint64_t(0x1020304050607080ULL);
    definition.aliases = {aliases, 2};
    definition.shortcut = {shortcuts, 2};
    definition.depends_on = {dependencies, 2};
    definition.default_serialized_value = "1.25";
    definition.enum_def.value_label_pairs = {enum_entries, 2};

    register_config_option_or_fail(orchestrator, definition);

    raw_config_option_def vector_definition = raw_config_option_def_init();
    vector_definition.opt_key = "test_config_definition_vector_introspection";
    vector_definition.type = RAW_CO_VECTOR_FLOAT;
    vector_definition.container_type = RAW_CONTAINER_TYPE_PROJECT;
    vector_definition.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    vector_definition.printer_technology = RAW_PT_FFF;
    vector_definition.is_vector_extruder = 1;
    vector_definition.default_serialized_value = "1.25";
    register_config_option_or_fail(orchestrator, vector_definition);

    raw_config_option_def point3_definition = raw_config_option_def_init();
    point3_definition.opt_key = "test_config_definition_point3_introspection";
    point3_definition.type = RAW_CO_POINT3;
    point3_definition.container_type = RAW_CONTAINER_TYPE_PROJECT;
    point3_definition.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    point3_definition.printer_technology = RAW_PT_FFF;
    point3_definition.default_serialized_value = "1,2,3";
    register_config_option_or_fail(orchestrator, point3_definition);

    raw_graph_settings graph_settings = {};
    graph_settings.title = "Plugin graph";
    graph_settings.description = "Configure a value along the X axis.";
    graph_settings.y_label = "Value";
    graph_settings.x_label = "Position";
    graph_settings.null_label = "No graph values";
    graph_settings.min_x = 0.0;
    graph_settings.max_x = 100.0;
    graph_settings.step_x = 1.0;
    graph_settings.min_y = 0.0;
    graph_settings.max_y = 50.0;
    graph_settings.step_y = 0.5;
    graph_settings.label_min_x = "Minimum position";
    graph_settings.label_max_x = "Maximum position";
    graph_settings.label_min_y = "Minimum value";
    graph_settings.label_max_y = "Maximum value";
    graph_settings.allowed_types = graph_types;
    graph_settings.allowed_type_count = 2;
    graph_settings.reset_values = {
        0, 2, RAW_GRAPH_TYPE_SPLINE, graph_reset_points, 2};
    graph_settings.enforced_values = graph_enforced_values;
    graph_settings.enforced_value_count = 2;

    raw_config_option_def graph_definition = raw_config_option_def_init();
    graph_definition.opt_key = "test_config_definition_graph_introspection";
    graph_definition.type = RAW_CO_GRAPH;
    graph_definition.container_type = RAW_CONTAINER_TYPE_PROJECT;
    graph_definition.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    graph_definition.printer_technology = RAW_PT_FFF;
    graph_definition.default_serialized_value = "0:2:1:0x10:100x20";
    graph_definition.graph_settings = &graph_settings;
    register_config_option_or_fail(orchestrator, graph_definition);
    // Alternative providers may publish the same definition. Register it a
    // second time to exercise compatibility with the native NaN endpoint
    // sentinel before PrintConfigDef is finalized.
    register_config_option_or_fail(orchestrator, graph_definition);

    raw_graph_settings small_graph_settings = graph_settings;
    small_graph_settings.max_y = 0.001;
    small_graph_settings.step_y = 0.0001;
    small_graph_settings.reset_values = {
        0, 2, RAW_GRAPH_TYPE_LINEAR, small_graph_points, 2};
    small_graph_settings.enforced_values = nullptr;
    small_graph_settings.enforced_value_count = 0;

    raw_config_option_def small_graph_definition = graph_definition;
    small_graph_definition.opt_key = "test_config_definition_small_graph_range";
    small_graph_definition.default_serialized_value = "0:2:1:0x0:100x0.001";
    small_graph_definition.graph_settings = &small_graph_settings;
    register_config_option_or_fail(orchestrator, small_graph_definition);
}

void activate_plugin_or_fail(Slic3r::Orchestrator &orchestrator, const char *plugin_id)
{
    if (!orchestrator.set_plugin_active(plugin_id, true)) {
        BOOST_LOG_TRIVIAL(error) << "Test plugin '" << plugin_id << "' was not registered.";
        assert(false);
    }
}

} // namespace

namespace Slic3r::Test::Plugins {

void ensure_plugin_test_runtime_initialized()
{
    static bool initialized = []() {
        PrintConfigDef::instance_mutable().init_common_params();
        init_fff_params(PrintConfigDef::instance_mutable());
        init_sla_params(PrintConfigDef::instance_mutable());

        Orchestrator &orchestrator = Orchestrator::instance();
        orchestrator_handle *orchestrator_handle_value =
            reinterpret_cast<orchestrator_handle *>(&orchestrator);
        register_config_introspection_fixture(orchestrator_handle_value);
        slic3r_api::StandardLayerHeightGeneratorPlugin::register_standard_layer_height_generator_plugin(
            orchestrator_handle_value);
        slic3r_api::SliceVolumePlugin::register_slice_volume_plugin(orchestrator_handle_value);
        slic3r_api::VaseMultiIslandConnectorPlugin::register_vase_multi_island_connector_plugin(
            orchestrator_handle_value);
        slic3r_api::GCodeGeneration::LegacyGCodeGeneratorPlugin::register_legacy_gcode_generator_plugin(
            orchestrator_handle_value);
        slic3r_api::GCodeGeneration::PrintingPlanFileWriterPlugin::register_printing_plan_file_writer_plugin(
            orchestrator_handle_value);
        slic3r_api::GCodeGeneration::SettingsGCodeScriptsPlugin::register_settings_gcode_scripts_plugin(
            orchestrator_handle_value);
        slic3r_api::GCodeGeneration::FeatureGCodePlugin::register_feature_gcode_plugin(
            orchestrator_handle_value);
        slic3r_api::GCodeGeneration::Reader::register_builtin_gcode_reader_plugins(
            orchestrator_handle_value);
        register_gcode_reader_test_providers(orchestrator_handle_value);
        slic3r_api::GCodeGeneration::CustomGCodePerPrintZPlugin::register_custom_gcode_per_print_z_plugins(
            orchestrator_handle_value);
        slic3r_api::GCodeGeneration::Firmware::register_builtin_gcode_firmware_plugins(
            orchestrator_handle_value);
        slic3r_api::GCodeGeneration::Thumbnail::register_builtin_gcode_thumbnail_plugins(
            orchestrator_handle_value);
        slic3r_api::FlatAreaLayerHeightPlugin::register_flat_area_layer_height_plugin(orchestrator_handle_value);
        slic3r_api::DenseInfillPlugin::register_dense_infill_plugins(orchestrator_handle_value);
        slic3r_api::Ordering::DefaultOrderingPlugin::register_default_ordering_plugins(orchestrator_handle_value);
        slic3r_api::SeamPlacement::CostPlugin::register_cost_seam_placer_plugin(orchestrator_handle_value);
        slic3r_api::SeamPlacement::FixedDirectionPlugin::register_fixed_direction_seam_placer_plugin(
            orchestrator_handle_value);
        slic3r_api::SeamPlacement::RandomPlugin::register_random_seam_placer_plugin(orchestrator_handle_value);
        register_regional_seam_test_provider(orchestrator_handle_value);
        slic3r_api::LayerExtrusionEdit::CreateToolChangePlugin::register_create_tool_change_plugin(
            orchestrator_handle_value);
        slic3r_api::LayerExtrusionEdit::CreateTransitionScopePlugin::register_create_transition_scope_plugin(
            orchestrator_handle_value);
        slic3r_api::LayerExtrusionEdit::CreateRetractionPlugin::register_create_retraction_plugins(
            orchestrator_handle_value);
        slic3r_api::LayerExtrusionEdit::CreateRetractionLiftPlugin::
            register_create_retraction_lift_plugin(orchestrator_handle_value);
        slic3r_api::LayerExtrusionEdit::CreateRetractionWipePlugin::
            register_create_retraction_wipe_plugin(orchestrator_handle_value);
        slic3r_api::LayerExtrusionEdit::DefaultLayerEntryStatePlugin::register_default_layer_entry_state_plugin(
            orchestrator_handle_value);
        slic3r_api::LayerExtrusionEdit::DefaultTravelPlugin::register_default_travel_plugin(
            orchestrator_handle_value);
        slic3r_api::LayerExtrusionEdit::AvoidCrossingPerimetersTravelPlugin::
            register_avoid_crossing_perimeters_travel_plugin(orchestrator_handle_value);
        slic3r_api::LayerExtrusionEdit::DefaultSpeedPlugin::register_default_speed_plugin(
            orchestrator_handle_value);
        slic3r_api::LayerExtrusionEdit::DefaultAccelerationPlugin::register_default_acceleration_plugin(
            orchestrator_handle_value);
        slic3r_api::LayerExtrusionEdit::DefaultTravelDecelerationPlugin::register_default_travel_deceleration_plugin(
            orchestrator_handle_value);
        slic3r_api::LayerExtrusionEdit::DefaultFanPlugin::register_default_fan_plugin(
            orchestrator_handle_value);
        slic3r_api::SkirtBrim::DefaultBrimGeneratorPlugin::register_default_brim_generator_plugin(
            orchestrator_handle_value);
        slic3r_api::SkirtBrim::DefaultSkirtGeneratorPlugin::register_default_skirt_generator_plugin(
            orchestrator_handle_value);
        slic3r_api::SkirtBrim::DefaultBrimSkirtTrimPlugin::register_default_brim_skirt_trim_plugin(
            orchestrator_handle_value);
        slic3r_api::PolyholesPlugin::register_polyholes_plugin(orchestrator_handle_value);
        slic3r_api::Support::SupportDemandOverhangsPlugin::register_support_demand_overhangs_plugin(
            orchestrator_handle_value);
        slic3r_api::Support::SupportDemandPaintingPlugin::register_support_demand_painting_plugin(
            orchestrator_handle_value);
        slic3r_api::Support::SupportDemandModifiersPlugin::register_support_demand_modifiers_plugin(
            orchestrator_handle_value);
        slic3r_api::Support::SupportDemandBridgeRemovalPlugin::register_support_demand_bridge_removal_plugin(
            orchestrator_handle_value);
        slic3r_api::Perimeter::ArachnePerimeterGeneratorPlugin::register_arachne_perimeter_generator_plugin(
            orchestrator_handle_value);
        slic3r_api::Perimeter::ClassicPerimeterGeneratorPlugin::register_classic_perimeter_generator_plugin(
            orchestrator_handle_value);
        slic3r_api::Perimeter::SimplePerimeterGeneratorPlugin::register_simple_perimeter_generator_plugin(
            orchestrator_handle_value);
        slic3r_api::Perimeter::ExtraPerimeterCountPlugin::register_extra_perimeter_count_plugin(
            orchestrator_handle_value);
        slic3r_api::Perimeter::ExtraPerimeterBelowAreaPlugin::register_extra_perimeter_below_area_plugin(
            orchestrator_handle_value);
        slic3r_api::Perimeter::ExtraPerimeterOddLayerPlugin::register_extra_perimeter_odd_layer_plugin(
            orchestrator_handle_value);
        slic3r_api::Perimeter::OnlyOnePerimeterFirstLayerPlugin::register_only_one_perimeter_first_layer_plugin(
            orchestrator_handle_value);
        slic3r_api::Perimeter::OnlyOnePerimeterOnTopPlugin::register_only_one_perimeter_on_top_plugin(
            orchestrator_handle_value);
        slic3r_api::Perimeter::SeparateHoleContourPlugin::register_separate_hole_contour_plugin(
            orchestrator_handle_value);
        slic3r_api::Perimeter::RemoveGapFillOnOverhangsPlugin::register_remove_gap_fill_on_overhangs_plugin(
            orchestrator_handle_value);
        slic3r_api::Perimeter::MarkFirstLoopPlugin::register_mark_first_loop_plugin(orchestrator_handle_value);
        slic3r_api::Perimeter::ExtraPerimetersOnOverhangsPlugin::register_extra_perimeters_on_overhangs_plugin(
            orchestrator_handle_value);
        slic3r_api::Perimeter::ExtraPerimeterOverhangWavePlugin::register_extra_perimeter_overhang_wave_plugin(
            orchestrator_handle_value);
        slic3r_api::Perimeter::DetectOverhangPlugin::register_detect_overhang_plugin(orchestrator_handle_value);
        slic3r_api::Perimeter::FuzzySkinPlugin::register_fuzzy_skin_plugin(orchestrator_handle_value);
        slic3r_api::SurfaceGeneration::InitialTypedSurfaceBuilderPlugin::register_initial_typed_surface_builder_plugin(
            orchestrator_handle_value);
        slic3r_api::SurfaceGeneration::SolidShellsPlugin::register_solid_shells_plugin(
            orchestrator_handle_value);
        slic3r_api::SurfaceGeneration::TopSurfaceExpansionPlugin::register_top_surface_expansion_plugin(
            orchestrator_handle_value);
        slic3r_api::SurfaceGeneration::CleanInfillSurfacesPlugin::register_clean_infill_surfaces_plugin(
            orchestrator_handle_value);
        slic3r_api::SurfaceGeneration::InfillRegionCompatibilitySplitterPlugin::
            register_infill_region_compatibility_splitter_plugin(orchestrator_handle_value);
#ifdef SLIC3R_TEST_PYTHON_PLUGINS
        g_python_plugins_loaded = load_python_plugins_for_tests(orchestrator_handle_value) &&
                                  orchestrator.get_plugin("python.polyholes") != nullptr &&
                                  orchestrator.get_plugin("python.polyholes.high_level") != nullptr &&
                                  orchestrator.get_plugin("python.perimeter.post_process.hairy_object") != nullptr &&
                                  orchestrator.get_plugin("python.perimeter.generator.simple") != nullptr &&
                                  orchestrator.get_plugin("python.external.package_root") != nullptr;
#endif

        activate_plugin_or_fail(orchestrator, "bridge_detector.default");
        activate_plugin_or_fail(orchestrator, "standard_layer_height_generator");
        activate_plugin_or_fail(orchestrator, "slice_volume");
        activate_plugin_or_fail(orchestrator, "vase.multi_island_connector");
        activate_plugin_or_fail(orchestrator, "gcode.legacy");
        activate_plugin_or_fail(orchestrator, "gcode.printing_plan_file_writer");
        activate_plugin_or_fail(orchestrator, "gcode.settings_scripts");
        activate_plugin_or_fail(orchestrator, "gcode.feature_gcode");
        activate_plugin_or_fail(orchestrator, "gcode.reader.default");
        activate_plugin_or_fail(orchestrator, "gcode.reader.klipper");
        activate_plugin_or_fail(orchestrator, "gcode.reader.marlin1");
        activate_plugin_or_fail(orchestrator, "gcode.reader.marlin2");
        activate_plugin_or_fail(orchestrator, "gcode.reader.prusa");
        activate_plugin_or_fail(orchestrator, "gcode.reader.reprap");
        activate_plugin_or_fail(orchestrator, "gcode.reader.sprinter");
        activate_plugin_or_fail(orchestrator, "gcode.reader.test.a");
        activate_plugin_or_fail(orchestrator, "gcode.reader.test.b");
        activate_plugin_or_fail(orchestrator, "gcode.reader.test.lower_priority");
        activate_plugin_or_fail(orchestrator, "gcode.reader.test.failing_probe");
        activate_plugin_or_fail(orchestrator, "gcode.reader.test.common_handlers");
        activate_plugin_or_fail(orchestrator, "gcode.reader.test.reprap_handlers");
        activate_plugin_or_fail(orchestrator, "gcode.reader.test.marlin_handlers");
        activate_plugin_or_fail(orchestrator, "gcode.reader.test.klipper_handlers");
        activate_plugin_or_fail(orchestrator, "gcode.custom_gcode_per_print_z");
        activate_plugin_or_fail(orchestrator, "gcode.firmware.marlin1");
        activate_plugin_or_fail(orchestrator, "gcode.firmware.marlin2");
        activate_plugin_or_fail(orchestrator, "gcode.firmware.prusa");
        activate_plugin_or_fail(orchestrator, "gcode.firmware.reprap");
        activate_plugin_or_fail(orchestrator, "gcode.firmware.sprinter");
        activate_plugin_or_fail(orchestrator, "gcode.firmware.klipper");
        activate_plugin_or_fail(orchestrator, "PNG");
        activate_plugin_or_fail(orchestrator, "JPG");
        activate_plugin_or_fail(orchestrator, "QOI");
        activate_plugin_or_fail(orchestrator, "BIQU");
        activate_plugin_or_fail(orchestrator, "flat_area_layer_height");
        activate_plugin_or_fail(orchestrator, "dense_infill.surface_marker");
        activate_plugin_or_fail(orchestrator, "dense_infill.recipe_modifier");
        activate_plugin_or_fail(orchestrator, "dense_infill.post_infill_order");
        activate_plugin_or_fail(orchestrator, "ordering.plan_builder.default");
        activate_plugin_or_fail(orchestrator, "ordering.object_instance_catalog");
        activate_plugin_or_fail(orchestrator, "ordering.custom_gcode_tool_overrides");
        activate_plugin_or_fail(orchestrator, "ordering.tool_groups.default");
        activate_plugin_or_fail(orchestrator, "ordering.custom_gcode_event_tools");
        activate_plugin_or_fail(orchestrator, "ordering.printing_extrusion.presort.default");
        activate_plugin_or_fail(orchestrator, "ordering.extrusion_tree.default");
        activate_plugin_or_fail(orchestrator, "seam_placer.cost");
        activate_plugin_or_fail(orchestrator, "seam_placer.fixed");
        activate_plugin_or_fail(orchestrator, "seam_placer.random");
        // Keep a second SeamPlacer active only while the generated regional
        // selector is registered and initialized for its dedicated tests.
        activate_plugin_or_fail(orchestrator, "slic3r.test.seam_placer.regional");
        activate_plugin_or_fail(orchestrator, "layer_extrusion_edit.transition_scope.default");
        activate_plugin_or_fail(orchestrator, "layer_extrusion_edit.entry_state.default");
        activate_plugin_or_fail(orchestrator, "layer_extrusion_edit.speed.default");
        activate_plugin_or_fail(orchestrator, "layer_extrusion_edit.acceleration.default");
        activate_plugin_or_fail(orchestrator, "layer_extrusion_edit.travel_deceleration.default");
        activate_plugin_or_fail(orchestrator, "layer_extrusion_edit.fan.default");
        activate_plugin_or_fail(orchestrator, "skirt_brim.brim.default");
        activate_plugin_or_fail(orchestrator, "skirt_brim.skirt.default");
        activate_plugin_or_fail(orchestrator, "skirt_brim.brim_skirt_trim.default");
        activate_plugin_or_fail(orchestrator, "polyholes");
        activate_plugin_or_fail(orchestrator, "support.demand.overhangs");
        activate_plugin_or_fail(orchestrator, "support.demand.painting");
        activate_plugin_or_fail(orchestrator, "support.demand.modifiers");
        activate_plugin_or_fail(orchestrator, "support.demand.bridge_removal");
        activate_plugin_or_fail(orchestrator, "perimeter.generator.arachne");
        activate_plugin_or_fail(orchestrator, "perimeter.generator.classic");
        activate_plugin_or_fail(orchestrator, "perimeter.generator.simple");
        activate_plugin_or_fail(orchestrator, "perimeter.module.extra_perimeter_count");
        activate_plugin_or_fail(orchestrator, "perimeter.module.extra_perimeter_below_area");
        activate_plugin_or_fail(orchestrator, "perimeter.module.extra_perimeter_odd_layer");
        activate_plugin_or_fail(orchestrator, "perimeter.module.only_one_perimeter_first_layer");
        activate_plugin_or_fail(orchestrator, "perimeter.module.only_one_perimeter_on_top");
        activate_plugin_or_fail(orchestrator, "perimeter.module.separate_hole_contour");
        activate_plugin_or_fail(orchestrator, "perimeter.module.remove_gap_fill_on_overhangs");
        activate_plugin_or_fail(orchestrator, "perimeter.module.mark_first_loop");
        activate_plugin_or_fail(orchestrator, "perimeter.post_process.extra_perimeters_on_overhangs");
        activate_plugin_or_fail(orchestrator, "perimeter.post_process.extra_perimeter_overhang_wave");
        activate_plugin_or_fail(orchestrator, "perimeter.post_process.detect_overhang");
        activate_plugin_or_fail(orchestrator, "perimeter.post_process.fuzzy_skin");
        activate_plugin_or_fail(orchestrator, "surface.initial_typed_surface_builder");
        activate_plugin_or_fail(orchestrator, "surface.solid_shells");
        activate_plugin_or_fail(orchestrator, "surface.top_surface_expansion");
        activate_plugin_or_fail(orchestrator, "surface.clean_infill_surfaces");
        activate_plugin_or_fail(orchestrator, "surface.infill_region_compatibility_splitter");
#ifdef SLIC3R_TEST_PYTHON_PLUGINS
        if (g_python_plugins_loaded) {
            activate_plugin_or_fail(orchestrator, "python.polyholes");
            activate_plugin_or_fail(orchestrator, "python.polyholes.high_level");
            activate_plugin_or_fail(orchestrator, "python.perimeter.post_process.hairy_object");
            activate_plugin_or_fail(orchestrator, "python.perimeter.generator.simple");
        }
#endif

        register_exclusive_step_group_options(orchestrator);
        // An isolated test run can prove that another encoder publishes every
        // shared option before PNG has ever been initialized.
        const bool without_png = std::getenv("SLIC3R_TEST_THUMBNAILS_WITHOUT_PNG") != nullptr;
        if (without_png) orchestrator.set_plugin_active("PNG", false);
        orchestrator.initialize_plugins();
        if (without_png) orchestrator.set_plugin_active("PNG", true);
        register_exclusive_step_group_ui_fragments(orchestrator);
        orchestrator.set_plugin_active("slic3r.test.seam_placer.regional", false);
        orchestrator.set_plugin_active("seam_placer.cost", false);
        orchestrator.set_plugin_active("seam_placer.random", false);
        initialize_fff_print_config_cache();
        initialize_sla_print_config_cache();
        PrintConfigDef::instance_mutable().finalize();
        return true;
    }();
    (void)initialized;
}

bool python_plugin_test_runtime_available()
{
    ensure_plugin_test_runtime_initialized();
#ifdef SLIC3R_TEST_PYTHON_PLUGINS
    return g_python_plugins_loaded;
#else
    return false;
#endif
}

} // namespace Slic3r::Test::Plugins
