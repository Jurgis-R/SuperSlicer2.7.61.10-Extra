#include <catch2/catch.hpp>

#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/Config/PrintConfig.hpp"

namespace {
using namespace Slic3r;
using namespace Slic3r::Test::PerimeterPluginTests;
}

TEST_CASE("Extra perimeter count module adds requested loops", "[plugins][perimeter]")
{
    // Uniform region: with one base perimeter and two extra perimeters, the
    // module should produce exactly three nested external loops.
    const ExPolygon area = rectangle_expolygon(-10., -10., 10., 10.);
    const DynamicPrintConfig disabled = perimeter_config({{"extra_perimeters_count", "0"}});
    const DynamicPrintConfig enabled = perimeter_config({{"extra_perimeters_count", "2"}});

    const PerimeterRunCapture base_run =
        run_perimeter_case(disabled, {SIMPLE_PERIMETER_GENERATOR, EXTRA_PERIMETER_COUNT}, area, 0);
    const PerimeterRunCapture enabled_run =
        run_perimeter_case(enabled, {SIMPLE_PERIMETER_GENERATOR, EXTRA_PERIMETER_COUNT}, area, 0);
    REQUIRE(external_perimeter_count(base_run) == 1);
    REQUIRE(external_perimeter_count(enabled_run) == 3);
    require_leaf_fill_area_consistency(base_run);
    require_leaf_fill_area_consistency(enabled_run);

    // Region-local case: the left-side region requests one extra loop.
    // We should keep the full-area base loop crossing x=0, then add exactly one
    // left-only loop. If the module applied the override globally, this would
    // still have two loops, but both would cross the split line.
    const ExPolygon left_override_area = rectangle_expolygon(-9., -9., -2., 9.);
    const PerimeterRunCapture overlap_run =
        run_perimeter_case(disabled,
                           {SIMPLE_PERIMETER_GENERATOR, EXTRA_PERIMETER_COUNT},
                           area,
                           0,
                           {{"extra_perimeters_count", "1"}},
                           &left_override_area);
    REQUIRE(external_perimeter_count(overlap_run) == 2);
    const VerticalSplitCounts overlap_split =
        vertical_split_counts(external_perimeters(overlap_run), 0);
    REQUIRE(overlap_split.crossing == 1);
    REQUIRE(overlap_split.left_only == 1);
    REQUIRE(overlap_split.right_only == 0);
    require_leaf_fill_area_consistency(overlap_run);

    // Zero base perimeter: extra_perimeters_count still asks for real loops.
    const DynamicPrintConfig no_base =
        perimeter_config({{"perimeters", "0"}, {"extra_perimeters_count", "2"}});
    const PerimeterRunCapture no_base_run =
        run_perimeter_case(no_base, {SIMPLE_PERIMETER_GENERATOR, EXTRA_PERIMETER_COUNT}, area, 0);
    REQUIRE(external_perimeter_count(no_base_run) == 2);
    require_leaf_fill_area_consistency(no_base_run);
}
TEST_CASE("Extra perimeter count owns its setting and activation rule",
          "[plugins][perimeter][count][gui-rules]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    Plugin *plugin = orchestrator.get_plugin("perimeter.module.extra_perimeter_count");
    REQUIRE(plugin != nullptr);

    std::vector<std::string> defined_keys;
    for (const Plugin::DefinedConfigKey &key : plugin->get_defined_config_keys())
        defined_keys.push_back(key.key);
    CHECK(defined_keys.size() == 1);
    CHECK(defined_keys.front() == "extra_perimeters_count");

    const ConfigOptionDef *def = PrintConfigDef::instance().get("extra_perimeters_count");
    REQUIRE(def != nullptr);
    CHECK(def->container_type == ConfigOptionContainerType::Region);
    CHECK(def->invalidates_step == STEP_PERIMETER);
    CHECK(PrintConfig::defaults().option("extra_perimeters_count") == nullptr);
    CHECK(PrintRegionConfig::defaults().option("extra_perimeters_count") != nullptr);

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    const auto enabled = [&orchestrator, &config]() {
        for (const Orchestrator::GuiRuleApplication &application :
             orchestrator.gui_rules().evaluate(config, RAW_PRESET_TYPE_FFF_PRINT, -1))
            if (application.target_key == "extra_perimeters_count")
                return application.enabled;
        return false;
    };

    config.set_deserialize("step_perimeter_plugin", "perimeter.generator.simple");
    config.option<ConfigOptionInt>("perimeters")->value = 0;
    CHECK_FALSE(enabled());
    config.option<ConfigOptionInt>("perimeters")->value = 2;
    CHECK(enabled());
}