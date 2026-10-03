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

void require_default_loop_count(const PerimeterRunCapture &run, const size_t expected_count)
{
    REQUIRE(external_perimeter_count(run) == expected_count);
    REQUIRE(count_loops_with_role(external_perimeters(run), elrDefault) == expected_count);
}
}

TEST_CASE("Extra perimeter odd layer module is layer-parity dependent", "[plugins][perimeter]")
{
    const ExPolygon area = rectangle_expolygon(-10., -10., 10., 10.);
    const DynamicPrintConfig disabled = perimeter_config({{"extra_perimeters_odd_layers", "0"}});
    const DynamicPrintConfig enabled = perimeter_config({{"extra_perimeters_odd_layers", "1"}});

    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, enabled);
    const size_t odd_idx = layer_index_for_odd_layer(prepared.print.object(0));

    const ExPolygon left_override_area = rectangle_expolygon(-10., -10., 0., 10.);

    SECTION("Even layer is inert")
    {
        // The setting is enabled globally, but layer 0 is even, so only the base
        // loop is generated.
        const PerimeterRunCapture run =
            run_perimeter_case(enabled, {SIMPLE_PERIMETER_GENERATOR, EXTRA_PERIMETER_ODD_LAYER}, area, 0);
        require_default_loop_count(run, 1);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("Odd layer adds one uniform perimeter")
    {
        // The whole island uses extra_perimeters_odd_layers, so an odd layer gets
        // one extra nested loop in addition to the base loop.
        const PerimeterRunCapture run =
            run_perimeter_case(enabled, {SIMPLE_PERIMETER_GENERATOR, EXTRA_PERIMETER_ODD_LAYER}, area, odd_idx);
        require_default_loop_count(run, 2);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("Region-local enabled area adds only one local perimeter")
    {
        // The base config disables the setting. A left-side region
        // enables it, so the base loop still crosses x=0 and the extra loop must
        // stay on the left side.
        const PerimeterRunCapture run =
            run_perimeter_case(disabled,
                               {SIMPLE_PERIMETER_GENERATOR, EXTRA_PERIMETER_ODD_LAYER},
                               area,
                               odd_idx,
                               {{"extra_perimeters_odd_layers", "1"}},
                               &left_override_area);
        require_default_loop_count(run, 2);
        const VerticalSplitCounts split = vertical_split_counts(external_perimeters(run), 0);
        REQUIRE(split.crossing == 1);
        REQUIRE(split.left_only == 1);
        REQUIRE(split.right_only == 0);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("Region-local disabled area prevents one local perimeter")
    {
        // The base config enables the setting. A left-side region
        // disables it, so the base loop still crosses x=0 and the extra loop
        // is generated only on the enabled right side.
        const PerimeterRunCapture run =
            run_perimeter_case(enabled,
                               {SIMPLE_PERIMETER_GENERATOR, EXTRA_PERIMETER_ODD_LAYER},
                               area,
                               odd_idx,
                               {{"extra_perimeters_odd_layers", "0"}},
                               &left_override_area);
        require_default_loop_count(run, 2);
        const VerticalSplitCounts split = vertical_split_counts(external_perimeters(run), 0);
        REQUIRE(split.crossing == 1);
        REQUIRE(split.left_only == 0);
        REQUIRE(split.right_only == 1);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("Zero base perimeter still allows the odd-layer extra")
    {
        // The host root node is only a traversal seed. With zero base perimeters,
        // the module should still request one real loop on odd layers, without
        // turning the seed into a hidden base perimeter.
        const DynamicPrintConfig no_base =
            perimeter_config({{"perimeters", "0"}, {"extra_perimeters_odd_layers", "1"}});
        PreparedPerimeterPrint no_base_prepared;
        prepare_cube_print(no_base_prepared, no_base);
        const size_t no_base_odd_idx = layer_index_for_odd_layer(no_base_prepared.print.object(0));
        const PerimeterRunCapture run =
            run_perimeter_case(no_base, {SIMPLE_PERIMETER_GENERATOR, EXTRA_PERIMETER_ODD_LAYER}, area, no_base_odd_idx);
        require_default_loop_count(run, 1);
        require_leaf_fill_area_consistency(run);
    }
}
TEST_CASE("Extra perimeter odd layer owns its setting and activation rule",
          "[plugins][perimeter][odd-layer][gui-rules]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    Plugin *plugin = orchestrator.get_plugin("perimeter.module.extra_perimeter_odd_layer");
    REQUIRE(plugin != nullptr);

    std::vector<std::string> defined_keys;
    for (const Plugin::DefinedConfigKey &key : plugin->get_defined_config_keys())
        defined_keys.push_back(key.key);
    CHECK(defined_keys.size() == 1);
    CHECK(defined_keys.front() == "extra_perimeters_odd_layers");

    const ConfigOptionDef *def = PrintConfigDef::instance().get("extra_perimeters_odd_layers");
    REQUIRE(def != nullptr);
    CHECK(def->container_type == ConfigOptionContainerType::Region);
    CHECK(def->invalidates_step == STEP_PERIMETER);
    CHECK(PrintConfig::defaults().option("extra_perimeters_odd_layers") == nullptr);
    CHECK(PrintRegionConfig::defaults().option("extra_perimeters_odd_layers") != nullptr);

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    const auto enabled = [&orchestrator, &config]() {
        for (const Orchestrator::GuiRuleApplication &application :
             orchestrator.gui_rules().evaluate(config, RAW_PRESET_TYPE_FFF_PRINT, -1))
            if (application.target_key == "extra_perimeters_odd_layers")
                return application.enabled;
        return false;
    };

    config.set_deserialize("step_perimeter_plugin", "perimeter.generator.simple");
    config.option<ConfigOptionInt>("perimeters")->value = 0;
    CHECK_FALSE(enabled());
    config.option<ConfigOptionInt>("perimeters")->value = 2;
    CHECK(enabled());
}