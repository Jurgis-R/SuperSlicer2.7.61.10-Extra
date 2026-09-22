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

TEST_CASE("Only one perimeter first layer limits first-layer branches", "[plugins][perimeter]")
{
    const ExPolygon area = rectangle_expolygon(-10., -10., 10., 10.);
    const ExPolygon left_half = rectangle_expolygon(-10., -10., 0., 10.);
    const ExPolygon right_half = rectangle_expolygon(0., -10., 10., 10.);
    const DynamicPrintConfig multi = perimeter_config({
        {"extra_perimeters_count", "3"},
        {"only_one_perimeter_first_layer", "0"}
    });
    const DynamicPrintConfig limited = perimeter_config({
        {"extra_perimeters_count", "3"},
        {"only_one_perimeter_first_layer", "1"}
    });

    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, limited);
    const size_t non_first_idx = layer_index_for_odd_layer(prepared.print.object(0));

    SECTION("Disabled setting keeps the requested multi-perimeter baseline")
    {
        // The rectangle is wide enough for one base perimeter plus the three
        // extra perimeters requested by ExtraPerimeterCount.
        const PerimeterRunCapture run =
            run_perimeter_case(multi,
                               {SIMPLE_PERIMETER_GENERATOR, EXTRA_PERIMETER_COUNT, ONLY_ONE_PERIMETER_FIRST_LAYER},
                               area,
                               0);
        require_default_loop_count(run, 4);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("Enabled setting clamps the first layer to one perimeter")
    {
        // The first perimeter is still emitted. The module runs after that ring
        // and clamps all child branches, so no second perimeter is generated.
        const PerimeterRunCapture run =
            run_perimeter_case(limited,
                               {SIMPLE_PERIMETER_GENERATOR, EXTRA_PERIMETER_COUNT, ONLY_ONE_PERIMETER_FIRST_LAYER},
                               area,
                               0);
        require_default_loop_count(run, 1);
        require_simple_generator_first_child_area_partition(run, area);
    }

    SECTION("Enabled setting is ignored after the first layer")
    {
        // Same config as the first-layer clamp, but layer 1 is not a first
        // layer. The module must be inert and keep the four-loop baseline.
        const PerimeterRunCapture run =
            run_perimeter_case(limited,
                               {SIMPLE_PERIMETER_GENERATOR, EXTRA_PERIMETER_COUNT, ONLY_ONE_PERIMETER_FIRST_LAYER},
                               area,
                               non_first_idx);
        require_default_loop_count(run, 4);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("Region-local enabled area clamps only that side")
    {
        // The base config disables the rule. A non-overlapping left-half
        // region enables it, so the first full loop still crosses x=0, the
        // active-left child branch stops, and the inactive-right branch keeps
        // its three extra perimeters.
        const PerimeterRunCapture run =
            run_perimeter_case(multi,
                               {SIMPLE_PERIMETER_GENERATOR, EXTRA_PERIMETER_COUNT, ONLY_ONE_PERIMETER_FIRST_LAYER},
                               area,
                               0,
                               {{"only_one_perimeter_first_layer", "1"}},
                               &left_half);
        require_default_loop_count(run, 4);
        const VerticalSplitCounts split_counts = vertical_split_counts(external_perimeters(run), scale_i(0.));
        REQUIRE(split_counts.crossing == 1);
        REQUIRE(split_counts.left_only == 0);
        REQUIRE(split_counts.right_only == 3);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("Complementary disabled and enabled areas are equivalent")
    {
        // These two configurations describe the same effective area:
        // - default disabled, right half enabled;
        // - default enabled, left half disabled.
        // LayerRegions are non-overlapping, so the region setting splitter must
        // see the same left/right partition in both cases.
        const PerimeterRunCapture right_enabled_run =
            run_perimeter_case(multi,
                               {SIMPLE_PERIMETER_GENERATOR, EXTRA_PERIMETER_COUNT, ONLY_ONE_PERIMETER_FIRST_LAYER},
                               area,
                               0,
                               {{"only_one_perimeter_first_layer", "1"}},
                               &right_half);
        const PerimeterRunCapture left_disabled_run =
            run_perimeter_case(limited,
                               {SIMPLE_PERIMETER_GENERATOR, EXTRA_PERIMETER_COUNT, ONLY_ONE_PERIMETER_FIRST_LAYER},
                               area,
                               0,
                               {{"only_one_perimeter_first_layer", "0"}},
                               &left_half);
        require_default_loop_count(right_enabled_run, 4);
        require_default_loop_count(left_disabled_run, 4);

        const VerticalSplitCounts right_enabled_split =
            vertical_split_counts(external_perimeters(right_enabled_run), scale_i(0.));
        const VerticalSplitCounts left_disabled_split =
            vertical_split_counts(external_perimeters(left_disabled_run), scale_i(0.));
        REQUIRE(right_enabled_split.crossing == 1);
        REQUIRE(right_enabled_split.left_only == 3);
        REQUIRE(right_enabled_split.right_only == 0);
        REQUIRE(left_disabled_split.crossing == right_enabled_split.crossing);
        REQUIRE(left_disabled_split.left_only == right_enabled_split.left_only);
        REQUIRE(left_disabled_split.right_only == right_enabled_split.right_only);
        require_leaf_fill_area_consistency(right_enabled_run);
        require_leaf_fill_area_consistency(left_disabled_run);
    }
}
TEST_CASE("Only one perimeter first layer owns its setting and activation rule",
          "[plugins][perimeter][only-one-first-layer][gui-rules]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    Plugin *plugin = orchestrator.get_plugin("perimeter.module.only_one_perimeter_first_layer");
    REQUIRE(plugin != nullptr);

    std::vector<std::string> defined_keys;
    for (const Plugin::DefinedConfigKey &key : plugin->get_defined_config_keys())
        defined_keys.push_back(key.key);
    CHECK(defined_keys.size() == 1);
    CHECK(defined_keys.front() == "only_one_perimeter_first_layer");

    const ConfigOptionDef *def = PrintConfigDef::instance().get("only_one_perimeter_first_layer");
    REQUIRE(def != nullptr);
    CHECK(def->container_type == ConfigOptionContainerType::Region);
    CHECK(def->invalidates_step == STEP_PERIMETER);
    CHECK(PrintConfig::defaults().option("only_one_perimeter_first_layer") == nullptr);
    CHECK(PrintRegionConfig::defaults().option("only_one_perimeter_first_layer") != nullptr);

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    const auto enabled = [&orchestrator, &config]() {
        for (const Orchestrator::GuiRuleApplication &application :
             orchestrator.gui_rules().evaluate(config, RAW_PRESET_TYPE_FFF_PRINT, -1))
            if (application.target_key == "only_one_perimeter_first_layer")
                return application.enabled;
        return false;
    };

    config.set_deserialize("step_perimeter_plugin", "perimeter.generator.simple");
    config.option<ConfigOptionInt>("perimeters")->value = 1;
    CHECK_FALSE(enabled());
    config.option<ConfigOptionInt>("perimeters")->value = 3;
    CHECK(enabled());
}