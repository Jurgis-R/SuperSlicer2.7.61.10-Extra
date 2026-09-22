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

TEST_CASE("Only one perimeter on top limits top branches", "[plugins][perimeter]")
{
    const ExPolygon area = rectangle_expolygon(-10., -10., 10., 10.);
    const ExPolygon left_half = rectangle_expolygon(-10., -10., 0., 10.);
    const ExPolygon right_half = rectangle_expolygon(0., -10., 10., 10.);
    const DynamicPrintConfig multi = perimeter_config({
        {"extra_perimeters_count", "3"},
        {"only_one_perimeter_top", "0"}
    });
    const DynamicPrintConfig limited = perimeter_config({
        {"extra_perimeters_count", "3"},
        {"only_one_perimeter_top", "1"}
    });

    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, multi);
    const size_t top_idx = layer_index_for_top(prepared.print.object(0));
    const size_t non_top_idx = layer_index_for_odd_layer(prepared.print.object(0));

    SECTION("Disabled setting keeps the requested top-layer baseline")
    {
        // The top rectangle is wide enough for one base perimeter plus the
        // three extra perimeters requested by ExtraPerimeterCount.
        const PerimeterRunCapture run =
            run_perimeter_case(multi,
                               {SIMPLE_PERIMETER_GENERATOR, EXTRA_PERIMETER_COUNT, ONLY_ONE_PERIMETER_ON_TOP},
                               area,
                               top_idx);
        require_default_loop_count(run, 4);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("Enabled setting clamps the top layer to one perimeter")
    {
        // With no upper island coverage, the whole area is a top surface. The
        // module stops every child branch after the first generated perimeter.
        const PerimeterRunCapture run =
            run_perimeter_case(limited,
                               {SIMPLE_PERIMETER_GENERATOR, EXTRA_PERIMETER_COUNT, ONLY_ONE_PERIMETER_ON_TOP},
                               area,
                               top_idx);
        require_default_loop_count(run, 1);
        require_simple_generator_first_child_area_partition(run, area);
    }

    SECTION("Enabled setting is ignored where an upper layer covers the island")
    {
        // Same config as the top clamp, but this layer has an upper island.
        // There is no top surface to clamp, so the four-loop baseline remains.
        const PerimeterRunCapture run =
            run_perimeter_case(limited,
                               {SIMPLE_PERIMETER_GENERATOR, EXTRA_PERIMETER_COUNT, ONLY_ONE_PERIMETER_ON_TOP},
                               area,
                               non_top_idx);
        require_default_loop_count(run, 4);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("Region-local enabled area clamps only that top side")
    {
        // The base config disables the rule. A non-overlapping left-half region
        // enables it, so the first full loop still crosses x=0, the active-left
        // child branch stops, and the inactive-right branch keeps its three
        // extra perimeters.
        const PerimeterRunCapture run =
            run_perimeter_case(multi,
                               {SIMPLE_PERIMETER_GENERATOR, EXTRA_PERIMETER_COUNT, ONLY_ONE_PERIMETER_ON_TOP},
                               area,
                               top_idx,
                               {{"only_one_perimeter_top", "1"}},
                               &left_half);
        require_default_loop_count(run, 4);
        const VerticalSplitCounts split_counts = vertical_split_counts(external_perimeters(run), scale_i(0.));
        REQUIRE(split_counts.crossing == 1);
        REQUIRE(split_counts.left_only == 0);
        REQUIRE(split_counts.right_only == 3);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("Complementary disabled and enabled top areas are equivalent")
    {
        // These two configurations describe the same effective top area:
        // - default disabled, right half enabled;
        // - default enabled, left half disabled.
        // LayerRegions are non-overlapping, so both partitions should clamp the
        // same side and leave the same side generating extra perimeters.
        const PerimeterRunCapture right_enabled_run =
            run_perimeter_case(multi,
                               {SIMPLE_PERIMETER_GENERATOR, EXTRA_PERIMETER_COUNT, ONLY_ONE_PERIMETER_ON_TOP},
                               area,
                               top_idx,
                               {{"only_one_perimeter_top", "1"}},
                               &right_half);
        const PerimeterRunCapture left_disabled_run =
            run_perimeter_case(limited,
                               {SIMPLE_PERIMETER_GENERATOR, EXTRA_PERIMETER_COUNT, ONLY_ONE_PERIMETER_ON_TOP},
                               area,
                               top_idx,
                               {{"only_one_perimeter_top", "0"}},
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

    SECTION("Partial top split preserves child area and fill area partitions")
    {
        // This layer is wider than the layer above it. The middle is covered by
        // the upper island, while the side bands are top surfaces. The module
        // therefore has to split the first child domain into stop/continue
        // pieces. The final fill outputs must still be a clean partition of the
        // simple generator's child areas.
        const ExPolygon wider_than_upper = rectangle_expolygon(-12., -10., 12., 10.);
        const PerimeterRunCapture run =
            run_perimeter_case(limited,
                               {SIMPLE_PERIMETER_GENERATOR, ONLY_ONE_PERIMETER_ON_TOP},
                               wider_than_upper,
                               non_top_idx);
        require_default_loop_count(run, 1);
        require_simple_generator_first_child_area_partition(run, wider_than_upper);
    }
}
TEST_CASE("Only one perimeter on top owns its settings and activation rule",
          "[plugins][perimeter][only-one-on-top][gui-rules]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    Plugin *plugin = orchestrator.get_plugin("perimeter.module.only_one_perimeter_on_top");
    REQUIRE(plugin != nullptr);

    std::vector<std::string> defined_keys;
    for (const Plugin::DefinedConfigKey &key : plugin->get_defined_config_keys())
        defined_keys.push_back(key.key);
    CHECK(defined_keys.size() == 2);
    CHECK(std::find(defined_keys.begin(), defined_keys.end(), "min_width_top_surface") != defined_keys.end());
    CHECK(std::find(defined_keys.begin(), defined_keys.end(), "only_one_perimeter_top_other_algo") != defined_keys.end());

    const ConfigOptionDef *def = PrintConfigDef::instance().get("min_width_top_surface");
    REQUIRE(def != nullptr);
    CHECK(def->container_type == ConfigOptionContainerType::Region);
    CHECK(def->invalidates_step == STEP_PERIMETER);
    CHECK(PrintConfig::defaults().option("min_width_top_surface") == nullptr);
    CHECK(PrintRegionConfig::defaults().option("min_width_top_surface") != nullptr);

    def = PrintConfigDef::instance().get("only_one_perimeter_top_other_algo");
    REQUIRE(def != nullptr);
    CHECK(def->container_type == ConfigOptionContainerType::Region);
    CHECK(PrintRegionConfig::defaults().option("only_one_perimeter_top_other_algo") != nullptr);

    // This is the only rule that uses the perimeters switch, so it is checked
    // through its structure: the generated implementation rule of the plugin's
    // own exclusive group may stay active.
    bool found_moved_rule = false;
    for (const Orchestrator::PluginGuiRule &rule : orchestrator.gui_rules().rules()) {
        if (rule.owner != "perimeter.module.only_one_perimeter_on_top" ||
            rule.target_key != "min_width_top_surface")
            continue;
        if (rule.clauses.size() != 2)
            continue;
        found_moved_rule = true;
        CHECK(rule.scope == RAW_PRESET_TYPE_FFF_PRINT);
        CHECK(rule.clauses[0].condition_key == "perimeters");
        CHECK(rule.clauses[1].condition_key == "only_one_perimeter_top");
    }
    CHECK(found_moved_rule);
}