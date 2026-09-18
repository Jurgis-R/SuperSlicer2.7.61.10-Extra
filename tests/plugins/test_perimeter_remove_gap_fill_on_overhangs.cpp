#include <catch2/catch.hpp>

#include "perimeter_test_helpers.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"

namespace {
using namespace Slic3r;
using namespace Slic3r::Test::PerimeterPluginTests;
}

TEST_CASE("Remove gap fill on overhangs clips unsupported open paths", "[plugins][perimeter]")
{
    // The current simple perimeter generator emits closed loops, while this
    // module edits open gap-fill candidates. These sections call the module
    // directly with a synthetic node containing one open gap-fill path.
    const DynamicPrintConfig disabled = perimeter_config({{"gap_fill_no_overhang", "0"}});
    const DynamicPrintConfig enabled = perimeter_config({{"gap_fill_no_overhang", "1"}});

    SECTION("Disabled setting leaves the open path unchanged")
    {
        double disabled_length = 0.;
        const size_t disabled_count = run_remove_gap_fill_module(disabled, false, &disabled_length);
        REQUIRE(disabled_count == 1);
        REQUIRE(disabled_length > 0.);
    }

    SECTION("Enabled setting removes an unsupported path")
    {
        double enabled_length = 0.;
        const size_t enabled_count = run_remove_gap_fill_module(enabled, false, &enabled_length);
        REQUIRE(enabled_count == 0);
        REQUIRE(enabled_length == 0.);
    }

    SECTION("Lower island support keeps the path")
    {
        // On layer 1 the synthetic node is supported by the layer below. Even
        // with the setting enabled, the forbidden area is empty and the path
        // must survive unchanged.
        double disabled_length = 0.;
        const size_t disabled_count = run_remove_gap_fill_module(disabled, false, &disabled_length, 1);
        double supported_length = 0.;
        const size_t supported_count = run_remove_gap_fill_module(enabled, false, &supported_length, 1);
        REQUIRE(disabled_count == 1);
        REQUIRE(supported_count == 1);
        REQUIRE(supported_length == Approx(disabled_length));
    }

    SECTION("Region-local enabled area clips only that side")
    {
        // The default config disables the setting. A right-side region enables
        // it, so only the part of the open path inside that unsupported region
        // is removed; the remaining left fragment stays printable.
        double disabled_length = 0.;
        const size_t disabled_count = run_remove_gap_fill_module(disabled, false, &disabled_length);
        double local_length = 0.;
        const size_t local_count = run_remove_gap_fill_module(disabled, true, &local_length);
        REQUIRE(disabled_count == 1);
        REQUIRE(local_count == 1);
        REQUIRE(local_length > 0.);
        REQUIRE(local_length < disabled_length);
    }
}

TEST_CASE("Remove gap fill on overhangs preserves perimeter tree fill areas", "[plugins][perimeter]")
{
    // Full pipeline smoke test: the module should be inert for the simple
    // generator's closed perimeter loops, and it must not damage the leaf
    // area/fill-area partition consumed later by infill.
    const DynamicPrintConfig enabled = perimeter_config({{"gap_fill_no_overhang", "1"}});
    const ExPolygon area = rectangle_expolygon(-10., -10., 10., 10.);
    const PerimeterRunCapture run =
        run_perimeter_case(enabled, {SIMPLE_PERIMETER_GENERATOR, REMOVE_GAP_FILL_ON_OVERHANGS}, area, 0);
    REQUIRE(external_perimeter_count(run) == 1);
    require_simple_generator_first_child_area_partition(run, area);
}

TEST_CASE("Remove gap fill on overhangs honors cancellation before nested geometry work",
          "[plugins][perimeter][cancellation]")
{
    // The direct fixture enters the module normally, then requests cancellation
    // immediately before module_after(). The callback must unwind before it
    // clips paths or replaces the node extrusion batch.
    Slic3r::Orchestrator &orchestrator = Slic3r::Orchestrator::instance();
    orchestrator.reset_plugin_cancel();
    CHECK_THROWS_AS(
        run_remove_gap_fill_module(perimeter_config({{"gap_fill_no_overhang", "1"}}),
                                   false,
                                   nullptr,
                                   0,
                                   true),
        slic3r_api::PluginCancelled);
    orchestrator.reset_plugin_cancel();
}
