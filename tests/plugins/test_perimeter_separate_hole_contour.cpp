#include <catch2/catch.hpp>

#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/Config/PrintConfig.hpp"

#include <initializer_list>
#include <string>
#include <utility>

namespace {
using namespace Slic3r;
using namespace Slic3r::Test::PerimeterPluginTests;

struct LoopCounts
{
    size_t total = 0;
    size_t contours = 0;
    size_t holes = 0;
};

Polygon clockwise_hole(const double min_x, const double min_y, const double max_x, const double max_y)
{
    Polygon hole({
        Point(scale_i(min_x), scale_i(min_y)),
        Point(scale_i(min_x), scale_i(max_y)),
        Point(scale_i(max_x), scale_i(max_y)),
        Point(scale_i(max_x), scale_i(min_y))
    });
    hole.make_clockwise();
    return hole;
}

ExPolygon rectangle_with_hole(const double outer_min_x,
                              const double outer_min_y,
                              const double outer_max_x,
                              const double outer_max_y,
                              const double hole_min_x,
                              const double hole_min_y,
                              const double hole_max_x,
                              const double hole_max_y)
{
    ExPolygon out = rectangle_expolygon(outer_min_x, outer_min_y, outer_max_x, outer_max_y);
    out.holes.push_back(clockwise_hole(hole_min_x, hole_min_y, hole_max_x, hole_max_y));
    return out;
}

ExPolygon rectangle_with_two_holes()
{
    ExPolygon out = rectangle_expolygon(-14., -10., 14., 10.);
    out.holes.push_back(clockwise_hole(-8., -3., -4., 3.));
    out.holes.push_back(clockwise_hole(4., -3., 8., 3.));
    return out;
}

ExPolygon rectangle_with_large_and_small_holes()
{
    ExPolygon out = rectangle_expolygon(-14., -10., 14., 10.);
    out.holes.push_back(clockwise_hole(-8., -4., -2., 4.));
    out.holes.push_back(clockwise_hole(5.5, -1., 6.5, 1.));
    return out;
}

ExPolygon rectangle_with_close_holes()
{
    ExPolygon out = rectangle_expolygon(-12., -10., 12., 10.);
    out.holes.push_back(clockwise_hole(-3.2, -3., -0.4, 3.));
    out.holes.push_back(clockwise_hole(0.4, -3., 3.2, 3.));
    return out;
}

ExPolygon concave_with_hole()
{
    Polygon contour({
        Point(scale_i(-12.), scale_i(-10.)),
        Point(scale_i(12.), scale_i(-10.)),
        Point(scale_i(12.), scale_i(-2.)),
        Point(scale_i(3.), scale_i(-2.)),
        Point(scale_i(3.), scale_i(10.)),
        Point(scale_i(-12.), scale_i(10.))
    });
    contour.make_counter_clockwise();

    ExPolygon out(contour);
    out.holes.push_back(clockwise_hole(-8., -6., -5., -3.));
    return out;
}

ExPolygon thin_ring()
{
    return rectangle_with_hole(-10., -10., 10., 10., -8., -8., 8., 8.);
}

ExPolygon vertical_slot_hole()
{
    return rectangle_with_hole(-10., -10., 10., 10., -1., -8., 1., 8.);
}

ExPolygon close_to_edge_hole()
{
    return rectangle_with_hole(-10., -10., 10., 10., 7.4, -3.5, 9.3, 3.5);
}

ExPolygon central_and_close_to_edge_holes()
{
    ExPolygon out = rectangle_expolygon(-12., -10., 12., 10.);
    out.holes.push_back(clockwise_hole(-4., -3., -1., 3.));
    out.holes.push_back(clockwise_hole(8.4, -3., 11.1, 3.));
    return out;
}

DynamicPrintConfig separate_config(const int perimeters, const std::string &perimeters_hole)
{
    return perimeter_config({
        {"perimeters", std::to_string(perimeters)},
        {"perimeters_hole", perimeters_hole}
    });
}

PerimeterRunCapture run_separate_case(const DynamicPrintConfig &config, const ExPolygon &surface)
{
    return run_perimeter_case(config, {SIMPLE_PERIMETER_GENERATOR, SEPARATE_HOLE_CONTOUR}, surface, 0);
}

PerimeterRunCapture run_separate_case(const int perimeters,
                                      const std::string &perimeters_hole,
                                      const ExPolygon &surface)
{
    return run_separate_case(separate_config(perimeters, perimeters_hole), surface);
}

LoopCounts loop_counts(const PerimeterRunCapture &capture)
{
    LoopCounts counts;
    counts.total = external_perimeter_count(capture);
    counts.contours = count_loops_with_role(external_perimeters(capture), elrDefault);
    counts.holes = count_loops_with_role(external_perimeters(capture), elrHole);
    return counts;
}

void check_loop_counts(const PerimeterRunCapture &capture, const size_t expected_contours, const size_t expected_holes)
{
    const LoopCounts counts = loop_counts(capture);
    CHECK(counts.contours == expected_contours);
    CHECK(counts.holes == expected_holes);
    CHECK(counts.total == expected_contours + expected_holes);
}

void check_no_unknown_simple_loops(const PerimeterRunCapture &capture)
{
    const LoopCounts counts = loop_counts(capture);
    CHECK(counts.total == counts.contours + counts.holes);
}

} // namespace

TEST_CASE("Separate hole contour configuration matrix", "[plugins][perimeter][separate-hole-contour]")
{
    const ExPolygon surface = rectangle_with_hole_expolygon();

    SECTION("01 disabled perimeters_hole keeps the normal contour/hole count")
    {
        // CASE 01: perimeters_hole is disabled. Hole loops and contour loops
        // should both use the normal perimeter count.
        const PerimeterRunCapture run = run_separate_case(3, "!0", surface);
        check_loop_counts(run, 3, 3);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("02 enabled perimeters_hole equal to perimeters is inert")
    {
        // CASE 02: enabling the option with the same value as perimeters should
        // not remove either class of loop.
        const PerimeterRunCapture run = run_separate_case(3, "3", surface);
        check_loop_counts(run, 3, 3);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("03 fewer hole perimeters stop holes before contours")
    {
        // CASE 03: the common use case keeps all contour perimeters but stops
        // hole loops after the configured hole count.
        const PerimeterRunCapture run = run_separate_case(3, "1", surface);
        check_loop_counts(run, 3, 1);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("04 more hole perimeters than contours keep generating holes")
    {
        // CASE 04: the inverse configuration asks for more shells around holes
        // than around the outside contour.
        const PerimeterRunCapture run = run_separate_case(1, "3", surface);
        check_loop_counts(run, 1, 3);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("05 zero hole perimeters removes all hole loops")
    {
        // CASE 05: perimeters_hole=0 means holes should not receive perimeter
        // loops while the outer contour still does.
        const PerimeterRunCapture run = run_separate_case(3, "0", surface);
        check_loop_counts(run, 3, 0);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("06 zero contour perimeters can still keep requested hole loops")
    {
        // CASE 06: perimeters=0 with a positive perimeters_hole value is a
        // holes-only request. It protects the generator seed/extra request path.
        const PerimeterRunCapture run = run_separate_case(0, "3", surface);
        check_loop_counts(run, 0, 3);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("07 zero contour and zero hole perimeters generate no loops")
    {
        // CASE 07: perimeters_hole is enabled, but equal to perimeters. The
        // module should be inert; the no-loop result comes from the simple
        // generator handling perimeters=0, not from SeparateHoleContour.
        const PerimeterRunCapture run = run_separate_case(0, "0", surface);
        check_loop_counts(run, 0, 0);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("08 large contour/hole difference propagates through children")
    {
        // CASE 08: a 5/2 split needs state propagation over several generated
        // child nodes, not just one root-level deletion.
        const PerimeterRunCapture run = run_separate_case(5, "2", surface);
        check_loop_counts(run, 5, 2);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("09 large hole/contour difference propagates through children")
    {
        // CASE 09: a 2/5 split exercises the same propagation path in the
        // opposite direction.
        const PerimeterRunCapture run = run_separate_case(2, "5", surface);
        check_loop_counts(run, 2, 5);
        require_leaf_fill_area_consistency(run);
    }
}

TEST_CASE("Separate hole contour geometry cases", "[plugins][perimeter][separate-hole-contour]")
{
    SECTION("10 wide central hole is the stable baseline shape")
    {
        // CASE 10: a large centered hole should keep exact independent contour
        // and hole counts without topology changes.
        const PerimeterRunCapture run = run_separate_case(4, "2", rectangle_with_hole_expolygon());
        check_loop_counts(run, 4, 2);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("11 no-hole island ignores hole-specific count")
    {
        // CASE 11: if the island has no holes, perimeters_hole must not remove
        // or add anything to the ordinary contour perimeter chain.
        const PerimeterRunCapture run = run_separate_case(3, "1", rectangle_expolygon(-10., -10., 10., 10.));
        check_loop_counts(run, 3, 0);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("12 two equal holes both receive the configured hole count")
    {
        // CASE 12: multiple holes in the same island should be treated as the
        // same loop class and should all stop at the same configured depth.
        const PerimeterRunCapture run = run_separate_case(3, "2", rectangle_with_two_holes());
        check_loop_counts(run, 3, 4);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("13 large and small holes do not require the small hole to survive")
    {
        // CASE 13: a tiny hole may disappear geometrically before the requested
        // count. The module must not use that disappearance as permission to
        // corrupt the remaining contour chain.
        const PerimeterRunCapture run = run_separate_case(4, "4", rectangle_with_large_and_small_holes());
        const LoopCounts counts = loop_counts(run);
        CHECK(counts.contours == 4);
        CHECK(counts.holes <= 8);
        CHECK(counts.holes >= 4);
        check_no_unknown_simple_loops(run);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("14 very small hole can still receive the requested count")
    {
        // CASE 14: a small hole is not like a small island contour. Hole
        // perimeter offsets expand into the surrounding material, so even a
        // tiny valid hole can keep receiving the requested hole count.
        const PerimeterRunCapture run =
            run_separate_case(5, "5", rectangle_with_hole(-10., -10., 10., 10., -0.05, -0.05, 0.05, 0.05));
        const LoopCounts counts = loop_counts(run);
        CHECK(counts.contours == 5);
        CHECK(counts.holes == 5);
        check_no_unknown_simple_loops(run);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("15 hole near outer contour may merge with contour offsets")
    {
        // CASE 15: a hole close to the outside contour can stop being a hole
        // after one or more offsets. The module must follow the generated loop
        // roles instead of assuming the initial topology stays unchanged.
        const PerimeterRunCapture run = run_separate_case(4, "4", close_to_edge_hole());
        const LoopCounts counts = loop_counts(run);
        CHECK(counts.contours >= 1);
        CHECK(counts.holes >= 1);
        CHECK(counts.holes <= 4);
        check_no_unknown_simple_loops(run);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("16 close holes may merge with each other while counts remain bounded")
    {
        // CASE 16: two nearby holes can fuse into one generated hole chain. The
        // module must not keep counting missing original holes forever.
        const PerimeterRunCapture run = run_separate_case(4, "4", rectangle_with_close_holes());
        const LoopCounts counts = loop_counts(run);
        CHECK(counts.contours == 4);
        CHECK(counts.holes >= 4);
        CHECK(counts.holes <= 8);
        check_no_unknown_simple_loops(run);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("17 concave island with a hole keeps valid child areas")
    {
        // CASE 17: concave contours stress the child-area rebuild because the
        // remaining kept loops do not form a simple rectangle-like shell.
        const PerimeterRunCapture run = run_separate_case(3, "1", concave_with_hole());
        const LoopCounts counts = loop_counts(run);
        CHECK(counts.contours == 3);
        CHECK(counts.holes == 1);
        check_no_unknown_simple_loops(run);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("18 thin ring can run out of area without invalid leaves")
    {
        // CASE 18: a thin ring is quickly consumed by offsets. The result may
        // contain fewer loops than requested, but the produced leaves must stay
        // valid.
        const PerimeterRunCapture run = run_separate_case(5, "5", thin_ring());
        const LoopCounts counts = loop_counts(run);
        CHECK(counts.contours <= 5);
        CHECK(counts.holes <= 5);
        check_no_unknown_simple_loops(run);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("19 slot-like hole can split remaining child areas")
    {
        // CASE 19: a long slot hole can split the remaining interior into two
        // branches. Once the tree splits, several contour loops can be emitted
        // at the same shell depth, so the useful invariant is "at least the
        // requested contour chain survived" rather than an exact loop count.
        const PerimeterRunCapture run = run_separate_case(4, "1", vertical_slot_hole());
        const LoopCounts counts = loop_counts(run);
        CHECK(counts.contours >= 4);
        CHECK(counts.holes == 1);
        check_no_unknown_simple_loops(run);
        require_leaf_fill_area_consistency(run);
    }
}

TEST_CASE("Separate hole contour tree rebuild cases", "[plugins][perimeter][separate-hole-contour]")
{
    const ExPolygon surface = rectangle_with_hole_expolygon();

    SECTION("20 removing only hole loops rebuilds children around kept contours")
    {
        // CASE 20: when only hole loops are removed, the next child areas should
        // be rebuilt from the kept contour coverage.
        const PerimeterRunCapture run = run_separate_case(3, "0", surface);
        check_loop_counts(run, 3, 0);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("21 removing only contour loops rebuilds children around kept holes")
    {
        // CASE 21: when only contour loops are removed, hole-only generation
        // still needs valid children around the kept hole coverage.
        const PerimeterRunCapture run = run_separate_case(0, "3", surface);
        check_loop_counts(run, 0, 3);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("22 equal zero counts are inert in direct module calls")
    {
        // CASE 22: perimeters_hole=0 with perimeters=0 is still an equal-count
        // configuration. The module should not erase artificial loops handed to
        // it by this direct harness; real no-loop behavior is tested on the
        // simple generator and on the full 0/0 pipeline case above.
        const SeparateHoleContourDirectResult result =
            run_separate_hole_contour_module_direct(separate_config(0, "0"), 0, 1, 1, 1);
        CHECK(result.contours == 1);
        CHECK(result.holes == 1);
        CHECK(result.total == 2);
        CHECK(result.perimeter_needed == 1);
    }

    SECTION("23 missing loop class does not advance the deletion counter")
    {
        // CASE 23: asking to remove holes from a node that has no hole loop
        // should not count as a successful deletion.
        const SeparateHoleContourDirectResult result =
            run_separate_hole_contour_module_direct(separate_config(1, "0"), 0, 1, 1, 0);
        CHECK(result.contours == 1);
        CHECK(result.holes == 0);
        CHECK(result.perimeter_needed == 1);
    }

    SECTION("24 equal positive counts are inert in direct module calls")
    {
        // CASE 24: when perimeters_hole equals perimeters, the separate
        // contour/hole module has no delta to apply and should leave node
        // state and extrusion classes unchanged.
        const SeparateHoleContourDirectResult result =
            run_separate_hole_contour_module_direct(separate_config(2, "2"), 0, 1, 1, 1);
        CHECK(result.contours == 1);
        CHECK(result.holes == 1);
        CHECK(result.total == 2);
        CHECK(result.perimeter_needed == 1);
    }

    SECTION("25 equal zero counts stay inert even with fake pending depth")
    {
        // CASE 25: this direct harness can create impossible state: equal zero
        // counts but pending perimeter depth. The module should still be inert
        // because equal counts mean there is no separate contour/hole policy to
        // apply.
        const SeparateHoleContourDirectResult result =
            run_separate_hole_contour_module_direct(separate_config(0, "0"), 0, 3, 1, 1);
        CHECK(result.contours == 1);
        CHECK(result.holes == 1);
        CHECK(result.total == 2);
        CHECK(result.children == 0);
        CHECK(result.perimeter_needed == 3);
    }
}

TEST_CASE("Separate hole contour region and interaction cases", "[plugins][perimeter][separate-hole-contour]")
{
    const ExPolygon surface = rectangle_with_hole_expolygon();

    SECTION("26 partial region override is currently ignored rather than half-applied")
    {
        // CASE 26: region-varying perimeters_hole is not implemented yet. Until
        // it is, a mixed config must be inert instead of partially corrupting
        // the island tree.
        const DynamicPrintConfig config = separate_config(3, "!0");
        const PerimeterRunCapture run =
            run_perimeter_case(config,
                               {SIMPLE_PERIMETER_GENERATOR, SEPARATE_HOLE_CONTOUR},
                               surface,
                               0,
                               {{"perimeters_hole", "1"}});
        check_loop_counts(run, 3, 3);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("27 tiny corner region override is inert while mixed values are unsupported")
    {
        // CASE 27: the test helper only accepts regions intersecting the island.
        // Use a tiny corner region to document the same current behavior: mixed
        // values are ignored instead of being half-applied.
        const DynamicPrintConfig config = separate_config(3, "!0");
        const ExPolygon corner = rectangle_expolygon(-10., -10., -8., -8.);
        const PerimeterRunCapture run =
            run_perimeter_case(config,
                               {SIMPLE_PERIMETER_GENERATOR, SEPARATE_HOLE_CONTOUR},
                               surface,
                               0,
                               {{"perimeters_hole", "1"}},
                               &corner);
        check_loop_counts(run, 3, 3);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("28 partial region override remains a documented no-op")
    {
        // CASE 28: this repeats the unsupported mixed-region path with an
        // explicit half-island area, documenting the current restriction.
        const DynamicPrintConfig config = separate_config(3, "!0");
        const ExPolygon half = rectangle_expolygon(-10., -10., 0., 10.);
        const PerimeterRunCapture run =
            run_perimeter_case(config,
                               {SIMPLE_PERIMETER_GENERATOR, SEPARATE_HOLE_CONTOUR},
                               surface,
                               0,
                               {{"perimeters_hole", "1"}},
                               &half);
        check_loop_counts(run, 3, 3);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("29 simple generator integration has no unknown loop classes")
    {
        // CASE 29: the main intended generator should produce only contour and
        // hole loops after the module has run.
        const PerimeterRunCapture run = run_separate_case(3, "1", surface);
        check_no_unknown_simple_loops(run);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("30 extra perimeter count composes with hole/contour separation")
    {
        // CASE 30: another module may request more children before
        // SeparateHoleContour runs. That request adds one contour-capable and
        // one hole-capable shell, so the configured contour/hole differential
        // must stay the same instead of keeping the raw perimeters_hole value.
        const DynamicPrintConfig config = perimeter_config({
            {"perimeters", "2"},
            {"perimeters_hole", "1"},
            {"extra_perimeters_count", "1"}
        });
        const PerimeterRunCapture run =
            run_perimeter_case(config, {SIMPLE_PERIMETER_GENERATOR, EXTRA_PERIMETER_COUNT, SEPARATE_HOLE_CONTOUR}, surface, 0);
        check_loop_counts(run, 3, 2);
        check_no_unknown_simple_loops(run);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("30b extra perimeter count keeps hole-greater-than-contour differential")
    {
        // CASE 30b: perimeters_hole asks for three more hole shells than
        // contours. ExtraPerimeterCount adds two shells before
        // SeparateHoleContour, so the final counts must become 4 contours and 7
        // holes, not the raw configured 2 contours and 5 holes.
        const ExPolygon roomy_surface = rectangle_with_hole(-25., -25., 25., 25., -5., -5., 5., 5.);
        const DynamicPrintConfig config = perimeter_config({
            {"perimeters", "2"},
            {"perimeters_hole", "5"},
            {"extra_perimeters_count", "2"}
        });
        const PerimeterRunCapture run =
            run_perimeter_case(config, {SIMPLE_PERIMETER_GENERATOR, EXTRA_PERIMETER_COUNT, SEPARATE_HOLE_CONTOUR}, roomy_surface, 0);
        check_loop_counts(run, 4, 7);
        check_no_unknown_simple_loops(run);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("30c extra perimeter count keeps contour-greater-than-hole differential")
    {
        // CASE 30c: symmetric to 30b. Contours start three shells ahead of
        // holes, and ExtraPerimeterCount adds two shells to both classes. The
        // final result must therefore be 7 contour shells and 4 hole shells.
        const ExPolygon roomy_surface = rectangle_with_hole(-25., -25., 25., 25., -5., -5., 5., 5.);
        const DynamicPrintConfig config = perimeter_config({
            {"perimeters", "5"},
            {"perimeters_hole", "2"},
            {"extra_perimeters_count", "2"}
        });
        const PerimeterRunCapture run =
            run_perimeter_case(config, {SIMPLE_PERIMETER_GENERATOR, EXTRA_PERIMETER_COUNT, SEPARATE_HOLE_CONTOUR}, roomy_surface, 0);
        check_loop_counts(run, 7, 4);
        check_no_unknown_simple_loops(run);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("31 only-one-perimeter module can reduce the same tree safely")
    {
        // CASE 31: modules that reduce requested perimeter depth should not
        // leave SeparateHoleContour with stale node state.
        const DynamicPrintConfig config = perimeter_config({
            {"perimeters", "4"},
            {"perimeters_hole", "1"},
            {"only_one_perimeter_first_layer", "1"}
        });
        const PerimeterRunCapture run =
            run_perimeter_case(config, {SIMPLE_PERIMETER_GENERATOR, ONLY_ONE_PERIMETER_FIRST_LAYER, SEPARATE_HOLE_CONTOUR}, surface, 0);
        const LoopCounts counts = loop_counts(run);
        CHECK(counts.contours <= 4);
        CHECK(counts.holes <= 1);
        check_no_unknown_simple_loops(run);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("32 arachne generator path stays defined")
    {
        // CASE 32: Arachne has a different loop-generation backend. The module
        // should at least leave a valid perimeter tree when both are active.
        const PerimeterRunCapture run =
            run_perimeter_case(separate_config(3, "1"), {ARACHNE_PERIMETER_GENERATOR, SEPARATE_HOLE_CONTOUR}, surface, 0);
        CHECK(external_perimeter_count(run) > 0);
        require_leaf_fill_area_consistency(run);
    }
}

TEST_CASE("Separate hole contour direct module cases", "[plugins][perimeter][separate-hole-contour]")
{
    SECTION("33 direct module removes only the requested loop class")
    {
        // CASE 33: call the module on an artificial node with one contour and
        // one hole so the erase-class branch is tested without generator noise.
        const SeparateHoleContourDirectResult result =
            run_separate_hole_contour_module_direct(separate_config(1, "0"), 0, 1, 1, 1);
        CHECK(result.contours == 1);
        CHECK(result.holes == 0);
    }

    SECTION("34 direct module keeps open gap-fill-like polylines")
    {
        // CASE 34: SeparateHoleContour is only allowed to erase closed perimeter
        // loops. Open local polylines must survive even when holes are removed.
        const SeparateHoleContourDirectResult result =
            run_separate_hole_contour_module_direct(separate_config(1, "0"), 0, 1, 1, 1, true);
        CHECK(result.contours == 1);
        CHECK(result.holes == 0);
        CHECK(result.total == 2);
    }

    SECTION("35 direct module does not treat an unclassified closed loop as a hole")
    {
        // CASE 35: a closed entity without the perimeter property is not a hole
        // loop and must not be removed by the hole-removal path.
        const SeparateHoleContourDirectResult result =
            run_separate_hole_contour_module_direct(separate_config(1, "0"), 0, 1, 0, 0, false, true);
        CHECK(result.holes == 0);
        CHECK(result.total == 1);
    }
}

TEST_CASE("Separate hole contour close-edge topology changes", "[plugins][perimeter][separate-hole-contour]")
{
    SECTION("36 close-edge hole-only generation stops when the hole becomes contour")
    {
        // CASE 36: a hole close enough to the border can produce one real hole
        // loop, then the next offset fuses into contour topology. Holes-only
        // generation must not invent a third hole loop after that contour was
        // removed.
        const PerimeterRunCapture run = run_separate_case(0, "3", close_to_edge_hole());
        const LoopCounts counts = loop_counts(run);
        CHECK(counts.contours == 0);
        CHECK(counts.holes == 1);
        check_no_unknown_simple_loops(run);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("37 close-edge contours-only keeps contours created after hole fusion")
    {
        // CASE 37: in contours-only mode, the original hole loops are removed,
        // but later loops that have become contour topology should remain.
        const PerimeterRunCapture run = run_separate_case(3, "0", close_to_edge_hole());
        const LoopCounts counts = loop_counts(run);
        CHECK(counts.contours >= 1);
        CHECK(counts.holes == 0);
        check_no_unknown_simple_loops(run);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("38 close-edge holes-greater-than-contours does not overcount missing holes")
    {
        // CASE 38: when holes request more depth than contours, a close-edge
        // hole that stops being a hole must stop the hole chain instead of
        // repeatedly requesting impossible hole loops.
        const PerimeterRunCapture run = run_separate_case(1, "3", close_to_edge_hole());
        const LoopCounts counts = loop_counts(run);
        CHECK(counts.contours <= 1);
        CHECK(counts.holes == 1);
        check_no_unknown_simple_loops(run);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("39 close-edge contours-greater-than-holes continues contour chain")
    {
        // CASE 39: after keeping the first hole loop, later generated contour
        // loops from the merged topology should be allowed to continue.
        const PerimeterRunCapture run = run_separate_case(3, "1", close_to_edge_hole());
        const LoopCounts counts = loop_counts(run);
        CHECK(counts.contours >= 1);
        CHECK(counts.holes == 1);
        check_no_unknown_simple_loops(run);
        require_leaf_fill_area_consistency(run);
    }

    SECTION("40 central and close-edge holes diverge independently")
    {
        // CASE 40: one central hole and one close-edge hole should not share a
        // naive global original-hole counter. The central hole may keep going
        // after the close-edge hole has fused into contour topology.
        const PerimeterRunCapture run = run_separate_case(0, "3", central_and_close_to_edge_holes());
        const LoopCounts counts = loop_counts(run);
        CHECK(counts.contours == 0);
        CHECK(counts.holes >= 3);
        CHECK(counts.holes < 6);
        check_no_unknown_simple_loops(run);
        require_leaf_fill_area_consistency(run);
    }
}
TEST_CASE("Separate hole contour owns its setting and activation rule",
          "[plugins][perimeter][separate-hole-contour][gui-rules]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    Plugin *plugin = orchestrator.get_plugin("perimeter.module.separate_hole_contour");
    REQUIRE(plugin != nullptr);

    std::vector<std::string> defined_keys;
    for (const Plugin::DefinedConfigKey &key : plugin->get_defined_config_keys())
        defined_keys.push_back(key.key);
    CHECK(defined_keys.size() == 1);
    CHECK(defined_keys.front() == "perimeters_hole");

    const ConfigOptionDef *def = PrintConfigDef::instance().get("perimeters_hole");
    REQUIRE(def != nullptr);
    CHECK(def->container_type == ConfigOptionContainerType::Region);
    CHECK(def->invalidates_step == STEP_PERIMETER);
    CHECK(PrintConfig::defaults().option("perimeters_hole") == nullptr);
    CHECK(PrintRegionConfig::defaults().option("perimeters_hole") != nullptr);

    // The not_arachne disjunction expands to two activation reasons. The
    // generated implementation rule of the plugin's own exclusive group stays
    // active, so the plugin rules are checked through their structure.
    size_t moved_rules = 0;
    for (const Orchestrator::PluginGuiRule &rule : orchestrator.gui_rules().rules()) {
        if (rule.owner != "perimeter.module.separate_hole_contour" ||
            rule.target_key != "perimeters_hole")
            continue;
        if (rule.clauses.size() != 1)
            continue;
        if (rule.clauses[0].condition_key != "perimeters" &&
            rule.clauses[0].condition_key != "perimeter_generator")
            continue;
        ++moved_rules;
        CHECK(rule.scope == RAW_PRESET_TYPE_FFF_PRINT);
    }
    CHECK(moved_rules == 2);
}