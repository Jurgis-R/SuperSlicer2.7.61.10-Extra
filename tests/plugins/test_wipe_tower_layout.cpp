/* Pure tower layout tests use millimeter values with no plan, plugin runtime or
   generated extrusion dependency. They check whole-line bands, suffix depth,
   sparse height limits, floating-band placement and the legacy cone formula for
   any positive angle. */
#include <catch2/catch.hpp>
#include <cmath>
#include <stdexcept>
#include "libslic3r/Flow.hpp"
#include "libslic3r/Plugins/WipeTower/WipeTowerLayout.hpp"

using namespace slic3r_api::WipeTower;

// Reserve ordered bands with known volumes and line widths, asserting rounded spacing, alternating entry sides and the suffix depth without mutating source layers.
TEST_CASE("Tower layout rounds whole bands and propagates the suffix depth", "[plugins][wipe-tower][layout]")
{
    const std::vector<LayoutLayerInput> input{
        {0.2, 0.2, 0.4, {{0, 1., 0.5}, {1, 11., 0.5}}},
        {0.4, 0.2, 0.4, {{2, 60., 0.5}}},
        {0.6, 0.2, 0.4, {{3, 1., 0.5}}},
        {0.8, 0.2, 0.4, {}}
    };
    const TowerLayout result = layout_tower(10., 1.5, false, 0., input);
    REQUIRE(result.layers.size() == 3);
    CHECK(result.layers[0].height == Approx(0.2));
    REQUIRE(result.layers[0].bands.size() == 2);
    CHECK(result.layers[0].bands[0].y0 == 0.);
    const double pitch = Slic3r::Flow::new_from_width(0.5f, 0.4f, 0.2f, 1.f).spacing() * 1.5;
    CHECK(result.layers[0].bands[0].y1 == Approx(2. * pitch));
    CHECK(result.layers[0].bands[1].y0 == Approx(2. * pitch));
    CHECK(result.layers[0].bands[1].y1 == Approx(14. * pitch));
    CHECK_FALSE(result.layers[0].bands[0].enter_right);
    CHECK(result.layers[0].bands[1].enter_right);
    CHECK(result.layers[1].used == Approx(62. * pitch));
    CHECK(result.layers[0].depth == Approx(62. * pitch));
    CHECK(result.layers[1].depth == Approx(62. * pitch));
    CHECK(result.layers[2].depth == Approx(2. * pitch));
    CHECK(result.cone_base_radius == 0.);
    for (size_t i = 1; i < result.layers.size(); ++i)
        CHECK(result.layers[i - 1].depth >= result.layers[i].depth);
}

// Select sparse-layer candidates with look-ahead, checking the first/request layers and accumulated heights against the common tool limit.
TEST_CASE("Tower layout skips sparse layers only within the next printed height limit", "[plugins][wipe-tower][layout]")
{
    const std::vector<LayoutLayerInput> input{
        {0.1, 0.1, 0.3, {}}, {0.2, 0.1, 0.3, {}}, {0.3, 0.1, 0.3, {}}, {0.4, 0.1, 0.3, {{0, 2., 0.5}}},
        {0.5, 0.1, 0.3, {}}, {0.6, 0.1, 0.3, {}}, {0.7, 0.1, 0.3, {{1, 2., 0.5}}}
    };
    const TowerLayout result = layout_tower(10., 1., true, 0., input);
    REQUIRE(result.layers.size() == 3);
    CHECK(result.layers[0].source_index == 0);
    CHECK(result.layers[1].source_index == 3);
    CHECK(result.layers[1].height == Approx(0.3));
    CHECK(result.layers[2].source_index == 6);
    CHECK(result.layers[2].height == Approx(0.3));
    std::vector<LayoutLayerInput> smaller = input;
    smaller[2].max_height = 0.2;
    const TowerLayout bounded = layout_tower(10., 1., true, 0., smaller);
    REQUIRE(bounded.layers.size() == 4);
    CHECK(bounded.layers[1].source_index == 2);
    CHECK(bounded.layers[1].height == Approx(0.2));
    CHECK(layout_tower(10., 1., false, 0., input).layers.size() == 7);
    CHECK(layout_tower(10., 1., true, 0., {{0.1, 0.1, 0.3, {}}}).layers.empty());
}

// Apply the cone for any positive angle, including on a tower whose depth is far below the width, and keep the legacy radius/scaling formula with visible validation failures.
TEST_CASE("Tower cone applies to a thin tower for a positive angle with legacy support scaling",
          "[plugins][wipe-tower][layout]")
{
    const Slic3r::Flow flow = Slic3r::Flow::new_from_width(0.6f, 0.4f, 0.2f, 1.f);
    const double pitch = flow.spacing();
    const size_t lines = wipe_band_line_count(140., 60., flow.mm3_per_mm(), pitch);
    const double used = double(lines) * pitch;
    REQUIRE(used < 60.);
    const std::vector<LayoutLayerInput> thin{
        {0.2, 0.2, 0.4, {{0, 140., 0.6}}}, {0.4, 0.2, 0.4, {{1, 140., 0.6}}}};
    const TowerLayout cone = layout_tower(60., 1., false, 60., thin);
    REQUIRE(cone.layers.size() == 2);
    CHECK(cone.layers[0].depth == Approx(used));
    CHECK(cone.layers[1].depth == Approx(used));
    CHECK(cone.cone_base_radius == Approx(std::tan(3.14159265358979323846 / 6.) * 0.4));
    CHECK(cone.cone_slope == Approx(std::tan(3.14159265358979323846 / 6.)));
    // A zero angle keeps the cone disabled.
    CHECK(layout_tower(60., 1., false, 0., thin).cone_base_radius == 0.);
    // A floating-only layout has no regular layer to measure, so it produces no cone.
    const TowerLayout floating = layout_tower(60., 1., false, 60., {{0.8, 0.2, 0.4, {{0, 2., 0.6, 0.4, true}}}});
    REQUIRE(floating.layers.size() == 1);
    CHECK(floating.layers[0].floating);
    CHECK(floating.cone_base_radius == 0.);
    // The legacy formula scales a tall thin tower horizontally so the cone spans the width.
    double radius = 0.;
    double support_scale = 0.;
    wipe_tower_cone_base(60., 60., used, 60., radius, support_scale);
    CHECK(radius == Approx(std::tan(3.14159265358979323846 / 6.) * 60.));
    CHECK(support_scale > 1.);
    CHECK_THROWS_AS(layout_tower(0., 1., false, 0., {}), std::invalid_argument);
    CHECK_THROWS_AS(layout_tower(60., 1., false, 0., {{0., 0.2, 0.3, {{0, 1., 0.6}}}}), std::invalid_argument);
    CHECK_THROWS_AS(layout_tower(60., 1., false, 0., {{0.2, 0.2, 0.3, {{0, -1., 0.6}}}}), std::invalid_argument);
}

// Assign close, equal-Z and lower-Z borrowed source groups with whole-line volumes: a close group
// always fuses even past the largest used depth, an equal-Z group fuses while it fits, a lower-Z
// group stacks once it no longer fits; owned results preserve request order and non-increasing
// tower depths.
TEST_CASE("Tower layout fuses close groups without a bound and bounds equal or lower groups",
          "[plugins][wipe-tower][layout]")
{
    const double pitch = Slic3r::Flow::new_from_width(0.5f, 0.4f, 0.2f, 1.f).spacing();
    const TowerLayout result = layout_tower(10., 1., false, 0., {
        {0.2, 0.2, 0.3, {{0, 24., 0.5}}},
        {0.3, 0.2, 0.3, {{1, 2., 0.5}}},
        {0.4, 0.2, 0.3, {{2, 2., 0.5}}},
        {0.4, 0.2, 0.3, {{3, 2., 0.5}}},
        {0.2, 0.2, 0.3, {{4, 30., 0.5}}},
        {0.8, 0.2, 0.3, {}}
    });
    REQUIRE(result.layers.size() == 3);
    CHECK(result.layers[0].print_z == Approx(0.2));
    REQUIRE(result.layers[0].bands.size() == 2);
    CHECK(result.layers[0].bands[0].request_index == 0);
    CHECK(result.layers[0].bands[1].request_index == 1);
    CHECK(result.layers[0].bands[1].y0 == Approx(26. * pitch));
    // The close group fuses although 29 pitches exceed the 26 pitches used so far.
    CHECK(result.layers[0].used == Approx(29. * pitch));
    CHECK(result.layers[1].print_z == Approx(0.4));
    CHECK(result.layers[1].height == Approx(0.2));
    REQUIRE(result.layers[1].bands.size() == 2);
    CHECK(result.layers[1].bands[0].request_index == 2);
    CHECK(result.layers[1].bands[1].request_index == 3);
    CHECK(result.layers[1].bands[1].y0 == Approx(3. * pitch));
    // The equal-Z group fuses while 6 pitches fit the 29 pitches used so far.
    CHECK(result.layers[1].used == Approx(6. * pitch));
    CHECK(result.layers[2].print_z == Approx(0.6));
    REQUIRE(result.layers[2].bands.size() == 1);
    CHECK(result.layers[2].bands[0].request_index == 4);
    // The lower-Z group stacks once 38 pitches no longer fit.
    CHECK(result.layers[2].used == Approx(32. * pitch));
    for (size_t i = 1; i < result.layers.size(); ++i) {
        CHECK(result.layers[i].print_z > result.layers[i - 1].print_z);
        CHECK(result.layers[i - 1].depth >= result.layers[i].depth);
    }
    // The suffix maximum carries the deepest layer down to every layer below it.
    CHECK(result.layers[0].depth == Approx(32. * pitch));
}

// Interleave 0.2 mm and 0.3 mm sources: every close group fuses, so no band is printed above its
// source Z, the tower top never passes the source Z, and only non-close source layers start a
// tower layer.
TEST_CASE("Tower layout fuses interleaved close groups without advancing the tower top",
          "[plugins][wipe-tower][layout]")
{
    std::vector<double> coarse;
    std::vector<double> fine;
    for (double z = 0.2; z < 3.0001; z += 0.2)
        fine.push_back(z);
    for (double z = 0.3; z < 3.0001; z += 0.3)
        coarse.push_back(z);
    // Merge both sources in print order, the finer layer first on equal Z.
    std::vector<LayoutLayerInput> input;
    size_t fi = 0;
    size_t ci = 0;
    while (fi < fine.size() || ci < coarse.size()) {
        const bool take_fine = ci == coarse.size() || (fi < fine.size() && fine[fi] <= coarse[ci] + 1e-9);
        const double z = take_fine ? fine[fi++] : coarse[ci++];
        const double height = take_fine ? 0.2 : 0.3;
        input.push_back({z, height, 0.4, {{input.size(), 2., 0.5}}});
    }
    REQUIRE(input.size() == 25);
    const TowerLayout result = layout_tower(10., 1., false, 0., input);
    // Only the fifteen 0.2 mm layers start a tower layer; every 0.3 mm layer fuses.
    REQUIRE(result.layers.size() == fine.size());
    size_t placed = 0;
    for (size_t i = 0; i < result.layers.size(); ++i) {
        const LayoutLayer &layer = result.layers[i];
        CHECK(layer.print_z == Approx(input[layer.source_index].print_z));
        CHECK(layer.print_z == Approx(fine[i]));
        if (i != 0)
            CHECK(layer.print_z > result.layers[i - 1].print_z);
        CHECK(layer.bands.size() <= 2);
        for (const LayoutBand &band : layer.bands) {
            // A band is never printed above the source layer that asked for it, so the tower
            // top stays at or below every source Z reached so far.
            CHECK(layer.print_z <= input[band.request_index].print_z + 1e-9);
            ++placed;
        }
    }
    CHECK(placed == input.size());
    CHECK(result.layers[0].bands.size() == 2);
    // Mirror the close test to count the non-close source layers; the layout may not create more
    // tower layers than those.
    size_t non_close = 0;
    double expected_top = 0.;
    for (const LayoutLayerInput &layer : input) {
        const bool close = expected_top != 0. &&
            layer.print_z > expected_top && layer.print_z - layer.height < expected_top - 1e-9;
        if (!close) {
            ++non_close;
            expected_top = std::max(expected_top, layer.print_z);
        }
    }
    CHECK(result.layers.size() <= non_close);
}

// The user-visible requirement: in the tower layer with the largest used depth, at most 10 % of
// that depth is sparse. Realistic purge volumes at width 60, 0.6 mm lines, 0.2 mm layers and
// 100 % extra spacing, with one and two tool changes per layer.
TEST_CASE("Tower layout keeps the sparse share of the most used layer below ten percent",
          "[plugins][wipe-tower][layout][sparse-share]")
{
    const double width = 60.;
    const double line_width = 0.6;
    const double layer_height = 0.2;
    const double purge_volume = 140.;
    const std::vector<LayoutLayerInput> input{
        {0.2, layer_height, 0.4, {{0, purge_volume, line_width}}},
        {0.4, layer_height, 0.4, {{1, purge_volume, line_width}, {2, purge_volume, line_width}}},
        {0.6, layer_height, 0.4, {{3, purge_volume, line_width}}},
        {0.8, layer_height, 0.4, {{4, purge_volume, line_width}, {5, purge_volume, line_width}}}
    };
    const TowerLayout result = layout_tower(width, 1., false, 0., input);
    REQUIRE_FALSE(result.layers.empty());
    const LayoutLayer *most_used = nullptr;
    for (const LayoutLayer &layer : result.layers)
        if (most_used == nullptr || layer.used > most_used->used)
            most_used = &layer;
    REQUIRE(most_used != nullptr);
    REQUIRE(most_used->depth > 0.);
    CAPTURE(most_used->used, most_used->depth, most_used->bands.size());
    CHECK((most_used->depth - most_used->used) / most_used->depth <= 0.1);

    // A single change per layer has the same requirement.
    const std::vector<LayoutLayerInput> single{
        {0.2, layer_height, 0.4, {{0, purge_volume, line_width}}},
        {0.4, layer_height, 0.4, {{1, purge_volume, line_width}}},
        {0.6, layer_height, 0.4, {{2, purge_volume, line_width}}}
    };
    const TowerLayout single_result = layout_tower(width, 1., false, 0., single);
    REQUIRE_FALSE(single_result.layers.empty());
    const LayoutLayer *single_most_used = nullptr;
    for (const LayoutLayer &layer : single_result.layers)
        if (single_most_used == nullptr || layer.used > single_most_used->used)
            single_most_used = &layer;
    REQUIRE(single_most_used != nullptr);
    REQUIRE(single_most_used->depth > 0.);
    CAPTURE(single_most_used->used, single_most_used->depth, single_most_used->bands.size());
    CHECK((single_most_used->depth - single_most_used->used) / single_most_used->depth <= 0.1);
}

// Lay out a floating band whose source Z reaches the regular tower top: it goes through the same
// fusion or stacking rule as a normal request.
TEST_CASE("Tower layout lays out a floating band that reaches the tower top", "[plugins][wipe-tower][layout]")
{
    const std::vector<LayoutLayerInput> input{
        {0.2, 0.2, 0.4, {{0, 2., 0.5}}},
        {0.4, 0.2, 0.4, {{1, 2., 0.5}}},
        {0.4, 0.2, 0.4, {{2, 2., 0.5, 0.4, true}}}
    };
    const TowerLayout result = layout_tower(10., 1., false, 0., input);
    REQUIRE(result.layers.size() == 3);
    for (const LayoutLayer &layer : result.layers)
        CHECK_FALSE(layer.floating);
    REQUIRE(result.layers[2].bands.size() == 1);
    CHECK(result.layers[2].bands[0].request_index == 2);
    CHECK(result.layers[2].bands[0].y0 == 0.);
    CHECK(result.layers[2].print_z == Approx(0.6));
    // The same bands without the floating flag give the identical layout.
    const TowerLayout regular = layout_tower(10., 1., false, 0., {
        {0.2, 0.2, 0.4, {{0, 2., 0.5}}},
        {0.4, 0.2, 0.4, {{1, 2., 0.5}}},
        {0.4, 0.2, 0.4, {{2, 2., 0.5}}}
    });
    REQUIRE(regular.layers.size() == result.layers.size());
    for (size_t i = 0; i < result.layers.size(); ++i) {
        CHECK(regular.layers[i].print_z == Approx(result.layers[i].print_z));
        CHECK(regular.layers[i].used == Approx(result.layers[i].used));
        CHECK(regular.layers[i].depth == Approx(result.layers[i].depth));
        REQUIRE(regular.layers[i].bands.size() == result.layers[i].bands.size());
    }
}

// Float a band above the regular tower top: it gets its own layer at its source Z and changes
// neither suffix depth nor cone, while the regular layers stay identical.
TEST_CASE("Tower layout floats a band above the tower top without changing depth or cone", "[plugins][wipe-tower][layout]")
{
    const Slic3r::Flow flow = Slic3r::Flow::new_from_width(0.5f, 0.4f, 0.1f, 1.f);
    const double ratio = 0.5 / flow.spacing();
    const double deep_volume = flow.mm3_per_mm() * (40. * 10. + 39. * 0.5);
    const std::vector<LayoutLayerInput> input{
        {0.1, 0.1, 0.3, {{0, deep_volume + flow.mm3_per_mm(), 0.5}}},
        {0.2, 0.1, 0.3, {{1, 2., 0.5}}},
        {0.8, 0.3, 0.3, {{2, 2., 0.5, 0.4, true}}}
    };
    const TowerLayout reference = layout_tower(10., ratio, false, 60., {input[0], input[1]});
    REQUIRE(reference.layers.size() == 2);
    REQUIRE(reference.cone_base_radius > 0.);
    const TowerLayout result = layout_tower(10., ratio, false, 60., input);
    REQUIRE(result.layers.size() == 3);
    // The regular layers and the cone are identical to the layout without the floating band.
    for (size_t i = 0; i < reference.layers.size(); ++i) {
        CHECK_FALSE(result.layers[i].floating);
        CHECK(result.layers[i].source_index == reference.layers[i].source_index);
        CHECK(result.layers[i].print_z == Approx(reference.layers[i].print_z));
        CHECK(result.layers[i].height == Approx(reference.layers[i].height));
        CHECK(result.layers[i].used == Approx(reference.layers[i].used));
        CHECK(result.layers[i].depth == Approx(reference.layers[i].depth));
        REQUIRE(result.layers[i].bands.size() == reference.layers[i].bands.size());
        for (size_t b = 0; b < reference.layers[i].bands.size(); ++b)
            CHECK(result.layers[i].bands[b].y1 == Approx(reference.layers[i].bands[b].y1));
    }
    CHECK(result.cone_base_radius == Approx(reference.cone_base_radius));
    CHECK(result.cone_slope == Approx(reference.cone_slope));
    // The floating layer sits at its source Z, starts a fresh band row and carries its own used depth.
    CHECK(result.layers[2].floating);
    CHECK(result.layers[2].print_z == Approx(0.8));
    CHECK(result.layers[2].height == Approx(0.3));
    CHECK(result.layers[2].depth == Approx(result.layers[2].used));
    REQUIRE(result.layers[2].bands.size() == 1);
    CHECK(result.layers[2].bands[0].request_index == 2);
    CHECK(result.layers[2].bands[0].y0 == 0.);
    const Slic3r::Flow floating_flow = Slic3r::Flow::new_from_width(0.5f, 0.4f, 0.3f, 1.f);
    const double floating_pitch = floating_flow.spacing() * ratio;
    CHECK(result.layers[2].bands[0].y1 ==
          Approx(double(wipe_band_line_count(2., 10., floating_flow.mm3_per_mm(), floating_pitch)) * floating_pitch));
}
