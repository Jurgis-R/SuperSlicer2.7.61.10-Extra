#include "libslic3r/HybridNonplanarSectionPlanner.hpp"

#include <catch2/catch.hpp>
#include <limits>

using Slic3r::HybridNonplanarSectionPlanner;

TEST_CASE("Hybrid section planner closes sections only at planar layer boundaries", "[HybridNonplanarSectionPlanner]")
{
    HybridNonplanarSectionPlanner planner(2.0);
    REQUIRE(planner.valid());
    REQUIRE(planner.next_boundary_z() == Approx(2.0));

    REQUIRE_FALSE(planner.after_planar_layer(0.0, 0.2).has_value());
    REQUIRE_FALSE(planner.after_planar_layer(0.2, 1.8).has_value());

    const auto section = planner.after_planar_layer(1.8, 2.0);
    REQUIRE(section.has_value());
    REQUIRE(section->start_z == Approx(0.0));
    REQUIRE(section->boundary_z == Approx(2.0));
    REQUIRE(section->boundary_layer_top_z == Approx(2.0));
    REQUIRE(section->planar_layer_count == 3);
    REQUIRE(planner.section_start_z() == Approx(2.0));
    REQUIRE(planner.next_boundary_z() == Approx(4.0));
    REQUIRE(planner.current_section_layer_count() == 0);
}

TEST_CASE("Hybrid section planner handles a boundary crossed by a variable-height layer", "[HybridNonplanarSectionPlanner]")
{
    HybridNonplanarSectionPlanner planner(2.0);
    REQUIRE_FALSE(planner.after_planar_layer(0.0, 0.4).has_value());
    const auto section = planner.after_planar_layer(0.4, 2.1);
    REQUIRE(section.has_value());
    REQUIRE(section->boundary_z == Approx(2.0));
    REQUIRE(section->boundary_layer_top_z == Approx(2.1));
    REQUIRE(section->planar_layer_count == 2);
}

TEST_CASE("Hybrid section planner fails closed for invalid or ambiguous layer groups", "[HybridNonplanarSectionPlanner]")
{
    HybridNonplanarSectionPlanner planner(2.0);
    REQUIRE_FALSE(planner.after_planar_layer(0.0, 0.2).has_value());
    REQUIRE_FALSE(planner.after_planar_layer(0.0, 0.1).has_value());
    REQUIRE_FALSE(planner.valid());

    planner.reset(2.0);
    REQUIRE_FALSE(planner.after_planar_layer(0.0, 4.2).has_value());
    REQUIRE_FALSE(planner.valid());

    planner.reset(0.0);
    REQUIRE_FALSE(planner.valid());

    planner.reset(2.0);
    REQUIRE_FALSE(planner.after_planar_layer(0.0, std::numeric_limits<double>::quiet_NaN()).has_value());
    REQUIRE_FALSE(planner.valid());
}

TEST_CASE("Hybrid section planner reset starts a fresh section sequence", "[HybridNonplanarSectionPlanner]")
{
    HybridNonplanarSectionPlanner planner(2.0);
    REQUIRE_FALSE(planner.after_planar_layer(0.0, 2.0).has_value());
    REQUIRE(planner.next_boundary_z() == Approx(4.0));

    planner.reset(0.75);
    REQUIRE(planner.valid());
    REQUIRE(planner.next_boundary_z() == Approx(0.75));
    const auto section = planner.after_planar_layer(0.0, 0.8);
    REQUIRE(section.has_value());
    REQUIRE(section->boundary_z == Approx(0.75));
}
