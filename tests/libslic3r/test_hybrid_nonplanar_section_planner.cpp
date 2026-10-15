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

TEST_CASE("Hybrid path constraints enforce endpoint ramps, offset and slope limits", "[HybridNonplanarSectionPlanner]")
{
    std::vector<double> distance { 0.0, 0.5, 1.0, 1.5, 2.0 };
    std::vector<double> offset { 0.0, 0.8, 0.8, 0.8, 0.0 };

    REQUIRE(Slic3r::constrain_hybrid_nonplanar_offsets(distance, offset, 0.4, 30.0));
    REQUIRE(offset.front() == Approx(0.0));
    REQUIRE(offset.back() == Approx(0.0));
    for (std::size_t i = 0; i < offset.size(); ++i) {
        REQUIRE(std::abs(offset[i]) <= 0.4 + 1e-8);
        if (i > 0)
            REQUIRE(std::abs(offset[i] - offset[i - 1]) <= std::tan(30.0 * 3.14159265358979323846 / 180.0) * (distance[i] - distance[i - 1]) + 1e-7);
    }
}

TEST_CASE("Hybrid path constraints reject malformed samples without mutating them", "[HybridNonplanarSectionPlanner]")
{
    std::vector<double> distance { 0.0, 1.0, 1.0 };
    std::vector<double> offset { 0.0, 0.2, 0.0 };
    const auto original = offset;
    REQUIRE_FALSE(Slic3r::constrain_hybrid_nonplanar_offsets(distance, offset, 0.4, 30.0));
    REQUIRE(offset == original);

    distance = { 0.0, std::numeric_limits<double>::quiet_NaN(), 2.0 };
    REQUIRE_FALSE(Slic3r::constrain_hybrid_nonplanar_offsets(distance, offset, 0.4, 30.0));
    REQUIRE(offset == original);
}
