#include "libslic3r/HybridNonplanarScheduler.hpp"

#include <catch2/catch.hpp>
#include <cmath>
#include <limits>
#include <vector>

using Slic3r::HybridNonplanarScheduler;

TEST_CASE("Hybrid non-planar scheduler uses a configurable interval", "[HybridNonplanarScheduler]")
{
    HybridNonplanarScheduler scheduler;
    REQUIRE(scheduler.valid());
    REQUIRE(scheduler.interval() == Approx(2.0));
    REQUIRE(scheduler.next_boundary() == Approx(2.0));

    REQUIRE(scheduler.after_planar_layer(0.0, 0.2).empty());
    REQUIRE(scheduler.after_planar_layer(0.2, 1.8).empty());

    const auto due_at_two = scheduler.after_planar_layer(1.8, 2.0);
    REQUIRE(due_at_two.size() == 1);
    REQUIRE(due_at_two[0] == Approx(2.0));
    REQUIRE(scheduler.next_boundary() == Approx(4.0));

    // Reprocessing the same layer must not schedule the same pass twice.
    REQUIRE(scheduler.after_planar_layer(1.8, 2.0).empty());
    REQUIRE(scheduler.after_planar_layer(2.0, 3.9).empty());

    const auto due_at_four = scheduler.after_planar_layer(3.9, 4.01);
    REQUIRE(due_at_four.size() == 1);
    REQUIRE(due_at_four[0] == Approx(4.0));
}

TEST_CASE("Hybrid non-planar scheduler handles variable-height layers", "[HybridNonplanarScheduler]")
{
    HybridNonplanarScheduler scheduler(1.25);

    REQUIRE(scheduler.after_planar_layer(0.0, 0.4).empty());
    const auto due = scheduler.after_planar_layer(0.4, 3.2);
    REQUIRE(due.size() == 2);
    REQUIRE(due[0] == Approx(1.25));
    REQUIRE(due[1] == Approx(2.5));
    REQUIRE(scheduler.next_boundary() == Approx(3.75));
}

TEST_CASE("Hybrid non-planar scheduler accepts a small boundary tolerance", "[HybridNonplanarScheduler]")
{
    HybridNonplanarScheduler scheduler(2.0, 1e-4);

    const auto due = scheduler.after_planar_layer(1.5, 1.99995);
    REQUIRE(due.size() == 1);
    REQUIRE(due[0] == Approx(2.0));
}

TEST_CASE("Hybrid non-planar scheduler rejects invalid intervals and layer ranges", "[HybridNonplanarScheduler]")
{
    HybridNonplanarScheduler zero_interval(0.0);
    REQUIRE_FALSE(zero_interval.valid());
    REQUIRE(zero_interval.after_planar_layer(0.0, 10.0).empty());

    HybridNonplanarScheduler negative_interval(-2.0);
    REQUIRE_FALSE(negative_interval.valid());

    HybridNonplanarScheduler non_finite_interval(std::numeric_limits<double>::infinity());
    REQUIRE_FALSE(non_finite_interval.valid());

    HybridNonplanarScheduler scheduler;
    REQUIRE(scheduler.after_planar_layer(1.0, 0.5).empty());
    REQUIRE(scheduler.after_planar_layer(0.0, std::numeric_limits<double>::quiet_NaN()).empty());
    REQUIRE(scheduler.next_boundary() == Approx(2.0));

    REQUIRE(scheduler.after_planar_layer(0.0, 2.0).size() == 1);
    // Out-of-order layers are ignored and cannot move the schedule backwards.
    REQUIRE(scheduler.after_planar_layer(0.0, 1.0).empty());
    REQUIRE(scheduler.next_boundary() == Approx(4.0));
}

TEST_CASE("Hybrid non-planar scheduler reset restarts the interval sequence", "[HybridNonplanarScheduler]")
{
    HybridNonplanarScheduler scheduler(2.0);
    REQUIRE(scheduler.after_planar_layer(0.0, 2.0).size() == 1);

    scheduler.reset(0.75);
    REQUIRE(scheduler.valid());
    REQUIRE(scheduler.next_boundary() == Approx(0.75));

    const auto due = scheduler.after_planar_layer(0.0, 1.6);
    REQUIRE(due.size() == 2);
    REQUIRE(due[0] == Approx(0.75));
    REQUIRE(due[1] == Approx(1.5));
}
