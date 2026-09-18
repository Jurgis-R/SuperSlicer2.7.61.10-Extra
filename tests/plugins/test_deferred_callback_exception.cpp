///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include <stdexcept>
#include <string>

#include <catch2/catch.hpp>

#include "libslic3r/Api/plugin/cpp/DeferredCallbackException.hpp"

using slic3r_api::DeferredCallbackException;

TEST_CASE("Deferred callback exceptions preserve the primary foreign-callback failure",
          "[plugins][callback-boundary]")
{
    DeferredCallbackException deferred;
    CHECK_FALSE(deferred.captured());
    CHECK_FALSE(deferred.capture_current());

    try {
        throw std::runtime_error("primary callback failure");
    } catch (...) {
        CHECK(deferred.capture_current());
    }

    try {
        throw std::logic_error("secondary cleanup failure");
    } catch (...) {
        CHECK_FALSE(deferred.capture_current());
    }

    REQUIRE(deferred.captured());
    try {
        deferred.rethrow_if_captured();
        FAIL("The retained callback exception was not rethrown.");
    } catch (const std::runtime_error &error) {
        CHECK(std::string(error.what()) == "primary callback failure");
    }
}
