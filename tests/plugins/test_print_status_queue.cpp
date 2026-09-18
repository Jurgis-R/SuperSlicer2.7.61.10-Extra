#include <catch2/catch.hpp>

#include <algorithm>
#include <atomic>
#include <map>
#include <stdexcept>
#include <thread>
#include <vector>

#include "libslic3r/Print.hpp"
#include "plugin_test_helpers.hpp"

using namespace Slic3r;

TEST_CASE("Print GUI status queue coalesces progress", "[plugins][status][gui]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    std::atomic_size_t wakeups { 0 };
    print.status().set_gui_wakeup_callback([&wakeups]() { ++wakeups; });
    const uint64_t generation = print.status().reset_pending();

    for (int percent = 0; percent < 1000; ++percent)
        print.status().set(percent, "Visibility", PrintBase::SlicingStatus::SECONDARY_STATE);

    CHECK(wakeups.load() == 1);
    PrintBase::PendingStatusBatch batch = print.status().take_pending(generation, true);
    REQUIRE(batch.secondary.has_value());
    CHECK(batch.secondary->percent == 999);
    CHECK(batch.secondary->main_text == "Visibility");
    CHECK_FALSE(batch.main.has_value());
    CHECK(batch.control_events.empty());

    print.status().set(12, "Ordering extrusions", PrintBase::SlicingStatus::MAIN_STATE);
    print.status().set_percent(37, PrintStatus::Channel::Main);
    print.status().set(48, "Ray tracing", PrintBase::SlicingStatus::SECONDARY_STATE);
    CHECK(wakeups.load() == 2);

    batch = print.status().take_pending(generation, true);
    REQUIRE(batch.main.has_value());
    REQUIRE(batch.secondary.has_value());
    CHECK(batch.main->percent == 37);
    CHECK(batch.main->main_text == "Ordering extrusions");
    CHECK(batch.secondary->percent == 48);
    CHECK(batch.secondary->main_text == "Ray tracing");

    const uint64_t percent_only_generation = print.status().reset_pending();
    print.status().set_percent(61, PrintStatus::Channel::Main);
    batch = print.status().take_pending(percent_only_generation, true);
    REQUIRE(batch.main.has_value());
    CHECK(batch.main->percent == 61);
    CHECK(batch.main->main_text.empty());

    const uint64_t dual_channel_generation = print.status().reset_pending();
    print.status().set(70, "both", PrintBase::SlicingStatus::MAIN_STATE |
                                 PrintBase::SlicingStatus::SECONDARY_STATE);
    batch = print.status().take_pending(dual_channel_generation, true);
    REQUIRE(batch.main.has_value());
    REQUIRE(batch.secondary.has_value());
    CHECK((batch.main->flags & PrintBase::SlicingStatus::MAIN_STATE) != 0);
    CHECK((batch.main->flags & PrintBase::SlicingStatus::SECONDARY_STATE) == 0);
    CHECK((batch.secondary->flags & PrintBase::SlicingStatus::MAIN_STATE) == 0);
    CHECK((batch.secondary->flags & PrintBase::SlicingStatus::SECONDARY_STATE) != 0);
}

TEST_CASE("Print GUI status queue preserves control events", "[plugins][status][gui]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    size_t wakeups = 0;
    print.status().set_gui_wakeup_callback([&wakeups]() { ++wakeups; });
    const uint64_t generation = print.status().reset_pending();

    print.status().set(10, "warning", PrintBase::SlicingStatus::UPDATE_PRINT_STEP_WARNINGS);
    print.status().set(20, "reload", PrintBase::SlicingStatus::RELOAD_SCENE);
    print.status().set(30, "forced", PrintBase::SlicingStatus::FORCE_SHOW);
    print.status().set(40, "slicing ended", PrintBase::SlicingStatus::SLICING_ENDED);
    print.status().set(50, "gcode ended", PrintBase::SlicingStatus::GCODE_ENDED);
    CHECK(wakeups == 1);

    PrintBase::PendingStatusBatch batch = print.status().take_pending(generation, false);
    CHECK(batch.control_events.empty());

    print.status().request_pending_delivery();
    CHECK(wakeups == 2);
    batch = print.status().take_pending(generation, true);
    REQUIRE(batch.control_events.size() == 5);
    CHECK(batch.control_events[0].main_text == "warning");
    CHECK(batch.control_events[1].main_text == "reload");
    CHECK(batch.control_events[2].main_text == "forced");
    CHECK(batch.control_events[3].main_text == "slicing ended");
    CHECK(batch.control_events[4].main_text == "gcode ended");
}

TEST_CASE("Print status generations and instances are independent", "[plugins][status][gui]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print first;
    Print second;
    size_t first_wakeups = 0;
    size_t second_wakeups = 0;
    first.status().set_gui_wakeup_callback([&first_wakeups]() { ++first_wakeups; });
    second.status().set_gui_wakeup_callback([&second_wakeups]() { ++second_wakeups; });
    const uint64_t old_generation = first.status().reset_pending();
    const uint64_t second_generation = second.status().reset_pending();

    first.status().set(10, "first");
    second.status().set(20, "second");
    CHECK(first_wakeups == 1);
    CHECK(second_wakeups == 1);

    const uint64_t new_generation = first.status().reset_pending();
    CHECK(new_generation != old_generation);
    CHECK(first.status().take_pending(old_generation, true).empty());
    REQUIRE(second.status().take_pending(second_generation, true).main.has_value());

    first.status().set(30, "new");
    PrintBase::PendingStatusBatch batch = first.status().take_pending(new_generation, true);
    REQUIRE(batch.main.has_value());
    CHECK(batch.main->main_text == "new");
}

TEST_CASE("Print GUI status queue keeps updates racing with consumption", "[plugins][status][gui]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    std::atomic_size_t wakeups { 0 };
    print.status().set_gui_wakeup_callback([&wakeups]() { ++wakeups; });
    const uint64_t generation = print.status().reset_pending();

    constexpr int last_percent = 20000;
    std::atomic_bool start { false };
    std::atomic_bool finished { false };
    std::thread producer([&] {
        while (!start.load(std::memory_order_acquire))
            std::this_thread::yield();
        for (int percent = 0; percent <= last_percent; ++percent)
            print.status().set_percent(percent, PrintStatus::Channel::Main);
        finished.store(true, std::memory_order_release);
    });

    int latest_seen = -1;
    start.store(true, std::memory_order_release);
    while (!finished.load(std::memory_order_acquire)) {
        const PrintBase::PendingStatusBatch batch =
            print.status().take_pending(generation, true);
        if (batch.main.has_value())
            latest_seen = std::max(latest_seen, batch.main->percent);
    }
    producer.join();
    const PrintBase::PendingStatusBatch final_batch =
        print.status().take_pending(generation, true);
    if (final_batch.main.has_value())
        latest_seen = std::max(latest_seen, final_batch.main->percent);

    CHECK(latest_seen == last_percent);
    CHECK(wakeups.load() > 0);
}

TEST_CASE("Print direct status callbacks remain immediate", "[plugins][status][direct]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    std::vector<PrintBase::SlicingStatus> statuses;
    print.status().set_callback([&statuses](const PrintBase::SlicingStatus &status) {
        statuses.push_back(status);
    });

    print.status().set(1, "one");
    print.status().set(2, "two", PrintBase::SlicingStatus::SECONDARY_STATE);
    print.status().set_percent(3, PrintStatus::Channel::Main);

    REQUIRE(statuses.size() == 3);
    CHECK(statuses[0].main_text == "one");
    CHECK(statuses[1].main_text == "two");
    CHECK(statuses[2].percent == 3);

    print.status().set_silent();
    print.status().set(4, "silent");
    CHECK(statuses.size() == 3);
    CHECK(print.status().take_pending(print.status().generation(), true).empty());
}

TEST_CASE("Print pipeline progress counts successful top-level units", "[plugins][status][pipeline]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    std::vector<PrintBase::SlicingStatus> statuses;
    print.status().set_callback([&statuses](const PrintBase::SlicingStatus &status) {
        statuses.push_back(status);
    });

    const std::map<slicing_step_t, std::vector<std::string>> plan {
        { STEP_ORDERING, { "ordering.first", "ordering.second" } },
        { STEP_WIPETOWER, { std::string() } }
    };
    print.pipeline_progress().initialize(plan);

    print.pipeline_progress().begin_step(STEP_ORDERING, "Ordering extrusions");
    print.pipeline_progress().complete_plugin(STEP_ORDERING, "ordering.first");
    // Ordering rebuilds its plan and marks itself plus downstream steps for
    // execution. That internal invalidation must not erase the active global
    // progress plan.
    print.mark_step_and_dependents_for_execution(STEP_ORDERING);
    CHECK(print.pipeline_progress().active());
    CHECK_THROWS_AS(
        print.pipeline_progress().complete_plugin(STEP_ORDERING, "ordering.first"),
        std::logic_error);
    print.pipeline_progress().complete_plugin(STEP_ORDERING, "ordering.second");
    print.pipeline_progress().verify_step_complete(STEP_ORDERING);

    print.pipeline_progress().begin_step(STEP_WIPETOWER, "Generating wipe tower");
    print.pipeline_progress().complete_step(STEP_WIPETOWER);
    print.pipeline_progress().verify_step_complete(STEP_WIPETOWER);

    std::vector<int> main_percentages;
    for (const PrintBase::SlicingStatus &status : statuses)
        if ((status.flags & PrintBase::SlicingStatus::MAIN_STATE) != 0)
            main_percentages.push_back(status.percent);
    REQUIRE(main_percentages.size() >= 5);
    CHECK(std::is_sorted(main_percentages.begin(), main_percentages.end()));
    CHECK(main_percentages.back() == 100);
}

TEST_CASE("Print pipeline progress does not complete failed work", "[plugins][status][pipeline]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    print.status().set_silent();
    const std::map<slicing_step_t, std::vector<std::string>> plan {
        { STEP_ORDERING, { "ordering.first", "ordering.second" } }
    };
    print.pipeline_progress().initialize(plan);
    print.pipeline_progress().begin_step(STEP_ORDERING, "Ordering extrusions");
    print.pipeline_progress().complete_plugin(STEP_ORDERING, "ordering.first");

    // The runner reaches neither the second completion nor the step finalizer
    // after a plugin exception or cooperative cancellation.
    CHECK(print.pipeline_progress().percent() == 50);
    CHECK_THROWS_AS(print.pipeline_progress().verify_step_complete(STEP_ORDERING), std::logic_error);
}
