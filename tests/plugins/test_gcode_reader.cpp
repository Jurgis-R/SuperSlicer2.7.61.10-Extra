#include <catch2/catch.hpp>

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "plugin_test_helpers.hpp"

#include "libslic3r/Api/host/GCodeReaderService.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_gcode_reader.h"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeStateTracker.hpp"

/*
G-code reader event tests
=========================

These tests exercise the plugin service rather than the legacy GCodeReader
class. Service tests use read_gcode() to verify the host-owned final state.
Tests which need individual actions invoke the built-in provider through its
real run payload and capture the same callbacks that the host implements.
*/

namespace {

using namespace Slic3r;
using Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized;

struct ObservedEvent
{
    uint32_t kind = 0;
    uint32_t line_number = UINT32_MAX;
    raw_gcode_reader_source source = {};
};

struct ReaderProbe
{
    std::vector<std::string_view> chunks;
    size_t next_chunk = 0;
    size_t cancel_after_chunks = 0;
    std::vector<ObservedEvent> observed;
    std::optional<slic3r_api::GCodeStateTracker> final_state;
};

// Supply stable borrowed views in sequence. An empty output marks the end of
// input exactly as the production service does.
void supply_chunk(void *context,
                  raw_gcode_reader_chunk *chunk,
                  raw_gcode_reader_result *result);
// Copy the small pieces of each borrowed event needed by assertions before
// returning control to the provider.
void collect_event(void *context,
                   const raw_gcode_reader_event *event,
                   raw_gcode_reader_result *result);
// Copy the borrowed final state into a tracker owned by the test probe.
void collect_final_state(void *context,
                         const raw_gcode_reader_state *state,
                         raw_gcode_reader_result *result);
// Execute the registered provider through Orchestrator so the common plugin
// cancellation and exception boundaries remain part of each test.
raw_plugin_execution_status execute_default_reader(
    ReaderProbe &probe, const raw_gcode_reader_state &initial);
raw_gcode_reader_state initial_state(std::vector<raw_gcode_reader_event_item> &items);

void supply_chunk(void *context,
                  raw_gcode_reader_chunk *chunk,
                  raw_gcode_reader_result *result)
{
    ReaderProbe *probe = static_cast<ReaderProbe *>(context);
    *chunk = {};
    if (probe->next_chunk < probe->chunks.size()) {
        const std::string_view bytes = probe->chunks[probe->next_chunk++];
        chunk->bytes = bytes.data();
        chunk->byte_count = static_cast<uint64_t>(bytes.size());
        if (probe->cancel_after_chunks != 0 &&
            probe->next_chunk == probe->cancel_after_chunks)
            Orchestrator::instance().request_plugin_cancel();
    }
    result->status = RAW_GCODE_READER_STATUS_SUCCESS;
}

void collect_event(void *context,
                   const raw_gcode_reader_event *event,
                   raw_gcode_reader_result *result)
{
    ReaderProbe *probe = static_cast<ReaderProbe *>(context);
    uint32_t line_number = UINT32_MAX;
    for (uint32_t index = 0; index < event->item_count; ++index) {
        const raw_gcode_reader_event_item &item = event->items[index];
        if (item.type == RAW_GCODE_READER_ITEM_LINE_NUMBER)
            line_number = static_cast<uint32_t>(item.value);
    }
    probe->observed.push_back({event->kind, line_number, event->source});
    result->status = RAW_GCODE_READER_STATUS_SUCCESS;
}

void collect_final_state(void *context,
                         const raw_gcode_reader_state *state,
                         raw_gcode_reader_result *result)
{
    ReaderProbe *probe = static_cast<ReaderProbe *>(context);
    probe->final_state.emplace(*state);
    result->status = RAW_GCODE_READER_STATUS_SUCCESS;
}

raw_plugin_execution_status execute_default_reader(
    ReaderProbe &probe, const raw_gcode_reader_state &initial)
{
    Orchestrator &orchestrator = Orchestrator::instance();
    Plugin *plugin = orchestrator.get_plugin("gcode.reader.default");
    if (plugin == nullptr)
        throw std::runtime_error("The default G-code reader test provider is not registered.");

    run_ctx_gcode_reader context = {};
    context.initial_state = &initial;
    context.callback_context = &probe;
    context.get_next_chunk = supply_chunk;
    context.emit_event = collect_event;
    context.emit_final_state = collect_final_state;
    raw_plugin_run_payload payload = {&context};
    return orchestrator.execute_plugin(*plugin, nullptr, &payload, 1);
}

raw_gcode_reader_state initial_state(std::vector<raw_gcode_reader_event_item> &items)
{
    items = {
        {RAW_GCODE_READER_ITEM_X, 0, 1.0},
        {RAW_GCODE_READER_ITEM_Y, 0, 2.0},
        {RAW_GCODE_READER_ITEM_Z, 0, 3.0},
        {RAW_GCODE_READER_ITEM_E, 0, 0.0},
        {RAW_GCODE_READER_ITEM_UNITS_MM, 0, 1.0},
        {RAW_GCODE_READER_ITEM_XYZ_ABSOLUTE, 0, 1.0},
        {RAW_GCODE_READER_ITEM_E_ABSOLUTE, 0, 1.0},
        {RAW_GCODE_READER_ITEM_ACTIVE_TOOL, 0, 0.0},
        {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 0.0}
    };
    raw_gcode_reader_state state = {};
    state.items = items.data();
    state.item_count = static_cast<uint32_t>(items.size());
    return state;
}

TEST_CASE("G-code reader publishes semantic actions and an owned final state",
          "[plugins][gcode][reader]")
{
    ensure_plugin_test_runtime_initialized();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    ReaderProbe probe;
    probe.chunks = {
        "G91\nG1 X2 E1\nG90\nG92 X10 E0\nT1\nM104 S215 T1\nM109 S215 T1\nM106 P2 S128\n"
    };
    REQUIRE(execute_default_reader(probe, initial) == RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(probe.final_state);
    const slic3r_api::GCodeStateTracker &state = *probe.final_state;

    REQUIRE(probe.observed.size() == 8);
    CHECK(probe.observed.front().kind == RAW_GCODE_READER_EVENT_STATE_UPDATE);
    CHECK(probe.observed[1].kind == RAW_GCODE_READER_EVENT_LINEAR_MOVE);
    CHECK(probe.observed[3].kind == RAW_GCODE_READER_EVENT_SET_POSITION);
    CHECK(probe.observed[5].kind == RAW_GCODE_READER_EVENT_SET_TEMPERATURE);
    CHECK(probe.observed[6].kind == RAW_GCODE_READER_EVENT_WAIT_TEMPERATURE);
    CHECK(probe.observed[1].line_number == 1u);
    CHECK(probe.observed[1].source.begin == 4u);
    CHECK(probe.observed[1].source.end == 12u);
    REQUIRE(state.x());
    REQUIRE(state.y());
    REQUIRE(state.z());
    REQUIRE(state.e(0));
    CHECK(*state.x() == Approx(10.0));
    CHECK(*state.y() == Approx(2.0));
    CHECK(*state.z() == Approx(3.0));
    CHECK(*state.e(0) == Approx(0.0));
    CHECK(state.active_tool() == 1u);
    REQUIRE(state.tool_temperature(1));
    CHECK(*state.tool_temperature(1) == Approx(215.0));
    CHECK(state.known(RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE_REACHED, 1));
    REQUIRE(state.fan_speed(2));
    CHECK(*state.fan_speed(2) == Approx(128.0 * 100.0 / 255.0));
    CHECK(state.line_number() == 8u);
}

TEST_CASE("G-code reader invalidates stale state after an unknown command",
          "[plugins][gcode][reader]")
{
    ensure_plugin_test_runtime_initialized();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    const slic3r_api::GCodeStateTracker state = read_gcode(
        Orchestrator::instance(), nullptr, initial, "M900 K0.05\n");

    CHECK_FALSE(state.x().has_value());
    CHECK_FALSE(state.active_tool().has_value());
    CHECK(state.line_number() == 1u);
}

TEST_CASE("G-code reader keeps modal state across an M117 display message",
          "[plugins][gcode][reader]")
{
    ensure_plugin_test_runtime_initialized();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    ReaderProbe probe;
    probe.chunks = {"M117 Printing perimeter\n"};
    REQUIRE(execute_default_reader(probe, initial) == RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(probe.final_state);
    const slic3r_api::GCodeStateTracker &state = *probe.final_state;

    REQUIRE(probe.observed.size() == 1);
    CHECK(probe.observed.front().kind == RAW_GCODE_READER_EVENT_DIAGNOSTIC);
    CHECK(state.x() == 1.0);
    CHECK(state.active_tool() == 0u);
    CHECK(state.line_number() == 1u);
}

TEST_CASE("G-code reader line number follows physical newlines", "[plugins][gcode][reader]")
{
    ensure_plugin_test_runtime_initialized();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    ReaderProbe probe;
    probe.chunks = {"\n; comment\r\nG1 X1\r\nG1 X2"};
    REQUIRE(execute_default_reader(probe, initial) == RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(probe.final_state);
    const slic3r_api::GCodeStateTracker &state = *probe.final_state;

    REQUIRE(probe.observed.size() == 2);
    CHECK(probe.observed[0].line_number == 2u);
    CHECK(probe.observed[1].line_number == 3u);
    CHECK(probe.observed[0].source.begin == 12u);
    CHECK(probe.observed[0].source.end == 17u);
    CHECK(probe.observed[1].source.begin == 19u);
    CHECK(probe.observed[1].source.end == 24u);
    CHECK(state.line_number() == 3u);
}

TEST_CASE("G-code reader requires a valid line number item", "[plugins][gcode][reader]")
{
    raw_gcode_reader_event_item missing_line_items[] = {
        {RAW_GCODE_READER_ITEM_X, 0, 1.0}
    };
    raw_gcode_reader_state missing_line = {};
    missing_line.items = missing_line_items;
    missing_line.item_count = 1;
    CHECK_THROWS_AS(slic3r_api::GCodeStateTracker(missing_line), std::invalid_argument);

    raw_gcode_reader_event missing_line_event = {};
    missing_line_event.kind = RAW_GCODE_READER_EVENT_STATE_UPDATE;
    missing_line_event.source = {0, 1};
    missing_line_event.items = missing_line_items;
    missing_line_event.item_count = 1;
    CHECK_THROWS_AS(slic3r_api::GCodeStateTracker().apply(missing_line_event), std::invalid_argument);

    raw_gcode_reader_event_item invalid_line_items[] = {
        {RAW_GCODE_READER_ITEM_LINE_NUMBER, 1, 1.0}
    };
    raw_gcode_reader_state invalid_line = {};
    invalid_line.items = invalid_line_items;
    invalid_line.item_count = 1;
    CHECK_THROWS_AS(slic3r_api::GCodeStateTracker(invalid_line), std::invalid_argument);

    invalid_line_items[0] = {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 0.5};
    CHECK_THROWS_AS(slic3r_api::GCodeStateTracker(invalid_line), std::invalid_argument);
}

TEST_CASE("G-code reader tracker copies own their raw state view",
          "[plugins][gcode][reader]")
{
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    slic3r_api::GCodeStateTracker original(initial);
    slic3r_api::GCodeStateTracker copied(original);

    REQUIRE(original.state().items != nullptr);
    REQUIRE(copied.state().items != nullptr);
    CHECK(copied.state().items != original.state().items);
    original.set_line_number(12);
    CHECK(copied.line_number() == 0u);

    const raw_gcode_reader_event_item *copied_items = copied.state().items;
    slic3r_api::GCodeStateTracker moved(std::move(copied));
    REQUIRE(moved.state().items != nullptr);
    CHECK(moved.state().items != copied_items);
    CHECK(moved.line_number() == 0u);

    slic3r_api::GCodeStateTracker assigned;
    assigned = moved;
    REQUIRE(assigned.state().items != nullptr);
    CHECK(assigned.state().items != moved.state().items);
    CHECK(assigned.line_number() == 0u);
}

TEST_CASE("G-code reader reconstructs lines split across borrowed chunks",
          "[plugins][gcode][reader]")
{
    ensure_plugin_test_runtime_initialized();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    const std::vector<std::string_view> chunks = {
        "G91\r", "\nG1 X", "2 E1\nG", "90\nG1 X5"
    };
    size_t next = 0;
    GCodeReaderChunkSource source = [&chunks, &next]() -> std::string_view {
        return next < chunks.size() ? chunks[next++] : std::string_view();
    };

    const slic3r_api::GCodeStateTracker state = read_gcode(
        Orchestrator::instance(), nullptr, initial, std::move(source));

    REQUIRE(state.x());
    REQUIRE(state.e(0));
    CHECK(*state.x() == Approx(5.0));
    CHECK(*state.e(0) == Approx(1.0));
    CHECK(state.line_number() == 3u);
    CHECK(next == chunks.size());
}

TEST_CASE("G-code reader rejects item types outside its declared vocabulary",
          "[plugins][gcode][reader]")
{
    std::vector<raw_gcode_reader_event_item> items;
    raw_gcode_reader_state state = initial_state(items);
    items.push_back({RAW_GCODE_READER_ITEM_LINE_NUMBER + 1u, 0, 1.0});
    state.items = items.data();
    state.item_count = static_cast<uint32_t>(items.size());
    CHECK_THROWS_AS(slic3r_api::GCodeStateTracker(state), std::invalid_argument);
}

TEST_CASE("G-code reader uses common plugin cancellation before reading",
          "[plugins][gcode][reader]")
{
    ensure_plugin_test_runtime_initialized();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    ReaderProbe probe;
    probe.chunks = {"G1 X1\n"};

    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator.request_plugin_cancel();
    const raw_plugin_execution_status status = execute_default_reader(probe, initial);
    orchestrator.reset_plugin_cancel();

    CHECK(status == RAW_PLUGIN_EXECUTION_CANCELLED);
    CHECK(probe.next_chunk == 0);
    CHECK_FALSE(probe.final_state);
}

TEST_CASE("G-code reader uses common plugin cancellation between chunks",
          "[plugins][gcode][reader]")
{
    ensure_plugin_test_runtime_initialized();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    ReaderProbe probe;
    probe.chunks = {"G91\n", "G1 X2\n"};
    probe.cancel_after_chunks = 1;

    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator.reset_plugin_cancel();
    const raw_plugin_execution_status status = execute_default_reader(probe, initial);
    orchestrator.reset_plugin_cancel();

    CHECK(status == RAW_PLUGIN_EXECUTION_CANCELLED);
    CHECK(probe.next_chunk == 1);
    CHECK_FALSE(probe.final_state);
}

} // namespace
