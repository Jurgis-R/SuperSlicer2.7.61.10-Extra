#include <catch2/catch.hpp>

#include <cstdint>
#include <limits>
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
#include "libslic3r/Api/plugin/cpp/gcode/GCodeReaderPlugin.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeStateTracker.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Plugins/GCode/Reader/RepRapGCodeReaderState.hpp"

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
using slic3r_api::GCodeGeneration::Reader::RepRapGCodeReaderState;
using slic3r_api::GCodeGeneration::Reader::RepRapToolState;

const char *const k_no_dependencies[] = {nullptr};

class ProbeTestReader final : public slic3r_api::GCodeReaderPluginBase
{
public:
    ProbeTestReader(orchestrator_handle *orchestrator, const char *id,
                    int32_t priority, bool fail_probe)
        : GCodeReaderPluginBase(orchestrator), m_id(id), m_priority(priority),
          m_fail_probe(fail_probe)
    {}

private:
    const char *id_impl() const noexcept override { return m_id; }
    const char *name_impl() const noexcept override { return m_id; }
    const char *description_impl() const noexcept override
    { return "Exercises deterministic G-code Reader probe routing."; }
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    int32_t priority_impl() const noexcept override { return m_priority; }
    uint32_t probe_impl(std::string_view prefix) const override
    {
        if (m_fail_probe && prefix.find(";probe-failure") != std::string_view::npos)
            throw std::runtime_error("Deliberate Reader probe failure.");
        if (m_fail_probe)
            return 0u;
        return prefix.find(";route-test") != std::string_view::npos ? 80u : 0u;
    }
    void read_impl(const plugin_run_context &run_context,
                   const run_ctx_gcode_reader &context,
                   slic3r_api::GCodeStateTracker &) const override
    {
        std::string_view chunk;
        while (next_chunk(run_context, context, chunk)) {}
    }

    const char *m_id;
    int32_t m_priority;
    bool m_fail_probe;
};

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
raw_plugin_execution_status execute_reader(
    const char *reader_id, ReaderProbe &probe,
    const raw_gcode_reader_state &initial, Print *print = nullptr);
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
    return execute_reader("gcode.reader.default", probe, initial);
}

raw_plugin_execution_status execute_reader(
    const char *reader_id, ReaderProbe &probe,
    const raw_gcode_reader_state &initial, Print *print)
{
    Orchestrator &orchestrator = Orchestrator::instance();
    Plugin *plugin = orchestrator.get_plugin(reader_id);
    if (plugin == nullptr)
        throw std::runtime_error(std::string("The G-code reader test provider is not registered: ") + reader_id);

    run_ctx_gcode_reader context = {};
    context.operation = RAW_GCODE_READER_OPERATION_READ;
    context.print = reinterpret_cast<const print_handle *>(print);
    context.initial_state = &initial;
    context.callback_context = &probe;
    context.get_next_chunk = supply_chunk;
    context.emit_event = collect_event;
    context.emit_final_state = collect_final_state;
    raw_plugin_run_payload payload = {&context};
    return orchestrator.execute_plugin(*plugin, print, &payload, 1);
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
        Orchestrator::instance(), nullptr, "gcode.reader.default", initial, "M900 K0.05\n");

    CHECK_FALSE(state.x().has_value());
    CHECK_FALSE(state.active_tool().has_value());
    CHECK(state.line_number() == 1u);
}

TEST_CASE("G-code reader detection recognizes firmware tags and falls back deterministically",
          "[plugins][gcode][reader][probe]")
{
    ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    CHECK(detect_gcode_reader(orchestrator, nullptr,
          "; generated for MaRlIn\nG1 X1\n") == "gcode.reader.marlin");
    CHECK(detect_gcode_reader(orchestrator, nullptr,
          "; RepRapFirmware\nG1 X1\n") == "gcode.reader.reprap");
    CHECK(detect_gcode_reader(orchestrator, nullptr,
          "; KLIPPER\nG1 X1\n") == "gcode.reader.klipper");
    // Equal confidence and priority use the service's lexical-id tie break.
    CHECK(detect_gcode_reader(orchestrator, nullptr,
          "; reprap and klipper\nG1 X1\n") == "gcode.reader.klipper");
    CHECK(detect_gcode_reader(orchestrator, nullptr,
          "; generated by SuperSlicer\nG1 X1\n") == "gcode.reader.default");
    CHECK(detect_gcode_reader(orchestrator, nullptr,
          "; unrelated generator\nG1 X1\n") == "gcode.reader.default");
}

TEST_CASE("Built-in G-code readers are active addressable services",
          "[plugins][gcode][reader][registration]")
{
    ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    const char *const ids[] = {
        "gcode.reader.default", "gcode.reader.marlin",
        "gcode.reader.reprap", "gcode.reader.klipper"
    };
    for (const char *id : ids) {
        Plugin *plugin = orchestrator.get_plugin(id);
        REQUIRE(plugin != nullptr);
        CHECK(plugin->get_step() == GCODE_READER);
        CHECK(plugin->get_exclusive_group().empty());
        CHECK(orchestrator.is_plugin_active(plugin));
    }
}

TEST_CASE("Dialect G-code readers preserve and update representable state",
          "[plugins][gcode][reader][dialects]")
{
    ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);

    const slic3r_api::GCodeStateTracker marlin = read_gcode(
        orchestrator, nullptr, "gcode.reader.marlin", initial,
        "M201 X1000 Y1000\nM204 S750\nM109 R180 T1\nM900 K0.04\nM600\n");
    REQUIRE(marlin.x());
    CHECK(*marlin.x() == Approx(1.0));
    REQUIRE(marlin.value(RAW_GCODE_READER_ITEM_ACCELERATION));
    CHECK(*marlin.value(RAW_GCODE_READER_ITEM_ACCELERATION) == Approx(750.0));
    REQUIRE(marlin.tool_temperature(1));
    CHECK(*marlin.tool_temperature(1) == Approx(180.0));
    CHECK(marlin.known(RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE_REACHED, 1));

    const slic3r_api::GCodeStateTracker reprap = read_gcode(
        orchestrator, nullptr, "gcode.reader.reprap", initial,
        "G10 P1 S215\nM116 P1\nM106 P2 S0.5\nM566 X600\nM572 D1 S0.03\n");
    REQUIRE(reprap.tool_temperature(1));
    CHECK(*reprap.tool_temperature(1) == Approx(215.0));
    CHECK(reprap.known(RAW_GCODE_READER_ITEM_TOOL_TEMPERATURE_REACHED, 1));
    REQUIRE(reprap.fan_speed(2));
    CHECK(*reprap.fan_speed(2) == Approx(50.0));

    const slic3r_api::GCodeStateTracker klipper = read_gcode(
        orchestrator, nullptr, "gcode.reader.klipper", initial,
        "ACTIVATE_EXTRUDER EXTRUDER=extruder2\n"
        "SET_PRESSURE_ADVANCE ADVANCE=0.04 EXTRUDER=extruder2\n"
        "M204 P1200 T900\nPAUSE\n");
    CHECK(klipper.active_tool() == 2u);
    REQUIRE(klipper.value(RAW_GCODE_READER_ITEM_ACCELERATION));
    CHECK(*klipper.value(RAW_GCODE_READER_ITEM_ACCELERATION) == Approx(900.0));
}

TEST_CASE("Klipper G-code reader resolves configured tool names conservatively",
          "[plugins][gcode][reader][dialects][klipper]")
{
    ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    Print print;
    PrintConfig &config = const_cast<PrintConfig &>(print.config());
    config.tool_name.set(std::vector<std::string>{"left_hotend", "right_hotend"});
    config.tool_name.set_enabled(true);

    const slic3r_api::GCodeStateTracker configured = read_gcode(
        orchestrator, &print, "gcode.reader.klipper", initial,
        "ACTIVATE_EXTRUDER EXTRUDER=right_hotend\n");
    CHECK(configured.active_tool() == 1u);

    const slic3r_api::GCodeStateTracker unavailable = read_gcode(
        orchestrator, nullptr, "gcode.reader.klipper", initial,
        "ACTIVATE_EXTRUDER EXTRUDER=right_hotend\n");
    CHECK_FALSE(unavailable.active_tool());

    const slic3r_api::GCodeStateTracker configured_unknown = read_gcode(
        orchestrator, &print, "gcode.reader.klipper", initial,
        "ACTIVATE_EXTRUDER EXTRUDER=extruder\n");
    CHECK_FALSE(configured_unknown.active_tool());

    config.tool_name.set(std::vector<std::string>{"duplicate", "duplicate"});
    const slic3r_api::GCodeStateTracker ambiguous = read_gcode(
        orchestrator, &print, "gcode.reader.klipper", initial,
        "ACTIVATE_EXTRUDER EXTRUDER=duplicate\n");
    CHECK_FALSE(ambiguous.active_tool());
}

TEST_CASE("Dialect G-code readers preserve state across recognized control commands",
          "[plugins][gcode][reader][dialects]")
{
    ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);

    const slic3r_api::GCodeStateTracker marlin = read_gcode(
        orchestrator, nullptr, "gcode.reader.marlin", initial,
        "G4 P1\nG10\nG11\nM201 X1000\nM203 X300\nM205 X8\n"
        "M572 D0 S0.02\nM900 K0.02\nM906 T0 E800\nM0\nM600\nM601\nM226\n");
    CHECK(marlin.x() == Approx(1.0));

    const slic3r_api::GCodeStateTracker reprap = read_gcode(
        orchestrator, nullptr, "gcode.reader.reprap", initial,
        "G4 P1\nM201 X1000\nM203 X18000\nM204 P800 T1000\n"
        "M566 X600\nM572 D0 S0.02\nM906 E800\nM600\nM226\n");
    CHECK(reprap.x() == Approx(1.0));

    const slic3r_api::GCodeStateTracker klipper = read_gcode(
        orchestrator, nullptr, "gcode.reader.klipper", initial,
        "G4 P1\nG10\nG11\nM201 X1000\nM203 X300\nM205 X8\nM906 E800\n");
    CHECK(klipper.x() == Approx(1.0));

    const slic3r_api::GCodeStateTracker malformed_reprap = read_gcode(
        orchestrator, nullptr, "gcode.reader.reprap", initial, "G10 P0\n");
    CHECK_FALSE(malformed_reprap.x());

    const slic3r_api::GCodeStateTracker unknown_marlin = read_gcode(
        orchestrator, nullptr, "gcode.reader.marlin", initial, "M9999\n");
    CHECK_FALSE(unknown_marlin.x());
}

TEST_CASE("G-code reader detection uses confidence priority and stable identifiers",
          "[plugins][gcode][reader][probe]")
{
    ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    // All three fixtures return 80. Numeric priority first removes the
    // lower-priority fixture, then the lexical id selects .a before .b.
    GCodeReaderService service(orchestrator, nullptr);
    CHECK(service.detect(";route-test\nG1 X1\n") == "gcode.reader.test.a");
    // One broken probe must not prevent a healthy provider from winning.
    CHECK(service.detect(";probe-failure\n;route-test\n") == "gcode.reader.test.a");
}

TEST_CASE("G-code reader detection never exposes more than 256 KiB to probes",
          "[plugins][gcode][reader][probe]")
{
    ensure_plugin_test_runtime_initialized();
    std::string text(GCODE_READER_PROBE_PREFIX_SIZE, ' ');
    text += ";klipper\n";
    CHECK(detect_gcode_reader(Orchestrator::instance(), nullptr, text) ==
          DEFAULT_GCODE_READER_PLUGIN_ID);
}

TEST_CASE("G-code reader detection rejects an unavailable default fallback",
          "[plugins][gcode][reader][probe]")
{
    ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    REQUIRE(orchestrator.set_plugin_active(DEFAULT_GCODE_READER_PLUGIN_ID, false));
    CHECK_THROWS_WITH(detect_gcode_reader(orchestrator, nullptr, ";unknown\n"),
                      Catch::Matchers::Contains("default Reader is unavailable"));
    REQUIRE(orchestrator.set_plugin_active(DEFAULT_GCODE_READER_PLUGIN_ID, true));
}

TEST_CASE("G-code reader C++ service view executes the requested Reader synchronously",
          "[plugins][gcode][reader][service]")
{
    ensure_plugin_test_runtime_initialized();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    GCodeReaderService service(Orchestrator::instance(), nullptr);
    const slic3r_api::GCodeStateTracker state =
        slic3r_api::GCodeReaderServiceView(service.c_service()).read("", initial, "G1 X5\n");

    REQUIRE(state.x());
    CHECK(*state.x() == Approx(5.0));
}

TEST_CASE("G-code reader execution rejects an unknown explicit provider",
          "[plugins][gcode][reader][selection]")
{
    ensure_plugin_test_runtime_initialized();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);
    CHECK_THROWS_WITH(read_gcode(Orchestrator::instance(), nullptr,
        "gcode.reader.missing", initial, "G1 X1\n"),
        Catch::Matchers::Contains("is not loaded"));
}

TEST_CASE("G-code reader execution rejects inactive and wrong-step providers",
          "[plugins][gcode][reader][selection]")
{
    ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);

    REQUIRE(orchestrator.set_plugin_active("gcode.reader.test.a", false));
    CHECK_THROWS_WITH(read_gcode(orchestrator, nullptr,
        "gcode.reader.test.a", initial, "G1 X1\n"),
        Catch::Matchers::Contains("is inactive"));
    REQUIRE(orchestrator.set_plugin_active("gcode.reader.test.a", true));

    CHECK_THROWS_WITH(read_gcode(orchestrator, nullptr,
        "gcode.firmware.marlin1", initial, "G1 X1\n"),
        Catch::Matchers::Contains("is not a G-code reader"));
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

TEST_CASE("G-code reader normalizes and validates its provider identity",
          "[plugins][gcode][reader][state]")
{
    std::vector<raw_gcode_reader_event_item> items;
    const raw_gcode_reader_state initial = initial_state(items);
    slic3r_api::GCodeStateTracker normalized(initial);
    CHECK(normalized.reader_id() == 0u);

    normalized.set_reader_id(42u);
    CHECK(normalized.reader_id() == 42u);

    raw_gcode_reader_event_item invalid[] = {
        {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 0.0},
        {RAW_GCODE_READER_ITEM_READER_ID, 1, 42.0}
    };
    raw_gcode_reader_state state = {invalid, 2};
    CHECK_THROWS_AS(slic3r_api::GCodeStateTracker(state), std::invalid_argument);
    invalid[1] = {RAW_GCODE_READER_ITEM_READER_ID, 0, 0.5};
    CHECK_THROWS_AS(slic3r_api::GCodeStateTracker(state), std::invalid_argument);
}

TEST_CASE("G-code reader transports provider-private scalar state opaquely",
          "[plugins][gcode][reader][state]")
{
    std::vector<raw_gcode_reader_event_item> items;
    raw_gcode_reader_state initial = initial_state(items);
    const uint32_t private_type = RAW_GCODE_READER_ITEM_CUSTOM_BEGIN + 17u;
    items.push_back({private_type, 93u, 12.5});
    initial.items = items.data();
    initial.item_count = static_cast<uint32_t>(items.size());

    slic3r_api::GCodeStateTracker state(initial);
    CHECK(state.value(private_type, 93u) == 12.5);
    state.set_custom_value(private_type, 94u, -8.25);
    CHECK(state.value(private_type, 94u) == -8.25);
    state.erase_custom_value(private_type, 93u);
    CHECK_FALSE(state.known(private_type, 93u));

    slic3r_api::GCodeStateTracker copied(state);
    slic3r_api::GCodeStateTracker moved(std::move(copied));
    CHECK(moved.value(private_type, 94u) == -8.25);
    const raw_gcode_reader_event_item update_items[] = {
        {private_type, 94u, 7.0},
        {RAW_GCODE_READER_ITEM_LINE_NUMBER, 0, 1.0}
    };
    raw_gcode_reader_event update = {
        RAW_GCODE_READER_EVENT_STATE_UPDATE, {0, 0}, update_items, 2
    };
    moved.apply(update);
    CHECK(moved.value(private_type, 94u) == 7.0);
    raw_gcode_reader_event invalidate = {
        RAW_GCODE_READER_EVENT_INVALIDATE, {0, 0}, update_items, 2
    };
    moved.apply(invalidate);
    CHECK_FALSE(moved.known(private_type, 94u));

    items.back().value = std::numeric_limits<double>::infinity();
    CHECK_THROWS_AS(slic3r_api::GCodeStateTracker(initial), std::invalid_argument);
}

TEST_CASE("G-code reader rejects a state owned by another Reader identity",
          "[plugins][gcode][reader][state]")
{
    slic3r_api::GCodeStateTracker tracker;
    tracker.set_reader_id(19u);
    CHECK_THROWS_WITH(RepRapGCodeReaderState::from_tracker(tracker),
                      Catch::Matchers::Contains("does not belong to RepRapFirmware"));

    RepRapGCodeReaderState state;
    CHECK_THROWS_WITH(state.write_to(tracker),
                      Catch::Matchers::Contains("another Reader identity"));
}

TEST_CASE("RepRap reader private state round-trips mappings and preserves unknown keys",
          "[plugins][gcode][reader][reprap][state]")
{
    using namespace slic3r_api::GCodeGeneration::Reader;
    slic3r_api::GCodeStateTracker tracker;
    const uint32_t future_key = RAW_GCODE_READER_ITEM_CUSTOM_BEGIN + 0x700000u;
    tracker.set_custom_value(future_key, 4u, 19.0);

    RepRapGCodeReaderState written;
    written.print_acceleration = 800.0;
    written.travel_acceleration = 1500.0;
    written.max_feedrate[0] = 300.0;
    written.pressure_advance[7] = 0.035;
    RepRapToolState &tool = written.tools[2];
    tool.drives = {7, 8};
    tool.heaters = {3};
    tool.fans = {1, 4};
    tool.standby_temperature = 145.0;
    tool.offset = {1.5, -2.0, 0.25};
    tool.retracted = true;
    written.tools[9];
    written.write_to(tracker);

    const RepRapGCodeReaderState decoded = RepRapGCodeReaderState::from_tracker(tracker);
    CHECK(decoded.print_acceleration == 800.0);
    CHECK(decoded.travel_acceleration == 1500.0);
    CHECK(decoded.max_feedrate[0] == 300.0);
    CHECK(decoded.pressure_advance.at(7) == Approx(0.035));
    REQUIRE(decoded.tools.count(2) == 1);
    CHECK(decoded.tools.at(2).drives == std::vector<uint32_t>{7, 8});
    CHECK(decoded.tools.at(2).heaters == std::vector<uint32_t>{3});
    CHECK(decoded.tools.at(2).fans == std::vector<uint32_t>{1, 4});
    CHECK(decoded.tools.at(2).standby_temperature == 145.0);
    CHECK(decoded.tools.at(2).offset[1] == -2.0);
    CHECK(decoded.tools.at(2).retracted == true);
    REQUIRE(decoded.tools.count(9) == 1);
    CHECK(decoded.tools.at(9).drives.empty());
    CHECK(tracker.value(future_key, 4u) == 19.0);
}

TEST_CASE("RepRap reader private mappings reject malformed slot lists",
          "[plugins][gcode][reader][reprap][state]")
{
    using namespace slic3r_api::GCodeGeneration::Reader;
    SECTION("missing slot") {
        slic3r_api::GCodeStateTracker tracker;
        tracker.set_custom_value(RRF_ITEM_TOOL_DRIVE_COUNT, 0, 2.0);
        tracker.set_custom_value(RRF_ITEM_TOOL_DRIVE_SLOT_BEGIN, 0, 3.0);
        CHECK_THROWS_WITH(RepRapGCodeReaderState::from_tracker(tracker),
                          Catch::Matchers::Contains("missing indexed slot"));
    }
    SECTION("duplicate device") {
        slic3r_api::GCodeStateTracker tracker;
        tracker.set_custom_value(RRF_ITEM_TOOL_DRIVE_COUNT, 0, 2.0);
        tracker.set_custom_value(RRF_ITEM_TOOL_DRIVE_SLOT_BEGIN, 0, 3.0);
        tracker.set_custom_value(RRF_ITEM_TOOL_DRIVE_SLOT_BEGIN + 1u, 0, 3.0);
        CHECK_THROWS_WITH(RepRapGCodeReaderState::from_tracker(tracker),
                          Catch::Matchers::Contains("duplicate device"));
    }
    SECTION("slot beyond count") {
        slic3r_api::GCodeStateTracker tracker;
        tracker.set_custom_value(RRF_ITEM_TOOL_DRIVE_COUNT, 0, 1.0);
        tracker.set_custom_value(RRF_ITEM_TOOL_DRIVE_SLOT_BEGIN, 0, 3.0);
        tracker.set_custom_value(RRF_ITEM_TOOL_DRIVE_SLOT_BEGIN + 1u, 0, 4.0);
        CHECK_THROWS_WITH(RepRapGCodeReaderState::from_tracker(tracker),
                          Catch::Matchers::Contains("beyond its count"));
    }
    SECTION("invalid count") {
        slic3r_api::GCodeStateTracker tracker;
        tracker.set_custom_value(RRF_ITEM_TOOL_DRIVE_COUNT, 0, -1.0);
        CHECK_THROWS_WITH(RepRapGCodeReaderState::from_tracker(tracker),
                          Catch::Matchers::Contains("exact uint32"));
        tracker.set_custom_value(RRF_ITEM_TOOL_DRIVE_COUNT, 0, 1.5);
        CHECK_THROWS_WITH(RepRapGCodeReaderState::from_tracker(tracker),
                          Catch::Matchers::Contains("exact uint32"));
    }
    SECTION("invalid device id") {
        slic3r_api::GCodeStateTracker tracker;
        tracker.set_custom_value(RRF_ITEM_TOOL_DRIVE_COUNT, 0, 1.0);
        tracker.set_custom_value(RRF_ITEM_TOOL_DRIVE_SLOT_BEGIN, 0, 2.5);
        CHECK_THROWS_WITH(RepRapGCodeReaderState::from_tracker(tracker),
                          Catch::Matchers::Contains("exact uint32"));
    }
}

TEST_CASE("RepRap reader parses its operational private state",
          "[plugins][gcode][reader][reprap][state]")
{
    ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    std::vector<raw_gcode_reader_event_item> initial_items;
    const raw_gcode_reader_state initial = initial_state(initial_items);

    const slic3r_api::GCodeStateTracker tracked = read_gcode(
        orchestrator, nullptr, "gcode.reader.reprap", initial,
        "M563 P2 D3:4 H5 F6:7\n"
        "T2\n"
        "G10 P2 S210 R145 X1.5 Y-2 Z0.25\n"
        "M201 X1000 Y1100 Z100 E1200\n"
        "M203 X18000 Y12000 Z600 E900 I60\n"
        "M205 X8 Y9 Z0.4 E2\n"
        "M566 X600 Y660 Z30 E120\n"
        "M204 P800 T1500\n"
        "M568 P2 S220 R150\n"
        "M572 D3 S0.035\n"
        "M106 S0.25\n"
        "G10\nG11\nM116\n");

    const RepRapGCodeReaderState state = RepRapGCodeReaderState::from_tracker(tracked);
    REQUIRE(state.tools.count(2) == 1);
    const RepRapToolState &tool = state.tools.at(2);
    CHECK(tool.drives == std::vector<uint32_t>{3, 4});
    CHECK(tool.heaters == std::vector<uint32_t>{5});
    CHECK(tool.fans == std::vector<uint32_t>{6, 7});
    CHECK(tool.standby_temperature == 150.0);
    CHECK(tool.offset[0] == 1.5);
    CHECK(tool.offset[1] == -2.0);
    CHECK(tool.offset[2] == 0.25);
    CHECK(tool.retracted == false);
    CHECK(state.max_acceleration[1] == 1100.0);
    CHECK(state.max_feedrate[0] == 300.0);
    CHECK(state.min_extruding_feedrate == 1.0);
    CHECK(state.job_jerk[0] == 8.0);
    CHECK(state.machine_jerk[1] == 11.0);
    CHECK(state.print_acceleration == 800.0);
    CHECK(state.travel_acceleration == 1500.0);
    CHECK(state.pressure_advance.at(3) == Approx(0.035));
    CHECK(tracked.tool_temperature(2) == 220.0);
    CHECK(tracked.fan_speed(6) == Approx(25.0));
    CHECK(tracked.fan_speed(7) == Approx(25.0));
}

TEST_CASE("G-code readers preserve opaque state only while commands stay understood",
          "[plugins][gcode][reader][state]")
{
    ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    std::vector<raw_gcode_reader_event_item> items;
    raw_gcode_reader_state initial = initial_state(items);
    const uint32_t private_type = RAW_GCODE_READER_ITEM_CUSTOM_BEGIN + 0x700001u;
    items.push_back({private_type, 12u, 3.5});
    initial.items = items.data();
    initial.item_count = static_cast<uint32_t>(items.size());

    const slic3r_api::GCodeStateTracker transported = read_gcode(
        orchestrator, nullptr, "gcode.reader.default", initial, "G1 X2\n");
    CHECK(transported.value(private_type, 12u) == 3.5);

    const slic3r_api::GCodeStateTracker invalidated = read_gcode(
        orchestrator, nullptr, "gcode.reader.default", initial, "M9999\n");
    CHECK_FALSE(invalidated.known(private_type, 12u));
    CHECK(invalidated.reader_id() == 0u);
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
        Orchestrator::instance(), nullptr, "gcode.reader.default", initial, std::move(source));

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
    items.push_back({RAW_GCODE_READER_ITEM_READER_ID + 1u, 0, 1.0});
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

namespace Slic3r::Test::Plugins {

void register_gcode_reader_test_providers(orchestrator_handle *orchestrator)
{
    static ProbeTestReader lexical_first(orchestrator, "gcode.reader.test.a", 10, false);
    static ProbeTestReader lexical_second(orchestrator, "gcode.reader.test.b", 10, false);
    static ProbeTestReader lower_priority(orchestrator, "gcode.reader.test.lower_priority", 20, false);
    static ProbeTestReader failing_probe(orchestrator, "gcode.reader.test.failing_probe", 0, true);
    // Register lexical_second first so the tie-break test also proves that
    // discovery order cannot change the selected id.
    orchestrator_register_plugin(orchestrator, lexical_second.c_instance());
    orchestrator_register_plugin(orchestrator, lexical_first.c_instance());
    orchestrator_register_plugin(orchestrator, lower_priority.c_instance());
    orchestrator_register_plugin(orchestrator, failing_probe.c_instance());
}

} // namespace Slic3r::Test::Plugins
