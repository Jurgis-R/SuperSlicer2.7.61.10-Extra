///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Tests for plugin-owned temporary configurations.

The suite exercises the C ABI as a plugin sees it, then uses the C++ wrappers
to verify their ownership contract. The core SCFG codec is covered separately
by libslic3r tests; this suite verifies the ABI buffer and merge semantics.
*/

#include <catch2/catch.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "plugin_test_helpers.hpp"

#include "libslic3r/Api/host/ApiHostUtils.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/c/slic3r_config.h"
#include "libslic3r/Api/plugin/cpp/ConfigDefinitionViews.hpp"
#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"
#include "libslic3r/Api/plugin/cpp/OrchestratorViews.hpp"
#include "libslic3r/Config/ConfigDef.hpp"
#include "libslic3r/Config/ConfigOption.hpp"

namespace {

// This small static store exercises the API rejection path without depending
// on the global print-config caches initialized by the larger slicing fixture.
class StaticTestConfig final : public Slic3r::StaticConfig
{
public:
    const Slic3r::ConfigDef *def() const override { return nullptr; }

    const Slic3r::ConfigOption *optptr(const Slic3r::t_config_option_key &key) const override
    {
        return key == "value" ? &m_value : nullptr;
    }

    Slic3r::ConfigOption *optptr(const Slic3r::t_config_option_key &key, bool /* create */) override
    {
        return key == "value" ? &m_value : nullptr;
    }

    Slic3r::t_config_option_keys keys() const override { return {"value"}; }

private:
    Slic3r::ConfigOptionInt m_value{17};
};

static std::string serialize_all(const config_handle *config);

// Use the public two-call protocol so tests also validate byte counts and null
// termination rather than reaching into the host implementation.
static std::string serialize_all(const config_handle *config)
{
    const uint32_t size = config_serialize_all(config, nullptr, 0);
    REQUIRE(size > 0);
    std::string serialized(size + 1, '\0');
    REQUIRE(config_serialize_all(config, &serialized[0], size + 1) == size);
    REQUIRE(serialized[size] == '\0');
    serialized.resize(size);
    return serialized;
}

} // namespace

TEST_CASE("Plugin storage owns temporary configs", "[plugins][config]")
{
    Slic3r::PluginStorage storage;
    storage_handle *storage_api = reinterpret_cast<storage_handle *>(&storage);
    REQUIRE(storage_size(storage_api) == 0);

    config_handle *first = storage_new_config(storage_api);
    REQUIRE(first != nullptr);
    CHECK(is_local_storage(storage_api, first) == 1);
    CHECK(storage_size(storage_api) == 1);

    // StableOwnedVector keeps the first ConfigBase subobject at the same address
    // while later allocations grow the owning vector.
    std::vector<config_handle *> configs;
    for (size_t idx = 0; idx < 32; ++idx)
        configs.push_back(storage_new_config(storage_api));
    CHECK(config_get_or_add_mutable(first, "stable", SLIC3R_CONFIG_OPTION_INT) != nullptr);
    CHECK(storage_size(storage_api) == 33);

    CHECK(storage_free(storage_api, first) == 1);
    CHECK(storage_size(storage_api) == 32);
    storage_clear(storage_api);
    CHECK(storage_size(storage_api) == 0);
}

TEST_CASE("Temporary configs create options and clear only dynamic configs", "[plugins][config]")
{
    Slic3r::PluginStorage storage;
    storage_handle *storage_api = reinterpret_cast<storage_handle *>(&storage);
    config_handle *config = storage_new_config(storage_api);
    REQUIRE(config != nullptr);

    config_option_handle *integer =
        config_get_or_add_mutable(config, "value", SLIC3R_CONFIG_OPTION_INT);
    REQUIRE(integer != nullptr);
    CHECK(config_get_or_add_mutable(config, "value", SLIC3R_CONFIG_OPTION_INT) == integer);
    CHECK(config_get_or_add_mutable(config, "value", SLIC3R_CONFIG_OPTION_STRING) == nullptr);
    CHECK(config_get_or_add_mutable(config, "", SLIC3R_CONFIG_OPTION_INT) == nullptr);
    CHECK(config_get_or_add_mutable(config, "none", SLIC3R_CONFIG_OPTION_NONE) == nullptr);
    CHECK(config_get_or_add_mutable(config, "unknown", static_cast<config_option_type>(999)) == nullptr);

    config_option_set_int(integer, 91, 0);
    CHECK(config_option_get_int(config_get(config, "value"), 0) == 91);
    CHECK(config_clear(config) == 1);
    CHECK(config_keys(config).size == 0);
    CHECK(config_clear(config) == 1);

    StaticTestConfig static_config;
    config_handle *static_handle = Slic3r::ApiHost::to_config_handle(&static_config);
    const size_t static_key_count = static_config.keys().size();
    CHECK(config_clear(static_handle) == 0);
    CHECK(config_get_or_add_mutable(static_handle, "temporary", SLIC3R_CONFIG_OPTION_INT) == nullptr);
    CHECK(static_config.keys().size() == static_key_count);
    CHECK(static_config.option<Slic3r::ConfigOptionInt>("value")->value == 17);
}

TEST_CASE("SCFG C API follows the two-call buffer contract", "[plugins][config][serialization]")
{
    Slic3r::DynamicConfig source;
    source.set_key_value("value", new Slic3r::ConfigOptionString("left:\nright"));
    const std::string serialized = serialize_all(Slic3r::ApiHost::to_config_handle(&source));
    CHECK(serialized.rfind("SCFG1\n", 0) == 0);

    Slic3r::DynamicConfig restored;
    REQUIRE(config_deserialize_all(Slic3r::ApiHost::to_config_handle(&restored), serialized.c_str()) == 1);
    CHECK(restored.option<Slic3r::ConfigOptionString>("value")->value == "left:\nright");
}

TEST_CASE("SCFG merge is atomic and allows an incoming type change", "[plugins][config][serialization]")
{
    Slic3r::DynamicConfig destination;
    destination.set_key_value("keep", new Slic3r::ConfigOptionInt(7));
    destination.set_key_value("replace", new Slic3r::ConfigOptionInt(8));

    Slic3r::DynamicConfig incoming;
    incoming.set_key_value("replace", new Slic3r::ConfigOptionString("changed"));
    incoming.set_key_value("added", new Slic3r::ConfigOptionBool(true));
    const std::string incoming_serialized = serialize_all(Slic3r::ApiHost::to_config_handle(&incoming));

    REQUIRE(config_deserialize_all(
        Slic3r::ApiHost::to_config_handle(&destination), incoming_serialized.c_str()) == 1);
    CHECK(destination.option< Slic3r::ConfigOptionInt>("keep")->value == 7);
    CHECK(destination.option<Slic3r::ConfigOptionString>("replace")->value == "changed");
    CHECK(destination.option<Slic3r::ConfigOptionBool>("added")->value);

    const std::string before_failure = serialize_all(Slic3r::ApiHost::to_config_handle(&destination));
    CHECK(config_deserialize_all(Slic3r::ApiHost::to_config_handle(&destination), "SCFG2\n0\n") == 0);
    CHECK(serialize_all(Slic3r::ApiHost::to_config_handle(&destination)) == before_failure);

    REQUIRE(config_clear(Slic3r::ApiHost::to_config_handle(&destination)) == 1);
    REQUIRE(config_deserialize_all(
        Slic3r::ApiHost::to_config_handle(&destination), incoming_serialized.c_str()) == 1);
    CHECK(destination.keys() == incoming.keys());
}

TEST_CASE("StoredConfig provides move-only C++ ownership", "[plugins][config][cpp]")
{
    Slic3r::PluginStorage storage;
    storage_handle *storage_api = reinterpret_cast<storage_handle *>(&storage);

    {
        slic3r_api::StoredConfig source(storage_api);
        slic3r_api::MutableConfigOption layer =
            source.get_or_add("layer_num", SLIC3R_CONFIG_OPTION_INT);
        layer.set_int(12);
        const std::string serialized = source.serialize_all();

        slic3r_api::StoredConfig destination(storage_api);
        destination.get_or_add("preserved", SLIC3R_CONFIG_OPTION_BOOL).set_bool(true);
        destination.deserialize_all(serialized);
        CHECK(destination.int_or_default("layer_num", 0) == 12);
        CHECK(destination.bool_or_default("preserved", false));

        slic3r_api::StoredConfig moved(std::move(destination));
        CHECK(moved.int_or_default("layer_num", 0) == 12);
        moved.clear();
        CHECK(moved.keys().empty());
    }

    CHECK(storage_size(storage_api) == 0);
}

TEST_CASE("Configuration definition keys are ordered and filterable",
          "[plugins][config][definition]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Slic3r::Orchestrator &orchestrator = Slic3r::Orchestrator::instance();
    const orchestrator_handle *handle = reinterpret_cast<const orchestrator_handle *>(&orchestrator);

    CHECK(orchestrator_get_option_keys(nullptr, RAW_CO_NONE, nullptr, 0) == 0);
    const uint32_t total = orchestrator_get_option_keys(handle, RAW_CO_NONE, nullptr, 0);
    REQUIRE(total > 2);

    std::vector<raw_config_option_key> keys(total);
    CHECK(orchestrator_get_option_keys(handle, RAW_CO_NONE, keys.data(), uint32_t(keys.size())) == total);
    CHECK(std::is_sorted(keys.begin(), keys.end(),
        [](const raw_config_option_key &lhs, const raw_config_option_key &rhs) {
            return std::strcmp(lhs.key, rhs.key) < 0;
        }));

    std::array<raw_config_option_key, 2> prefix = {};
    CHECK(orchestrator_get_option_keys(handle, RAW_CO_NONE, prefix.data(), uint32_t(prefix.size())) == total);
    CHECK(std::strcmp(prefix[0].key, keys[0].key) == 0);
    CHECK(std::strcmp(prefix[1].key, keys[1].key) == 0);

    for (int value = int(RAW_CO_BOOL); value <= int(RAW_CO_POINT3); ++value) {
        const raw_config_option_type filter = static_cast<raw_config_option_type>(value);
        const uint32_t filtered_count = orchestrator_get_option_keys(handle, filter, nullptr, 0);
        std::vector<raw_config_option_key> filtered(filtered_count);
        CHECK(orchestrator_get_option_keys(
            handle, filter, filtered.data(), uint32_t(filtered.size())) == filtered_count);
        for (const raw_config_option_key &entry : filtered)
            CHECK(entry.type == filter);
    }

    const std::vector<raw_config_option_key>::const_iterator point3 = std::find_if(
        keys.begin(), keys.end(), [](const raw_config_option_key &entry) {
            return std::strcmp(entry.key, "test_config_definition_point3_introspection") == 0;
        });
    REQUIRE(point3 != keys.end());
    CHECK(point3->type == RAW_CO_POINT3);
}

TEST_CASE("Configuration definitions expose normalized plugin metadata",
          "[plugins][config][definition]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Slic3r::Orchestrator &orchestrator = Slic3r::Orchestrator::instance();
    const orchestrator_handle *handle = reinterpret_cast<const orchestrator_handle *>(&orchestrator);

    raw_config_option_def raw;
    std::memset(&raw, 0xA5, sizeof(raw));
    CHECK(orchestrator_get_option_def(handle, "missing.test.option", &raw) == 0);
    const unsigned char *bytes = reinterpret_cast<const unsigned char *>(&raw);
    CHECK(std::all_of(bytes, bytes + sizeof(raw), [](unsigned char value) { return value == 0; }));
    CHECK(orchestrator_get_option_def(nullptr, "layer_height", &raw) == 0);
    CHECK(orchestrator_get_option_def(handle, nullptr, &raw) == 0);
    CHECK(orchestrator_get_option_def(handle, "layer_height", nullptr) == 0);

    REQUIRE(orchestrator_get_option_def(handle, "test_config_definition_introspection", &raw) == 1);
    CHECK(std::string(raw.opt_key) == "test_config_definition_introspection");
    CHECK(raw.type == RAW_CO_FLOAT);
    CHECK(raw.category == RAW_OPTION_CATEGORY_ADVANCED);
    CHECK(raw.level == RAW_OPTION_LEVEL_EXPERT);
    CHECK(raw.container_type == RAW_CONTAINER_TYPE_PROJECT);
    CHECK(raw.option_preset_type == RAW_PRESET_TYPE_FFF_PRINT);
    CHECK(raw.gui_type == RAW_GUI_TYPE_F_ENUM_OPEN);
    CHECK(raw.printer_technology == RAW_PT_FFF);
    CHECK(raw.invalidates_step == STEP_INFILL);
    CHECK(raw.is_optional == 1);
    CHECK(std::string(raw.gui_flags) == "serialized,show_value");
    CHECK(std::string(raw.label) == "Introspection value");
    CHECK(std::string(raw.full_label) == "Plugin configuration introspection value");
    CHECK(std::string(raw.tooltip) == "Exercises the complete normalized definition snapshot.");
    CHECK(std::string(raw.sidetext) == "mm");
    CHECK(std::string(raw.cli) == "test-config-definition-introspection=f");
    CHECK(std::string(raw.ratio_over) == "nozzle_diameter");
    CHECK(raw.multiline == 1);
    CHECK(raw.full_width == 1);
    CHECK(raw.is_code == 1);
    CHECK(raw.is_vector_extruder == 0);
    CHECK(raw.readonly == 1);
    CHECK(raw.can_phony == 1);
    CHECK(raw.can_be_disabled == 1);
    CHECK(raw.height == 7);
    CHECK(raw.width == 19);
    CHECK(raw.label_width == 23);
    CHECK(raw.aligned_label_left == 1);
    CHECK(raw.sidetext_width == 11);
    CHECK(raw.has_min == 1);
    CHECK(raw.min_value == Approx(-2.5));
    CHECK(raw.has_max == 1);
    CHECK(raw.max_value == Approx(8.5));
    CHECK(raw.has_max_literal == 1);
    CHECK(raw.max_literal_value == Approx(4.5));
    CHECK(raw.max_literal_is_percent == 1);
    CHECK(raw.precision == 4);
    CHECK(raw.mode == RAW_CONFIG_OPTION_MODE_EXPERT);
    CHECK(raw.flags == uint64_t(0x1020304050607080ULL));
    REQUIRE(raw.aliases.size == 2);
    CHECK(std::string(raw.aliases.items[0]) == "test_introspection_old");
    CHECK(std::string(raw.aliases.items[1]) == "test_introspection_legacy");
    REQUIRE(raw.shortcut.size == 2);
    CHECK(std::string(raw.shortcut.items[0]) == "perimeter_speed");
    CHECK(std::string(raw.shortcut.items[1]) == "infill_speed");
    REQUIRE(raw.depends_on.size == 2);
    CHECK(std::string(raw.depends_on.items[0]) == "layer_height#0");
    CHECK(std::string(raw.depends_on.items[1]) == "nozzle_diameter#0");
    CHECK(std::string(raw.default_serialized_value) == "1.25");
    REQUIRE(raw.enum_def.value_label_pairs.count == 2);
    CHECK(std::string(raw.enum_def.value_label_pairs.items[0].value) == "0.5");
    CHECK(std::string(raw.enum_def.value_label_pairs.items[0].label) == "Half");
    CHECK(std::string(raw.enum_def.value_label_pairs.items[1].value) == "1.25");
    CHECK(std::string(raw.enum_def.value_label_pairs.items[1].label) == "Nominal");
    for (uint64_t value : raw.reserved_u64)
        CHECK(value == 0);
    for (const void *value : raw.reserved_ptr)
        CHECK(value == nullptr);

    REQUIRE(orchestrator_get_option_def(
        handle, "test_config_definition_vector_introspection", &raw) == 1);
    CHECK(raw.type == RAW_CO_VECTOR_FLOAT);
    CHECK(raw.is_vector_extruder == 1);
}

TEST_CASE("C++ configuration definition views own their copied data",
          "[plugins][config][definition][cpp]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Slic3r::Orchestrator &orchestrator = Slic3r::Orchestrator::instance();
    slic3r_api::OrchestratorView view(reinterpret_cast<orchestrator_handle *>(&orchestrator));

    const std::vector<slic3r_api::ConfigOptionKey> keys = view.option_keys(RAW_CO_POINT3);
    REQUIRE_FALSE(keys.empty());
    CHECK(std::any_of(keys.begin(), keys.end(), [](const slic3r_api::ConfigOptionKey &entry) {
        return entry.key == "test_config_definition_point3_introspection" && entry.type == RAW_CO_POINT3;
    }));

    const std::optional<slic3r_api::ConfigOptionDefinition> definition =
        view.option_definition("test_config_definition_introspection");
    REQUIRE(definition.has_value());
    REQUIRE(view.option_definition("layer_height").has_value());
    CHECK(definition->label == std::optional<std::string>("Introspection value"));
    CHECK(definition->aliases == std::vector<std::string>{
        "test_introspection_old", "test_introspection_legacy"});
    REQUIRE(definition->enum_entries.size() == 2);
    CHECK(definition->enum_entries[1].label == std::optional<std::string>("Nominal"));
    CHECK_FALSE(view.option_definition("missing.test.option").has_value());
}

TEST_CASE("Configuration definition snapshots are isolated between threads",
          "[plugins][config][definition][threading]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Slic3r::Orchestrator &orchestrator = Slic3r::Orchestrator::instance();
    const orchestrator_handle *handle = reinterpret_cast<const orchestrator_handle *>(&orchestrator);
    std::atomic<int> ready {0};
    std::array<std::string, 2> before;
    std::array<std::string, 2> after;
    std::array<uintptr_t, 2> addresses = {};
    std::array<bool, 2> succeeded = {};

    const auto query = [&](size_t index, const char *key) {
        raw_config_option_def raw = {};
        succeeded[index] = orchestrator_get_option_def(handle, key, &raw) && raw.label != nullptr;
        if (succeeded[index]) {
            before[index] = raw.label;
            addresses[index] = reinterpret_cast<uintptr_t>(raw.label);
        }
        ready.fetch_add(1, std::memory_order_release);
        while (ready.load(std::memory_order_acquire) < 2)
            std::this_thread::yield();
        if (succeeded[index])
            after[index] = raw.label;
    };

    std::thread first(query, 0, "test_config_definition_introspection");
    std::thread second(query, 1, "layer_height");
    first.join();
    second.join();

    REQUIRE(ready.load(std::memory_order_relaxed) == 2);
    REQUIRE(succeeded[0]);
    REQUIRE(succeeded[1]);
    CHECK(before == after);
    CHECK(addresses[0] != addresses[1]);
}
