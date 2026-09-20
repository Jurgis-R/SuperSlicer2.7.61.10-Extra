#include <catch2/catch.hpp>

#include <algorithm>
#include <string>
#include <vector>

#include "plugin_test_helpers.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/Config/PrintConfig.hpp"

/*
Plugin-defined option lists
===========================

These tests cover the generic machinery behind plugin-defined enum settings:
contains_plugin_id pruning, deferred option definition patches and the
STRING_EQUALS GUI condition. The infill pattern lists are the reference user of
that machinery.

The option fixtures are registered by the shared test runtime before it applies
deferred patches, because PrintConfigDef can only be extended before it is
finalized.
*/

using namespace Slic3r;

namespace {

const ConfigOptionDef *option_def(const char *key)
{
    return PrintConfigDef::instance().get(key);
}

std::vector<std::string> option_values(const char *key)
{
    const ConfigOptionDef *def = option_def(key);
    if (def == nullptr || def->enum_def.get() == nullptr)
        return {};
    return def->enum_def->values();
}

bool has_value(const std::vector<std::string> &values, const std::string &value)
{
    return std::find(values.begin(), values.end(), value) != values.end();
}

} // namespace

TEST_CASE("Infill pattern lists are plugin-defined enums", "[plugins][infill][options]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    for (const char *key : { "fill_pattern", "top_fill_pattern", "bottom_fill_pattern",
                             "solid_fill_pattern", "bridge_fill_pattern" }) {
        const ConfigOptionDef *def = option_def(key);
        INFO(key);
        REQUIRE(def != nullptr);
        CHECK(def->contains_plugin_id);
        CHECK(def->type == coEnum);
    }

    const std::vector<std::string> solid = option_values("solid_fill_pattern");
    CHECK(has_value(solid, "ensuring"));
    CHECK(has_value(solid, "smooth"));
    CHECK_FALSE(has_value(solid, "monotonicgapfill"));

    const std::vector<std::string> sparse = option_values("fill_pattern");
    CHECK(has_value(sparse, "stars"));
    CHECK(has_value(sparse, "gyroid"));
    CHECK_FALSE(has_value(sparse, "ensuring"));

    // Each slot keeps its own subset instead of sharing one merged list.
    CHECK(has_value(option_values("top_fill_pattern"), "sawtooth"));
    CHECK_FALSE(has_value(option_values("bottom_fill_pattern"), "sawtooth"));
    CHECK_FALSE(has_value(option_values("solid_fill_pattern"), "grid"));

    const std::vector<std::string> bridge = option_values("bridge_fill_pattern");
    REQUIRE(bridge.size() == 2);
    CHECK(has_value(bridge, "rectilinear"));
    CHECK(has_value(bridge, "monotonic"));

    // Defaults are owned by the plugin definition, not reset to a single value.
    REQUIRE(option_def("solid_fill_pattern")->default_value);
    CHECK(option_def("solid_fill_pattern")->default_value->serialize() == "ensuring");
    REQUIRE(option_def("fill_pattern")->default_value);
    CHECK(option_def("fill_pattern")->default_value->serialize() == "stars");
    REQUIRE(option_def("top_fill_pattern")->default_value);
    CHECK(option_def("top_fill_pattern")->default_value->serialize() == "monotonic");
}

TEST_CASE("Plugin-id option lists prune inactive plugins", "[plugins][infill][options]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    // The fixture starts with one active plugin id and one unknown id.
    CHECK(option_values("test.infill.prune_list") == std::vector<std::string>{ "rectilinear" });

    // A list left empty by pruning is reported as a non-fatal error.
    CHECK(option_values("test.infill.empty_list").empty());
    CHECK(Slic3r::Test::Plugins::plugin_test_empty_option_list_error_reported());
}

TEST_CASE("REPLICATE_ENUM copies a resolved list and breaks cycles softly", "[plugins][infill][options]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    const ConfigOptionDef *target = option_def("test.replicate.target");
    REQUIRE(target != nullptr);
    REQUIRE(target->enum_def.get() != nullptr);
    CHECK(target->enum_def->values() == std::vector<std::string>{ "rectilinear" });

    // A->B and B->A: the cyclic edge is ignored, so each option still resolves
    // once and deterministically.
    const ConfigOptionDef *cycle_a = option_def("test.cycle.a");
    const ConfigOptionDef *cycle_b = option_def("test.cycle.b");
    REQUIRE(cycle_a != nullptr);
    REQUIRE(cycle_b != nullptr);
    REQUIRE(cycle_a->enum_def.get() != nullptr);
    REQUIRE(cycle_b->enum_def.get() != nullptr);
    CHECK_FALSE(cycle_a->enum_def->values().empty());
    CHECK(cycle_a->enum_def->values() == cycle_b->enum_def->values());
}

TEST_CASE("STRING_EQUALS compares serialized option values", "[plugins][gui-rules]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    const char *const group = "test.string_equals";

    raw_gui_rule_clause clause = raw_gui_rule_clause_init();
    clause.condition = RAW_GUI_RULE_CONDITION_STRING_EQUALS;
    clause.condition_key = "solid_fill_pattern";
    clause.condition_string_value = "smooth";

    raw_gui_rule rule = raw_gui_rule_init();
    rule.action = RAW_GUI_RULE_ACTION_ENABLE_ANY;
    rule.target_key = "solid_fill_pattern";
    rule.rule_group = group;
    rule.clauses = &clause;
    rule.clause_count = 1;
    CHECK(orchestrator.add_gui_rule(&rule) == Orchestrator::GuiRuleRegistration::Added);

    const auto target_enabled = [&orchestrator, group](const DynamicPrintConfig &config) {
        for (const Orchestrator::GuiRuleApplication &application :
             orchestrator.evaluate_gui_rules(config, RAW_PRESET_TYPE_FFF_PRINT, -1, group))
            if (application.target_key == "solid_fill_pattern")
                return application.enabled;
        return false;
    };

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict("solid_fill_pattern", "smooth");
    CHECK(target_enabled(config));

    config.set_deserialize_strict("solid_fill_pattern", "rectilinear");
    CHECK_FALSE(target_enabled(config));
}
