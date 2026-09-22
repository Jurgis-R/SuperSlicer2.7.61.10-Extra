#include <catch2/catch.hpp>

#include <limits>
#include <string>
#include <vector>

#include "plugin_test_helpers.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/cpp/GuiRuleHelpers.hpp"
#include "libslic3r/Config/PrintConfig.hpp"

/*
GuiRule helper tests
====================

GuiRuleHelpers.hpp builds owning GuiCondition/GuiRule expressions and registers
them through the host or the C ABI. These tests cover the eager validation, the
DNF normalization, the multi-target expansion, the duplicate contract and the
link with evaluate_gui_rules(). Every helper rule registered from the test
process has the "core" owner because no plugin is being initialized.
*/

using namespace Slic3r;
using namespace slic3r_api;

namespace {

Orchestrator &test_orchestrator()
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    return Orchestrator::instance();
}

std::vector<const Orchestrator::PluginGuiRule *> stored_rules(const Orchestrator &orchestrator,
                                                              const std::string &target_key)
{
    std::vector<const Orchestrator::PluginGuiRule *> out;
    for (const Orchestrator::PluginGuiRule &rule : orchestrator.gui_rules().rules())
        if (rule.target_key == target_key && rule.owner == CORE_GUI_RULE_OWNER)
            out.push_back(&rule);
    return out;
}

DynamicPrintConfig helper_config()
{
    DynamicPrintConfig config;
    config.set_key_value("gui_rule_int", new ConfigOptionInt(0));
    config.set_key_value("gui_rule_bool", new ConfigOptionBool(false));
    config.set_key_value("gui_rule_percent", new ConfigOptionPercent(50.0));
    config.set_key_value("gui_rule_vector", new ConfigOptionInts());
    config.set_key_value("gui_rule_eval_target", new ConfigOptionBool(false));
    config.set_key_value("gui_rule_eval_and_target", new ConfigOptionBool(false));
    return config;
}

bool evaluated_enabled(const Orchestrator &orchestrator,
                       const DynamicPrintConfig &config,
                       const std::string &target_key)
{
    for (const Orchestrator::GuiRuleApplication &application :
         orchestrator.gui_rules().evaluate(config, RAW_PRESET_TYPE_NONE, -1, CORE_GUI_RULE_OWNER))
        if (application.target_key == target_key)
            return application.enabled;
    return false;
}

} // namespace

TEST_CASE("GuiRule helpers normalize expressions to DNF", "[plugins][gui-rules][helpers]")
{
    Orchestrator &orchestrator = test_orchestrator();

    GuiRule::activate({ "gui_rule_dnf_a", "gui_rule_dnf_b" })
        .when(all_of({
            GuiCondition::non_zero("gui_rule_int"),
            any_of({
                GuiCondition::true_("gui_rule_bool"),
                GuiCondition::int_equals("gui_rule_int", 3)
            })
        }))
        .scope(RAW_PRESET_TYPE_NONE)
        .add_to(orchestrator.handle());

    const std::vector<const Orchestrator::PluginGuiRule *> rules_a = stored_rules(orchestrator, "gui_rule_dnf_a");
    const std::vector<const Orchestrator::PluginGuiRule *> rules_b = stored_rules(orchestrator, "gui_rule_dnf_b");
    REQUIRE(rules_a.size() == 2);
    REQUIRE(rules_b.size() == 2);

    size_t bool_disjunct_count = 0;
    size_t int_disjunct_count = 0;
    for (const Orchestrator::PluginGuiRule *rule : rules_a) {
        CHECK(rule->scope == RAW_PRESET_TYPE_NONE);
        CHECK(rule->target_index == RAW_GUI_RULE_INDEX_ALL);
        REQUIRE(rule->clauses.size() == 2);
        CHECK(rule->clauses[0].condition == RAW_GUI_RULE_CONDITION_VALUE_NON_ZERO);
        CHECK(rule->clauses[0].condition_key == "gui_rule_int");

        if (rule->clauses[1].condition == RAW_GUI_RULE_CONDITION_BOOL_TRUE) {
            CHECK(rule->clauses[1].condition_key == "gui_rule_bool");
            ++bool_disjunct_count;
        } else {
            CHECK(rule->clauses[1].condition == RAW_GUI_RULE_CONDITION_INT_EQUALS);
            CHECK(rule->clauses[1].condition_key == "gui_rule_int");
            CHECK(rule->clauses[1].condition_float_value == 3.0);
            ++int_disjunct_count;
        }
    }
    CHECK(bool_disjunct_count == 1);
    CHECK(int_disjunct_count == 1);

    // A duplicate registration is a silent no-op.
    GuiRule::activate({ "gui_rule_dnf_a", "gui_rule_dnf_b" })
        .when(all_of({
            GuiCondition::non_zero("gui_rule_int"),
            any_of({
                GuiCondition::true_("gui_rule_bool"),
                GuiCondition::int_equals("gui_rule_int", 3)
            })
        }))
        .add_to(orchestrator.handle());
    CHECK(stored_rules(orchestrator, "gui_rule_dnf_a").size() == 2);
    CHECK(stored_rules(orchestrator, "gui_rule_dnf_b").size() == 2);
}

TEST_CASE("GuiRule helpers preserve index and scope", "[plugins][gui-rules][helpers]")
{
    Orchestrator &orchestrator = test_orchestrator();

    GuiRule::activate("gui_rule_attr_target")
        .when(GuiCondition::enabled("gui_rule_bool", 2))
        .target_index(RAW_GUI_RULE_INDEX_CURRENT)
        .scope(RAW_PRESET_TYPE_FFF_PRINT)
        .add_to(orchestrator.handle());

    const std::vector<const Orchestrator::PluginGuiRule *> rules = stored_rules(orchestrator, "gui_rule_attr_target");
    REQUIRE(rules.size() == 1);
    CHECK(rules[0]->owner == CORE_GUI_RULE_OWNER);
    CHECK(rules[0]->target_key == "gui_rule_attr_target");
    CHECK(rules[0]->target_index == RAW_GUI_RULE_INDEX_CURRENT);
    CHECK(rules[0]->scope == RAW_PRESET_TYPE_FFF_PRINT);
    REQUIRE(rules[0]->clauses.size() == 1);
    CHECK(rules[0]->clauses[0].condition == RAW_GUI_RULE_CONDITION_OPTION_ENABLED);
    CHECK(rules[0]->clauses[0].condition_key == "gui_rule_bool");
    CHECK(rules[0]->clauses[0].condition_index == 2);
}

TEST_CASE("GuiRule helpers reject invalid input eagerly", "[plugins][gui-rules][helpers]")
{
    Orchestrator &orchestrator = test_orchestrator();

    CHECK_THROWS_AS(GuiRule::activate(""), std::invalid_argument);
    CHECK_THROWS_AS(GuiRule::activate(std::vector<std::string>{}), std::invalid_argument);
    CHECK_THROWS_AS((GuiRule::activate({ "", "gui_rule_bool" })), std::invalid_argument);
    CHECK_THROWS_AS(GuiCondition::true_(""), std::invalid_argument);
    CHECK_THROWS_AS(GuiCondition::non_zero("gui_rule_int", -3), std::invalid_argument);
    CHECK_THROWS_AS(GuiCondition::float_less_than("gui_rule_percent", std::numeric_limits<double>::quiet_NaN()),
                    std::invalid_argument);
    CHECK_THROWS_AS(any_of({}), std::invalid_argument);
    CHECK_THROWS_AS(all_of({}), std::invalid_argument);
    CHECK_THROWS_AS((any_of({ GuiCondition{} })), std::invalid_argument);

    CHECK_THROWS_AS(GuiRule::activate("gui_rule_bool").add_to(orchestrator.handle()), std::invalid_argument);
    CHECK_THROWS_AS(GuiRule::activate("gui_rule_bool").when(GuiCondition{}), std::invalid_argument);
    CHECK_THROWS_AS(GuiRule::activate("gui_rule_bool").when_true("gui_rule_bool").target_index(-3),
                    std::invalid_argument);
    CHECK_THROWS_AS(GuiRule::activate("gui_rule_bool").when_true("gui_rule_bool")
                        .scope(static_cast<raw_option_preset_type>(RAW_PRESET_TYPE_COUNT + 1)),
                    std::invalid_argument);

    orchestrator_handle *null_handle = nullptr;
    CHECK_THROWS_AS(GuiRule::activate("gui_rule_bool").when_true("gui_rule_bool").add_to(null_handle),
                    std::invalid_argument);
}

TEST_CASE("GuiRule helpers drive evaluate_gui_rules", "[plugins][gui-rules][helpers]")
{
    Orchestrator &orchestrator = test_orchestrator();
    DynamicPrintConfig config = helper_config();

    // A disjunction expands to several activation reasons.
    GuiRule::activate("gui_rule_eval_target")
        .when(any_of({
            GuiCondition::int_equals("gui_rule_int", 3),
            all_of({
                GuiCondition::true_("gui_rule_bool"),
                GuiCondition::percent("gui_rule_percent")
            }),
            GuiCondition::vector_empty("gui_rule_vector")
        }))
        .add_to(orchestrator.handle());

    // vector_empty keeps the target enabled while the vector holds no item.
    CHECK(evaluated_enabled(orchestrator, config, "gui_rule_eval_target"));

    config.option<ConfigOptionInt>("gui_rule_int")->value = 3;
    CHECK(evaluated_enabled(orchestrator, config, "gui_rule_eval_target"));

    config.option<ConfigOptionInt>("gui_rule_int")->value = 0;
    config.set_key_value("gui_rule_vector", new ConfigOptionInts({ 1 }));
    CHECK_FALSE(evaluated_enabled(orchestrator, config, "gui_rule_eval_target"));

    config.option<ConfigOptionBool>("gui_rule_bool")->value = true;
    CHECK(evaluated_enabled(orchestrator, config, "gui_rule_eval_target"));

    config.option<ConfigOptionBool>("gui_rule_bool")->value = false;
    CHECK_FALSE(evaluated_enabled(orchestrator, config, "gui_rule_eval_target"));

    config.set_key_value("gui_rule_vector", new ConfigOptionInts());
    CHECK(evaluated_enabled(orchestrator, config, "gui_rule_eval_target"));

    // Several when() calls on one rule are ANDed.
    config.option<ConfigOptionInt>("gui_rule_int")->value = 1;
    config.option<ConfigOptionBool>("gui_rule_bool")->value = true;
    GuiRule::activate("gui_rule_eval_and_target")
        .when(GuiCondition::non_zero("gui_rule_int"))
        .when_true("gui_rule_bool")
        .add_to(orchestrator.handle());
    CHECK(evaluated_enabled(orchestrator, config, "gui_rule_eval_and_target"));

    config.option<ConfigOptionBool>("gui_rule_bool")->value = false;
    CHECK_FALSE(evaluated_enabled(orchestrator, config, "gui_rule_eval_and_target"));
}

TEST_CASE("GuiRule helpers register through the C ABI and free wrappers", "[plugins][gui-rules][helpers]")
{
    Orchestrator &orchestrator = test_orchestrator();
    orchestrator_handle *handle = orchestrator.handle();

    GuiRule::activate("gui_rule_abi_target")
        .when_true("gui_rule_bool")
        .add_to(handle);
    CHECK(stored_rules(orchestrator, "gui_rule_abi_target").size() == 1);

    add_gui_rule(handle, GuiRule::activate("gui_rule_abi_target_b").when_true("gui_rule_bool"));
    add_gui_rule(orchestrator.handle(), GuiRule::activate("gui_rule_abi_target_c").when_true("gui_rule_bool"));
    add_gui_rules(orchestrator.handle(), {
        GuiRule::activate("gui_rule_abi_target_d").when_true("gui_rule_bool"),
        GuiRule::activate("gui_rule_abi_target_e").when_true("gui_rule_bool")
    });
    CHECK(stored_rules(orchestrator, "gui_rule_abi_target").size() == 1);
    CHECK(stored_rules(orchestrator, "gui_rule_abi_target_b").size() == 1);
    CHECK(stored_rules(orchestrator, "gui_rule_abi_target_c").size() == 1);
    CHECK(stored_rules(orchestrator, "gui_rule_abi_target_d").size() == 1);
    CHECK(stored_rules(orchestrator, "gui_rule_abi_target_e").size() == 1);

    raw_gui_rule_clause clause = raw_gui_rule_clause_init();
    clause.condition = RAW_GUI_RULE_CONDITION_BOOL_TRUE;
    clause.condition_key = "gui_rule_bool";
    raw_gui_rule raw = raw_gui_rule_init();
    raw.target_key = "gui_rule_abi_target";
    raw.clauses = &clause;
    raw.clause_count = 1;
    CHECK(orchestrator_add_gui_rule(handle, &raw) == 0);

    // A distinct condition is a new activation reason, not a duplicate.
    clause.condition = RAW_GUI_RULE_CONDITION_BOOL_FALSE;
    CHECK(orchestrator_add_gui_rule(handle, &raw) == 1);
    CHECK(stored_rules(orchestrator, "gui_rule_abi_target").size() == 2);
}
