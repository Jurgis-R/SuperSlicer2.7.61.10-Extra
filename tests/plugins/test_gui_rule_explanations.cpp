#include <catch2/catch.hpp>

#include <initializer_list>
#include <string>
#include <vector>

#include "plugin_test_helpers.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Config/ConfigDef.hpp"
#include "libslic3r/Config/GuiRuleTexts.hpp"
#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/Config/PrintConfig.hpp"

using namespace Slic3r;

namespace {

const char *const explain_owner = "gui_rule_explain_test_owner";

raw_gui_rule_clause explain_clause(raw_gui_rule_condition condition,
                                   const char *key,
                                   int32_t index = RAW_GUI_RULE_INDEX_ALL,
                                   double value = 0.0)
{
    raw_gui_rule_clause clause = raw_gui_rule_clause_init();
    clause.condition = condition;
    clause.condition_key = key;
    clause.condition_index = index;
    clause.condition_float_value = value;
    return clause;
}

void add_explain_rule(const char *target,
                      std::initializer_list<raw_gui_rule_clause> clauses,
                      int32_t target_index = RAW_GUI_RULE_INDEX_ALL,
                      raw_option_preset_type scope = RAW_PRESET_TYPE_FFF_PRINT)
{
    std::vector<raw_gui_rule_clause> storage(clauses);
    raw_gui_rule rule = raw_gui_rule_init();
    rule.target_key = target;
    rule.target_index = target_index;
    rule.scope = scope;
    rule.clauses = storage.data();
    rule.clause_count = uint32_t(storage.size());
    REQUIRE(Orchestrator::instance().gui_rules().add(&rule, explain_owner) ==
            Orchestrator::GuiRuleRegistration::Added);
}

// Config with a bool target and two condition options. Rules can target keys
// that have no ConfigOptionDef: the explanation reads the config snapshot.
DynamicPrintConfig explain_config(const std::string &target)
{
    DynamicPrintConfig config;
    config.set_key_value(target, new ConfigOptionBool(true));
    config.set_key_value("gui_rule_explain_bool", new ConfigOptionBool(false));
    config.set_key_value("gui_rule_explain_int", new ConfigOptionInt(3));
    return config;
}

} // namespace

TEST_CASE("GUI rule explanation lists the clauses of every activation reason", "[gui-rules][explanation]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    const std::string target = "gui_rule_explain_target_a";
    DynamicPrintConfig config = explain_config(target);
    const OptionKeyIdx key{target, -1};

    // No rule targets the key: nothing to explain.
    CHECK(orchestrator.gui_rules().explain_disabled(config, key, RAW_PRESET_TYPE_FFF_PRINT).empty());

    add_explain_rule(target.c_str(), {explain_clause(RAW_GUI_RULE_CONDITION_BOOL_TRUE, "gui_rule_explain_bool")});

    const std::string explanation =
        orchestrator.gui_rules().explain_disabled(config, key, RAW_PRESET_TYPE_FFF_PRINT);
    INFO(explanation);
    CHECK(explanation.rfind(gui_rule_text(GuiRuleText::TO_ENABLE), 0) == 0);
    CHECK(explanation.find("gui_rule_explain_bool") != std::string::npos);
    CHECK(explanation.find(gui_rule_text(GuiRuleText::IS_TRUE)) != std::string::npos);
    CHECK(explanation.find(gui_rule_text(GuiRuleText::UNMET)) != std::string::npos);
    CHECK(explanation.find(gui_rule_text(GuiRuleText::SATISFIED)) == std::string::npos);

    // A satisfied rule means the widget should be enabled: no explanation.
    config.set_key_value("gui_rule_explain_bool", new ConfigOptionBool(true));
    orchestrator.gui_rules().evaluate(config, RAW_PRESET_TYPE_FFF_PRINT, -1);
    CHECK(orchestrator.gui_rules().explain_disabled(config, key, RAW_PRESET_TYPE_FFF_PRINT).empty());
}

TEST_CASE("GUI rule explanation marks satisfied and unmet clauses", "[gui-rules][explanation]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    const std::string target = "gui_rule_explain_target_b";
    DynamicPrintConfig config = explain_config(target);
    config.set_key_value("gui_rule_explain_bool", new ConfigOptionBool(true));

    add_explain_rule(target.c_str(),
                     {explain_clause(RAW_GUI_RULE_CONDITION_BOOL_TRUE, "gui_rule_explain_bool"),
                      explain_clause(RAW_GUI_RULE_CONDITION_INT_EQUALS, "gui_rule_explain_int", RAW_GUI_RULE_INDEX_ALL, 5.0)});

    const std::string explanation =
        orchestrator.gui_rules().explain_disabled(config, OptionKeyIdx{target, -1}, RAW_PRESET_TYPE_FFF_PRINT);
    INFO(explanation);
    CHECK(explanation.find("gui_rule_explain_bool " + gui_rule_text(GuiRuleText::IS_TRUE) + " " +
                           gui_rule_text(GuiRuleText::SATISFIED)) != std::string::npos);
    CHECK(explanation.find("gui_rule_explain_int " + gui_rule_text(GuiRuleText::EQUALS) + " 5 " +
                           gui_rule_text(GuiRuleText::UNMET)) != std::string::npos);
    CHECK(explanation.find(" " + gui_rule_text(GuiRuleText::AND) + " ") != std::string::npos);
}

TEST_CASE("GUI rule explanation resolves enum operands", "[gui-rules][explanation]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    const std::string target = "gui_rule_explain_target_c";
    DynamicPrintConfig config = explain_config(target);
    config.set_key_value("perimeter_generator",
                         new ConfigOptionEnum<PerimeterGeneratorType>(PerimeterGeneratorType::Classic));

    add_explain_rule(target.c_str(),
                     {explain_clause(RAW_GUI_RULE_CONDITION_INT_EQUALS, "perimeter_generator",
                                     RAW_GUI_RULE_INDEX_ALL, double(int(PerimeterGeneratorType::Arachne)))});

    const std::string explanation =
        orchestrator.gui_rules().explain_disabled(config, OptionKeyIdx{target, -1}, RAW_PRESET_TYPE_FFF_PRINT);
    INFO(explanation);
    CHECK(explanation.find("perimeter_generator " + gui_rule_text(GuiRuleText::EQUALS) + " Arachne") != std::string::npos);
    CHECK(explanation.find(gui_rule_text(GuiRuleText::UNMET)) != std::string::npos);
}

TEST_CASE("GUI rule explanation follows the explained index", "[gui-rules][explanation]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    // Explicit target index: applied by the global pass for that item only.
    const std::string explicit_target = "gui_rule_explain_target_d";
    DynamicPrintConfig explicit_config = explain_config(explicit_target);
    add_explain_rule(explicit_target.c_str(),
                     {explain_clause(RAW_GUI_RULE_CONDITION_BOOL_TRUE, "gui_rule_explain_bool")},
                     /*target_index*/ 1);
    CHECK_FALSE(orchestrator.gui_rules().explain_disabled(explicit_config, OptionKeyIdx{explicit_target, 1}, RAW_PRESET_TYPE_FFF_PRINT).empty());
    CHECK(orchestrator.gui_rules().explain_disabled(explicit_config, OptionKeyIdx{explicit_target, 0}, RAW_PRESET_TYPE_FFF_PRINT).empty());
    CHECK(orchestrator.gui_rules().explain_disabled(explicit_config, OptionKeyIdx{explicit_target, -1}, RAW_PRESET_TYPE_FFF_PRINT).empty());

    // CURRENT target index on an extruder-sized vector, with a CURRENT
    // condition index: each item is explained with its own value.
    const std::string current_target = "gui_rule_explain_target_e";
    DynamicPrintConfig current_config = explain_config(current_target);
    ConfigOptionFloats *target_vector = new ConfigOptionFloats({0.0, 0.0});
    target_vector->set_is_extruder_size(true);
    current_config.set_key_value(current_target, target_vector);
    ConfigOptionFloats *nozzles = new ConfigOptionFloats({0.4, 0.6});
    nozzles->set_is_extruder_size(true);
    current_config.set_key_value("nozzle_diameter", nozzles);

    add_explain_rule(current_target.c_str(),
                     {explain_clause(RAW_GUI_RULE_CONDITION_FLOAT_GREATER_OR_EQUAL, "nozzle_diameter",
                                     RAW_GUI_RULE_INDEX_CURRENT, 0.5)},
                     RAW_GUI_RULE_INDEX_CURRENT);

    const std::string first =
        orchestrator.gui_rules().explain_disabled(current_config, OptionKeyIdx{current_target, 0}, RAW_PRESET_TYPE_FFF_PRINT);
    INFO(first);
    CHECK(first.find("nozzle_diameter[0] " + gui_rule_text(GuiRuleText::GREATER_OR_EQUAL) + " 0.5") != std::string::npos);
    CHECK(orchestrator.gui_rules().explain_disabled(current_config, OptionKeyIdx{current_target, 1}, RAW_PRESET_TYPE_FFF_PRINT).empty());
}

TEST_CASE("GUI rule explanation uses translated option labels", "[gui-rules][explanation]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    const std::string target = "gui_rule_explain_target_f";
    DynamicPrintConfig config = explain_config(target);
    config.set_key_value("perimeters", new ConfigOptionInt(0));

    add_explain_rule(target.c_str(),
                     {explain_clause(RAW_GUI_RULE_CONDITION_VALUE_NON_ZERO, "perimeters")});

    const std::string raw =
        orchestrator.gui_rules().explain_disabled(config, OptionKeyIdx{target, -1}, RAW_PRESET_TYPE_FFF_PRINT,
                                                  GuiRuleRegistry::ExplanationStyle::RawKeys);
    CHECK(raw.find("perimeters " + gui_rule_text(GuiRuleText::NON_ZERO)) != std::string::npos);

    // Fill the translated labels through the same post-finalize entry point the
    // GUI uses.
    CHECK(PrintConfigDef::update_definition_translations([](const std::string &text, const std::string &) {
        return text == "Perimeters" ? std::string("Périmètres") : text;
    }) >= 1);

    const std::string translated =
        orchestrator.gui_rules().explain_disabled(config, OptionKeyIdx{target, -1}, RAW_PRESET_TYPE_FFF_PRINT,
                                                  GuiRuleRegistry::ExplanationStyle::TranslatedLabels);
    INFO(translated);
    CHECK(translated.find("Périmètres " + gui_rule_text(GuiRuleText::NON_ZERO)) != std::string::npos);

    // Restore the raw labels for the other tests.
    PrintConfigDef::update_definition_translations([](const std::string &text, const std::string &) { return text; });
    orchestrator.gui_rules().invalidate_explanations();
    CHECK(PrintConfigDef::instance().get("perimeters")->label_translated.empty());
}

TEST_CASE("GUI rule explanation cache follows registrations and evaluations", "[gui-rules][explanation]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    const std::string target = "gui_rule_explain_target_g";
    DynamicPrintConfig config = explain_config(target);
    const OptionKeyIdx key{target, -1};

    // Empty explanations are cached too; adding a rule must drop them.
    CHECK(orchestrator.gui_rules().explain_disabled(config, key, RAW_PRESET_TYPE_FFF_PRINT).empty());
    add_explain_rule(target.c_str(), {explain_clause(RAW_GUI_RULE_CONDITION_BOOL_TRUE, "gui_rule_explain_bool")});
    CHECK_FALSE(orchestrator.gui_rules().explain_disabled(config, key, RAW_PRESET_TYPE_FFF_PRINT).empty());

    // The condition now holds, but the cached text is kept until the next
    // evaluation invalidates it.
    config.set_key_value("gui_rule_explain_bool", new ConfigOptionBool(true));
    CHECK_FALSE(orchestrator.gui_rules().explain_disabled(config, key, RAW_PRESET_TYPE_FFF_PRINT).empty());
    orchestrator.gui_rules().evaluate(config, RAW_PRESET_TYPE_FFF_PRINT, -1);
    CHECK(orchestrator.gui_rules().explain_disabled(config, key, RAW_PRESET_TYPE_FFF_PRINT).empty());
}

TEST_CASE("GUI rule text vocabulary can be translated repeatedly", "[gui-rules][explanation]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    const std::string base_true = gui_rule_text(GuiRuleText::IS_TRUE);

    const size_t changed = update_gui_rule_texts([](const std::string &text) {
        return text == "is true" ? std::string("est vrai") : text;
    });
    CHECK(changed == 1);
    CHECK(gui_rule_text(GuiRuleText::IS_TRUE) == "est vrai");

    // Re-running the same translation changes nothing.
    CHECK(update_gui_rule_texts([](const std::string &text) {
        return text == "is true" ? std::string("est vrai") : text;
    }) == 0);

    const std::string target = "gui_rule_explain_target_h";
    DynamicPrintConfig config = explain_config(target);
    add_explain_rule(target.c_str(), {explain_clause(RAW_GUI_RULE_CONDITION_BOOL_TRUE, "gui_rule_explain_bool")});
    orchestrator.gui_rules().invalidate_explanations();
    const std::string explanation =
        orchestrator.gui_rules().explain_disabled(config, OptionKeyIdx{target, -1}, RAW_PRESET_TYPE_FFF_PRINT);
    CHECK(explanation.find("est vrai") != std::string::npos);

    // Restore the English base for the other tests.
    update_gui_rule_texts([](const std::string &text) { return text; });
    CHECK(gui_rule_text(GuiRuleText::IS_TRUE) == base_true);
    orchestrator.gui_rules().invalidate_explanations();
}
