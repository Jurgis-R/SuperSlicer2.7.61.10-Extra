///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "GuiRuleRegistry.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <boost/log/trivial.hpp>

#include "GuiRuleTexts.hpp"
#include "PrintConfig.hpp"

namespace Slic3r {

namespace {

using PluginGuiRuleTarget = std::pair<std::string, int32_t>;

// A vector option sized like the extruder count can be indexed per extruder.
// Other vectors keep their automatic "any item satisfies the clause" read.
const ConfigOptionVectorBase *extruder_sized_vector(const ConfigOption *option)
{
    if (option == nullptr || !option->is_vector())
        return nullptr;

    const ConfigOptionVectorBase *vector_option = dynamic_cast<const ConfigOptionVectorBase *>(option);
    return vector_option != nullptr && vector_option->is_extruder_size() ? vector_option : nullptr;
}

bool gui_rule_condition_value_at_index(const ConfigOption &condition_option,
                                       const GuiRuleRegistry::Clause &clause,
                                       int32_t condition_index)
{
    // Scalar get_bool() ignores the index. Vector get_bool() needs a real item
    // index, so "-1 / unspecified" falls back to 0 only for this low-level read.
    const size_t value_index = condition_index < 0 ? 0 : size_t(condition_index);

    switch (clause.condition) {
    case RAW_GUI_RULE_CONDITION_BOOL_TRUE:
        return condition_option.get_bool(value_index);
    case RAW_GUI_RULE_CONDITION_BOOL_FALSE:
        return !condition_option.get_bool(value_index);
    case RAW_GUI_RULE_CONDITION_OPTION_ENABLED:
        return condition_option.is_enabled(condition_index);
    case RAW_GUI_RULE_CONDITION_OPTION_DISABLED:
        return !condition_option.is_enabled(condition_index);
    case RAW_GUI_RULE_CONDITION_VALUE_NON_ZERO:
        return condition_option.get_bool(value_index);
    case RAW_GUI_RULE_CONDITION_INT_EQUALS:
        return condition_option.get_int(value_index) == int32_t(std::llround(clause.condition_float_value));
    case RAW_GUI_RULE_CONDITION_INT_NOT_EQUALS:
        return condition_option.get_int(value_index) != int32_t(std::llround(clause.condition_float_value));
    case RAW_GUI_RULE_CONDITION_FLOAT_EQUALS:
        return condition_option.get_float(value_index) == clause.condition_float_value;
    case RAW_GUI_RULE_CONDITION_FLOAT_LESS_THAN:
        return condition_option.get_float(value_index) < clause.condition_float_value;
    case RAW_GUI_RULE_CONDITION_FLOAT_GREATER_OR_EQUAL:
        return condition_option.get_float(value_index) >= clause.condition_float_value;
    case RAW_GUI_RULE_CONDITION_IS_PERCENT:
        return condition_option.is_percent(value_index);
    case RAW_GUI_RULE_CONDITION_VECTOR_EMPTY: {
        const ConfigOptionVectorBase *vector_option =
            dynamic_cast<const ConfigOptionVectorBase *>(&condition_option);
        return vector_option != nullptr && vector_option->empty();
    }
    case RAW_GUI_RULE_CONDITION_NONE:
    default:
        return false;
    }
}

bool gui_rule_condition_value(const ConfigOption &condition_option,
                              const GuiRuleRegistry::Clause &clause,
                              int current_index)
{
    // VECTOR_EMPTY describes the whole option, not one item, so it must not be
    // routed through the "any item satisfies the clause" vector read.
    if (clause.condition == RAW_GUI_RULE_CONDITION_VECTOR_EMPTY)
        return gui_rule_condition_value_at_index(condition_option, clause, -1);

    // Rule-provided index is always explicit and therefore wins.
    if (clause.condition_index >= 0)
        return gui_rule_condition_value_at_index(condition_option, clause, clause.condition_index);

    if (clause.condition_index == RAW_GUI_RULE_INDEX_CURRENT) {
        if (current_index < 0 || extruder_sized_vector(&condition_option) == nullptr)
            return false;
        return gui_rule_condition_value_at_index(condition_option, clause, current_index);
    }

    // If the caller is currently processing extruder i, and the condition is an
    // extruder-sized vector, evaluate the same i-th item.
    if (current_index >= 0 && extruder_sized_vector(&condition_option) != nullptr)
        return gui_rule_condition_value_at_index(condition_option, clause, current_index);

    // No explicit index and no usable current index: for a vector condition,
    // enable the target if at least one vector item satisfies the clause. This
    // is safer than silently reading item 0.
    if (condition_option.is_vector()) {
        for (size_t condition_index = 0; condition_index < condition_option.size(); ++condition_index)
            if (gui_rule_condition_value_at_index(condition_option, clause, int32_t(condition_index)))
                return true;
        return false;
    }

    // Scalar condition, no index needed.
    return gui_rule_condition_value_at_index(condition_option, clause, -1);
}

bool gui_rule_target_index_for_context(const ConfigOption &target_option,
                                       const GuiRuleRegistry::Rule &rule,
                                       int current_index,
                                       int &target_index)
{
    // Rule-provided target index is explicit and handled once by the global
    // pass. Running it again in the per-extruder pass would duplicate the call.
    target_index = rule.target_index;
    if (target_index >= 0)
        return current_index < 0;

    // RAW_GUI_RULE_INDEX_CURRENT means "toggle the item currently processed by
    // the extruder loop".
    if (rule.target_index == RAW_GUI_RULE_INDEX_CURRENT) {
        if (current_index < 0 || extruder_sized_vector(&target_option) == nullptr)
            return false;

        target_index = current_index;
        return true;
    }

    // RAW_GUI_RULE_INDEX_ALL / default: apply the whole GUI field once during
    // the global pass.
    if (current_index >= 0)
        return false;

    target_index = RAW_GUI_RULE_INDEX_ALL;
    return true;
}

bool gui_rule_matches_scope(raw_option_preset_type rule_scope, raw_option_preset_type scope)
{
    return rule_scope == RAW_PRESET_TYPE_NONE || rule_scope == scope;
}

bool gui_rule_condition_is_valid(raw_gui_rule_condition condition)
{
    return condition > RAW_GUI_RULE_CONDITION_NONE && condition < RAW_GUI_RULE_CONDITION_COUNT;
}

bool gui_rule_index_is_valid(int32_t index)
{
    return index >= 0 || index == RAW_GUI_RULE_INDEX_ALL || index == RAW_GUI_RULE_INDEX_CURRENT;
}

bool gui_rule_condition_needs_finite_float(raw_gui_rule_condition condition)
{
    // INT_* rounds the same field, and llround() of a non-finite value is
    // undefined behavior.
    return condition == RAW_GUI_RULE_CONDITION_INT_EQUALS ||
           condition == RAW_GUI_RULE_CONDITION_INT_NOT_EQUALS ||
           condition == RAW_GUI_RULE_CONDITION_FLOAT_EQUALS ||
           condition == RAW_GUI_RULE_CONDITION_FLOAT_LESS_THAN ||
           condition == RAW_GUI_RULE_CONDITION_FLOAT_GREATER_OR_EQUAL;
}

// Per-call evaluation state shared by the full scan and the per-page scan.
// The clause memo removes the duplication created by the per-target expansion
// of the rule builders: the same condition is often evaluated for dozens of
// targets. The option cache removes the repeated string map lookups for target
// and condition keys. Both borrow their strings from m_rules, which is
// stable for the whole call.
struct GuiRuleClauseCacheKey
{
    raw_gui_rule_condition condition;
    int32_t condition_index;
    double condition_float_value;
    std::string_view condition_key;

    bool operator==(const GuiRuleClauseCacheKey &rhs) const
    {
        return condition == rhs.condition &&
               condition_index == rhs.condition_index &&
               condition_float_value == rhs.condition_float_value &&
               condition_key == rhs.condition_key;
    }
};

struct GuiRuleClauseCacheHash
{
    size_t operator()(const GuiRuleClauseCacheKey &key) const
    {
        size_t seed = 0;
        config_hash_combine(seed, int32_t(key.condition));
        config_hash_combine(seed, key.condition_index);
        config_hash_combine(seed, key.condition_float_value);
        config_hash_combine(seed, key.condition_key);
        return seed;
    }
};

enum class GuiRuleClauseResult : uint8_t
{
    False,
    True,
    Invalid
};

struct GuiRuleEvaluationContext
{
    const DynamicPrintConfig &config;
    raw_option_preset_type scope = RAW_PRESET_TYPE_NONE;
    int32_t current_index = -1;
    const std::string &owner_filter;
    std::unordered_map<GuiRuleClauseCacheKey, GuiRuleClauseResult, GuiRuleClauseCacheHash> clause_cache;
    std::unordered_map<std::string_view, const ConfigOption *> option_cache;
    std::map<PluginGuiRuleTarget, bool> enabled_by_target;

    GuiRuleEvaluationContext(const DynamicPrintConfig &config_in,
                             raw_option_preset_type scope_in,
                             int32_t current_index_in,
                             const std::string &owner_filter_in) :
        config(config_in),
        scope(scope_in),
        current_index(current_index_in),
        owner_filter(owner_filter_in)
    {
    }

    const ConfigOption *option_for(const std::string &key)
    {
        const std::string_view view(key);
        const auto existing = option_cache.find(view);
        if (existing != option_cache.end())
            return existing->second;
        const ConfigOption *option = config.optptr(key);
        option_cache.emplace(view, option);
        return option;
    }

    // Evaluate one rule and OR its result into enabled_by_target.
    void evaluate_rule(const GuiRuleRegistry::Rule &rule)
    {
        if (!gui_rule_matches_scope(rule.scope, scope))
            return;
        if (!owner_filter.empty() && rule.owner != owner_filter)
            return;
        if (rule.clauses.empty())
            return;

        // Missing keys mean "not applicable to this config". A rule only
        // participates when its target and every clause option exist.
        const ConfigOption *target_option = option_for(rule.target_key);
        if (target_option == nullptr)
            return;
        bool applicable = true;
        for (const GuiRuleRegistry::Clause &clause : rule.clauses)
            if (clause.condition_key.empty() || option_for(clause.condition_key) == nullptr) {
                applicable = false;
                break;
            }
        if (!applicable)
            return;

        int target_index = -1;
        if (!gui_rule_target_index_for_context(*target_option, rule, current_index, target_index))
            return;

        bool condition_enabled = true;
        bool rule_skipped = false;
        for (const GuiRuleRegistry::Clause &clause : rule.clauses) {
            const GuiRuleClauseCacheKey cache_key{
                clause.condition, clause.condition_index, clause.condition_float_value, clause.condition_key };
            GuiRuleClauseResult result = GuiRuleClauseResult::False;
            const auto cached = clause_cache.find(cache_key);
            if (cached != clause_cache.end()) {
                result = cached->second;
            } else {
                try {
                    const ConfigOption *condition_option = option_for(clause.condition_key);
                    result = condition_option != nullptr &&
                             gui_rule_condition_value(*condition_option, clause, current_index)
                                 ? GuiRuleClauseResult::True : GuiRuleClauseResult::False;
                } catch (...) {
                    // Malformed plugin input must not crash the GUI refresh.
                    result = GuiRuleClauseResult::Invalid;
                }
                clause_cache.emplace(cache_key, result);
            }

            if (result == GuiRuleClauseResult::Invalid) {
                rule_skipped = true;
                break;
            }
            if (result == GuiRuleClauseResult::False) {
                condition_enabled = false;
                break;
            }
        }
        if (rule_skipped)
            return;

        const PluginGuiRuleTarget target(rule.target_key, target_index);
        enabled_by_target[target] = enabled_by_target[target] || condition_enabled;
    }
};

// Return nullptr when the descriptor is valid, otherwise the rejection reason.
// Unknown enum values must never be stored: the evaluator would skip them
// silently or read an out-of-range index as RAW_GUI_RULE_INDEX_ALL.
//
// The message is built here because a truncated struct_size means the
// descriptor may be allocated with fewer bytes than raw_gui_rule: reading
// target_key from such an allocation would be out of bounds. target_key is
// therefore only read once struct_size proves the member exists.
std::string gui_rule_validation_error(const raw_gui_rule *rule)
{
    if (rule == nullptr)
        return "Rejected GUI rule: null descriptor.";
    if (rule->struct_size < sizeof(raw_gui_rule))
        return "Rejected GUI rule: struct_size is smaller than the host descriptor.";

    const std::string prefix = std::string("Rejected GUI rule for target '") +
        (rule->target_key != nullptr ? rule->target_key : "<null>") + "':";
    const auto reject = [&prefix](const char *reason) {
        return prefix + " " + reason + ".";
    };

    if (rule->target_key == nullptr || rule->target_key[0] == '\0')
        return reject("missing target_key");
    if (!gui_rule_index_is_valid(rule->target_index))
        return reject("invalid target_index");
    if (rule->scope >= RAW_PRESET_TYPE_COUNT)
        return reject("unknown scope");
    if (rule->clause_count == 0)
        return reject("no condition clause");
    if (rule->clauses == nullptr)
        return reject("null clause array");

    for (uint32_t clause_idx = 0; clause_idx < rule->clause_count; ++clause_idx) {
        const raw_gui_rule_clause &clause = rule->clauses[clause_idx];
        if (!gui_rule_condition_is_valid(clause.condition))
            return reject("unknown condition");
        if (clause.condition_key == nullptr || clause.condition_key[0] == '\0')
            return reject("missing clause condition_key");
        if (!gui_rule_index_is_valid(clause.condition_index))
            return reject("invalid clause condition_index");
        if (gui_rule_condition_needs_finite_float(clause.condition) &&
            !std::isfinite(clause.condition_float_value))
            return reject("non-finite condition_float_value");
    }
    return std::string();
}

// Format a numeric operand without trailing zeros.
std::string gui_rule_float_operand(double value)
{
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%g", value);
    return buffer;
}

// Name used for an option in the explanation text.
std::string gui_rule_option_name(const std::string &key, GuiRuleRegistry::ExplanationStyle style)
{
    if (style == GuiRuleRegistry::ExplanationStyle::TranslatedLabels) {
        const ConfigOptionDef *def = PrintConfigDef::instance().get(key);
        if (def != nullptr) {
            const std::string &label = def->translated_label();
            if (!label.empty())
                return label;
        }
    }
    return key;
}

// Numeric operand of one clause: an enum label when the condition key resolves
// to a closed enum value, otherwise the number.
std::string gui_rule_operand_text(const GuiRuleRegistry::Clause &clause, GuiRuleRegistry::ExplanationStyle style)
{
    if (clause.condition == RAW_GUI_RULE_CONDITION_INT_EQUALS ||
        clause.condition == RAW_GUI_RULE_CONDITION_INT_NOT_EQUALS) {
        const int32_t value = int32_t(std::llround(clause.condition_float_value));
        const ConfigOptionDef *def = PrintConfigDef::instance().get(clause.condition_key);
        if (def != nullptr && def->type == coEnum && def->enum_def && def->enum_def->is_valid_closed_enum()) {
            if (const std::optional<int> index = def->enum_def->enum_to_index(value); index.has_value()) {
                const std::vector<std::string> &labels =
                    style == GuiRuleRegistry::ExplanationStyle::TranslatedLabels ? def->enum_def->translated_labels()
                                                                                : def->enum_def->labels();
                if (size_t(*index) < labels.size())
                    return labels[*index];
            }
        }
        return std::to_string(value);
    }
    return gui_rule_float_operand(clause.condition_float_value);
}

// Human readable text of one clause, without its state marker.
std::string gui_rule_clause_text(const GuiRuleRegistry::Clause &clause,
                                 int32_t current_index,
                                 GuiRuleRegistry::ExplanationStyle style)
{
    std::string text = gui_rule_option_name(clause.condition_key, style);
    if (clause.condition_index >= 0)
        text += "[" + std::to_string(clause.condition_index) + "]";
    else if (clause.condition_index == RAW_GUI_RULE_INDEX_CURRENT) {
        text += "[";
        text += current_index >= 0 ? std::to_string(current_index) : gui_rule_text(GuiRuleText::CURRENT_INDEX);
        text += "]";
    }

    switch (clause.condition) {
    case RAW_GUI_RULE_CONDITION_BOOL_TRUE:
        text += " " + gui_rule_text(GuiRuleText::IS_TRUE);
        break;
    case RAW_GUI_RULE_CONDITION_BOOL_FALSE:
        text += " " + gui_rule_text(GuiRuleText::IS_FALSE);
        break;
    case RAW_GUI_RULE_CONDITION_OPTION_ENABLED:
        text += " " + gui_rule_text(GuiRuleText::IS_ENABLED);
        break;
    case RAW_GUI_RULE_CONDITION_OPTION_DISABLED:
        text += " " + gui_rule_text(GuiRuleText::IS_DISABLED);
        break;
    case RAW_GUI_RULE_CONDITION_VALUE_NON_ZERO:
        text += " " + gui_rule_text(GuiRuleText::NON_ZERO);
        break;
    case RAW_GUI_RULE_CONDITION_INT_EQUALS:
    case RAW_GUI_RULE_CONDITION_FLOAT_EQUALS:
        text += " " + gui_rule_text(GuiRuleText::EQUALS) + " " + gui_rule_operand_text(clause, style);
        break;
    case RAW_GUI_RULE_CONDITION_INT_NOT_EQUALS:
        text += " " + gui_rule_text(GuiRuleText::NOT_EQUALS) + " " + gui_rule_operand_text(clause, style);
        break;
    case RAW_GUI_RULE_CONDITION_FLOAT_LESS_THAN:
        text += " " + gui_rule_text(GuiRuleText::LESS_THAN) + " " + gui_rule_operand_text(clause, style);
        break;
    case RAW_GUI_RULE_CONDITION_FLOAT_GREATER_OR_EQUAL:
        text += " " + gui_rule_text(GuiRuleText::GREATER_OR_EQUAL) + " " + gui_rule_operand_text(clause, style);
        break;
    case RAW_GUI_RULE_CONDITION_IS_PERCENT:
        text += " " + gui_rule_text(GuiRuleText::IS_PERCENT);
        break;
    case RAW_GUI_RULE_CONDITION_VECTOR_EMPTY:
        text += " " + gui_rule_text(GuiRuleText::IS_EMPTY);
        break;
    case RAW_GUI_RULE_CONDITION_NONE:
    default:
        break;
    }
    return text;
}

// Decide whether one rule explains the widget (target_key, target_index), and
// with which clause evaluation index. This mirrors the pass that produced the
// widget state: the global pass handles ALL and explicit-index rules, while the
// per-extruder pass handles CURRENT rules on extruder-sized targets.
bool gui_rule_applies_to_widget(const GuiRuleRegistry::Rule &rule,
                                const ConfigOption &target_option,
                                int32_t target_index,
                                int32_t &evaluation_index)
{
    if (target_index < 0) {
        // Whole field: only the global pass applies.
        if (rule.target_index != RAW_GUI_RULE_INDEX_ALL)
            return false;
        evaluation_index = -1;
        return true;
    }
    if (rule.target_index == RAW_GUI_RULE_INDEX_CURRENT) {
        if (extruder_sized_vector(&target_option) == nullptr)
            return false;
        evaluation_index = target_index;
        return true;
    }
    if (rule.target_index == target_index) {
        // Explicit index: applied by the global pass.
        evaluation_index = -1;
        return true;
    }
    return false;
}

} // namespace

// Attach one already validated and stored rule to its setting activation group.
// Returns false when the setting definition is not registered yet.
bool GuiRuleRegistry::attach_rule_to_setting(const std::string &setting_key, size_t rule_index)
{
    // Read through the finalized-safe accessor: activation rules may be
    // registered after PrintConfigDef::finalize() (deferred plugin init, tests).
    const ConfigOptionDef *option_def = PrintConfigDef::instance().get(setting_key);
    if (option_def == nullptr)
        return false;

    SettingActivation *activation = nullptr;
    if (option_def->gui_activation_id != 0 && option_def->gui_activation_id <= m_setting_activations.size())
        activation = &m_setting_activations[option_def->gui_activation_id - 1];

    if (activation == nullptr) {
        m_setting_activations.push_back(SettingActivation{ setting_key, {} });
        activation = &m_setting_activations.back();
        PrintConfigDef::set_gui_activation_id(setting_key, static_cast<uint32_t>(m_setting_activations.size()));
        assert(m_setting_activations.back().setting_key == setting_key);
    }
    else {
        // The stored id must never point at another setting; a mismatch means a
        // definition was replaced without clearing its host-only link.
        assert(activation->setting_key == setting_key);
        if (activation->setting_key != setting_key)
            return false;
    }

    activation->rule_indices.push_back(rule_index);
    return true;
}

// Validate and retain one GUI rule; an identical rule is not inserted twice.
GuiRuleRegistry::Registration GuiRuleRegistry::add(const raw_gui_rule *rule, const char *owner)
{
    const std::string validation_error = gui_rule_validation_error(rule);
    if (!validation_error.empty()) {
        BOOST_LOG_TRIVIAL(warning) << validation_error;
        return GuiRuleRegistration::Invalid;
    }

    PluginGuiRule gui_rule;
    gui_rule.target_key = rule->target_key;
    gui_rule.target_index = rule->target_index;
    gui_rule.scope = rule->scope;
    gui_rule.owner = owner != nullptr ? owner : std::string(CORE_GUI_RULE_OWNER);
    gui_rule.clauses.reserve(rule->clause_count);
    for (uint32_t clause_idx = 0; clause_idx < rule->clause_count; ++clause_idx) {
        const raw_gui_rule_clause &clause = rule->clauses[clause_idx];
        PluginGuiRuleClause stored;
        stored.condition = clause.condition;
        stored.condition_key = clause.condition_key;
        stored.condition_index = clause.condition_index;
        stored.condition_float_value = clause.condition_float_value;
        gui_rule.clauses.emplace_back(std::move(stored));
    }

    for (const PluginGuiRule &existing : m_rules) {
        if (existing.target_key != gui_rule.target_key ||
            existing.target_index != gui_rule.target_index ||
            existing.scope != gui_rule.scope ||
            existing.owner != gui_rule.owner ||
            existing.clauses.size() != gui_rule.clauses.size())
            continue;

        bool identical = true;
        for (size_t clause_idx = 0; clause_idx < gui_rule.clauses.size() && identical; ++clause_idx) {
            const PluginGuiRuleClause &lhs = existing.clauses[clause_idx];
            const PluginGuiRuleClause &rhs = gui_rule.clauses[clause_idx];
            identical = lhs.condition == rhs.condition &&
                lhs.condition_key == rhs.condition_key &&
                lhs.condition_index == rhs.condition_index &&
                lhs.condition_float_value == rhs.condition_float_value;
        }
        if (identical)
            return GuiRuleRegistration::Duplicate;
    }

    m_rules.emplace_back(std::move(gui_rule));
    invalidate_explanations();
    const size_t rule_index = m_rules.size() - 1;
    if (!this->attach_rule_to_setting(m_rules[rule_index].target_key, rule_index))
        m_pending_rules[m_rules[rule_index].target_key].push_back(rule_index);
    return GuiRuleRegistration::Added;
}

const GuiRuleRegistry::SettingActivation *GuiRuleRegistry::setting_activation_for(const std::string &setting_key) const
{
    const ConfigOptionDef *option_def = PrintConfigDef::instance().get(setting_key);
    if (option_def == nullptr || option_def->gui_activation_id == 0 ||
        option_def->gui_activation_id > m_setting_activations.size())
        return nullptr;
    return &m_setting_activations[option_def->gui_activation_id - 1];
}

size_t GuiRuleRegistry::warn_pending_rules() const
{
    size_t reported = 0;
    for (const std::pair<const std::string, std::vector<size_t>> &entry : m_pending_rules) {
        if (entry.second.empty())
            continue;
        ++reported;
        BOOST_LOG_TRIVIAL(warning)
            << "GUI activation rule targets undefined setting '" << entry.first
            << "' (" << entry.second.size() << " rule(s) queued).";
    }
    return reported;
}

std::vector<GuiRuleRegistry::Application> GuiRuleRegistry::evaluate(
    const DynamicPrintConfig &config,
    raw_option_preset_type scope,
    int32_t current_index,
    const std::string &owner_filter,
    const std::vector<OptionKeyIdx> &setting_filter) const
{
    // A refresh follows every setting change: drop the cached explanations so
    // a hover cannot show a stale reason.
    invalidate_explanations();

    GuiRuleEvaluationContext context(config, scope, current_index, owner_filter);

    if (setting_filter.empty()) {
        // Full scan: used by tests, tools and the fallback when the caller has
        // no field list (page not built yet).
        for (const PluginGuiRule &rule : m_rules)
            context.evaluate_rule(rule);
    } else {
        // Per-page scan: visit only the rules attached to the displayed
        // settings. Rules whose setting has no definition (still queued) are
        // only reachable through the full scan.
        //
        // current_index >= 0 is the per-extruder pass: only settings with a
        // widget at that index can receive a RAW_GUI_RULE_INDEX_CURRENT rule.
        std::unordered_set<std::string_view> visited_keys;
        for (const OptionKeyIdx &field : setting_filter) {
            if (current_index >= 0 && field.idx != current_index)
                continue;
            if (!visited_keys.insert(std::string_view(field.key)).second)
                continue;

            const SettingActivation *activation = this->setting_activation_for(field.key);
            if (activation == nullptr)
                continue;
            for (size_t rule_index : activation->rule_indices)
                if (rule_index < m_rules.size())
                    context.evaluate_rule(m_rules[rule_index]);
        }
    }

    std::vector<GuiRuleApplication> out;
    out.reserve(context.enabled_by_target.size());
    for (const auto &entry : context.enabled_by_target) {
        GuiRuleApplication application;
        application.target_key = entry.first.first;
        application.target_index = entry.first.second;
        application.enabled = entry.second;
        out.emplace_back(std::move(application));
    }

    return out;
}

std::string GuiRuleRegistry::explain_disabled(const DynamicPrintConfig &config,
                                              const OptionKeyIdx &target,
                                              raw_option_preset_type scope,
                                              ExplanationStyle style) const
{
    const ExplanationCacheKey cache_key{style, target.key, target.idx};
    const auto cached = m_explanation_cache.find(cache_key);
    if (cached != m_explanation_cache.end())
        return cached->second;

    std::string explanation;
    const ConfigOption *target_option = config.optptr(target.key);
    if (target_option != nullptr) {
        std::vector<std::string> lines;
        bool any_rule = false;
        bool rule_satisfied = false;

        for (const PluginGuiRule &rule : m_rules) {
            if (rule.target_key != target.key)
                continue;
            if (!gui_rule_matches_scope(rule.scope, scope))
                continue;
            if (rule.clauses.empty())
                continue;

            int32_t evaluation_index = -1;
            if (!gui_rule_applies_to_widget(rule, *target_option, target.idx, evaluation_index))
                continue;

            // Same applicability rule as the evaluator: a rule only
            // participates when its target and every clause option exist.
            bool applicable = true;
            for (const Clause &clause : rule.clauses)
                if (clause.condition_key.empty() || config.optptr(clause.condition_key) == nullptr) {
                    applicable = false;
                    break;
                }
            if (!applicable)
                continue;

            any_rule = true;

            std::vector<std::pair<const Clause *, bool>> clause_states;
            clause_states.reserve(rule.clauses.size());
            bool rule_true = true;
            bool invalid = false;
            for (const Clause &clause : rule.clauses) {
                bool value = false;
                try {
                    const ConfigOption *condition_option = config.optptr(clause.condition_key);
                    value = condition_option != nullptr &&
                            gui_rule_condition_value(*condition_option, clause, evaluation_index);
                } catch (...) {
                    // Malformed plugin input must not crash the tooltip.
                    invalid = true;
                    break;
                }
                clause_states.emplace_back(&clause, value);
                if (!value)
                    rule_true = false;
            }
            if (invalid)
                continue;
            if (rule_true) {
                // The widget should be enabled: there is nothing to explain.
                rule_satisfied = true;
                break;
            }

            std::string line;
            for (size_t i = 0; i < clause_states.size(); ++i) {
                if (i > 0)
                    line += " " + gui_rule_text(GuiRuleText::AND) + " ";
                line += gui_rule_clause_text(*clause_states[i].first, evaluation_index, style);
                line += " ";
                line += gui_rule_text(clause_states[i].second ? GuiRuleText::SATISFIED : GuiRuleText::UNMET);
            }
            lines.emplace_back(std::move(line));
        }

        if (any_rule && !rule_satisfied && !lines.empty()) {
            explanation = gui_rule_text(GuiRuleText::TO_ENABLE);
            for (const std::string &line : lines) {
                explanation += "\n";
                explanation += "\xE2\x80\xA2 "; // UTF-8 bullet
                explanation += line;
            }
        }
    }

    m_explanation_cache.emplace(cache_key, explanation);
    return explanation;
}

void GuiRuleRegistry::invalidate_explanations() const
{
    m_explanation_cache.clear();
}

// Attach the rules queued for one setting once its definition exists.
bool GuiRuleRegistry::attach_pending_for_setting(const std::string &setting_key)
{
    const std::map<std::string, std::vector<size_t>>::iterator pending = m_pending_rules.find(setting_key);
    if (pending == m_pending_rules.end())
        return false;

    invalidate_explanations();

    std::vector<size_t> queued;
    queued.swap(pending->second);
    m_pending_rules.erase(pending);

    bool attached = false;
    for (size_t rule_index : queued)
        attached = this->attach_rule_to_setting(setting_key, rule_index) || attached;
    return attached;
}

} // namespace Slic3r
