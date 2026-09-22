///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_plugin_cpp_GuiRuleHelpers_hpp_
#define slic3r_Api_plugin_cpp_GuiRuleHelpers_hpp_

#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"

/*
GuiRuleHelpers.hpp
==================

Owning builders for the GUI activation rules registered through
orchestrator_add_gui_rule().

A GuiCondition is an expression tree: a leaf predicate on one config option, or
an any_of/all_of combination of sub-expressions. A GuiRule owns one or more
target keys and one root condition. add_to() normalizes the tree to
disjunctive normal form (an OR of ANDs) and registers one raw rule per
disjunction and target. Each raw rule is an activation reason: the target is
enabled when at least one applicable rule is true, so a disjunction naturally
expands to several reasons. This is the usual form for "support is active when
support_material is true or raft_layers is non zero".

Builder input is validated eagerly: an empty key, an invalid target or
condition index, a non-finite numeric operand or an empty any_of/all_of throws
std::invalid_argument before the host is called. add_to() maps the C result
codes to exceptions: an invalid descriptor throws std::invalid_argument, an
internal host failure throws std::runtime_error, and a duplicate registration
is a silent no-op exactly like the C return code 0.

The class owns every string and value referenced by the raw descriptor, so a
GuiRule may be stored and registered later; only add_to() builds the short-lived
raw view. Host-side code passes an orchestrator_handle obtained from
Orchestrator::handle(); plugins pass the handle received at registration.
*/

namespace slic3r_api {

class GuiCondition;

GuiCondition any_of(std::initializer_list<GuiCondition> conditions);
GuiCondition any_of(std::vector<GuiCondition> conditions);
GuiCondition all_of(std::initializer_list<GuiCondition> conditions);
GuiCondition all_of(std::vector<GuiCondition> conditions);

class GuiCondition
{
public:
    enum class Kind
    {
        Invalid,
        Leaf,
        AllOf,
        AnyOf
    };

    GuiCondition() = default;

    bool valid() const { return m_kind != Kind::Invalid; }
    Kind kind() const { return m_kind; }

    // Leaf predicates. condition_index defaults to RAW_GUI_RULE_INDEX_ALL.
    static GuiCondition true_(std::string key, int32_t index = RAW_GUI_RULE_INDEX_ALL)
        { return leaf(RAW_GUI_RULE_CONDITION_BOOL_TRUE, std::move(key), index, 0.0); }
    static GuiCondition false_(std::string key, int32_t index = RAW_GUI_RULE_INDEX_ALL)
        { return leaf(RAW_GUI_RULE_CONDITION_BOOL_FALSE, std::move(key), index, 0.0); }
    static GuiCondition non_zero(std::string key, int32_t index = RAW_GUI_RULE_INDEX_ALL)
        { return leaf(RAW_GUI_RULE_CONDITION_VALUE_NON_ZERO, std::move(key), index, 0.0); }
    static GuiCondition enabled(std::string key, int32_t index = RAW_GUI_RULE_INDEX_ALL)
        { return leaf(RAW_GUI_RULE_CONDITION_OPTION_ENABLED, std::move(key), index, 0.0); }
    static GuiCondition disabled(std::string key, int32_t index = RAW_GUI_RULE_INDEX_ALL)
        { return leaf(RAW_GUI_RULE_CONDITION_OPTION_DISABLED, std::move(key), index, 0.0); }
    static GuiCondition int_equals(std::string key, int32_t value, int32_t index = RAW_GUI_RULE_INDEX_ALL)
        { return leaf(RAW_GUI_RULE_CONDITION_INT_EQUALS, std::move(key), index, double(value)); }
    static GuiCondition int_not_equals(std::string key, int32_t value, int32_t index = RAW_GUI_RULE_INDEX_ALL)
        { return leaf(RAW_GUI_RULE_CONDITION_INT_NOT_EQUALS, std::move(key), index, double(value)); }
    static GuiCondition float_equals(std::string key, double value, int32_t index = RAW_GUI_RULE_INDEX_ALL)
        { return leaf(RAW_GUI_RULE_CONDITION_FLOAT_EQUALS, std::move(key), index, value); }
    static GuiCondition float_less_than(std::string key, double value, int32_t index = RAW_GUI_RULE_INDEX_ALL)
        { return leaf(RAW_GUI_RULE_CONDITION_FLOAT_LESS_THAN, std::move(key), index, value); }
    static GuiCondition float_greater_or_equal(std::string key, double value, int32_t index = RAW_GUI_RULE_INDEX_ALL)
        { return leaf(RAW_GUI_RULE_CONDITION_FLOAT_GREATER_OR_EQUAL, std::move(key), index, value); }
    static GuiCondition percent(std::string key, int32_t index = RAW_GUI_RULE_INDEX_ALL)
        { return leaf(RAW_GUI_RULE_CONDITION_IS_PERCENT, std::move(key), index, 0.0); }
    static GuiCondition vector_empty(std::string key, int32_t index = RAW_GUI_RULE_INDEX_ALL)
        { return leaf(RAW_GUI_RULE_CONDITION_VECTOR_EMPTY, std::move(key), index, 0.0); }
    static GuiCondition string_equals(std::string key, std::string value, int32_t index = RAW_GUI_RULE_INDEX_ALL)
    {
        if (value.empty())
            throw std::invalid_argument("A string condition needs a nonempty expected value.");
        GuiCondition out = leaf(RAW_GUI_RULE_CONDITION_STRING_EQUALS, std::move(key), index, 0.0);
        out.m_clause.condition_string_value = std::move(value);
        return out;
    }

private:
    friend class GuiRule;
    friend GuiCondition any_of(std::initializer_list<GuiCondition> conditions);
    friend GuiCondition any_of(std::vector<GuiCondition> conditions);
    friend GuiCondition all_of(std::initializer_list<GuiCondition> conditions);
    friend GuiCondition all_of(std::vector<GuiCondition> conditions);

    struct GuiClause
    {
        raw_gui_rule_condition condition = RAW_GUI_RULE_CONDITION_NONE;
        std::string condition_key;
        int32_t condition_index = RAW_GUI_RULE_INDEX_ALL;
        double condition_float_value = 0.0;
        std::string condition_string_value;
    };

    static void validate_index(int32_t index)
    {
        if (index < 0 && index != RAW_GUI_RULE_INDEX_ALL && index != RAW_GUI_RULE_INDEX_CURRENT)
            throw std::invalid_argument("A GUI condition index must be nonnegative, INDEX_ALL or INDEX_CURRENT.");
    }

    static GuiCondition leaf(raw_gui_rule_condition condition, std::string key, int32_t index, double value)
    {
        if (key.empty())
            throw std::invalid_argument("A GUI condition needs a nonempty config key.");
        validate_index(index);
        if (!std::isfinite(value))
            throw std::invalid_argument("A GUI condition needs a finite numeric operand.");

        GuiCondition out;
        out.m_kind = Kind::Leaf;
        out.m_clause.condition = condition;
        out.m_clause.condition_key = std::move(key);
        out.m_clause.condition_index = index;
        out.m_clause.condition_float_value = value;
        return out;
    }

    static GuiCondition make(Kind kind, std::vector<GuiCondition> children)
    {
        if (children.empty())
            throw std::invalid_argument("A GUI condition combinator needs at least one child.");
        for (const GuiCondition &child : children)
            if (!child.valid())
                throw std::invalid_argument("A GUI condition combinator cannot contain an empty condition.");

        GuiCondition out;
        out.m_kind = kind;
        out.m_children = std::move(children);
        return out;
    }

    Kind m_kind = Kind::Invalid;
    GuiClause m_clause;
    std::vector<GuiCondition> m_children;
};

inline GuiCondition any_of(std::initializer_list<GuiCondition> conditions)
{
    return GuiCondition::make(GuiCondition::Kind::AnyOf, std::vector<GuiCondition>(conditions));
}

inline GuiCondition any_of(std::vector<GuiCondition> conditions)
{
    return GuiCondition::make(GuiCondition::Kind::AnyOf, std::move(conditions));
}

inline GuiCondition all_of(std::initializer_list<GuiCondition> conditions)
{
    return GuiCondition::make(GuiCondition::Kind::AllOf, std::vector<GuiCondition>(conditions));
}

inline GuiCondition all_of(std::vector<GuiCondition> conditions)
{
    return GuiCondition::make(GuiCondition::Kind::AllOf, std::move(conditions));
}

// Free leaf factories. They forward to GuiCondition and may be imported
// directly by name, for example with "using slic3r_api::non_zero;", which keeps
// rule expressions readable without the class qualifier.
inline GuiCondition true_(std::string key, int32_t index = RAW_GUI_RULE_INDEX_ALL)
    { return GuiCondition::true_(std::move(key), index); }
inline GuiCondition false_(std::string key, int32_t index = RAW_GUI_RULE_INDEX_ALL)
    { return GuiCondition::false_(std::move(key), index); }
inline GuiCondition non_zero(std::string key, int32_t index = RAW_GUI_RULE_INDEX_ALL)
    { return GuiCondition::non_zero(std::move(key), index); }
inline GuiCondition enabled(std::string key, int32_t index = RAW_GUI_RULE_INDEX_ALL)
    { return GuiCondition::enabled(std::move(key), index); }
inline GuiCondition disabled(std::string key, int32_t index = RAW_GUI_RULE_INDEX_ALL)
    { return GuiCondition::disabled(std::move(key), index); }
inline GuiCondition int_equals(std::string key, int32_t value, int32_t index = RAW_GUI_RULE_INDEX_ALL)
    { return GuiCondition::int_equals(std::move(key), value, index); }
inline GuiCondition int_not_equals(std::string key, int32_t value, int32_t index = RAW_GUI_RULE_INDEX_ALL)
    { return GuiCondition::int_not_equals(std::move(key), value, index); }
inline GuiCondition float_equals(std::string key, double value, int32_t index = RAW_GUI_RULE_INDEX_ALL)
    { return GuiCondition::float_equals(std::move(key), value, index); }
inline GuiCondition float_less_than(std::string key, double value, int32_t index = RAW_GUI_RULE_INDEX_ALL)
    { return GuiCondition::float_less_than(std::move(key), value, index); }
inline GuiCondition float_greater_or_equal(std::string key, double value, int32_t index = RAW_GUI_RULE_INDEX_ALL)
    { return GuiCondition::float_greater_or_equal(std::move(key), value, index); }
inline GuiCondition percent(std::string key, int32_t index = RAW_GUI_RULE_INDEX_ALL)
    { return GuiCondition::percent(std::move(key), index); }
inline GuiCondition vector_empty(std::string key, int32_t index = RAW_GUI_RULE_INDEX_ALL)
    { return GuiCondition::vector_empty(std::move(key), index); }
inline GuiCondition string_equals(std::string key, std::string value, int32_t index = RAW_GUI_RULE_INDEX_ALL)
    { return GuiCondition::string_equals(std::move(key), std::move(value), index); }

class GuiRule
{
public:
    // Register one activation reason for every target under the given
    // condition. A disjunction in the condition becomes several reasons.
    static GuiRule activate(std::string target) { return GuiRule(targets_from(std::move(target))); }
    static GuiRule activate(std::vector<std::string> targets) { return GuiRule(validated_targets(std::move(targets))); }
    static GuiRule activate(std::initializer_list<std::string> targets) { return GuiRule(validated_targets(std::vector<std::string>(targets))); }

    GuiRule &when(GuiCondition condition)
    {
        if (!condition.valid())
            throw std::invalid_argument("A GUI rule condition must be built with a GuiCondition factory.");
        m_conditions.emplace_back(std::move(condition));
        return *this;
    }

    GuiRule &when(std::initializer_list<GuiCondition> conditions)
    {
        if (conditions.size() == 0)
            throw std::invalid_argument("A GUI rule needs at least one condition per when() call.");
        for (const GuiCondition &condition : conditions)
            when(condition);
        return *this;
    }

    GuiRule &when_true(std::string key, int32_t index = RAW_GUI_RULE_INDEX_ALL)
        { return when(GuiCondition::true_(std::move(key), index)); }
    GuiRule &when_false(std::string key, int32_t index = RAW_GUI_RULE_INDEX_ALL)
        { return when(GuiCondition::false_(std::move(key), index)); }
    GuiRule &when_non_zero(std::string key, int32_t index = RAW_GUI_RULE_INDEX_ALL)
        { return when(GuiCondition::non_zero(std::move(key), index)); }
    GuiRule &when_enabled(std::string key, int32_t index = RAW_GUI_RULE_INDEX_ALL)
        { return when(GuiCondition::enabled(std::move(key), index)); }
    GuiRule &when_disabled(std::string key, int32_t index = RAW_GUI_RULE_INDEX_ALL)
        { return when(GuiCondition::disabled(std::move(key), index)); }
    GuiRule &when_int_equals(std::string key, int32_t value, int32_t index = RAW_GUI_RULE_INDEX_ALL)
        { return when(GuiCondition::int_equals(std::move(key), value, index)); }
    GuiRule &when_int_not_equals(std::string key, int32_t value, int32_t index = RAW_GUI_RULE_INDEX_ALL)
        { return when(GuiCondition::int_not_equals(std::move(key), value, index)); }
    GuiRule &when_float_equals(std::string key, double value, int32_t index = RAW_GUI_RULE_INDEX_ALL)
        { return when(GuiCondition::float_equals(std::move(key), value, index)); }
    GuiRule &when_float_less_than(std::string key, double value, int32_t index = RAW_GUI_RULE_INDEX_ALL)
        { return when(GuiCondition::float_less_than(std::move(key), value, index)); }
    GuiRule &when_float_greater_or_equal(std::string key, double value, int32_t index = RAW_GUI_RULE_INDEX_ALL)
        { return when(GuiCondition::float_greater_or_equal(std::move(key), value, index)); }
    GuiRule &when_percent(std::string key, int32_t index = RAW_GUI_RULE_INDEX_ALL)
        { return when(GuiCondition::percent(std::move(key), index)); }
    GuiRule &when_vector_empty(std::string key, int32_t index = RAW_GUI_RULE_INDEX_ALL)
        { return when(GuiCondition::vector_empty(std::move(key), index)); }
    GuiRule &when_string_equals(std::string key, std::string value, int32_t index = RAW_GUI_RULE_INDEX_ALL)
        { return when(GuiCondition::string_equals(std::move(key), std::move(value), index)); }

    GuiRule &target_index(int32_t index)
    {
        GuiCondition::validate_index(index);
        m_target_index = index;
        return *this;
    }

    GuiRule &scope(raw_option_preset_type scope)
    {
        if (scope < RAW_PRESET_TYPE_NONE || scope >= RAW_PRESET_TYPE_COUNT)
            throw std::invalid_argument("A GUI rule scope must be a declared raw_option_preset_type value.");
        m_scope = scope;
        return *this;
    }

    // Register every disjunct on every target. Throws on invalid descriptors.
    void add_to(orchestrator_handle *orchestrator) const
    {
        if (orchestrator == nullptr)
            throw std::invalid_argument("A GUI rule cannot be registered on a null orchestrator.");

        const std::vector<std::vector<GuiCondition::GuiClause>> disjuncts = normalized_disjuncts();
        for (const std::string &target : m_targets) {
            for (const std::vector<GuiCondition::GuiClause> &disjunct : disjuncts) {
                const std::vector<raw_gui_rule_clause> clauses = make_raw_clauses(disjunct);
                raw_gui_rule rule = raw_gui_rule_init();
                rule.target_key = target.c_str();
                rule.target_index = m_target_index;
                rule.scope = m_scope;
                rule.clauses = clauses.data();
                rule.clause_count = uint32_t(clauses.size());

                const int32_t result = orchestrator_add_gui_rule(orchestrator, &rule);
                if (result == -1)
                    throw std::invalid_argument("The host rejected the GUI rule for target '" + target + "'.");
                if (result < 0)
                    throw std::runtime_error("The host failed to register the GUI rule for target '" + target + "'.");
            }
        }
    }

private:
    explicit GuiRule(std::vector<std::string> targets) :
        m_targets(std::move(targets))
    {
    }

    static std::vector<std::string> targets_from(std::string target)
    {
        return validated_targets({ std::move(target) });
    }

    static std::vector<std::string> validated_targets(std::vector<std::string> targets)
    {
        if (targets.empty())
            throw std::invalid_argument("A GUI rule needs at least one target key.");
        for (const std::string &target : targets)
            if (target.empty())
                throw std::invalid_argument("A GUI rule target key must not be empty.");
        return targets;
    }

    GuiCondition combined_condition() const
    {
        if (m_conditions.empty())
            throw std::invalid_argument("A GUI rule needs at least one condition before it can be registered.");
        if (m_conditions.size() == 1)
            return m_conditions.front();
        return all_of(m_conditions);
    }

    void append_disjuncts(const GuiCondition &condition,
                          std::vector<std::vector<GuiCondition::GuiClause>> &out) const
    {
        switch (condition.m_kind) {
        case GuiCondition::Kind::Leaf:
            out.push_back({ condition.m_clause });
            break;
        case GuiCondition::Kind::AnyOf:
            for (const GuiCondition &child : condition.m_children)
                append_disjuncts(child, out);
            break;
        case GuiCondition::Kind::AllOf: {
            std::vector<std::vector<GuiCondition::GuiClause>> product = { {} };
            for (const GuiCondition &child : condition.m_children) {
                std::vector<std::vector<GuiCondition::GuiClause>> child_disjuncts;
                append_disjuncts(child, child_disjuncts);

                std::vector<std::vector<GuiCondition::GuiClause>> merged_product;
                for (const std::vector<GuiCondition::GuiClause> &prefix : product) {
                    for (const std::vector<GuiCondition::GuiClause> &part : child_disjuncts) {
                        std::vector<GuiCondition::GuiClause> merged = prefix;
                        merged.insert(merged.end(), part.begin(), part.end());
                        merged_product.emplace_back(std::move(merged));
                    }
                }
                product = std::move(merged_product);
            }
            out.insert(out.end(), product.begin(), product.end());
            break;
        }
        case GuiCondition::Kind::Invalid:
        default:
            throw std::invalid_argument("A GUI rule contains an empty condition.");
        }
    }

    std::vector<std::vector<GuiCondition::GuiClause>> normalized_disjuncts() const
    {
        std::vector<std::vector<GuiCondition::GuiClause>> out;
        append_disjuncts(combined_condition(), out);

        if (out.empty())
            throw std::invalid_argument("A GUI rule needs at least one condition.");
        return out;
    }

    std::vector<raw_gui_rule_clause> make_raw_clauses(
        const std::vector<GuiCondition::GuiClause> &disjunct) const
    {
        std::vector<raw_gui_rule_clause> clauses;
        clauses.reserve(disjunct.size());
        for (const GuiCondition::GuiClause &clause : disjunct) {
            raw_gui_rule_clause raw = raw_gui_rule_clause_init();
            raw.condition = clause.condition;
            raw.condition_key = clause.condition_key.c_str();
            raw.condition_index = clause.condition_index;
            raw.condition_float_value = clause.condition_float_value;
            raw.condition_string_value = clause.condition_string_value.empty() ?
                nullptr : clause.condition_string_value.c_str();
            clauses.push_back(raw);
        }
        return clauses;
    }

    std::vector<std::string> m_targets;
    int32_t m_target_index = RAW_GUI_RULE_INDEX_ALL;
    raw_option_preset_type m_scope = RAW_PRESET_TYPE_NONE;
    std::vector<GuiCondition> m_conditions;
};

inline void add_gui_rule(orchestrator_handle *orchestrator, const GuiRule &rule)
{
    rule.add_to(orchestrator);
}

inline void add_gui_rules(orchestrator_handle *orchestrator, std::initializer_list<GuiRule> rules)
{
    for (const GuiRule &rule : rules)
        rule.add_to(orchestrator);
}

} // namespace slic3r_api

#endif // slic3r_Api_plugin_cpp_GuiRuleHelpers_hpp_
