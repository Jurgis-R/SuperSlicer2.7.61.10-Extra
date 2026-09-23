///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Config_GuiRuleRegistry_hpp_
#define slic3r_Config_GuiRuleRegistry_hpp_

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "ConfigOption.hpp"

namespace Slic3r {

// Owner tag used by the built-in GUI activation rules and by any host caller
// that does not run inside a plugin initialization.
inline constexpr const char *CORE_GUI_RULE_OWNER = "core";

class DynamicPrintConfig;

// Host-side registry of per-setting GUI activation rules.
//
// It owns the stored rules, the per-setting activation groups, the queue of
// rules whose setting definition does not exist yet, and the evaluation of a
// config snapshot into per-target enabled states. Orchestrator owns one
// instance and exposes it through Orchestrator::gui_rules().
class GuiRuleRegistry
{
public:
    struct PluginGuiRuleClause
    {
        raw_gui_rule_condition condition = RAW_GUI_RULE_CONDITION_NONE;
        std::string condition_key;
        int32_t condition_index = RAW_GUI_RULE_INDEX_ALL;
        // Numeric operand: llround() for INT_EQUALS/INT_NOT_EQUALS, normalized
        // float for FLOAT_* conditions, ignored by IS_PERCENT/VECTOR_EMPTY.
        double condition_float_value = 0.0;
        // Serialized operand for STRING_EQUALS; for a vector option it is
        // compared against the item resolved from condition_index.
        std::string condition_string_value;
    };

    struct PluginGuiRule
    {
        // One activation reason for target_key: the clauses are ANDed and the
        // target is enabled when at least one of its rules is true.
        std::string target_key;
        int32_t target_index = RAW_GUI_RULE_INDEX_ALL;
        raw_option_preset_type scope = RAW_PRESET_TYPE_NONE;
        // Host-only origin of the rule: "core" for built-in settings, the
        // plugin id otherwise. Used for diagnostics, deduplication and tests.
        std::string owner;
        std::vector<PluginGuiRuleClause> clauses;
    };

    // All activation rules attached to one setting, in registration order.
    struct SettingActivation
    {
        std::string setting_key;
        std::vector<size_t> rule_indices;
    };

    struct GuiRuleApplication
    {
        std::string target_key;
        int32_t target_index = RAW_GUI_RULE_INDEX_ALL;
        bool enabled = true;
    };

    // Result of add(). Values mirror the public C result codes.
    enum class GuiRuleRegistration : int32_t
    {
        Invalid = -1,
        Duplicate = 0,
        Added = 1
    };

    // Short aliases used by the implementation and by Orchestrator.
    using Clause = PluginGuiRuleClause;
    using Rule = PluginGuiRule;
    using Application = GuiRuleApplication;
    using Registration = GuiRuleRegistration;

    // How the explanation text names the options: raw keys, or the translated
    // option labels the GUI stored in the definitions.
    enum class ExplanationStyle : uint8_t
    {
        RawKeys,
        TranslatedLabels
    };

    // Register a GUI activation rule (conjunctive clauses and scope).
    // The complete descriptor is validated before it is retained: unknown
    // condition or scope values, invalid indexes, missing keys and non-finite
    // float compares are rejected with GuiRuleRegistration::Invalid and a
    // warning log instead of being stored and silently ignored later.
    //
    // owner is the host-only origin of the rule. When null, "core" is used.
    GuiRuleRegistration add(const raw_gui_rule *rule, const char *owner = nullptr);
    // Return GUI rules by borrowed reference.
    const std::vector<PluginGuiRule> &rules() const { return m_rules; }
    // Return the per-setting activation groups by borrowed reference.
    const std::vector<SettingActivation> &setting_activations() const { return m_setting_activations; }
    // Return the activation group of one setting, or nullptr when it has none.
    const SettingActivation *setting_activation_for(const std::string &setting_key) const;
    // Attach the rules queued for one setting once its definition exists.
    // Returns true when at least one queued rule was attached.
    bool attach_pending_for_setting(const std::string &setting_key);
    // Log one non-blocking warning per setting that still has queued rules.
    // Returns the number of settings reported.
    size_t warn_pending_rules() const;
    // Resolve the enabled state of every GUI target for one config snapshot.
    //
    // scope selects the refreshed GUI context: rules whose scope is neither
    // RAW_PRESET_TYPE_NONE nor the requested scope are ignored. The target is
    // enabled when at least one applicable rule is true. owner_filter restricts
    // the rules to one origin; an empty filter evaluates every owner.
    //
    // setting_filter is the field list of the refreshed page. When non-empty,
    // only the rules attached to those settings are evaluated (and, in a
    // per-extruder pass, only fields of the current index). An empty list keeps
    // the full scan used by tests, tools and the first refresh before the page
    // is built.
    std::vector<GuiRuleApplication> evaluate(const DynamicPrintConfig &config,
                                             raw_option_preset_type scope,
                                             int32_t current_index = -1,
                                             const std::string &owner_filter = std::string(),
                                             const std::vector<OptionKeyIdx> &setting_filter = std::vector<OptionKeyIdx>()) const;

    // Build the text explaining why one setting widget is disabled: every rule
    // registered for the target that applies to this item is listed with the
    // state of its clauses. Returns an empty string when no rule targets the
    // key, when a rule is satisfied (the widget should be enabled) or when the
    // rules are not applicable to this config, so callers keep their normal
    // tooltip.
    //
    // target.idx selects the explained item: -1 explains the whole field (only
    // rules applied by the global pass), a real index explains that item (the
    // rules applied by the per-extruder pass, plus explicit-index rules).
    std::string explain_disabled(const DynamicPrintConfig &config,
                                 const OptionKeyIdx &target,
                                 raw_option_preset_type scope,
                                 ExplanationStyle style = ExplanationStyle::RawKeys) const;
    // Drop the cached explanations. Called on every registration and at the
    // beginning of every evaluation; the GUI also calls it after refreshing the
    // translated option labels.
    void invalidate_explanations() const;

private:
    // Attach one stored rule to its setting activation group. Returns false
    // when the setting definition does not exist yet.
    bool attach_rule_to_setting(const std::string &setting_key, size_t rule_index);

    struct ExplanationCacheKey
    {
        ExplanationStyle style;
        std::string target_key;
        int32_t target_index;

        bool operator<(const ExplanationCacheKey &rhs) const
        {
            if (style != rhs.style)
                return style < rhs.style;
            if (target_key != rhs.target_key)
                return target_key < rhs.target_key;
            return target_index < rhs.target_index;
        }
    };

    std::vector<PluginGuiRule> m_rules;
    std::vector<SettingActivation> m_setting_activations;
    // Indices into m_rules for settings whose ConfigOptionDef does not exist
    // yet. They are attached automatically when the definition appears.
    std::map<std::string, std::vector<size_t>> m_pending_rules;
    // Explanation text per (style, target), rebuilt on demand.
    mutable std::map<ExplanationCacheKey, std::string> m_explanation_cache;
};

} // namespace Slic3r

#endif // slic3r_Config_GuiRuleRegistry_hpp_
