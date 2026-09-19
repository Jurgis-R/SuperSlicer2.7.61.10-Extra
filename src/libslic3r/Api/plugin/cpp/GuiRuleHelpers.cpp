///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "libslic3r/Api/plugin/cpp/GuiRuleHelpers.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"

namespace slic3r_api {

void GuiRule::add_to(Slic3r::Orchestrator &orchestrator) const
{
    const std::vector<std::vector<GuiCondition::GuiClause>> disjuncts = normalized_disjuncts();
    for (const std::string &target : m_targets) {
        for (const std::vector<GuiCondition::GuiClause> &disjunct : disjuncts) {
            const std::vector<raw_gui_rule_clause> clauses = make_raw_clauses(disjunct);
            raw_gui_rule rule = raw_gui_rule_init();
            rule.action = m_action;
            rule.target_key = target.c_str();
            rule.target_index = m_target_index;
            rule.scope = m_scope;
            rule.rule_group = m_group.empty() ? nullptr : m_group.c_str();
            rule.clauses = clauses.data();
            rule.clause_count = uint32_t(clauses.size());

            const Slic3r::Orchestrator::GuiRuleRegistration result = orchestrator.add_gui_rule(&rule);
            if (result == Slic3r::Orchestrator::GuiRuleRegistration::Invalid)
                throw std::invalid_argument("The host rejected the GUI rule for target '" + target + "'.");
        }
    }
}

} // namespace slic3r_api
