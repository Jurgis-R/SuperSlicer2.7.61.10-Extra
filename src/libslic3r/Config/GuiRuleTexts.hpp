///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Config_GuiRuleTexts_hpp_
#define slic3r_Config_GuiRuleTexts_hpp_

#include <functional>
#include <string>

namespace Slic3r {

// Fixed vocabulary used to build the "why is this setting disabled" text from
// the GUI activation rules. The strings are translated as a block: the GUI
// calls update_gui_rule_texts() after loading a language, which resets every
// entry from the English base and applies the translator, so the call is safe
// to repeat after a language change.
enum class GuiRuleText {
    TO_ENABLE,
    AND,
    SATISFIED,
    UNMET,
    IS_TRUE,
    IS_FALSE,
    IS_ENABLED,
    IS_DISABLED,
    NON_ZERO,
    EQUALS,
    NOT_EQUALS,
    LESS_THAN,
    GREATER_OR_EQUAL,
    IS_PERCENT,
    IS_EMPTY,
    CURRENT_INDEX,
    COUNT
};

// Apply a translator to the whole vocabulary. The English base text is always
// the input, so calling this again after a language change is safe. An empty
// translation keeps the English base. Returns the number of entries that
// changed.
size_t update_gui_rule_texts(const std::function<std::string(const std::string &text)> &translate);

// Return the current text of one entry.
const std::string &gui_rule_text(GuiRuleText id);

} // namespace Slic3r

#endif // slic3r_Config_GuiRuleTexts_hpp_
