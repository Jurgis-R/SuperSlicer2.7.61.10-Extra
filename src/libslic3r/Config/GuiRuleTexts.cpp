///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "GuiRuleTexts.hpp"

#include <array>
#include <cstddef>
#include <utility>

namespace Slic3r {

namespace {

constexpr std::array<const char *, size_t(GuiRuleText::COUNT)> gui_rule_base_text = {
    "To enable this setting:",
    "and",
    "(satisfied)",
    "(unmet)",
    "is true",
    "is false",
    "is enabled",
    "is disabled",
    "is not zero",
    "=",
    "!=",
    "<",
    ">=",
    "is a percentage",
    "is empty",
    "current",
};

std::array<std::string, size_t(GuiRuleText::COUNT)> gui_rule_texts = []() {
    std::array<std::string, size_t(GuiRuleText::COUNT)> text;
    for (size_t i = 0; i < text.size(); ++i)
        text[i] = gui_rule_base_text[i];
    return text;
}();

} // namespace

size_t update_gui_rule_texts(const std::function<std::string(const std::string &text)> &translate)
{
    size_t changed = 0;
    for (size_t i = 0; i < gui_rule_texts.size(); ++i) {
        std::string translated = translate ? translate(gui_rule_base_text[i]) : std::string();
        if (translated.empty())
            translated = gui_rule_base_text[i];
        if (gui_rule_texts[i] != translated) {
            gui_rule_texts[i] = std::move(translated);
            ++changed;
        }
    }
    return changed;
}

const std::string &gui_rule_text(GuiRuleText id)
{
    return gui_rule_texts[size_t(id)];
}

} // namespace Slic3r
