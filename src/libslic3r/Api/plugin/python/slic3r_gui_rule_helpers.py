#/|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
#/|/
#/|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#/|/

"""Owning GUI activation rule builders for Python plugins.

``GuiCondition`` and ``GuiRule`` mirror ``Api/plugin/cpp/GuiRuleHelpers.hpp``.
A condition is a leaf predicate or an ``any_of``/``all_of`` combination of
sub-conditions; a rule owns one or more target keys and one root condition.
``add_to()`` normalizes the tree to disjunctive normal form and calls
``Slic3rAPI.add_gui_rule`` once per disjunction and target. Every registered
rule is one activation reason: the target is enabled when at least one rule is
true, so a disjunction naturally expands to several reasons.

Builder input is validated eagerly with ``ValueError``; the host rejection of a
descriptor is also raised as ``ValueError`` while a duplicate registration is a
silent no-op.
"""

from __future__ import annotations

from collections.abc import Iterable

from slic3r_api_generated import (
    RAW_GUI_RULE_CONDITION_BOOL_FALSE,
    RAW_GUI_RULE_CONDITION_BOOL_TRUE,
    RAW_GUI_RULE_CONDITION_FLOAT_EQUALS,
    RAW_GUI_RULE_CONDITION_FLOAT_GREATER_OR_EQUAL,
    RAW_GUI_RULE_CONDITION_FLOAT_LESS_THAN,
    RAW_GUI_RULE_CONDITION_INT_EQUALS,
    RAW_GUI_RULE_CONDITION_INT_NOT_EQUALS,
    RAW_GUI_RULE_CONDITION_IS_PERCENT,
    RAW_GUI_RULE_CONDITION_OPTION_DISABLED,
    RAW_GUI_RULE_CONDITION_OPTION_ENABLED,
    RAW_GUI_RULE_CONDITION_VALUE_NON_ZERO,
    RAW_GUI_RULE_CONDITION_VECTOR_EMPTY,
    RAW_GUI_RULE_INDEX_ALL,
    RAW_GUI_RULE_INDEX_CURRENT,
    RAW_PRESET_TYPE_COUNT,
    RAW_PRESET_TYPE_NONE,
)

_LEAF = "leaf"
_ALL = "all"
_ANY = "any"


def _checked_index(index, what: str) -> int:
    index = int(index)
    if index < 0 and index not in (RAW_GUI_RULE_INDEX_ALL, RAW_GUI_RULE_INDEX_CURRENT):
        raise ValueError(
            f"A GUI {what} index must be nonnegative, INDEX_ALL or INDEX_CURRENT."
        )
    return index


class GuiCondition:
    """One owning GUI rule predicate or ``any_of``/``all_of`` combination."""

    __slots__ = ("_kind", "_clause", "_children")

    def __init__(self) -> None:
        self._kind = None
        self._clause = None
        self._children = ()

    @staticmethod
    def _leaf(condition, key, index, value) -> "GuiCondition":
        key = str(key or "")
        if not key:
            raise ValueError("A GUI condition needs a nonempty config key.")
        value = float(value)
        if value != value or value in (float("inf"), float("-inf")):
            raise ValueError("A GUI condition needs a finite numeric operand.")

        out = GuiCondition()
        out._kind = _LEAF
        out._clause = (int(condition), key, _checked_index(index, "condition"), value)
        return out

    @staticmethod
    def true_(key, index=RAW_GUI_RULE_INDEX_ALL) -> "GuiCondition":
        return GuiCondition._leaf(RAW_GUI_RULE_CONDITION_BOOL_TRUE, key, index, 0.0)

    @staticmethod
    def false_(key, index=RAW_GUI_RULE_INDEX_ALL) -> "GuiCondition":
        return GuiCondition._leaf(RAW_GUI_RULE_CONDITION_BOOL_FALSE, key, index, 0.0)

    @staticmethod
    def non_zero(key, index=RAW_GUI_RULE_INDEX_ALL) -> "GuiCondition":
        return GuiCondition._leaf(RAW_GUI_RULE_CONDITION_VALUE_NON_ZERO, key, index, 0.0)

    @staticmethod
    def enabled(key, index=RAW_GUI_RULE_INDEX_ALL) -> "GuiCondition":
        return GuiCondition._leaf(RAW_GUI_RULE_CONDITION_OPTION_ENABLED, key, index, 0.0)

    @staticmethod
    def disabled(key, index=RAW_GUI_RULE_INDEX_ALL) -> "GuiCondition":
        return GuiCondition._leaf(RAW_GUI_RULE_CONDITION_OPTION_DISABLED, key, index, 0.0)

    @staticmethod
    def int_equals(key, value, index=RAW_GUI_RULE_INDEX_ALL) -> "GuiCondition":
        return GuiCondition._leaf(RAW_GUI_RULE_CONDITION_INT_EQUALS, key, index, value)

    @staticmethod
    def int_not_equals(key, value, index=RAW_GUI_RULE_INDEX_ALL) -> "GuiCondition":
        return GuiCondition._leaf(RAW_GUI_RULE_CONDITION_INT_NOT_EQUALS, key, index, value)

    @staticmethod
    def float_equals(key, value, index=RAW_GUI_RULE_INDEX_ALL) -> "GuiCondition":
        return GuiCondition._leaf(RAW_GUI_RULE_CONDITION_FLOAT_EQUALS, key, index, value)

    @staticmethod
    def float_less_than(key, value, index=RAW_GUI_RULE_INDEX_ALL) -> "GuiCondition":
        return GuiCondition._leaf(RAW_GUI_RULE_CONDITION_FLOAT_LESS_THAN, key, index, value)

    @staticmethod
    def float_greater_or_equal(key, value, index=RAW_GUI_RULE_INDEX_ALL) -> "GuiCondition":
        return GuiCondition._leaf(RAW_GUI_RULE_CONDITION_FLOAT_GREATER_OR_EQUAL, key, index, value)

    @staticmethod
    def percent(key, index=RAW_GUI_RULE_INDEX_ALL) -> "GuiCondition":
        return GuiCondition._leaf(RAW_GUI_RULE_CONDITION_IS_PERCENT, key, index, 0.0)

    @staticmethod
    def vector_empty(key, index=RAW_GUI_RULE_INDEX_ALL) -> "GuiCondition":
        return GuiCondition._leaf(RAW_GUI_RULE_CONDITION_VECTOR_EMPTY, key, index, 0.0)


def _combine(kind: str, conditions) -> GuiCondition:
    children = tuple(conditions)
    if not children:
        raise ValueError("A GUI condition combinator needs at least one child.")
    for child in children:
        if not isinstance(child, GuiCondition) or child._kind is None:
            raise ValueError("A GUI condition combinator cannot contain an empty condition.")

    out = GuiCondition()
    out._kind = kind
    out._children = children
    return out


def _condition_list(conditions) -> tuple:
    if len(conditions) == 1 and isinstance(conditions[0], Iterable) \
            and not isinstance(conditions[0], GuiCondition):
        return tuple(conditions[0])
    return tuple(conditions)


def any_of(*conditions) -> GuiCondition:
    """Return a disjunction of the supplied conditions."""
    return _combine(_ANY, _condition_list(conditions))


def all_of(*conditions) -> GuiCondition:
    """Return a conjunction of the supplied conditions."""
    return _combine(_ALL, _condition_list(conditions))


def _append_disjuncts(condition: GuiCondition, out: list) -> None:
    if not isinstance(condition, GuiCondition) or condition._kind is None:
        raise ValueError("A GUI rule contains an empty condition.")
    if condition._kind == _LEAF:
        out.append([condition._clause])
        return
    if condition._kind == _ANY:
        for child in condition._children:
            _append_disjuncts(child, out)
        return

    product = [[]]
    for child in condition._children:
        child_disjuncts: list = []
        _append_disjuncts(child, child_disjuncts)
        product = [prefix + part for prefix in product for part in child_disjuncts]
    out.extend(product)


class GuiRule:
    """One owning GUI activation rule for one or more target keys."""

    __slots__ = ("_targets", "_target_index", "_scope", "_conditions")

    def __init__(self, targets) -> None:
        self._targets = self._normalize_targets(targets)
        self._target_index = RAW_GUI_RULE_INDEX_ALL
        self._scope = RAW_PRESET_TYPE_NONE
        self._conditions: tuple = ()

    @staticmethod
    def _normalize_targets(targets) -> tuple:
        if isinstance(targets, str):
            targets = (targets,)
        else:
            targets = tuple(targets)
        if not targets:
            raise ValueError("A GUI rule needs at least one target key.")
        for target in targets:
            if not str(target or ""):
                raise ValueError("A GUI rule target key must not be empty.")
        return tuple(str(target) for target in targets)

    @classmethod
    def activate(cls, targets) -> "GuiRule":
        """Return one activation reason per target under the given condition."""
        return cls(targets)

    def when(self, *conditions) -> "GuiRule":
        """Append conditions that must all hold (AND)."""
        if not conditions:
            raise ValueError("A GUI rule needs at least one condition per when() call.")
        for condition in conditions:
            if not isinstance(condition, GuiCondition) or condition._kind is None:
                raise ValueError("A GUI rule condition must be built with a GuiCondition factory.")
        self._conditions = self._conditions + tuple(conditions)
        return self

    def when_true(self, key, index=RAW_GUI_RULE_INDEX_ALL) -> "GuiRule":
        return self.when(GuiCondition.true_(key, index))

    def when_false(self, key, index=RAW_GUI_RULE_INDEX_ALL) -> "GuiRule":
        return self.when(GuiCondition.false_(key, index))

    def when_non_zero(self, key, index=RAW_GUI_RULE_INDEX_ALL) -> "GuiRule":
        return self.when(GuiCondition.non_zero(key, index))

    def when_enabled(self, key, index=RAW_GUI_RULE_INDEX_ALL) -> "GuiRule":
        return self.when(GuiCondition.enabled(key, index))

    def when_disabled(self, key, index=RAW_GUI_RULE_INDEX_ALL) -> "GuiRule":
        return self.when(GuiCondition.disabled(key, index))

    def when_int_equals(self, key, value, index=RAW_GUI_RULE_INDEX_ALL) -> "GuiRule":
        return self.when(GuiCondition.int_equals(key, value, index))

    def when_int_not_equals(self, key, value, index=RAW_GUI_RULE_INDEX_ALL) -> "GuiRule":
        return self.when(GuiCondition.int_not_equals(key, value, index))

    def when_float_equals(self, key, value, index=RAW_GUI_RULE_INDEX_ALL) -> "GuiRule":
        return self.when(GuiCondition.float_equals(key, value, index))

    def when_float_less_than(self, key, value, index=RAW_GUI_RULE_INDEX_ALL) -> "GuiRule":
        return self.when(GuiCondition.float_less_than(key, value, index))

    def when_float_greater_or_equal(self, key, value, index=RAW_GUI_RULE_INDEX_ALL) -> "GuiRule":
        return self.when(GuiCondition.float_greater_or_equal(key, value, index))

    def when_percent(self, key, index=RAW_GUI_RULE_INDEX_ALL) -> "GuiRule":
        return self.when(GuiCondition.percent(key, index))

    def when_vector_empty(self, key, index=RAW_GUI_RULE_INDEX_ALL) -> "GuiRule":
        return self.when(GuiCondition.vector_empty(key, index))

    def target_index(self, index) -> "GuiRule":
        self._target_index = _checked_index(index, "target")
        return self

    def scope(self, scope) -> "GuiRule":
        scope = int(scope)
        if scope < RAW_PRESET_TYPE_NONE or scope >= RAW_PRESET_TYPE_COUNT:
            raise ValueError("A GUI rule scope must be a declared RAW_PRESET_TYPE_* value.")
        self._scope = scope
        return self

    def _normalized_disjuncts(self) -> list:
        if not self._conditions:
            raise ValueError("A GUI rule needs at least one condition before it can be registered.")
        root = self._conditions[0] if len(self._conditions) == 1 else _combine(_ALL, self._conditions)

        disjuncts: list = []
        _append_disjuncts(root, disjuncts)
        if not disjuncts:
            raise ValueError("A GUI rule needs at least one condition.")
        return disjuncts

    def add_to(self, api) -> None:
        """Register every disjunct on every target through ``api.add_gui_rule``."""
        if api is None or not hasattr(api, "add_gui_rule"):
            raise TypeError("GuiRule.add_to() needs a Slic3rAPI instance.")

        disjuncts = self._normalized_disjuncts()
        for target in self._targets:
            for disjunct in disjuncts:
                clauses = [
                    {
                        "condition": clause[0],
                        "condition_key": clause[1],
                        "condition_index": clause[2],
                        "condition_float_value": clause[3],
                    }
                    for clause in disjunct
                ]
                api.add_gui_rule(
                    target_key=target,
                    clauses=clauses,
                    target_index=self._target_index,
                    scope=self._scope,
                )


__all__ = ["GuiCondition", "GuiRule", "all_of", "any_of"]
