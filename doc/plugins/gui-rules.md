# GUI Activation Rules

GUI activation rules enable or disable a setting in a preset tab. The host
evaluates them from `ConfigManipulation` whenever a page refreshes its enabled
state, so plugin settings do not need hardcoded logic there. A rule never
changes a stored value; it only forwards an enabled/disabled state to the GUI
field.

The C ABI is declared in `Api/plugin/c/slic3r_orchestrator.h`
(`orchestrator_add_gui_rule`). `Api/plugin/cpp/GuiRuleHelpers.hpp` provides the
owning `slic3r_api::GuiCondition` / `slic3r_api::GuiRule` builders used by the
core and by C++ plugins.

## Rule model

One raw rule carries:

- one `target_key` and an optional `target_index`;
- an optional `scope` (`RAW_PRESET_TYPE_NONE` applies everywhere);
- one or more clauses that are combined with a logical AND.

A rule is one activation reason: the target is enabled as soon as at least one
rule registered for it is true (OR). There is no separate "must hold" rule
kind, and a setting with no rule at all stays enabled. A disjunction therefore
becomes several rules, one per disjunct.

Rules whose target setting is not defined yet are queued and attached when the
definition appears; a non-blocking warning lists the keys that never appeared.
`struct_size` must be `sizeof(raw_gui_rule)` so later additions to the
structure stay source-compatible.

## Conditions

Every clause reads one option (`condition_key`) at `condition_index` and
compares it using `condition_float_value`:

| Condition | True when |
|---|---|
| `BOOL_TRUE` / `BOOL_FALSE` | the boolean value is true / false |
| `OPTION_ENABLED` / `OPTION_DISABLED` | the option is not disabled / is disabled |
| `VALUE_NON_ZERO` | the numeric value is not zero |
| `INT_EQUALS` / `INT_NOT_EQUALS` | the integer value equals / differs from `llround(condition_float_value)` |
| `FLOAT_EQUALS` / `FLOAT_LESS_THAN` / `FLOAT_GREATER_OR_EQUAL` | `get_float()` equals / is below / is at least `condition_float_value` |
| `IS_PERCENT` | the option stores a percentage |
| `VECTOR_EMPTY` | the option is a vector with no item |
| `STRING_EQUALS` | the serialized value equals `condition_string_value` |

`condition_float_value` is the single numeric operand: `INT_*` conditions round
it, so enum comparisons pass the enum value directly. `FLOAT_*` compares the
normalized value: a percent option uses a fraction, so 50% is `0.5`.
`IS_PERCENT` and `VECTOR_EMPTY` ignore the numeric value.
`STRING_EQUALS` compares the serialized value of any option type, so it works
for plugin-defined enums that store plugin ids: for example
`string_equals("solid_fill_pattern", "smooth")`. A leading `!` (disabled
option) is not part of the compared value.

`condition_index` may be a real item index, `RAW_GUI_RULE_INDEX_ALL` (default)
or `RAW_GUI_RULE_INDEX_CURRENT`. For an extruder-sized vector, `ALL` means
"true when any item satisfies the clause". `target_index` follows the same
values: `ALL` toggles the whole field once, `CURRENT` toggles the item processed
by the current extruder loop.

## C++ builder

`GuiCondition` is an owning expression tree: leaf factories and the `any_of` /
`all_of` combinators. `GuiRule::activate()` normalizes the tree to disjunctive
normal form and registers one activation reason per disjunction and target, so
an `any_of` condition naturally becomes an OR.

```cpp
using namespace slic3r_api;

// One conjunction: every clause must hold.
GuiRule::activate("hole_to_polyhole_threshold")
    .when_true("hole_to_polyhole")
    .add_to(orch);

// Disjunction: the target is enabled for support material or for a raft.
GuiRule::activate("dont_support_bridges")
    .when(any_of({
        GuiCondition::true_("support_material"),
        GuiCondition::non_zero("raft_layers")
    }))
    .add_to(orch);

// One expression for several targets, restricted to the FFF print tab.
GuiRule::activate({ "brim_ears_max_angle", "brim_ears_pattern" })
    .when(GuiCondition::true_("brim_ears"))
    .when_non_zero("brim_width")
    .scope(RAW_PRESET_TYPE_FFF_PRINT)
    .add_to(orch);
```

`add_to` accepts an `orchestrator_handle *`: plugins pass the handle received at
registration, and host code passes `Orchestrator::instance().handle()`. The free
`add_gui_rule(orch, rule)` and `add_gui_rules(orch, { ... })` wrappers cover the
one-shot and batch cases.

`GuiCondition` also exposes `false_`, `enabled`, `disabled`, `int_equals`,
`int_not_equals`, `float_equals`, `float_less_than`, `float_greater_or_equal`,
`percent` and `vector_empty`, each taking an optional condition index.

## Python builder

`slic3r_gui_rule_helpers.py` mirrors the C++ surface and is re-exported by
`slic3r_api`, so plugins can import `GuiRule`, `GuiCondition`, `any_of` and
`all_of` from `slic3r_api`:

```python
GuiRule.activate([
    POLYHOLES_THRESHOLD_KEY,
    POLYHOLES_TWISTED_KEY,
]).when_true(POLYHOLES_KEY).add_to(self.api)

GuiRule.activate("support_material_interface_speed").when(
    any_of([
        all_of([GuiCondition.true_("support_material"),
                GuiCondition.non_zero("support_material_interface_layers")]),
        all_of([GuiCondition.non_zero("raft_layers"),
                GuiCondition.non_zero("support_material_interface_layers")]),
    ])
).scope(RAW_PRESET_TYPE_FFF_PRINT).add_to(api)
```

The raw wrapper `Slic3rAPI.add_gui_rule(...)` is also available for callers
that build clause mappings or tuples themselves.

## Validation and errors

The builders validate eagerly: empty keys, invalid indexes, non-finite numeric
operands, empty `any_of`/`all_of` combinations and an empty target list raise
`std::invalid_argument` / `ValueError` before the host is called.

`add_to` maps the host result: an invalid descriptor raises
`std::invalid_argument` / `ValueError`, an internal failure raises
`std::runtime_error` / `RuntimeError`, and an identical registration is a
silent no-op like the C return code 0.

The numeric operand field is `condition_float_value`; `INT_*` conditions round
it, so enum comparisons pass the enum value directly. Plugins compiled against
an older SDK must be rebuilt to match the raw clause layout.

## Current users

`initialize_fff_print_gui_rules()` in `Config/FFFPrintConfig.cpp` publishes the
complete FFF print activation logic: every legacy enable/disable branch becomes
an activation reason. Official plugins (`Polyholes`, `MaxOverhangThreshold`,
`SupportDemandBridgeRemoval`, `DefaultBrimGenerator`, Python Polyholes) and
`GuiRulesExample` demonstrate the builder for simple conjunctions and
disjunctions. `PluginLoader` generates exclusive-step selector rules through
the raw C API.
