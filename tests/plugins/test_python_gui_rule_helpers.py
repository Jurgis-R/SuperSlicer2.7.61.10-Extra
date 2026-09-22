"""Behavior tests for the Python GUI rule helpers."""

import ctypes
import importlib
import importlib.util
from pathlib import Path
import sys
import tempfile
import types
import unittest


ROOT = Path(__file__).resolve().parents[2]
RUNTIME = ROOT / "src/libslic3r/Api/plugin/python"
sys.dont_write_bytecode = True
sys.path.insert(0, str(RUNTIME))


def load_python_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class RecordingGuiRuleApi:
    """Minimal Slic3rAPI stand-in recording add_gui_rule keyword calls."""

    def __init__(self, generated):
        self.generated = generated
        self.calls = []
        self.duplicate_after_first = False

    def add_gui_rule(self, **kwargs):
        self.calls.append(kwargs)
        if self.duplicate_after_first and len(self.calls) > 1:
            return 0
        return 1


class RecordingHost:
    """Fake host ABI used through the real Slic3rAPI wrapper."""

    def __init__(self):
        self.rules = []

    def orchestrator_add_gui_rule(self, _orchestrator, rule_pointer):
        rule = rule_pointer._obj
        clauses = []
        for index in range(int(rule.clause_count)):
            clause = rule.clauses[index]
            clauses.append(
                (
                    int(clause.condition),
                    clause.condition_key.decode("utf-8"),
                    int(clause.condition_index),
                    float(clause.condition_float_value),
                )
            )
        self.rules.append(
            {
                "target_key": rule.target_key.decode("utf-8"),
                "target_index": int(rule.target_index),
                "scope": int(rule.scope),
                "clauses": clauses,
            }
        )
        return 1


class PythonGuiRuleHelperTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        generator = load_python_module(
            "gui_rule_test_api_generator",
            RUNTIME / "generate_slic3r_api.py",
        )
        cls.generated_directory = tempfile.TemporaryDirectory()
        generated_path = Path(cls.generated_directory.name) / "slic3r_api_generated.py"
        generated_path.write_text(generator.generate(), encoding="utf-8")
        sys.path.insert(0, cls.generated_directory.name)
        cls.generated = importlib.import_module("slic3r_api_generated")
        cls.helpers = importlib.import_module("slic3r_gui_rule_helpers")
        cls.api_module = importlib.import_module("slic3r_api")

    @classmethod
    def tearDownClass(cls):
        sys.path.remove(cls.generated_directory.name)
        cls.generated_directory.cleanup()

    def test_dnf_expansion_over_multiple_targets(self):
        api = RecordingGuiRuleApi(self.generated)
        GuiCondition = self.helpers.GuiCondition
        GuiRule = self.helpers.GuiRule

        GuiRule.activate(["gui_rule_target_a", "gui_rule_target_b"]).when(
            self.helpers.all_of(
                GuiCondition.non_zero("gui_rule_int"),
                self.helpers.any_of(
                    GuiCondition.true_("gui_rule_bool"),
                    GuiCondition.int_equals("gui_rule_int", 3),
                ),
            )
        ).add_to(api)

        self.assertEqual(len(api.calls), 4)
        for call in api.calls:
            self.assertEqual(call["target_index"], self.generated.RAW_GUI_RULE_INDEX_ALL)
            self.assertEqual(call["scope"], self.generated.RAW_PRESET_TYPE_NONE)
            self.assertEqual(len(call["clauses"]), 2)
            self.assertEqual(call["clauses"][0]["condition"], self.generated.RAW_GUI_RULE_CONDITION_VALUE_NON_ZERO)
            self.assertEqual(call["clauses"][0]["condition_key"], "gui_rule_int")

        targets = sorted(call["target_key"] for call in api.calls)
        self.assertEqual(targets, ["gui_rule_target_a"] * 2 + ["gui_rule_target_b"] * 2)
        second_clauses = sorted(call["clauses"][1]["condition"] for call in api.calls)
        self.assertEqual(second_clauses, [self.generated.RAW_GUI_RULE_CONDITION_BOOL_TRUE] * 2
                         + [self.generated.RAW_GUI_RULE_CONDITION_INT_EQUALS] * 2)

    def test_index_and_scope_are_preserved(self):
        api = RecordingGuiRuleApi(self.generated)
        GuiCondition = self.helpers.GuiCondition
        GuiRule = self.helpers.GuiRule

        GuiRule.activate("gui_rule_target").when(
            GuiCondition.enabled("gui_rule_bool", 2)
        ).target_index(
            self.generated.RAW_GUI_RULE_INDEX_CURRENT
        ).scope(
            self.generated.RAW_PRESET_TYPE_FFF_PRINT
        ).add_to(api)

        self.assertEqual(len(api.calls), 1)
        call = api.calls[0]
        self.assertEqual(call["target_index"], self.generated.RAW_GUI_RULE_INDEX_CURRENT)
        self.assertEqual(call["scope"], self.generated.RAW_PRESET_TYPE_FFF_PRINT)
        self.assertEqual(call["clauses"][0]["condition_index"], 2)

    def test_invalid_input_is_rejected_eagerly(self):
        api = RecordingGuiRuleApi(self.generated)
        GuiCondition = self.helpers.GuiCondition
        GuiRule = self.helpers.GuiRule
        any_of = self.helpers.any_of
        all_of = self.helpers.all_of

        with self.assertRaises(ValueError):
            GuiRule.activate("")
        with self.assertRaises(ValueError):
            GuiRule.activate([])
        with self.assertRaises(ValueError):
            GuiCondition.true_("")
        with self.assertRaises(ValueError):
            GuiCondition.non_zero("gui_rule_int", -3)
        with self.assertRaises(ValueError):
            GuiCondition.float_less_than("gui_rule_float", float("nan"))
        with self.assertRaises(ValueError):
            any_of([])
        with self.assertRaises(ValueError):
            all_of([])
        with self.assertRaises(ValueError):
            GuiRule.activate("gui_rule_target").add_to(api)
        with self.assertRaises(ValueError):
            GuiRule.activate("gui_rule_target").when(object())
        with self.assertRaises(ValueError):
            GuiRule.activate("gui_rule_target").when_true("gui_rule_bool").target_index(-3)
        with self.assertRaises(ValueError):
            GuiRule.activate("gui_rule_target").when_true("gui_rule_bool").scope(
                self.generated.RAW_PRESET_TYPE_COUNT + 1
            )
        with self.assertRaises(TypeError):
            GuiRule.activate("gui_rule_target").when_true("gui_rule_bool").add_to(None)
        self.assertEqual(api.calls, [])

    def test_duplicate_registration_is_not_an_error(self):
        api = RecordingGuiRuleApi(self.generated)
        api.duplicate_after_first = True
        GuiRule = self.helpers.GuiRule

        rule = GuiRule.activate("gui_rule_target").when_true("gui_rule_bool")
        rule.add_to(api)
        rule.add_to(api)
        self.assertEqual(len(api.calls), 2)

    def test_real_slic3r_api_maps_clauses_and_accepts_lists(self):
        host = RecordingHost()
        api = object.__new__(self.api_module.Slic3rAPI)
        api.host = host
        api.orchestrator = ctypes.c_void_p(10)

        GuiCondition = self.helpers.GuiCondition
        GuiRule = self.helpers.GuiRule

        GuiRule.activate("gui_rule_target").when(
            self.helpers.any_of([
                GuiCondition.true_("gui_rule_bool"),
                self.helpers.all_of(
                    GuiCondition.non_zero("gui_rule_int"),
                    GuiCondition.percent("gui_rule_percent"),
                ),
            ])
        ).add_to(api)

        self.assertEqual(len(host.rules), 2)
        for rule in host.rules:
            self.assertEqual(rule["target_key"], "gui_rule_target")
        self.assertEqual(host.rules[0]["clauses"][0][1], "gui_rule_bool")
        self.assertEqual(
            [(clause[0], clause[1]) for clause in host.rules[1]["clauses"]],
            [
                (self.generated.RAW_GUI_RULE_CONDITION_VALUE_NON_ZERO, "gui_rule_int"),
                (self.generated.RAW_GUI_RULE_CONDITION_IS_PERCENT, "gui_rule_percent"),
            ],
        )


if __name__ == "__main__":
    unittest.main()
