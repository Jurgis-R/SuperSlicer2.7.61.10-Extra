"""Behavior tests for Python step contexts, progress and cooperative cancellation."""

import ctypes
import gc
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


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class PythonExecutionContextTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        generator = load_module("execution_context_api_generator", RUNTIME / "generate_slic3r_api.py")
        cls.generated_directory = tempfile.TemporaryDirectory()
        generated_path = Path(cls.generated_directory.name) / "slic3r_api_generated.py"
        generated_path.write_text(generator.generate(), encoding="utf-8")
        sys.path.insert(0, cls.generated_directory.name)
        cls.generated = importlib.import_module("slic3r_api_generated")
        cls.run_context = importlib.import_module("slic3r_run_context")
        cls.execution = importlib.import_module("steps.execution")
        cls.api_module = importlib.import_module("slic3r_api")

    @classmethod
    def tearDownClass(cls):
        sys.path.remove(cls.generated_directory.name)
        cls.generated_directory.cleanup()

    def make_common(self, step, payload, *, cancelled=False, progress=None, errors=None):
        generated = self.generated
        keepalive = []
        cancel_cb = generated.PLUGIN_IS_CANCELLED(lambda _ctx: 1 if cancelled else 0)
        warning_cb = generated.PLUGIN_REPORT(lambda _ctx, _message: None)
        error_cb = generated.PLUGIN_REPORT(
            lambda _ctx, message: errors.append(message.decode("utf-8")) if errors is not None else None
        )
        progress_cb = generated.PLUGIN_REPORT_PROGRESS(
            lambda _ctx, value, message: progress.append(
                (float(value), None if not message else message.decode("utf-8"))
            ) if progress is not None else None
        )
        keepalive.extend((cancel_cb, warning_cb, error_cb, progress_cb, payload))
        common = generated.PluginRunContext(
            int(step),
            ctypes.c_void_p(900),
            ctypes.c_void_p(ctypes.addressof(payload)),
            ctypes.c_void_p(901),
            cancel_cb,
            warning_cb,
            error_cb,
            progress_cb,
        )
        keepalive.append(common)
        return common, keepalive

    def test_plugin_base_distinguishes_default_and_absent_exclusive_groups(self):
        singleton = self.api_module.PluginBase("test.singleton", 1)
        service = self.api_module.PluginBase("test.service", 1, exclusive_group="")

        self.assertEqual(singleton.exclusive_group, "test.singleton")
        self.assertEqual(service.exclusive_group, "")

    def test_simple_context_validates_step_and_wraps_borrowed_handles(self):
        payload = self.generated.RunCtxPrepareForPerimeters(ctypes.c_void_p(11), ctypes.c_void_p(12))
        common, keepalive = self.make_common(self.generated.STEP_PRE_PERIMETER, payload)
        api = types.SimpleNamespace(host=types.SimpleNamespace())

        context = self.execution.PrePerimeterContext.from_pre_perimeter_run_context(
            api, ctypes.addressof(common)
        )
        self.assertIsNotNone(context)
        self.assertEqual(context.pre_perimeter_print().address, 11)
        self.assertEqual(context.pre_perimeter_object().address, 12)
        common.step = self.generated.STEP_PRE_INFILL
        self.assertIsNone(self.execution.PrePerimeterContext.from_pre_perimeter_run_context(
            api, ctypes.addressof(common)
        ))
        self.assertTrue(keepalive)

    def test_plan_and_gcode_contexts_copy_only_the_output_path(self):
        firmware = self.generated.RawGcodeFirmwareInstance()
        payload = self.generated.RunCtxGenerateGcode(
            ctypes.c_void_p(21), ctypes.c_void_p(22), ctypes.pointer(firmware), b"out/test.gcode"
        )
        common, keepalive = self.make_common(self.generated.STEP_GCODE, payload)
        api = types.SimpleNamespace(host=types.SimpleNamespace())
        context = self.execution.GCodeContext.from_gcode_run_context(api, ctypes.addressof(common))

        self.assertEqual(context.gcode_print().address, 21)
        self.assertEqual(context.gcode_plan_handle(), 22)
        self.assertEqual(context.gcode_plan().address, 22)
        self.assertEqual(
            context.gcode_firmware_handle(), ctypes.addressof(firmware)
        )
        self.assertEqual(context.gcode_output_path(), "out/test.gcode")
        self.assertTrue(keepalive)

    def test_infill_pattern_params_are_copied_but_output_is_borrowed(self):
        params = self.generated.RawInfillPatternParams()
        params.pattern_id = 42
        payload = self.generated.RunCtxInfillPattern(
            ctypes.c_void_p(1), ctypes.c_void_p(2), ctypes.c_void_p(3), ctypes.c_void_p(4),
            ctypes.c_void_p(5), ctypes.c_void_p(6), ctypes.c_void_p(7), ctypes.c_void_p(8),
            ctypes.pointer(params), ctypes.c_void_p(9),
        )
        common, keepalive = self.make_common(self.generated.INFILL_PATTERN, payload)
        context = self.execution.InfillPatternContext.from_infill_pattern_run_context(
            types.SimpleNamespace(host=types.SimpleNamespace()), ctypes.addressof(common)
        )

        copied = context.infill_pattern_params()
        copied.pattern_id = 99
        self.assertEqual(params.pattern_id, 42)
        self.assertEqual(context.infill_pattern_output().address, 9)
        self.assertTrue(keepalive)

    def test_support_demand_callbacks_preserve_move_and_borrow_semantics(self):
        generated = self.generated
        islands = (ctypes.c_void_p * 1)(ctypes.c_void_p(41))
        state = {"polygons": 51, "cleared": False, "set": None}
        entry_count = generated.SUPPORT_DEMAND_ENTRY_COUNT(lambda _demand: 1)
        entry_island = generated.SUPPORT_DEMAND_ENTRY_ISLAND(lambda _demand, idx: islands[int(idx)])
        get_cb = generated.SUPPORT_DEMAND_GET(lambda _demand, _island: state["polygons"])

        def set_value(_demand, island, polygons):
            state["set"] = (int(island or 0), int(polygons or 0))
            return 1

        set_cb = generated.SUPPORT_DEMAND_SET(set_value)
        clear_cb = generated.SUPPORT_DEMAND_CLEAR(lambda _demand: state.__setitem__("cleared", True))
        payload = generated.RunCtxSupportDemand(
            ctypes.c_void_p(1), ctypes.c_void_p(2), ctypes.c_void_p(3),
            entry_count, entry_island, get_cb, set_cb, clear_cb,
        )
        common, keepalive = self.make_common(generated.STEP_SUPPORT_DEMAND, payload)
        keepalive.extend((islands, entry_count, entry_island, get_cb, set_cb, clear_cb))
        api = types.SimpleNamespace(host=types.SimpleNamespace())
        context = self.execution.SupportDemandContext.from_support_demand_run_context(
            api, ctypes.addressof(common)
        )

        self.assertEqual(context.support_demand_entry_count(), 1)
        island = context.support_demand_entry_island(0)
        self.assertEqual(island.address, 41)
        self.assertEqual(context.support_demand_get(island).address, 51)
        source = types.SimpleNamespace(c_handle=lambda: ctypes.c_void_p(61))
        self.assertTrue(context.support_demand_set(island, source))
        self.assertEqual(state["set"], (41, 61))
        context.support_demand_clear()
        self.assertTrue(state["cleared"])

    def test_progress_reports_visible_phase_and_finishes_after_all_runs(self):
        reports = []
        payload = self.generated.RunCtxPrepareInfill(ctypes.c_void_p(1), ctypes.c_void_p(2))
        common, keepalive = self.make_common(
            self.generated.STEP_PRE_INFILL, payload, progress=reports
        )
        progress = self.run_context.PluginProgress()
        progress.reset(ctypes.addressof(common), 2, "Work {done}/{max}")
        progress.add_max(2)
        progress.increment()
        progress.finish_run()
        self.assertEqual(progress.done(), 1)
        progress.increment()
        progress.finish_run()

        self.assertEqual(progress.done(), 2)
        self.assertEqual(reports[-1], (1.0, "Work 2/2"))
        progress.increment_by(-1)
        self.assertEqual(progress.done(), 1)
        self.assertTrue(keepalive)

    def test_progress_finish_reports_terminal_completion_without_duplicates(self):
        reports = []
        payload = self.generated.RunCtxPrepareInfill(ctypes.c_void_p(1), ctypes.c_void_p(2))
        common, keepalive = self.make_common(
            self.generated.STEP_PRE_INFILL, payload, progress=reports
        )
        progress = self.run_context.PluginProgress()
        progress.reset(ctypes.addressof(common), 1, "Work {done}/{max}")
        progress.add_max(10)
        progress.increment_by(4)
        progress.finish_run()
        self.assertEqual(reports[-1], (1.0, "Work 10/10"))
        self.assertEqual(progress.done(), 10)

        reports.clear()
        progress.reset(ctypes.addressof(common), 2, "Work {done}/{max}")
        progress.add_max(10)
        progress.increment_by(4)
        progress.finish_run()
        self.assertEqual(reports, [(0.4, "Work 4/10")])

        progress.finish_run()
        self.assertEqual(reports[-1], (1.0, "Work 10/10"))
        self.assertEqual(progress.done(), 10)
        report_count = len(reports)
        progress.finish_run()
        self.assertEqual(len(reports), report_count)

        reports.clear()
        progress.reset(ctypes.addressof(common), 1, "Work {done}/{max}")
        progress.add_max(10)
        progress.increment_by(10)
        self.assertEqual(len(reports), 1)
        progress.finish_run()
        self.assertEqual(len(reports), 1)
        self.assertTrue(keepalive)

    def test_progress_finish_keeps_the_next_phase_visible(self):
        reports = []
        payload = self.generated.RunCtxPrepareInfill(ctypes.c_void_p(1), ctypes.c_void_p(2))
        common, keepalive = self.make_common(
            self.generated.STEP_PRE_INFILL, payload, progress=reports
        )
        progress = self.run_context.PluginProgress()
        progress.reset(ctypes.addressof(common), 1, "First {done}/{max}")
        progress.add_max(10, phase=0)
        progress.set_phase_format(1, "Second {done}/{max}")
        progress.add_max(5, phase=1)
        progress.increment_by(4, phase=0)
        progress.finish_run(phase=0)

        self.assertEqual(reports[-1], (0.0, "Second 0/5"))
        self.assertTrue(keepalive)

    def test_plugin_base_distinguishes_cancellation_from_failure(self):
        errors = []
        payload = self.generated.RunCtxPrepareInfill(ctypes.c_void_p(1), ctypes.c_void_p(2))
        common, keepalive = self.make_common(self.generated.STEP_PRE_INFILL, payload, errors=errors)

        class Plugin(self.api_module.PluginBase):
            def __init__(self):
                super().__init__("test.execution", self_outer.generated.STEP_PRE_INFILL)

            def setup(self, _address, _count):
                raise self_outer.run_context.PluginCancelled()

            def run(self, _address):
                raise RuntimeError("broken worker")

        self_outer = self
        plugin = Plugin()
        plugin._slic3r_setup(ctypes.addressof(common), 3)
        self.assertEqual(plugin.progress.run_count(), 3)
        self.assertEqual(errors, [])
        plugin._slic3r_run(ctypes.addressof(common))
        self.assertEqual(errors, ["Python plugin run failed: broken worker"])
        self.assertTrue(keepalive)

    def test_plugin_base_reports_progress_initialization_failure(self):
        errors = []
        payload = self.generated.RunCtxPrepareInfill(ctypes.c_void_p(1), ctypes.c_void_p(2))
        common, keepalive = self.make_common(self.generated.STEP_PRE_INFILL, payload, errors=errors)

        class Plugin(self.api_module.PluginBase):
            def __init__(self):
                super().__init__("test.progress.failure", self_outer.generated.STEP_PRE_INFILL)
                self.setup_called = False

            def progress_message_format(self):
                raise RuntimeError("invalid progress format")

            def setup(self, _address, _count):
                self.setup_called = True

        self_outer = self
        plugin = Plugin()
        plugin._slic3r_setup(ctypes.addressof(common), 1)

        self.assertFalse(plugin.setup_called)
        self.assertEqual(errors, ["Python plugin setup failed: invalid progress format"])
        self.assertTrue(keepalive)

    def test_parallel_helper_rethrows_worker_failure_after_native_callbacks(self):
        calls = []

        class Host:
            @staticmethod
            def slic3r_parallel_for_storage(begin, end, user_data, callback):
                for index in range(int(begin), int(end)):
                    callback(index, ctypes.c_void_p(100 + index), user_data)

        def worker(index, storage):
            calls.append((index, storage))
            if index == 1:
                raise ValueError("worker failed")

        api = types.SimpleNamespace(host=Host())
        with self.assertRaisesRegex(ValueError, "worker failed"):
            self.run_context.parallel_for_storage_with_progress(api, 0, 3, None, None, worker)
        self.assertEqual(calls, [(0, 100), (1, 101)])

    def test_parallel_helper_releases_chained_tracebacks_before_scratch_destruction(self):
        releases = []

        class Host:
            scratch_alive = False

            def slic3r_parallel_for_storage(self, begin, end, user_data, callback):
                for index in range(int(begin), int(end)):
                    self.scratch_alive = True
                    callback(index, ctypes.c_void_p(100 + index), user_data)
                    self.scratch_alive = False

        host = Host()

        class ScratchOwned:
            def __del__(self):
                releases.append(host.scratch_alive)

        def worker(_index, _storage):
            scratch_object = ScratchOwned()
            try:
                raise ValueError("inner failure")
            except ValueError as inner:
                raise RuntimeError("outer failure") from inner

        api = types.SimpleNamespace(host=host)
        with self.assertRaisesRegex(RuntimeError, "outer failure"):
            self.run_context.parallel_for_storage_with_progress(api, 0, 1, None, None, worker)
        gc.collect()

        self.assertEqual(releases, [True])


if __name__ == "__main__":
    unittest.main()
