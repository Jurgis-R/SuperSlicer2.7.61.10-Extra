"""Behavior tests for the Python GCODE_FIRMWARE provider and consumer adapters."""

import ctypes
import gc
import importlib
import importlib.util
from pathlib import Path
import sys
import tempfile
import types
import unittest
import weakref


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


def address(handle):
    if handle is None:
        return 0
    if isinstance(handle, ctypes.c_void_p):
        return int(handle.value or 0)
    try:
        return int(handle)
    except (TypeError, ValueError):
        return int(ctypes.cast(handle, ctypes.c_void_p).value or 0)


def native_fixture_path():
    """Locate the CMake-built native firmware callback probe."""
    names = (
        "python_gcode_firmware_native_fixture.dll",
        "libpython_gcode_firmware_native_fixture.so",
        "libpython_gcode_firmware_native_fixture.dylib",
    )
    directories = (
        ROOT / "build/tests/plugins/Debug",
        ROOT / "build/tests/plugins/Release",
        ROOT / "build/tests/plugins/RelWithDebInfo",
        ROOT / "build/tests/plugins",
    )
    for directory in directories:
        for name in names:
            candidate = directory / name
            if candidate.is_file():
                return candidate
    return None


class PythonGCodeFirmwareTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        generator = load_python_module(
            "gcode_firmware_test_api_generator", RUNTIME / "generate_slic3r_api.py"
        )
        cls.generated_directory = tempfile.TemporaryDirectory()
        generated_path = Path(cls.generated_directory.name) / "slic3r_api_generated.py"
        generated_path.write_text(generator.generate(), encoding="utf-8")
        sys.path.insert(0, cls.generated_directory.name)
        cls.generated = importlib.import_module("slic3r_api_generated")
        cls.firmware = importlib.import_module("steps.gcode_firmware")
        cls.datatree = importlib.import_module("slic3r_datatree_views")
        cls.extrusion = importlib.import_module("slic3r_extrusion_views")
        cls.plan = importlib.import_module("slic3r_printing_plan_views")
        cls.execution = importlib.import_module("steps.execution")

    @classmethod
    def tearDownClass(cls):
        sys.path.remove(cls.generated_directory.name)
        cls.generated_directory.cleanup()

    def setUp(self):
        self.api = types.SimpleNamespace(host=types.SimpleNamespace())

    def destroy_instance(self, instance):
        vtable = ctypes.cast(
            instance.vtable, ctypes.POINTER(self.firmware.RawGcodeFirmwareVTable)
        )
        vtable.contents.destroy(instance.session)
        instance.session = None
        instance.vtable = None

    def complete_session(self, *, fail_at=None):
        calls = []
        firmware = self.firmware

        class Session(firmware.GCodeFirmwareSession):
            def begin_print(self, value):
                calls.append(("begin_print", value.address))
                return "print\n"

            def begin_group(self, value):
                calls.append(("begin_group", value.address))
                return "group\n"

            def begin_layer(self, value):
                calls.append(("begin_layer", value.address))
                return "layer\n"

            def begin_tool_group(self, value):
                calls.append(("begin_tool_group", value.address))
                return "tool\n"

            def write_extrusion(self, value):
                calls.append(("write_extrusion", value.address))
                return "A\0B"

            def write_event(self, value):
                calls.append(("write_event", value.address))
                if fail_at == "write_event":
                    raise RuntimeError("event failed")
                return "event\n"

            def end_tool_group(self):
                calls.append(("end_tool_group", None))
                return "end tool\n"

            def end_layer(self):
                calls.append(("end_layer", None))
                return "end layer\n"

            def end_group(self):
                calls.append(("end_group", None))
                return "end group\n"

            def end_print(self):
                calls.append(("end_print", None))
                return "end print\n"

        return Session(), calls

    def test_complete_callback_order_and_embedded_nul_text(self):
        session, calls = self.complete_session()
        instance = self.firmware.make_gcode_firmware_instance(session, self.api)
        try:
            view = self.firmware.GCodeFirmwareView(self.api, instance)
            self.assertEqual(view.begin_print(self.datatree.Print(self.api, 11)), "print\n")
            self.assertEqual(view.begin_group(self.plan.PrintingGroup(self.api, 12)), "group\n")
            self.assertEqual(view.begin_layer(self.plan.PrintingLayerGroup(self.api, 13)), "layer\n")
            self.assertEqual(view.begin_tool_group(self.plan.PrintingToolGroup(self.api, 14)), "tool\n")
            self.assertEqual(view.write_extrusion(self.plan.PrintingExtrusion(self.api, 15)), "A\0B")
            self.assertEqual(view.write_event(self.extrusion.ExtrusionEntity(self.api, 16)), "event\n")
            self.assertEqual(view.end_tool_group(), "end tool\n")
            self.assertEqual(view.end_layer(), "end layer\n")
            self.assertEqual(view.end_group(), "end group\n")
            self.assertEqual(view.end_print(), "end print\n")
        finally:
            self.destroy_instance(instance)

        self.assertEqual(
            calls,
            [
                ("begin_print", 11),
                ("begin_group", 12),
                ("begin_layer", 13),
                ("begin_tool_group", 14),
                ("write_extrusion", 15),
                ("write_event", 16),
                ("end_tool_group", None),
                ("end_layer", None),
                ("end_group", None),
                ("end_print", None),
            ],
        )

    def test_exception_is_contained_and_diagnostic_lives_until_next_call(self):
        session, _calls = self.complete_session(fail_at="write_event")
        instance = self.firmware.make_gcode_firmware_instance(session, self.api)
        published = self.firmware._published_sessions[address(instance.session)]
        view = self.firmware.GCodeFirmwareView(self.api, instance)
        try:
            with self.assertRaisesRegex(RuntimeError, "event failed"):
                view.write_event(self.extrusion.ExtrusionEntity(self.api, 16))
            self.assertIsNotNone(published.error)
            self.assertEqual(published.error.value, b"event failed")
            self.assertEqual(view.end_print(), "end print\n")
            self.assertIsNone(published.error)
        finally:
            self.destroy_instance(instance)
        self.assertIsNone(published.output)
        self.assertIsNone(published.error)

    def test_publication_transfers_unique_session_and_destroys_once(self):
        destroyed = []
        firmware = self.firmware

        class Session(firmware.GCodeFirmwareSession):
            def begin_print(self, _value): return ""
            def begin_group(self, _value): return ""
            def begin_layer(self, _value): return ""
            def begin_tool_group(self, _value): return ""
            def write_extrusion(self, _value): return ""
            def write_event(self, _value): return ""
            def end_tool_group(self): return ""
            def end_layer(self): return ""
            def end_group(self): return ""
            def end_print(self): return ""
            def __del__(self): destroyed.append(1)

        payload = self.generated.RunCtxGcodeFirmware()
        payload.print = ctypes.c_void_p(21)
        common = self.generated.PluginRunContext()
        common.step = self.generated.GCODE_FIRMWARE
        common.data = ctypes.addressof(payload)
        context = firmware.GCodeFirmwareContext.from_gcode_firmware_run_context(
            self.api, ctypes.addressof(common)
        )
        session = Session()
        reference = weakref.ref(session)
        context.publish_gcode_firmware(session)
        with self.assertRaisesRegex(ValueError, "already owns"):
            context.publish_gcode_firmware(session)
        with self.assertRaisesRegex(ValueError, "already been consumed"):
            firmware.make_gcode_firmware_instance(session, self.api)

        del session
        gc.collect()
        self.assertIsNotNone(reference())
        self.destroy_instance(payload.instance)
        gc.collect()
        self.assertIsNone(reference())
        self.assertEqual(destroyed, [1])

    def test_gcode_context_returns_borrowed_firmware_view(self):
        session, _calls = self.complete_session()
        instance = self.firmware.make_gcode_firmware_instance(session, self.api)
        payload = self.generated.RunCtxGenerateGcode(
            ctypes.c_void_p(1), ctypes.c_void_p(2), ctypes.pointer(instance), b"out.gcode"
        )
        common = self.generated.PluginRunContext()
        common.step = self.generated.STEP_GCODE
        common.data = ctypes.addressof(payload)
        try:
            context = self.execution.GCodeContext.from_gcode_run_context(
                self.api, ctypes.addressof(common)
            )
            self.assertEqual(context.gcode_firmware().end_print(), "end print\n")
        finally:
            self.destroy_instance(instance)

    def test_script_processor_copies_text_and_invalidates_old_context(self):
        output = ctypes.create_string_buffer(b"script\0result")
        config_address = 401

        def prepare(_context, _script_type, _producer):
            return config_address

        def process(_context, _script, _extruder, result):
            result.contents.status = self.generated.RAW_GCODE_SCRIPT_STATUS_SUCCESS
            result.contents.text = ctypes.cast(output, ctypes.c_char_p)
            result.contents.text_size = len(b"script\0result")

        prepare_callback = self.generated.GCODE_SCRIPT_PREPARE(prepare)
        process_callback = self.generated.GCODE_SCRIPT_PROCESS(process)
        raw = self.generated.RawGcodeScriptProcessor(
            ctypes.sizeof(self.generated.RawGcodeScriptProcessor),
            ctypes.c_void_p(99),
            prepare_callback,
            process_callback,
        )
        processor = self.firmware.GCodeScriptProcessorView(
            self.api, ctypes.pointer(raw)
        )
        first = processor.prepare(self.generated.GCODE_SCRIPT_TYPE_START_GCODE)
        self.assertEqual(first.process("start", 0), "script\0result")
        second = processor.prepare(self.generated.GCODE_SCRIPT_TYPE_END_GCODE)
        with self.assertRaisesRegex(RuntimeError, "invalidated"):
            first.process("stale", 0)
        self.assertEqual(second.process("end", 1), "script\0result")

    def test_script_processor_invalidation_is_shared_between_views(self):
        output = ctypes.create_string_buffer(b"shared")

        def prepare(_context, script_type, _producer):
            return 500 + int(script_type)

        def process(_context, _script, _extruder, result):
            result.contents.status = self.generated.RAW_GCODE_SCRIPT_STATUS_SUCCESS
            result.contents.text = ctypes.cast(output, ctypes.c_char_p)
            result.contents.text_size = len(b"shared")

        prepare_callback = self.generated.GCODE_SCRIPT_PREPARE(prepare)
        process_callback = self.generated.GCODE_SCRIPT_PROCESS(process)
        raw = self.generated.RawGcodeScriptProcessor(
            ctypes.sizeof(self.generated.RawGcodeScriptProcessor),
            ctypes.c_void_p(99),
            prepare_callback,
            process_callback,
        )
        first_view = self.firmware.GCodeScriptProcessorView(
            self.api, ctypes.pointer(raw)
        )
        second_view = self.firmware.GCodeScriptProcessorView(
            self.api, ctypes.pointer(raw)
        )

        stale = first_view.prepare(self.generated.GCODE_SCRIPT_TYPE_START_GCODE)
        current = second_view.prepare(self.generated.GCODE_SCRIPT_TYPE_END_GCODE)

        with self.assertRaisesRegex(RuntimeError, "invalidated"):
            stale.process("stale", 0)
        self.assertEqual(current.process("current", 0), "shared")

    def test_invalid_instances_and_non_string_results_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "incomplete"):
            self.firmware.GCodeFirmwareView(
                self.api, self.generated.RawGcodeFirmwareInstance()
            )

        session, _calls = self.complete_session()
        session.end_print = lambda: b"bytes are not text"
        instance = self.firmware.make_gcode_firmware_instance(session, self.api)
        try:
            with self.assertRaisesRegex(RuntimeError, "must return str"):
                self.firmware.GCodeFirmwareView(self.api, instance).end_print()
        finally:
            self.destroy_instance(instance)

    def test_output_and_error_survive_native_thread_state_detach(self):
        fixture_path = native_fixture_path()
        if fixture_path is None:
            self.skipTest("python_gcode_firmware_native_fixture has not been built")

        fixture = ctypes.CDLL(str(fixture_path))
        fixture.python_gcode_firmware_native_probe_create.argtypes = [
            ctypes.POINTER(self.generated.RawGcodeFirmwareInstance)
        ]
        fixture.python_gcode_firmware_native_probe_create.restype = ctypes.c_void_p
        fixture.python_gcode_firmware_native_probe_status.argtypes = [ctypes.c_void_p]
        fixture.python_gcode_firmware_native_probe_status.restype = ctypes.c_int32
        for name in ("copy_text", "copy_error"):
            callback = getattr(fixture, f"python_gcode_firmware_native_probe_{name}")
            callback.argtypes = [
                ctypes.c_void_p,
                ctypes.POINTER(ctypes.c_char),
                ctypes.c_uint32,
            ]
            callback.restype = ctypes.c_uint32
        fixture.python_gcode_firmware_native_probe_destroy.argtypes = [ctypes.c_void_p]
        fixture.python_gcode_firmware_native_probe_destroy.restype = None

        for fails, expected_status, expected in (
            (False, self.generated.RAW_GCODE_FIRMWARE_STATUS_SUCCESS, b"native\0text"),
            (True, self.generated.RAW_GCODE_FIRMWARE_STATUS_ERROR, b"native firmware failure"),
        ):
            with self.subTest(fails=fails):
                session, _calls = self.complete_session()
                if fails:
                    session.end_print = lambda: (_ for _ in ()).throw(
                        RuntimeError("native firmware failure")
                    )
                else:
                    session.end_print = lambda: "native\0text"
                instance = self.firmware.make_gcode_firmware_instance(session, self.api)
                probe = None
                try:
                    probe = fixture.python_gcode_firmware_native_probe_create(
                        ctypes.byref(instance)
                    )
                    self.assertTrue(probe)
                    self.assertEqual(
                        fixture.python_gcode_firmware_native_probe_status(probe),
                        expected_status,
                    )
                    output = ctypes.create_string_buffer(128)
                    copy = (
                        fixture.python_gcode_firmware_native_probe_copy_error
                        if fails
                        else fixture.python_gcode_firmware_native_probe_copy_text
                    )
                    length = copy(probe, output, len(output))
                    self.assertEqual(length, len(expected))
                    self.assertEqual(output.raw[:length], expected)
                finally:
                    if probe:
                        fixture.python_gcode_firmware_native_probe_destroy(probe)
                    self.destroy_instance(instance)


if __name__ == "__main__":
    unittest.main()
