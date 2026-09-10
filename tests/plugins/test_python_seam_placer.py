"""Behavior tests for the Python SEAM_PLACER provider and consumer bridge."""

import copy
import ctypes
from concurrent.futures import ThreadPoolExecutor
import gc
import importlib
import importlib.util
from pathlib import Path
import sys
import tempfile
import threading
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
    return int(handle)


def native_fixture_path():
    """Locate the CMake-built native-thread probe for the active platform."""
    names = (
        "python_seam_placer_native_fixture.dll",
        "libpython_seam_placer_native_fixture.so",
        "libpython_seam_placer_native_fixture.dylib",
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


class SeamHost:
    def __init__(self):
        self.loops = set()
        self.segments = {}
        self.children = {}

    def extrusion_is_loop(self, entity):
        return int(address(entity) in self.loops)

    def extrusion_polyline_segment_count(self, entity):
        return len(self.segments.get(address(entity), ()))

    def extrusion_polyline_copy_segments(self, entity, output, capacity):
        segments = self.segments.get(address(entity), ())
        if not output or int(capacity) == 0:
            return len(segments)
        written = min(len(segments), int(capacity))
        for idx in range(written):
            output[idx] = segments[idx]
        return written

    def extrusion_child_count(self, entity):
        return len(self.children.get(address(entity), ()))

    def extrusion_child(self, entity, idx):
        return self.children[address(entity)][int(idx)]


class PythonSeamPlacerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        generator = load_python_module(
            "seam_placer_test_api_generator", RUNTIME / "generate_slic3r_api.py"
        )
        cls.generated_directory = tempfile.TemporaryDirectory()
        generated_path = Path(cls.generated_directory.name) / "slic3r_api_generated.py"
        generated_path.write_text(generator.generate(), encoding="utf-8")
        sys.path.insert(0, cls.generated_directory.name)
        cls.generated = importlib.import_module("slic3r_api_generated")
        cls.seam = importlib.import_module("steps.seam_placer")
        cls.datatree = importlib.import_module("slic3r_datatree_views")
        cls.extrusion = importlib.import_module("slic3r_extrusion_views")
        cls.printing_plan = importlib.import_module("slic3r_printing_plan_views")

    @classmethod
    def tearDownClass(cls):
        sys.path.remove(cls.generated_directory.name)
        cls.generated_directory.cleanup()

    def setUp(self):
        self.host = SeamHost()
        self.api = types.SimpleNamespace(host=self.host, orchestrator=ctypes.c_void_p(9))

    def point(self, x, y):
        return self.generated.CPoint(int(x), int(y))

    def segment(self, a, b, radius=0.0, orientation=0):
        return self.generated.CExtrusionSegment(
            self.point(*a),
            self.point(*b),
            float(radius),
            int(orientation),
            0,
            0,
        )

    def add_square(self, handle=101):
        self.host.loops.add(handle)
        self.host.segments[handle] = [
            self.segment((0, 0), (1000, 0)),
            self.segment((1000, 0), (1000, 1000)),
            self.segment((1000, 1000), (0, 1000)),
            self.segment((0, 1000), (0, 0)),
        ]
        return self.extrusion.ExtrusionEntity(self.api, handle)

    def add_arc_loop(self, handle=102):
        self.host.loops.add(handle)
        ccw = self.generated.RAW_EXTRUSION_ARC_ORIENTATION_CCW
        self.host.segments[handle] = [
            self.segment((-1000, 0), (1000, 0), 1000.0, ccw),
            self.segment((1000, 0), (-1000, 0), 1000.0, ccw),
        ]
        return self.extrusion.ExtrusionEntity(self.api, handle)

    def add_composite_loop(self, handle=103):
        self.host.loops.add(handle)
        self.host.children[handle] = [104, 105]
        self.host.segments[104] = [
            self.segment((0, 0), (1000, 0)),
            self.segment((1000, 0), (1000, 1000)),
        ]
        self.host.segments[105] = [
            self.segment((1000, 1000), (0, 1000)),
            self.segment((0, 1000), (0, 0)),
        ]
        return self.extrusion.ExtrusionEntity(self.api, handle)

    def publish(self, session):
        plan = self.printing_plan.PrintingPlan(self.api, 77)
        instance = self.seam.make_seam_placer_instance(session, plan)
        return self.seam.SeamPlacer(self.api, instance)

    def region(self, handle=201):
        return self.datatree.LayerRegion(self.api, handle)

    def test_context_initializes_once_and_transfers_unique_ownership(self):
        destroyed = []

        class Session(self.seam.SeamPlacerSession):
            def __init__(self):
                self.initialize_count = 0

            def initialize(self, plan):
                self.initialize_count += 1
                self.plan_address = plan.address

            def place_seam(self, loop, _layer_region, start_position):
                return loop.front()

            def __del__(self):
                destroyed.append(1)

        payload = self.generated.RunCtxSeamPlacer()
        payload.plan = ctypes.c_void_p(77)
        common = self.generated.PluginRunContext()
        common.step = self.generated.SEAM_PLACER
        common.data = ctypes.addressof(payload)
        context = self.seam.SeamPlacerContext.from_seam_placer_run_context(
            self.api, ctypes.addressof(common)
        )
        self.assertIsNotNone(context)

        session = Session()
        session_reference = weakref.ref(session)
        context.publish_seam_placer(session)
        self.assertEqual(session.initialize_count, 1)
        self.assertEqual(session.plan_address, 77)

        placer = self.seam.SeamPlacer(self.api, payload.instance)
        self.assertTrue(placer)
        self.assertFalse(payload.instance.session)
        with self.assertRaises(TypeError):
            copy.copy(placer)

        del session
        gc.collect()
        self.assertIsNotNone(session_reference())
        placer.close()
        placer.close()
        gc.collect()
        self.assertIsNone(session_reference())
        self.assertEqual(destroyed, [1])

    def test_initialization_failure_does_not_publish_an_instance(self):
        class FailingSession(self.seam.SeamPlacerSession):
            def initialize(self, _plan):
                raise RuntimeError("initialization failed")

            def place_seam(self, _loop, _layer_region, _start_position):
                return (0, 0)

        plan = self.printing_plan.PrintingPlan(self.api, 77)
        with self.assertRaisesRegex(RuntimeError, "initialization failed"):
            self.seam.make_seam_placer_instance(FailingSession(), plan)

    def test_one_session_cannot_be_initialized_or_published_twice(self):
        class Session(self.seam.SeamPlacerSession):
            def __init__(self):
                self.initialize_count = 0

            def initialize(self, _plan):
                self.initialize_count += 1

            def place_seam(self, _loop, _layer_region, _start_position):
                return (0, 0)

        plan = self.printing_plan.PrintingPlan(self.api, 77)
        session = Session()
        instance = self.seam.make_seam_placer_instance(session, plan)
        placer = self.seam.SeamPlacer(self.api, instance)
        try:
            with self.assertRaisesRegex(ValueError, "already been consumed"):
                self.seam.make_seam_placer_instance(session, plan)
            self.assertEqual(session.initialize_count, 1)
        finally:
            placer.close()

    def test_consumer_accepts_line_and_arc_points_and_rejects_outside(self):
        square = self.add_square()
        arc = self.add_arc_loop()
        composite = self.add_composite_loop()
        received_regions = []

        class Session(self.seam.SeamPlacerSession):
            def initialize(self, _plan):
                pass

            def place_seam(self, loop, layer_region, _start_position):
                received_regions.append(layer_region.address)
                if loop.address == square.address:
                    return (500, 0)
                if loop.address == composite.address:
                    return (1000, 500)
                return (0, -1000)

        with self.publish(Session()) as placer:
            region = self.region()
            line_result = placer.place_seam(square, region, (0, 0))
            arc_result = placer.place_seam(arc, region, (-2000, 0))
            composite_result = placer.place_seam(composite, region, (0, 0))
            self.assertEqual((line_result.x, line_result.y), (500, 0))
            self.assertEqual((arc_result.x, arc_result.y), (0, -1000))
            self.assertEqual(
                (composite_result.x, composite_result.y), (1000, 500)
            )
            self.assertEqual(received_regions, [region.address] * 3)

        class OutsideSession(self.seam.SeamPlacerSession):
            def initialize(self, _plan):
                pass

            def place_seam(self, _loop, _layer_region, _start_position):
                return (500, 500)

        with self.publish(OutsideSession()) as placer:
            with self.assertRaisesRegex(RuntimeError, "outside the loop path"):
                placer.place_seam(square, self.region(), (0, 0))

    def test_provider_exceptions_have_isolated_diagnostics(self):
        square = self.add_square()
        barrier = threading.Barrier(2)

        class FailingSession(self.seam.SeamPlacerSession):
            def initialize(self, _plan):
                pass

            def place_seam(self, _loop, _layer_region, start_position):
                barrier.wait(timeout=5)
                raise RuntimeError(f"seam failure {start_position.x}")

        with self.publish(FailingSession()) as placer:
            def invoke(value):
                try:
                    placer.place_seam(square, self.region(), (value, 0))
                except RuntimeError as exc:
                    return str(exc)
                return ""

            with ThreadPoolExecutor(max_workers=2) as executor:
                messages = list(executor.map(invoke, (11, 22)))
        self.assertEqual(messages, ["seam failure 11", "seam failure 22"])

    def test_provider_diagnostic_survives_native_thread_state_detach(self):
        fixture_path = native_fixture_path()
        if fixture_path is None:
            self.skipTest("python_seam_placer_native_fixture has not been built")

        square = self.add_square()

        class FailingSession(self.seam.SeamPlacerSession):
            def initialize(self, _plan):
                pass

            def place_seam(self, _loop, _layer_region, _start_position):
                raise RuntimeError("native thread seam failure")

        fixture = ctypes.CDLL(str(fixture_path))
        fixture.python_seam_placer_native_probe_create.argtypes = [
            ctypes.POINTER(self.generated.RawSeamPlacerInstance),
            ctypes.c_void_p,
            ctypes.c_void_p,
            self.generated.CPoint,
        ]
        fixture.python_seam_placer_native_probe_create.restype = ctypes.c_void_p
        fixture.python_seam_placer_native_probe_status.argtypes = [ctypes.c_void_p]
        fixture.python_seam_placer_native_probe_status.restype = ctypes.c_int32
        fixture.python_seam_placer_native_probe_copy_message.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_char),
            ctypes.c_uint32,
        ]
        fixture.python_seam_placer_native_probe_copy_message.restype = ctypes.c_uint32
        fixture.python_seam_placer_native_probe_destroy.argtypes = [ctypes.c_void_p]
        fixture.python_seam_placer_native_probe_destroy.restype = None

        plan = self.printing_plan.PrintingPlan(self.api, 77)
        instance = self.seam.make_seam_placer_instance(FailingSession(), plan)
        published = self.seam._published_sessions[address(instance.session)]
        probe = None
        try:
            probe = fixture.python_seam_placer_native_probe_create(
                ctypes.byref(instance), square.c_handle(), self.region().c_handle(), self.point(0, 0)
            )
            self.assertTrue(probe)
            self.assertEqual(
                fixture.python_seam_placer_native_probe_status(probe),
                self.generated.RAW_SEAM_PLACER_STATUS_ERROR,
            )
            message = ctypes.create_string_buffer(256)
            length = fixture.python_seam_placer_native_probe_copy_message(
                probe, message, len(message)
            )
            self.assertEqual(length, len(b"native thread seam failure"))
            self.assertEqual(message.value, b"native thread seam failure")
            self.assertEqual(len(published.diagnostics), 1)
        finally:
            if probe:
                fixture.python_seam_placer_native_probe_destroy(probe)
            self.seam.SeamPlacer(self.api, instance).close()
        self.assertEqual(published.diagnostics, {})

    def test_invalid_loop_and_incomplete_instances_are_rejected(self):
        square = self.add_square()

        class Session(self.seam.SeamPlacerSession):
            def initialize(self, _plan):
                pass

            def place_seam(self, loop, _layer_region, _start_position):
                return loop.front()

        with self.publish(Session()) as placer:
            self.host.loops.remove(square.address)
            with self.assertRaisesRegex(ValueError, "fixed continuous closed loop"):
                placer.place_seam(square, self.region(), (0, 0))

        incomplete = self.generated.RawSeamPlacerInstance()
        incomplete.struct_size = ctypes.sizeof(self.generated.RawSeamPlacerInstance)
        with self.assertRaisesRegex(ValueError, "incomplete"):
            self.seam.SeamPlacer(self.api, incomplete)
        self.assertEqual(
            incomplete.struct_size,
            ctypes.sizeof(self.generated.RawSeamPlacerInstance),
        )

        destroy_calls = []
        destroy_callback = self.generated.SEAM_PLACER_DESTROY(
            lambda session: destroy_calls.append(address(session))
        )
        missing_place = self.generated.SEAM_PLACER_PLACE()
        vtable = self.seam.RawSeamPlacerVTable(
            ctypes.sizeof(self.seam.RawSeamPlacerVTable),
            destroy_callback,
            missing_place,
        )
        incomplete_vtable_instance = self.generated.RawSeamPlacerInstance(
            ctypes.sizeof(self.generated.RawSeamPlacerInstance),
            ctypes.c_void_p(501),
            ctypes.cast(ctypes.pointer(vtable), ctypes.c_void_p),
        )
        with self.assertRaisesRegex(ValueError, "incomplete"):
            self.seam.SeamPlacer(self.api, incomplete_vtable_instance)
        self.assertEqual(address(incomplete_vtable_instance.session), 501)
        self.assertEqual(destroy_calls, [])

    def test_external_vtable_is_destroyed_once(self):
        calls = []

        def destroy(session):
            calls.append(address(session))

        def place(_session, _loop, _layer_region, _start, result):
            result.contents.status = self.generated.RAW_SEAM_PLACER_STATUS_SUCCESS
            result.contents.point = self.point(0, 0)

        destroy_callback = self.generated.SEAM_PLACER_DESTROY(destroy)
        place_callback = self.generated.SEAM_PLACER_PLACE(place)
        vtable = self.seam.RawSeamPlacerVTable(
            ctypes.sizeof(self.seam.RawSeamPlacerVTable),
            destroy_callback,
            place_callback,
        )
        instance = self.generated.RawSeamPlacerInstance(
            ctypes.sizeof(self.generated.RawSeamPlacerInstance),
            ctypes.c_void_p(501),
            ctypes.cast(ctypes.pointer(vtable), ctypes.c_void_p),
        )
        placer = self.seam.SeamPlacer(self.api, instance)
        placer.close()
        placer.close()
        self.assertEqual(calls, [501])


if __name__ == "__main__":
    unittest.main()
