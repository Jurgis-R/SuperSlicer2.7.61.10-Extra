"""Behavior tests for Python orchestrator and bridge-detector wrappers."""

import copy
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


def address(handle):
    if handle is None:
        return 0
    if isinstance(handle, ctypes.c_void_p):
        return int(handle.value or 0)
    return int(handle)


class OrchestratorBridgeHost:
    def __init__(self, generated):
        self.generated = generated
        self.executions = []
        self.bridge_input = None
        self.bridge_destroy_count = 0
        self.coverage_output = None
        self.edges_output = None
        self.bridge_angle = 0.75
        self.bridge_max_length = 2500.0
        self.bridge_layer_id = 6
        self.storage_frees = []
        self.collection_values = {
            301: ["old polygon"],
            302: ["old edge", "old edge"],
        }

    def orchestrator_register_step(self, orchestrator, name, invalidates):
        self.registered_step = (address(orchestrator), name.decode(), int(invalidates))
        return 10001 if name else 0

    def orchestrator_step_name(self, _orchestrator, step):
        return b"example.service" if int(step) == 10001 else None

    def orchestrator_register_runtime_id(self, orchestrator, id_type, name):
        self.registered_runtime_id = (
            address(orchestrator), id_type.decode(), name.decode()
        )
        if not id_type or not name:
            return self.generated.ORCHESTRATOR_RUNTIME_ID_INVALID
        return self.generated.ORCHESTRATOR_RUNTIME_ID_DYNAMIC_BEGIN + 7

    def orchestrator_runtime_id_name(self, _orchestrator, id_type, runtime_id):
        if (
            id_type == b"example.kind"
            and int(runtime_id) == self.generated.ORCHESTRATOR_RUNTIME_ID_DYNAMIC_BEGIN + 7
        ):
            return b"example.value"
        return None

    def orchestrator_find_plugin(self, _orchestrator, plugin_id):
        return 101 if plugin_id == b"example.provider" else 0

    def orchestrator_select_plugin(self, _orchestrator, config, step, group):
        self.selection = (address(config), int(step), group.decode())
        return 101

    def orchestrator_plugin_id(self, plugin):
        return b"example.provider" if address(plugin) == 101 else None

    def orchestrator_plugin_step(self, plugin):
        return 10001 if address(plugin) == 101 else 0

    def orchestrator_execute_plugin(
        self, orchestrator, plugin, print_handle, payloads, run_count
    ):
        data = []
        if payloads:
            data = [address(payloads[idx].data) for idx in range(int(run_count))]
        self.executions.append(
            (
                address(orchestrator),
                address(plugin),
                address(print_handle),
                data,
            )
        )
        return self.generated.RAW_PLUGIN_EXECUTION_SUCCESS

    def orchestrator_register_activation_group(self, _orchestrator, _name, _members):
        return 1

    def orchestrator_create_bridge_detector(self, orchestrator, create_input):
        value = create_input._obj
        self.bridge_input = (
            address(orchestrator),
            address(value.expolygon),
            address(value.expolygons),
            address(value.lower_slices),
            int(value.spacing),
            int(value.precision),
            int(value.layer_id),
        )
        return self.generated.BridgeDetectorInstance(
            ctypes.c_void_p(701), ctypes.c_void_p(702)
        )

    def bridge_detector_destroy(self, detector):
        value = detector._obj
        if value.ctx and value.vt:
            self.bridge_destroy_count += 1
        value.ctx = None
        value.vt = None

    def bridge_detector_detect_angle(self, detector, angle):
        return int(bool(detector._obj.ctx) and float(angle) >= -1.0)

    def bridge_detector_coverage(self, _detector, _angle, output):
        self.coverage_output = address(output)
        self.collection_values[self.coverage_output] = ["coverage 0", "coverage 1"]
        return len(self.collection_values[self.coverage_output])

    def bridge_detector_unsupported_edges(self, _detector, _angle, output):
        self.edges_output = address(output)
        self.collection_values[self.edges_output] = ["edge 0", "edge 1", "edge 2"]
        return len(self.collection_values[self.edges_output])

    def polygons_size(self, collection):
        return len(self.collection_values.get(address(collection), []))

    def polylines_size(self, collection):
        return len(self.collection_values.get(address(collection), []))

    def bridge_detector_get_angle(self, detector):
        return self.bridge_angle if detector._obj.ctx else -1.0

    def bridge_detector_set_max_bridge_length(self, _detector, value):
        self.bridge_max_length = float(value)

    def bridge_detector_get_max_bridge_length(self, detector):
        return self.bridge_max_length if detector._obj.ctx else -1.0

    def bridge_detector_set_layer_id(self, _detector, value):
        self.bridge_layer_id = int(value)

    def bridge_detector_get_layer_id(self, detector):
        return self.bridge_layer_id if detector._obj.ctx else -1

    def storage_free(self, storage, value):
        self.storage_frees.append((address(storage), address(value)))
        return 1


class PythonOrchestratorBridgeViewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        generator = load_python_module(
            "orchestrator_bridge_test_api_generator",
            RUNTIME / "generate_slic3r_api.py",
        )
        cls.generated_directory = tempfile.TemporaryDirectory()
        generated_path = Path(cls.generated_directory.name) / "slic3r_api_generated.py"
        generated_path.write_text(generator.generate(), encoding="utf-8")
        sys.path.insert(0, cls.generated_directory.name)
        cls.generated = importlib.import_module("slic3r_api_generated")
        cls.api_module = importlib.import_module("slic3r_api")
        cls.orchestrator_views = importlib.import_module(
            "slic3r_orchestrator_views"
        )
        cls.bridge_views = importlib.import_module("slic3r_bridge_detector_views")
        cls.geometry = importlib.import_module("slic3r_geometry_views")
        cls.datatree = importlib.import_module("slic3r_datatree_views")

    @classmethod
    def tearDownClass(cls):
        sys.path.remove(cls.generated_directory.name)
        cls.generated_directory.cleanup()

    def make_api(self, orchestrator=10):
        host = OrchestratorBridgeHost(self.generated)
        api = types.SimpleNamespace(
            host=host, orchestrator=ctypes.c_void_p(orchestrator)
        )
        return api, host

    def test_plugin_lookup_selection_and_generic_execution(self):
        api, host = self.make_api()
        orchestrator = self.orchestrator_views.OrchestratorView(api)

        step = orchestrator.register_step("example.service", 500)
        self.assertEqual(step, 10001)
        self.assertEqual(orchestrator.step_name(step), "example.service")
        plugin = orchestrator.find_plugin("example.provider")
        self.assertTrue(plugin)
        self.assertEqual(plugin.id(), "example.provider")
        self.assertEqual(plugin.step(), step)
        self.assertFalse(orchestrator.find_plugin("missing"))

        config = self.datatree.Config(api, 30)
        self.assertTrue(orchestrator.select_plugin(config, step, "providers"))
        first = ctypes.c_int32(12)
        second = ctypes.c_int64(34)
        print_view = self.datatree.Print(api, 40)
        status = orchestrator.execute(plugin, [first, second], print_view)
        self.assertEqual(status, self.generated.RAW_PLUGIN_EXECUTION_SUCCESS)
        self.assertEqual(
            host.executions[-1],
            (10, 101, 40, [ctypes.addressof(first), ctypes.addressof(second)]),
        )
        orchestrator.execute_one(plugin, None)
        self.assertEqual(host.executions[-1][3], [0])

        pointed_payload = ctypes.c_int32(56)
        orchestrator.execute_one(plugin, ctypes.pointer(pointed_payload))
        self.assertEqual(host.executions[-1][3], [ctypes.addressof(pointed_payload)])

        null_pointer = ctypes.POINTER(ctypes.c_int32)()
        orchestrator.execute_one(plugin, null_pointer)
        self.assertEqual(host.executions[-1][3], [0])

        orchestrator.execute_empty(plugin)
        self.assertEqual(host.executions[-1][3], [])

        foreign = self.orchestrator_views.OrchestratorView(
            types.SimpleNamespace(host=host, orchestrator=ctypes.c_void_p(11))
        )
        with self.assertRaises(ValueError):
            foreign.execute(plugin)

    def test_runtime_ids_are_registered_and_resolved_inside_their_type(self):
        api, host = self.make_api()
        orchestrator = self.orchestrator_views.OrchestratorView(api)

        runtime_id = orchestrator.register_runtime_id("example.kind", "example.value")
        self.assertEqual(
            runtime_id, self.generated.ORCHESTRATOR_RUNTIME_ID_DYNAMIC_BEGIN + 7
        )
        self.assertEqual(
            host.registered_runtime_id, (10, "example.kind", "example.value")
        )
        self.assertEqual(
            orchestrator.runtime_id_name("example.kind", runtime_id),
            "example.value",
        )
        self.assertEqual(orchestrator.runtime_id_name("other.kind", runtime_id), "")

        public_api = object.__new__(self.api_module.Slic3rAPI)
        public_api.host = host
        public_api.orchestrator = ctypes.c_void_p(10)
        self.assertEqual(
            public_api.register_runtime_id("example.kind", "example.value"),
            runtime_id,
        )
        self.assertEqual(
            public_api.runtime_id_name("example.kind", runtime_id),
            "example.value",
        )

    def test_generated_runtime_id_contract(self):
        signatures = {
            name: (result, arguments)
            for name, result, arguments in self.generated.C_FUNCTION_SIGNATURES
        }
        self.assertEqual(self.generated.GCODE_SCRIPT_ID_TYPE, "gcode_script_type")
        self.assertEqual(self.generated.ORCHESTRATOR_RUNTIME_ID_INVALID, 0xFFFFFFFF)
        self.assertEqual(
            self.generated.ORCHESTRATOR_RUNTIME_ID_DYNAMIC_BEGIN, 0x80000000
        )
        self.assertEqual(
            signatures["orchestrator_register_runtime_id"],
            (ctypes.c_uint32, [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p]),
        )
        self.assertEqual(
            signatures["orchestrator_runtime_id_name"],
            (ctypes.c_char_p, [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_uint32]),
        )
        self.assertNotIn("gcode_script_register_type", signatures)
        self.assertNotIn("gcode_script_type_name", signatures)

    def test_bridge_detector_owns_instance_and_writes_to_owned_outputs(self):
        api, host = self.make_api()
        orchestrator = self.orchestrator_views.OrchestratorView(api)
        subject = self.geometry.ExPolygon(api, 201)
        lower = self.geometry.ExPolygonCollection(api, 202)

        with orchestrator.create_bridge_detector(
            expolygon=subject,
            lower_slices=lower,
            spacing=400,
            precision=25,
            layer_id=6,
        ) as detector:
            self.assertTrue(detector)
            self.assertEqual(host.bridge_input, (10, 201, 0, 202, 400, 25, 6))
            self.assertTrue(detector.detect_angle())
            self.assertEqual(detector.angle(), 0.75)
            detector.set_max_bridge_length(3200)
            self.assertEqual(detector.max_bridge_length(), 3200)
            detector.set_layer_id(8)
            self.assertEqual(detector.layer_id(), 8)

            polygons = self.geometry.StoredPolygonCollection.adopt_owned(
                api, 900, 301
            )
            polylines = self.geometry.StoredPolylineCollection.adopt_owned(
                api, 900, 302
            )
            try:
                self.assertEqual(polygons.size(), 1)
                self.assertEqual(polylines.size(), 2)
                self.assertEqual(detector.coverage(polygons), 2)
                self.assertEqual(detector.unsupported_edges(polylines), 3)
                self.assertEqual(host.coverage_output, 301)
                self.assertEqual(host.edges_output, 302)
                self.assertEqual(polygons.size(), 2)
                self.assertEqual(polylines.size(), 3)
                self.assertNotIn("old polygon", host.collection_values[301])
                self.assertNotIn("old edge", host.collection_values[302])
            finally:
                polygons.release()
                polylines.release()

            with self.assertRaises(TypeError):
                copy.copy(detector)

        self.assertFalse(detector)
        self.assertEqual(host.bridge_destroy_count, 1)
        detector.reset()
        self.assertEqual(host.bridge_destroy_count, 1)

    def test_bridge_creation_rejects_missing_or_foreign_geometry(self):
        api, _host = self.make_api()
        orchestrator = self.orchestrator_views.OrchestratorView(api)
        with self.assertRaises(ValueError):
            orchestrator.create_bridge_detector(
                spacing=1, precision=1, layer_id=0
            )

        foreign_api, _foreign_host = self.make_api(11)
        foreign_subject = self.geometry.ExPolygon(foreign_api, 201)
        with self.assertRaises(ValueError):
            orchestrator.create_bridge_detector(
                expolygon=foreign_subject,
                spacing=1,
                precision=1,
                layer_id=0,
            )


if __name__ == "__main__":
    unittest.main()
