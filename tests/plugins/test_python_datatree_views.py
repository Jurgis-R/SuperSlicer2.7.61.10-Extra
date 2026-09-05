"""Behavior tests for the small public Python data-tree accessors."""

import ctypes
import importlib
import importlib.util
from pathlib import Path
import sys
import tempfile
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
    if isinstance(handle, ctypes.c_void_p):
        return int(handle.value or 0)
    return int(handle)


class DataTreeHost:
    def __init__(self, generated):
        self.generated = generated
        self.flow_calls = []
        self.bridge_calls = []
        self.curled = [
            generated.CCurledLine(
                generated.CPoint(10, 20), generated.CPoint(30, 40), 0.5
            ),
            generated.CCurledLine(
                generated.CPoint(50, 60), generated.CPoint(70, 80), 1.5
            ),
        ]

    def layer_region_get_flow(self, region, role):
        self.flow_calls.append((address(region), int(role)))
        return self.generated.CFlow()

    def layer_region_get_bridging_flow(self, region, role):
        self.bridge_calls.append((address(region), int(role)))
        return self.generated.CFlow()

    def layer_region_island_count_region(self, region_island):
        return 2

    def layer_region_island_get_region(self, region_island, idx):
        return 200 + int(idx)

    def layer_get_height(self, layer):
        return 40

    def layer_get_print_z(self, layer):
        return 140

    def layer_get_object(self, layer):
        return 300

    def layer_count_curled_line(self, layer):
        return len(self.curled)

    def layer_get_curled_line(self, layer, idx):
        return self.curled[int(idx)]

    def object_count_instance(self, obj):
        return 2

    def object_get_instance_shift(self, obj, idx):
        shifts = ((1000, 2000), (3000, 4000))
        return self.generated.CPoint(*shifts[int(idx)])


class PythonDataTreeViewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        generator = load_python_module(
            "datatree_test_api_generator", RUNTIME / "generate_slic3r_api.py"
        )
        cls.generated_directory = tempfile.TemporaryDirectory()
        generated_path = (
            Path(cls.generated_directory.name) / "slic3r_api_generated.py"
        )
        generated_path.write_text(generator.generate(), encoding="utf-8")
        sys.path.insert(0, cls.generated_directory.name)
        cls.generated = importlib.import_module("slic3r_api_generated")
        cls.views = importlib.import_module("slic3r_datatree_views")

    @classmethod
    def tearDownClass(cls):
        sys.path.remove(cls.generated_directory.name)
        cls.generated_directory.cleanup()

    def make_api(self):
        return type("Api", (), {"host": DataTreeHost(self.generated)})()

    def test_layer_region_flows_and_region_island_iteration(self):
        api = self.make_api()
        layer_region = self.views.LayerRegion(api, 20)

        self.assertIsInstance(
            layer_region.flow(7), self.generated.CFlow
        )
        self.assertIsInstance(layer_region.bridging_flow(8), self.generated.CFlow)
        self.assertEqual(api.host.flow_calls, [(20, 7)])
        self.assertEqual(api.host.bridge_calls, [(20, 8)])

        region_island = self.views.LayerRegionIsland(api, 30)
        self.assertEqual(region_island.region_count(), 2)
        self.assertEqual(
            [region.address for region in region_island.regions()], [200, 201]
        )
        with self.assertRaises(IndexError):
            region_island.region(-1)
        with self.assertRaises(IndexError):
            region_island.region(2)

    def test_layer_owner_bottom_and_curled_line_snapshots(self):
        api = self.make_api()
        layer = self.views.Layer(api, 40)

        self.assertEqual(layer.bottom_z(), 100)
        self.assertEqual(layer.object().address, 300)
        self.assertEqual(layer.curled_line_count(), 2)
        lines = layer.curled_lines()
        self.assertEqual(
            [(line.a.x, line.b.y) for line in lines], [(10, 40), (50, 80)]
        )
        with self.assertRaises(IndexError):
            layer.curled_line(2)

    def test_object_instance_shifts_are_scaled_value_snapshots(self):
        api = self.make_api()
        obj = self.views.Object(api, 50)

        self.assertEqual(obj.instance_count(), 2)
        shift = obj.instance_shift(1)
        self.assertEqual((shift.x, shift.y), (3000, 4000))
        with self.assertRaises(IndexError):
            obj.instance_shift(-1)
        with self.assertRaises(IndexError):
            obj.instance_shift(2)


if __name__ == "__main__":
    unittest.main()
