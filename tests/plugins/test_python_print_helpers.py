"""Behavior tests for the Python ports of the C++ print helpers."""

import importlib
import importlib.util
import math
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch


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


class FakeOption:
    def __init__(self, values, percent=False):
        self.values = list(values) if isinstance(values, (list, tuple)) else [values]
        self.percent = bool(percent)

    def size(self):
        return len(self.values)

    def get_int(self, idx=0):
        return int(self.values[int(idx)])

    def get_float(self, idx=0):
        return float(self.values[int(idx)])

    def is_percent(self, _idx=0):
        return self.percent

    def get_effective_value(self, ratio_over, idx=0):
        value = self.get_float(idx)
        return value * float(ratio_over) / 100.0 if self.percent else value


class FakeConfig:
    def __init__(self, **options):
        self.options = {
            key: value if isinstance(value, FakeOption) else FakeOption(value)
            for key, value in options.items()
        }

    def option(self, key):
        if key not in self.options:
            raise KeyError(key)
        return self.options[key]

    def get(self, key):
        return self.options.get(key)

    def int_or_default(self, key, fallback):
        option = self.options.get(key)
        return int(fallback) if option is None else option.get_int()

    def enum_or_default(self, key, fallback):
        return self.int_or_default(key, fallback)

    def float_or_default(self, key, fallback):
        option = self.options.get(key)
        return float(fallback) if option is None else option.get_float()

    def vector_float_or_default(self, key, idx, fallback):
        option = self.options.get(key)
        return (
            float(fallback)
            if option is None or int(idx) < 0 or int(idx) >= option.size()
            else option.get_float(idx)
        )

    def bool_or_default(self, key, fallback):
        option = self.options.get(key)
        return bool(fallback) if option is None else bool(option.get_int())


class FakeProperties:
    def __init__(self, has_support):
        self.has_support = bool(has_support)

    def get(self, _key):
        return object() if self.has_support else None


class FakeLayer:
    def __init__(self, has_support):
        self._properties = FakeProperties(has_support)

    def properties(self):
        return self._properties


class AuxiliaryHost:
    def __init__(self):
        self.created = 501

    def object_add_auxiliary_layer(self, _obj, _height, _print_z, _slice_z):
        return self.created


class PythonPrintHelperTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        generator = load_python_module(
            "print_helper_test_api_generator", RUNTIME / "generate_slic3r_api.py"
        )
        cls.generated_directory = tempfile.TemporaryDirectory()
        generated_path = Path(cls.generated_directory.name) / "slic3r_api_generated.py"
        generated_path.write_text(generator.generate(), encoding="utf-8")
        sys.path.insert(0, cls.generated_directory.name)
        cls.generated = importlib.import_module("slic3r_api_generated")
        cls.datatree = importlib.import_module("slic3r_datatree_views")
        cls.geometry = importlib.import_module("slic3r_geometry_views")
        cls.helpers = importlib.import_module("slic3r_print_helpers")

    @classmethod
    def tearDownClass(cls):
        sys.path.remove(cls.generated_directory.name)
        cls.generated_directory.cleanup()

    def make_print(self, print_config):
        api = types.SimpleNamespace(host=AuxiliaryHost(), orchestrator=None)
        print_view = self.datatree.Print(api, 101)
        print_view.config = lambda: print_config
        return api, print_view

    def make_object(self, api, config, regions, auxiliary_layers=()):
        obj = self.datatree.Object(api, 201 + len(regions))
        obj.config = lambda: config
        obj.print_regions = lambda: iter(regions)
        obj.auxiliary_layers = lambda: iter(auxiliary_layers)
        return obj

    def make_region(self, api, config, address):
        region = self.datatree.PrintRegion(api, address)
        region.config = lambda: config
        return region

    @staticmethod
    def object_config(**overrides):
        values = {
            "brim_width": 0.0,
            "brim_width_interior": 0.0,
            "first_layer_height": FakeOption(0.2),
            "support_material_extruder": 0,
            "support_material_interface_extruder": 0,
            "support_material_interface_layers": 0,
        }
        values.update(overrides)
        return FakeConfig(**values)

    @staticmethod
    def region_config(**overrides):
        values = {
            "perimeters": 0,
            "perimeter_extruder": 1,
            "fill_density": 0.0,
            "infill_extruder": 1,
            "top_solid_layers": 0,
            "bottom_solid_layers": 0,
            "solid_infill_every_layers": 0,
            "solid_infill_extruder": 1,
        }
        values.update(overrides)
        return FakeConfig(**values)

    def test_region_and_object_extruders_follow_cxx_rules(self):
        api, print_view = self.make_print(FakeConfig(nozzle_diameter=[0.4, 0.6, 0.8]))
        obj_config = self.object_config()
        region = self.make_region(
            api,
            self.region_config(
                perimeters=2,
                perimeter_extruder=3,
                fill_density=20.0,
                infill_extruder=2,
                top_solid_layers=1,
                solid_infill_extruder=7,
            ),
            301,
        )
        empty_region = self.make_region(api, self.region_config(), 302)
        obj = self.make_object(api, obj_config, (region, empty_region))

        self.assertEqual(
            self.helpers.collect_object_printing_extruders(print_view, obj, region),
            (0, 1, 2),
        )
        self.assertEqual(self.helpers.object_extruders(print_view, obj), (0, 1, 2))

    def test_skirt_extruders_cover_support_and_tool_zero_fallback(self):
        api, print_view = self.make_print(FakeConfig(nozzle_diameter=[0.4, 0.6, 0.8]))
        region = self.make_region(
            api, self.region_config(perimeters=1, perimeter_extruder=1), 301
        )
        obj = self.make_object(
            api,
            self.object_config(
                support_material_extruder=2,
                support_material_interface_extruder=3,
                support_material_interface_layers=1,
            ),
            (region,),
            (FakeLayer(True),),
        )
        print_view.objects = lambda: iter((obj,))

        self.assertEqual(
            self.helpers.collect_print_extruders_for_skirt(print_view), (0, 1, 2)
        )
        self.assertEqual(
            self.helpers.collect_print_extruders_for_skirt(print_view, []), (0,)
        )

    def test_first_layer_heights_cover_absolute_percent_and_empty_print(self):
        api, print_view = self.make_print(FakeConfig(nozzle_diameter=[0.4, 0.8]))
        region = self.make_region(
            api, self.region_config(perimeters=1, perimeter_extruder=2), 301
        )
        absolute = self.make_object(api, self.object_config(), (region,))
        percent = self.make_object(
            api,
            self.object_config(first_layer_height=FakeOption(50.0, percent=True)),
            (region,),
        )
        expected_absolute = self.scale_to_layer_coord(0.2)
        expected_percent = self.scale_to_layer_coord(0.4)
        self.assertEqual(
            self.helpers.get_object_first_layer_height(print_view, absolute),
            expected_absolute,
        )
        self.assertEqual(
            self.helpers.get_object_first_layer_height(print_view, percent),
            expected_percent,
        )

        print_view.objects = lambda: iter((absolute, percent))
        self.assertEqual(
            self.helpers.get_min_first_layer_height(print_view), expected_absolute
        )
        print_view.objects = lambda: iter(())
        with self.assertRaises(RuntimeError):
            self.helpers.get_min_first_layer_height(print_view)

    def test_missing_nozzle_uses_tool_zero_and_point_four_mm_fallback(self):
        api, print_view = self.make_print(FakeConfig())
        region = self.make_region(
            api, self.region_config(perimeters=1, perimeter_extruder=2), 301
        )
        obj = self.make_object(
            api,
            self.object_config(first_layer_height=FakeOption(50.0, percent=True)),
            (region,),
        )

        self.assertEqual(self.helpers.object_extruders(print_view, obj), (0,))
        self.assertEqual(
            self.helpers.get_object_first_layer_height(print_view, obj),
            self.scale_to_layer_coord(0.2),
        )

    def scale_to_layer_coord(self, value):
        coord = int(
            (float(value) + float(self.generated.EPSILON) / 2.0)
            * float(self.generated.UNSCALING_FACTOR)
        )
        return (
            coord
            // int(self.generated.SCALED_LAYER_Z_STEP)
            * int(self.generated.SCALED_LAYER_Z_STEP)
        )

    def test_e_per_mm_and_z_step_match_the_cxx_formulas(self):
        api, print_view = self.make_print(
            FakeConfig(
                extrusion_multiplier=[1.0, 0.5],
                filament_diameter=[2.0, 1.75],
                use_volumetric_e=0,
            )
        )
        self.assertAlmostEqual(
            self.helpers.e_per_mm(print_view, 0, 2.0), 2.0 / math.pi
        )
        print_view.config = lambda: FakeConfig(
            extrusion_multiplier=[1.0, 0.5],
            filament_diameter=[2.0, 1.75],
            use_volumetric_e=1,
        )
        self.assertEqual(self.helpers.e_per_mm(print_view, 1, 2.0), 1.0)
        print_view.config = lambda: FakeConfig(
            extrusion_multiplier=[1.0], filament_diameter=[0.0], use_volumetric_e=0
        )
        with self.assertRaises(RuntimeError):
            self.helpers.e_per_mm(print_view, 0, 1.0)

        z_step = int(self.generated.SCALED_EPSILON) * 2
        self.assertEqual(
            self.helpers.check_z_step(z_step + z_step // 4, z_step), z_step
        )
        self.assertEqual(self.helpers.check_z_step(126, 0), 126)

    def test_views_from_different_apis_are_rejected(self):
        api, print_view = self.make_print(FakeConfig(nozzle_diameter=[0.4]))
        foreign_api, _foreign_print = self.make_print(FakeConfig(nozzle_diameter=[0.4]))
        obj = self.make_object(api, self.object_config(), ())
        foreign_region = self.make_region(foreign_api, self.region_config(), 301)
        with self.assertRaises(ValueError):
            self.helpers.collect_object_printing_extruders(
                print_view, obj, foreign_region
            )

    def test_auxiliary_result_borrows_layer_and_forwards_scratch_storage(self):
        api, print_view = self.make_print(FakeConfig())
        obj = self.make_object(api, self.object_config(), ())
        obj.print_region_count = lambda: 1
        subject = self.geometry.ExPolygonCollection(api, 301)
        calls = []

        def assign(storage, received_print, received_obj, layer, received_subject, slice_z):
            calls.append(
                (storage, received_print, received_obj, layer, received_subject, slice_z)
            )

        with patch.object(
            self.datatree, "_auxiliary_assign_regions_from_masks", side_effect=assign
        ):
            result = self.datatree.build_auxiliary_layer_regions_from_subject(
                900, print_view, obj, subject, 200, 300, 250
            )

        self.assertTrue(result.created)
        self.assertIsInstance(result.layer, self.datatree.MutableLayer)
        self.assertEqual(result.layer.address, 501)
        self.assertEqual(calls[0][0], 900)
        self.assertIs(calls[0][3], result.layer)
        self.assertIs(calls[0][4], subject)


if __name__ == "__main__":
    unittest.main()
