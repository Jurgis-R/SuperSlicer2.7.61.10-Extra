"""Behavior and ownership tests for Python mesh, volume and painting views."""

import ctypes
import importlib
import importlib.util
import math
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
    return int(handle or 0)


class VolumeHost:
    """Small host that records mesh slicing and painting projection calls."""

    def __init__(self, generated):
        self.generated = generated
        self.next_handle = 100
        self.mesh_triangle_counts = {11: 0, 12: 4}
        self.volume_types = {}
        self.painting = set()
        self.slice_calls = []
        self.projection_calls = []
        self.freed = []

    def triangle_mesh_triangle_count(self, mesh):
        return self.mesh_triangle_counts[address(mesh)]

    def storage_new_polygons(self, storage):
        self.next_handle += 1
        return self.next_handle

    def storage_new_expolygons(self, storage):
        self.next_handle += 1
        return self.next_handle

    def _record_slice(self, name, mesh, params, z_values, output, count):
        self.slice_calls.append(
            {
                "name": name,
                "mesh": address(mesh),
                "params": params,
                "z": [float(z_values[idx]) for idx in range(int(count))],
                "output": [address(output[idx]) for idx in range(int(count))],
            }
        )

    def triangle_mesh_slice_to_polygons(self, mesh, transform, z_values, output, count):
        self._record_slice("polygons", mesh, transform, z_values, output, count)

    def triangle_mesh_slice_to_polygons_with_params(self, mesh, params, z_values, output, count):
        self._record_slice("polygons_with_params", mesh, params, z_values, output, count)

    def triangle_mesh_slice_to_expolygons(self, mesh, transform, z_values, output, count):
        self._record_slice("expolygons", mesh, transform, z_values, output, count)

    def triangle_mesh_slice_to_expolygons_with_params(self, mesh, params, z_values, output, count):
        self._record_slice("expolygons_with_params", mesh, params, z_values, output, count)

    def volume_get_type(self, volume):
        return self.volume_types[address(volume)]

    def volume_has_painting(self, volume, key):
        return (address(volume), bytes(key)) in self.painting

    def object_count_layer(self, obj):
        return 2 if address(obj) == 51 else 0

    def object_project_painting_to_polygons(
        self, obj, paint_key, painting_value, output, layer_count
    ):
        self.projection_calls.append(
            (
                address(obj),
                bytes(paint_key),
                int(painting_value),
                [address(output[idx]) for idx in range(int(layer_count))],
            )
        )

    def storage_free(self, storage, handle):
        self.freed.append((address(storage), address(handle)))
        return 1


class PythonVolumeViewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        generator = load_python_module(
            "volume_test_api_generator", RUNTIME / "generate_slic3r_api.py"
        )
        cls.generated_directory = tempfile.TemporaryDirectory()
        generated_path = Path(cls.generated_directory.name) / "slic3r_api_generated.py"
        generated_path.write_text(generator.generate(), encoding="utf-8")
        sys.path.insert(0, cls.generated_directory.name)
        cls.generated = importlib.import_module("slic3r_api_generated")
        cls.views = importlib.import_module("slic3r_datatree_views")
        cls.geometry = importlib.import_module("slic3r_geometry_views")
        cls.api_module = importlib.import_module("slic3r_api")

    @classmethod
    def tearDownClass(cls):
        sys.path.remove(cls.generated_directory.name)
        cls.generated_directory.cleanup()

    def make_api(self):
        api = self.api_module.Slic3rAPI.__new__(self.api_module.Slic3rAPI)
        api.host = VolumeHost(self.generated)
        return api

    def test_mesh_empty_and_slicing_ownership(self):
        api = self.make_api()
        self.assertTrue(self.views.TriangleMesh(api, 11).empty())
        mesh = self.views.TriangleMesh(api, 12)
        self.assertFalse(mesh.empty())
        transform = self.generated.CMatrix4d()
        params = self.generated.CMeshSlicingParams()

        polygons = mesh.slice_to_polygons(7, transform, [0.2, 0.4])
        expolygons = mesh.slice_to_expolygons(7, transform, [0.3], params)

        self.assertEqual(len(polygons), 2)
        self.assertEqual(len(expolygons), 1)
        self.assertTrue(all(
            isinstance(item, self.geometry.StoredPolygonCollection)
            for item in polygons
        ))
        self.assertIsInstance(expolygons[0], self.geometry.StoredExPolygonCollection)
        self.assertEqual([call["name"] for call in api.host.slice_calls], [
            "polygons",
            "expolygons_with_params",
        ])
        self.assertEqual(api.host.slice_calls[0]["z"], [
            ctypes.c_float(0.2).value,
            ctypes.c_float(0.4).value,
        ])
        for result in [*polygons, *expolygons]:
            self.assertEqual(result.storage(), 7)
            self.assertTrue(result.free_from_storage())

    def test_mesh_slicing_rejects_invalid_height_order_and_values(self):
        api = self.make_api()
        mesh = self.views.TriangleMesh(api, 12)
        transform = self.generated.CMatrix4d()

        for heights in ([0.4, 0.2], [0.2, math.nan], [math.inf]):
            with self.subTest(heights=heights), self.assertRaises(ValueError):
                mesh.slice_to_polygons(7, transform, heights)
        self.assertEqual(mesh.slice_to_expolygons(7, transform, []), [])
        self.assertEqual(api.host.slice_calls, [])

    def test_volume_role_and_painting_predicates(self):
        api = self.make_api()
        role_checks = {
            self.generated.RAW_VOLUME_TYPE_MODEL_PART: "is_model_part",
            self.generated.RAW_VOLUME_TYPE_NEGATIVE_VOLUME: "is_negative",
            self.generated.RAW_VOLUME_TYPE_PARAMETER_MODIFIER: "is_modifier",
            self.generated.RAW_VOLUME_TYPE_SUPPORT_ENFORCER: "is_support_enforcer",
            self.generated.RAW_VOLUME_TYPE_SUPPORT_BLOCKER: "is_support_blocker",
            self.generated.RAW_VOLUME_TYPE_SEAM_POSITION_CENTER: "is_seam_position",
            self.generated.RAW_VOLUME_TYPE_SEAM_POSITION_CENTER_Z: "is_seam_position",
            self.generated.RAW_VOLUME_TYPE_SEAM_POSITION_INSIDE_CENTER: "is_seam_position",
            self.generated.RAW_VOLUME_TYPE_SEAM_POSITION_INSIDE: "is_seam_position",
            self.generated.RAW_VOLUME_TYPE_BRIM_PATCH: "is_brim",
            self.generated.RAW_VOLUME_TYPE_BRIM_NEGATIVE: "is_brim",
        }
        for handle, (volume_type, predicate) in enumerate(role_checks.items(), 20):
            api.host.volume_types[handle] = volume_type
            volume = self.views.Volume(api, handle)
            with self.subTest(volume_type=volume_type):
                self.assertTrue(getattr(volume, predicate)())
                if predicate in {"is_support_enforcer", "is_support_blocker"}:
                    self.assertTrue(volume.is_support_modifier())

        painted_handle = 45
        api.host.volume_types[painted_handle] = self.generated.RAW_VOLUME_TYPE_MODEL_PART
        api.host.painting.update({
            (painted_handle, self.generated.RAW_FACET_PAINTING_FDM_SUPPORT.encode()),
            (painted_handle, self.generated.RAW_FACET_PAINTING_SEAM.encode()),
            (painted_handle, self.generated.RAW_FACET_PAINTING_MMU_SEGMENTATION.encode()),
        })
        volume = self.views.Volume(api, painted_handle)
        self.assertTrue(volume.is_fdm_support_painted())
        self.assertTrue(volume.is_seam_painted())
        self.assertTrue(volume.is_mm_painted())

    def test_generic_painting_projection_returns_owned_layer_results(self):
        api = self.make_api()
        obj = self.views.Object(api, 51)

        projected = api.project_painting_to_polygons(
            8,
            obj,
            self.generated.RAW_FACET_PAINTING_SEAM,
            self.generated.RAW_FACET_PAINTING_ENFORCER,
        )

        self.assertEqual(len(projected), 2)
        self.assertEqual(api.host.projection_calls[0][:3], (
            51,
            self.generated.RAW_FACET_PAINTING_SEAM.encode(),
            self.generated.RAW_FACET_PAINTING_ENFORCER,
        ))
        self.assertEqual(
            api.host.projection_calls[0][3],
            [item.handle() for item in projected],
        )
        for result in projected:
            self.assertEqual(result.storage(), 8)
            self.assertTrue(result.free_from_storage())

        empty_object = self.views.Object(api, 52)
        self.assertEqual(api.project_painting_to_polygons(
            8,
            empty_object,
            self.generated.RAW_FACET_PAINTING_SEAM,
            self.generated.RAW_FACET_PAINTING_ENFORCER,
        ), [])
        with self.assertRaises(ValueError):
            api.project_painting_to_polygons(
                8,
                self.views.Object(self.make_api(), 51),
                self.generated.RAW_FACET_PAINTING_SEAM,
                self.generated.RAW_FACET_PAINTING_ENFORCER,
            )


if __name__ == "__main__":
    unittest.main()
