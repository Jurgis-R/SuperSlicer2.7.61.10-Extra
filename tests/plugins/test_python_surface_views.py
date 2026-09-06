"""Behavior tests for the high-level Python Surface wrappers."""

import copy
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


class ReallocatingSurfaceHost:
    """Tiny ABI host that invalidates every element handle on append."""

    def __init__(self):
        self._next_handle = 100
        self.collections = {}
        self.surfaces = {}
        self.properties = {}
        self.geometry_point_counts = {}
        self.freed = []

    @staticmethod
    def _int(handle):
        if isinstance(handle, ctypes.c_void_p):
            return int(handle.value or 0)
        return int(handle)

    def _new_handle(self):
        self._next_handle += 1
        return self._next_handle

    def storage_new_surface_collection(self, _storage):
        handle = self._new_handle()
        self.collections[handle] = {"items": [], "handles": []}
        return handle

    def surface_collection_size(self, collection):
        return len(self.collections[self._int(collection)]["items"])

    def _rebuild_element_handles(self, collection):
        record = self.collections[self._int(collection)]
        for old_handle in record["handles"]:
            self.surfaces.pop(old_handle, None)
        record["handles"] = []
        for item in record["items"]:
            handle = self._new_handle()
            property_handle = self._new_handle()
            record["handles"].append(handle)
            self.surfaces[handle] = item
            self.properties[property_handle] = item["properties"]
            item["property_handle"] = property_handle

    def surface_collection_append_expolygon_copy(
        self, collection, area, surface_type
    ):
        source_area = self._int(area)
        copied_area = self._new_handle()
        self.geometry_point_counts[copied_area] = self.geometry_point_counts.get(
            source_area, 4
        )
        record = self.collections[self._int(collection)]
        record["items"].append(
            {
                "area": copied_area,
                "type": int(surface_type),
                "properties": {},
            }
        )
        self._rebuild_element_handles(collection)

    def surface_collection_append_expolygon_move(
        self, collection, area, surface_type
    ):
        source_area = self._int(area)
        self.surface_collection_append_expolygon_copy(
            collection, area, surface_type
        )
        self.geometry_point_counts[source_area] = 0

    def surface_collection_at(self, collection, idx):
        return self.collections[self._int(collection)]["handles"][int(idx)]

    def surface_collection_at_mutable(self, collection, idx):
        return self.surface_collection_at(collection, idx)

    def _surface(self, handle):
        address = self._int(handle)
        if address not in self.surfaces:
            raise RuntimeError("stale Surface handle used after reallocation")
        return self.surfaces[address]

    def surface_get_expolygon(self, surface):
        return self._surface(surface)["area"]

    def surface_get_type(self, surface):
        return self._surface(surface)["type"]

    def surface_get_properties(self, surface):
        return self._surface(surface)["property_handle"]

    def expolygon_contour_const(self, expolygon):
        return self._int(expolygon)

    def polygon_as_multipoint_const(self, polygon):
        return self._int(polygon)

    def multipoint_size(self, multipoint):
        return self.geometry_point_counts.get(self._int(multipoint), 4)

    def plugin_property_copy_all(self, destination, source):
        dst = self.properties[self._int(destination)]
        src_address = self._int(source)
        if src_address:
            dst.clear()
            dst.update(self.properties[src_address])

    def storage_free(self, _storage, handle):
        address = self._int(handle)
        self.freed.append((self._int(_storage), address))
        record = self.collections.pop(address, None)
        if record is not None:
            for surface_handle in record["handles"]:
                self.surfaces.pop(surface_handle, None)
        return 1


class PythonSurfaceViewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        generator = load_python_module(
            "surface_test_api_generator", RUNTIME / "generate_slic3r_api.py"
        )
        cls.generated_directory = tempfile.TemporaryDirectory()
        generated_path = (
            Path(cls.generated_directory.name) / "slic3r_api_generated.py"
        )
        generated_path.write_text(generator.generate(), encoding="utf-8")
        sys.path.insert(0, cls.generated_directory.name)
        cls.geometry = importlib.import_module("slic3r_geometry_views")
        cls.surfaces = importlib.import_module("slic3r_surface_views")

    @classmethod
    def tearDownClass(cls):
        sys.path.remove(cls.generated_directory.name)
        cls.generated_directory.cleanup()

    def test_append_like_snapshots_template_before_destination_reallocation(self):
        host = ReallocatingSurfaceHost()
        api = type("Api", (), {"host": host})()
        collection = self.surfaces.StoredSurfaceCollection(api, 1)
        collection.append(self.geometry.ExPolygon(api, 900), 17)
        template = collection.front()
        host._surface(template.c_handle())["properties"][42] = b"metadata"

        collection.append_like(self.geometry.ExPolygon(api, 901), template)

        self.assertEqual(collection.size(), 2)
        appended = host._surface(collection.back().c_handle())
        self.assertEqual(appended["type"], 17)
        self.assertEqual(appended["properties"], {42: b"metadata"})
        collection.free_from_storage()

    def test_append_like_move_uses_the_same_stable_snapshot(self):
        host = ReallocatingSurfaceHost()
        api = type("Api", (), {"host": host})()
        collection = self.surfaces.StoredSurfaceCollection(api, 1)
        collection.append(self.geometry.ExPolygon(api, 910), 23)
        template = collection.front()
        host._surface(template.c_handle())["properties"][7] = b"moved"
        area = self.geometry.StoredExPolygon.adopt_owned(api, 1, 911)

        collection.append_like_move(area, template)

        self.assertEqual(collection.size(), 2)
        appended = host._surface(collection.back().c_handle())
        self.assertEqual(appended["type"], 23)
        self.assertEqual(appended["properties"], {7: b"moved"})
        self.assertTrue(area)
        self.assertTrue(area.contour().empty())
        area.free_from_storage()
        collection.free_from_storage()

    def test_stored_surface_wrappers_have_unique_ownership(self):
        host = ReallocatingSurfaceHost()
        api = type("Api", (), {"host": host})()
        collection = self.surfaces.StoredSurfaceCollection(api, 1)
        area = self.geometry.StoredExPolygon.adopt_owned(api, 1, 920)
        surface = self.surfaces.StoredSurface(area, 17)

        for owner in (collection, surface):
            for copier in (copy.copy, copy.deepcopy):
                with self.subTest(owner=type(owner).__name__, copier=copier.__name__):
                    with self.assertRaisesRegex(
                        TypeError, rf"{type(owner).__name__}.*unique native resource"
                    ):
                        copier(owner)
        self.assertEqual(host.freed, [])
        self.assertTrue(collection.free_from_storage())
        self.assertTrue(surface.free_from_storage())
        self.assertEqual(len(host.freed), 2)


if __name__ == "__main__":
    unittest.main()
