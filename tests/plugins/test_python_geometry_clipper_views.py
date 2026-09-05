"""Ownership and dispatch tests for Python geometry and Clipper helpers."""

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
    return int(handle or 0)


class GeometryClipperHost:
    """Small host that records calls and ownership releases."""

    def __init__(self):
        self.next_handle = 100
        self.calls = []
        self.freed = []
        self.fail_next = False
        self.polyline_sizes = {21: 2, 32: 2}
        self.polyline_collection_sizes = {}

    def _result(self, name, *arguments):
        self.calls.append((name, tuple(address(arg) for arg in arguments)))
        if self.fail_next:
            self.fail_next = False
            return 0
        self.next_handle += 1
        return self.next_handle

    def polygon_convex_hull(self, storage, polygon):
        return self._result("convex_hull", storage, polygon)

    def expolygons_to_polylines(self, storage, src):
        return self._result("expolygons_to_polylines", storage, src)

    def clipper_diff_polyline_expolygons(self, storage, subject, clip):
        return self._result("polyline_diff", storage, subject, clip)

    def clipper_intersection_polyline_expolygons(self, storage, subject, clip):
        return self._result("polyline_intersection", storage, subject, clip)

    def clipper_clip_expolygons_with_subject_bbox(self, storage, src, bbox):
        self.calls.append(("clip_expolygons_bbox", (address(storage), address(src)), bbox))
        if self.fail_next:
            self.fail_next = False
            return 0
        self.next_handle += 1
        return self.next_handle

    def polyline_as_multipoint_const(self, polyline):
        return address(polyline)

    def multipoint_size(self, multipoint):
        return self.polyline_sizes.get(address(multipoint), 0)

    def storage_new_polylines(self, storage):
        self.next_handle += 1
        self.polyline_collection_sizes[self.next_handle] = 0
        return self.next_handle

    def polylines_size(self, polylines):
        return self.polyline_collection_sizes[address(polylines)]

    def storage_free(self, storage, handle):
        self.freed.append((address(storage), address(handle)))
        return 1


class PythonGeometryClipperViewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        generator = load_python_module(
            "geometry_clipper_test_api_generator",
            RUNTIME / "generate_slic3r_api.py",
        )
        cls.generated_directory = tempfile.TemporaryDirectory()
        generated_path = Path(cls.generated_directory.name) / "slic3r_api_generated.py"
        generated_path.write_text(generator.generate(), encoding="utf-8")
        sys.path.insert(0, cls.generated_directory.name)
        cls.generated = importlib.import_module("slic3r_api_generated")
        cls.geometry = importlib.import_module("slic3r_geometry_views")
        cls.clipper = importlib.import_module("slic3r_clipper_views")
        cls.api_module = importlib.import_module("slic3r_api")

    @classmethod
    def tearDownClass(cls):
        sys.path.remove(cls.generated_directory.name)
        cls.generated_directory.cleanup()

    def make_api(self):
        api = self.api_module.Slic3rAPI.__new__(self.api_module.Slic3rAPI)
        api.host = GeometryClipperHost()
        return api

    def test_geometry_conversions_return_storage_owned_results(self):
        api = self.make_api()
        polygon = self.geometry.Polygon(api, 11)
        expolygons = self.geometry.ExPolygonCollection(api, 12)

        hull = api.convex_hull(7, polygon)
        polylines = api.expolygons_to_polylines(7, expolygons)

        self.assertIsInstance(hull, self.geometry.StoredPolygon)
        self.assertIsInstance(polylines, self.geometry.StoredPolylineCollection)
        self.assertEqual(hull.storage(), 7)
        self.assertEqual(polylines.storage(), 7)
        self.assertEqual(polygon.handle(), 11)
        self.assertEqual(expolygons.handle(), 12)
        hull_handle = hull.handle()
        polylines_handle = polylines.handle()
        self.assertTrue(hull.free_from_storage())
        self.assertTrue(polylines.free_from_storage())
        self.assertFalse(hull.free_from_storage())
        self.assertEqual(api.host.freed, [(7, hull_handle), (7, polylines_handle)])

    def test_polyline_clipping_and_bbox_helpers_own_their_results(self):
        api = self.make_api()
        subject = self.geometry.Polyline(api, 21)
        clip = self.geometry.ExPolygonCollection(api, 22)
        bbox = self.generated.CBoundingBox()
        context = api.clipper(9)

        outside = context.diff_polyline_expolygons(subject, clip)
        inside = context.intersection_polyline_expolygons(subject, clip)
        bounded = context.clip_expolygons_with_subject_bbox(clip, bbox)

        self.assertIsInstance(outside, self.geometry.StoredPolylineCollection)
        self.assertIsInstance(inside, self.geometry.StoredPolylineCollection)
        self.assertIsInstance(bounded, self.geometry.StoredExPolygonCollection)
        self.assertEqual([call[0] for call in api.host.calls], [
            "polyline_diff",
            "polyline_intersection",
            "clip_expolygons_bbox",
        ])
        self.assertIs(api.host.calls[-1][2], bbox)
        for result in (outside, inside, bounded):
            self.assertEqual(result.storage(), 9)
            self.assertTrue(result.free_from_storage())

    def test_helpers_reject_invalid_storage_api_and_native_results(self):
        api = self.make_api()
        other_api = self.make_api()
        polygon = self.geometry.Polygon(api, 31)
        subject = self.geometry.Polyline(api, 32)
        foreign_clip = self.geometry.ExPolygonCollection(other_api, 33)

        with self.assertRaises(ValueError):
            self.geometry.convex_hull(0, polygon)
        with self.assertRaises(ValueError):
            self.clipper.clipper_diff_polyline_expolygons(
                5, subject, foreign_clip
            )
        with self.assertRaises(ValueError):
            api.clipper(5).diff_polyline_expolygons(subject, foreign_clip)
        with self.assertRaises(ValueError):
            api.convex_hull(5, self.geometry.Polygon(other_api, 34))

        api.host.fail_next = True
        with self.assertRaises(ValueError):
            self.geometry.convex_hull(5, polygon)

    def test_polyline_difference_short_circuits_an_empty_subject(self):
        api = self.make_api()
        subject = self.geometry.Polyline(api, 41)
        clip = self.geometry.ExPolygonCollection(api, 42)

        result = self.clipper.clipper_diff_polyline_expolygons(6, subject, clip)

        self.assertIsInstance(result, self.geometry.StoredPolylineCollection)
        self.assertTrue(result.empty())
        self.assertFalse(any(call[0] == "polyline_diff" for call in api.host.calls))
        self.assertTrue(result.free_from_storage())


if __name__ == "__main__":
    unittest.main()
