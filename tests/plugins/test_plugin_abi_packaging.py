"""Offline ABI packaging checks, independent of the platform DLL loader."""
import importlib.util
import json
from pathlib import Path
import sys
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.dont_write_bytecode = True
sys.path.insert(0, str(ROOT / "src/plugins_cpp"))
sys.path.insert(0, str(ROOT / "src/libslic3r/Api/plugin/python"))
import write_plugin_abi_manifest as manifest
from slic3r_abi import ABI_HEADERS_ATTRIBUTE, abi_requires_headers


def load_python_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class AbiPackagingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        generator_path = ROOT / "src/libslic3r/Api/plugin/python/generate_slic3r_api.py"
        analyzer_path = ROOT / "src/libslic3r/Api/plugin/python/generate_python_plugin_abi.py"
        cls.generator = load_python_module("abi_generator", generator_path)
        cls.analyzer = load_python_module("python_plugin_abi_generator", analyzer_path)

    def test_python_abi_decorator_preserves_and_composes_targets(self):
        def binding():
            return 42

        original = binding
        binding = abi_requires_headers(" first.h ", "first.h")(binding)
        binding = abi_requires_headers("steps/second.h")(binding)
        self.assertIs(binding, original)
        self.assertEqual(binding(), 42)
        self.assertEqual(getattr(binding, ABI_HEADERS_ATTRIBUTE), ("first.h", "steps/second.h"))

        @abi_requires_headers("class.h")
        class Binding:
            @abi_requires_headers("method.h")
            def method(self):
                return 7

        self.assertEqual(getattr(Binding, ABI_HEADERS_ATTRIBUTE), ("class.h",))
        self.assertEqual(getattr(Binding.method, ABI_HEADERS_ATTRIBUTE), ("method.h",))
        self.assertEqual(Binding().method(), 7)
        with self.assertRaises(ValueError):
            abi_requires_headers()
        with self.assertRaises(ValueError):
            abi_requires_headers("")

    def test_union_and_failed_generation_never_publish_partial_output(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            binary, base, abi, output = (root / name for name in ("plugin.dll", "base.ini", "abi.ini", "version.ini"))
            binary.write_bytes(manifest.BEGIN + b"slic3r_plugin_types.h=1.0\nslic3r_extrusion_entity.h=0.0\n" + manifest.END)
            base.write_text("[plugin]\npackage_version=1.0.0\nslicer_version=999.0.0\n", encoding="utf-8")
            abi.write_text("[abi]\nslic3r_extrusion_entity.h=1.0\n", encoding="utf-8")
            command = [sys.executable, manifest.__file__, "--base", str(base), "--binary", str(binary),
                       "--abi", str(abi), "--output", str(output)]
            subprocess.run(command, check=True, capture_output=True)
            self.assertEqual(manifest.read_ini_abi(output), {
                "slic3r_plugin_types.h": (1, 0), "slic3r_extrusion_entity.h": (1, 0)})
            previous = output.read_bytes()
            binary.write_bytes(b"missing metadata")
            self.assertNotEqual(subprocess.run(command, capture_output=True).returncode, 0)
            self.assertEqual(output.read_bytes(), previous)

    def test_record_validation(self):
        good = manifest.BEGIN + b"slic3r_plugin_types.h=1u.0u\n" + manifest.END
        with tempfile.TemporaryDirectory() as directory:
            binary = Path(directory) / "plugin.dll"
            binary.write_bytes(b"not an executable" + good)
            self.assertEqual(manifest.read_binary_abi(binary), {"slic3r_plugin_types.h": (1, 0)})
            for bad in (b"", good + good, good[:-1], manifest.BEGIN + b"a.h=1.0\na.h=1.0\n" + manifest.END):
                binary.write_bytes(bad)
                with self.assertRaises(ValueError):
                    manifest.read_binary_abi(binary)

    def test_python_header_coverage(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "abi.ini"
            output.write_text(self.generator.generate_abi_manifest(), encoding="utf-8")
            versions = manifest.read_ini_abi(output)
            self.assertIn("slic3r_plugin_types.h", versions)
            for header in self.generator.header_paths():
                if re.search(r"^#define SLIC3R_PLUGIN_API_\w+_MAJOR\s+\d+", header.read_text(encoding="utf-8"), re.MULTILINE):
                    self.assertIn(header.relative_to(self.generator.HEADER_ROOT).as_posix(), versions)

            first = self.generator.generate_abi_catalog()
            self.assertEqual(first, self.generator.generate_abi_catalog())
            catalog = json.loads(first)
            self.assertIn("slic3r_extrusion_entity.h", catalog["c_functions"]["extrusion_is_loop"])
            self.assertIn("slic3r_geometry.h", catalog["python_symbols"]["CPoint"])
            self.assertIn("slic3r_plugin_types.h", catalog["python_symbols"]["PLUGIN_SETUP"])
            self.assertIn("slic3r_slicing_step.h", catalog["python_symbols"]["STEP_ORDERING"])
            self.assertEqual(
                catalog["headers"]["slic3r_extrusion_entity.h"]["dependencies"],
                sorted(catalog["headers"]["slic3r_extrusion_entity.h"]["dependencies"]))

    def test_high_level_helpers_declare_their_native_contracts(self):
        runtime = ROOT / "src/libslic3r/Api/plugin/python"
        with tempfile.TemporaryDirectory() as directory:
            catalog, _catalog_path = self._write_catalog(Path(directory))
            bindings = self.analyzer.build_binding_index(runtime, catalog)

            for method in (
                "path_count",
                "bounding_box",
            ):
                with self.subTest(method=method):
                    self.assertIn("slic3r_clipper.h", bindings.methods[method])

            for method in (
                "empty",
                "slice_to_polygons",
                "slice_to_expolygons",
                "is_model_part",
                "is_negative",
                "is_modifier",
                "is_support_enforcer",
                "is_support_blocker",
                "is_support_modifier",
                "is_seam_position",
                "is_brim",
                "is_fdm_support_painted",
                "is_seam_painted",
                "is_mm_painted",
            ):
                with self.subTest(method=method):
                    self.assertIn("slic3r_volume.h", bindings.methods[method])

            self.assertIn(
                "slic3r_volume.h",
                bindings.symbols["project_painting_to_polygons"],
            )

            for surface_type in (
                "MutableSurface",
                "StoredSurface",
                "StoredSurfaceCollection",
            ):
                with self.subTest(surface_type=surface_type):
                    self.assertIn(
                        "slic3r_data_tree.h",
                        bindings.symbols[surface_type],
                    )

            for surface_method in (
                "has_any_flag",
                "has_all_flags",
                "copy_properties_from",
                "append_like",
                "append_like_move",
                "mutable_at",
                "back_mutable",
            ):
                with self.subTest(surface_method=surface_method):
                    self.assertIn(
                        "slic3r_data_tree.h",
                        bindings.methods[surface_method],
                    )

            for data_tree_method in (
                "bridging_flow",
                "bottom_z",
                "copy_from",
                "curled_line",
                "curled_line_count",
                "curled_lines",
                "flow",
                "instance_count",
                "instance_shift",
                "object",
                "region",
                "region_count",
                "regions",
            ):
                with self.subTest(data_tree_method=data_tree_method):
                    self.assertIn(
                        "slic3r_data_tree.h",
                        bindings.methods[data_tree_method],
                    )

            self.assertIn("slic3r_geometry.h", bindings.symbols["LineDistancer"])
            for visitor in ("ExtrusionTreeConstVisitor", "ExtrusionTreeVisitor"):
                with self.subTest(visitor=visitor):
                    self.assertIn(
                        "slic3r_extrusion_entity.h",
                        bindings.symbols[visitor],
                    )
                    self.assertIn(
                        "slic3r_extrusion_property.h",
                        bindings.symbols[visitor],
                    )

            auxiliary_headers = bindings.symbols[
                "build_auxiliary_layer_regions_from_subject"
            ]
            for header in (
                "slic3r_clipper.h",
                "slic3r_config.h",
                "slic3r_config_option.h",
                "slic3r_data_tree.h",
                "slic3r_geometry.h",
                "slic3r_volume.h",
            ):
                with self.subTest(auxiliary_header=header):
                    self.assertIn(header, auxiliary_headers)

    def _write_catalog(self, root):
        catalog_path = root / "catalog.json"
        catalog_path.write_text(self.generator.generate_abi_catalog(), encoding="utf-8")
        return self.analyzer.AbiCatalog.load(catalog_path), catalog_path

    def test_python_plugin_analysis_combines_raw_symbols_wrappers_and_local_imports(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            runtime = root / "runtime"
            runtime.mkdir()
            catalog, _catalog_path = self._write_catalog(root)
            runtime.joinpath("slic3r_api.py").write_text(
                "from slic3r_abi import abi_requires_headers\n"
                "@abi_requires_headers('slic3r_config.h')\n"
                "def wrapped(): pass\n"
                "class First:\n"
                " @abi_requires_headers('slic3r_geometry.h')\n"
                " def touch(self): pass\n"
                "class Second:\n"
                " @abi_requires_headers('slic3r_extrusion_entity.h')\n"
                " def touch(self): pass\n",
                encoding="utf-8")
            runtime.joinpath("slic3r_geometry_views.py").write_text(
                "from slic3r_abi import abi_requires_headers\n"
                "@abi_requires_headers('slic3r_geometry.h')\n"
                "class GeometryView: pass\n",
                encoding="utf-8")
            runtime.joinpath("slic3r_abi.py").write_text("def abi_requires_headers(*headers): pass\n", encoding="utf-8")
            root.joinpath("helper.py").write_text(
                "from slic3r_api import CPoint\n"
                "def use_type(value: CPoint): return value\n", encoding="utf-8")
            root.joinpath("relative_helper.py").write_text(
                "from slic3r_api import STEP_ORDERING\n"
                "def ordering_step(): return STEP_ORDERING\n", encoding="utf-8")
            entry = root / "plugin.py"
            entry.write_text(
                "from slic3r_api import wrapped as wrapped_alias\n"
                "from slic3r_geometry_views import GeometryView\n"
                "from helper import use_type\n"
                "from .relative_helper import ordering_step\n"
                "def register_plugin(api):\n"
                " wrapped_alias()\n"
                " GeometryView()\n"
                " api.host.extrusion_is_loop(0)\n"
                " unknown.touch()\n",
                encoding="utf-8")

            bindings = self.analyzer.build_binding_index(runtime, catalog)
            requirements = self.analyzer.analyze_plugin(entry, root, catalog, bindings)
            self.assertTrue({
                "slic3r_plugin_types.h",
                "slic3r_config.h",
                "slic3r_geometry.h",
                "slic3r_extrusion_entity.h",
                "slic3r_slicing_step.h",
            }.issubset(requirements))

    def test_common_context_methods_require_only_the_common_run_context(self):
        runtime = ROOT / "src/libslic3r/Api/plugin/python"
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            catalog, _catalog_path = self._write_catalog(root)
            bindings = self.analyzer.build_binding_index(runtime, catalog)
            for method in (
                "plugin_storage",
                "is_cancelled",
                "report_warning",
                "report_error",
                "report_progress",
            ):
                with self.subTest(method=method):
                    self.assertEqual(
                        bindings.methods[method],
                        {"slic3r_plugin_run_context.h"},
                    )

    def test_ambiguous_dunder_init_call_is_ignored(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            runtime = root / "runtime"
            runtime.mkdir()
            runtime.joinpath("slic3r_api.py").write_text("", encoding="utf-8")
            catalog, _catalog_path = self._write_catalog(root)
            bindings = self.analyzer.build_binding_index(runtime, catalog)
            self.assertNotIn("__init__", bindings.methods)
            entry = root / "plugin.py"
            entry.write_text(
                "class Local:\n"
                " def __init__(self): pass\n"
                "def run(value):\n"
                " value.__init__()\n",
                encoding="utf-8",
            )
            self.assertEqual(
                self.analyzer.analyze_plugin(entry, root, catalog, bindings),
                {"slic3r_plugin_types.h"},
            )

    def test_python_plugin_dynamic_access_requires_an_explicit_override(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            runtime = root / "runtime"
            runtime.mkdir()
            runtime.joinpath("slic3r_api.py").write_text("", encoding="utf-8")
            runtime.joinpath("slic3r_abi.py").write_text("def abi_requires_headers(*headers): pass\n", encoding="utf-8")
            catalog, _catalog_path = self._write_catalog(root)
            bindings = self.analyzer.build_binding_index(runtime, catalog)
            entry = root / "plugin.py"
            entry.write_text("def run(api, name):\n getattr(api.host, name)()\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, r"plugin\.py:2:2: dynamic plugin API access"):
                self.analyzer.analyze_plugin(entry, root, catalog, bindings)

            entry.write_text(
                "from slic3r_abi import abi_requires_headers\n"
                "@abi_requires_headers('slic3r_geometry.h')\n"
                "def run(api, name):\n getattr(api.host, name)()\n",
                encoding="utf-8")
            requirements = self.analyzer.analyze_plugin(entry, root, catalog, bindings)
            self.assertIn("slic3r_geometry.h", requirements)

    def test_failed_python_analysis_does_not_replace_a_valid_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            runtime = root / "runtime"
            runtime.mkdir()
            runtime.joinpath("slic3r_api.py").write_text("", encoding="utf-8")
            runtime.joinpath("slic3r_abi.py").write_text("def abi_requires_headers(*headers): pass\n", encoding="utf-8")
            _catalog, catalog_path = self._write_catalog(root)
            entry = root / "plugin.py"
            entry.write_text("def run(api, name):\n getattr(api.host, name)()\n", encoding="utf-8")
            output = root / "abi.ini"
            output.write_text("previous valid manifest\n", encoding="utf-8")
            command = [
                sys.executable,
                str(ROOT / "src/libslic3r/Api/plugin/python/generate_python_plugin_abi.py"),
                "--entry", str(entry),
                "--source-root", str(root),
                "--runtime-root", str(runtime),
                "--catalog", str(catalog_path),
                "--output", str(output),
            ]
            result = subprocess.run(command, capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertRegex(result.stderr, r"plugin\.py:2:2: dynamic plugin API access")
            self.assertEqual(output.read_text(encoding="utf-8"), "previous valid manifest\n")

    def test_python_plugin_rejects_star_import_and_unknown_decorator_header(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            runtime = root / "runtime"
            runtime.mkdir()
            runtime.joinpath("slic3r_api.py").write_text("", encoding="utf-8")
            catalog, _catalog_path = self._write_catalog(root)
            bindings = self.analyzer.build_binding_index(runtime, catalog)
            entry = root / "plugin.py"
            entry.write_text("from slic3r_api import *\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "wildcard imports"):
                self.analyzer.analyze_plugin(entry, root, catalog, bindings)
            entry.write_text(
                "from slic3r_abi import abi_requires_headers\n"
                "@abi_requires_headers('missing.h')\n"
                "def run(): pass\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "unknown C ABI header 'missing.h'"):
                self.analyzer.analyze_plugin(entry, root, catalog, bindings)
            entry.write_text("from slic3r_api import missing_binding\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "unknown Python plugin API symbol 'missing_binding'"):
                self.analyzer.analyze_plugin(entry, root, catalog, bindings)

            runtime.joinpath("slic3r_api.py").write_text(
                "class Binding:\n"
                " def method_only(self): pass\n",
                encoding="utf-8",
            )
            bindings = self.analyzer.build_binding_index(runtime, catalog)
            entry.write_text("from slic3r_api import method_only\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "unknown Python plugin API symbol 'method_only'"):
                self.analyzer.analyze_plugin(entry, root, catalog, bindings)

    def test_repository_python_plugins_generate_targeted_manifests(self):
        runtime = ROOT / "src/libslic3r/Api/plugin/python"
        plugins = ROOT / "src/plugins_python"
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            catalog, _catalog_path = self._write_catalog(root)
            bindings = self.analyzer.build_binding_index(runtime, catalog)
            full_count = len(catalog.headers)
            entries = [
                (plugins / "examples/example_python_plugin.py", None),
                (plugins / "polyholes/polyholes_low_level.py", "steps/slic3r_step_post_slicing.h"),
                (plugins / "polyholes/polyholes_high_level.py", "steps/slic3r_step_post_slicing.h"),
                (plugins / "perimeter/simple_perimeter_generator.py", "steps/slic3r_step_perimeter.h"),
                (plugins / "perimeter/hairy_object.py", "steps/slic3r_step_post_perimeter.h"),
                (
                    plugins / "surface/initial_typed_surface_builder.py",
                    "steps/slic3r_step_surface_generation.h",
                ),
            ]
            for entry, expected_step in entries:
                with self.subTest(entry=entry.name):
                    requirements = self.analyzer.analyze_plugin(entry, plugins, catalog, bindings)
                    self.assertIn("slic3r_plugin_types.h", requirements)
                    self.assertLess(len(requirements), full_count)
                    if expected_step is not None:
                        self.assertIn(expected_step, requirements)

            perimeter_requirements = self.analyzer.analyze_plugin(
                plugins / "perimeter/simple_perimeter_generator.py",
                plugins,
                catalog,
                bindings,
            )
            self.assertIn("steps/slic3r_step_perimeter.h", perimeter_requirements)
            self.assertTrue({
                "steps/slic3r_step_layer_height.h",
                "steps/slic3r_step_post_infill.h",
                "steps/slic3r_step_post_perimeter.h",
                "steps/slic3r_step_post_slicing.h",
                "steps/slic3r_step_slicing.h",
                "steps/slic3r_step_surface_generation.h",
            }.isdisjoint(perimeter_requirements))


if __name__ == "__main__":
    unittest.main()
