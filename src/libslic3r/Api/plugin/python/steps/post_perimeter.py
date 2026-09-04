#/|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
#/|/
#/|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#/|/

"""
Python helper for STEP_POST_PERIMETER plugins.

Post-perimeter plugins run after perimeter generation has attached extrusion
trees and fill areas to each LayerRegionIsland. They are allowed to edit the
perimeter extrusion buckets in place and, when needed, replace the island fill
areas that the surface-generation step will consume.

Context contents
----------------

PostPerimeterContext exposes:

* read-only post_perimeter_print() and post_perimeter_object() views; topology
  is not edited directly here;
* post_perimeter_mutable_extrusion(), the callback that borrows one extrusion bucket from a
  LayerRegionIsland for a specific role;
* set_island_fill_areas() and set_island_fill_free_areas(), which replace the
  island-level areas consumed by surface generation;
* project_painting_to_polygons(), which projects a generic facet-painting key
  into one polygon collection per object layer;
* plugin_storage(), cancellation, progress, warning, and error helpers.

Typical use
-----------

    ctx = api.post_perimeter_context(run_ctx_address)
    for layer in ctx.post_perimeter_object().layers():
        for island in layer.islands():
            for region_island in island.region_islands():
                root = ctx.post_perimeter_mutable_extrusion(region_island, RAW_EXTRUSION_ROLE_PERIMETER)
                if root is not None:
                    root.disable_sort()

Handles returned by this module are borrowed from the host and are valid only
for the current PluginBase.run() call.
"""

from __future__ import annotations

import ctypes

from slic3r_api_generated import (
    RAW_FACET_PAINTING_ENFORCER,
    RunCtxPostPerimeterGeneration,
    PluginRunContext,
    STEP_POST_PERIMETER,
)
from slic3r_datatree_views import LayerIsland, LayerRegionIsland, Object, Print
from slic3r_extrusion_views import MutableExtrusionEntity
from slic3r_geometry_views import StoredExPolygonCollection, StoredPolygonCollection
from slic3r_abi import abi_requires_headers
from slic3r_run_context import PluginRunContextView


def _address(handle) -> int:
    if handle is None:
        return 0
    if isinstance(handle, ctypes.c_void_p):
        return int(handle.value or 0)
    if hasattr(handle, "c_handle"):
        return _address(handle.c_handle())
    return int(handle)


def _void_p(handle) -> ctypes.c_void_p:
    return ctypes.c_void_p(_address(handle))


def _as_bytes(text: str) -> bytes:
    return text.encode("utf-8")


@abi_requires_headers("steps/slic3r_step_post_perimeter.h")
class PostPerimeterContext(PluginRunContextView):
    """
    High-level wrapper around the STEP_POST_PERIMETER payload.

    The context exposes the current print/object, the explicit callback that
    grants mutable access to one extrusion bucket, and helpers for projecting
    generic facet painting into per-layer polygon collections.
    """

    def __init__(self, api, common: PluginRunContext, payload_ptr) -> None:
        super().__init__(api, common)
        self._payload_ptr = payload_ptr
        self.payload = payload_ptr.contents

    @classmethod
    def from_post_perimeter_run_context(cls, api, run_ctx_address: int) -> "PostPerimeterContext | None":
        if not run_ctx_address:
            return None
        common = PluginRunContext.from_address(int(run_ctx_address))
        if common.step != STEP_POST_PERIMETER or not common.data:
            return None
        payload_ptr = ctypes.cast(common.data, ctypes.POINTER(RunCtxPostPerimeterGeneration))
        payload = payload_ptr.contents
        if not payload.print or not payload.object:
            return None
        return cls(api, common, payload_ptr)

    def post_perimeter_print(self) -> Print:
        return Print(self.api, self.payload.print)

    def post_perimeter_object(self) -> Object:
        return Object(self.api, self.payload.object)

    def post_perimeter_mutable_extrusion(
        self, region_island: LayerRegionIsland, role: int
    ) -> MutableExtrusionEntity | None:
        if not self.payload.get_region_island_mutable_extrusion:
            return None
        handle = self.payload.get_region_island_mutable_extrusion(region_island.c_handle(), int(role))
        if not handle:
            return None
        return MutableExtrusionEntity(self.api, handle)

    def set_island_fill_areas(self, island: LayerIsland, areas: StoredExPolygonCollection | None) -> bool:
        if not self.payload.set_island_fill_areas:
            return False
        areas_handle = None if areas is None else areas.c_handle()
        return bool(self.payload.set_island_fill_areas(island.c_handle(), areas_handle))

    def set_island_fill_free_areas(self, island: LayerIsland, areas: StoredExPolygonCollection | None) -> bool:
        if not self.payload.set_island_fill_free_areas:
            return False
        areas_handle = None if areas is None else areas.c_handle()
        return bool(self.payload.set_island_fill_free_areas(island.c_handle(), areas_handle))

    def project_painting_to_polygons(
        self,
        paint_key: str,
        painting_value: int = RAW_FACET_PAINTING_ENFORCER,
    ) -> list[StoredPolygonCollection]:
        """
        Project one generic facet painting kind onto every object layer.

        The returned StoredPolygonCollection objects are owned by plugin
        storage. Keep them alive as long as Clipper operands reference them.
        Empty collections simply mean that the layer has no matching painted
        facets.
        """
        obj = self.post_perimeter_object()
        storage = self.plugin_storage()
        layer_count = obj.layer_count()
        by_layer = [StoredPolygonCollection(self.api, storage) for _ in range(layer_count)]
        raw_array = (ctypes.c_void_p * layer_count)(*[collection.c_handle() for collection in by_layer])
        self.api.host.object_project_painting_to_polygons(
            obj.c_handle(),
            _as_bytes(paint_key),
            int(painting_value),
            raw_array,
            int(layer_count),
        )
        return by_layer


__all__ = ["PostPerimeterContext"]
