#/|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
#/|/
#/|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#/|/

"""
Python helper for the post-slicing plugin step.

The post-slicing step receives the current print and object, plus callbacks for
editing layer-region slices. Normal plugin code should create this context from
the run context address passed to PluginBase.run_impl(), then use the view
objects returned here instead of manually casting the raw payload.

Context contents
----------------

PostSlicingContext exposes:

* mutable post_slicing_print() and post_slicing_object() views because this
  step is allowed to edit the freshly produced layer slices;
* post_slicing_mutable_layer(), plus borrowed layer, LayerRegion, and
  LayerIsland polygon views for targeted edits;
* assign_islands_by_moving_contents(), for replacing a layer's islands from a
  storage-owned ExPolygon collection;
* recompute_slices_from_islands() and
  recompute_slices_and_islands_from_layer_region(), the host callbacks that
  rebuild derived caches after polygon edits;
* plugin_storage(), temporary storage cleanup, cancellation, progress, warning,
  and error helpers.

Typical use
-----------

    ctx = api.post_slicing_context(run_ctx_address)
    if ctx is None:
        return

    obj = ctx.post_slicing_object()
    for layer_idx in range(obj.layer_count()):
        layer = obj.layer_mutable(layer_idx)
        for region_idx in range(layer.region_count()):
            region = layer.region_mutable(region_idx)
            slices = ctx.post_slicing_borrow_layer_region_slices(region)
            # edit slices here
        ctx.recompute_slices_and_islands_from_layer_region(layer)

After editing a layer region's raw slices, call
recompute_slices_and_islands_from_layer_region() on the owning layer so the host
can rebuild its derived slice/island caches.
"""

from __future__ import annotations

import ctypes

from slic3r_api_generated import (
    PluginRunContext,
    RunCtxPostSlicing,
    STEP_POST_SLICING,
)
from slic3r_datatree_views import MutableLayer, MutableLayerIsland, MutableLayerRegion, MutableObject, MutablePrint
from slic3r_geometry_views import ExPolygonCollection, MutableExPolygonCollection, MutableExPolygon
from slic3r_abi import abi_requires_headers
from slic3r_run_context import PluginRunContextView


def _address(handle) -> int:
    if handle is None:
        return 0
    if isinstance(handle, ctypes.c_void_p):
        return int(handle.value or 0)
    return int(handle)


def _void_p(handle) -> ctypes.c_void_p:
    return ctypes.c_void_p(_address(handle))


# High-level wrapper around the STEP_POST_SLICING payload.
@abi_requires_headers("steps/slic3r_step_post_slicing.h")
class PostSlicingContext(PluginRunContextView):
    def __init__(self, api, common: PluginRunContext, payload: RunCtxPostSlicing) -> None:
        super().__init__(api, common)
        self.payload = payload

    @classmethod
    def from_post_slicing_run_context(cls, api, run_ctx_address: int) -> "PostSlicingContext | None":
        if not run_ctx_address:
            return None
        common = PluginRunContext.from_address(int(run_ctx_address))
        if common.step != STEP_POST_SLICING or not common.data:
            return None
        payload = ctypes.cast(common.data, ctypes.POINTER(RunCtxPostSlicing)).contents
        if not payload.print or not payload.object:
            return None
        return cls(api, common, payload)

    def post_slicing_print(self) -> MutablePrint:
        return MutablePrint(self.api, self.payload.print)

    def post_slicing_object(self) -> MutableObject:
        return MutableObject(self.api, self.payload.object)

    def post_slicing_mutable_layer(self, idx: int) -> MutableLayer:
        """
        Borrow one mutable object layer through the step callback.

        The object handle in the payload is const, so this callback is the
        explicit permission granted by STEP_POST_SLICING to edit a layer.
        """
        if not self.payload.object_borrow_mutable_layer:
            raise RuntimeError("STEP_POST_SLICING did not expose mutable layer access.")
        return MutableLayer(self.api, self.payload.object_borrow_mutable_layer(self.payload.object, int(idx)))

    def borrow_layer_slices(self, layer: MutableLayer) -> MutableExPolygonCollection:
        handle = self.payload.layer_borrow_mutable_slices(layer.mutable_c_handle())
        return MutableExPolygonCollection(self.api, handle)

    def post_slicing_borrow_layer_region_slices(
        self, layer_region: MutableLayerRegion
    ) -> MutableExPolygonCollection:
        handle = self.payload.layer_region_borrow_mutable_slices(layer_region.mutable_c_handle())
        return MutableExPolygonCollection(self.api, handle)

    def borrow_layer_island_slice(self, island: MutableLayerIsland) -> MutableExPolygon:
        handle = self.payload.layer_island_borrow_mutable_slice(island.mutable_c_handle())
        return MutableExPolygon(self.api, handle)

    def assign_islands_by_moving_contents(self, layer: MutableLayer, islands: ExPolygonCollection) -> None:
        """
        Replace a layer's islands from a mutable ExPolygonCollection.

        The host moves contents out of islands. Use a storage-owned collection
        when you want to transfer ownership, then consider the source empty.
        """
        self.payload.layer_assign_islands_by_moving_contents(layer.mutable_c_handle(), islands.c_handle())

    def recompute_slices_from_islands(self, layer: MutableLayer) -> None:
        self.payload.layer_recompute_slices_from_islands(layer.mutable_c_handle())

    def recompute_slices_and_islands_from_layer_region(self, layer: MutableLayer) -> None:
        self.payload.layer_recompute_slices_and_islands_from_layer_region(layer.mutable_c_handle())


__all__ = ["PostSlicingContext"]
