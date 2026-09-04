#/|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
#/|/
#/|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#/|/

"""
Python helper for STEP_POST_INFILL plugins.

Post-infill plugins run after infill pattern generation has written extrusion
trees into each LayerRegionIsland. They are cleanup or finishing passes: gap
fill, ironing preparation, local extrusion edits, or other algorithms that need
the final infill geometry as input.

Context contents
----------------

PostInfillContext exposes:

* read-only post_infill_print() and post_infill_object() views;
* data-tree helpers such as LayerIsland.get_or_create_region_island(), used
  after the plugin has resolved the destination extruder;
* post_infill_mutable_extrusion(), the explicit callback that borrows one infill-owned
  extrusion bucket from a LayerRegionIsland;
* plugin_storage(), cancellation, progress, warning, and error helpers.

Typical use
-----------

    ctx = api.post_infill_context(run_ctx_address)
    for layer in ctx.post_infill_object().layers():
        for island in layer.islands():
            region_island = island.get_or_create_region_island(list(island.regions()), extruder_id=0)
            if region_island is not None:
                root = ctx.post_infill_mutable_extrusion(region_island, RAW_EXTRUSION_ROLE_INTERNAL_INFILL)
                if root is not None:
                    # Edit the generated infill tree in place.
                    root.disable_reverse()

The mutable handle is borrowed from the host and remains valid only for the
current PluginBase.run() call. For infill, gap-fill, and ironing roles the
callback returns a root collection, creating an empty root when needed. It
returns None for roles owned by another step, such as perimeter or support
roles.
"""

from __future__ import annotations

import ctypes

from slic3r_api_generated import (
    PluginRunContext,
    RunCtxPostInfillGeneration,
    STEP_POST_INFILL,
)
from slic3r_datatree_views import LayerRegionIsland, Object, Print
from slic3r_extrusion_views import MutableExtrusionEntity
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


@abi_requires_headers("steps/slic3r_step_post_infill.h")
class PostInfillContext(PluginRunContextView):
    """
    High-level wrapper around the STEP_POST_INFILL payload.

    The context keeps normal data-tree traversal read-only, then exposes the
    one host-approved mutation for this step: borrowing a mutable extrusion root
    for infill, gap-fill, or ironing output already present on a
    LayerRegionIsland.
    """

    def __init__(self, api, common: PluginRunContext, payload_ptr) -> None:
        super().__init__(api, common)
        self._payload_ptr = payload_ptr
        self.payload = payload_ptr.contents

    @classmethod
    def from_post_infill_run_context(cls, api, run_ctx_address: int) -> "PostInfillContext | None":
        if not run_ctx_address:
            return None
        common = PluginRunContext.from_address(int(run_ctx_address))
        if common.step != STEP_POST_INFILL or not common.data:
            return None
        payload_ptr = ctypes.cast(common.data, ctypes.POINTER(RunCtxPostInfillGeneration))
        payload = payload_ptr.contents
        if not payload.print or not payload.object:
            return None
        if not payload.get_region_island_mutable_extrusion:
            return None
        return cls(api, common, payload_ptr)

    def post_infill_print(self) -> Print:
        return Print(self.api, self.payload.print)

    def post_infill_object(self) -> Object:
        return Object(self.api, self.payload.object)

    def post_infill_mutable_extrusion(
        self, region_island: LayerRegionIsland, role: int
    ) -> MutableExtrusionEntity | None:
        handle = self.payload.get_region_island_mutable_extrusion(region_island.c_handle(), int(role))
        if not handle:
            return None
        return MutableExtrusionEntity(self.api, handle)


__all__ = ["PostInfillContext"]
