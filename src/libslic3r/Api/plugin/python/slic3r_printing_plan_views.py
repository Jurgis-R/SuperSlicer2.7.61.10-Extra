#/|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
#/|/
#/|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#/|/

"""Python views over the host-owned PrintingPlan.

The printing plan is the ordered work copy consumed after slicing:

    PrintingPlan
      -> PrintingGroup
        -> PrintingLayerGroup
          -> PrintingToolGroup
            -> PrintingExtrusion

Every wrapper in this module is borrowed. It never frees its handle. Appending,
clearing, moving, or transferring an element may reallocate a native vector and
invalidate views previously obtained from that same parent. Fetch child views
again after structural mutations.

Read-only classes expose inspection and plugin properties. Their ``Mutable*``
subclasses expose structural changes only when a step context supplied a
mutable handle. Source Layers, Objects, and LayerRegionIslands remain owned by
the slicing data tree. A PrintingExtrusion instead owns its cloned root; editing
``mutable_root()`` changes only the plan copy.

Move operations consume the contents of a mutable extrusion entity but do not
take ownership of its handle. A ``StoredExtrusionEntity`` therefore remains a
valid, empty storage-owned object and must still be released normally.
"""

from __future__ import annotations

import ctypes
from typing import Iterator

from slic3r_abi import abi_requires_headers
from slic3r_datatree_views import (
    Layer,
    LayerRegionIsland,
    Object,
    PluginProperties,
)
from slic3r_extrusion_views import ExtrusionEntity, MutableExtrusionEntity


def _address(handle) -> int:
    """Return an integer address without acquiring ownership of the handle."""
    if handle is None:
        return 0
    if isinstance(handle, ctypes.c_void_p):
        return int(handle.value or 0)
    if hasattr(handle, "c_handle"):
        return _address(handle.c_handle())
    if hasattr(handle, "address"):
        return int(handle.address)
    return int(handle)


def _void_p(handle) -> ctypes.c_void_p:
    return ctypes.c_void_p(_address(handle))


def _require_handle(handle, name: str = "handle") -> int:
    address = _address(handle)
    if not address:
        raise ValueError(f"{name} must not be null")
    return address


def _require_index(idx: int, count: int, name: str) -> int:
    value = int(idx)
    if value < 0 or value >= count:
        raise IndexError(f"{name} index {value} is outside [0, {count})")
    return value


def _require_same_api(owner, value, name: str) -> None:
    if getattr(value, "api", None) is not owner.api:
        raise ValueError(f"{name} must use this API instance")


@abi_requires_headers("slic3r_data_tree.h")
class _ReadOnlyPlanProperties:
    """Borrow plan metadata without exposing native pointers or write operations.

    Typed reads return independent ctypes copies. The parent plan must remain
    alive; this view owns no native memory and must never be freed separately.
    """

    def __init__(self, api, handle) -> None:
        self.api = api
        self._properties = PluginProperties(api, handle)

    def valid(self) -> bool:
        return self._properties.valid()

    def count(self) -> int:
        return self._properties.count()

    def type_at(self, idx: int) -> int:
        return self._properties.type_at(idx)

    def types(self) -> list[int]:
        return self._properties.types()

    def has(self, payload_cls_or_type) -> bool:
        return self._properties.has(payload_cls_or_type)

    def data_size(self, payload_cls_or_type) -> int:
        return self._properties.data_size(payload_cls_or_type)

    def get(self, payload_cls_or_key):
        return self._properties.get(payload_cls_or_key)


@abi_requires_headers("slic3r_printing_plan.h")
class _PrintingPlanView:
    """Common non-owning handle behavior for all printing-plan views."""

    def __init__(self, api, handle) -> None:
        self.api = api
        self.address = _require_handle(handle)

    def c_handle(self) -> ctypes.c_void_p:
        return _void_p(self.address)

    def same_handle(self, other: "_PrintingPlanView") -> bool:
        return self.api is other.api and self.address == other.address

    def valid(self) -> bool:
        return self.address != 0

    def __bool__(self) -> bool:
        return self.valid()


@abi_requires_headers("slic3r_printing_plan.h")
class PrintingObjectInstance:
    """Value snapshot identifying one source Object instance in a print group.

    The small native structure is copied by value. ``object()`` is still a
    borrowed view of the host-owned Object and must not be freed or retained
    after the Print is destroyed. ``instance_idx()`` is context only: geometry
    cloned into the plan has already been shifted for that instance.
    """

    def __init__(self, api, value) -> None:
        self.api = api
        self._object_address = _require_handle(value.object, "object instance object")
        self._instance_idx = int(value.instance_idx)

    def object(self) -> Object:
        return Object(self.api, self._object_address)

    def instance_idx(self) -> int:
        return self._instance_idx


@abi_requires_headers("slic3r_printing_plan.h")
class PrintingScopeEvents(_PrintingPlanView):
    """Read-only view of the fixed before/after sequences of one plan scope.

    The sequence roots always exist, even when ``has_before()`` or
    ``has_after()`` is false. They are fixed roots and are exposed read-only;
    append complete event trees through ``MutablePrintingScopeEvents``.
    """

    def has_before(self) -> bool:
        return bool(self.api.host.printing_scope_events_has_before(self.c_handle()))

    def has_after(self) -> bool:
        return bool(self.api.host.printing_scope_events_has_after(self.c_handle()))

    def before(self) -> ExtrusionEntity:
        return ExtrusionEntity(
            self.api, self.api.host.printing_scope_events_get_before(self.c_handle())
        )

    def after(self) -> ExtrusionEntity:
        return ExtrusionEntity(
            self.api, self.api.host.printing_scope_events_get_after(self.c_handle())
        )


@abi_requires_headers("slic3r_printing_plan.h")
class MutablePrintingScopeEvents(PrintingScopeEvents):
    """Mutable borrowed scope-events view; appended roots become host-owned."""

    def mutable_c_handle(self) -> ctypes.c_void_p:
        return self.c_handle()

    def append_before_clone(self, event: ExtrusionEntity) -> MutableExtrusionEntity:
        """Append an independent copy; ``event`` remains unchanged and owned."""
        _require_same_api(self, event, "event")
        return MutableExtrusionEntity(
            self.api,
            self.api.host.printing_scope_events_append_before_clone(
                self.mutable_c_handle(), event.c_handle()
            ),
        )

    def append_before_move(self, event: MutableExtrusionEntity) -> MutableExtrusionEntity:
        """Move event content into the before sequence, leaving ``event`` empty."""
        _require_same_api(self, event, "event")
        return MutableExtrusionEntity(
            self.api,
            self.api.host.printing_scope_events_append_before_move(
                self.mutable_c_handle(), event.mutable_c_handle()
            ),
        )

    def append_after_clone(self, event: ExtrusionEntity) -> MutableExtrusionEntity:
        """Append an independent copy; ``event`` remains unchanged and owned."""
        _require_same_api(self, event, "event")
        return MutableExtrusionEntity(
            self.api,
            self.api.host.printing_scope_events_append_after_clone(
                self.mutable_c_handle(), event.c_handle()
            ),
        )

    def append_after_move(self, event: MutableExtrusionEntity) -> MutableExtrusionEntity:
        """Move event content into the after sequence, leaving ``event`` empty."""
        _require_same_api(self, event, "event")
        return MutableExtrusionEntity(
            self.api,
            self.api.host.printing_scope_events_append_after_move(
                self.mutable_c_handle(), event.mutable_c_handle()
            ),
        )


@abi_requires_headers("slic3r_printing_plan.h")
class PrintingExtrusion(_PrintingPlanView):
    """Read-only view of one cloned extrusion root and its source context."""

    def region_island(self) -> LayerRegionIsland | None:
        """Borrow the source island, or return None for auxiliary work."""
        handle = self.api.host.printing_extrusion_get_region_island(self.c_handle())
        return None if not _address(handle) else LayerRegionIsland(self.api, handle)

    def role(self) -> int:
        return int(self.api.host.printing_extrusion_get_role(self.c_handle()))

    def object_instance_idx(self) -> int:
        return int(
            self.api.host.printing_extrusion_get_object_instance_idx(self.c_handle())
        )

    def root(self) -> ExtrusionEntity:
        """Borrow the plan-owned cloned root without granting mutation."""
        return ExtrusionEntity(
            self.api, self.api.host.printing_extrusion_get_root(self.c_handle())
        )


@abi_requires_headers("slic3r_printing_plan.h")
class MutablePrintingExtrusion(PrintingExtrusion):
    """Mutable borrowed view of a PrintingExtrusion owned by its tool group."""

    def mutable_c_handle(self) -> ctypes.c_void_p:
        return self.c_handle()

    def set_role(self, role: int) -> None:
        self.api.host.printing_extrusion_set_role(self.mutable_c_handle(), int(role))

    def set_object_instance_idx(self, idx: int) -> None:
        if idx < 0 or idx > 0xFFFF:
            raise ValueError("object instance index must fit in uint16")
        self.api.host.printing_extrusion_set_object_instance_idx(
            self.mutable_c_handle(), int(idx)
        )

    def mutable_root(self) -> MutableExtrusionEntity:
        """Borrow the mutable plan clone; changes never alter the source island."""
        return MutableExtrusionEntity(
            self.api,
            self.api.host.printing_extrusion_get_root_mutable(self.mutable_c_handle()),
        )

    def set_root_clone(self, root: ExtrusionEntity | None) -> bool:
        """Replace the owned root with an independent copy, or an empty root."""
        if root is not None:
            _require_same_api(self, root, "root")
        return bool(
            self.api.host.printing_extrusion_set_root_clone(
                self.mutable_c_handle(), _void_p(root)
            )
        )

    def set_root_move(self, root: MutableExtrusionEntity) -> bool:
        """Move content into the plan; ``root`` remains owned but becomes empty."""
        _require_same_api(self, root, "root")
        return bool(
            self.api.host.printing_extrusion_set_root_move(
                self.mutable_c_handle(), root.mutable_c_handle()
            )
        )


@abi_requires_headers("slic3r_printing_plan.h")
class PrintingToolGroup(_PrintingPlanView):
    """Read-only view of one extruder visit at a single print Z."""

    def events(self) -> PrintingScopeEvents:
        return PrintingScopeEvents(
            self.api, self.api.host.printing_tool_group_get_events(self.c_handle())
        )

    def properties(self) -> _ReadOnlyPlanProperties:
        return _ReadOnlyPlanProperties(
            self.api, self.api.host.printing_tool_group_get_properties(self.c_handle())
        )

    def extruder_id(self) -> int:
        return int(self.api.host.printing_tool_group_get_extruder_id(self.c_handle()))

    def region_island_count(self) -> int:
        return int(
            self.api.host.printing_tool_group_count_region_island(self.c_handle())
        )

    def region_island(self, idx: int) -> LayerRegionIsland:
        index = _require_index(idx, self.region_island_count(), "region island")
        return LayerRegionIsland(
            self.api,
            self.api.host.printing_tool_group_get_region_island(
                self.c_handle(), index
            ),
        )

    def region_islands(self) -> Iterator[LayerRegionIsland]:
        """Iterate borrowed source islands in their current context order."""
        for idx in range(self.region_island_count()):
            yield self.region_island(idx)

    def extrusion_count(self) -> int:
        return int(self.api.host.printing_tool_group_count_extrusion(self.c_handle()))

    def extrusion(self, idx: int) -> PrintingExtrusion:
        index = _require_index(idx, self.extrusion_count(), "printing extrusion")
        return PrintingExtrusion(
            self.api,
            self.api.host.printing_tool_group_get_extrusion(self.c_handle(), index),
        )

    def extrusions(self) -> Iterator[PrintingExtrusion]:
        """Iterate borrowed plan roots in their current execution order."""
        for idx in range(self.extrusion_count()):
            yield self.extrusion(idx)

    def __len__(self) -> int:
        return self.extrusion_count()

    def __iter__(self) -> Iterator[PrintingExtrusion]:
        return self.extrusions()


@abi_requires_headers("slic3r_printing_plan.h")
class MutablePrintingToolGroup(PrintingToolGroup):
    """Mutable borrowed tool group; structural edits may invalidate children."""

    def properties(self) -> PluginProperties:
        """Borrow writable metadata only through this fully mutable plan view."""
        return PluginProperties(
            self.api, self.api.host.printing_tool_group_get_properties(self.c_handle())
        )

    def mutable_c_handle(self) -> ctypes.c_void_p:
        return self.c_handle()

    def events(self) -> MutablePrintingScopeEvents:
        return MutablePrintingScopeEvents(
            self.api,
            self.api.host.printing_tool_group_get_events_mutable(
                self.mutable_c_handle()
            ),
        )

    def set_extruder_id(self, extruder_id: int) -> None:
        if extruder_id < 0 or extruder_id > 0xFFFF:
            raise ValueError("extruder id must fit in uint16")
        self.api.host.printing_tool_group_set_extruder_id(
            self.mutable_c_handle(), int(extruder_id)
        )

    def append_region_island(self, region_island: LayerRegionIsland) -> None:
        """Append a non-owning source-context reference; geometry is not copied."""
        _require_same_api(self, region_island, "region island")
        self.api.host.printing_tool_group_append_region_island(
            self.mutable_c_handle(), region_island.c_handle()
        )

    def extrusion(self, idx: int) -> MutablePrintingExtrusion:
        index = _require_index(idx, self.extrusion_count(), "printing extrusion")
        return MutablePrintingExtrusion(
            self.api,
            self.api.host.printing_tool_group_get_extrusion_mutable(
                self.mutable_c_handle(), index
            ),
        )

    def extrusions(self) -> Iterator[MutablePrintingExtrusion]:
        for idx in range(self.extrusion_count()):
            yield self.extrusion(idx)

    def __iter__(self) -> Iterator[MutablePrintingExtrusion]:
        return self.extrusions()

    def append_extrusion_clone(
        self,
        source: LayerRegionIsland | None,
        role: int,
        root: ExtrusionEntity | None,
        object_instance_idx: int,
    ) -> MutablePrintingExtrusion:
        """Append a plan-owned copy of ``root``; None creates an empty root."""
        if source is not None:
            _require_same_api(self, source, "source region island")
        if root is not None:
            _require_same_api(self, root, "root")
        if object_instance_idx < 0 or object_instance_idx > 0xFFFF:
            raise ValueError("object instance index must fit in uint16")
        return MutablePrintingExtrusion(
            self.api,
            self.api.host.printing_tool_group_append_extrusion_clone(
                self.mutable_c_handle(),
                _void_p(source),
                int(role),
                _void_p(root),
                int(object_instance_idx),
            ),
        )

    def append_extrusion_clone_from_region_island(
        self,
        source: LayerRegionIsland,
        role: int,
        object_instance_idx: int,
    ) -> MutablePrintingExtrusion:
        """Clone one existing role bucket from a source LayerRegionIsland."""
        _require_same_api(self, source, "source region island")
        if object_instance_idx < 0 or object_instance_idx > 0xFFFF:
            raise ValueError("object instance index must fit in uint16")
        return MutablePrintingExtrusion(
            self.api,
            self.api.host.printing_tool_group_append_extrusion_clone_from_region_island(
                self.mutable_c_handle(),
                source.c_handle(),
                int(role),
                int(object_instance_idx),
            ),
        )

    def append_extrusion_move(
        self,
        source: LayerRegionIsland | None,
        role: int,
        root: MutableExtrusionEntity,
        object_instance_idx: int,
    ) -> MutablePrintingExtrusion:
        """Move ``root`` content into the plan while leaving its handle empty."""
        if source is not None:
            _require_same_api(self, source, "source region island")
        _require_same_api(self, root, "root")
        if object_instance_idx < 0 or object_instance_idx > 0xFFFF:
            raise ValueError("object instance index must fit in uint16")
        return MutablePrintingExtrusion(
            self.api,
            self.api.host.printing_tool_group_append_extrusion_move(
                self.mutable_c_handle(),
                _void_p(source),
                int(role),
                root.mutable_c_handle(),
                int(object_instance_idx),
            ),
        )

    def move_extrusion(self, from_idx: int, to_idx: int) -> bool:
        count = self.extrusion_count()
        source = _require_index(from_idx, count, "source printing extrusion")
        destination = _require_index(to_idx, count, "destination printing extrusion")
        return bool(
            self.api.host.printing_tool_group_move_extrusion(
                self.mutable_c_handle(), source, destination
            )
        )

    def transfer_extrusion_to(
        self, source_idx: int, destination: "MutablePrintingToolGroup"
    ) -> bool:
        """Move one extrusion to a sibling tool group in the same layer group."""
        _require_same_api(self, destination, "destination tool group")
        source = _require_index(
            source_idx, self.extrusion_count(), "source printing extrusion"
        )
        return bool(
            self.api.host.printing_tool_group_transfer_extrusion(
                self.mutable_c_handle(), source, destination.mutable_c_handle()
            )
        )


@abi_requires_headers("slic3r_printing_plan.h")
class PrintingLayerGroup(_PrintingPlanView):
    """Read-only view of all source layers printed at one scaled print Z."""

    def events(self) -> PrintingScopeEvents:
        return PrintingScopeEvents(
            self.api, self.api.host.printing_layer_group_get_events(self.c_handle())
        )

    def properties(self) -> _ReadOnlyPlanProperties:
        return _ReadOnlyPlanProperties(
            self.api,
            self.api.host.printing_layer_group_get_properties(self.c_handle()),
        )

    def print_z(self) -> int:
        return int(self.api.host.printing_layer_group_get_print_z(self.c_handle()))

    def layer_count(self) -> int:
        return int(self.api.host.printing_layer_group_count_layer(self.c_handle()))

    def layer(self, idx: int) -> Layer:
        index = _require_index(idx, self.layer_count(), "layer")
        return Layer(
            self.api,
            self.api.host.printing_layer_group_get_layer(self.c_handle(), index),
        )

    def layers(self) -> Iterator[Layer]:
        """Iterate borrowed source Layers; the plan never owns these objects."""
        for idx in range(self.layer_count()):
            yield self.layer(idx)

    def tool_group_count(self) -> int:
        return int(
            self.api.host.printing_layer_group_count_tool_group(self.c_handle())
        )

    def tool_group(self, idx: int) -> PrintingToolGroup:
        index = _require_index(idx, self.tool_group_count(), "tool group")
        return PrintingToolGroup(
            self.api,
            self.api.host.printing_layer_group_get_tool_group(
                self.c_handle(), index
            ),
        )

    def tool_groups(self) -> Iterator[PrintingToolGroup]:
        for idx in range(self.tool_group_count()):
            yield self.tool_group(idx)

    def __len__(self) -> int:
        return self.tool_group_count()

    def __iter__(self) -> Iterator[PrintingToolGroup]:
        return self.tool_groups()


@abi_requires_headers("slic3r_printing_plan.h")
class MutablePrintingLayerGroup(PrintingLayerGroup):
    """Mutable borrowed layer group; source Layer references stay non-owning."""

    def properties(self) -> PluginProperties:
        """Borrow writable metadata only through this fully mutable plan view."""
        return PluginProperties(
            self.api, self.api.host.printing_layer_group_get_properties(self.c_handle())
        )

    def mutable_c_handle(self) -> ctypes.c_void_p:
        return self.c_handle()

    def events(self) -> MutablePrintingScopeEvents:
        return MutablePrintingScopeEvents(
            self.api,
            self.api.host.printing_layer_group_get_events_mutable(
                self.mutable_c_handle()
            ),
        )

    def set_print_z(self, print_z: int) -> None:
        self.api.host.printing_layer_group_set_print_z(
            self.mutable_c_handle(), int(print_z)
        )

    def append_layer(self, layer: Layer) -> None:
        """Append a borrowed context reference; the Layer remains host-owned."""
        _require_same_api(self, layer, "layer")
        self.api.host.printing_layer_group_append_layer(
            self.mutable_c_handle(), layer.c_handle()
        )

    def tool_group(self, idx: int) -> MutablePrintingToolGroup:
        index = _require_index(idx, self.tool_group_count(), "tool group")
        return MutablePrintingToolGroup(
            self.api,
            self.api.host.printing_layer_group_get_tool_group_mutable(
                self.mutable_c_handle(), index
            ),
        )

    def tool_groups(self) -> Iterator[MutablePrintingToolGroup]:
        for idx in range(self.tool_group_count()):
            yield self.tool_group(idx)

    def __iter__(self) -> Iterator[MutablePrintingToolGroup]:
        return self.tool_groups()

    def append_tool_group(self, extruder_id: int) -> MutablePrintingToolGroup:
        if extruder_id < 0 or extruder_id > 0xFFFF:
            raise ValueError("extruder id must fit in uint16")
        return MutablePrintingToolGroup(
            self.api,
            self.api.host.printing_layer_group_append_tool_group(
                self.mutable_c_handle(), int(extruder_id)
            ),
        )

    def move_tool_group(self, from_idx: int, to_idx: int) -> bool:
        count = self.tool_group_count()
        source = _require_index(from_idx, count, "source tool group")
        destination = _require_index(to_idx, count, "destination tool group")
        return bool(
            self.api.host.printing_layer_group_move_tool_group(
                self.mutable_c_handle(), source, destination
            )
        )

    def remove_empty_tool_group(self, idx: int) -> bool:
        """Remove only a group with no extrusion, event, or plugin property."""
        index = _require_index(idx, self.tool_group_count(), "tool group")
        return bool(
            self.api.host.printing_layer_group_remove_empty_tool_group(
                self.mutable_c_handle(), index
            )
        )


@abi_requires_headers("slic3r_printing_plan.h")
class LayerExtrusionEditPrintingExtrusion(PrintingExtrusion):
    """Restricted extrusion view for ``STEP_LAYER_EXTRUSION_EDIT``.

    The step may mutate the cloned extrusion root but must not replace it or
    alter its source role and object-instance metadata. The returned root is
    borrowed from the plan and must never be freed. Structural changes inside
    that root are allowed by the step contract.
    """

    def mutable_root(self) -> MutableExtrusionEntity:
        return MutableExtrusionEntity(
            self.api,
            self.api.host.printing_extrusion_get_root_mutable(self.c_handle()),
        )


@abi_requires_headers("slic3r_printing_plan.h")
class LayerExtrusionEditPrintingToolGroup(PrintingToolGroup):
    """Tool-group view that permits root editing but no group mutation."""

    def extrusion(self, idx: int) -> LayerExtrusionEditPrintingExtrusion:
        index = _require_index(idx, self.extrusion_count(), "printing extrusion")
        return LayerExtrusionEditPrintingExtrusion(
            self.api,
            self.api.host.printing_tool_group_get_extrusion_mutable(
                self.c_handle(), index
            ),
        )

    def extrusions(self) -> Iterator[LayerExtrusionEditPrintingExtrusion]:
        """Iterate extrusions whose cloned roots may be edited in this job."""
        for idx in range(self.extrusion_count()):
            yield self.extrusion(idx)

    def __iter__(self) -> Iterator[LayerExtrusionEditPrintingExtrusion]:
        return self.extrusions()


@abi_requires_headers("slic3r_printing_plan.h")
class LayerExtrusionEditPrintingLayerGroup(PrintingLayerGroup):
    """Restricted layer-group view for parallel layer extrusion editing.

    The layer-group hierarchy, print Z, events, properties, source layers and
    tool-group order are read-only. Descending through ``tool_group()`` grants
    mutable access only to each plan-owned cloned extrusion root. This keeps
    parallel jobs from reallocating shared plan vectors or changing scheduling
    metadata while another job is running.
    """

    def tool_group(self, idx: int) -> LayerExtrusionEditPrintingToolGroup:
        index = _require_index(idx, self.tool_group_count(), "tool group")
        return LayerExtrusionEditPrintingToolGroup(
            self.api,
            self.api.host.printing_layer_group_get_tool_group_mutable(
                self.c_handle(), index
            ),
        )

    def tool_groups(self) -> Iterator[LayerExtrusionEditPrintingToolGroup]:
        for idx in range(self.tool_group_count()):
            yield self.tool_group(idx)

    def __iter__(self) -> Iterator[LayerExtrusionEditPrintingToolGroup]:
        return self.tool_groups()


@abi_requires_headers("slic3r_printing_plan.h")
class PrintingGroup(_PrintingPlanView):
    """Read-only view of one independent batch of printable work."""

    def events(self) -> PrintingScopeEvents:
        return PrintingScopeEvents(
            self.api, self.api.host.printing_group_get_events(self.c_handle())
        )

    def properties(self) -> _ReadOnlyPlanProperties:
        return _ReadOnlyPlanProperties(
            self.api, self.api.host.printing_group_get_properties(self.c_handle())
        )

    def object_instance_count(self) -> int:
        return int(
            self.api.host.printing_group_count_object_instance(self.c_handle())
        )

    def object_instance(self, idx: int) -> PrintingObjectInstance:
        index = _require_index(idx, self.object_instance_count(), "object instance")
        return PrintingObjectInstance(
            self.api,
            self.api.host.printing_group_get_object_instance(self.c_handle(), index),
        )

    def object_instances(self) -> Iterator[PrintingObjectInstance]:
        for idx in range(self.object_instance_count()):
            yield self.object_instance(idx)

    def layer_group_count(self) -> int:
        return int(self.api.host.printing_group_count_layer_group(self.c_handle()))

    def layer_group(self, idx: int) -> PrintingLayerGroup:
        index = _require_index(idx, self.layer_group_count(), "layer group")
        return PrintingLayerGroup(
            self.api,
            self.api.host.printing_group_get_layer_group(self.c_handle(), index),
        )

    def layer_groups(self) -> Iterator[PrintingLayerGroup]:
        for idx in range(self.layer_group_count()):
            yield self.layer_group(idx)

    def __len__(self) -> int:
        return self.layer_group_count()

    def __iter__(self) -> Iterator[PrintingLayerGroup]:
        return self.layer_groups()


@abi_requires_headers("slic3r_printing_plan.h")
class MutablePrintingGroup(PrintingGroup):
    """Mutable borrowed print batch; structural edits invalidate child views."""

    def properties(self) -> PluginProperties:
        """Borrow writable metadata only through this fully mutable plan view."""
        return PluginProperties(
            self.api, self.api.host.printing_group_get_properties(self.c_handle())
        )

    def mutable_c_handle(self) -> ctypes.c_void_p:
        return self.c_handle()

    def events(self) -> MutablePrintingScopeEvents:
        return MutablePrintingScopeEvents(
            self.api,
            self.api.host.printing_group_get_events_mutable(self.mutable_c_handle()),
        )

    def clear(self) -> None:
        """Remove all group content, events, context references and properties."""
        self.api.host.printing_group_clear(self.mutable_c_handle())

    def append_object_instance(self, obj: Object, instance_idx: int) -> None:
        """Append a borrowed source Object reference and its instance index."""
        _require_same_api(self, obj, "object")
        if instance_idx < 0 or instance_idx > 0xFFFFFFFFFFFFFFFF:
            raise ValueError("instance index must fit in uint64")
        self.api.host.printing_group_append_object_instance(
            self.mutable_c_handle(), obj.c_handle(), int(instance_idx)
        )

    def layer_group(self, idx: int) -> MutablePrintingLayerGroup:
        index = _require_index(idx, self.layer_group_count(), "layer group")
        return MutablePrintingLayerGroup(
            self.api,
            self.api.host.printing_group_get_layer_group_mutable(
                self.mutable_c_handle(), index
            ),
        )

    def layer_groups(self) -> Iterator[MutablePrintingLayerGroup]:
        for idx in range(self.layer_group_count()):
            yield self.layer_group(idx)

    def __iter__(self) -> Iterator[MutablePrintingLayerGroup]:
        return self.layer_groups()

    def append_layer_group(self, print_z: int) -> MutablePrintingLayerGroup:
        return MutablePrintingLayerGroup(
            self.api,
            self.api.host.printing_group_append_layer_group(
                self.mutable_c_handle(), int(print_z)
            ),
        )

    def move_layer_group(self, from_idx: int, to_idx: int) -> bool:
        count = self.layer_group_count()
        source = _require_index(from_idx, count, "source layer group")
        destination = _require_index(to_idx, count, "destination layer group")
        return bool(
            self.api.host.printing_group_move_layer_group(
                self.mutable_c_handle(), source, destination
            )
        )


@abi_requires_headers("slic3r_printing_plan.h")
class PrintingPlan(_PrintingPlanView):
    """Read-only borrowed top-level plan in its current execution order."""

    def events(self) -> PrintingScopeEvents:
        return PrintingScopeEvents(
            self.api, self.api.host.printing_plan_get_events(self.c_handle())
        )

    def properties(self) -> _ReadOnlyPlanProperties:
        return _ReadOnlyPlanProperties(
            self.api, self.api.host.printing_plan_get_properties(self.c_handle())
        )

    def group_count(self) -> int:
        return int(self.api.host.printing_plan_count_group(self.c_handle()))

    def group(self, idx: int) -> PrintingGroup:
        index = _require_index(idx, self.group_count(), "printing group")
        return PrintingGroup(
            self.api, self.api.host.printing_plan_get_group(self.c_handle(), index)
        )

    def groups(self) -> Iterator[PrintingGroup]:
        for idx in range(self.group_count()):
            yield self.group(idx)

    def __len__(self) -> int:
        return self.group_count()

    def __iter__(self) -> Iterator[PrintingGroup]:
        return self.groups()


@abi_requires_headers("slic3r_printing_plan.h")
class MutablePrintingPlan(PrintingPlan):
    """Mutable borrowed PrintingPlan supplied by a compatible step payload."""

    def properties(self) -> PluginProperties:
        """Borrow writable metadata only through this fully mutable plan view."""
        return PluginProperties(
            self.api, self.api.host.printing_plan_get_properties(self.c_handle())
        )

    def mutable_c_handle(self) -> ctypes.c_void_p:
        return self.c_handle()

    def events(self) -> MutablePrintingScopeEvents:
        return MutablePrintingScopeEvents(
            self.api,
            self.api.host.printing_plan_get_events_mutable(self.mutable_c_handle()),
        )

    def clear(self) -> None:
        """Remove all groups, events and properties before rebuilding the plan."""
        self.api.host.printing_plan_clear(self.mutable_c_handle())

    def group(self, idx: int) -> MutablePrintingGroup:
        index = _require_index(idx, self.group_count(), "printing group")
        return MutablePrintingGroup(
            self.api,
            self.api.host.printing_plan_get_group_mutable(
                self.mutable_c_handle(), index
            ),
        )

    def groups(self) -> Iterator[MutablePrintingGroup]:
        for idx in range(self.group_count()):
            yield self.group(idx)

    def __iter__(self) -> Iterator[MutablePrintingGroup]:
        return self.groups()

    def append_group(self) -> MutablePrintingGroup:
        return MutablePrintingGroup(
            self.api, self.api.host.printing_plan_append_group(self.mutable_c_handle())
        )

    def move_group(self, from_idx: int, to_idx: int) -> bool:
        count = self.group_count()
        source = _require_index(from_idx, count, "source printing group")
        destination = _require_index(to_idx, count, "destination printing group")
        return bool(
            self.api.host.printing_plan_move_group(
                self.mutable_c_handle(), source, destination
            )
        )


__all__ = [
    "LayerExtrusionEditPrintingExtrusion",
    "LayerExtrusionEditPrintingLayerGroup",
    "LayerExtrusionEditPrintingToolGroup",
    "MutablePrintingExtrusion",
    "MutablePrintingGroup",
    "MutablePrintingLayerGroup",
    "MutablePrintingPlan",
    "MutablePrintingScopeEvents",
    "MutablePrintingToolGroup",
    "PrintingExtrusion",
    "PrintingGroup",
    "PrintingLayerGroup",
    "PrintingObjectInstance",
    "PrintingPlan",
    "PrintingScopeEvents",
    "PrintingToolGroup",
]
