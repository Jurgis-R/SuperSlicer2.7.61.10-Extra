#/|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
#/|/
#/|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#/|/

"""
Python views over the SuperSlicer print data tree.

The data tree is the host-owned hierarchy a plugin usually receives through a
step payload:

    Print -> Object -> Layer -> LayerIsland / LayerRegion -> LayerRegionIsland

The wrappers here keep raw C handles out of normal plugin code. Use read-only
views for borrowed const handles, and Mutable* views only when a step payload or
host callback explicitly gives a mutable handle.

Typical use
-----------

Read layers and islands:

    obj = api.object(ctx.object)
    for layer in obj.layers():
        for island in layer.islands():
            bbox = island.bounding_box()

Read configuration:

    config = obj.config()
    density = config.float("fill_density")
    enabled = config.bool("support_material")

Inspect region-island extrusions:

    region_island = island.region_island(0)
    if region_island.has_extrusion(RAW_EXTRUSION_ROLE_PERIMETER):
        extrusion = region_island.extrusion(RAW_EXTRUSION_ROLE_PERIMETER)
        print(extrusion.length())

Property guide: [Using Plugin Properties](/doc/plugins/properties.md)

Extrusion guide: [Using Unified Extrusion Entities](/doc/plugins/extrusions.md)
"""

from __future__ import annotations

import ctypes
import math
from typing import Iterable, Iterator, NamedTuple, Sequence

from slic3r_api_generated import (
    CLayerAdhesionProperty,
    CLayerBrimProperty,
    CLayerSupportProperty,
    CFlow,
    CFloatOrPercent,
    CMatrix4d,
    CMeshSlicingParams,
    CSurface,
    CTriangleIndices,
    PLUGIN_PROPERTY_TYPE_LAYER_ADHESION,
    PLUGIN_PROPERTY_TYPE_LAYER_BRIM,
    PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT,
    RAW_SURFACE_TYPE_DENS_SOLID,
    RAW_SURFACE_TYPE_DENS_SPARSE,
    RAW_SURFACE_TYPE_DENS_VOID,
    RAW_SURFACE_TYPE_MOD_BRIDGE,
    RAW_SURFACE_TYPE_MOD_OVERBRIDGE,
    RAW_SURFACE_TYPE_NONE,
    RAW_SURFACE_TYPE_POS_BOTTOM,
    RAW_SURFACE_TYPE_POS_INTERNAL,
    RAW_SURFACE_TYPE_POS_PERIMETER,
    RAW_SURFACE_TYPE_POS_TOP,
    RAW_FACET_PAINTING_FDM_SUPPORT,
    RAW_FACET_PAINTING_MMU_SEGMENTATION,
    RAW_FACET_PAINTING_SEAM,
    RAW_MESH_SLICING_MODE_EVEN_ODD,
    RAW_MESH_SLICING_MODE_POSITIVE,
    RAW_MESH_SLICING_MODE_REGULAR,
    RAW_VOLUME_TYPE_BRIM_NEGATIVE,
    RAW_VOLUME_TYPE_BRIM_PATCH,
    RAW_VOLUME_TYPE_MODEL_PART,
    RAW_VOLUME_TYPE_NEGATIVE_VOLUME,
    RAW_VOLUME_TYPE_PARAMETER_MODIFIER,
    RAW_VOLUME_TYPE_SEAM_POSITION_CENTER,
    RAW_VOLUME_TYPE_SEAM_POSITION_CENTER_Z,
    RAW_VOLUME_TYPE_SEAM_POSITION_INSIDE,
    RAW_VOLUME_TYPE_SEAM_POSITION_INSIDE_CENTER,
    RAW_VOLUME_TYPE_SUPPORT_BLOCKER,
    RAW_VOLUME_TYPE_SUPPORT_ENFORCER,
    EPSILON,
    SCALING_FACTOR,
)

from slic3r_clipper_views import ClipperContext
from slic3r_extrusion_views import ExtrusionEntity, MutableExtrusionEntity
from slic3r_geometry_views import (
    CPoint,
    ExPolygon,
    ExPolygonCollection,
    StoredExPolygonCollection,
    StoredPolygonCollection,
)
from slic3r_abi import abi_requires_headers


def _address(handle) -> int:
    if handle is None:
        return 0
    if isinstance(handle, ctypes.c_void_p):
        return int(handle.value or 0)
    return int(handle)


def _void_p(handle) -> ctypes.c_void_p:
    return ctypes.c_void_p(_address(handle))


def _require_handle(handle, name: str = "handle") -> int:
    address = _address(handle)
    if not address:
        raise ValueError(f"{name} must not be null")
    return address


def _as_bytes(text: str) -> bytes:
    return text.encode("utf-8")


def _optional(cls, api, handle):
    return None if not _address(handle) else cls(api, handle)


def _decode_const_string(ptr) -> str:
    if not ptr:
        return ""
    return ctypes.cast(ptr, ctypes.c_char_p).value.decode("utf-8", errors="replace")


def _payload_type(payload_cls_or_type) -> int:
    if isinstance(payload_cls_or_type, int):
        return int(payload_cls_or_type)
    return int(payload_cls_or_type.property_type)


def _payload_copy(payload_cls, ptr: int):
    if not ptr:
        return None
    return payload_cls.from_buffer_copy(ctypes.string_at(_void_p(ptr), ctypes.sizeof(payload_cls)))


def _payload_mutable(payload_cls, ptr: int):
    if not ptr:
        return None
    return ctypes.cast(_void_p(ptr), ctypes.POINTER(payload_cls)).contents


def unscaled(value: int | float) -> float:
    return float(value) * SCALING_FACTOR


CLayerSupportProperty.property_type = PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT
LayerSupportProperty = CLayerSupportProperty
CLayerBrimProperty.property_type = PLUGIN_PROPERTY_TYPE_LAYER_BRIM
LayerBrimProperty = CLayerBrimProperty
CLayerAdhesionProperty.property_type = PLUGIN_PROPERTY_TYPE_LAYER_ADHESION
LayerAdhesionProperty = CLayerAdhesionProperty


# Base for borrowed data-tree views. It never owns or frees the handle.
@abi_requires_headers("slic3r_data_tree.h")
class DataTreeView:
    def __init__(self, api, handle) -> None:
        self.api = api
        self.address = _require_handle(handle)

    def c_handle(self) -> ctypes.c_void_p:
        return _void_p(self.address)

    def same_handle(self, other: "DataTreeView") -> bool:
        return self.address == other.address

    def __bool__(self) -> bool:
        return self.address != 0


class PluginProperties:
    """
    Generic plugin-property view for data-tree objects.

    Properties are small C payloads indexed by numeric property ids. A borrowed
    const Layer or Surface may still return a mutable property container: this
    only allows metadata annotations and does not make geometry mutable.

    Use has()/get() for read-only code:

        support = layer.properties().get(CLayerSupportProperty)
        if support is not None:
            print(support.interface_id)

    Use get_or_add() only when the current step intentionally annotates the
    object. Custom plugin properties must have been registered with the
    orchestrator before they are created.
    """

    def __init__(self, api, handle) -> None:
        self.api = api
        self.address = _address(handle)

    def c_handle(self) -> ctypes.c_void_p:
        return _void_p(self.address)

    def valid(self) -> bool:
        return self.address != 0

    def count(self) -> int:
        return int(self.api.host.plugin_property_count(self.c_handle()))

    def type_at(self, idx: int) -> int:
        return int(self.api.host.plugin_property_type_at(self.c_handle(), int(idx)))

    def types(self) -> list[int]:
        return [self.type_at(idx) for idx in range(self.count())]

    def has(self, payload_cls_or_type) -> bool:
        return bool(self.api.host.plugin_property_has(self.c_handle(), _payload_type(payload_cls_or_type)))

    def data_size(self, payload_cls_or_type) -> int:
        return int(self.api.host.plugin_property_data_size(self.c_handle(), _payload_type(payload_cls_or_type)))

    def data_address(self, payload_cls_or_type) -> int:
        return _address(self.api.host.plugin_property_data(self.c_handle(), _payload_type(payload_cls_or_type)))

    def data_mutable_address(self, payload_cls_or_type) -> int:
        return _address(self.api.host.plugin_property_data_mutable(self.c_handle(), _payload_type(payload_cls_or_type)))

    def get(self, payload_cls):
        if self.data_size(payload_cls) != ctypes.sizeof(payload_cls):
            return None
        return _payload_copy(payload_cls, self.data_address(payload_cls))

    def get_mutable(self, payload_cls):
        if self.data_size(payload_cls) != ctypes.sizeof(payload_cls):
            return None
        return _payload_mutable(payload_cls, self.data_mutable_address(payload_cls))

    def get_or_add(self, payload_cls, orchestrator=None):
        orch = self.api.orchestrator if orchestrator is None else _void_p(orchestrator)
        ptr = self.api.host.plugin_property_get_or_add_data_mutable(
            orch, self.c_handle(), _payload_type(payload_cls)
        )
        return _payload_mutable(payload_cls, _address(ptr))

    def remove(self, payload_cls_or_type) -> bool:
        return bool(self.api.host.plugin_property_remove(self.c_handle(), _payload_type(payload_cls_or_type)))

    def clear(self) -> None:
        self.api.host.plugin_property_clear(self.c_handle())


# Read-only view over one ConfigOption. It covers common scalar/vector access
# without exposing raw config_option_handle values.
class ConfigOption(DataTreeView):
    def type(self) -> int:
        return int(self.api.host.config_option_type_get(self.c_handle()))

    def flags(self) -> int:
        return int(self.api.host.config_option_flags_get(self.c_handle()))

    def size(self) -> int:
        return int(self.api.host.config_option_size(self.c_handle()))

    def is_scalar(self) -> bool:
        return bool(self.api.host.config_option_is_scalar(self.c_handle()))

    def is_vector(self) -> bool:
        return bool(self.api.host.config_option_is_vector(self.c_handle()))

    def is_enabled(self, idx: int = 0) -> bool:
        return bool(self.api.host.config_option_is_enabled(self.c_handle(), int(idx)))

    def can_be_disabled(self) -> bool:
        return bool(self.api.host.config_option_can_be_disabled(self.c_handle()))

    def is_phony(self) -> bool:
        return bool(self.api.host.config_option_is_phony(self.c_handle()))

    def get_bool(self, idx: int = 0) -> bool:
        return bool(self.api.host.config_option_get_bool(self.c_handle(), int(idx)))

    def get_int(self, idx: int = 0) -> int:
        return int(self.api.host.config_option_get_int(self.c_handle(), int(idx)))

    def get_float(self, idx: int = 0) -> float:
        return float(self.api.host.config_option_get_float(self.c_handle(), int(idx)))

    def get_float_or_percent(self, idx: int = 0) -> CFloatOrPercent:
        return self.api.host.config_option_get_float_or_percent(self.c_handle(), int(idx))

    def is_percent(self, idx: int = 0) -> bool:
        return bool(self.get_float_or_percent(idx).percent)

    def get_effective_value(self, ratio_over: float, idx: int = 0) -> float:
        value = self.get_float_or_percent(idx)
        return float(self.api.host.c_float_or_percent_get_effective_value(ctypes.byref(value), float(ratio_over)))

    def serialize(self) -> str:
        needed = int(self.api.host.config_option_serialize(self.c_handle(), None, 0))
        if needed == 0:
            return ""
        out = ctypes.create_string_buffer(needed + 1)
        written = int(self.api.host.config_option_serialize(self.c_handle(), out, len(out)))
        return out.value[:written].decode("utf-8", errors="replace")

    def vector_empty(self) -> bool:
        vector = self.api.host.config_option_vector_cast(self.c_handle())
        return not vector or bool(self.api.host.config_option_vector_empty(vector))

    def vector_serialize_at(self, idx: int) -> str:
        vector = self.api.host.config_option_vector_cast(self.c_handle())
        if not vector:
            return ""
        needed = int(self.api.host.config_option_vector_serialize_at(vector, int(idx), None, 0))
        if needed == 0:
            return ""
        out = ctypes.create_string_buffer(needed + 1)
        written = int(self.api.host.config_option_vector_serialize_at(vector, int(idx), out, len(out)))
        return out.value[:written].decode("utf-8", errors="replace")


# Mutable config option. Use only on a MutableConfig result; changing options may
# require the host step to invalidate/recompute dependent data.
class MutableConfigOption(ConfigOption):
    def mutable_c_handle(self) -> ctypes.c_void_p:
        return self.c_handle()

    def set_enabled(self, enabled: bool, idx: int = 0) -> bool:
        return bool(self.api.host.config_option_set_enabled(self.mutable_c_handle(), int(enabled), int(idx)))

    def set_can_be_disabled(self, enabled: bool) -> bool:
        return bool(self.api.host.config_option_set_can_be_disabled(self.mutable_c_handle(), int(enabled)))

    def set_phony(self, phony: bool) -> bool:
        return bool(self.api.host.config_option_set_phony(self.mutable_c_handle(), int(phony)))

    def copy_from(self, src: ConfigOption) -> None:
        self.api.host.config_option_copy(self.mutable_c_handle(), src.c_handle())

    def deserialize(self, text: str, append: bool = False) -> bool:
        return bool(self.api.host.config_option_deserialize(self.mutable_c_handle(), _as_bytes(text), int(append)))

    def set_int(self, value: int, idx: int = 0) -> None:
        self.api.host.config_option_set_int(self.mutable_c_handle(), int(value), int(idx))

    def set_float(self, value: float, idx: int = 0) -> None:
        self.api.host.config_option_set_float(self.mutable_c_handle(), float(value), int(idx))

    def set_float_or_percent(self, value: CFloatOrPercent, idx: int = 0) -> None:
        self.api.host.config_option_set_float_or_percent(self.mutable_c_handle(), value, int(idx))

    def set_bool(self, value: bool, idx: int = 0) -> None:
        self.api.host.config_option_set_bool(self.mutable_c_handle(), int(value), int(idx))

    def set_string(self, value: str, idx: int = 0) -> None:
        self.api.host.config_option_set_string(self.mutable_c_handle(), _as_bytes(value), int(idx))

    def resize(self, size: int) -> None:
        vector = self.api.host.config_option_vector_cast_mutable(self.mutable_c_handle())
        if not vector:
            raise TypeError("Cannot resize a scalar configuration option")
        self.api.host.config_option_vector_resize(vector, int(size), None)

    def clear_vector(self) -> None:
        vector = self.api.host.config_option_vector_cast_mutable(self.mutable_c_handle())
        if not vector:
            raise TypeError("Cannot clear a scalar configuration option as a vector")
        self.api.host.config_option_vector_clear(vector)


# Borrowed read-only config view. get() returns None for an unknown key.
class Config(DataTreeView):
    def keys(self) -> list[str]:
        raw = self.api.host.config_keys(self.c_handle())
        out: list[str] = []
        for idx in range(raw.size):
            out.append(_decode_const_string(raw.items[idx]))
        return out

    def get(self, key: str) -> ConfigOption | None:
        return _optional(ConfigOption, self.api, self.api.host.config_get(self.c_handle(), _as_bytes(key)))

    def option(self, key: str) -> ConfigOption:
        option = self.get(key)
        if option is None:
            raise KeyError(key)
        return option

    def bool(self, key: str, idx: int = 0, default: bool = False) -> bool:
        option = self.get(key)
        return default if option is None else option.get_bool(idx)

    def int(self, key: str, idx: int = 0, default: int = 0) -> int:
        option = self.get(key)
        return default if option is None else option.get_int(idx)

    def float(self, key: str, idx: int = 0, default: float = 0.0) -> float:
        option = self.get(key)
        return default if option is None else option.get_float(idx)

    def float_or_percent(self, key: str, idx: int = 0) -> CFloatOrPercent | None:
        option = self.get(key)
        return None if option is None else option.get_float_or_percent(idx)

    def serialize_all(self) -> str:
        """Serialize every option into the host's versioned SCFG format."""
        needed = int(self.api.host.config_serialize_all(self.c_handle(), None, 0))
        if needed == 0:
            raise RuntimeError("The configuration could not be serialized")
        output = ctypes.create_string_buffer(needed + 1)
        written = int(self.api.host.config_serialize_all(self.c_handle(), output, len(output)))
        if written != needed:
            raise RuntimeError("The configuration changed while it was being serialized")
        return output.value[:written].decode("utf-8")


# Mutable config view. Only use it when the host explicitly exposes a mutable
# config handle to this plugin step.
class MutableConfig(Config):
    def mutable_c_handle(self) -> ctypes.c_void_p:
        return self.c_handle()

    def get_mutable(self, key: str) -> MutableConfigOption | None:
        return _optional(MutableConfigOption, self.api, self.api.host.config_get_mutable(self.mutable_c_handle(), _as_bytes(key)))

    def mutable_option(self, key: str) -> MutableConfigOption:
        option = self.get_mutable(key)
        if option is None:
            raise KeyError(key)
        return option

    def get_or_add(self, key: str, option_type: int) -> MutableConfigOption:
        handle = self.api.host.config_get_or_add_mutable(
            self.mutable_c_handle(), _as_bytes(key), int(option_type))
        option = _optional(MutableConfigOption, self.api, handle)
        if option is None:
            raise ValueError(f"Cannot create configuration option {key!r} with type {option_type}")
        return option

    def clear(self) -> None:
        if not self.api.host.config_clear(self.mutable_c_handle()):
            raise RuntimeError("The configuration cannot be cleared")

    def deserialize_all(self, serialized: str) -> None:
        """Atomically merge one SCFG document into this dynamic config."""
        if not self.api.host.config_deserialize_all(self.mutable_c_handle(), _as_bytes(serialized)):
            raise ValueError("The serialized configuration is invalid")


# Owned mutable DynamicConfig allocated in a storage_handle. The storage must
# outlive this object and must not be cleared until close() has released it.
class StoredConfig(MutableConfig):
    def __init__(self, api, storage) -> None:
        self._storage = _require_handle(storage, "storage")
        handle = api.host.storage_new_config(_void_p(self._storage))
        super().__init__(api, handle)
        self._owns_handle = True

    def close(self) -> bool:
        if not self._owns_handle or not self.address:
            return False
        freed = bool(self.api.host.storage_free(_void_p(self._storage), self.c_handle()))
        if freed:
            self.address = 0
            self._owns_handle = False
        return freed

    def release(self) -> int:
        handle = self.address
        self.address = 0
        self._owns_handle = False
        return handle

    def __enter__(self) -> "StoredConfig":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass


# Small builder for raw_surface_type. It mirrors the intended order:
# position -> density -> optional modifiers.
class SurfaceTypeBuilder:
    def __init__(self, value: int = RAW_SURFACE_TYPE_NONE) -> None:
        self.value = int(value)

    def top(self) -> "SurfaceTypeBuilder":
        return SurfaceTypeBuilder(self.value | RAW_SURFACE_TYPE_POS_TOP)

    def bottom(self) -> "SurfaceTypeBuilder":
        return SurfaceTypeBuilder(self.value | RAW_SURFACE_TYPE_POS_BOTTOM)

    def internal(self) -> "SurfaceTypeBuilder":
        return SurfaceTypeBuilder(self.value | RAW_SURFACE_TYPE_POS_INTERNAL)

    def perimeter(self) -> "SurfaceTypeBuilder":
        return SurfaceTypeBuilder(self.value | RAW_SURFACE_TYPE_POS_PERIMETER)

    def solid(self) -> "SurfaceTypeBuilder":
        return SurfaceTypeBuilder(self.value | RAW_SURFACE_TYPE_DENS_SOLID)

    def sparse(self) -> "SurfaceTypeBuilder":
        return SurfaceTypeBuilder(self.value | RAW_SURFACE_TYPE_DENS_SPARSE)

    def empty(self) -> "SurfaceTypeBuilder":
        return SurfaceTypeBuilder(self.value | RAW_SURFACE_TYPE_DENS_VOID)

    def bridge(self) -> "SurfaceTypeBuilder":
        return SurfaceTypeBuilder(self.value | RAW_SURFACE_TYPE_MOD_BRIDGE)

    def overbridge(self) -> "SurfaceTypeBuilder":
        return SurfaceTypeBuilder(self.value | RAW_SURFACE_TYPE_MOD_OVERBRIDGE)

    def raw(self) -> int:
        return self.value

    def __int__(self) -> int:
        return self.value


def srf_type() -> SurfaceTypeBuilder:
    return SurfaceTypeBuilder()


# Borrowed surface view. It can either wrap a host surface_handle or a c_surface
# snapshot returned by the ABI.
class Surface:
    def __init__(self, api, handle=None, surface: CSurface | None = None) -> None:
        if handle is None and surface is None:
            raise ValueError("surface handle or c_surface is required")
        self.api = api
        self.address = _address(handle)
        self._surface = surface

    @classmethod
    def from_c_surface(cls, api, surface: CSurface) -> "Surface":
        return cls(api, surface=surface)

    def has_handle(self) -> bool:
        return self.address != 0

    def c_handle(self) -> ctypes.c_void_p:
        return _void_p(self.address)

    def c_view(self) -> CSurface:
        if self.has_handle():
            return self.api.host.surface_c_view(self.c_handle())
        return self._surface

    def expolygon(self) -> ExPolygon:
        handle = self.api.host.surface_get_expolygon(self.c_handle()) if self.has_handle() else self._surface.expolygon
        return ExPolygon(self.api, handle)

    def type(self) -> int:
        return int(self.api.host.surface_get_type(self.c_handle())) if self.has_handle() else int(self._surface.type)

    def id(self) -> int:
        return int(self.api.host.surface_get_id(self.c_handle())) if self.has_handle() else int(self._surface.id)

    def properties(self) -> PluginProperties:
        if not self.has_handle():
            return PluginProperties(self.api, None)
        return PluginProperties(self.api, self.api.host.surface_get_properties(self.c_handle()))

    def has_flag(self, flag: int) -> bool:
        if self.has_handle():
            return bool(self.api.host.surface_get_flag(self.c_handle(), int(flag)))
        return (self.type() & int(flag)) != 0


# Borrowed read-only surface collection.
class SurfaceCollection(DataTreeView):
    def size(self) -> int:
        return int(self.api.host.surface_collection_size(self.c_handle()))

    def empty(self) -> bool:
        return self.size() == 0

    def at(self, idx: int) -> Surface:
        return Surface(self.api, self.api.host.surface_collection_at(self.c_handle(), int(idx)))

    def front(self) -> Surface:
        if self.empty():
            raise IndexError("empty surface collection")
        return self.at(0)

    def back(self) -> Surface:
        if self.empty():
            raise IndexError("empty surface collection")
        return self.at(self.size() - 1)

    def __len__(self) -> int:
        return self.size()

    def __iter__(self) -> Iterator[Surface]:
        for idx in range(self.size()):
            yield self.at(idx)

    def __getitem__(self, idx: int) -> Surface:
        if idx < 0:
            idx += self.size()
        if idx < 0 or idx >= self.size():
            raise IndexError(idx)
        return self.at(idx)


def _validated_mesh_slice_z_values(z_mm_by_layer: Iterable[float]) -> list[float]:
    """Materialize finite float-compatible heights in native slicing order."""
    z_values: list[float] = []
    for idx, value in enumerate(z_mm_by_layer):
        try:
            z_value = float(value)
            native_z = ctypes.c_float(z_value).value
        except (TypeError, ValueError, OverflowError) as error:
            raise ValueError(f"mesh slice Z value at index {idx} is not a finite number") from error
        if not math.isfinite(z_value) or not math.isfinite(native_z):
            raise ValueError(f"mesh slice Z value at index {idx} must be finite")
        if z_values and z_value < z_values[-1]:
            raise ValueError(
                "mesh slice Z values must be sorted in non-decreasing order "
                f"(index {idx}: {z_value} follows {z_values[-1]})"
            )
        z_values.append(z_value)
    return z_values


@abi_requires_headers("slic3r_volume.h")
class TriangleMesh(DataTreeView):
    """
    Borrowed read-only triangle mesh view.

    Mesh vertices use unscaled millimeters, unlike most 2D geometry in the
    slicing data tree. Slicing plugins usually get this through Volume.mesh().
    """

    def vertex_count(self) -> int:
        return int(self.api.host.triangle_mesh_vertex_count(self.c_handle()))

    def triangle_count(self) -> int:
        return int(self.api.host.triangle_mesh_triangle_count(self.c_handle()))

    def empty(self) -> bool:
        """
        Return whether the borrowed mesh contains no triangles.

        Vertices without triangles do not form printable mesh geometry and are
        therefore considered empty. This check does not validate manifoldness
        or repair the mesh, and it never modifies or frees the host-owned mesh.
        """
        return self.triangle_count() == 0

    def vertex_at(self, idx: int):
        return self.api.host.triangle_mesh_vertex_at(self.c_handle(), int(idx))

    def triangle_at(self, idx: int) -> CTriangleIndices:
        return self.api.host.triangle_mesh_triangle_at(self.c_handle(), int(idx))

    def slice_to_polygons(
        self,
        storage_address: int,
        transform: CMatrix4d,
        z_mm_by_layer: Iterable[float],
        params: CMeshSlicingParams | None = None,
    ) -> list[StoredPolygonCollection]:
        """
        Slice this mesh into one flat PolygonCollection per requested Z.

        ``transform`` and every Z use unscaled model-space millimeters. Convert
        a scaled ``Layer.slice_z()`` with ``unscaled()`` before passing it here.
        Heights must be sorted in non-decreasing order because the native mesh
        slicer performs binary searches over them. Repeated heights are allowed.
        With no ``params``, the host uses regular slicing and the supplied
        transform. When ``params`` is supplied, its transform and slicing modes
        are authoritative; the separate ``transform`` argument is ignored.

        Each returned collection owns a handle allocated in ``storage_address``.
        Keep that storage alive, and call ``free_from_storage()`` on every
        collection, or manage each one with a ``with`` block, before clearing
        the storage. Polygons borrowed from a returned collection become invalid
        when that collection is freed or modified. The method reads the mesh
        only and never changes host model data.
        """
        z_values = _validated_mesh_slice_z_values(z_mm_by_layer)
        if not z_values:
            return []

        output = [StoredPolygonCollection(self.api, storage_address) for _unused in z_values]
        z_array = (ctypes.c_float * len(z_values))(*z_values)
        output_array = (ctypes.c_void_p * len(output))(
            *[collection.c_handle() for collection in output]
        )
        if params is None:
            self.api.host.triangle_mesh_slice_to_polygons(
                self.c_handle(), transform, z_array, output_array, len(output)
            )
        else:
            self.api.host.triangle_mesh_slice_to_polygons_with_params(
                self.c_handle(), ctypes.byref(params), z_array, output_array, len(output)
            )
        return output

    def slice_to_expolygons(
        self,
        storage_address: int,
        transform: CMatrix4d,
        z_mm_by_layer: Iterable[float],
        params: CMeshSlicingParams | None = None,
    ) -> list[StoredExPolygonCollection]:
        """
        Slice this mesh into topology-aware ExPolygonCollections.

        Coordinate, parameter and ownership rules are identical to
        ``slice_to_polygons()``. Prefer this form when contours and holes must
        remain associated. Every returned collection is allocated in the
        caller's ``storage_address`` and must be released with
        ``free_from_storage()``, or managed with a ``with`` block, before that
        storage is cleared. This read-only operation never changes the borrowed
        mesh.
        """
        z_values = _validated_mesh_slice_z_values(z_mm_by_layer)
        if not z_values:
            return []

        output = [
            StoredExPolygonCollection(self.api, storage_address)
            for _unused in z_values
        ]
        z_array = (ctypes.c_float * len(z_values))(*z_values)
        output_array = (ctypes.c_void_p * len(output))(
            *[collection.c_handle() for collection in output]
        )
        if params is None:
            self.api.host.triangle_mesh_slice_to_expolygons(
                self.c_handle(), transform, z_array, output_array, len(output)
            )
        else:
            self.api.host.triangle_mesh_slice_to_expolygons_with_params(
                self.c_handle(), ctypes.byref(params), z_array, output_array, len(output)
            )
        return output


@abi_requires_headers("slic3r_volume.h")
class Volume(DataTreeView):
    """
    Borrowed read-only model volume view.

    Volumes are the raw model inputs used by STEP_SLICING. They expose the mesh,
    transform and volume-level config needed to decide how a slicing plugin
    should write layer-region polygons.
    """

    def type(self) -> int:
        return int(self.api.host.volume_get_type(self.c_handle()))

    def is_model_part(self) -> bool:
        """Return whether this volume contributes normal printable material."""
        return self.type() == RAW_VOLUME_TYPE_MODEL_PART

    def is_negative(self) -> bool:
        """Return whether this volume subtracts material from model parts."""
        return self.type() == RAW_VOLUME_TYPE_NEGATIVE_VOLUME

    def is_modifier(self) -> bool:
        """Return whether this volume changes settings without adding material."""
        return self.type() == RAW_VOLUME_TYPE_PARAMETER_MODIFIER

    def is_support_enforcer(self) -> bool:
        """Return whether this volume marks areas where support is enforced."""
        return self.type() == RAW_VOLUME_TYPE_SUPPORT_ENFORCER

    def is_support_blocker(self) -> bool:
        """Return whether this volume marks areas where support is blocked."""
        return self.type() == RAW_VOLUME_TYPE_SUPPORT_BLOCKER

    def is_support_modifier(self) -> bool:
        """Return whether this is either kind of support-control volume."""
        return self.is_support_enforcer() or self.is_support_blocker()

    def is_seam_position(self) -> bool:
        """
        Return whether this volume is one of the seam-position helper types.

        This classification only describes the volume's immutable model role;
        it does not calculate a seam or inspect painted seam facets.
        """
        return self.type() in {
            RAW_VOLUME_TYPE_SEAM_POSITION_CENTER,
            RAW_VOLUME_TYPE_SEAM_POSITION_CENTER_Z,
            RAW_VOLUME_TYPE_SEAM_POSITION_INSIDE_CENTER,
            RAW_VOLUME_TYPE_SEAM_POSITION_INSIDE,
        }

    def is_brim(self) -> bool:
        """Return whether this is a positive or negative brim-control volume."""
        return self.type() in {RAW_VOLUME_TYPE_BRIM_PATCH, RAW_VOLUME_TYPE_BRIM_NEGATIVE}

    def id(self) -> int:
        return int(self.api.host.volume_get_id(self.c_handle()))

    def config(self) -> Config:
        return Config(self.api, self.api.host.volume_get_config(self.c_handle()))

    def extruder_id(self) -> int:
        return int(self.api.host.volume_get_extruder_id(self.c_handle()))

    def matrix(self) -> CMatrix4d:
        return self.api.host.volume_get_matrix(self.c_handle())

    def matrix_no_offset(self) -> CMatrix4d:
        return self.api.host.volume_get_matrix_no_offset(self.c_handle())

    def has_painting(self, paint_key: str) -> bool:
        """
        Return whether this borrowed volume has facets for ``paint_key``.

        The key must be a built-in painting key or a generic annotation key
        registered with the orchestrator. This method reads host-owned facet
        metadata and neither copies nor modifies it.
        """
        return bool(self.api.host.volume_has_painting(self.c_handle(), _as_bytes(paint_key)))

    def is_fdm_support_painted(self) -> bool:
        """Return whether the volume contains FDM support painting."""
        return self.has_painting(RAW_FACET_PAINTING_FDM_SUPPORT)

    def is_seam_painted(self) -> bool:
        """Return whether the volume contains seam painting."""
        return self.has_painting(RAW_FACET_PAINTING_SEAM)

    def is_mm_painted(self) -> bool:
        """Return whether the volume contains multi-material painting."""
        return self.has_painting(RAW_FACET_PAINTING_MMU_SEGMENTATION)

    def mesh(self) -> TriangleMesh:
        """
        Return a borrowed, read-only view of this volume's triangle mesh.

        The mesh remains owned by the host and is valid only while this volume
        and its model object remain alive. Do not free the returned handle.
        """
        return TriangleMesh(self.api, self.api.host.volume_get_mesh(self.c_handle()))


# Borrowed print-region view.
class PrintRegion(DataTreeView):
    def config(self) -> Config:
        return Config(self.api, self.api.host.print_region_get_config(self.c_handle()))


class MutablePrintRegion(PrintRegion):
    def mutable_c_handle(self) -> ctypes.c_void_p:
        return self.c_handle()

    def config_mutable(self) -> MutableConfig:
        return MutableConfig(self.api, self.api.host.print_region_get_config_mutable(self.mutable_c_handle()))


# Borrowed layer-region view.
class LayerRegion(DataTreeView):
    def flow(self, role: int) -> CFlow:
        return self.api.host.layer_region_get_flow(self.c_handle(), int(role))

    def properties(self) -> PluginProperties:
        return PluginProperties(self.api, self.api.host.layer_region_get_properties(self.c_handle()))

    def slices(self) -> ExPolygonCollection:
        return ExPolygonCollection(self.api, self.api.host.layer_region_get_slices(self.c_handle()))

    def bounding_box(self):
        return self.api.host.layer_region_get_bounding_box(self.c_handle())

    def print_region(self) -> PrintRegion:
        return PrintRegion(self.api, self.api.host.layer_region_get_print_region(self.c_handle()))

    def layer(self) -> "Layer":
        return Layer(self.api, self.api.host.layer_region_get_layer(self.c_handle()))


class MutableLayerRegion(LayerRegion):
    def mutable_c_handle(self) -> ctypes.c_void_p:
        return self.c_handle()


# Borrowed layer-region island view. It is the usual entry point for extrusion
# trees attached to one island/region pair.
class LayerRegionIsland(DataTreeView):
    def extruder_id(self) -> int:
        return int(self.api.host.layer_region_island_extruder_id(self.c_handle()))

    def properties(self) -> PluginProperties:
        return PluginProperties(self.api, self.api.host.layer_region_island_get_properties(self.c_handle()))

    def has_extrusions(self) -> bool:
        return bool(self.api.host.layer_region_island_has_extrusions(self.c_handle()))

    def has_extrusion(self, role: int) -> bool:
        return bool(self.api.host.layer_region_island_has_extrusion(self.c_handle(), int(role)))

    def extrusion(self, role: int) -> ExtrusionEntity | None:
        return _optional(ExtrusionEntity, self.api, self.api.host.layer_region_island_get_extrusion(self.c_handle(), int(role)))

    def fill_surfaces_collection(self) -> SurfaceCollection:
        return SurfaceCollection(self.api, self.api.host.layer_region_island_get_fill_surfaces(self.c_handle()))

    def fill_surface_count(self) -> int:
        return int(self.api.host.layer_region_island_count_fill_surface(self.c_handle()))

    def fill_surface(self, idx: int) -> Surface:
        return Surface(self.api, self.api.host.layer_region_island_get_fill_surface(self.c_handle(), int(idx)))

    def fill_surfaces(self) -> list[Surface]:
        return [self.fill_surface(idx) for idx in range(self.fill_surface_count())]


class MutableLayerRegionIsland(LayerRegionIsland):
    def mutable_c_handle(self) -> ctypes.c_void_p:
        return self.c_handle()

    def extrusion_mutable(self, role: int) -> MutableExtrusionEntity | None:
        return _optional(
            MutableExtrusionEntity,
            self.api,
            self.api.host.layer_region_island_get_mutable_extrusion(self.mutable_c_handle(), int(role)),
        )


# Borrowed layer island view.
class LayerIsland(DataTreeView):
    def slice(self) -> ExPolygon:
        return ExPolygon(self.api, self.api.host.layer_island_get_slice(self.c_handle()))

    def properties(self) -> PluginProperties:
        return PluginProperties(self.api, self.api.host.layer_island_get_properties(self.c_handle()))

    def bounding_box(self):
        return self.api.host.layer_island_get_bounding_box(self.c_handle())

    def infill_slice(self) -> ExPolygon:
        return ExPolygon(self.api, self.api.host.layer_island_get_infill_slice(self.c_handle()))

    def infill_areas(self) -> ExPolygonCollection:
        return ExPolygonCollection(self.api, self.api.host.layer_island_get_infill_areas(self.c_handle()))

    def infill_bounding_box(self):
        return self.api.host.layer_island_get_infill_bounding_box(self.c_handle())

    def infill_no_overlap_slice(self) -> ExPolygon:
        return ExPolygon(self.api, self.api.host.layer_island_get_infill_no_overlap_slice(self.c_handle()))

    def infill_no_overlap_areas(self) -> ExPolygonCollection:
        return ExPolygonCollection(self.api, self.api.host.layer_island_get_infill_no_overlap_areas(self.c_handle()))

    def region_count(self) -> int:
        return int(self.api.host.layer_island_count_region(self.c_handle()))

    def region(self, idx: int) -> LayerRegion:
        return LayerRegion(self.api, self.api.host.layer_island_get_region(self.c_handle(), int(idx)))

    def regions(self) -> Iterator[LayerRegion]:
        for idx in range(self.region_count()):
            yield self.region(idx)

    def region_island_count(self) -> int:
        return int(self.api.host.layer_island_count_region_island(self.c_handle()))

    def region_island(self, idx: int) -> LayerRegionIsland:
        return LayerRegionIsland(self.api, self.api.host.layer_island_get_region_island(self.c_handle(), int(idx)))

    def region_islands(self) -> Iterator[LayerRegionIsland]:
        for idx in range(self.region_island_count()):
            yield self.region_island(idx)

    def get_or_create_region_island(
        self,
        regions: Sequence[LayerRegion],
        extruder_id: int = -1,
    ) -> LayerRegionIsland | None:
        """
        Return the LayerRegionIsland for this island, region list, and extruder.

        The caller must already know which extruder should own the output.
        Passing an empty region list means "all regions of this island", just
        like get_or_create_full_region_island().
        """
        region_addresses = [_address(region.c_handle()) for region in regions]
        region_array = None
        if region_addresses:
            region_array = (ctypes.c_void_p * len(region_addresses))(*region_addresses)

        handle = self.api.host.layer_island_get_or_create_region_island(
            self.c_handle(),
            region_array,
            len(region_addresses),
            int(extruder_id),
        )
        return _optional(LayerRegionIsland, self.api, handle)

    def get_or_create_full_region_island(self, extruder_id: int = -1) -> LayerRegionIsland | None:
        """
        Return the LayerRegionIsland that represents every region on this island.

        Use this when the plugin output is not split by region settings. The
        extruder is still explicit, because the data-tree API does not infer it
        from an extrusion role.
        """
        handle = self.api.host.layer_island_get_or_create_region_island(
            self.c_handle(),
            None,
            0,
            int(extruder_id),
        )
        return _optional(LayerRegionIsland, self.api, handle)

    def layer(self) -> "Layer":
        return Layer(self.api, self.api.host.layer_island_get_layer(self.c_handle()))

    def lower_island_count(self) -> int:
        return int(self.api.host.layer_island_count_lower_island(self.c_handle()))

    def lower_island(self, idx: int) -> "LayerIsland":
        return LayerIsland(self.api, self.api.host.layer_island_get_lower_island(self.c_handle(), int(idx)))

    def lower_islands(self) -> Iterator["LayerIsland"]:
        for idx in range(self.lower_island_count()):
            yield self.lower_island(idx)

    def upper_island_count(self) -> int:
        return int(self.api.host.layer_island_count_upper_island(self.c_handle()))

    def upper_island(self, idx: int) -> "LayerIsland":
        return LayerIsland(self.api, self.api.host.layer_island_get_upper_island(self.c_handle(), int(idx)))

    def upper_islands(self) -> Iterator["LayerIsland"]:
        for idx in range(self.upper_island_count()):
            yield self.upper_island(idx)


class MutableLayerIsland(LayerIsland):
    def mutable_c_handle(self) -> ctypes.c_void_p:
        return self.c_handle()

    def slice_mutable(self) -> ExPolygon:
        return ExPolygon(self.api, self.api.host.layer_island_get_slice_mutable(self.mutable_c_handle()))

    def region_mutable(self, idx: int) -> MutableLayerRegion:
        return MutableLayerRegion(self.api, self.api.host.layer_island_get_region_mutable(self.mutable_c_handle(), int(idx)))

    def region_island_mutable(self, idx: int) -> MutableLayerRegionIsland:
        return MutableLayerRegionIsland(
            self.api,
            self.api.host.layer_island_get_region_island_mutable(self.mutable_c_handle(), int(idx)),
        )


# Borrowed layer view.
class Layer(DataTreeView):
    def height(self) -> int:
        return int(self.api.host.layer_get_height(self.c_handle()))

    def print_z(self) -> int:
        return int(self.api.host.layer_get_print_z(self.c_handle()))

    def slice_z(self) -> int:
        return int(self.api.host.layer_get_slice_z(self.c_handle()))

    def properties(self) -> PluginProperties:
        return PluginProperties(self.api, self.api.host.layer_get_properties(self.c_handle()))

    def upper_layer(self) -> "Layer | None":
        return _optional(Layer, self.api, self.api.host.layer_get_upper_layer(self.c_handle()))

    def lower_layer(self) -> "Layer | None":
        return _optional(Layer, self.api, self.api.host.layer_get_lower_layer(self.c_handle()))

    def region_count(self) -> int:
        return int(self.api.host.layer_count_region(self.c_handle()))

    def island_count(self) -> int:
        return int(self.api.host.layer_count_island(self.c_handle()))

    def slices(self) -> ExPolygonCollection:
        return ExPolygonCollection(self.api, self.api.host.layer_get_slices(self.c_handle()))

    def region(self, idx: int) -> LayerRegion:
        return LayerRegion(self.api, self.api.host.layer_get_region(self.c_handle(), int(idx)))

    def regions(self) -> Iterator[LayerRegion]:
        for idx in range(self.region_count()):
            yield self.region(idx)

    def island(self, idx: int) -> LayerIsland:
        return LayerIsland(self.api, self.api.host.layer_get_island(self.c_handle(), int(idx)))

    def islands(self) -> Iterator[LayerIsland]:
        for idx in range(self.island_count()):
            yield self.island(idx)


class MutableLayer(Layer):
    def mutable_c_handle(self) -> ctypes.c_void_p:
        return self.c_handle()

    def upper_layer_mutable(self) -> "MutableLayer | None":
        return _optional(MutableLayer, self.api, self.api.host.layer_get_upper_layer_mutable(self.mutable_c_handle()))

    def lower_layer_mutable(self) -> "MutableLayer | None":
        return _optional(MutableLayer, self.api, self.api.host.layer_get_lower_layer_mutable(self.mutable_c_handle()))

    def region_mutable(self, idx: int) -> MutableLayerRegion:
        return MutableLayerRegion(self.api, self.api.host.layer_get_region_mutable(self.mutable_c_handle(), int(idx)))

    def island_mutable(self, idx: int) -> MutableLayerIsland:
        return MutableLayerIsland(self.api, self.api.host.layer_get_island_mutable(self.mutable_c_handle(), int(idx)))


# Borrowed print object view.
class Object(DataTreeView):
    def properties(self) -> PluginProperties:
        return PluginProperties(self.api, self.api.host.object_get_properties(self.c_handle()))

    def config(self) -> Config:
        return Config(self.api, self.api.host.object_get_config(self.c_handle()))

    def max_z(self) -> int:
        return int(self.api.host.object_get_max_z(self.c_handle()))

    def transform(self) -> CMatrix4d:
        return self.api.host.object_get_transform(self.c_handle())

    def center_offset(self) -> CPoint:
        return self.api.host.object_get_center_offset(self.c_handle())

    def transform_centered(self) -> CMatrix4d:
        offset = self.center_offset()
        translation = self.api.host.matrix4d_translation(-unscaled(offset.x), -unscaled(offset.y), 0.0)
        return self.api.host.matrix4d_mul(translation, self.transform())

    def layer_count(self) -> int:
        return int(self.api.host.object_count_layer(self.c_handle()))

    def layer(self, idx: int) -> Layer:
        return Layer(self.api, self.api.host.object_get_layer(self.c_handle(), int(idx)))

    def layers(self) -> Iterator[Layer]:
        for idx in range(self.layer_count()):
            yield self.layer(idx)

    def auxiliary_layer_count(self) -> int:
        return int(self.api.host.object_count_auxiliary_layer(self.c_handle()))

    def auxiliary_layer(self, idx: int) -> Layer:
        return Layer(self.api, self.api.host.object_get_auxiliary_layer(self.c_handle(), int(idx)))

    def auxiliary_layers(self) -> Iterator[Layer]:
        for idx in range(self.auxiliary_layer_count()):
            yield self.auxiliary_layer(idx)

    def add_auxiliary_layer(self, height: int, print_z: int, slice_z: int) -> "MutableLayer":
        """
        Create an empty plugin-mutable auxiliary layer.

        The returned layer has the object's LayerRegion list but no raw slices.
        Write region slices, then call the relevant recompute callbacks from
        the step/helper that owns the geometry.
        """
        return MutableLayer(
            self.api,
            self.api.host.object_add_auxiliary_layer(
                self.c_handle(),
                int(height),
                int(print_z),
                int(slice_z),
            ),
        )

    def remove_auxiliary_layer(self, layer: Layer) -> bool:
        """
        Remove an auxiliary layer previously obtained from this Object.

        Normal object layers are not accepted by the host. The method returns
        False if the handle is null or does not belong to the auxiliary layer
        vector.
        """
        return bool(self.api.host.object_remove_auxiliary_layer(self.c_handle(), layer.c_handle()))

    def volume_count(self) -> int:
        return int(self.api.host.object_volume_count(self.c_handle()))

    def volume(self, idx: int) -> Volume:
        return Volume(self.api, self.api.host.object_volume_at(self.c_handle(), int(idx)))

    def volumes(self) -> Iterator[Volume]:
        for idx in range(self.volume_count()):
            yield self.volume(idx)

    def print_region_count(self) -> int:
        return int(self.api.host.object_count_region(self.c_handle()))

    def print_region(self, idx: int) -> PrintRegion:
        return PrintRegion(self.api, self.api.host.object_get_print_region(self.c_handle(), int(idx)))

    def print_regions(self) -> Iterator[PrintRegion]:
        for idx in range(self.print_region_count()):
            yield self.print_region(idx)


@abi_requires_headers("slic3r_volume.h")
def project_painting_to_polygons(
    storage_address: int,
    obj: Object,
    paint_key: str,
    painting_value: int,
) -> list[StoredPolygonCollection]:
    """
    Project one facet-painting value from an object onto all of its layers.

    ``paint_key`` may name a built-in painting channel or a generic facets
    annotation registered with the orchestrator. For support and seam channels,
    ``painting_value`` is normally an enforcer or blocker value; for MMU
    painting it is a one-based extruder index. This low-level helper performs
    geometric projection only and does not apply later strategy-specific
    refinements such as full MMU segmentation.

    The object and its model volumes are borrowed, read-only host data. Each
    returned collection is newly allocated in ``storage_address`` and belongs
    to the caller: keep the storage alive and release each collection with
    ``free_from_storage()``, or manage it with a ``with`` block. Borrowed
    polygons obtained from a collection become invalid if that collection is
    modified or freed. The result contains one collection per current object
    layer, including empty collections for layers without matching painted
    facets.
    """
    layer_count = obj.layer_count()
    output = [
        StoredPolygonCollection(obj.api, storage_address)
        for _unused in range(layer_count)
    ]
    if not output:
        return []

    output_array = (ctypes.c_void_p * layer_count)(
        *[collection.c_handle() for collection in output]
    )
    obj.api.host.object_project_painting_to_polygons(
        obj.c_handle(),
        _as_bytes(paint_key),
        int(painting_value),
        output_array,
        layer_count,
    )
    return output


class MutableObject(Object):
    def mutable_c_handle(self) -> ctypes.c_void_p:
        return self.c_handle()

    def config_mutable(self) -> MutableConfig:
        return MutableConfig(self.api, self.api.host.object_get_config_mutable(self.mutable_c_handle()))

    def layer_mutable(self, idx: int) -> MutableLayer:
        return MutableLayer(self.api, self.api.host.object_get_layer_mutable(self.mutable_c_handle(), int(idx)))

    def print_region_mutable(self, idx: int) -> MutablePrintRegion:
        return MutablePrintRegion(self.api, self.api.host.object_get_print_region_mutable(self.mutable_c_handle(), int(idx)))


# Borrowed print view.
class Print(DataTreeView):
    def config(self) -> Config:
        return Config(self.api, self.api.host.print_get_config(self.c_handle()))

    def records(self) -> "PrintRecords":
        return PrintRecords(self.api, self.c_handle())

    def object_count(self) -> int:
        return int(self.api.host.print_count_object(self.c_handle()))

    def object(self, idx: int) -> Object:
        return Object(self.api, self.api.host.print_get_object(self.c_handle(), int(idx)))

    def objects(self) -> Iterator[Object]:
        for idx in range(self.object_count()):
            yield self.object(idx)

    def auxiliary_object(self) -> Object:
        """Hidden object that owns print-level auxiliary layers."""
        return Object(self.api, self.api.host.print_get_auxiliary_object(self.c_handle()))


class MutablePrint(Print):
    def mutable_c_handle(self) -> ctypes.c_void_p:
        return self.c_handle()

    def config_mutable(self) -> MutableConfig:
        return MutableConfig(self.api, self.api.host.print_get_config_mutable(self.mutable_c_handle()))

    def object_mutable(self, idx: int) -> MutableObject:
        return MutableObject(self.api, self.api.host.print_get_object_mutable(self.mutable_c_handle(), int(idx)))


class PrintRecords(DataTreeView):
    """
    Borrowed access to variable Config channels owned by one Print.

    Mutations are only valid from sequential pipeline callbacks. Configs may be
    read concurrently once their producer has finished publishing the channel.
    """

    def channels(self) -> list[str]:
        names = self.api.host.print_records_channels(self.c_handle())
        return [_decode_const_string(names.items[idx]) for idx in range(int(names.size))]

    def get(self, channel: str) -> Config | None:
        handle = self.api.host.print_records_get(self.c_handle(), _as_bytes(channel))
        return _optional(Config, self.api, handle)

    def get_or_add(self, channel: str) -> MutableConfig:
        handle = self.api.host.print_records_get_or_add(self.c_handle(), _as_bytes(channel))
        if not _address(handle):
            raise RuntimeError("The Print record channel could not be created.")
        return MutableConfig(self.api, handle)

    def remove(self, channel: str) -> bool:
        return bool(self.api.host.print_records_remove(self.c_handle(), _as_bytes(channel)))

    def allocate_id(self) -> int:
        record_id = int(self.api.host.print_records_allocate_id(self.c_handle()))
        if record_id == 0:
            raise RuntimeError("The Print record identifier space is exhausted.")
        return record_id


class AuxiliaryLayerBuildResult(NamedTuple):
    """Result of building one host-owned auxiliary layer."""

    layer: Layer | None
    created: bool


def _auxiliary_layer_range_for_z(obj: Object, slice_z: int):
    """Select the active range; a shared boundary belongs to the upper range."""
    selected = None
    range_count = int(obj.api.host.object_count_slicing_layer_range(obj.c_handle()))
    for range_idx in range(range_count):
        layer_range = obj.api.host.object_get_slicing_layer_range(obj.c_handle(), range_idx)
        if not layer_range:
            continue
        z_min = int(obj.api.host.slicing_layer_range_get_z_min(layer_range))
        z_max = int(obj.api.host.slicing_layer_range_get_z_max(layer_range))
        if z_min <= slice_z <= z_max:
            selected = layer_range
            if slice_z < z_max:
                break
    return selected


def _auxiliary_mask_slicing_params(print_view: Print, obj: Object) -> CMeshSlicingParams:
    """Build native-equivalent parameters for settings masks without XY growth."""
    print_config = print_view.config()
    object_config = obj.config()
    slicing_mode = object_config.int("slicing_mode", default=0)

    params = CMeshSlicingParams()
    if slicing_mode == 1:
        params.mode = RAW_MESH_SLICING_MODE_EVEN_ODD
    elif slicing_mode == 2:
        params.mode = RAW_MESH_SLICING_MODE_POSITIVE
    else:
        params.mode = RAW_MESH_SLICING_MODE_REGULAR
    params.mode_below = params.mode
    params.transform = obj.transform_centered()
    params.closing_radius = float(object_config.float("slice_closing_radius", default=0.0))
    params.extra_offset = 0.0
    params.resolution = float(print_config.float("resolution", default=EPSILON))
    params.model_resolution = float(object_config.float("model_precision", default=0.0))
    return params


def _auxiliary_slice_volume_mask(
    storage_address: int,
    print_view: Print,
    obj: Object,
    volume_region,
    subject: ExPolygonCollection,
    slice_z: int,
) -> StoredExPolygonCollection:
    """Return one storage-owned model-part/modifier mask clipped to the subject."""
    empty = StoredExPolygonCollection(obj.api, storage_address)
    if subject.empty():
        return empty

    z_mm = unscaled(slice_z)
    bbox = obj.api.host.slicing_volume_region_get_bbox(volume_region)
    if bbox.min.z > z_mm + EPSILON or bbox.max.z < z_mm - EPSILON:
        return empty

    volume_handle = obj.api.host.slicing_volume_region_get_volume(volume_region)
    if not volume_handle:
        return empty
    volume = Volume(obj.api, volume_handle)
    if not volume.is_model_part() and not volume.is_modifier():
        return empty

    params = _auxiliary_mask_slicing_params(print_view, obj)
    slices = volume.mesh().slice_to_expolygons(
        storage_address,
        params.transform,
        [z_mm],
        params,
    )
    if not slices or slices[0].empty():
        return empty

    clipper = ClipperContext(obj.api, storage_address)
    with clipper(slices[0]) as mask_operand, clipper(subject) as subject_operand:
        with clipper.intersection(mask_operand, subject_operand) as clipped:
            return clipped.to_expolygon_collection()


def _auxiliary_assign_regions_from_masks(
    storage_address: int,
    print_view: Print,
    obj: Object,
    layer: MutableLayer,
    subject: ExPolygonCollection,
    slice_z: int,
) -> None:
    """Publish disjoint region slices and rebuild one new layer's derived islands."""
    region_count = obj.print_region_count()
    if region_count == 0:
        return

    region_slices = [
        StoredExPolygonCollection(obj.api, storage_address)
        for _unused in range(region_count)
    ]
    region_slices[0].copy_from(subject)

    layer_range = _auxiliary_layer_range_for_z(obj, slice_z)
    volume_region_count = int(
        obj.api.host.slicing_layer_range_count_volume_region(layer_range)
    )
    effective_masks = [
        StoredExPolygonCollection(obj.api, storage_address)
        for _unused in range(volume_region_count)
    ]
    clipper = ClipperContext(obj.api, storage_address)

    for volume_region_idx in range(volume_region_count):
        volume_region = obj.api.host.slicing_layer_range_get_volume_region(
            layer_range, volume_region_idx
        )
        if not volume_region:
            continue
        target_region_idx = int(
            obj.api.host.slicing_volume_region_get_layer_region_idx(volume_region)
        )
        if target_region_idx < 0 or target_region_idx >= len(region_slices):
            continue

        mask = _auxiliary_slice_volume_mask(
            storage_address, print_view, obj, volume_region, subject, slice_z
        )
        if mask.empty():
            mask.free_from_storage()
            continue

        parent_idx = int(obj.api.host.slicing_volume_region_get_parent(volume_region))
        if 0 <= parent_idx < len(effective_masks):
            with clipper(mask) as mask_operand, clipper(effective_masks[parent_idx]) as parent_operand:
                with clipper.intersection(mask_operand, parent_operand) as clipped:
                    parent_clipped = clipped.to_expolygon_collection()
            mask.free_from_storage()
            mask = parent_clipped
            if mask.empty():
                mask.free_from_storage()
                continue

        effective_masks[volume_region_idx].copy_from(mask)
        for region_slice in region_slices:
            with clipper(region_slice) as region_operand, clipper(mask) as mask_operand:
                with clipper.diff(region_operand, mask_operand) as difference:
                    remaining = difference.to_expolygon_collection()
            region_slice.move_from(remaining)

        region_slices[target_region_idx].append_move_from(mask)
        with clipper(region_slices[target_region_idx]) as target_operand:
            with clipper.union(target_operand) as united:
                merged = united.to_expolygon_collection()
        region_slices[target_region_idx].move_from(merged)

    for region_idx, region_slice in enumerate(region_slices):
        region = obj.api.host.layer_get_region_mutable(layer.mutable_c_handle(), region_idx)
        destination = obj.api.host.layer_region_borrow_mutable_slices(region)
        if destination:
            obj.api.host.expolygons_move(destination, region_slice.mutable_c_handle())

    obj.api.host.layer_recompute_slices_and_islands_from_layer_regions(layer.mutable_c_handle())
    obj.api.host.layer_add_regions_to_islands(layer.mutable_c_handle())


@abi_requires_headers(
    "slic3r_clipper.h",
    "slic3r_config.h",
    "slic3r_config_option.h",
    "slic3r_data_tree.h",
    "slic3r_geometry.h",
    "slic3r_volume.h",
)
def build_auxiliary_layer_regions_from_subject(
    storage_address: int,
    print_view: Print,
    obj: Object,
    subject: ExPolygonCollection,
    height: int,
    print_z: int,
    slice_z: int,
) -> AuxiliaryLayerBuildResult:
    """
    Create and region-classify an auxiliary layer from an existing 2D subject.

    ``height``, ``print_z`` and ``slice_z`` are scaled coordinates. ``subject``
    is borrowed and read-only: the helper copies or clips it but never mutates
    or frees it. ``print_view`` and ``obj`` must belong to the same live print,
    and the call must run in a sequential phase where adding a layer is allowed.

    Region zero initially owns the complete subject. Model-part and parameter-
    modifier volumes active at ``slice_z`` are sliced to choose alternate
    PrintRegion settings; negative and special-purpose volumes are ignored.
    The helper then rebuilds layer slices, islands and region-island membership.
    It does not generate perimeters, infill, support, or extrusion paths.

    Intermediate polygons are allocated in ``storage_address`` and released by
    their ``Stored*`` wrappers before return. The successful ``Layer`` result is
    a borrowed view owned by ``obj`` and must never be freed by the plugin. A
    false result means no layer was created. If an exception occurs after host
    creation, there is no rollback: the object may retain a partially populated
    auxiliary layer, so callers should validate inputs before invoking it.
    """
    storage = _require_handle(storage_address, "storage")
    if obj.print_region_count() == 0 or int(height) <= 0 or int(print_z) <= 0:
        return AuxiliaryLayerBuildResult(None, False)

    layer_handle = obj.api.host.object_add_auxiliary_layer(
        obj.c_handle(), int(height), int(print_z), int(slice_z)
    )
    if not layer_handle:
        return AuxiliaryLayerBuildResult(None, False)

    layer = MutableLayer(obj.api, layer_handle)
    _auxiliary_assign_regions_from_masks(
        storage, print_view, obj, layer, subject, int(slice_z)
    )
    return AuxiliaryLayerBuildResult(layer, True)


__all__ = [
    "AuxiliaryLayerBuildResult",
    "Config",
    "ConfigOption",
    "DataTreeView",
    "Layer",
    "LayerIsland",
    "LayerRegion",
    "LayerRegionIsland",
    "MutableConfig",
    "MutableConfigOption",
    "MutableLayer",
    "MutableLayerIsland",
    "MutableLayerRegion",
    "MutableLayerRegionIsland",
    "MutableObject",
    "MutablePrint",
    "MutablePrintRegion",
    "Object",
    "Print",
    "PrintRecords",
    "PrintRegion",
    "project_painting_to_polygons",
    "StoredConfig",
    "Surface",
    "SurfaceCollection",
    "SurfaceTypeBuilder",
    "TriangleMesh",
    "Volume",
    "build_auxiliary_layer_regions_from_subject",
    "srf_type",
    "unscaled",
]
