#/|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
#/|/
#/|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#/|/

"""
Python helper for STEP_LAYER_HEIGHT plugins.

Layer-height plugins receive one object and must publish an explicit list of
object-local layer descriptors. Each descriptor is a pair:

    [layer_top_z_scaled, layer_height_scaled]

The host later creates exactly those object layers. Keeping the height next to
the top Z lets a plugin intentionally leave a vertical gap before a layer.

Context contents
----------------

LayerHeightContext exposes:

* read-only layer_height_print() and layer_height_object() views for the object
  being sliced;
* layer_height_max_z(), the object-local scaled Z height the descriptor list
  must cover;
* enforced_layer_descriptors(), the host-provided fixed boundaries that a
  plugin should preserve unless it intentionally overrides the layer plan;
* layer_config_ranges(), the object-local Z ranges with their effective config;
* set_layer_descriptors(), the only publishing callback for this step;
* plugin_storage(), cancellation, progress, warning, and error helpers.

Typical use
-----------

    ctx = api.layer_height_context(run_ctx_address)
    if ctx is None:
        return

    descriptors = []
    z = scale_i(0.2)
    while z <= ctx.layer_height_max_z():
        descriptors.append((z, scale_i(0.2)))
        z += scale_i(0.2)
    ctx.set_layer_descriptors(descriptors)

All handles returned by this module are borrowed and valid only during the
current PluginBase.run() call.
"""

from __future__ import annotations

import ctypes
from dataclasses import dataclass
from typing import Iterable, Iterator

from slic3r_api_generated import (
    CLayerConfigRange,
    PluginRunContext,
    RunCtxLayerHeightGeneration,
    STEP_LAYER_HEIGHT,
)
from slic3r_datatree_views import Config, Object, Print
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


@dataclass(frozen=True)
class LayerDescriptor:
    """
    One explicit layer interval.

    top_z and height are scaled object-local coordinates. bottom_z is derived
    for convenience and is also object-local.
    """

    top_z: int
    height: int

    @property
    def bottom_z(self) -> int:
        return self.top_z - self.height


class LayerConfigRange:
    """
    Borrowed view over one object layer-config range.

    The range config contains modifier settings that apply between z_min and
    z_max. It is read-only because layer-height plugins should only decide
    where layers go, not mutate model configuration.
    """

    def __init__(self, api, raw: CLayerConfigRange) -> None:
        self.api = api
        self.raw = raw

    def z_min(self) -> int:
        return int(self.raw.z_min)

    def z_max(self) -> int:
        return int(self.raw.z_max)

    def config(self) -> Config:
        return Config(self.api, self.raw.config)


@abi_requires_headers("steps/slic3r_step_layer_height.h")
class LayerHeightContext(PluginRunContextView):
    """
    High-level wrapper around the STEP_LAYER_HEIGHT payload.

    Use enforced_layer_descriptors() to read GUI variable-layer-height input,
    layer_config_ranges() to inspect model-object layer ranges, and
    set_layer_descriptors() exactly once to publish the plugin result.
    """

    def __init__(self, api, common: PluginRunContext, payload: RunCtxLayerHeightGeneration) -> None:
        super().__init__(api, common)
        self.payload = payload
        self._last_descriptor_array = None

    @classmethod
    def from_layer_height_run_context(cls, api, run_ctx_address: int) -> "LayerHeightContext | None":
        if not run_ctx_address:
            return None
        common = PluginRunContext.from_address(int(run_ctx_address))
        if common.step != STEP_LAYER_HEIGHT or not common.data:
            return None
        payload = ctypes.cast(common.data, ctypes.POINTER(RunCtxLayerHeightGeneration)).contents
        if not payload.print or not payload.object or not payload.set_layer_height_profile:
            return None
        return cls(api, common, payload)

    def layer_height_print(self) -> Print:
        return Print(self.api, self.payload.print)

    def layer_height_object(self) -> Object:
        return Object(self.api, self.payload.object)

    def layer_height_max_z(self) -> int:
        return int(self.payload.max_z)

    def enforced_layer_descriptor_values(self) -> list[int]:
        if not self.payload.enforce_layer_zs or self.payload.enforce_layer_zs_size == 0:
            return []
        return [int(self.payload.enforce_layer_zs[idx]) for idx in range(int(self.payload.enforce_layer_zs_size))]

    def enforced_layer_descriptors(self) -> list[LayerDescriptor]:
        raw = self.enforced_layer_descriptor_values()
        return [
            LayerDescriptor(raw[idx], raw[idx + 1])
            for idx in range(0, len(raw) - 1, 2)
            if raw[idx + 1] > 0
        ]

    def layer_config_range_count(self) -> int:
        return int(self.payload.layer_config_ranges_size)

    def layer_config_range(self, idx: int) -> LayerConfigRange:
        if idx < 0 or idx >= self.layer_config_range_count():
            raise IndexError(idx)
        return LayerConfigRange(self.api, self.payload.layer_config_ranges[idx])

    def layer_config_ranges(self) -> Iterator[LayerConfigRange]:
        for idx in range(self.layer_config_range_count()):
            yield self.layer_config_range(idx)

    def set_layer_descriptor_values(self, values: Iterable[int]) -> None:
        descriptor_values = [int(value) for value in values]
        if len(descriptor_values) % 2 != 0:
            raise ValueError("Layer descriptor array must contain [top_z, height] pairs.")

        # Keep the ctypes array alive until a later call replaces it. The host
        # copies the values immediately today, but retaining it makes the helper
        # safe if that callback ever validates asynchronously.
        if descriptor_values:
            array_type = ctypes.c_int64 * len(descriptor_values)
            self._last_descriptor_array = array_type(*descriptor_values)
            self.payload.set_layer_height_profile(
                self.payload.object,
                self._last_descriptor_array,
                len(descriptor_values),
            )
        else:
            self._last_descriptor_array = None
            self.payload.set_layer_height_profile(self.payload.object, None, 0)

    def set_layer_descriptors(self, descriptors: Iterable[LayerDescriptor | tuple[int, int]]) -> None:
        values: list[int] = []
        for descriptor in descriptors:
            if isinstance(descriptor, LayerDescriptor):
                top_z = descriptor.top_z
                height = descriptor.height
            else:
                top_z, height = descriptor
            values.extend([int(top_z), int(height)])
        self.set_layer_descriptor_values(values)

__all__ = [
    "LayerConfigRange",
    "LayerDescriptor",
    "LayerHeightContext",
]
