#/|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
#/|/
#/|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#/|/

"""Owning Python wrapper for the bridge-detector plugin service.

``BridgeDetector`` owns the native service instance returned by the
orchestrator. Use it as a context manager or call ``reset()`` when its lifetime
must end early. Source geometry supplied during creation remains owned by its
caller; the built-in provider copies the data it needs before creation returns.

Coverage and unsupported-edge results replace the contents of caller-owned
``Stored*`` collections. All element views previously borrowed from an output
collection become invalid. The collections keep their normal storage lifetime
and must be released independently. Angles use radians and ``-1.0`` requests
the provider's automatically detected direction. Lengths use scaled geometry
coordinates.
"""

from __future__ import annotations

import ctypes

from slic3r_abi import abi_requires_headers
from slic3r_api_generated import BridgeDetectorCreateInput, BridgeDetectorInstance
from slic3r_geometry_views import (
    ExPolygon,
    ExPolygonCollection,
    StoredPolygonCollection,
    StoredPolylineCollection,
)


def _address(handle) -> int:
    if handle is None:
        return 0
    if isinstance(handle, ctypes.c_void_p):
        return int(handle.value or 0)
    if hasattr(handle, "c_handle"):
        return _address(handle.c_handle())
    if hasattr(handle, "address"):
        return int(handle.address)
    return int(handle)


def _require_same_api(api, value, name: str) -> None:
    if value is not None and getattr(value, "api", None) is not api:
        raise ValueError(f"{name} must use this API instance")


@abi_requires_headers("slic3r_bridge_detector.h", "slic3r_orchestrator.h")
class BridgeDetector:
    """Unique owner of one native bridge-detector service instance.

    Python assignment does not copy this object. Explicit shallow or deep copy
    is rejected so two wrappers cannot destroy the same native context. The
    object may safely be reset more than once; after reset, queries follow the
    C contract and return false, zero, or ``-1`` as appropriate.
    """

    def __init__(self, api, instance: BridgeDetectorInstance) -> None:
        self.api = api
        # Copy the two-pointer record by value. The caller's temporary ctypes
        # result may disappear, while this record remains the sole owner.
        self._instance = BridgeDetectorInstance(instance.ctx, instance.vt)

    def valid(self) -> bool:
        return bool(self._instance.ctx and self._instance.vt)

    def detect_angle(self, bridge_direction_override: float = -1.0) -> bool:
        """Detect or apply a bridge direction without changing source geometry."""
        return bool(
            self.api.host.bridge_detector_detect_angle(
                ctypes.byref(self._instance), float(bridge_direction_override)
            )
        )

    def coverage(
        self, out_polygons: StoredPolygonCollection, angle: float = -1.0
    ) -> int:
        """Replace the output with covered polygons and return its new size.

        ``out_polygons`` remains owned by its storage, but all of its previous
        contents are discarded. Every element view borrowed from the collection
        before this call is invalid afterwards and must no longer be accessed.
        """
        if not isinstance(out_polygons, StoredPolygonCollection):
            raise TypeError("out_polygons must be a StoredPolygonCollection")
        _require_same_api(self.api, out_polygons, "out_polygons")
        return int(
            self.api.host.bridge_detector_coverage(
                ctypes.byref(self._instance),
                float(angle),
                out_polygons.mutable_c_handle(),
            )
        )

    def unsupported_edges(
        self, out_polylines: StoredPolylineCollection, angle: float = -1.0
    ) -> int:
        """Replace the output with unsupported paths and return its new size.

        ``out_polylines`` remains storage-owned and must be released separately.
        Its previous contents are discarded, invalidating every element view
        borrowed from it before this call.
        """
        if not isinstance(out_polylines, StoredPolylineCollection):
            raise TypeError("out_polylines must be a StoredPolylineCollection")
        _require_same_api(self.api, out_polylines, "out_polylines")
        return int(
            self.api.host.bridge_detector_unsupported_edges(
                ctypes.byref(self._instance),
                float(angle),
                out_polylines.mutable_c_handle(),
            )
        )

    def angle(self) -> float:
        """Return the selected direction, or -1 when no result is available."""
        return float(
            self.api.host.bridge_detector_get_angle(ctypes.byref(self._instance))
        )

    def set_max_bridge_length(self, max_bridge_length: float) -> None:
        """Set the maximum accepted bridge length in scaled coordinate units."""
        self.api.host.bridge_detector_set_max_bridge_length(
            ctypes.byref(self._instance), float(max_bridge_length)
        )

    def max_bridge_length(self) -> float:
        """Return the configured scaled maximum, or -1 for an invalid instance."""
        return float(
            self.api.host.bridge_detector_get_max_bridge_length(
                ctypes.byref(self._instance)
            )
        )

    def set_layer_id(self, layer_id: int) -> None:
        self.api.host.bridge_detector_set_layer_id(
            ctypes.byref(self._instance), int(layer_id)
        )

    def layer_id(self) -> int:
        return int(
            self.api.host.bridge_detector_get_layer_id(ctypes.byref(self._instance))
        )

    def reset(self) -> None:
        """Destroy the owned service context and leave this wrapper invalid."""
        self.api.host.bridge_detector_destroy(ctypes.byref(self._instance))
        self._instance.ctx = None
        self._instance.vt = None

    def __bool__(self) -> bool:
        return self.valid()

    def __enter__(self) -> "BridgeDetector":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.reset()

    def __copy__(self):
        raise TypeError("BridgeDetector owns a unique native instance")

    def __deepcopy__(self, memo):
        raise TypeError("BridgeDetector owns a unique native instance")

    def __del__(self) -> None:
        try:
            self.reset()
        except Exception:
            pass


@abi_requires_headers("slic3r_bridge_detector.h", "slic3r_orchestrator.h")
def create_bridge_detector(
    api,
    orchestrator_handle,
    *,
    expolygon: ExPolygon | None = None,
    expolygons: ExPolygonCollection | None = None,
    lower_slices: ExPolygonCollection | None = None,
    spacing: int,
    precision: int,
    layer_id: int,
) -> BridgeDetector:
    """Create an owning detector from borrowed geometry views.

    Supply at least one subject. If both subject forms are present, the service
    gives priority to ``expolygon``. Geometry may be released after this call
    returns because providers must retain their own state. A missing active
    provider produces an invalid ``BridgeDetector`` rather than an exception.
    """
    if expolygon is None and expolygons is None:
        raise ValueError("expolygon or expolygons must be supplied")
    if expolygon is not None and not isinstance(expolygon, ExPolygon):
        raise TypeError("expolygon must be an ExPolygon")
    if expolygons is not None and not isinstance(expolygons, ExPolygonCollection):
        raise TypeError("expolygons must be an ExPolygonCollection")
    if lower_slices is not None and not isinstance(
        lower_slices, ExPolygonCollection
    ):
        raise TypeError("lower_slices must be an ExPolygonCollection")
    _require_same_api(api, expolygon, "expolygon")
    _require_same_api(api, expolygons, "expolygons")
    _require_same_api(api, lower_slices, "lower_slices")

    create_input = BridgeDetectorCreateInput(
        ctypes.c_void_p(_address(expolygon)),
        ctypes.c_void_p(_address(expolygons)),
        ctypes.c_void_p(_address(lower_slices)),
        int(spacing),
        int(precision),
        int(layer_id),
    )
    instance = api.host.orchestrator_create_bridge_detector(
        ctypes.c_void_p(_address(orchestrator_handle)), ctypes.byref(create_input)
    )
    return BridgeDetector(api, instance)


__all__ = ["BridgeDetector", "create_bridge_detector"]
