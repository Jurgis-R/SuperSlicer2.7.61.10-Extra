#/|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
#/|/
#/|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#/|/

"""Provider and consumer views for the ``SEAM_PLACER`` service.

A provider derives :class:`SeamPlacerSession`, then publishes one initialized
session from its plugin ``run()`` callback::

    context = api.seam_placer_context(run_ctx_address)
    if context is not None:
        context.publish_seam_placer(MySeamPlacerSession())

The published C instance owns the session. The provider must not publish the
same Python object twice or use it again after publication. A consumer selects
and executes the plugin through :class:`OrchestratorView`, then transfers the
payload instance into :class:`SeamPlacer`. Closing that wrapper invokes the C
destroy callback exactly once.

Initialization is sequential and occurs exactly once before publication.
``place_seam()`` may subsequently be called by several native worker threads,
so provider implementations must protect mutable state themselves. Callback
exceptions are contained at the C boundary and diagnostics are stored per
calling native thread in session-owned storage that survives temporary Python
thread states.

Printing plans and extrusion entities are borrowed host views. They must outlive
the session and must never be freed by provider code. The consumer validates
that every returned point belongs to a line or analytic arc of the loop within
``SCALED_EPSILON``; it never projects an invalid result silently.
"""

from __future__ import annotations

import ctypes
import math
import threading
from typing import Iterator

from slic3r_abi import abi_requires_headers
from slic3r_api_generated import (
    CPoint,
    PluginRunContext,
    RAW_EXTRUSION_ARC_ORIENTATION_CCW,
    RAW_SEAM_PLACER_STATUS_ERROR,
    RAW_SEAM_PLACER_STATUS_INVALID_ARGUMENT,
    RAW_SEAM_PLACER_STATUS_SUCCESS,
    RAW_SEAM_PLACER_STATUS_UNSET,
    RawSeamPlacerInstance,
    RawSeamPlacerResult,
    RunCtxSeamPlacer,
    SCALED_EPSILON,
    SEAM_PLACER,
    SEAM_PLACER_DESTROY,
    SEAM_PLACER_PLACE,
)
from slic3r_extrusion_views import ExtrusionEntity
from slic3r_geometry_views import as_point
from slic3r_printing_plan_views import PrintingPlan


def _address(handle) -> int:
    """Return one borrowed pointer value without changing its ownership."""
    if handle is None:
        return 0
    if isinstance(handle, ctypes.c_void_p):
        return int(handle.value or 0)
    if hasattr(handle, "c_handle"):
        return _address(handle.c_handle())
    if hasattr(handle, "address"):
        return int(handle.address)
    return int(handle)


@abi_requires_headers("steps/slic3r_step_seam_placer.h")
class RawSeamPlacerVTable(ctypes.Structure):
    """ctypes layout of ``raw_seam_placer_vtable`` from the public C header."""

    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("destroy", SEAM_PLACER_DESTROY),
        ("place_seam", SEAM_PLACER_PLACE),
    ]


@abi_requires_headers("steps/slic3r_step_seam_placer.h")
class SeamPlacerSession:
    """Base class implemented by Python seam-placement providers.

    ``initialize()`` receives a borrowed plan and is called once before the
    session becomes visible to consumers. ``place_seam()`` receives a borrowed,
    nonempty, fixed, continuous and closed extrusion entity. Its result may be
    a ``CPoint`` or a two-coordinate value accepted by ``as_point()``.

    The plan and loops remain host-owned. Implementations may retain their
    handles only while the documented host lifetimes are respected; they must
    not free either object. A session is uniquely owned after publication.
    """

    def initialize(self, plan: PrintingPlan) -> None:
        """Build immutable provider state for one borrowed PrintingPlan."""
        raise NotImplementedError

    def place_seam(self, loop: ExtrusionEntity, start_position: CPoint):
        """Return a stable point on ``loop`` for this scaled start position."""
        raise NotImplementedError


class _PublishedSession:
    """Keep one initialized session and its callback diagnostics alive."""

    def __init__(self, api, session: SeamPlacerSession) -> None:
        self.api = api
        self.session = session
        # ctypes may create and destroy a temporary Python thread state whenever
        # a native worker enters a callback. These buffers therefore belong to
        # the session registry, not threading.local(). A native thread ID keeps
        # one diagnostic alive until that thread's next seam callback.
        self.diagnostics: dict[int, ctypes.Array] = {}


_published_lock = threading.RLock()
_published_sessions: dict[int, _PublishedSession] = {}


def _set_result_defaults(result: RawSeamPlacerResult) -> None:
    """Initialize every writable result field before calling provider code."""
    result.status = RAW_SEAM_PLACER_STATUS_UNSET
    result.point = CPoint()
    result.error_message = None


def _destroy_published_session(session_address) -> None:
    """Release one registry-owned session; repeated destroy calls are harmless."""
    key = _address(session_address)
    if not key:
        return
    try:
        with _published_lock:
            published = _published_sessions.pop(key, None)
            if published is not None:
                published.diagnostics.clear()
    except BaseException:
        # ctypes callbacks must never propagate, including during interpreter
        # shutdown when module globals may already be partially unavailable.
        return


def _place_published_seam(
    session_address,
    loop_address,
    start_position: CPoint,
    result_pointer,
) -> None:
    """Invoke one Python provider while containing every exception at the ABI."""
    published = None
    native_thread_id = threading.get_native_id()
    try:
        if not result_pointer:
            return
        result = result_pointer.contents
        if int(result.struct_size) < ctypes.sizeof(RawSeamPlacerResult):
            return
        _set_result_defaults(result)

        key = _address(session_address)
        loop_handle = _address(loop_address)
        with _published_lock:
            published = _published_sessions.get(key)
            if published is not None:
                published.diagnostics.pop(native_thread_id, None)
        if published is None or not loop_handle:
            result.status = RAW_SEAM_PLACER_STATUS_INVALID_ARGUMENT
            return

        loop = ExtrusionEntity(published.api, loop_handle)
        if not loop.is_loop():
            result.status = RAW_SEAM_PLACER_STATUS_INVALID_ARGUMENT
            return

        result.point = as_point(published.session.place_seam(loop, start_position))
        result.status = RAW_SEAM_PLACER_STATUS_SUCCESS
    except BaseException as exc:
        # result_pointer and its declared size were validated above whenever
        # possible. If even that validation failed there is no safe output area.
        try:
            if result_pointer and int(result_pointer.contents.struct_size) >= ctypes.sizeof(
                RawSeamPlacerResult
            ):
                result_pointer.contents.status = RAW_SEAM_PLACER_STATUS_ERROR
                message = str(exc) or exc.__class__.__name__
                encoded = message.encode("utf-8", errors="replace")
                diagnostic = ctypes.create_string_buffer(encoded)
                if published is not None:
                    with _published_lock:
                        published.diagnostics[native_thread_id] = diagnostic
                    result_pointer.contents.error_message = ctypes.cast(
                        diagnostic, ctypes.c_char_p
                    )
        except BaseException:
            return


_destroy_callback = SEAM_PLACER_DESTROY(_destroy_published_session)
_place_callback = SEAM_PLACER_PLACE(_place_published_seam)
_session_vtable = RawSeamPlacerVTable(
    ctypes.sizeof(RawSeamPlacerVTable),
    _destroy_callback,
    _place_callback,
)
_session_vtable_pointer = ctypes.pointer(_session_vtable)


@abi_requires_headers("steps/slic3r_step_seam_placer.h")
def make_seam_placer_instance(
    session: SeamPlacerSession, plan: PrintingPlan
) -> RawSeamPlacerInstance:
    """Initialize and transfer one Python provider session to a C instance.

    ``session`` is consumed by this call, including when initialization raises,
    and cannot be submitted a second time. After success, the caller must
    publish or destroy the returned instance and discard every other reference
    to the session. An initialization exception creates no C instance and is
    propagated to ``PluginBase``, which reports it through the normal plugin
    error channel.

    The borrowed ``plan`` is not retained as an owning object. Its host storage
    and every queried loop must remain valid until the instance is destroyed.
    """
    if not isinstance(session, SeamPlacerSession):
        raise TypeError("session must derive from SeamPlacerSession")
    if not isinstance(plan, PrintingPlan) or not plan.valid():
        raise ValueError("a seam placer instance needs a valid PrintingPlan")

    with _published_lock:
        if getattr(session, "_slic3r_seam_placer_consumed", False):
            raise ValueError("the seam placer session has already been consumed")
        session._slic3r_seam_placer_consumed = True

    session.initialize(plan)
    published = _PublishedSession(plan.api, session)
    key = id(published)
    with _published_lock:
        if key in _published_sessions:
            raise RuntimeError("the seam placer session handle is already registered")
        _published_sessions[key] = published

    return RawSeamPlacerInstance(
        ctypes.sizeof(RawSeamPlacerInstance),
        ctypes.c_void_p(key),
        ctypes.cast(_session_vtable_pointer, ctypes.c_void_p),
    )


def _instance_vtable(instance: RawSeamPlacerInstance):
    """Return a complete callable vtable pointer or ``None``."""
    if int(instance.struct_size) < ctypes.sizeof(RawSeamPlacerInstance):
        return None
    if not _address(instance.session) or not _address(instance.vtable):
        return None
    pointer = ctypes.cast(
        ctypes.c_void_p(_address(instance.vtable)),
        ctypes.POINTER(RawSeamPlacerVTable),
    )
    if int(pointer.contents.struct_size) < ctypes.sizeof(RawSeamPlacerVTable):
        return None
    if not pointer.contents.destroy or not pointer.contents.place_seam:
        return None
    return pointer


def _point_distance_squared(point: CPoint, other: CPoint) -> float:
    """Return squared Euclidean distance in scaled coordinates."""
    dx = float(point.x) - float(other.x)
    dy = float(point.y) - float(other.y)
    return dx * dx + dy * dy


def _cross(lhs: tuple[float, float], rhs: tuple[float, float]) -> float:
    """Return the two-dimensional cross product of two vectors."""
    return lhs[0] * rhs[1] - lhs[1] * rhs[0]


def _inside_arc_wedge(
    start: tuple[float, float],
    end: tuple[float, float],
    query: tuple[float, float],
    shorter_arc: bool,
    counter_clockwise: bool,
) -> bool:
    """Mirror ArcWelder's strict angular test for short and long arcs."""
    if shorter_arc:
        if counter_clockwise:
            return _cross(start, query) > 0.0 and _cross(query, end) > 0.0
        return _cross(start, query) < 0.0 and _cross(query, end) < 0.0
    if counter_clockwise:
        return _cross(end, query) < 0.0 or _cross(query, start) < 0.0
    return _cross(end, query) > 0.0 or _cross(query, start) > 0.0


def _point_belongs_to_segment(point: CPoint, segment, tolerance: float) -> bool:
    """Validate a point against one finite line or analytic circular arc."""
    if (
        _point_distance_squared(point, segment.point_a) <= tolerance * tolerance
        or _point_distance_squared(point, segment.point_b) <= tolerance * tolerance
    ):
        return True

    ax = float(segment.point_a.x)
    ay = float(segment.point_a.y)
    bx = float(segment.point_b.x)
    by = float(segment.point_b.y)
    px = float(point.x)
    py = float(point.y)
    dx = bx - ax
    dy = by - ay
    chord_squared = dx * dx + dy * dy
    if chord_squared <= 0.0:
        return False

    radius = float(segment.radius)
    if radius == 0.0:
        projection = ((px - ax) * dx + (py - ay) * dy) / chord_squared
        projection = max(0.0, min(1.0, projection))
        nearest_x = ax + projection * dx
        nearest_y = ay + projection * dy
        return math.hypot(px - nearest_x, py - nearest_y) <= tolerance

    counter_clockwise = int(segment.orientation) == int(
        RAW_EXTRUSION_ARC_ORIENTATION_CCW
    )
    center_factor_squared = radius * radius / chord_squared - 0.25
    center_factor = math.sqrt(center_factor_squared) if center_factor_squared > 0.0 else 0.0
    mid_x = (ax + bx) * 0.5
    mid_y = (ay + by) * 0.5
    perpendicular_x = -dy * center_factor
    perpendicular_y = dx * center_factor
    if (radius > 0.0) == counter_clockwise:
        center_x = int(mid_x + perpendicular_x)
        center_y = int(mid_y + perpendicular_y)
    else:
        center_x = int(mid_x - perpendicular_x)
        center_y = int(mid_y - perpendicular_y)

    start_vector = (ax - center_x, ay - center_y)
    end_vector = (bx - center_x, by - center_y)
    query_vector = (px - center_x, py - center_y)
    if not _inside_arc_wedge(
        start_vector,
        end_vector,
        query_vector,
        radius > 0.0,
        counter_clockwise,
    ):
        return False
    return abs(math.hypot(*query_vector) - abs(radius)) <= tolerance


def _entity_segments(entity: ExtrusionEntity) -> Iterator[object]:
    """Yield local and descendant segments in the entity's traversal order."""
    yield from entity.segments()
    for child in entity.children():
        yield from _entity_segments(child)


def _point_belongs_to_loop(loop: ExtrusionEntity, point: CPoint) -> bool:
    """Check the complete line-and-arc path without flattening curved segments."""
    tolerance = float(SCALED_EPSILON)
    return any(
        _point_belongs_to_segment(point, segment, tolerance)
        for segment in _entity_segments(loop)
    )


@abi_requires_headers("steps/slic3r_step_seam_placer.h")
class SeamPlacer:
    """Unique owner and consumer of one published seam-placer instance.

    Construction transfers the raw instance and clears its source structure.
    Use ``close()`` or a ``with`` block for deterministic destruction. The
    fallback finalizer is best-effort only. This wrapper never owns the plugin,
    PrintingPlan, or loop handles used by the session.
    """

    def __init__(self, api, instance: RawSeamPlacerInstance) -> None:
        if not isinstance(instance, RawSeamPlacerInstance):
            raise TypeError("instance must be a RawSeamPlacerInstance")
        if _instance_vtable(instance) is None:
            raise ValueError("the seam placer instance is incomplete")
        self.api = api
        self._instance = RawSeamPlacerInstance(
            int(instance.struct_size),
            ctypes.c_void_p(_address(instance.session)),
            ctypes.c_void_p(_address(instance.vtable)),
        )
        instance.struct_size = 0
        instance.session = None
        instance.vtable = None

    def valid(self) -> bool:
        """Return whether this wrapper still owns a complete callable session."""
        return _instance_vtable(self._instance) is not None

    def close(self) -> None:
        """Destroy the owned session exactly once and invalidate this wrapper."""
        vtable = _instance_vtable(self._instance)
        if vtable is not None:
            session = ctypes.c_void_p(_address(self._instance.session))
            self._instance = RawSeamPlacerInstance()
            vtable.contents.destroy(session)
        else:
            self._instance = RawSeamPlacerInstance()

    @abi_requires_headers("steps/slic3r_step_seam_placer.h")
    def place_seam(self, loop: ExtrusionEntity, start_position) -> CPoint:
        """Return a validated seam on one borrowed loop in scaled coordinates.

        The loop must use this API instance and belong to the plan used to
        initialize the provider. Plan membership is a caller precondition: the
        generic C API has no index that can verify it cheaply. Provider failures
        become ``RuntimeError`` and points outside the path are rejected without
        projection.
        """
        vtable = _instance_vtable(self._instance)
        if vtable is None:
            raise RuntimeError("the seam placer instance is invalid")
        if not isinstance(loop, ExtrusionEntity):
            raise TypeError("loop must be an ExtrusionEntity")
        if loop.api is not self.api:
            raise ValueError("loop must use this API instance")
        if not loop.is_loop():
            raise ValueError(
                "seam placement needs a nonempty fixed continuous closed loop"
            )

        result = RawSeamPlacerResult()
        result.struct_size = ctypes.sizeof(RawSeamPlacerResult)
        vtable.contents.place_seam(
            ctypes.c_void_p(_address(self._instance.session)),
            loop.c_handle(),
            as_point(start_position),
            ctypes.byref(result),
        )
        if int(result.status) != int(RAW_SEAM_PLACER_STATUS_SUCCESS):
            message = (
                result.error_message.decode("utf-8", errors="replace")
                if result.error_message
                else "the seam placer callback failed"
            )
            raise RuntimeError(message)
        if not _point_belongs_to_loop(loop, result.point):
            raise RuntimeError("the seam placer returned a point outside the loop path")
        return result.point

    def __enter__(self) -> "SeamPlacer":
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()

    def __copy__(self):
        raise TypeError("SeamPlacer has unique ownership and cannot be copied")

    def __deepcopy__(self, memo):
        raise TypeError("SeamPlacer has unique ownership and cannot be copied")

    def __bool__(self) -> bool:
        return self.valid()

    def __del__(self) -> None:
        try:
            self.close()
        except BaseException:
            pass


@abi_requires_headers("steps/slic3r_step_seam_placer.h")
class SeamPlacerContext:
    """Borrowed ``run_ctx_seam_placer`` used by a Python provider's run hook."""

    def __init__(
        self, api, common: PluginRunContext, payload: RunCtxSeamPlacer
    ) -> None:
        self.api = api
        self.common = common
        self.payload = payload

    @classmethod
    def from_seam_placer_run_context(
        cls, api, run_ctx_address: int
    ) -> "SeamPlacerContext | None":
        """Borrow a seam payload only from a valid SEAM_PLACER callback."""
        if not run_ctx_address:
            return None
        common = PluginRunContext.from_address(int(run_ctx_address))
        if int(common.step) != int(SEAM_PLACER) or not common.data:
            return None
        payload = ctypes.cast(
            common.data, ctypes.POINTER(RunCtxSeamPlacer)
        ).contents
        return cls(api, common, payload)

    @abi_requires_headers("steps/slic3r_step_seam_placer.h")
    def seam_placer_plan(self) -> PrintingPlan:
        """Borrow the plan that must outlive the session being published."""
        if not self.payload.plan:
            raise ValueError("the seam placer context has no PrintingPlan")
        return PrintingPlan(self.api, self.payload.plan)

    @abi_requires_headers("steps/slic3r_step_seam_placer.h")
    def publish_seam_placer(self, session: SeamPlacerSession) -> None:
        """Initialize and transfer one provider session into this payload."""
        if self.payload.instance.session or self.payload.instance.vtable:
            raise ValueError("the seam placer payload already owns a session")
        self.payload.instance = make_seam_placer_instance(
            session, self.seam_placer_plan()
        )


__all__ = [
    "RawSeamPlacerVTable",
    "SeamPlacer",
    "SeamPlacerContext",
    "SeamPlacerSession",
    "make_seam_placer_instance",
]
