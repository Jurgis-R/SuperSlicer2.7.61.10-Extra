#/|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
#/|/
#/|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#/|/

"""
Python helper for PERIMETER_GENERATION_MODULE plugins.

A perimeter module is not a full slicing step. It is called from the host-owned
perimeter loop while a STEP_PERIMETER generator walks one temporary
perimeter-node tree. A Python module publishes a vtable with up to four hooks:

    start(context) -> user_state
    before(user_state, context, node)
    after(user_state, context, node)
    end(user_state, context)

Context contents
----------------

PerimeterModuleContext only publishes the module vtable. The real work happens
later, when the host calls the published hooks with:

* PerimeterGenerationContextView, containing the current print/object/layer,
  island, LayerRegionIsland, temporary root node, plugin storage, and node
  editing callbacks;
* PerimeterNodeView, containing the node area, fill area, mutable extrusion
  bucket, perimeter indexes, and stable child snapshots;
* the optional user_state returned by start(), which lets a module keep
  per-run caches without using global data.

Typical use
-----------

    class MyModule(PluginBase):
        def __init__(self, api):
            super().__init__("my.module", PERIMETER_GENERATION_MODULE)
            self.api = api
            self._published_module = None

        def run(self, run_ctx_address):
            ctx = self.api.perimeter_module_context(run_ctx_address)
            if ctx is not None:
                self._published_module = ctx.publish(after=self.after)

        def after(self, state, context, node):
            if node.is_last_perimeter():
                node.add_perimeters(1)

The object returned by publish() owns the ctypes callbacks. Store it on the
plugin instance so the C vtable remains valid while the host perimeter loop is
using it.
"""

from __future__ import annotations

import ctypes
from typing import Callable

from slic3r_api_generated import (
    PERIMETER_GENERATION_MODULE,
    PERIMETER_GENERATION_MODULE_END,
    PERIMETER_GENERATION_MODULE_NODE,
    PERIMETER_GENERATION_MODULE_START,
    PLUGIN_REPORT,
    PerimeterGenerationContext,
    PerimeterNode,
    PluginRunContext,
    RunCtxPerimeterGenerationModule,
)
from .perimeter import PerimeterGenerationContextView, PerimeterNodeView
from slic3r_abi import abi_requires_headers


def _address(handle) -> int:
    if handle is None:
        return 0
    if isinstance(handle, ctypes.c_void_p):
        return int(handle.value or 0)
    return int(handle)


def _as_bytes(text: str) -> bytes:
    return text.encode("utf-8")


class PerimeterGenerationModuleVTable(ctypes.Structure):
    _fields_ = [
        ("start", PERIMETER_GENERATION_MODULE_START),
        ("before", PERIMETER_GENERATION_MODULE_NODE),
        ("after", PERIMETER_GENERATION_MODULE_NODE),
        ("end", PERIMETER_GENERATION_MODULE_END),
    ]


class PublishedPerimeterGenerationModule:
    """
    Owns the ctypes vtable and callback state exposed to the host.

    The host only stores raw function pointers, so this Python object must stay
    alive until perimeter generation finishes. PerimeterModuleContext.publish()
    returns it for the plugin instance to keep.
    """

    def __init__(
        self,
        api,
        start: Callable[[PerimeterGenerationContextView], object] | None,
        before: Callable[[object, PerimeterGenerationContextView, PerimeterNodeView], None] | None,
        after: Callable[[object, PerimeterGenerationContextView, PerimeterNodeView], None] | None,
        end: Callable[[object, PerimeterGenerationContextView], None] | None,
    ) -> None:
        self.api = api
        self._start_impl = start
        self._before_impl = before
        self._after_impl = after
        self._end_impl = end
        self._states: dict[int, object] = {}

        self._start_cb = PERIMETER_GENERATION_MODULE_START(self._start)
        self._before_cb = PERIMETER_GENERATION_MODULE_NODE(self._before)
        self._after_cb = PERIMETER_GENERATION_MODULE_NODE(self._after)
        self._end_cb = PERIMETER_GENERATION_MODULE_END(self._end)
        self.vtable = PerimeterGenerationModuleVTable(
            self._start_cb,
            self._before_cb,
            self._after_cb,
            self._end_cb,
        )
        self._vtable_ptr = ctypes.pointer(self.vtable)

    def vtable_handle(self) -> ctypes.c_void_p:
        return ctypes.cast(self._vtable_ptr, ctypes.c_void_p)

    def module_context_handle(self) -> ctypes.c_void_p:
        return ctypes.c_void_p(id(self))

    def _context(self, raw_context) -> PerimeterGenerationContextView:
        return PerimeterGenerationContextView(self.api, raw_context)

    def _node(self, raw_node) -> PerimeterNodeView:
        return PerimeterNodeView(self.api, raw_node)

    def _state_from_user_context(self, user_context) -> object | None:
        key = _address(user_context)
        return None if key == 0 else self._states.get(key)

    def _report_error(self, raw_context, message: str) -> None:
        if not raw_context:
            return
        context = raw_context.contents
        if context.run_ctx and context.run_ctx.contents.report_error:
            PLUGIN_REPORT(context.run_ctx.contents.report_error)(
                context.run_ctx.contents.host_context,
                _as_bytes(message),
            )

    def _start(self, module_ctx, raw_context: ctypes.POINTER(PerimeterGenerationContext)):
        try:
            state = None if self._start_impl is None else self._start_impl(self._context(raw_context))
            if state is None:
                return None
            key = id(state)
            self._states[key] = state
            return key
        except Exception as exc:
            self._report_error(raw_context, f"Python perimeter module start failed: {exc}")
            return None

    def _before(self, module_ctx, user_context, raw_context, raw_node: ctypes.POINTER(PerimeterNode)) -> None:
        if self._before_impl is None:
            return
        try:
            self._before_impl(self._state_from_user_context(user_context), self._context(raw_context), self._node(raw_node))
        except Exception as exc:
            self._report_error(raw_context, f"Python perimeter module before failed: {exc}")

    def _after(self, module_ctx, user_context, raw_context, raw_node: ctypes.POINTER(PerimeterNode)) -> None:
        if self._after_impl is None:
            return
        try:
            self._after_impl(self._state_from_user_context(user_context), self._context(raw_context), self._node(raw_node))
        except Exception as exc:
            self._report_error(raw_context, f"Python perimeter module after failed: {exc}")

    def _end(self, module_ctx, user_context, raw_context) -> None:
        key = _address(user_context)
        state = None if key == 0 else self._states.pop(key, None)
        if self._end_impl is None:
            return
        try:
            self._end_impl(state, self._context(raw_context))
        except Exception as exc:
            self._report_error(raw_context, f"Python perimeter module end failed: {exc}")


@abi_requires_headers("steps/slic3r_step_perimeter.h")
class PerimeterModuleContext:
    """
    High-level wrapper around the PERIMETER_GENERATION_MODULE payload.

    Use publish() from PluginBase.run() to provide the module callbacks to the
    host. The returned PublishedPerimeterGenerationModule must be kept alive by
    the plugin instance.
    """

    def __init__(self, api, common: PluginRunContext, payload: RunCtxPerimeterGenerationModule) -> None:
        self.api = api
        self.common = common
        self.payload = payload

    @classmethod
    def from_perimeter_module_run_context(cls, api, run_ctx_address: int) -> "PerimeterModuleContext | None":
        if not run_ctx_address:
            return None
        common = PluginRunContext.from_address(int(run_ctx_address))
        if common.step != PERIMETER_GENERATION_MODULE or not common.data:
            return None
        payload = ctypes.cast(common.data, ctypes.POINTER(RunCtxPerimeterGenerationModule)).contents
        return cls(api, common, payload)

    def publish(
        self,
        *,
        start: Callable[[PerimeterGenerationContextView], object] | None = None,
        before: Callable[[object, PerimeterGenerationContextView, PerimeterNodeView], None] | None = None,
        after: Callable[[object, PerimeterGenerationContextView, PerimeterNodeView], None] | None = None,
        end: Callable[[object, PerimeterGenerationContextView], None] | None = None,
    ) -> PublishedPerimeterGenerationModule:
        module = PublishedPerimeterGenerationModule(self.api, start, before, after, end)
        self.payload.module.ctx = module.module_context_handle()
        self.payload.module.vt = module.vtable_handle()
        return module


__all__ = [
    "PerimeterModuleContext",
    "PublishedPerimeterGenerationModule",
]
