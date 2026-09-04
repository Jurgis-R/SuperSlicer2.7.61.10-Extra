#/|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
#/|/
#/|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#/|/

"""Common high-level view over ``plugin_run_context``.

Every object-step payload starts with the same host-owned run context.  This
module keeps those operations in one binding class so calls such as
``ctx.report_error()`` require only ``slic3r_plugin_run_context.h``.  Step
modules inherit this view and give their payload-specific accessors unique
names; the static ABI analyzer can therefore identify the precise step contract
without inferring the receiver's Python type.

The view borrows the run context and its storage.  It is valid only for the
current plugin callback and never frees either object.
"""

from __future__ import annotations

import ctypes

from slic3r_api_generated import (
    PLUGIN_IS_CANCELLED,
    PLUGIN_REPORT,
    PLUGIN_REPORT_PROGRESS,
    PluginRunContext,
)
from slic3r_abi import abi_requires_headers


def _address(handle) -> int:
    if handle is None:
        return 0
    if isinstance(handle, ctypes.c_void_p):
        return int(handle.value or 0)
    if hasattr(handle, "c_handle"):
        return _address(handle.c_handle())
    return int(handle)


def _as_bytes(text: str) -> bytes:
    return text.encode("utf-8")


def _optional_bytes(text: str | None) -> bytes | None:
    return None if text is None else text.encode("utf-8")


@abi_requires_headers("slic3r_plugin_run_context.h")
class PluginRunContextView:
    """Expose operations shared by every step-specific run context."""

    def __init__(self, api, common: PluginRunContext) -> None:
        self.api = api
        self.common = common

    def plugin_storage(self) -> int:
        return _address(self.common.plugin_storage)

    def is_cancelled(self) -> bool:
        if not self.common.is_cancelled:
            return False
        return bool(PLUGIN_IS_CANCELLED(self.common.is_cancelled)(self.common.host_context))

    def report_warning(self, message: str) -> None:
        if self.common.report_warning:
            PLUGIN_REPORT(self.common.report_warning)(self.common.host_context, _as_bytes(message))

    def report_error(self, message: str) -> None:
        if self.common.report_error:
            PLUGIN_REPORT(self.common.report_error)(self.common.host_context, _as_bytes(message))

    def report_progress(self, progress: float, message: str | None = None) -> None:
        if self.common.report_progress:
            PLUGIN_REPORT_PROGRESS(self.common.report_progress)(
                self.common.host_context, float(progress), _optional_bytes(message)
            )


__all__ = ["PluginRunContextView"]
