#/|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
#/|/
#/|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#/|/

"""Declarative C ABI requirements for Python plugin bindings.

The decorator records stable C header names on Python callables and classes.
It deliberately does not resolve versions: package generation resolves them
from the SDK headers used for the build, so source code never duplicates ABI
version numbers.

Binding authors use it to expose the native contract behind a high-level
wrapper. Plugin authors may also put it on a function or class containing an
intentional dynamic API lookup. In that case every C header reachable through
the dynamic lookup must be listed, because the static packager treats the
declaration as the author's complete override for that scope.
"""

from __future__ import annotations


ABI_HEADERS_ATTRIBUTE = "__slic3r_abi_headers__"


def abi_requires_headers(*headers: str):
    """Attach C API header requirements without wrapping or executing the target."""
    normalized = []
    for header in headers:
        if not isinstance(header, str) or not header.strip():
            raise ValueError("ABI header names must be nonempty strings")
        name = header.strip().replace("\\", "/")
        if name not in normalized:
            normalized.append(name)
    if not normalized:
        raise ValueError("At least one ABI header is required")

    def decorate(target):
        existing = tuple(getattr(target, ABI_HEADERS_ATTRIBUTE, ()))
        merged = tuple(dict.fromkeys(existing + tuple(normalized)))
        setattr(target, ABI_HEADERS_ATTRIBUTE, merged)
        return target

    return decorate


__all__ = ["ABI_HEADERS_ATTRIBUTE", "abi_requires_headers"]
