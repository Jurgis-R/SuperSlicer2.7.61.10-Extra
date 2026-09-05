#/|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
#/|/
#/|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#/|/

"""
Typed keys for plugin properties stored on data-tree and extrusion objects.

A key couples a ctypes payload layout with the numeric property id understood
by the host. Built-in keys are process-wide. Dynamic keys additionally retain
the API/orchestrator that registered them, because their numeric ids are local
to that orchestrator and must never be reused with another one.

The key owns no native payload and allocates no storage. Values returned by
``get()`` are independent Python copies; values returned by ``get_mutable()``
or ``get_or_add()`` are borrowed views into host memory and become invalid when
their owning object or property container is destroyed or structurally changed.
"""

from __future__ import annotations

import ctypes

from slic3r_api_generated import SLIC3R_PROPERTY_TYPE_INVALID
from slic3r_abi import abi_requires_headers


def _address(handle) -> int:
    if handle is None:
        return 0
    if isinstance(handle, ctypes.c_void_p):
        return int(handle.value or 0)
    return int(handle)


def _validate_payload_class(payload_cls) -> None:
    if not isinstance(payload_cls, type) or not issubclass(payload_cls, ctypes.Structure):
        raise TypeError("a plugin property payload must be a ctypes.Structure class")
    if ctypes.sizeof(payload_cls) == 0:
        raise ValueError("a plugin property payload must not be empty")


@abi_requires_headers("slic3r_orchestrator.h")
class PluginPropertyKey:
    """
    Typed identity of one property payload.

    Construct keys with ``built_in()`` or ``register_dynamic()``. Dynamic keys
    keep their originating Slic3rAPI alive and reject owners from another API.
    The object itself needs no explicit release; the orchestrator owns the
    registration and the data-tree/extrusion object owns every stored payload.
    """

    __slots__ = ("payload_cls", "property_type", "api", "orchestrator", "name")

    def __init__(
        self,
        payload_cls,
        property_type: int,
        *,
        api=None,
        orchestrator=None,
        name: str = "",
    ) -> None:
        _validate_payload_class(payload_cls)
        numeric_type = int(property_type)
        if numeric_type == int(SLIC3R_PROPERTY_TYPE_INVALID):
            raise ValueError("property_type must identify a registered property")
        self.payload_cls = payload_cls
        self.property_type = numeric_type
        self.api = api
        self.orchestrator = None if api is None else ctypes.c_void_p(_address(orchestrator))
        self.name = str(name)

    @classmethod
    def built_in(cls, payload_cls) -> "PluginPropertyKey":
        """Create a typed key for a built-in payload class carrying ``property_type``."""
        if not hasattr(payload_cls, "property_type"):
            raise TypeError("a built-in payload class must define property_type")
        return cls(payload_cls, payload_cls.property_type)

    @classmethod
    def register_dynamic(
        cls,
        api,
        namespaced_name: str,
        payload_cls,
        orchestrator=None,
    ) -> "PluginPropertyKey":
        """
        Register a stable payload layout and return its orchestrator-bound key.

        ``namespaced_name`` must be globally descriptive for the plugin, such
        as ``com.example.plugin.surface_priority``. Re-registering that name is
        accepted only when size and alignment are identical. The numeric id is
        runtime-local and must not be serialized or copied to another API.
        """
        _validate_payload_class(payload_cls)
        if not isinstance(namespaced_name, str) or not namespaced_name:
            raise ValueError("a dynamic property needs a nonempty namespaced name")
        selected_orchestrator = api.orchestrator if orchestrator is None else orchestrator
        orchestrator_address = _address(selected_orchestrator)
        if orchestrator_address == 0:
            raise ValueError("a dynamic property needs a valid orchestrator")
        property_type = int(api.host.orchestrator_register_property(
            ctypes.c_void_p(orchestrator_address),
            namespaced_name.encode("utf-8"),
            ctypes.sizeof(payload_cls),
            ctypes.alignment(payload_cls),
        ))
        if property_type == int(SLIC3R_PROPERTY_TYPE_INVALID):
            raise ValueError(
                f"cannot register property {namespaced_name!r}: the name is invalid "
                "or is already associated with a different payload layout"
            )
        return cls(
            payload_cls,
            property_type,
            api=api,
            orchestrator=orchestrator_address,
            name=namespaced_name,
        )

    def validate_owner(self, owner) -> None:
        """Reject use of an orchestrator-local key through another API instance."""
        if self.api is not None and getattr(owner, "api", None) is not self.api:
            raise ValueError("a dynamic property key belongs to a different API/orchestrator")

    def type(self) -> int:
        """Return the numeric host property id; never serialize dynamic ids."""
        return self.property_type

    def has(self, owner) -> bool:
        """Test presence without creating a payload or changing its owner."""
        self.validate_owner(owner)
        method = getattr(owner, "has_property", None)
        return bool(method(self) if method is not None else owner.has(self))

    def get(self, owner):
        """Return an independent payload copy, or ``None`` when it is absent."""
        self.validate_owner(owner)
        if hasattr(owner, "readonly"):
            owner = owner.readonly()
        method = getattr(owner, "property", None)
        return method(self) if method is not None else owner.get(self)

    def get_mutable(self, owner):
        """Return a borrowed mutable payload view, without creating it."""
        self.validate_owner(owner)
        method = getattr(owner, "property_mutable", None)
        if method is not None:
            return method(self)
        if not hasattr(owner, "get_mutable"):
            raise TypeError("this property owner does not expose mutable payload access")
        return owner.get_mutable(self)

    def get_or_add(self, owner):
        """Return or create a borrowed mutable payload in the owning object."""
        self.validate_owner(owner)
        method = getattr(owner, "property", None)
        if method is not None and hasattr(owner, "mutable_c_handle"):
            payload = method(self, self.orchestrator)
        elif hasattr(owner, "get_or_add"):
            payload = owner.get_or_add(self, self.orchestrator)
        else:
            raise TypeError("this property owner does not expose mutable payload creation")
        if payload is None:
            raise RuntimeError("the host could not create the registered plugin property")
        return payload

    def remove(self, owner) -> bool:
        """Remove the payload from its owner; existing borrowed views expire."""
        self.validate_owner(owner)
        method = getattr(owner, "remove_property", None)
        return bool(method(self) if method is not None else owner.remove(self))


__all__ = ["PluginPropertyKey"]
