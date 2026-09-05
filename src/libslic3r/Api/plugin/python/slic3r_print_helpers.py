#/|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
#/|/
#/|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#/|/

"""Python ports of slicer print calculations.

These helpers intentionally use only the public C-backed Python views. They are
algorithm snapshots, not independent policy: whenever the cited C++ source is
changed, compare this module with that source and update the port if necessary.

``Print``, ``Object`` and ``PrintRegion`` arguments are borrowed and must remain
alive for each synchronous call. Returned tuples and scalar values are ordinary
Python values and require no explicit release.
"""

from __future__ import annotations

import math
from collections.abc import Iterable

from slic3r_abi import abi_requires_headers
from slic3r_api_generated import EPSILON, SCALED_EPSILON, UNSCALING_FACTOR
from slic3r_datatree_views import LayerSupportProperty, Object, Print, PrintRegion


# Ported from src/libslic3r/Api/plugin/cpp/PrintHelpers.cpp
# at commit cca788f1cecab755cf623cb53d5e6298b4f6b6c4 (05/09/2026).
def _require_print_and_object(print_view: Print, obj: Object) -> None:
    if not isinstance(print_view, Print):
        raise TypeError("print_view must be a Print")
    if not isinstance(obj, Object):
        raise TypeError("obj must be an Object")
    if print_view.api is not obj.api:
        raise ValueError("print_view and obj must use the same API instance")


def _normalize_extruder_id(print_view: Print, one_based_id: int) -> int:
    nozzle_diameter = print_view.config().get("nozzle_diameter")
    idx = max(0, int(one_based_id) - 1)
    return (
        0
        if nozzle_diameter is None
        or nozzle_diameter.size() == 0
        or idx >= nozzle_diameter.size()
        else idx
    )


@abi_requires_headers("slic3r_config.h", "slic3r_config_option.h", "slic3r_data_tree.h")
def collect_object_printing_extruders(
    print_view: Print, obj: Object, region: PrintRegion
) -> tuple[int, ...]:
    """Return sorted zero-based tools used by one printing region.

    The three views are borrowed. ``obj`` must belong to ``print_view`` and
    ``region`` must belong to ``obj``; the public C API does not provide a cheap
    generic ownership check, so violating either precondition is an error. The
    returned tuple is copied and remains valid after the data tree changes.
    """
    _require_print_and_object(print_view, obj)
    if not isinstance(region, PrintRegion):
        raise TypeError("region must be a PrintRegion")
    if print_view.api is not region.api:
        raise ValueError("print_view and region must use the same API instance")

    object_config = obj.config()
    region_config = region.config()
    extruders: set[int] = set()

    def add(one_based_id: int) -> None:
        extruders.add(_normalize_extruder_id(print_view, one_based_id))

    if (
        region_config.option("perimeters").get_int() > 0
        or object_config.option("brim_width").get_float() > 0.0
        or object_config.option("brim_width_interior").get_float() > 0.0
    ):
        add(region_config.option("perimeter_extruder").get_int())
    if region_config.option("fill_density").get_float() > 0.0:
        add(region_config.option("infill_extruder").get_int())
    if (
        region_config.option("top_solid_layers").get_int() > 0
        or region_config.option("bottom_solid_layers").get_int() > 0
        or (
            region_config.option("solid_infill_every_layers").get_int() > 0
            and region_config.option("fill_density").get_float() > 0.0
        )
    ):
        add(region_config.option("solid_infill_extruder").get_int())
    return tuple(sorted(extruders))


@abi_requires_headers("slic3r_config.h", "slic3r_config_option.h", "slic3r_data_tree.h")
def object_extruders(print_view: Print, obj: Object) -> tuple[int, ...]:
    """Return sorted zero-based tools used by all printing regions of ``obj``.

    Input views are borrowed and must belong to the same live Print. The tuple
    is a copied value and requires no explicit release.
    """
    _require_print_and_object(print_view, obj)
    extruders: set[int] = set()
    for region in obj.print_regions():
        extruders.update(collect_object_printing_extruders(print_view, obj, region))
    return tuple(sorted(extruders))


@abi_requires_headers("slic3r_config.h", "slic3r_config_option.h", "slic3r_data_tree.h")
def collect_print_extruders_for_skirt(
    print_view: Print, objects: Iterable[Object] | None = None
) -> tuple[int, ...]:
    """Return sorted zero-based extruders that may print skirt.

    ``None`` examines every regular object in the borrowed Print. An explicit
    empty iterable is distinct and produces the native fallback tool zero.
    Support tools are considered only for objects carrying a support auxiliary
    layer, exactly as in the cited C++ helper. Nothing is retained or freed.
    """
    if not isinstance(print_view, Print):
        raise TypeError("print_view must be a Print")
    selected = list(print_view.objects() if objects is None else objects)
    extruders: set[int] = set()
    support_uses_current_extruder = False

    for obj in selected:
        _require_print_and_object(print_view, obj)
        extruders.update(object_extruders(print_view, obj))

        has_support_auxiliary = any(
            layer.properties().get(LayerSupportProperty.key) is not None
            for layer in obj.auxiliary_layers()
        )
        if not has_support_auxiliary:
            continue

        object_config = obj.config()
        support_extruder = object_config.int_or_default("support_material_extruder", 0)
        interface_extruder = object_config.int_or_default(
            "support_material_interface_extruder", 0
        )
        if support_extruder == 0:
            support_uses_current_extruder = True
        else:
            extruders.add(_normalize_extruder_id(print_view, support_extruder))
        if object_config.int_or_default("support_material_interface_layers", 0) > 0:
            if interface_extruder == 0:
                support_uses_current_extruder = True
            else:
                extruders.add(_normalize_extruder_id(print_view, interface_extruder))

    if support_uses_current_extruder:
        for obj in selected:
            extruders.update(object_extruders(print_view, obj))
    if not extruders:
        extruders.add(0)
    return tuple(sorted(extruders))


@abi_requires_headers("slic3r_config.h", "slic3r_config_option.h", "slic3r_data_tree.h")
def get_object_first_layer_height(print_view: Print, obj: Object) -> int:
    """Return one object's derived first-layer height in scaled coordinates.

    Absolute values are scaled directly. Percent values are evaluated against
    each used nozzle and retain the smallest result. Inputs are borrowed and no
    reference to configuration storage is returned.
    """
    _require_print_and_object(print_view, obj)
    first_layer_height = obj.config().option("first_layer_height")
    if not first_layer_height.is_percent():
        z_mm = first_layer_height.get_float()
        coord_z = int((z_mm + float(EPSILON) / 2.0) * float(UNSCALING_FACTOR))
        return coord_z // int(SCALED_EPSILON) * int(SCALED_EPSILON)

    print_config = print_view.config()
    extruders = object_extruders(print_view, obj)
    if not extruders:
        extruders = (0,)
    return min(
        (
            int(
                (
                    first_layer_height.get_effective_value(
                        print_config.vector_float_or_default(
                            "nozzle_diameter", extruder_id, 0.4
                        )
                    )
                    + float(EPSILON) / 2.0
                )
                * float(UNSCALING_FACTOR)
            )
            // int(SCALED_EPSILON)
            * int(SCALED_EPSILON)
        )
        for extruder_id in extruders
    )


@abi_requires_headers("slic3r_config.h", "slic3r_config_option.h", "slic3r_data_tree.h")
def get_min_first_layer_height(print_view: Print) -> int:
    """Return the smallest scaled first-layer height among regular objects.

    The Print is borrowed. An empty Print has no meaningful first-layer context
    and raises ``RuntimeError`` instead of returning a sentinel.
    """
    if not isinstance(print_view, Print):
        raise TypeError("print_view must be a Print")
    objects = list(print_view.objects())
    if not objects:
        raise RuntimeError("Cannot compute first-layer height without Print objects.")
    return min(get_object_first_layer_height(print_view, obj) for obj in objects)


@abi_requires_headers("slic3r_config.h", "slic3r_config_option.h", "slic3r_data_tree.h")
def e_per_mm(print_view: Print, extruder_id: int, mm3_per_mm: float) -> float:
    """Convert volumetric flow to filament E per path millimeter.

    ``extruder_id`` is zero-based and values use unscaled millimeters. The Print
    is borrowed and unmodified. A non-positive effective filament diameter is
    rejected explicitly; the returned float owns no native memory.
    """
    if not isinstance(print_view, Print):
        raise TypeError("print_view must be a Print")
    if int(extruder_id) < 0:
        raise ValueError("extruder_id must be non-negative")
    print_config = print_view.config()
    multiplier = print_config.vector_float_or_default(
        "extrusion_multiplier", int(extruder_id), 1.0
    )
    if print_config.bool_or_default("use_volumetric_e", False):
        return float(mm3_per_mm) * multiplier

    diameter = print_config.vector_float_or_default(
        "filament_diameter", int(extruder_id), 1.75
    )
    filament_area = diameter * diameter * 0.25 * math.pi
    if filament_area <= 0.0:
        raise RuntimeError("Cannot compute E per mm with a non-positive filament diameter.")
    return float(mm3_per_mm) * multiplier / filament_area


@abi_requires_headers("slic3r_def.h")
def check_z_step(value: int, z_step: int) -> int:
    """Round one scaled coordinate to the C++ helper's Z-step grid.

    A disabled or negligible step leaves the value unchanged. This pure helper
    owns no native resources and requires no API instance.
    """
    value = int(value)
    z_step = int(z_step)
    if z_step <= int(SCALED_EPSILON):
        return value
    numerator = value * 2 + z_step
    denominator = z_step * 2
    quotient = abs(numerator) // denominator
    if numerator < 0:
        quotient = -quotient
    return quotient * z_step


__all__ = [
    "check_z_step",
    "collect_object_printing_extruders",
    "collect_print_extruders_for_skirt",
    "e_per_mm",
    "get_min_first_layer_height",
    "get_object_first_layer_height",
    "object_extruders",
]
