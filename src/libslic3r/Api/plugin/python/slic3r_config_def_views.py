#/|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
#/|/
#/|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#/|/

"""Owning Python values for the global print configuration definitions.

The native query reuses ``RawConfigOptionDef`` as a compact snapshot whose
pointer fields last only until the next query on the same thread. This module
copies every field before returning, so callers may retain these frozen values
without owning or freeing any native memory. Editing them never changes the
host definition; new settings are still registered through ``create_option_def``.
"""

from __future__ import annotations

import ctypes
from dataclasses import dataclass

from slic3r_abi import abi_requires_headers
from slic3r_api_generated import RAW_CO_NONE, RawConfigOptionDef, RawConfigOptionKey


def _decode(value) -> str | None:
    """Copy one nullable UTF-8 pointer before its native snapshot expires."""
    if value is None:
        return None
    if isinstance(value, bytes):
        raw = value
    else:
        raw = ctypes.cast(value, ctypes.c_char_p).value
    return None if raw is None else raw.decode("utf-8", errors="replace")


def _copy_strings(values) -> tuple[str, ...]:
    """Copy a const_strings_t while silently ignoring defensive null entries."""
    copied = []
    if not values.items:
        return ()
    for index in range(int(values.size)):
        text = _decode(values.items[index])
        if text is not None:
            copied.append(text)
    return tuple(copied)


@dataclass(frozen=True)
class ConfigOptionKey:
    """One copied global option key and its raw ABI value type."""

    key: str
    type: int


@dataclass(frozen=True)
class ConfigOptionEnumEntry:
    """One copied enum value and its optional display label."""

    value: str | None
    label: str | None


@abi_requires_headers("slic3r_config_def.h")
@dataclass(frozen=True)
class ConfigOptionDefinition:
    """A complete immutable copy of one normalized host option definition."""

    opt_key: str
    type: int
    category: int
    level: int
    container_type: int
    option_preset_type: int
    gui_type: int
    printer_technology: int
    invalidates_step: int
    is_optional: bool
    gui_flags: str | None
    label: str | None
    full_label: str | None
    tooltip: str | None
    translation_domain: str | None
    sidetext: str | None
    cli: str | None
    ratio_over: str | None
    multiline: bool
    full_width: bool
    is_code: bool
    is_vector_extruder: bool
    readonly: bool
    can_phony: bool
    can_be_disabled: bool
    height: int
    width: int
    label_width: int
    aligned_label_left: bool
    sidetext_width: int
    min_value: float | None
    max_value: float | None
    max_literal_value: float | None
    max_literal_is_percent: bool
    precision: int
    mode: int
    flags: int
    aliases: tuple[str, ...]
    shortcut: tuple[str, ...]
    depends_on: tuple[str, ...]
    default_serialized_value: str | None
    enum_entries: tuple[ConfigOptionEnumEntry, ...]

    @classmethod
    def from_raw(cls, raw: RawConfigOptionDef) -> "ConfigOptionDefinition":
        """Deep-copy one native snapshot without retaining any native pointer."""
        pairs = raw.enum_def.value_label_pairs
        enum_entries = []
        if pairs.items:
            for index in range(int(pairs.count)):
                pair = pairs.items[index]
                enum_entries.append(ConfigOptionEnumEntry(_decode(pair.value), _decode(pair.label)))
        return cls(
            opt_key=_decode(raw.opt_key) or "",
            type=int(raw.type),
            category=int(raw.category),
            level=int(raw.level),
            container_type=int(raw.container_type),
            option_preset_type=int(raw.option_preset_type),
            gui_type=int(raw.gui_type),
            printer_technology=int(raw.printer_technology),
            invalidates_step=int(raw.invalidates_step),
            is_optional=bool(raw.is_optional),
            gui_flags=_decode(raw.gui_flags),
            label=_decode(raw.label),
            full_label=_decode(raw.full_label),
            tooltip=_decode(raw.tooltip),
            translation_domain=_decode(raw.translation_domain),
            sidetext=_decode(raw.sidetext),
            cli=_decode(raw.cli),
            ratio_over=_decode(raw.ratio_over),
            multiline=bool(raw.multiline),
            full_width=bool(raw.full_width),
            is_code=bool(raw.is_code),
            is_vector_extruder=bool(raw.is_vector_extruder),
            readonly=bool(raw.readonly),
            can_phony=bool(raw.can_phony),
            can_be_disabled=bool(raw.can_be_disabled),
            height=int(raw.height),
            width=int(raw.width),
            label_width=int(raw.label_width),
            aligned_label_left=bool(raw.aligned_label_left),
            sidetext_width=int(raw.sidetext_width),
            min_value=float(raw.min_value) if raw.has_min else None,
            max_value=float(raw.max_value) if raw.has_max else None,
            max_literal_value=float(raw.max_literal_value) if raw.has_max_literal else None,
            max_literal_is_percent=bool(raw.max_literal_is_percent),
            precision=int(raw.precision),
            mode=int(raw.mode),
            flags=int(raw.flags),
            aliases=_copy_strings(raw.aliases),
            shortcut=_copy_strings(raw.shortcut),
            depends_on=_copy_strings(raw.depends_on),
            default_serialized_value=_decode(raw.default_serialized_value),
            enum_entries=tuple(enum_entries),
        )


@abi_requires_headers("slic3r_config_def.h")
def option_keys(api, type_filter: int = RAW_CO_NONE) -> tuple[ConfigOptionKey, ...]:
    """Copy ordered option keys, optionally restricted to one exact raw type."""
    required = int(api.host.orchestrator_get_option_keys(api.orchestrator, int(type_filter), None, 0))
    while True:
        raw_values = (RawConfigOptionKey * required)()
        observed = int(
            api.host.orchestrator_get_option_keys(
                api.orchestrator,
                int(type_filter),
                raw_values if required else None,
                required,
            )
        )
        if observed <= required:
            return tuple(
                ConfigOptionKey(_decode(raw_values[index].key) or "", int(raw_values[index].type))
                for index in range(observed)
            )
        required = observed


@abi_requires_headers("slic3r_config_def.h")
def option_definition(api, key: str) -> ConfigOptionDefinition | None:
    """Copy one definition, returning None when the global key is unknown."""
    raw = RawConfigOptionDef()
    if not api.host.orchestrator_get_option_def(
        api.orchestrator, key.encode("utf-8"), ctypes.byref(raw)
    ):
        return None
    return ConfigOptionDefinition.from_raw(raw)


__all__ = [
    "ConfigOptionDefinition",
    "ConfigOptionEnumEntry",
    "ConfigOptionKey",
    "option_definition",
    "option_keys",
]
