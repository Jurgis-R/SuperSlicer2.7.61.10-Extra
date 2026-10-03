///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_plugin_cpp_properties_PrintingExtrusionScopeProperty_hpp_
#define slic3r_Api_plugin_cpp_properties_PrintingExtrusionScopeProperty_hpp_

/*
Shared metadata for ordered extrusion transitions
=================================================

STEP_LAYER_EXTRUSION_EDIT can mark fixed extrusion roots with this compact
payload. Later plan editors use the flags to identify the optional travel,
before-content and after-content phases without relying on child indexes.

The payload contains no pointers and is registered by name for each
orchestrator. Consumers must retain the returned typed key only for that
orchestrator's lifetime.
*/

#include <cstdint>

#include "libslic3r/Api/plugin/cpp/PluginPropertyKey.hpp"

#define PRINTING_EXTRUSION_SCOPE_PROPERTY_NAME \
    "slic3r.printing_plan.extrusion_scope"

namespace slic3r_api {

enum PrintingExtrusionScopeFlag : uint8_t
{
    PRINTING_EXTRUSION_SCOPE_INCOMING_TRANSITION = uint8_t(1u << 0),
    PRINTING_EXTRUSION_SCOPE_OUTGOING_TRANSITION = uint8_t(1u << 1),
    PRINTING_EXTRUSION_SCOPE_INCOMING_TRAVEL_MATERIALIZED = uint8_t(1u << 2),
    PRINTING_EXTRUSION_SCOPE_OUTGOING_TRAVEL_MATERIALIZED = uint8_t(1u << 3),
    PRINTING_EXTRUSION_SCOPE_START = uint8_t(1u << 4),
    PRINTING_EXTRUSION_SCOPE_TERMINAL = uint8_t(1u << 5),
    PRINTING_EXTRUSION_SCOPE_INCOMING_TOOLCHANGE = uint8_t(1u << 6),
    PRINTING_EXTRUSION_SCOPE_OUTGOING_TOOLCHANGE = uint8_t(1u << 7)
};

struct PrintingExtrusionScopeProperty
{
    uint8_t flags = 0;
};

inline bool printing_extrusion_scope_has_flag(
    const PrintingExtrusionScopeProperty &property,
    const PrintingExtrusionScopeFlag flag)
{
    return (property.flags & uint8_t(flag)) != 0;
}

inline void printing_extrusion_scope_set_flag(
    PrintingExtrusionScopeProperty &property,
    const PrintingExtrusionScopeFlag flag,
    const bool enabled)
{
    if (enabled)
        property.flags = uint8_t(property.flags | uint8_t(flag));
    else
        property.flags = uint8_t(property.flags & ~uint8_t(flag));
}

inline PluginPropertyKey<PrintingExtrusionScopeProperty>
printing_extrusion_scope_property_key(orchestrator_handle *orchestrator)
{
    return PluginPropertyKey<PrintingExtrusionScopeProperty>::register_dynamic(
        orchestrator, PRINTING_EXTRUSION_SCOPE_PROPERTY_NAME);
}

} // namespace slic3r_api

#endif // slic3r_Api_plugin_cpp_properties_PrintingExtrusionScopeProperty_hpp_
