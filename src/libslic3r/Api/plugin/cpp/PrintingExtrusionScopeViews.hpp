///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_plugin_cpp_PrintingExtrusionScopeViews_hpp_
#define slic3r_Api_plugin_cpp_PrintingExtrusionScopeViews_hpp_

/*
Named views over prepared extrusion transitions
===============================================

The transition-scope producer stores a small property on fixed extrusion roots.
This borrowed view exposes the optional travel, pre-content, printable-content
and post-content entities while validating the producer's child layout. It
allows plan-edit plugins to insert ordered work without copying the producer's
private child-index rules.
*/

#include <stdexcept>

#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/properties/PrintingExtrusionScopeProperty.hpp"

namespace slic3r_api {

class PrintingExtrusionScopeView
{
public:
    PrintingExtrusionScopeView(
        MutableExtrusionEntity root,
        PluginPropertyKey<PrintingExtrusionScopeProperty> key) :
        m_root(root), m_key(key)
    {
        if (!m_root.valid())
            throw std::invalid_argument("A printing extrusion scope needs a valid root.");
        validate_shape();
    }

    MutableExtrusionEntity root() const { return m_root; }

    bool has_incoming_transition() const
    {
        return flag(PRINTING_EXTRUSION_SCOPE_INCOMING_TRANSITION);
    }

    bool has_outgoing_transition() const
    {
        return flag(PRINTING_EXTRUSION_SCOPE_OUTGOING_TRANSITION);
    }

    bool has_incoming_toolchange() const
    {
        return flag(PRINTING_EXTRUSION_SCOPE_INCOMING_TOOLCHANGE);
    }

    MutableExtrusionEntity travel() const
    {
        return has_incoming_transition() ? m_root.child_mutable(0) : MutableExtrusionEntity();
    }

    MutableExtrusionEntity before() const
    {
        return has_incoming_transition() ? m_root.child_mutable(1) : MutableExtrusionEntity();
    }

    MutableExtrusionEntity content() const
    {
        if (has_incoming_transition())
            return m_root.child_mutable(2);
        return has_outgoing_transition() ? m_root.child_mutable(0) : m_root;
    }

    MutableExtrusionEntity after() const
    {
        if (!has_outgoing_transition())
            return MutableExtrusionEntity();
        return m_root.child_mutable(has_incoming_transition() ? 3 : 1);
    }

private:
    bool flag(PrintingExtrusionScopeFlag value) const
    {
        const PrintingExtrusionScopeProperty *property = m_root.get(m_key);
        if (property == nullptr)
            throw std::runtime_error("A printing extrusion scope lost its marker.");
        return printing_extrusion_scope_has_flag(*property, value);
    }

    void validate_shape() const
    {
        const PrintingExtrusionScopeProperty *property = m_root.get(m_key);
        if (property == nullptr)
            throw std::invalid_argument("The entity is not a marked printing extrusion scope.");
        if (m_root.readonly().sortable() || m_root.readonly().reversible())
            throw std::runtime_error("A printing extrusion scope must have fixed child order.");

        const bool incoming = printing_extrusion_scope_has_flag(
            *property, PRINTING_EXTRUSION_SCOPE_INCOMING_TRANSITION);
        const bool outgoing = printing_extrusion_scope_has_flag(
            *property, PRINTING_EXTRUSION_SCOPE_OUTGOING_TRANSITION);
        if ((printing_extrusion_scope_has_flag(
                 *property, PRINTING_EXTRUSION_SCOPE_INCOMING_TRAVEL_MATERIALIZED) ||
             printing_extrusion_scope_has_flag(
                 *property, PRINTING_EXTRUSION_SCOPE_INCOMING_TOOLCHANGE)) && !incoming)
            throw std::runtime_error(
                "Incoming transition facts need an incoming extrusion phase.");
        if ((printing_extrusion_scope_has_flag(
                 *property, PRINTING_EXTRUSION_SCOPE_OUTGOING_TRAVEL_MATERIALIZED) ||
             printing_extrusion_scope_has_flag(
                 *property, PRINTING_EXTRUSION_SCOPE_OUTGOING_TOOLCHANGE)) && !outgoing)
            throw std::runtime_error(
                "Outgoing transition facts need an outgoing extrusion phase.");
        const uint32_t expected_children = incoming && outgoing ? 4u :
            (incoming || outgoing ? (incoming ? 3u : 2u) : 0u);
        if (expected_children != 0 && m_root.child_count() != expected_children)
            throw std::runtime_error("A printing extrusion scope has an invalid phase layout.");
    }

    MutableExtrusionEntity m_root;
    PluginPropertyKey<PrintingExtrusionScopeProperty> m_key;
};

inline bool is_printing_extrusion_scope(
    const ExtrusionEntity &entity,
    const PluginPropertyKey<PrintingExtrusionScopeProperty> &key)
{
    return key.get(entity) != nullptr;
}

} // namespace slic3r_api

#endif // slic3r_Api_plugin_cpp_PrintingExtrusionScopeViews_hpp_
