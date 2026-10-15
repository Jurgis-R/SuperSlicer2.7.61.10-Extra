///|/ Copyright (c) 2026 Jurgis-R
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
#ifndef slic3r_HybridNonplanarSectionPlanner_hpp_
#define slic3r_HybridNonplanarSectionPlanner_hpp_

#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>

namespace Slic3r {

// Describes one completed planar H section. This class deliberately does not
// retain extrusion paths: those paths belong to their original Z and must not
// be replayed later as a substitute for generating a new surface path set.
struct HybridNonplanarSection
{
    double start_z { 0.0 };
    double boundary_z { 0.0 };
    double boundary_layer_top_z { 0.0 };
    std::size_t planar_layer_count { 0 };
};

// Tracks section boundaries at the layer-group level. Call exactly once for
// each completed planar layer group, after all objects and tools at that print
// Z have finished. A caller must not emit a finishing pass until a section is
// complete and a separate surface-path planner has produced safe paths for it.
//
// If a single layer spans multiple H boundaries, or layer order is invalid, the
// planner enters a failed state rather than inventing ambiguous section splits.
class HybridNonplanarSectionPlanner
{
public:
    explicit HybridNonplanarSectionPlanner(double interval_mm = 2.0,
                                           double epsilon_mm = 1e-6)
    {
        reset(interval_mm, epsilon_mm);
    }

    void reset(double interval_mm = 2.0, double epsilon_mm = 1e-6,
               double initial_z = 0.0)
    {
        m_interval = interval_mm;
        m_epsilon = std::isfinite(epsilon_mm) && epsilon_mm >= 0.0
            ? epsilon_mm : 1e-6;
        m_valid = std::isfinite(interval_mm) && interval_mm > 0.0 &&
                  std::isfinite(initial_z) && initial_z >= 0.0;
        m_section_start_z = initial_z;
        m_next_boundary_z = m_valid ? initial_z + interval_mm
                                    : std::numeric_limits<double>::infinity();
        m_last_layer_top_z = initial_z;
        m_layer_count = 0;
        if (!std::isfinite(m_next_boundary_z) ||
            (m_valid && m_next_boundary_z <= initial_z))
            m_valid = false;
    }

    bool valid() const { return m_valid; }
    double next_boundary_z() const { return m_next_boundary_z; }
    double section_start_z() const { return m_section_start_z; }
    std::size_t current_section_layer_count() const { return m_layer_count; }

    // Returns a completed section only when this layer reaches its boundary.
    // nullopt means either the section is still in progress or input failed;
    // inspect valid() to distinguish those cases.
    std::optional<HybridNonplanarSection> after_planar_layer(
        double bottom_z, double top_z)
    {
        if (!m_valid || !std::isfinite(bottom_z) || !std::isfinite(top_z) ||
            top_z + m_epsilon < bottom_z ||
            bottom_z + m_epsilon < m_last_layer_top_z ||
            top_z <= m_last_layer_top_z + m_epsilon) {
            m_valid = false;
            return std::nullopt;
        }

        ++m_layer_count;
        m_last_layer_top_z = top_z;
        if (top_z + m_epsilon < m_next_boundary_z)
            return std::nullopt;

        // Do not pretend one indivisible planar layer can complete two H
        // sections. A section-level path generator must explicitly handle that
        // case before this planner can safely accept it.
        if (top_z - m_interval >= m_next_boundary_z - m_epsilon) {
            m_valid = false;
            return std::nullopt;
        }

        HybridNonplanarSection completed {
            m_section_start_z,
            m_next_boundary_z,
            top_z,
            m_layer_count
        };

        m_section_start_z = m_next_boundary_z;
        m_layer_count = 0;
        m_next_boundary_z += m_interval;
        if (!std::isfinite(m_next_boundary_z) ||
            m_next_boundary_z <= m_section_start_z)
            m_valid = false;
        return completed;
    }

private:
    double m_interval { 2.0 };
    double m_epsilon { 1e-6 };
    double m_section_start_z { 0.0 };
    double m_next_boundary_z { 2.0 };
    double m_last_layer_top_z { 0.0 };
    std::size_t m_layer_count { 0 };
    bool m_valid { true };
};

} // namespace Slic3r

#endif // slic3r_HybridNonplanarSectionPlanner_hpp_
