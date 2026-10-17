///|/ Copyright (c) 2026 Jurgis-R
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
#ifndef slic3r_HybridNonplanarScheduler_hpp_
#define slic3r_HybridNonplanarScheduler_hpp_

#include <cmath>
#include <limits>
#include <vector>

namespace Slic3r {

// Pure, deterministic scheduling primitive for hybrid non-planar passes.
// It does not emit G-code or decide whether a pass is geometrically safe.
// Call after each planar layer has been processed; returned values are the
// H-interval boundaries that became due at that layer's top Z.
class HybridNonplanarScheduler
{
public:
    explicit HybridNonplanarScheduler(double interval_mm = 2.0, double epsilon_mm = 1e-6)
    {
        reset(interval_mm, epsilon_mm);
    }

    void reset(double interval_mm = 2.0, double epsilon_mm = 1e-6)
    {
        m_interval = interval_mm;
        m_epsilon = std::isfinite(epsilon_mm) && epsilon_mm >= 0.0 ? epsilon_mm : 1e-6;
        m_valid = std::isfinite(interval_mm) && interval_mm > 0.0;
        m_next_boundary = m_valid ? interval_mm : std::numeric_limits<double>::infinity();
        m_last_layer_top = -std::numeric_limits<double>::infinity();
    }

    bool valid() const { return m_valid; }
    double interval() const { return m_interval; }
    double next_boundary() const { return m_next_boundary; }

    // Returns every interval boundary reached by this planar layer. The
    // caller is responsible for running the returned passes before the next
    // planar layer. Invalid or out-of-order input fails closed (returns none).
    std::vector<double> after_planar_layer(double bottom_z, double top_z)
    {
        std::vector<double> due;
        if (!m_valid || !std::isfinite(bottom_z) || !std::isfinite(top_z) ||
            top_z + m_epsilon < bottom_z ||
            top_z + m_epsilon < m_last_layer_top)
            return due;

        while (m_next_boundary <= top_z + m_epsilon) {
            // Normally boundaries are emitted once because next_boundary is
            // advanced monotonically. This guard also protects future changes
            // to the scheduling loop from duplicating a boundary on retries.
            if (m_next_boundary > m_last_layer_top + m_epsilon)
                due.push_back(m_next_boundary);

            const double next = m_next_boundary + m_interval;
            if (!std::isfinite(next) || next <= m_next_boundary) {
                m_valid = false;
                m_next_boundary = std::numeric_limits<double>::infinity();
                break;
            }
            m_next_boundary = next;
        }

        if (top_z > m_last_layer_top)
            m_last_layer_top = top_z;
        return due;
    }

private:
    double m_interval { 2.0 };
    double m_epsilon { 1e-6 };
    double m_next_boundary { 2.0 };
    double m_last_layer_top { -std::numeric_limits<double>::infinity() };
    bool   m_valid { true };
};

} // namespace Slic3r

#endif // slic3r_HybridNonplanarScheduler_hpp_
