///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ Copyright (c) 2021 Ultimaker B.V.
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#ifndef plugins_lightning_distance_field_hpp_
#define plugins_lightning_distance_field_hpp_

#include "LightningGeometry.hpp"

namespace slic3r_api { namespace Infill { namespace Lightning {

// Keep sampling and cell removal synchronized with Fill/Lightning/DistanceField.cpp.
class DistanceField
{
public:
    struct Cell
    {
        work::Point location;
        coord_t distance;
        bool erased = false;
    };
    DistanceField(coord_t radius, const work::BoundingBox &bounds, const work::Polygons &overhang,
                   storage_handle *scratch, const plugin_run_context *run_ctx);
    // Stable cell order survives erasure so branch-growth priorities do not change.
    bool next(work::Point &point, size_t &index, size_t begin = 0) const;
    // The native reduced-bit distance test decides which supported cells stop requesting branches.
    void update(const work::Point &ground, const work::Point &leaf);
    const std::vector<Cell> &cells() const { return m_cells; }

private:
    coord_t m_radius;
    coord_t m_spacing;
    work::BoundingBox m_bounds;
    work::BoundingBox m_grid_range;
    coord_t m_grid_width = 0;
    std::vector<Cell> m_cells;
    std::vector<size_t> m_grid;
};
}}}
#endif
