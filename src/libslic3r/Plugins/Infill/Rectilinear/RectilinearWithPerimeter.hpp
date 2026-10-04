///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef plugins_infill_rectilinear_with_perimeter_hpp_
#define plugins_infill_rectilinear_with_perimeter_hpp_

/*
Rectilinear-with-perimeter infill
=================================

This helper builds the perimeter-aware extrusion tree used by the
`rectiwithperimeter` pattern. It creates the perimeter from two clipped offset
operations, fills the area behind it with the already configured rectilinear
pattern, and publishes both parts under one storage-owned root.
*/

#include "RectilinearFills.hpp"

#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"

namespace slic3r_api { namespace Infill { namespace RectilinearPatternsPlugin {
namespace recti {

class RectiWithPerimeter
{
public:
    static StoredExtrusionEntity fill_with_perimeter(
        storage_handle *storage,
        const slic3r_api::ExPolygon &surface,
        const fills::FillRectilinear &infill,
        const raw_infill_pattern_params &params);
};

} // namespace recti
}}} // namespace slic3r_api::Infill::RectilinearPatternsPlugin

#endif // plugins_infill_rectilinear_with_perimeter_hpp_
