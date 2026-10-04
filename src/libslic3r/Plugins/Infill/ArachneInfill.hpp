///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef plugins_infill_arachneinfill_hpp_
#define plugins_infill_arachneinfill_hpp_

#include <cstddef>
#include <optional>

#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_infill.h"
#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "libslic3r/Arachne/WallToolPaths.hpp"

/*
Arachne helpers for infill pattern plugins
==========================================

The infill patterns that need variable-width walls run Arachne::WallToolPaths
directly. These helpers keep the plugin boundary of such a pattern: API
geometry and configuration views in, storage-owned API extrusion out.

    read_settings()       resolve the WallToolPaths settings from the configs
    to_polygons()         copy API geometry into an Arachne outline
    to_api()              copy Arachne polygons back into the API, for Clipper
    to_extrusion()        convert one Arachne line with the recipe role and flow

The Arachne lines are Slic3r::ThickPolyline values, whose widths are spacings.
to_extrusion() converts them like the legacy Fill::fill_surface_extrusion():
the flow is rebuilt from the recipe spacing, with a spacing ratio of 1 when the
recipe ratio is lower and the surface is not a bridge.
*/

namespace slic3r_api { namespace Infill { namespace ArachneInfill {

// Number of configuration keys read by read_settings().
constexpr size_t k_settings_key_count = 7;
// Configuration keys read by read_settings(); a pattern declares them in its used_config_keys().
extern const raw_used_config_key k_settings_keys[k_settings_key_count];

// Resolve the WallToolPaths settings from the region and the print configurations.
Slic3r::Arachne::WallToolPathsSettings read_settings(const Config &region, const Config &print);
// Resolve the WallToolPaths settings of the region and the print of one surface.
Slic3r::Arachne::WallToolPathsSettings read_settings(const run_ctx_infill_pattern &ctx);
// Copy expolygons into an Arachne outline: each contour, then its holes.
Slic3r::Polygons to_polygons(const ExPolygonCollection &areas);
// Copy one expolygon into an Arachne outline: its contour, then its holes.
Slic3r::Polygons to_polygons(const ExPolygon &area);
// Copy polygons into an Arachne outline.
Slic3r::Polygons to_polygons(const PolygonCollection &polygons);
// Copy Arachne polygons into a storage-owned API collection.
StoredPolygonCollection to_api(storage_handle *storage, const Slic3r::Polygons &polygons);
// Index of the first point nearest to a reference point, like nearest_point_index().
size_t nearest_point_index(const Slic3r::Points &points, const Slic3r::Point &reference);
// Convert one Arachne line with the recipe role and flow; nothing if it does not extrude.
std::optional<StoredExtrusionEntity> to_extrusion(storage_handle *storage, const Slic3r::ThickPolyline &line,
                                                  const raw_infill_pattern_params &params);

}}} // namespace slic3r_api::Infill::ArachneInfill

#endif // plugins_infill_arachneinfill_hpp_
