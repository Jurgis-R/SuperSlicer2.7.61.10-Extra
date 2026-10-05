///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "ArachneInfill.hpp"

#include <algorithm>
#include <limits>
#include <vector>

#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"

namespace slic3r_api { namespace Infill { namespace ArachneInfill {

namespace {

const char *const k_min_feature_size_key = "min_feature_size";
const char *const k_min_bead_width_key = "min_bead_width";
const char *const k_wall_transition_filter_deviation_key = "wall_transition_filter_deviation";
const char *const k_wall_transition_length_key = "wall_transition_length";
const char *const k_wall_transition_angle_key = "wall_transition_angle";
const char *const k_wall_distribution_count_key = "wall_distribution_count";
const char *const k_nozzle_diameter_key = "nozzle_diameter";

// A length in millimeters, or a percentage of the smallest nozzle diameter, as a scaled length.
coord_t resolve_length(const Config &region, const char *key, float min_nozzle_diameter);
// Append one ring to an Arachne outline.
void append_ring(const Polygon &ring, Slic3r::Polygons &out);
// Append a contour, then its holes, to an Arachne outline.
void append_area(const ExPolygon &area, Slic3r::Polygons &out);

coord_t resolve_length(const Config &region, const char *key, float min_nozzle_diameter)
{
    // Same operations as Arachne::wall_tool_paths_settings().
    const c_float_or_percent value = region.float_or_percent_or_default(key, c_float_or_percent{0., 0});
    return value.percent != 0 ? scale_i(value.value * 0.01 * min_nozzle_diameter) : scale_i(value.value);
}

void append_ring(const Polygon &ring, Slic3r::Polygons &out)
{
    const multipoint_const_view points = ring.view();
    Slic3r::Polygon &polygon = out.emplace_back();
    polygon.points.reserve(points.size);
    for (uint32_t index = 0; index < points.size; ++index)
        polygon.points.emplace_back(points.array[index].x, points.array[index].y);
}

void append_area(const ExPolygon &area, Slic3r::Polygons &out)
{
    append_ring(area.contour(), out);
    for (const Polygon hole : area.holes())
        append_ring(hole, out);
}

} // namespace

const raw_used_config_key k_settings_keys[k_settings_key_count] = {
    { k_min_feature_size_key, RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { k_min_bead_width_key, RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { k_wall_transition_filter_deviation_key, RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { k_wall_transition_length_key, RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { k_wall_transition_angle_key, RAW_CO_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { k_wall_distribution_count_key, RAW_CO_INT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { k_nozzle_diameter_key, RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE }
};

Slic3r::Arachne::WallToolPathsSettings read_settings(const Config &region, const Config &print)
{
    Slic3r::Arachne::WallToolPathsSettings settings;
    double min_nozzle_diameter = std::numeric_limits<double>::max();
    const uint32_t nozzle_count = print.size(k_nozzle_diameter_key, 0);
    for (uint32_t index = 0; index < nozzle_count; ++index)
        min_nozzle_diameter = std::min(min_nozzle_diameter, print.vector_float_or_default(k_nozzle_diameter_key, index, 0.));
    settings.min_nozzle_diameter = float(min_nozzle_diameter);
    settings.min_feature_size = resolve_length(region, k_min_feature_size_key, settings.min_nozzle_diameter);
    settings.min_bead_width = resolve_length(region, k_min_bead_width_key, settings.min_nozzle_diameter);
    settings.wall_transition_filter_deviation =
        resolve_length(region, k_wall_transition_filter_deviation_key, settings.min_nozzle_diameter);
    settings.wall_transition_length = resolve_length(region, k_wall_transition_length_key, settings.min_nozzle_diameter);
    settings.wall_transition_angle = region.float_or_default(k_wall_transition_angle_key, settings.wall_transition_angle);
    settings.wall_distribution_count =
        region.int_or_default(k_wall_distribution_count_key, settings.wall_distribution_count);
    return settings;
}

Slic3r::Arachne::WallToolPathsSettings read_settings(const run_ctx_infill_pattern &ctx)
{
    return read_settings(LayerRegion(ctx.primary_region).print_region().config(), Print(ctx.print).config());
}

Slic3r::Polygons to_polygons(const ExPolygonCollection &areas)
{
    Slic3r::Polygons out;
    for (const ExPolygon area : areas)
        append_area(area, out);
    return out;
}

Slic3r::Polygons to_polygons(const ExPolygon &area)
{
    Slic3r::Polygons out;
    append_area(area, out);
    return out;
}

Slic3r::Polygons to_polygons(const PolygonCollection &polygons)
{
    Slic3r::Polygons out;
    for (const Polygon polygon : polygons)
        append_ring(polygon, out);
    return out;
}

StoredPolygonCollection to_api(storage_handle *storage, const Slic3r::Polygons &polygons)
{
    StoredPolygonCollection out(storage);
    for (const Slic3r::Polygon &polygon : polygons) {
        StoredPolygon ring(storage);
        for (const Slic3r::Point &point : polygon.points)
            ring.push_back(c_point{point.x(), point.y()});
        out.push_back(ring);
    }
    return out;
}

size_t nearest_point_index(const Slic3r::Points &points, const Slic3r::Point &reference)
{
    lengthsqr_t best = std::numeric_limits<lengthsqr_t>::max();
    size_t nearest = 0;
    for (size_t index = 0; index < points.size(); ++index) {
        // The x distance alone already rules out most candidates.
        lengthsqr_t distance = coord_int_sqr(points[index].x() - reference.x());
        if (distance < best) {
            distance += coord_int_sqr(points[index].y() - reference.y());
            if (distance < best) {
                nearest = index;
                best = distance;
            }
        }
    }
    return nearest;
}

std::optional<StoredExtrusionEntity> to_extrusion(storage_handle *storage, const Slic3r::ThickPolyline &line,
                                                  const raw_infill_pattern_params &params)
{
    std::vector<c_point> points;
    std::vector<coord_t> widths;
    points.reserve(line.points.size());
    widths.reserve(line.points.size());
    for (size_t index = 0; index < line.points.size(); ++index) {
        points.push_back(c_point{line.points[index].x(), line.points[index].y()});
        widths.push_back(line.points_width[index]);
    }

    c_thick_polyline_extrusion_params conversion = {};
    conversion.role = params.extrusion_role;
    conversion.flow = params.flow;
    // A larger spacing than usual keeps the flow of the regular spacing.
    if (conversion.flow.spacing_ratio < 1.f && conversion.flow.is_bridge == 0)
        conversion.flow.spacing_ratio = 1.f;
    conversion.flow_mult = params.flow_mult;
    conversion.flags = THICK_POLYLINE_EXTRUSION_FLOW_FROM_SPACING;
    if ((params.infill_flags & C_INFILL_FLAG_MONOTONIC) == 0)
        conversion.flags |= THICK_POLYLINE_EXTRUSION_CAN_REVERSE;
    return thick_polyline_extrusion(storage, points, widths, conversion);
}

}}} // namespace slic3r_api::Infill::ArachneInfill
