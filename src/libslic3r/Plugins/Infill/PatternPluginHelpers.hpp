///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef plugins_infill_patternpluginhelpers_hpp_
#define plugins_infill_patternpluginhelpers_hpp_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_def.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_infill.h"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "libslic3r/Api/plugin/cpp/OrderingEngine.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/Plugins/Ordering/NearestNeighborOrderingEngine.hpp"

/*
Shared helpers for built-in INFILL_PATTERN plugins
==================================================

These helpers factor out the pieces every simple pattern needs: recipe
interpretation, the legacy infill frame, path ordering through the shared
OrderingEngine and extrusion publication. They use the public plugin API and
the ordering engine interface.

The angle convention follows the legacy Fill pipeline: the pattern receives the
configured fill angle, and the effective pattern direction is that angle plus a
quarter turn because Fill::_infill_direction() added M_PI/2 before calling the
pattern. Pattern-specific layer rotations (honeycomb thirds, the gyroid -45
degree override) are added by the plugin on top of infill_angle().
*/

namespace slic3r_api { namespace Infill { namespace PatternHelpers {

constexpr double k_pi = 3.14159265358979323846;
constexpr double k_half_pi = k_pi / 2.;
constexpr double k_quarter_pi = k_pi / 4.;

// Match Fill::_adjust_solid_spacing(): widen intervals to fit the object, bounded by factor_max.
inline coord_t adjust_solid_spacing(coord_t width, coord_t distance, double factor_max = 1.2)
{
    assert(width >= 0);
    assert(distance > 0);
    const coord_t number_of_intervals = coord_t((width - EPSILON) / distance);
    coord_t distance_new = number_of_intervals == 0 ? distance : coord_t((width - EPSILON) / number_of_intervals);
    const double factor = coordf_t(distance_new) / coordf_t(distance);
    if (factor > factor_max)
        distance_new = coord_t(std::floor(coordf_t(distance) * factor_max + 0.5));
    return distance_new;
}

// Distance between infill lines for the requested density, in scaled units.
// Honors the sparse spacing cap prepared by the generator when the pattern
// supports it.
inline coord_t line_spacing_for_density(const raw_infill_pattern_params &params)
{
    if (params.density <= 0.f)
        return 0;
    // Both candidates are already unscaled millimeters.
    const double spacing = params.max_sparse_infill_spacing > 0.f ?
        double(params.max_sparse_infill_spacing) : params.spacing;
    if (spacing <= 0.)
        return 0;
    return scale_i(spacing / double(params.density));
}

// Direction of the pattern in radians, including the quarter turn that
// Fill::_infill_direction() applied on top of the configured fill angle.
inline double infill_angle(const raw_infill_pattern_params &params)
{
    return double(params.angle) + k_half_pi;
}

// Layer rotation of the former FillGyroid::_layer_angle(): a constant -45
// degree correction when the fill angle may cross the model, instead of the
// default per-layer quarter turn.
inline double gyroid_layer_angle(const raw_infill_pattern_params &params)
{
    return (params.infill_flags & C_INFILL_FLAG_CAN_ANGLE_CROSS) != 0 ? -k_quarter_pi : 0.;
}

// Shrink the surface by half the line spacing, exactly like
// Fill::fill_surface() did before dispatching one expolygon to the pattern.
inline StoredExPolygonCollection shrink_for_fill(storage_handle *storage,
                                                 const ExPolygon &surface_expolygon,
                                                 double spacing)
{
    ClipperOperand subject(storage, surface_expolygon);
    return clipper_offset(storage, subject, scale_d(-0.5 * spacing)).to_expolygon_collection();
}

/*
Clip one generated polyline against the shrunk surface and append the resulting
pieces to a collection. Pieces shorter than min_length are dropped.

Every piece stays a separate path so STEP_ORDERING can insert travels. Joining
two pieces instead would extrude a straight segment across any hole or void that
the clipping left between them.
*/
inline void append_clipped_pieces(storage_handle *storage,
                                  const std::vector<c_point> &points,
                                  const ExPolygonCollection &clip,
                                  double min_length,
                                  StoredPolylineCollection &out)
{
    StoredPolyline polyline(storage);
    for (const c_point point : points)
        polyline.push_back(point);
    if (polyline.size() < 2)
        return;

    const StoredPolylineCollection clipped = clipper_intersection_polyline_expolygons(storage, polyline, clip);
    for (const Polyline piece : clipped) {
        if (piece.size() < 2 || piece.length() < min_length)
            continue;
        out.push_back(piece);
    }
}

/*
Order the clipped pieces in place with the shared connect_infill() helper.

Ordering here only reduces the initial travel distance; the final order and
reversals still belong to STEP_ORDERING.
*/
inline void order_pieces(StoredPolylineCollection &pieces)
{
    Ordering::connect_infill(pieces, Ordering::DefaultOrderingPlugin::NearestNeighborOrderingEngine{});
}

// Apply the flow attributes shared by every stored extrusion path.
inline void set_path_attributes(StoredExtrusionEntity &path, const raw_infill_pattern_params &params)
{
    if ((params.infill_flags & C_INFILL_FLAG_MONOTONIC) != 0)
        path.disable_reverse();
    path.get_or_add(EPropertyAttributes::key)
        .extrusion_role(int32_t(params.extrusion_role))
        .mm3_per_mm(params.flow.mm3_per_mm * double(params.flow_mult))
        .width(float(unscaled(params.flow.width) * double(params.flow_mult)))
        .height(float(unscaled(params.flow.height)));
}

/*
Create one stored path carrying the recipe flow attributes, exactly like
extrusion_entities_append_paths() did for the legacy Fill path.
*/
inline StoredExtrusionEntity make_path(storage_handle *storage,
                                       const std::vector<c_point> &points,
                                       const raw_infill_pattern_params &params)
{
    StoredExtrusionEntity path(storage, points);
    set_path_attributes(path, params);
    return path;
}

// Convert a rectilinear work polyline to API points in one shared helper.
template<class Point>
inline StoredExtrusionEntity make_path(storage_handle *storage,
                                       const std::vector<Point> &points,
                                       const raw_infill_pattern_params &params)
{
    std::vector<c_point> converted;
    converted.reserve(points.size());
    for (const Point &point : points)
        converted.push_back(c_point{ point.x(), point.y() });
    return make_path(storage, converted, params);
}

/*
Create one stored path from an API polyline view without first copying its
points into a temporary vector.
*/
inline StoredExtrusionEntity make_path(storage_handle *storage,
                                       const Polyline &polyline,
                                       const raw_infill_pattern_params &params)
{
    StoredExtrusionEntity path(storage);
    path.set_polyline(polyline);
    set_path_attributes(path, params);
    return path;
}

// Append one stored path carrying the recipe flow attributes.
inline void append_polyline(MutableExtrusionEntity parent, storage_handle *storage,
                            const Polyline &polyline, const raw_infill_pattern_params &params)
{
    StoredExtrusionEntity path = make_path(storage, polyline, params);
    parent.append_child_move(path.mutable_view());
}

inline void append_polyline(MutableExtrusionEntity parent, storage_handle *storage,
                            const std::vector<c_point> &points, const raw_infill_pattern_params &params)
{
    StoredExtrusionEntity path = make_path(storage, points, params);
    parent.append_child_move(path.mutable_view());
}

// Wrap paths into a sortable group and publish them into the pattern output.
inline void publish_infill_paths(const run_ctx_infill_pattern &ctx, storage_handle *storage,
                                 const StoredPolylineCollection &paths,
                                 const raw_infill_pattern_params &params)
{
    StoredExtrusionEntity group(storage);
    group.set_flags(group.flags() | RAW_EXTRUSION_FLAG_SORTABLE | RAW_EXTRUSION_FLAG_REVERSIBLE);
    for (uint32_t index = 0; index < paths.size(); ++index) {
        const Polyline path = paths.at(index);
        if (path.size() >= 2)
            append_polyline(group.mutable_view(), storage, path, params);
    }

    if (!group.empty()) {
        MutableExtrusionEntity output(ctx.output);
        output.append_child_move(group.mutable_view());
    }
}

// Point emitted by a plane-path generator, in units of the line spacing.
struct GridPoint
{
    double x = 0.;
    double y = 0.;
};

// Common implementation of the legacy FillPlanePath::_fill_surface_single()
// frame handling. generate() receives the grid bounds in units of the line
// spacing and appends grid points to out.
template<class GenerateFn>
void fill_plane_path_surface(const run_ctx_infill_pattern &ctx,
                             storage_handle *storage,
                             const ExPolygon &surface_expolygon,
                             const raw_infill_pattern_params &params,
                             bool centered,
                             GenerateFn &&generate)
{
    if (params.spacing <= 0. || params.density <= 0.f)
        return;

    StoredExPolygonCollection clip = shrink_for_fill(storage, surface_expolygon, params.spacing);
    if (clip.empty())
        return;

    const double angle = infill_angle(params);
    const double line_spacing = double(scale_d(params.spacing)) / double(params.density);
    if (line_spacing <= 0.)
        return;

    // Frame of the pattern in the rotated frame. Sparse infill is aligned across
    // layers on the object bounding box, while solid infill only needs the
    // shrunk surface, exactly like the legacy FillPlanePath.
    c_bounding_box pattern = bounding_box_empty();
    if (params.density < 0.995) {
        pattern = bounding_box_rotate(params.object_bounding_box, -angle, c_point{0, 0});
    } else {
        for (const ExPolygon &expolygon : clip)
            bounding_box_merge_rotated(pattern, expolygon, -angle, c_point{0, 0});
    }
    if (!bounding_box_valid(pattern))
        return;

    const c_point shift = centered ? bounding_box_center(pattern) : pattern.min;

    const coord_t min_x = coord_t(std::ceil(double(pattern.min.x - shift.x) / line_spacing));
    const coord_t min_y = coord_t(std::ceil(double(pattern.min.y - shift.y) / line_spacing));
    const coord_t max_x = coord_t(std::ceil(double(pattern.max.x - shift.x) / line_spacing));
    const coord_t max_y = coord_t(std::ceil(double(pattern.max.y - shift.y) / line_spacing));
    const double resolution = double(params.fill_resolution) / line_spacing;

    std::vector<GridPoint> grid_points;
    generate(min_x, min_y, max_x, max_y, resolution, grid_points);
    if (grid_points.size() < 2)
        return;

    // Move the generated points from the pattern frame back into the surface
    // frame, where they can be clipped against the original area. The rounding
    // matches the legacy InfillPolylineOutput::scaled().
    const double cos_angle = std::cos(angle);
    const double sin_angle = std::sin(angle);
    std::vector<c_point> surface_points;
    surface_points.reserve(grid_points.size());
    for (const GridPoint point : grid_points) {
        const double x = std::floor(point.x * line_spacing + 0.5) + double(shift.x);
        const double y = std::floor(point.y * line_spacing + 0.5) + double(shift.y);
        surface_points.push_back({ coord_t(std::llround(x * cos_angle - y * sin_angle)),
                                   coord_t(std::llround(x * sin_angle + y * cos_angle)) });
    }

    StoredPolylineCollection pieces(storage);
    append_clipped_pieces(storage, surface_points, clip, 0., pieces);
    if (pieces.empty())
        return;

    order_pieces(pieces);
    publish_infill_paths(ctx, storage, pieces, params);
}

// Declare that this plugin belongs to one or more pattern option lists. The
// host keeps the contribution even when the list is created later.
inline void add_self_to_pattern_list(orchestrator_handle *orchestrator,
                                     const char *plugin_id,
                                     const char *label,
                                     std::initializer_list<const char *> keys)
{
    for (const char *key : keys) {
        key_value_string_pair_t entry = { plugin_id, label };
        key_value_string_pair_array_t entries = { &entry, 1 };

        raw_option_def_patch patch = raw_option_def_patch_init();
        patch.kind = RAW_OPTION_DEF_PATCH_ADD_ENUM_VALUES;
        patch.opt_key = key;
        patch.enum_values = entries;
        orchestrator_modify_option_def(orchestrator, &patch);
    }
}

}}} // namespace slic3r_api::Infill::PatternHelpers

#endif // plugins_infill_patternpluginhelpers_hpp_
