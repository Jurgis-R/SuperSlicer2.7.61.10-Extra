///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "RectilinearSawtooth.hpp"

#include <cmath>
#include <cstdlib>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_utils.h"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Plugins/Infill/PatternPluginHelpers.hpp"

/*
Sawtooth rectilinear implementation
====================================

The legacy filler first creates ordinary rectilinear polylines and then splits
each one at randomized tooth distances. Each split is represented here by a
storage-owned leaf with explicit Z offsets and the same flow attributes as the
legacy ExtrusionPath. The intermediate multipath keeps its child order fixed;
the outer collection remains sortable and reversible like the legacy output.
*/

namespace slic3r_api { namespace Infill { namespace RectilinearPatternsPlugin {
namespace sawtooth {
namespace {

namespace rect_work = work;

struct SawtoothFlow
{
    double mm3_per_mm;
    float width;
    float height;
};

struct SawtoothSettings
{
    SawtoothFlow normal_flow;
    SawtoothFlow rise_flow;
    SawtoothFlow offset_flow;
    SawtoothFlow descend_flow;
    coord_t scaled_nozzle_diam;
    coord_t clearance;
    coord_t tooth_spacing_min;
    coord_t tooth_spacing_max;
    coord_t tooth_zhop;
};

// Split one base rectilinear polyline into the legacy sawtooth path sequence.
void append_sawtooth_polyline(StoredExtrusionEntity &root,
                              storage_handle *storage,
                              const rect_work::Polyline &polyline,
                              const raw_infill_pattern_params &params,
                              float fill_angle,
                              const SawtoothSettings &settings);
// Append one extrusion leaf with its per-point Z profile and flow attributes.
void append_sawtooth_path(StoredExtrusionEntity &destination,
                          storage_handle *storage,
                          const std::vector<c_point> &points,
                          const std::vector<coord_t> &z_offsets,
                          const SawtoothFlow &flow,
                          const raw_infill_pattern_params &params);

} // namespace

void publish_sawtooth(const run_ctx_infill_pattern &ctx,
                      storage_handle *storage,
                      const rect_work::Polylines &lines,
                      const raw_infill_pattern_params &params,
                      float fill_angle)
{
    const float nozzle = float(unscaled(params.flow.nozzle_diameter));
    const coord_t scaled_nozzle_diam = scale_i(nozzle);
    const coord_t clearance = scaled_nozzle_diam * 2.5;
    const coord_t tooth_spacing_min = scaled_nozzle_diam;
    const coord_t tooth_spacing_max = scaled_nozzle_diam * 3;
    const coord_t tooth_zhop = scaled_nozzle_diam;
    const double sqrt_two = std::sqrt(2.);
    const float flow_width = float(unscaled(params.flow.width));
    const float flow_height = float(unscaled(params.flow.height));

    const SawtoothSettings settings = {
        {
            params.flow.mm3_per_mm * double(params.flow_mult),
            flow_width * params.flow_mult,
            flow_height
        },
        { nozzle * nozzle * PI / 4., nozzle, nozzle },
        { 0., nozzle / 10.f, nozzle / 10.f },
        {
            params.flow.mm3_per_mm / sqrt_two,
            float(flow_width / sqrt_two),
            flow_height
        },
        scaled_nozzle_diam,
        clearance,
        tooth_spacing_min,
        tooth_spacing_max,
        tooth_zhop
    };

    StoredExtrusionEntity root(storage);
    for (const rect_work::Polyline &polyline : lines) {
        if (!polyline.is_valid())
            continue;
        append_sawtooth_polyline(root, storage, polyline, params, fill_angle, settings);
    }

    if (root.empty())
        return;

    root.set_flags(RAW_EXTRUSION_FLAG_SORTABLE | RAW_EXTRUSION_FLAG_REVERSIBLE);
    MutableExtrusionEntity output(ctx.output);
    output.append_child_move(root.mutable_view());
}

namespace {

void append_sawtooth_polyline(StoredExtrusionEntity &root,
                              storage_handle *storage,
                              const rect_work::Polyline &polyline,
                              const raw_infill_pattern_params &params,
                              float fill_angle,
                              const SawtoothSettings &settings)
{
    StoredExtrusionEntity multipath(storage);
    std::vector<c_point> current_points;
    std::vector<coord_t> current_z_offsets;
    current_points.reserve(polyline.points.size());
    current_z_offsets.reserve(polyline.points.size());

    const rect_work::Points &points = polyline.points;
    const bool compare_y = std::abs(std::abs(int(fill_angle * 180 / PI) % 180) - 90) > 45;
    coord_t next_zhop = settings.tooth_spacing_min + coord_t(std::abs(
        (slic3r_safe_rand(RAND_MAX) / float(RAND_MAX)) *
        (settings.tooth_spacing_max - settings.tooth_spacing_min)));
    size_t idx = 1;
    current_points.push_back({ points[0].x(), points[0].y() });
    current_z_offsets.push_back(0);
    rect_work::Point last = points[0];
    coord_t line_length = coord_t(points[idx - 1].distance_to(points[idx]));
    coord_t max_length = coord_t(polyline.length());

    while (idx < polyline.size() && max_length > settings.tooth_spacing_max) {
        // Skip return and tangent segments without placing a tooth on them.
        while (idx < polyline.size() && max_length > settings.tooth_spacing_min &&
               (next_zhop >= line_length || line_length < settings.clearance ||
                (compare_y ? points[idx].y() < points[idx - 1].y() :
                             points[idx].x() < points[idx - 1].x()))) {
            if (line_length >= settings.clearance &&
                !(compare_y ? points[idx].y() < points[idx - 1].y() :
                              points[idx].x() < points[idx - 1].x()))
                next_zhop -= line_length;

            max_length -= line_length;
            current_points.push_back({ points[idx].x(), points[idx].y() });
            current_z_offsets.push_back(0);
            last = points[idx];
            ++idx;
            if (idx < polyline.size())
                line_length = coord_t(last.distance_to(points[idx]));
        }

        // Split the line into normal extrusion, lift, clearance, descent, and normal extrusion.
        if (idx < polyline.size() && max_length > settings.clearance) {
            if (next_zhop > line_length - settings.scaled_nozzle_diam * 2)
                next_zhop = line_length - settings.scaled_nozzle_diam * 2.5;
            last = last.interpolate(next_zhop / double(line_length), points[idx]);
            if (last != points[idx - 1]) {
                current_points.push_back({ last.x(), last.y() });
                current_z_offsets.push_back(0);
            }
            append_sawtooth_path(multipath, storage, current_points, current_z_offsets,
                                 settings.normal_flow, params);
            current_points.clear();
            current_z_offsets.clear();

            current_points.push_back({ last.x(), last.y() });
            current_z_offsets.push_back(0);
            current_points.push_back({ last.x(), last.y() });
            current_z_offsets.push_back(settings.tooth_zhop);
            append_sawtooth_path(multipath, storage, current_points, current_z_offsets,
                                 settings.rise_flow, params);
            current_points.clear();
            current_z_offsets.clear();

            const rect_work::Point offset_start = last;
            line_length = coord_t(last.distance_to(points[idx]));
            last = last.interpolate(settings.scaled_nozzle_diam / double(line_length), points[idx]);
            append_sawtooth_path(multipath, storage,
                                 std::vector<c_point>{{ offset_start.x(), offset_start.y() },
                                                      { last.x(), last.y() }},
                                 std::vector<coord_t>{ settings.tooth_zhop, settings.tooth_zhop },
                                 settings.offset_flow, params);
            current_points.clear();
            current_z_offsets.clear();

            const rect_work::Point descend_start = last;
            line_length = coord_t(last.distance_to(points[idx]));
            last = last.interpolate(settings.scaled_nozzle_diam / double(line_length), points[idx]);
            append_sawtooth_path(multipath, storage,
                                 std::vector<c_point>{{ descend_start.x(), descend_start.y() },
                                                      { last.x(), last.y() }},
                                 std::vector<coord_t>{ settings.tooth_zhop, 0 },
                                 settings.descend_flow, params);
            current_points.clear();
            current_z_offsets.clear();

            current_points.push_back({ last.x(), last.y() });
            current_z_offsets.push_back(0);
            line_length = coord_t(last.distance_to(points[idx]));
            // Continue the original line at normal flow after the tooth.
            next_zhop = settings.tooth_spacing_min + coord_t(std::abs(
                (slic3r_safe_rand(RAND_MAX) / float(RAND_MAX)) *
                (settings.tooth_spacing_max - settings.tooth_spacing_min)));
        }
    }

    // Finish the remaining rectilinear line after the final tooth.
    while (idx < polyline.size()) {
        current_points.push_back({ points[idx].x(), points[idx].y() });
        current_z_offsets.push_back(0);
        ++idx;
    }
    append_sawtooth_path(multipath, storage, current_points, current_z_offsets,
                         settings.normal_flow, params);

    if (multipath.empty())
        return;
    multipath.set_flags(0);
    root.append_child_move(multipath.mutable_view());
}

void append_sawtooth_path(StoredExtrusionEntity &destination,
                          storage_handle *storage,
                          const std::vector<c_point> &points,
                          const std::vector<coord_t> &z_offsets,
                          const SawtoothFlow &flow,
                          const raw_infill_pattern_params &params)
{
    // The legacy tree can retain a one-point normal segment when interpolation
    // lands on the previous point. The plugin drops that non-printable leaf as
    // an accepted boundary-case parity limitation.
    if (points.size() < 2)
        return;

    StoredExtrusionEntity path(storage, points);
    for (size_t idx = 0; idx < z_offsets.size(); ++idx)
        path.set_z_offset(uint32_t(idx), z_offsets[idx]);
    path.get_or_add(EPropertyAttributes::key)
        .extrusion_role(int32_t(params.extrusion_role))
        .mm3_per_mm(flow.mm3_per_mm)
        .width(flow.width)
        .height(flow.height);
    path.disable_reverse();
    destination.append_child_move(path.mutable_view());
}

} // namespace
} // namespace sawtooth
}}} // namespace slic3r_api::Infill::RectilinearPatternsPlugin
