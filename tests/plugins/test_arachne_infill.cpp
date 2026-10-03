#include <catch2/catch.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"

#include "libslic3r/Api/host/ApiHostUtils.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityVisitors.hpp"
#include "libslic3r/Fill/FillBase.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/Geometry/MedialAxis.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Plugins/Infill/ArachneInfill.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepGenerateInfill.hpp"
#include "libslic3r/Steps/StepGeneratePerimeter.hpp"
#include "libslic3r/Steps/StepSurfaceGeneration.hpp"
#include "libslic3r/Surface.hpp"

/*
Arachne infill plugin tests
===========================

The host conversion of variable-width lines is compared with
Geometry::thin_variable_width(), and the plugin settings with the settings the
legacy WallToolPaths constructor resolves.
*/

using namespace Slic3r;
using namespace Slic3r::Test::PerimeterPluginTests;

namespace {

// Region settings of one parity case, as option values.
using Settings = std::vector<std::pair<std::string, std::string>>;

// A deterministic text of a native entity: flags, then the points, role and flow of each leaf.
std::string snapshot(const ExtrusionEntity &entity);
// The same text of an API entity.
std::string snapshot(const slic3r_api::ExtrusionEntity &entity);
// The text of one leaf.
std::string leaf_text(const std::vector<std::pair<coord_t, coord_t>> &points, int32_t role, double mm3_per_mm,
                      float width, float height);
// A region configuration with some settings changed.
PrintRegionConfig region_config(const Settings &settings);

std::string leaf_text(const std::vector<std::pair<coord_t, coord_t>> &points, int32_t role, double mm3_per_mm,
                      float width, float height)
{
    std::string text;
    for (const std::pair<coord_t, coord_t> &point : points)
        text += std::to_string(point.first) + "," + std::to_string(point.second) + ";";
    char flow[96];
    std::snprintf(flow, sizeof(flow), "|%d|%.17g|%.9g|%.9g", int(role), mm3_per_mm, double(width), double(height));
    return text + flow;
}

std::string snapshot(const ExtrusionEntity &entity)
{
    std::string text = std::string(entity.is_loop() ? "L" : "") + (entity.can_sort() ? "S" : "") +
                       (entity.can_reverse() ? "R" : "") + "{";
    if (entity.is_leaf()) {
        const ExtrusionPath *path = dynamic_cast<const ExtrusionPath *>(&entity);
        REQUIRE(path != nullptr);
        std::vector<std::pair<coord_t, coord_t>> points;
        for (size_t index = 0; index < path->polyline().size(); ++index)
            points.emplace_back(path->polyline().get_point(index).x(), path->polyline().get_point(index).y());
        const ExtrusionAttributes *attributes = path->get_property<ExtrusionAttributes>();
        REQUIRE(attributes != nullptr);
        return text + leaf_text(points, int32_t(attributes->role), attributes->mm3_per_mm, attributes->width,
                                attributes->height) + "}";
    }
    for (size_t index = 0; index < entity.child_count(); ++index)
        text += snapshot(entity.child(index));
    return text + "}";
}

std::string snapshot(const slic3r_api::ExtrusionEntity &entity)
{
    std::string text = std::string(entity.is_loop() ? "L" : "") + (entity.sortable() ? "S" : "") +
                       (entity.reversible() ? "R" : "") + "{";
    if (entity.is_leaf()) {
        std::vector<std::pair<coord_t, coord_t>> points;
        for (const c_point point : entity.points())
            points.emplace_back(point.x, point.y);
        const slic3r_api::EPropertyAttributes *attributes = entity.get(slic3r_api::EPropertyAttributes::key);
        REQUIRE(attributes != nullptr);
        const c_extrusion_property_attributes &flow = *attributes;
        return text + leaf_text(points, attributes->extrusion_role(), flow.mm3_per_mm, flow.width, flow.height) + "}";
    }
    for (const slic3r_api::ExtrusionEntity child : entity.children())
        text += snapshot(child);
    return text + "}";
}

PrintRegionConfig region_config(const Settings &settings)
{
    DynamicPrintConfig changes;
    for (const std::pair<std::string, std::string> &setting : settings)
        changes.set_deserialize_strict(setting.first, setting.second);
    PrintRegionConfig region;
    region.apply(changes, true);
    return region;
}

} // namespace

TEST_CASE("Thick polyline extrusion matches thin_variable_width", "[plugins][api][extrusion][arachne]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ThickPolyline open;
    open.points = { Point(0, 0), Point(scale_i(3.), 0), Point(scale_i(6.), scale_i(1.)), Point(scale_i(9.), scale_i(1.)) };
    open.points_width = { scale_i(0.45), scale_i(0.45), scale_i(0.7), scale_i(0.31) };
    ThickPolyline closed;
    closed.points = { Point(0, 0), Point(scale_i(5.), 0), Point(scale_i(5.), scale_i(5.)), Point(0, scale_i(5.)), Point(0, 0) };
    closed.points_width = { scale_i(0.45), scale_i(0.5), scale_i(0.6), scale_i(0.5), scale_i(0.45) };
    ThickPolyline constant;
    constant.points = { Point(0, 0), Point(scale_i(4.), scale_i(3.)) };
    constant.points_width = { scale_i(0.45), scale_i(0.45) };

    for (const ThickPolyline &line : { open, closed, constant })
        for (const float ratio : { 1.f, 0.7f })
            for (const bool can_reverse : { true, false })
                for (const coord_t tolerance : { coord_t(0), scale_i(0.02) }) {
                    CAPTURE(line.points.size(), ratio, can_reverse, tolerance);
                    // The conversion rebuilds the flow from the scaled spacing.
                    const Flow requested = Flow::new_from_spacing(0.45f, 0.4f, 0.2f, ratio, false);
                    const Flow flow = Flow::new_from_spacing(float(unscaled(requested.scaled_spacing())), 0.4f, 0.2f, ratio, false);
                    const coord_t resolution = flow.scaled_width() / 8;
                    ExtrusionEntitiesPtr native = tolerance > 0 ?
                        Geometry::thin_variable_width({line}, ExtrusionRole::SolidInfill, flow, resolution, tolerance, can_reverse) :
                        Geometry::thin_variable_width({line}, ExtrusionRole::SolidInfill, flow, resolution, can_reverse);
                    REQUIRE(native.size() == 1);
                    // Fill multiplies by the float flow_mult of the recipe.
                    ExtrusionModifyFlow(double(0.9f)).traverse(*native.front());
                    const std::string expected = snapshot(*native.front());
                    delete native.front();

                    c_thick_polyline_extrusion_params params = {};
                    params.role = RAW_EXTRUSION_ROLE_SOLID_INFILL;
                    params.flow.width = flow.scaled_width();
                    params.flow.spacing = flow.scaled_spacing();
                    params.flow.height = flow.scaled_height();
                    params.flow.nozzle_diameter = scale_i(flow.nozzle_diameter());
                    params.flow.spacing_ratio = flow.spacing_ratio();
                    params.width_change_tolerance = tolerance;
                    params.flow_mult = 0.9f;
                    params.flags = THICK_POLYLINE_EXTRUSION_FLOW_FROM_SPACING |
                                   (can_reverse ? THICK_POLYLINE_EXTRUSION_CAN_REVERSE : 0u);
                    std::vector<c_point> points;
                    for (const Point &point : line.points)
                        points.push_back(c_point{point.x(), point.y()});
                    PluginStorage storage;
                    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
                    const std::optional<slic3r_api::StoredExtrusionEntity> converted =
                        slic3r_api::thick_polyline_extrusion(handle, points, line.points_width, params);
                    REQUIRE(converted.has_value());
                    CHECK(snapshot(converted->readonly()) == expected);
                }

    PluginStorage storage;
    c_thick_polyline_extrusion_params params = {};
    params.flow = c_flow{scale_i(0.45), scale_i(0.4), scale_i(0.2), scale_i(0.4), 0, 1.f, 0.};
    params.flow_mult = 1.f;
    CHECK_FALSE(slic3r_api::thick_polyline_extrusion(reinterpret_cast<storage_handle *>(&storage),
                                                     { c_point{0, 0} }, { scale_i(0.45) }, params).has_value());
}

TEST_CASE("Arachne infill settings match the WallToolPaths configuration", "[plugins][infill][arachne]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PrintConfig print;
    print.set_deserialize_strict("nozzle_diameter", "0.6,0.4,0.5");
    const std::vector<Settings> cases = {
        {},
        { { "min_bead_width", "60%" }, { "min_feature_size", "0.1" }, { "wall_transition_length", "0.8" },
          { "wall_transition_filter_deviation", "30%" }, { "wall_transition_angle", "20" },
          { "wall_distribution_count", "3" } }
    };
    for (const Settings &settings : cases) {
        CAPTURE(settings.size());
        const PrintRegionConfig region = region_config(settings);
        const Arachne::WallToolPathsSettings expected = Arachne::wall_tool_paths_settings(region, print);
        const Arachne::WallToolPathsSettings actual = slic3r_api::Infill::ArachneInfill::read_settings(
            slic3r_api::Config(ApiHost::to_config_handle(&region)), slic3r_api::Config(ApiHost::to_config_handle(&print)));
        CHECK(actual.min_nozzle_diameter == expected.min_nozzle_diameter);
        CHECK(actual.min_feature_size == expected.min_feature_size);
        CHECK(actual.min_bead_width == expected.min_bead_width);
        CHECK(actual.wall_transition_filter_deviation == expected.wall_transition_filter_deviation);
        CHECK(actual.wall_transition_length == expected.wall_transition_length);
        CHECK(actual.wall_transition_angle == expected.wall_transition_angle);
        CHECK(actual.wall_distribution_count == expected.wall_distribution_count);
    }
}
