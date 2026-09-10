#include <catch2/catch.hpp>

#include <algorithm>
#include <array>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "plugin_test_helpers.hpp"
#include "perimeter_test_helpers.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/internal/LayerRegionAccess.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_seam_placer.h"
#include "libslic3r/Api/plugin/cpp/OrchestratorViews.hpp"
#include "libslic3r/Api/plugin/cpp/SeamPlacerViews.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Geometry/ArcWelder.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Plugins/Ordering/ExtrusionTreeOrderingGeometry.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"

/*
Fixed-direction seam placer tests
=================================

These tests execute the registered provider through the public service rather
than reaching into its implementation. Real LayerRegion objects provide the
regional setting and real source islands exercise the ordering provider's
region lookup. The loops remain small so every projection and tie-break is
visible in the expected point.
*/

namespace {

using namespace Slic3r;
using namespace Slic3r::Printing;
using namespace Slic3r::Test::PerimeterPluginTests;

const char *const k_fixed_plugin = "seam_placer.fixed";
const char *const k_seam_group = "seam_placer_plugin";
const char *const k_direction_key = "seam_fixed_direction";

/* Own one sliced cube whose first LayerRegion carries a requested direction. */
struct DirectionRegionFixture
{
    explicit DirectionRegionFixture(const char *direction,
                                    const ExPolygon *replacement = nullptr)
    {
        DynamicPrintConfig config = perimeter_config({{k_direction_key, direction}});
        prepare_cube_print(prepared, config);
        LayerRegion &layer_region = prepared.print.object(0).layer(0).region(0);
        if (replacement != nullptr) {
            ExPolygons &raw_slices = ApiInternal::LayerRegionAccess::slices_mutable(layer_region);
            raw_slices = ExPolygons{*replacement};
            ApiInternal::LayerRegionAccess::surfaces_mutable(layer_region).set(
                raw_slices, stPosInternal | stDensSolid);
        }
        region = &layer_region;
    }

    slic3r_api::LayerRegion view() const
    {
        return slic3r_api::LayerRegion(
            reinterpret_cast<const layer_region_handle *>(region));
    }

    PreparedPerimeterPrint prepared;
    LayerRegion *region = nullptr;
};

/* Convert one native extrusion to a borrowed API view. */
slic3r_api::ExtrusionEntity extrusion_view(const ExtrusionEntity &entity)
{
    return slic3r_api::ExtrusionEntity(
        reinterpret_cast<const extrusion_entity_handle *>(&entity));
}

ExtrusionEntity square_loop()
{
    return ExtrusionEntity(false, ArcPolyline(Points{
        Point::new_scale(0.0, 0.0),
        Point::new_scale(10.0, 0.0),
        Point::new_scale(10.0, 10.0),
        Point::new_scale(0.0, 10.0),
        Point::new_scale(0.0, 0.0)}));
}

ExtrusionEntity composed_loop()
{
    ExtrusionEntity::Children children;
    children.emplace_back(new ExtrusionEntity(
        false, ArcPolyline(Points{Point::new_scale(0.0, 0.0),
                                  Point::new_scale(10.0, 0.0)})));
    children.emplace_back(new ExtrusionEntity(
        false, ArcPolyline(Points{Point::new_scale(10.0, 0.0),
                                  Point::new_scale(10.0, 10.0)})));
    children.emplace_back(new ExtrusionEntity(
        false, ArcPolyline(Points{Point::new_scale(10.0, 10.0),
                                  Point::new_scale(0.0, 10.0),
                                  Point::new_scale(0.0, 0.0)})));
    return ExtrusionEntity(std::move(children), false, false, false);
}

ExtrusionEntity arc_loop()
{
    const coord_t radius = scale_i(10.0);
    Geometry::ArcWelder::Path path;
    path.emplace_back(Point(-radius, 0), 0.f,
                      Geometry::ArcWelder::Orientation::Unknown);
    path.emplace_back(Point(radius, 0), static_cast<float>(radius),
                      Geometry::ArcWelder::Orientation::CCW);
    path.emplace_back(Point(-radius, 0), static_cast<float>(radius),
                      Geometry::ArcWelder::Orientation::CCW);
    return ExtrusionEntity(false, ArcPolyline(path));
}

/* Execute the built-in provider and take ownership of its published session. */
slic3r_api::SeamPlacer create_fixed_session(Print &print)
{
    Orchestrator &orchestrator = Orchestrator::instance();
    slic3r_api::OrchestratorView view(
        reinterpret_cast<orchestrator_handle *>(&orchestrator));
    const slic3r_api::PluginView provider = view.find_plugin(k_fixed_plugin);
    REQUIRE(provider.valid());

    PrintingPlan &plan = print.mutable_printing_plan();
    run_ctx_seam_placer context = {};
    context.plan = reinterpret_cast<const printing_plan_handle *>(&plan);
    REQUIRE(view.execute(
                provider,
                reinterpret_cast<const print_handle *>(&print),
                context) == RAW_PLUGIN_EXECUTION_SUCCESS);
    REQUIRE(context.instance.session != nullptr);
    return slic3r_api::SeamPlacer(context.instance);
}

bool same_point(const c_point lhs, const c_point rhs)
{
    return lhs.x == rhs.x && lhs.y == rhs.y;
}

c_point mm_point(const double x, const double y)
{
    return c_point{scale_i(x), scale_i(y)};
}

} // namespace

TEST_CASE("Fixed-direction seam placer registers its regional contract",
          "[plugins][seam-placer][fixed][config]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    Plugin *plugin = orchestrator.get_plugin(k_fixed_plugin);
    REQUIRE(plugin != nullptr);
    CHECK(plugin->get_name() == "Fixed direction");
    CHECK(plugin->get_step() == SEAM_PLACER);
    CHECK(plugin->get_priority() == 1000);
    CHECK(plugin->get_exclusive_group() == k_seam_group);

    const std::vector<Plugin::UsedConfigKey> used = plugin->get_used_config_keys();
    REQUIRE(used.size() == 1);
    CHECK(used[0].key == k_direction_key);
    CHECK(used[0].type == RAW_CO_ENUM);
    CHECK(used[0].container_type == RAW_CONTAINER_TYPE_REGION);

    raw_config_option_def definition = {};
    REQUIRE(orchestrator_get_option_def(
                reinterpret_cast<orchestrator_handle *>(&orchestrator),
                k_direction_key, &definition) == 1);
    CHECK(definition.type == RAW_CO_ENUM);
    CHECK(definition.container_type == RAW_CONTAINER_TYPE_REGION);
    CHECK(definition.invalidates_step == STEP_ORDERING);
    REQUIRE(definition.default_serialized_value != nullptr);
    CHECK(std::string(definition.default_serialized_value) == "rear");
    const std::array<const char *, 8> expected = {{
        "rear", "front", "right", "left",
        "rear_right", "rear_left", "front_right", "front_left"}};
    REQUIRE(definition.enum_def.value_label_pairs.count == expected.size());
    for (size_t idx = 0; idx < expected.size(); ++idx)
        CHECK(std::string(definition.enum_def.value_label_pairs.items[idx].value) == expected[idx]);

    std::ifstream stream(
        std::string(TEST_DATA_DIR) + "/../../src/plugins/default_activated.ini.in");
    REQUIRE(stream.good());
    std::ostringstream contents;
    contents << stream.rdbuf();
    CHECK(contents.str().find("seam_placer.fixed = 1") != std::string::npos);
}

TEST_CASE("Legacy rear seam values remain readable but are hidden from choices",
          "[plugins][seam-placer][fixed][legacy]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    REQUIRE_NOTHROW(config.set_deserialize_strict("seam_position", "rear"));
    CHECK(config.opt_enum<SeamPosition>("seam_position") == spRear);

    const ConfigOptionDef *definition = PrintConfigDef::instance().get("seam_position");
    REQUIRE(definition != nullptr);
    REQUIRE(static_cast<bool>(definition->enum_def));
    CHECK(std::find(definition->enum_def->values().begin(),
                    definition->enum_def->values().end(),
                    "rear") == definition->enum_def->values().end());
}

TEST_CASE("Fixed-direction seam placement covers every compass direction",
          "[plugins][seam-placer][fixed][geometry]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print session_print;
    slic3r_api::SeamPlacer placer = create_fixed_session(session_print);
    ExtrusionEntity loop = square_loop();

    const std::array<std::pair<const char *, c_point>, 8> cases = {{
        {"rear", mm_point(10.0, 10.0)},
        {"front", mm_point(0.0, 0.0)},
        {"right", mm_point(10.0, 0.0)},
        {"left", mm_point(0.0, 10.0)},
        {"rear_right", mm_point(10.0, 10.0)},
        {"rear_left", mm_point(0.0, 10.0)},
        {"front_right", mm_point(10.0, 0.0)},
        {"front_left", mm_point(0.0, 0.0)}}};

    for (const std::pair<const char *, c_point> &entry : cases) {
        CAPTURE(entry.first);
        DirectionRegionFixture region(entry.first);
        const c_point first = placer.place_seam(
            extrusion_view(loop), region.view(), mm_point(-500.0, -500.0));
        const c_point second = placer.place_seam(
            extrusion_view(loop), region.view(), mm_point(500.0, 500.0));
        CHECK(same_point(first, entry.second));
        CHECK(same_point(second, entry.second));
    }
}

TEST_CASE("Fixed-direction uses stored vertices in composed and arc loops",
          "[plugins][seam-placer][fixed][geometry]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print session_print;
    slic3r_api::SeamPlacer placer = create_fixed_session(session_print);
    DirectionRegionFixture rear("rear");

    ExtrusionEntity composed = composed_loop();
    CHECK(same_point(
        placer.place_seam(extrusion_view(composed), rear.view(), {}),
        mm_point(10.0, 10.0)));

    ExtrusionEntity arcs = arc_loop();
    // The analytic arc reaches +Y, but that generated extremum is not a stored
    // vertex. Rear therefore ties the two endpoints and then chooses +X.
    CHECK(same_point(
        placer.place_seam(extrusion_view(arcs), rear.view(), {}),
        mm_point(10.0, 0.0)));
}

TEST_CASE("One Fixed-direction session reads each LayerRegion independently",
          "[plugins][seam-placer][fixed][region]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print session_print;
    slic3r_api::SeamPlacer placer = create_fixed_session(session_print);
    DirectionRegionFixture rear("rear");
    DirectionRegionFixture left("left");
    ExtrusionEntity loop = square_loop();

    CHECK(same_point(
        placer.place_seam(extrusion_view(loop), rear.view(), {}),
        mm_point(10.0, 10.0)));
    CHECK(same_point(
        placer.place_seam(extrusion_view(loop), left.view(), {}),
        mm_point(0.0, 10.0)));
}

TEST_CASE("Ordering resolves the first intersecting source LayerRegion",
          "[plugins][seam-placer][fixed][region-selection]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ExPolygon left_area = rectangle_expolygon(-20.0, -20.0, 1.0, 20.0);
    const ExPolygon right_area = rectangle_expolygon(-1.0, -20.0, 20.0, 20.0);
    DirectionRegionFixture left("left", &left_area);
    DirectionRegionFixture right("right", &right_area);

    LayerRegionSetCPtrs regions = {left.region, right.region};
    LayerRegionIsland source(regions, 0);
    ExtrusionEntity crossing(false, ArcPolyline(Points{
        Point::new_scale(-5.0, 0.0), Point::new_scale(5.0, 0.0)}));
    source.mutable_extrusion(LayerRegionIsland::PERIMETERS).append(crossing);

    PrintingExtrusion native;
    native.region_island = &source;
    native.sregion_island_role = LayerRegionIsland::PERIMETERS;
    native.root.reset(crossing.clone());
    slic3r_api::PrintingExtrusion extrusion(
        reinterpret_cast<printing_extrusion_handle *>(&native));

    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    const slic3r_api::LayerRegion selected =
        slic3r_api::Ordering::TreeOrderingGeometry::resolve_source_layer_region(
            storage, extrusion);
    REQUIRE_FALSE(source.regions().empty());
    CHECK(selected.handle() == reinterpret_cast<const layer_region_handle *>(
                                  *source.regions().begin()));
    plugin_storage.clear();

    LayerRegionIsland unique({right.region}, 0);
    unique.mutable_extrusion(LayerRegionIsland::PERIMETERS).append(crossing);
    native.region_island = &unique;
    const slic3r_api::LayerRegion unique_selected =
        slic3r_api::Ordering::TreeOrderingGeometry::resolve_source_layer_region(
            storage, extrusion);
    CHECK(unique_selected.handle() == reinterpret_cast<const layer_region_handle *>(
                                         right.region));
    plugin_storage.clear();

    // Region lookup follows the analytic source path rather than treating an
    // arc as the straight chord between its stored endpoints.
    const ExPolygon arc_area = rectangle_expolygon(-2.0, 8.0, 2.0, 12.0);
    DirectionRegionFixture arc_region("rear", &arc_area);
    LayerRegionIsland arc_source({arc_region.region}, 0);
    ExtrusionEntity source_arc = arc_loop();
    arc_source.mutable_extrusion(LayerRegionIsland::PERIMETERS).append(source_arc);
    native.region_island = &arc_source;
    native.root.reset(source_arc.clone());
    const slic3r_api::LayerRegion arc_selected =
        slic3r_api::Ordering::TreeOrderingGeometry::resolve_source_layer_region(
            storage, extrusion);
    CHECK(arc_selected.handle() == reinterpret_cast<const layer_region_handle *>(
                                      arc_region.region));
    plugin_storage.clear();

    const ExPolygon distant_area = rectangle_expolygon(100.0, 100.0, 110.0, 110.0);
    DirectionRegionFixture distant("rear", &distant_area);
    LayerRegionIsland missing({distant.region}, 0);
    missing.mutable_extrusion(LayerRegionIsland::PERIMETERS).append(crossing);
    native.region_island = &missing;
    CHECK_THROWS_WITH(
        slic3r_api::Ordering::TreeOrderingGeometry::resolve_source_layer_region(
            storage, extrusion),
        "A PrintingExtrusion does not intersect any LayerRegion from its source island.");
}
