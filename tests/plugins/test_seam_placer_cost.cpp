#include <catch2/catch.hpp>

#include <algorithm>
#include <functional>
#include <fstream>
#include <future>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/internal/ArcPolylineAccess.hpp"
#include "libslic3r/Api/internal/LayerAccess.hpp"
#include "libslic3r/Api/internal/LayerRegionAccess.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Api/plugin/cpp/SeamPlacerViews.hpp"
#include "libslic3r/Api/plugin/cpp/properties/ExtrusionProperties.hpp"
#include "libslic3r/Geometry/ArcWelder.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Plugins/SeamPlacer/CostSeamPlacer.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/PrintObjectRegion.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"
#include "libslic3r/TriangleSelector.hpp"

/*
Cost seam provider tests
========================
These focused checks exercise the built-in metadata and a published immutable
session over a real generated PrintingPlan. The functional case asks the
service adapter to validate every returned point, so line/arc projection cannot
silently return a point outside the requested loop.
*/

namespace {

using namespace Slic3r;
using namespace Slic3r::Printing;
using namespace Slic3r::Test::PerimeterPluginTests;

const char *const k_cost_plugin = "seam_placer.cost";

bool find_loop(const slic3r_api::ExtrusionEntity &entity,
               slic3r_api::ExtrusionEntity &loop)
{
    if (entity.is_loop() && entity.point_count() != 0) {
        loop = entity;
        return true;
    }
    for (uint32_t index = 0; index < entity.child_count(); ++index)
        if (find_loop(entity.child(index), loop)) return true;
    return false;
}

slic3r_api::SeamPlacer create_cost_session(Print &print)
{
    PrintingPlan &native_plan = print.mutable_printing_plan();
    const slic3r_api::PrintingPlan plan(reinterpret_cast<printing_plan_handle *>(&native_plan));
    const slic3r_api::Print print_view(reinterpret_cast<const print_handle *>(&print));
    std::vector<slic3r_api::PrintRegion> regions;
    for (uint32_t object_index = 0; object_index < print_view.object_count(); ++object_index) {
        const slic3r_api::Object object = print_view.object(object_index);
        for (uint32_t region_index = 0; region_index < object.print_region_count(); ++region_index)
            regions.push_back(object.print_region(region_index));
    }
    PluginStorage storage;
    raw_seam_placer_instance instance = slic3r_api::make_seam_placer_instance(
        slic3r_api::SeamPlacement::CostPlugin::make_cost_seam_session(
            reinterpret_cast<storage_handle *>(&storage)), plan, regions);
    return slic3r_api::SeamPlacer(instance);
}

slic3r_api::SeamPlacer create_cost_session_for(
    Print &print, const std::vector<slic3r_api::PrintRegion> &regions)
{
    PrintingPlan &native_plan = print.mutable_printing_plan();
    PluginStorage storage;
    raw_seam_placer_instance instance = slic3r_api::make_seam_placer_instance(
        slic3r_api::SeamPlacement::CostPlugin::make_cost_seam_session(
            reinterpret_cast<storage_handle *>(&storage)),
        slic3r_api::PrintingPlan(reinterpret_cast<printing_plan_handle *>(&native_plan)),
        regions);
    return slic3r_api::SeamPlacer(instance);
}

ExtrusionEntity rectangle_loop(double left, double bottom, double right, double top,
                               bool reverse = false)
{
    Points points{Point::new_scale(left, bottom), Point::new_scale(right, bottom),
                  Point::new_scale(right, top), Point::new_scale(left, top),
                  Point::new_scale(left, bottom)};
    if (reverse) std::reverse(points.begin(), points.end());
    return ExtrusionEntity(false, ArcPolyline(points));
}

ExtrusionEntity arc_loop()
{
    const coord_t radius = scale_i(10.0);
    Geometry::ArcWelder::Path path;
    path.emplace_back(Point(-radius, 0), 0.f, Geometry::ArcWelder::Orientation::Unknown);
    path.emplace_back(Point(0, -radius), float(radius), Geometry::ArcWelder::Orientation::CCW);
    path.emplace_back(Point(radius, 0), float(radius), Geometry::ArcWelder::Orientation::CCW);
    path.emplace_back(Point(0, radius), float(radius), Geometry::ArcWelder::Orientation::CCW);
    path.emplace_back(Point(-radius, 0), float(radius), Geometry::ArcWelder::Orientation::CCW);
    ArcPolyline polyline(path);
    ApiInternal::ArcPolylineAccess::refresh_after_bulk_replace(polyline);
    return ExtrusionEntity(false, polyline);
}

struct CostFixture
{
    explicit CostFixture(std::initializer_list<std::pair<std::string, std::string>> settings =
                             {{"seam_visibility", "0"}, {"seam_angle_cost", "0"},
                              {"seam_travel_cost", "0"}},
                         size_t layer_index = 0)
    {
        prepare_cube_print(prepared, perimeter_config(settings));
        object = &prepared.print.object(0);
        REQUIRE(layer_index < object->layer_count());
        region = &object->layer(layer_index).region(0);
        source.reset(new LayerRegionIsland({region}, 0));
        source_ptr = source.get();
        prepared.print.mutable_printing_plan() = PrintingPlan();
    }

    slic3r_api::LayerRegion region_view() const
    {
        return slic3r_api::LayerRegion(reinterpret_cast<const layer_region_handle *>(region));
    }

    ModelVolume *box(double x, double y, double sx, double sy, ModelVolumeType type,
                     EnforcerBlockerType painting = EnforcerBlockerType::NONE,
                     double z = 0, double height = 30)
    {
        TriangleMesh mesh = make_cube(sx, sy, height);
        mesh.translate(Vec3f(float(x), float(y), float(z)));
        mesh.transform(object->trafo_centered().inverse());
        ModelVolume *volume = const_cast<ModelObject *>(object->model_object())->add_volume(
            std::move(mesh), type, false);
        if (painting != EnforcerBlockerType::NONE) {
            TriangleSelector selector(volume->mesh());
            for (size_t index = 0; index < volume->mesh().its.indices.size(); ++index)
                selector.set_facet(int(index), painting);
            volume->facets_annotation_mutable("builtin:seam").set(selector);
        }
        return volume;
    }

    slic3r_api::PrintingExtrusion add(const ExtrusionEntity &loop, raw_extrusion_role role,
                                      uint16_t instance = 0,
                                      LayerRegionIsland *island = nullptr,
                                      bool perimeter = true)
    {
        PrintingPlan &native_plan = prepared.print.mutable_printing_plan();
        slic3r_api::PrintingPlan plan(reinterpret_cast<printing_plan_handle *>(&native_plan));
        if (plan.group_count() == 0)
            plan.append_group().append_layer_group(region_view().layer().print_z()).append_tool_group(0);
        const uint32_t instance_id = plan.find_or_add_object_instance(
            region_view().layer().object(), instance);
        LayerRegionIsland *selected = island == nullptr ? source_ptr : island;
        slic3r_api::PrintingExtrusion extrusion =
            plan.group(0).layer_group(0).tool_group(0).append_extrusion_clone(
                plan,
                slic3r_api::LayerRegionIsland(
                    reinterpret_cast<const layer_region_island_handle *>(selected)),
                RAW_EXTRUSION_ROLE_PERIMETER,
                slic3r_api::ExtrusionEntity(
                    reinterpret_cast<const extrusion_entity_handle *>(&loop)),
                instance_id);
        slic3r_api::MutableExtrusionEntity root = extrusion.mutable_root();
        c_extrusion_property_attributes *attributes =
            static_cast<c_extrusion_property_attributes *>(root.get_or_add_property_data_mutable(
                reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()),
                EXTRUSION_PROPERTY_TYPE_ATTRIBUTES));
        REQUIRE(attributes != nullptr);
        attributes->role = role;
        attributes->width = 0.4f;
        if (perimeter) {
            c_extrusion_property_perimeter *property =
                static_cast<c_extrusion_property_perimeter *>(root.get_or_add_property_data_mutable(
                    reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance()),
                    EXTRUSION_PROPERTY_TYPE_PERIMETER));
            REQUIRE(property != nullptr);
            property->perimeter_flags = C_EXTRUSION_PERIMETER_FLAG_LOOP;
        }
        const c_point shift = region_view().layer().object().instance_shift(instance);
        std::function<void(slic3r_api::MutableExtrusionEntity)> translate;
        translate = [&](slic3r_api::MutableExtrusionEntity entity) {
            if (entity.point_count() != 0) REQUIRE(entity.translate(shift));
            for (uint32_t index = 0; index < entity.child_count(); ++index)
                translate(entity.child_mutable(index));
        };
        translate(root);
        return extrusion;
    }

    c_point local(c_point point, uint16_t instance = 0) const
    {
        const c_point shift = region_view().layer().object().instance_shift(instance);
        return {point.x - shift.x, point.y - shift.y};
    }

    slic3r_api::PrintingExtrusion extrusion(uint32_t index)
    {
        slic3r_api::PrintingPlan plan(reinterpret_cast<printing_plan_handle *>(
            &prepared.print.mutable_printing_plan()));
        return plan.group(0).layer_group(0).tool_group(0).extrusion(index);
    }

    PreparedPerimeterPrint prepared;
    PrintObject *object = nullptr;
    LayerRegion *region = nullptr;
    std::unique_ptr<LayerRegionIsland> source;
    LayerRegionIsland *source_ptr = nullptr;
};

// Build the native LayerIsland relationship used by boundary and embedding
// calculations; the compact perimeter fixture intentionally leaves it empty.
LayerRegionIsland *attach_native_source(CostFixture &fixture)
{
    Layer &layer = *fixture.region->layer();
    if (layer.islands().empty()) {
        ExPolygons islands;
        for (const ExPolygon &slice : fixture.region->get_raw_slices())
            islands.push_back(slice);
        ApiInternal::LayerAccess::set_islands(layer, std::move(islands));
    }
    REQUIRE(!layer.islands().empty());
    layer.island(0).mutable_regions_islands().emplace_back(
        new LayerRegionIsland({fixture.region}, 0));
    return &layer.island(0).regions_island(
        layer.island(0).regions_islands().size() - 1);
}

LayerRegion *add_neighbor_region(CostFixture &fixture, const ExPolygon &area)
{
    Layer &layer = *fixture.region->layer();
    PrintRegionConfig config = fixture.region->region().config();
    config.set_deserialize_strict(
        "perimeters", std::to_string(fixture.region->region().config().opt_int("perimeters") + 1));
    const PrintRegion *print_region = &fixture.prepared.print.register_print_region(config);
    PrintObjectRegions &associations = *const_cast<PrintObjectRegions *>(
        fixture.object->shared_regions());
    associations.all_regions.push_back(print_region);
    ApiInternal::LayerAccess::add_region(layer, *print_region);
    LayerRegion &region = layer.region(layer.region_count() - 1);
    ExPolygons &slices = ApiInternal::LayerRegionAccess::slices_mutable(region);
    slices = {area};
    ApiInternal::LayerRegionAccess::surfaces_mutable(region).set(
        slices, stPosInternal | stDensSolid);
    layer.island(0).mutable_regions_islands().emplace_back(
        new LayerRegionIsland({&region}, 0));
    return &region;
}

} // namespace

TEST_CASE("Cost seam metadata is the default regional provider",
          "[plugins][seam-placer][cost]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    const Plugin *cost = orchestrator.get_plugin(k_cost_plugin);
    const Plugin *fixed = orchestrator.get_plugin("seam_placer.fixed");
    const Plugin *random = orchestrator.get_plugin("seam_placer.random");
    REQUIRE(cost != nullptr);
    REQUIRE(fixed != nullptr);
    REQUIRE(random != nullptr);
    CHECK(cost->get_name() == "Cost-based");
    CHECK(cost->get_priority() == 900);
    CHECK(fixed->get_priority() == 1000);
    CHECK(random->get_priority() == 1100);
    CHECK(cost->get_dependencies().empty());
    const std::vector<Plugin::UsedConfigKey> used = cost->get_used_config_keys();
    REQUIRE(used.size() == 3);
    CHECK(used[0].key == "seam_angle_cost");
    CHECK(used[0].type == RAW_CO_PERCENT);
    CHECK(used[1].key == "seam_travel_cost");
    CHECK(used[1].type == RAW_CO_PERCENT);
    CHECK(used[2].key == "seam_visibility");
    CHECK(used[2].type == RAW_CO_BOOL);
    CHECK(used[2].container_type == RAW_CONTAINER_TYPE_REGION);
    CHECK(used[2].option_preset_type == RAW_PRESET_TYPE_FFF_PRINT);
    const std::vector<Plugin::DefinedConfigKey> defined = cost->get_defined_config_keys();
    REQUIRE(defined.size() == 3);
    CHECK(defined[0].key == "seam_angle_cost");
    CHECK(defined[0].type == RAW_CO_PERCENT);
    CHECK(defined[1].key == "seam_travel_cost");
    CHECK(defined[1].type == RAW_CO_PERCENT);
    CHECK(defined[2].key == "seam_visibility");
    CHECK(defined[2].type == RAW_CO_BOOL);
    for (size_t index = 0; index < 3; ++index) {
        const Plugin::UsedConfigKey &key = used[index];
        CHECK(key.container_type == RAW_CONTAINER_TYPE_REGION);
        CHECK(key.option_preset_type == RAW_PRESET_TYPE_FFF_PRINT);
    }
    // The shared runtime disables non-Fixed providers after registration. A
    // default-enabled Cost provider must still activate without dependency
    // diagnostics, then return to the shared inactive test state.
    CHECK_FALSE(orchestrator.is_plugin_active(k_cost_plugin));
    CHECK(orchestrator.set_plugin_active(k_cost_plugin, true));
    CHECK(orchestrator.is_plugin_active(k_cost_plugin));
    const std::vector<std::string> dependency_errors =
        orchestrator.active_plugin_dependency_errors();
    CHECK(std::none_of(dependency_errors.begin(), dependency_errors.end(),
                       [](const std::string &error) {
                           return error.find(k_cost_plugin) != std::string::npos;
                       }));
    CHECK(orchestrator.set_plugin_active(k_cost_plugin, false));
    CHECK_FALSE(orchestrator.is_plugin_active(k_cost_plugin));

    PrintRegionConfig region;
    const std::string selector = Steps::plugin_exclusive_group_option_key(
        orchestrator, SEAM_PLACER, "seam_placer_plugin");
    REQUIRE(region.option(selector) != nullptr);
    CHECK(region.option(selector)->serialize() == k_cost_plugin);

    std::ifstream stream(
        std::string(TEST_DATA_DIR) + "/../../src/plugins/default_activated.ini.in");
    REQUIRE(stream.good());
    std::ostringstream contents;
    contents << stream.rdbuf();
    CHECK(contents.str().find("seam_placer.cost = 1") != std::string::npos);
}

TEST_CASE("Cost seam projects candidates and supports parallel placement",
          "[plugins][seam-placer][cost]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({
        {"seam_visibility", "0"}, {"seam_angle_cost", "0"}, {"seam_travel_cost", "100%"}}));
    PrintObject &object = prepared.print.object(0);
    LayerRegion &native_region = object.layer(0).region(0);
    LayerRegionSetCPtrs regions{&native_region};
    std::unique_ptr<LayerRegionIsland> source(new LayerRegionIsland(regions, 0));
    const Point shift = object.instances().front().shift;
    std::unique_ptr<ExtrusionEntity> root(new ExtrusionEntity(false, ArcPolyline(Points{
        shift + Point::new_scale(-5, -5), shift + Point::new_scale(5, -5),
        shift + Point::new_scale(5, 5), shift + Point::new_scale(-5, 5),
        shift + Point::new_scale(-5, -5)})));
    slic3r_api::MutableExtrusionEntity mutable_root(
        reinterpret_cast<extrusion_entity_handle *>(root.get()));
    orchestrator_handle *orchestrator_handle_value = reinterpret_cast<orchestrator_handle *>(
        &Orchestrator::instance());
    c_extrusion_property_attributes *attributes = static_cast<c_extrusion_property_attributes *>(
        mutable_root.get_or_add_property_data_mutable(
            orchestrator_handle_value, EXTRUSION_PROPERTY_TYPE_ATTRIBUTES));
    REQUIRE(attributes != nullptr);
    attributes->role = RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER;
    attributes->width = 0.4f;
    c_extrusion_property_perimeter *perimeter = static_cast<c_extrusion_property_perimeter *>(
        mutable_root.get_or_add_property_data_mutable(
            orchestrator_handle_value, EXTRUSION_PROPERTY_TYPE_PERIMETER));
    REQUIRE(perimeter != nullptr);
    perimeter->perimeter_flags = C_EXTRUSION_PERIMETER_FLAG_LOOP;

    PrintingPlan &native_plan = prepared.print.mutable_printing_plan();
    native_plan = PrintingPlan();
    const PrintingObjectInstanceId catalog_id = native_plan.find_or_add_object_instance(object, 0);
    native_plan.groups.emplace_back();
    native_plan.groups.back().layers.emplace_back();
    native_plan.groups.back().layers.back().layers.push_back(&object.layer(0));
    native_plan.groups.back().layers.back().tool_groups.emplace_back();
    PrintingToolGroup &tool = native_plan.groups.back().layers.back().tool_groups.back();
    tool.region_islands.push_back(source.get());
    tool.extrusions.emplace_back();
    tool.extrusions.back().region_island = source.get();
    tool.extrusions.back().sregion_island_role = LayerRegionIsland::PERIMETERS;
    tool.extrusions.back().printing_catalog_id = catalog_id;
    tool.extrusions.back().root = std::move(root);

    slic3r_api::SeamPlacer placer = create_cost_session(prepared.print);
    REQUIRE(placer.valid());

    const slic3r_api::PrintingPlan plan(reinterpret_cast<printing_plan_handle *>(
        &prepared.print.mutable_printing_plan()));
    const slic3r_api::PrintingExtrusion extrusion = plan.group(0).layer_group(0).tool_group(0).extrusion(0);
    slic3r_api::ExtrusionEntity loop;
    REQUIRE(find_loop(extrusion.root(), loop));
    const slic3r_api::LayerRegion region = extrusion.region_island().region(0);
    CHECK(placer.get_priority(loop, region) == 50.f);

    const c_point first = placer.place_seam(loop, region, loop.front());
    std::vector<std::future<c_point>> calls;
    for (size_t index = 0; index < 8; ++index)
        calls.push_back(std::async(std::launch::async, [&placer, loop, region, index]() {
            return placer.place_seam(loop, region, index % 2 == 0 ? loop.front() : loop.back());
        }));
    for (std::future<c_point> &call : calls) CHECK_NOTHROW(call.get());
    CHECK_NOTHROW(placer.place_seam(loop, region, first));

    const ExtrusionEntity foreign(false, ArcPolyline(Points{
        Point::new_scale(0, 0), Point::new_scale(1, 0),
        Point::new_scale(1, 1), Point::new_scale(0, 0)}));
    const slic3r_api::ExtrusionEntity foreign_view(
        reinterpret_cast<const extrusion_entity_handle *>(&foreign));
    CHECK_THROWS_WITH(placer.place_seam(foreign_view, region, {0, 0}),
                      "Cost seam loop was not present in the initialized plan.");
}

TEST_CASE("Cost seam takes candidates only from effective external perimeter loops",
          "[plugins][seam-placer][cost][geometry]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    CostFixture fixture;
    const ExtrusionEntity internal(false, ArcPolyline(Points{
        Point::new_scale(2, 2), Point::new_scale(-2, 2), Point::new_scale(-2, -2),
        Point::new_scale(2, -2), Point::new_scale(2, 2)}));
    const ExtrusionEntity external = rectangle_loop(-5, -5, 5, 5);
    fixture.add(internal, RAW_EXTRUSION_ROLE_PERIMETER);
    fixture.add(external, RAW_EXTRUSION_ROLE_OVERHANG_EXTERNAL_PERIMETER);

    const slic3r_api::SeamPlacer placer = create_cost_session(fixture.prepared.print);
    const slic3r_api::PrintingExtrusion internal_plan = fixture.extrusion(0);
    const c_point projected = fixture.local(placer.place_seam(
        internal_plan.root(), fixture.region_view(), internal_plan.root().front()));
    CHECK(projected.x == scale_i(-2));
    CHECK(projected.y == scale_i(-2));
}

TEST_CASE("Cost seam binds composed loops and projects analytically onto arcs",
          "[plugins][seam-placer][cost][geometry]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    SECTION("composed line loop")
    {
        CostFixture fixture;
        ExtrusionEntity::Children children;
        children.emplace_back(new ExtrusionEntity(false, ArcPolyline(Points{
            Point::new_scale(0, 0), Point::new_scale(10, 0)})));
        children.emplace_back(new ExtrusionEntity(false, ArcPolyline(Points{
            Point::new_scale(10, 0), Point::new_scale(10, 10)})));
        children.emplace_back(new ExtrusionEntity(false, ArcPolyline(Points{
            Point::new_scale(10, 10), Point::new_scale(0, 10), Point::new_scale(0, 0)})));
        const ExtrusionEntity composed(std::move(children), false, false, false);
        const slic3r_api::PrintingExtrusion extrusion = fixture.add(
            composed, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
        const slic3r_api::SeamPlacer placer = create_cost_session(fixture.prepared.print);
        CHECK_NOTHROW(placer.place_seam(extrusion.root(), fixture.region_view(), {}));
    }

    SECTION("arc loop")
    {
        CostFixture fixture({{"seam_visibility", "0"}, {"seam_angle_cost", "0"},
                             {"seam_travel_cost", "100%"}});
        fixture.box(5.9, 7.8, 0.2, 0.4, ModelVolumeType::SEAM_POSITION_INSIDE);
        const slic3r_api::PrintingExtrusion extrusion = fixture.add(
            arc_loop(), RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
        const slic3r_api::SeamPlacer placer = create_cost_session(fixture.prepared.print);
        const c_point point = fixture.local(placer.place_seam(
            extrusion.root(), fixture.region_view(), {scale_i(6), scale_i(8)}));
        CHECK(std::hypot(unscaled(point.x), unscaled(point.y)) == Approx(10).margin(0.0001));
        CHECK((point.x != extrusion.root().front().x || point.y != extrusion.root().front().y));
    }
}

TEST_CASE("Cost seam applies modifier precedence and all-blocked fallback",
          "[plugins][seam-placer][cost][constraints]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const bool arc = GENERATE(false, true);
    CostFixture fixture;
    fixture.box(arc ? 5.9 : 4.0, arc ? 7.8 : -1.0, arc ? 0.2 : 2.0,
                arc ? 0.4 : 2.0, ModelVolumeType::SEAM_POSITION_INSIDE);
    const slic3r_api::PrintingExtrusion extrusion = fixture.add(
        arc ? arc_loop() : rectangle_loop(0, 0, 10, 10),
        RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    slic3r_api::SeamPlacer placer = create_cost_session(fixture.prepared.print);
    c_point point = fixture.local(placer.place_seam(extrusion.root(), fixture.region_view(), {}));
    if (arc) {
        CHECK(std::hypot(unscaled(point.x), unscaled(point.y)) == Approx(10).margin(0.0001));
        CHECK(unscaled(point.x) >= 5.89);
    } else {
        CHECK(point.y == 0);
        CHECK(unscaled(point.x) >= 3.99);
        CHECK(unscaled(point.x) <= 6.01);
    }

    CostFixture blocked;
    blocked.box(-20, -20, 40, 40, ModelVolumeType::MODEL_PART,
                EnforcerBlockerType::BLOCKER);
    const slic3r_api::PrintingExtrusion blocked_loop = blocked.add(
        rectangle_loop(0, 0, 10, 10), RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    placer = create_cost_session(blocked.prepared.print);
    CHECK_NOTHROW(placer.place_seam(blocked_loop.root(), blocked.region_view(), {}));

    CostFixture overlap;
    overlap.box(4, -1, 2, 2, ModelVolumeType::SEAM_POSITION_INSIDE);
    overlap.box(3, -2, 4, 4, ModelVolumeType::MODEL_PART,
                EnforcerBlockerType::BLOCKER);
    const slic3r_api::PrintingExtrusion overlap_loop = overlap.add(
        rectangle_loop(0, 0, 10, 10), RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    placer = create_cost_session(overlap.prepared.print);
    point = overlap.local(placer.place_seam(overlap_loop.root(), overlap.region_view(), {}));
    CHECK_FALSE((point.y == 0 && unscaled(point.x) > 3 && unscaled(point.x) < 7));
}

TEST_CASE("Cost seam applies legacy angle and dynamic travel weights",
          "[plugins][seam-placer][cost][weights]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ExtrusionEntity concave(false, ArcPolyline(Points{
        Point::new_scale(0, 0), Point::new_scale(10, 0), Point::new_scale(10, 10),
        Point::new_scale(5, 5), Point::new_scale(0, 10), Point::new_scale(0, 0)}));
    CostFixture angle({{"seam_visibility", "0"}, {"seam_angle_cost", "1000%"},
                       {"seam_travel_cost", "0"}});
    const slic3r_api::PrintingExtrusion angle_loop = angle.add(
        concave, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    const slic3r_api::SeamPlacer angle_placer = create_cost_session(angle.prepared.print);
    const c_point hidden_corner = angle.local(angle_placer.place_seam(
        angle_loop.root(), angle.region_view(), {}));
    CHECK(hidden_corner.x == scale_i(5));
    CHECK(hidden_corner.y == scale_i(5));

    CostFixture travel({{"seam_visibility", "0"}, {"seam_angle_cost", "0"},
                        {"seam_travel_cost", "1000%"}});
    const slic3r_api::PrintingExtrusion travel_loop = travel.add(
        rectangle_loop(0, 0, 10, 10), RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    const slic3r_api::SeamPlacer travel_placer = create_cost_session(travel.prepared.print);
    const c_point requested = {scale_i(10) + travel_loop.root().front().x,
                               scale_i(10) + travel_loop.root().front().y};
    const c_point nearest = travel.local(travel_placer.place_seam(
        travel_loop.root(), travel.region_view(), requested));
    CHECK(nearest.x == scale_i(10));
    CHECK(nearest.y == scale_i(10));
}

TEST_CASE("Cost seam boundary uses half the effective width and follows filter precedence",
          "[plugins][seam-placer][cost][boundary]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const double gap = GENERATE(0.19, 0.21);
    CostFixture fixture;
    fixture.source_ptr = attach_native_source(fixture);
    LayerRegion *neighbor = add_neighbor_region(
        fixture, rectangle_expolygon(5 + gap, -1, 7, 1));
    REQUIRE(fixture.object->layer(0).island(0).regions_islands().size() == 2);
    REQUIRE(neighbor->slices().surfaces.size() == 1);
    const slic3r_api::PrintingExtrusion extrusion = fixture.add(
        rectangle_loop(-5, -5, 5, 5), RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    const slic3r_api::SeamPlacer placer = create_cost_session(fixture.prepared.print);
    const c_point point = fixture.local(placer.place_seam(
        extrusion.root(), fixture.region_view(), extrusion.root().front()));
    if (gap < 0.2) {
        CHECK(point.x == scale_i(5));
        CHECK(std::abs(unscaled(point.y)) == Approx(1.2).margin(0.002));
    } else {
        CHECK(point.x == scale_i(-5));
        CHECK(point.y == scale_i(-5));
    }

    CostFixture precedence;
    precedence.source_ptr = attach_native_source(precedence);
    add_neighbor_region(precedence, rectangle_expolygon(5.19, -1, 7, 1));
    precedence.box(-3, -6, 2, 2, ModelVolumeType::SEAM_POSITION_INSIDE);
    const slic3r_api::PrintingExtrusion precedence_loop = precedence.add(
        rectangle_loop(-5, -5, 5, 5), RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    const slic3r_api::SeamPlacer precedence_placer = create_cost_session(
        precedence.prepared.print);
    const c_point enforced = precedence.local(precedence_placer.place_seam(
        precedence_loop.root(), precedence.region_view(), {}));
    CHECK(enforced.y == scale_i(-5));
    CHECK(unscaled(enforced.x) >= -3.01);
    CHECK(unscaled(enforced.x) <= -0.99);
}

TEST_CASE("Cost seam finds boundary transitions in nested line and arc segments",
          "[plugins][seam-placer][cost][boundary]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    CostFixture fixture;
    fixture.source_ptr = attach_native_source(fixture);
    add_neighbor_region(fixture, rectangle_expolygon(5.8, 7.8, 6.2, 8.2));
    ExtrusionEntity::Children inner;
    inner.emplace_back(new ExtrusionEntity(arc_loop()));
    ExtrusionEntity::Children outer;
    outer.emplace_back(new ExtrusionEntity(std::move(inner), false, false, false));
    const ExtrusionEntity nested(std::move(outer), false, false, false);
    const slic3r_api::PrintingExtrusion extrusion = fixture.add(
        nested, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    const slic3r_api::SeamPlacer placer = create_cost_session(fixture.prepared.print);
    const c_point point = fixture.local(placer.place_seam(
        extrusion.root(), fixture.region_view(), {}));
    CHECK(std::hypot(unscaled(point.x), unscaled(point.y)) == Approx(10).margin(0.0001));
    CHECK(unscaled(point.x) > 5.5);
    CHECK(unscaled(point.x) < 6.5);
    CHECK(unscaled(point.y) > 7.5);
}

TEST_CASE("Cost seam keeps supported candidates ahead of significant overhangs",
          "[plugins][seam-placer][cost][overhang]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ExtrusionEntity loop(false, ArcPolyline(Points{
        Point::new_scale(5, -5), Point::new_scale(5, 5), Point::new_scale(-5, 5),
        Point::new_scale(-5, -5), Point::new_scale(5, -5)}));

    CostFixture supported({{"seam_visibility", "0"}, {"seam_angle_cost", "0"},
                           {"seam_travel_cost", "0"}}, 1);
    const slic3r_api::PrintingExtrusion all_supported = supported.add(
        loop, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    const slic3r_api::SeamPlacer supported_placer = create_cost_session(
        supported.prepared.print);
    const c_point baseline = supported.local(supported_placer.place_seam(
        all_supported.root(), supported.region_view(), {}));
    CHECK(baseline.x == scale_i(5));
    CHECK(baseline.y == scale_i(-5));

    CostFixture partial({{"seam_visibility", "0"}, {"seam_angle_cost", "0"},
                         {"seam_travel_cost", "0"}}, 1);
    ApiInternal::LayerAccess::slices_mutable(partial.object->layer(0)) = {
        rectangle_expolygon(-10, -10, 0, 10)};
    const slic3r_api::PrintingExtrusion partly_overhanging = partial.add(
        loop, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    const slic3r_api::SeamPlacer partial_placer = create_cost_session(
        partial.prepared.print);
    const c_point preferred = partial.local(partial_placer.place_seam(
        partly_overhanging.root(), partial.region_view(), {}));
    CHECK(preferred.x == scale_i(-5));
    CHECK(preferred.y == scale_i(5));
}

TEST_CASE("Cost seam prefers candidates embedded more than half a millimeter",
          "[plugins][seam-placer][cost][embedding]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ExtrusionEntity loop(false, ArcPolyline(Points{
        Point::new_scale(-10, 0), Point::new_scale(0, 0), Point::new_scale(0, 1),
        Point::new_scale(-10, 1), Point::new_scale(-10, 0)}));

    CostFixture single;
    single.source_ptr = attach_native_source(single);
    single.source_ptr->mutable_extrusion(LayerRegionIsland::PERIMETERS).append(loop);
    const slic3r_api::PrintingExtrusion single_loop = single.add(
        loop, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    const slic3r_api::SeamPlacer single_placer = create_cost_session(single.prepared.print);
    const c_point baseline = single.local(single_placer.place_seam(
        single_loop.root(), single.region_view(), {}));
    CHECK(baseline.x == scale_i(-10));
    CHECK(baseline.y == 0);

    CostFixture multiple;
    Layer &layer = multiple.object->layer(0);
    multiple.source_ptr = attach_native_source(multiple);
    multiple.source_ptr->mutable_extrusion(LayerRegionIsland::PERIMETERS).append(loop);
    layer.island(0).mutable_regions_islands().emplace_back(
        new LayerRegionIsland({multiple.region}, 0));
    LayerRegionIsland &second = layer.island(0).regions_island(
        layer.island(0).regions_islands().size() - 1);
    second.mutable_extrusion(LayerRegionIsland::PERIMETERS).append(loop);
    const slic3r_api::PrintingExtrusion embedded_loop = multiple.add(
        loop, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    const slic3r_api::SeamPlacer multiple_placer = create_cost_session(
        multiple.prepared.print);
    const c_point embedded = multiple.local(multiple_placer.place_seam(
        embedded_loop.root(), multiple.region_view(), {}));
    CHECK(embedded.x == 0);
    CHECK(embedded.y == 0);
}

TEST_CASE("Cost seam visibility switch changes the selected model-facing point",
          "[plugins][seam-placer][cost][visibility]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ExtrusionEntity loop(false, ArcPolyline(Points{
        Point::new_scale(10, -10), Point::new_scale(10, 10),
        Point::new_scale(-10, 10), Point::new_scale(-10, -10),
        Point::new_scale(10, -10)}));
    CostFixture disabled({{"seam_visibility", "0"}, {"seam_angle_cost", "0"},
                          {"seam_travel_cost", "0"}});
    disabled.box(9, -11, 2, 2, ModelVolumeType::MODEL_PART);
    const slic3r_api::PrintingExtrusion disabled_loop = disabled.add(
        loop, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    const slic3r_api::SeamPlacer disabled_placer = create_cost_session(
        disabled.prepared.print);
    const c_point without_visibility = disabled.local(disabled_placer.place_seam(
        disabled_loop.root(), disabled.region_view(), {}));
    CHECK(without_visibility.x == scale_i(10));
    CHECK(without_visibility.y == scale_i(-10));

    CostFixture enabled({{"seam_visibility", "1"}, {"seam_angle_cost", "0"},
                         {"seam_travel_cost", "0"}});
    enabled.box(9, -11, 2, 2, ModelVolumeType::MODEL_PART);
    const slic3r_api::PrintingExtrusion enabled_loop = enabled.add(
        loop, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    const slic3r_api::SeamPlacer enabled_placer = create_cost_session(
        enabled.prepared.print);
    const c_point with_visibility = enabled.local(enabled_placer.place_seam(
        enabled_loop.root(), enabled.region_view(), {}));
    CHECK(with_visibility.x == scale_i(-10));
    CHECK(with_visibility.y == scale_i(-10));
}

TEST_CASE("Cost seam visibility samples at the legacy layer slice Z",
          "[plugins][seam-placer][cost][visibility]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ExtrusionEntity loop = rectangle_loop(-10, -10, 10, 10, true);
    CostFixture fixture({{"seam_visibility", "1"}, {"seam_angle_cost", "0"},
                         {"seam_travel_cost", "0"}, {"nozzle_diameter", "20"},
                         {"layer_height", "8"}, {"first_layer_height", "8"}});
    const slic3r_api::Layer layer = fixture.region_view().layer();
    REQUIRE(unscaled(layer.slice_z()) == Approx(4.0));
    REQUIRE(unscaled(layer.print_z()) == Approx(8.0));
    fixture.box(9, -11, 2, 2, ModelVolumeType::MODEL_PART,
                EnforcerBlockerType::NONE, 0, 5);
    const slic3r_api::PrintingExtrusion extrusion = fixture.add(
        loop, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    const slic3r_api::SeamPlacer placer = create_cost_session(fixture.prepared.print);
    const c_point seam = fixture.local(placer.place_seam(
        extrusion.root(), fixture.region_view(), {}));

    CHECK(seam.x == scale_i(10));
    CHECK(seam.y == scale_i(10));
}

TEST_CASE("Cost seam reuses object candidates for translated instances and stable islands",
          "[plugins][seam-placer][cost][instances]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    CostFixture fixture;
    PrintInstances &instances = const_cast<PrintInstances &>(fixture.object->instances());
    instances.push_back(instances.front());
    instances.back().shift += Point::new_scale(75, -40);
    const ExtrusionEntity loop = rectangle_loop(-5, -5, 5, 5);
    fixture.add(loop, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER, 0);
    fixture.add(loop, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER, 1);
    std::unique_ptr<LayerRegionIsland> other(new LayerRegionIsland({fixture.region}, 0));
    fixture.add(loop, RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER, 0, other.get());
    const slic3r_api::SeamPlacer placer = create_cost_session(fixture.prepared.print);

    const slic3r_api::PrintingExtrusion first = fixture.extrusion(0);
    const slic3r_api::PrintingExtrusion second = fixture.extrusion(1);
    const slic3r_api::PrintingExtrusion other_loop = fixture.extrusion(2);

    const c_point first_point = fixture.local(placer.place_seam(
        first.root(), fixture.region_view(), first.root().front()), 0);
    const c_point second_point = fixture.local(placer.place_seam(
        second.root(), fixture.region_view(), second.root().front()), 1);
    const c_point other_point = fixture.local(placer.place_seam(
        other_loop.root(), fixture.region_view(), other_loop.root().front()), 0);
    CHECK(first_point.x == second_point.x);
    CHECK(first_point.y == second_point.y);
    CHECK(first_point.x == other_point.x);
    CHECK(first_point.y == other_point.y);
}

TEST_CASE("Cost seam initializes only the assigned regional providers",
          "[plugins][seam-placer][cost][regions]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    CostFixture fixture;
    fixture.source_ptr = attach_native_source(fixture);
    LayerRegion *other_region = add_neighbor_region(
        fixture, rectangle_expolygon(5.19, -1, 7, 1));
    LayerSliceIsland &layer_island = fixture.object->layer(0).island(0);
    LayerRegionIsland &other_island = layer_island.regions_island(
        layer_island.regions_islands().size() - 1);
    fixture.add(rectangle_loop(-5, -5, 5, 5),
                RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    fixture.add(rectangle_loop(6, -1, 7, 1),
                RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER, 0, &other_island);

    const std::vector<slic3r_api::PrintRegion> assigned = {
        fixture.region_view().print_region()};
    const slic3r_api::SeamPlacer placer = create_cost_session_for(
        fixture.prepared.print, assigned);
    const slic3r_api::PrintingExtrusion included = fixture.extrusion(0);
    const slic3r_api::PrintingExtrusion excluded = fixture.extrusion(1);
    CHECK_NOTHROW(placer.place_seam(
        included.root(), fixture.region_view(), included.root().front()));
    const slic3r_api::LayerRegion excluded_region(
        reinterpret_cast<const layer_region_handle *>(other_region));
    CHECK_THROWS_WITH(placer.place_seam(
        excluded.root(), excluded_region, excluded.root().front()),
        "Cost seam LayerRegion was not assigned to this provider session.");
}

TEST_CASE("Cost seam constraints ignore hostile unassigned regional extrusions",
          "[plugins][seam-placer][cost][regions][isolation]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    CostFixture fixture;
    fixture.source_ptr = attach_native_source(fixture);
    LayerRegion *other_region = add_neighbor_region(
        fixture, rectangle_expolygon(20, 20, 22, 22));
    LayerSliceIsland &layer_island = fixture.object->layer(0).island(0);
    LayerRegionIsland &other_island = layer_island.regions_island(
        layer_island.regions_islands().size() - 1);
    fixture.add(rectangle_loop(-5, -5, 5, 5),
                RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    fixture.add(rectangle_loop(20, 20, 22, 22),
                RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER, 0, &other_island);
    PrintingExtrusion &hostile = fixture.prepared.print.mutable_printing_plan()
        .groups.front().layers.front().tool_groups.front().extrusions[1];
    // A full-plan initializer reaches this malformed unassigned extrusion and
    // throws while resolving its catalog entry; the routed initializer must skip it.
    hostile.printing_catalog_id = 999999;

    const std::vector<slic3r_api::PrintRegion> assigned = {
        fixture.region_view().print_region()};
    const slic3r_api::SeamPlacer placer = create_cost_session_for(
        fixture.prepared.print, assigned);
    const slic3r_api::PrintingExtrusion included = fixture.extrusion(0);
    const c_point point = fixture.local(placer.place_seam(
        included.root(), fixture.region_view(), included.root().front()));
    CHECK(point.x == scale_i(-5));
    CHECK(point.y == scale_i(-5));
    CHECK(other_region != nullptr);
}

TEST_CASE("Cost seam includes a multi-region island when any source region is assigned",
          "[plugins][seam-placer][cost][regions]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    CostFixture fixture;
    LayerRegion *other_region = add_neighbor_region(
        fixture, rectangle_expolygon(20, 20, 22, 22));
    LayerRegionSetCPtrs combined_regions{fixture.region, other_region};
    REQUIRE(combined_regions.size() == 2);
    // Choose the second pointer as assigned so a first-region-only filter is
    // caught regardless of allocation order in this synthetic fixture.
    LayerRegion *assigned_region = *combined_regions.begin() == fixture.region ?
        other_region : fixture.region;
    const slic3r_api::LayerRegion assigned_view(
        reinterpret_cast<const layer_region_handle *>(assigned_region));
    std::unique_ptr<LayerRegionIsland> combined(
        new LayerRegionIsland(combined_regions, 0));
    fixture.box(0, -2, 2, 4, ModelVolumeType::SEAM_POSITION_INSIDE);
    const slic3r_api::PrintingExtrusion extrusion = fixture.add(
        rectangle_loop(-1, -1, 1, 1), RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER,
        0, combined.get());
    const std::vector<slic3r_api::PrintRegion> assigned_regions = {
        assigned_view.print_region()};

    const slic3r_api::SeamPlacer placer = create_cost_session_for(
        fixture.prepared.print, assigned_regions);
    const c_point point = fixture.local(placer.place_seam(
        extrusion.root(), assigned_view, extrusion.root().front()));
    CHECK(point.x >= scale_i(0));
}

TEST_CASE("Cost seam sessions reject repeated initialization without changing state",
          "[plugins][seam-placer][cost][lifecycle]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    CostFixture fixture;
    fixture.add(rectangle_loop(-5, -5, 5, 5),
                RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    PrintingPlan &native_plan = fixture.prepared.print.mutable_printing_plan();
    const slic3r_api::PrintingPlan plan(
        reinterpret_cast<printing_plan_handle *>(&native_plan));
    const std::vector<slic3r_api::PrintRegion> regions = {
        fixture.region_view().print_region()};
    PluginStorage storage;
    std::unique_ptr<slic3r_api::SeamPlacerSession> session =
        slic3r_api::SeamPlacement::CostPlugin::make_cost_seam_session(
            reinterpret_cast<storage_handle *>(&storage));
    session->initialize(plan, regions);
    CHECK_THROWS_WITH(session->initialize(plan, regions),
        "Cost-based seam placement sessions may only be initialized once.");

    const slic3r_api::PrintingExtrusion extrusion = fixture.extrusion(0);
    const c_point point = fixture.local(session->place_seam(
        extrusion.root(), fixture.region_view(), extrusion.root().front()));
    CHECK(point.x == scale_i(-5));
    CHECK(point.y == scale_i(-5));
}

TEST_CASE("Cost seam diagnoses perimeter islands without external geometry",
          "[plugins][seam-placer][cost][errors]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    CostFixture fixture;
    fixture.add(rectangle_loop(0, 0, 10, 10), RAW_EXTRUSION_ROLE_PERIMETER);
    CHECK_THROWS_WITH(create_cost_session(fixture.prepared.print),
        Catch::Matchers::Contains("found perimeter loops but no external geometric candidate"));
}

TEST_CASE("Cost seam diagnoses invalid source islands",
          "[plugins][seam-placer][cost][errors]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    CostFixture fixture;
    fixture.add(rectangle_loop(0, 0, 10, 10), RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
    PrintingExtrusion &invalid = fixture.prepared.print.mutable_printing_plan().groups.front()
        .layers.front().tool_groups.front().extrusions.front();
    invalid.region_island = nullptr;
    CHECK_THROWS_WITH(create_cost_session(fixture.prepared.print),
        "Cost-based seam placement found a perimeter loop with an invalid source LayerRegionIsland.");
}
