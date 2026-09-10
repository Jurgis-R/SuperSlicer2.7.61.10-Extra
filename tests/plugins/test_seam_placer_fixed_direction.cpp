#include <catch2/catch.hpp>

#include <algorithm>
#include <array>
#include <fstream>
#include <functional>
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
#include "libslic3r/Api/internal/ArcPolylineAccess.hpp"
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

#include <future>
#include "libslic3r/TriangleSelector.hpp"
#include "libslic3r/Api/plugin/cpp/VolumeViews.hpp"
#include "libslic3r/Plugins/SeamPlacer/SeamPlacementConstraints.hpp"

namespace {
/* Quarter arcs avoid the native diagnostic orientation ambiguity at exactly
   180 degrees; their caches are initialized before applying an instance shift. */
ExtrusionEntity constraint_arc_loop()
{
    const coord_t radius=scale_i(10.0);
    Geometry::ArcWelder::Path path;
    path.emplace_back(Point(-radius,0),0.f,Geometry::ArcWelder::Orientation::Unknown);
    path.emplace_back(Point(0,-radius),float(radius),Geometry::ArcWelder::Orientation::CCW);
    path.emplace_back(Point(radius,0),float(radius),Geometry::ArcWelder::Orientation::CCW);
    path.emplace_back(Point(0,radius),float(radius),Geometry::ArcWelder::Orientation::CCW);
    path.emplace_back(Point(-radius,0),float(radius),Geometry::ArcWelder::Orientation::CCW);
    ArcPolyline polyline(path);
    ApiInternal::ArcPolylineAccess::refresh_after_bulk_replace(polyline);
    return ExtrusionEntity(false,polyline);
}

/* Own real source metadata and a plan clone; modifier meshes are expressed in
   centered object coordinates then converted back to model coordinates. */
struct ConstraintFixture : DirectionRegionFixture
{
    ConstraintFixture(const char *direction = "rear") : DirectionRegionFixture(direction), source({region},0) {}
    LayerRegionIsland source;

    ModelVolume *box(double x,double y,double sx,double sy,ModelVolumeType type,
                     EnforcerBlockerType painting=EnforcerBlockerType::NONE,double z=0,double sz=30)
    {
        PrintObject &object=prepared.print.object(0);
        TriangleMesh mesh=make_cube(sx,sy,sz);
        mesh.translate(Vec3f(float(x),float(y),float(z)));
        mesh.transform(object.trafo_centered().inverse());
        ModelVolume *volume=const_cast<ModelObject *>(object.model_object())->add_volume(std::move(mesh),type,false);
        if (painting!=EnforcerBlockerType::NONE) {
            TriangleSelector selector(volume->mesh());
            for (size_t i=0;i<volume->mesh().its.indices.size();++i) selector.set_facet(int(i),painting);
            volume->facets_annotation_mutable("builtin:seam").set(selector);
        }
        return volume;
    }

    slic3r_api::PrintingExtrusion add(const ExtrusionEntity &loop, uint16_t instance=0)
    {
        slic3r_api::PrintingPlan plan(reinterpret_cast<printing_plan_handle *>(&prepared.print.mutable_printing_plan()));
        if (plan.group_count()==0) {
            plan.append_group().append_layer_group(view().layer().print_z()).append_tool_group(0);
        }
        slic3r_api::PrintingExtrusion extrusion=plan.group(0).layer_group(0).tool_group(0).append_extrusion_clone(
            slic3r_api::LayerRegionIsland(reinterpret_cast<const layer_region_island_handle *>(&source)),
            RAW_EXTRUSION_ROLE_PERIMETER,extrusion_view(loop),instance);
        const c_point shift=view().layer().object().instance_shift(instance);
        std::function<void(slic3r_api::MutableExtrusionEntity)> translate;
        translate=[&](slic3r_api::MutableExtrusionEntity entity) {
            if (entity.point_count()>0) REQUIRE(entity.translate(shift));
            for (uint32_t i=0;i<entity.child_count();++i) translate(entity.child_mutable(i));
        };
        translate(extrusion.mutable_root());
        return extrusion;
    }

    c_point local(c_point point, uint16_t instance=0) const
    {
        const c_point shift=view().layer().object().instance_shift(instance);
        return {point.x-shift.x,point.y-shift.y};
    }
};
}

TEST_CASE("Fixed seam constraints enforce narrow interior segments and true arcs", "[plugins][seam-placer][fixed][constraints]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ConstraintFixture fixture("right");
    const bool arc=GENERATE(false,true);
    fixture.box(arc?5.9:4.9,arc?7.8:-0.1,0.2,0.4,ModelVolumeType::SEAM_POSITION_INSIDE);
    ExtrusionEntity loop=arc?constraint_arc_loop():square_loop();
    const slic3r_api::PrintingExtrusion extrusion=fixture.add(loop);
    slic3r_api::SeamPlacer session=create_fixed_session(fixture.prepared.print);
    const c_point point=fixture.local(session.place_seam(extrusion.root(),fixture.view(),{0,0}));
    CHECK(unscaled(point.x)>= (arc?5.899:4.899));
    CHECK(unscaled(point.x)<= (arc?6.101:5.101));
    if (arc) {
        CHECK(std::hypot(unscaled(point.x),unscaled(point.y))==Approx(10).margin(0.0001));
        CHECK(unscaled(point.y)>7.8);
    } else CHECK(point.y==0);
    CHECK(extrusion_view(loop).point_count()==extrusion.root().point_count());
}

TEST_CASE("Fixed seam constraints prioritize paint blockers over paint and volume enforcers", "[plugins][seam-placer][fixed][constraints]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ConstraintFixture fixture;
    const bool volume=GENERATE(false,true);
    fixture.box(4,-1,2,2,volume?ModelVolumeType::SEAM_POSITION_INSIDE:ModelVolumeType::MODEL_PART,
                volume?EnforcerBlockerType::NONE:EnforcerBlockerType::ENFORCER);
    const bool block=GENERATE(false,true);
    if (block) fixture.box(3,-2,4,4,ModelVolumeType::MODEL_PART,EnforcerBlockerType::BLOCKER);
    const slic3r_api::PrintingExtrusion extrusion=fixture.add(square_loop());
    slic3r_api::SeamPlacer session=create_fixed_session(fixture.prepared.print);
    const c_point point=fixture.local(session.place_seam(extrusion.root(),fixture.view(),{0,0}));
    if (block) CHECK(same_point(point,mm_point(10,10)));
    else {
        CHECK(point.y==0);
        CHECK(unscaled(point.x)>3);
        CHECK(unscaled(point.x)<7);
    }
}

TEST_CASE("Fixed seam constraints ignore attractors and absent layers and fall back when fully blocked", "[plugins][seam-placer][fixed][constraints]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ConstraintFixture fixture;
    const int mode=GENERATE(0,1,2,3,4);
    if (mode<3) fixture.box(4,-1,2,2,mode==0?ModelVolumeType::SEAM_POSITION_CENTER:
        mode==1?ModelVolumeType::SEAM_POSITION_CENTER_Z:ModelVolumeType::SEAM_POSITION_INSIDE_CENTER);
    if (mode==3) fixture.box(4,-1,2,2,ModelVolumeType::SEAM_POSITION_INSIDE,EnforcerBlockerType::NONE,25,5);
    if (mode==4) fixture.box(-10,-10,40,40,ModelVolumeType::MODEL_PART,EnforcerBlockerType::BLOCKER);
    const slic3r_api::PrintingExtrusion extrusion=fixture.add(square_loop());
    slic3r_api::SeamPlacer session=create_fixed_session(fixture.prepared.print);
    CHECK(same_point(fixture.local(session.place_seam(extrusion.root(),fixture.view(),{0,0})),mm_point(10,10)));
    if (mode==4) {
        PluginStorage storage;
        slic3r_api::SeamPlacement::SeamPlacementConstraints constraints;
        constraints.initialize(slic3r_api::PrintingPlan(reinterpret_cast<printing_plan_handle *>(&fixture.prepared.print.mutable_printing_plan())),
                               reinterpret_cast<storage_handle *>(&storage));
        const slic3r_api::SeamPlacement::SeamPlacementConstraints::Result result=constraints.portions(extrusion.root(),fixture.view());
        REQUIRE(result.relevant);
        for (const slic3r_api::SeamPlacement::SeamPlacementConstraints::Portion &portion:result.portions)
            CHECK(portion.status==slic3r_api::SeamPlacement::SeamPlacementConstraints::Status::Blocked);
    }
}

TEST_CASE("Fixed seam constraint cache is immutable across parallel requests", "[plugins][seam-placer][fixed][constraints]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ConstraintFixture fixture;
    fixture.box(4,-1,2,2,ModelVolumeType::SEAM_POSITION_INSIDE);
    const slic3r_api::PrintingExtrusion extrusion=fixture.add(composed_loop());
    slic3r_api::SeamPlacer session=create_fixed_session(fixture.prepared.print);
    const c_point expected=session.place_seam(extrusion.root(),fixture.view(),{0,0});
    std::vector<std::future<c_point>> futures;
    for (int i=0;i<8;++i) futures.push_back(std::async(std::launch::async,[&session,&fixture,&extrusion,i] {
        return session.place_seam(extrusion.root(),fixture.view(),{i*100,i*300});
    }));
    for (std::future<c_point> &future:futures) CHECK(same_point(future.get(),expected));
}

TEST_CASE("Seam constraints use the plan instance rather than the incoming position", "[plugins][seam-placer][fixed][constraints]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ConstraintFixture fixture;
    PrintObject &object=fixture.prepared.print.object(0);
    // Synthetic second placement: keep the sliced source tree unchanged.
    PrintInstances &instances=const_cast<PrintInstances &>(object.instances());
    instances.push_back(instances.front());
    instances.back().shift+=Point::new_scale(100,-50);
    fixture.box(4,-1,2,2,ModelVolumeType::SEAM_POSITION_INSIDE);
    fixture.add(square_loop(),0);
    fixture.add(square_loop(),1);
    slic3r_api::PrintingPlan plan(reinterpret_cast<printing_plan_handle *>(&fixture.prepared.print.mutable_printing_plan()));
    slic3r_api::SeamPlacer session=create_fixed_session(fixture.prepared.print);
    for (uint16_t i=0;i<2;++i) {
        const slic3r_api::PrintingExtrusion extrusion=plan.group(0).layer_group(0).tool_group(0).extrusion(i);
        const c_point point=fixture.local(session.place_seam(extrusion.root(),fixture.view(),{-100000,200000}),i);
        CHECK(point.y==0);
        CHECK(unscaled(point.x)==Approx(6).margin(0.001));
    }
}

TEST_CASE("Seam volume slices preserve holes and disconnected contours", "[plugins][seam-placer][fixed][constraints]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ConstraintFixture fixture("left");
    TriangleMesh mesh=make_cube(8,4,30);
    mesh.translate(Vec3f(1,-2,0));
    TriangleMesh hole=make_cube(4,2,30);
    hole.translate(Vec3f(3,-1,0));
    for (stl_triangle_vertex_indices &face:hole.its.indices) std::swap(face[0],face[1]);
    mesh.merge(hole);
    TriangleMesh separate=make_cube(0.2,0.2,30);
    separate.translate(Vec3f(-0.1f,4,0));
    mesh.merge(separate);
    PrintObject &object=fixture.prepared.print.object(0);
    mesh.transform(object.trafo_centered().inverse());
    const_cast<ModelObject *>(object.model_object())->add_volume(std::move(mesh),ModelVolumeType::SEAM_POSITION_INSIDE,false);
    const slic3r_api::PrintingExtrusion extrusion=fixture.add(square_loop());
    PluginStorage storage;
    slic3r_api::SeamPlacement::SeamPlacementConstraints constraints;
    constraints.initialize(slic3r_api::PrintingPlan(reinterpret_cast<printing_plan_handle *>(&fixture.prepared.print.mutable_printing_plan())),
                           reinterpret_cast<storage_handle *>(&storage));
    CHECK(storage.size()==0);
    const c_point shift=fixture.view().layer().object().instance_shift(0);
    const std::pair<c_point,slic3r_api::SeamPlacement::SeamPlacementConstraints::Status> checks[]={
        {mm_point(2,0),slic3r_api::SeamPlacement::SeamPlacementConstraints::Status::Enforced},
        {mm_point(5,0),slic3r_api::SeamPlacement::SeamPlacementConstraints::Status::Free},
        {mm_point(0,4.1),slic3r_api::SeamPlacement::SeamPlacementConstraints::Status::Enforced}
    };
    for (const std::pair<c_point,slic3r_api::SeamPlacement::SeamPlacementConstraints::Status> &check:checks)
        CHECK(constraints.classify_point(extrusion.root(),{check.first.x+shift.x,check.first.y+shift.y},0)==check.second);
}

TEST_CASE("Seam volume transformations are composed in object space", "[plugins][seam-placer][fixed][constraints]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ConstraintFixture fixture;
    PrintObject &object=fixture.prepared.print.object(0);
    TriangleMesh mesh=make_cube(2,0.2,30);
    ModelVolume *volume=const_cast<ModelObject *>(object.model_object())->add_volume(std::move(mesh),ModelVolumeType::SEAM_POSITION_INSIDE,false);
    Transform3d placement=Transform3d::Identity();
    placement.translate(Vec3d(5.1,-1,0));
    placement.rotate(Eigen::AngleAxisd(3.14159265358979323846/2,Vec3d::UnitZ()));
    volume->set_transformation(object.trafo_centered().inverse()*placement);
    const slic3r_api::PrintingExtrusion extrusion=fixture.add(square_loop());
    slic3r_api::SeamPlacer session=create_fixed_session(fixture.prepared.print);
    const c_point point=fixture.local(session.place_seam(extrusion.root(),fixture.view(),{0,0}));
    CHECK(point.y==0);
    CHECK(unscaled(point.x)==Approx(5.1).margin(0.001));
}

TEST_CASE("Seam paint margins inherit width and fall back to regional flow", "[plugins][seam-placer][fixed][constraints]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ConstraintFixture fixture;
    fixture.box(4,0.5,2,0.1,ModelVolumeType::MODEL_PART,EnforcerBlockerType::ENFORCER);
    const slic3r_api::PrintingExtrusion extrusion=fixture.add(composed_loop());
    const bool explicit_width=GENERATE(false,true);
    if (explicit_width) {
        orchestrator_handle *orch=reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance());
        c_extrusion_property_attributes *parent=static_cast<c_extrusion_property_attributes *>(
            extrusion.mutable_root().get_or_add_property_data_mutable(orch,EXTRUSION_PROPERTY_TYPE_ATTRIBUTES));
        REQUIRE(parent!=nullptr);
        parent->width=0.8f;
        parent->role=RAW_EXTRUSION_ROLE_PERIMETER;
        c_extrusion_property_attributes *child=static_cast<c_extrusion_property_attributes *>(
            extrusion.mutable_root().child_mutable(1).get_or_add_property_data_mutable(orch,EXTRUSION_PROPERTY_TYPE_ATTRIBUTES));
        REQUIRE(child!=nullptr);
        child->width=0.2f;
        child->role=RAW_EXTRUSION_ROLE_PERIMETER;
    }
    PluginStorage storage;
    slic3r_api::SeamPlacement::SeamPlacementConstraints constraints;
    constraints.initialize(slic3r_api::PrintingPlan(reinterpret_cast<printing_plan_handle *>(&fixture.prepared.print.mutable_printing_plan())),
                           reinterpret_cast<storage_handle *>(&storage));
    const slic3r_api::SeamPlacement::SeamPlacementConstraints::Result portions=constraints.portions(extrusion.root(),fixture.view());
    REQUIRE_FALSE(portions.portions.empty());
    CHECK(portions.portions.front().paint_margin==Approx(explicit_width?Slic3r::scale_d(0.8f):fixture.view().flow(RAW_EXTRUSION_ROLE_PERIMETER).width));
    if (explicit_width) {
        bool narrow=false;
        for (const slic3r_api::SeamPlacement::SeamPlacementConstraints::Portion &portion:portions.portions)
            narrow|=std::abs(portion.paint_margin-Slic3r::scale_d(0.2f))<1;
        CHECK(narrow);
        const c_point shift=fixture.view().layer().object().instance_shift(0),point=mm_point(5,0);
        CHECK(constraints.classify_point(extrusion.root(),{point.x+shift.x,point.y+shift.y},Slic3r::scale_d(0.8f))==
            slic3r_api::SeamPlacement::SeamPlacementConstraints::Status::Enforced);
        CHECK(constraints.classify_point(extrusion.root(),{point.x+shift.x,point.y+shift.y},Slic3r::scale_d(0.2f))==
            slic3r_api::SeamPlacement::SeamPlacementConstraints::Status::Free);
    }
    CHECK(storage.size()==0);
}

TEST_CASE("Seam constraints isolate source objects and layers in one plan", "[plugins][seam-placer][fixed][constraints]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ConstraintFixture first, second;
    first.box(4,-1,2,2,ModelVolumeType::SEAM_POSITION_INSIDE,EnforcerBlockerType::NONE,0,1);
    second.box(1,-1,1,2,ModelVolumeType::SEAM_POSITION_INSIDE);
    const slic3r_api::PrintingExtrusion low=first.add(square_loop());
    PrintObject &object=first.prepared.print.object(0);
    LayerRegion &upper=object.layer(object.layer_count()-1).region(0);
    LayerRegionIsland upper_source({&upper},0);
    slic3r_api::PrintingPlan plan(reinterpret_cast<printing_plan_handle *>(&first.prepared.print.mutable_printing_plan()));
    const slic3r_api::LayerRegion upper_view(reinterpret_cast<const layer_region_handle *>(&upper));
    const ExtrusionEntity square=square_loop();
    const slic3r_api::PrintingExtrusion high=plan.group(0).append_layer_group(upper_view.layer().print_z()).append_tool_group(0).append_extrusion_clone(
        slic3r_api::LayerRegionIsland(reinterpret_cast<const layer_region_island_handle *>(&upper_source)),
        RAW_EXTRUSION_ROLE_PERIMETER,extrusion_view(square),0);
    REQUIRE(high.mutable_root().translate(upper_view.layer().object().instance_shift(0)));
    // A second independently sliced source object shares this synthetic plan.
    const slic3r_api::PrintingExtrusion other=plan.append_group().append_layer_group(second.view().layer().print_z()).append_tool_group(0).append_extrusion_clone(
        slic3r_api::LayerRegionIsland(reinterpret_cast<const layer_region_island_handle *>(&second.source)),
        RAW_EXTRUSION_ROLE_PERIMETER,extrusion_view(square),0);
    REQUIRE(other.mutable_root().translate(second.view().layer().object().instance_shift(0)));
    slic3r_api::SeamPlacer session=create_fixed_session(first.prepared.print);
    CHECK(same_point(first.local(session.place_seam(low.root(),first.view(),{0,0})),mm_point(6,0)));
    CHECK(same_point(first.local(session.place_seam(high.root(),upper_view,{0,0})),mm_point(10,10)));
    const c_point other_point=second.local(session.place_seam(other.root(),second.view(),{0,0}));
    CHECK(other_point.y==0);
    CHECK(unscaled(other_point.x)==Approx(2).margin(0.00001));
}

TEST_CASE("Seam constraint boundaries and degenerate segments remain classified", "[plugins][seam-placer][fixed][constraints]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    ConstraintFixture fixture;
    fixture.box(4,-1,2,2,ModelVolumeType::SEAM_POSITION_INSIDE);
    slic3r_api::SeamPlacement::SeamPlacementConstraints::Curve degenerate;
    degenerate.segment.point_a=degenerate.segment.point_b=mm_point(4,0);
    CHECK(same_point(degenerate.point(0.5),mm_point(4,0)));
    const ExtrusionEntity loop(false,ArcPolyline(Points{Point::new_scale(0,0),
        Point::new_scale(10,0),Point::new_scale(10,10),Point::new_scale(0,10),Point::new_scale(0,0)}));
    const slic3r_api::PrintingExtrusion extrusion=fixture.add(loop);
    PluginStorage storage;
    slic3r_api::SeamPlacement::SeamPlacementConstraints constraints;
    constraints.initialize(slic3r_api::PrintingPlan(reinterpret_cast<printing_plan_handle *>(&fixture.prepared.print.mutable_printing_plan())),
                           reinterpret_cast<storage_handle *>(&storage));
    const c_point shift=fixture.view().layer().object().instance_shift(0);
    {
        const slic3r_api::Object object=fixture.view().layer().object();
        const slic3r_api::Volume volume=object.volume(object.volume_count()-1);
        const c_point center=object.center_offset();
        const c_matrix4d transform=matrix4d_mul(
            matrix4d_translation(-unscaled(center.x),-unscaled(center.y),0),object.transform());
        // Use actual sliced boundaries, including native float-to-integer rounding.
        const std::vector<slic3r_api::StoredPolygonCollection> slices=volume.mesh().slice_to_polygons(
            reinterpret_cast<storage_handle *>(&storage),matrix4d_mul(transform,volume_get_matrix(volume.handle())),
            {float(unscaled(fixture.view().layer().print_z()))});
        REQUIRE(slices.front().readonly().size()>0);
        const slic3r_api::Polygon polygon=slices.front().readonly().at(0);
        for (uint32_t i=0;i<polygon.size();++i) {
            const c_point point=polygon.at(i);
            CHECK(constraints.classify_point(extrusion.root(),{point.x+shift.x,point.y+shift.y},0)==
                slic3r_api::SeamPlacement::SeamPlacementConstraints::Status::Enforced);
        }
    }
    slic3r_api::SeamPlacer session=create_fixed_session(fixture.prepared.print);
    const c_point result=fixture.local(session.place_seam(extrusion.root(),fixture.view(),{0,0}));
    CHECK(result.y==0);
    CHECK(unscaled(result.x)==Approx(6).margin(0.00001));
    CHECK(storage.size()==0);
}
