#include <catch2/catch.hpp>

#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/internal/LayerAccess.hpp"
#include "libslic3r/Api/internal/LayerRegionAccess.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepGeneratePerimeter.hpp"
#include "libslic3r/Steps/StepPostSlicing.hpp"
#include "libslic3r/Steps/StepSurfaceGeneration.hpp"
#include "libslic3r/Surface.hpp"
#include "libslic3r/SurfaceCollection.hpp"

#include <cmath>
#include <initializer_list>
#include <string>
#include <vector>

namespace {
using namespace Slic3r;
using namespace Slic3r::Test::PerimeterPluginTests;

class ScopedActivePlugins
{
public:
    explicit ScopedActivePlugins(std::initializer_list<const char *> plugin_ids) :
        m_orchestrator(Orchestrator::instance())
    {
        // These tests need a very small surface-generation pipeline. Save the
        // global plugin state and restore it in the destructor so one test case
        // cannot leak active plugins into another one.
        m_previous_active_plugins.reserve(m_orchestrator.active_plugins().size());
        for (Plugin *plugin : m_orchestrator.active_plugins())
            m_previous_active_plugins.push_back(plugin);

        m_orchestrator.clear_active_plugins();
        for (const char *plugin_id : plugin_ids) {
            INFO("Activating test plugin " << plugin_id);
            REQUIRE(m_orchestrator.set_plugin_active(plugin_id, true));
        }
    }

    ~ScopedActivePlugins()
    {
        m_orchestrator.clear_active_plugins();
        for (Plugin *plugin : m_previous_active_plugins)
            m_orchestrator.set_plugin_active(plugin, true);
    }

private:
    Orchestrator &m_orchestrator;
    std::vector<Plugin *> m_previous_active_plugins;
};

double area_sum(const ExPolygons &areas)
{
    double out = 0.;
    for (const ExPolygon &area : areas)
        out += std::abs(area.area());
    return out;
}

double area_tolerance()
{
    const double side = double(scale_i(0.005));
    return side * side;
}

ExPolygons surface_expolygons(const SurfaceCollection &surfaces)
{
    ExPolygons out;
    out.reserve(surfaces.size());
    for (const Surface &surface : surfaces)
        if (!surface.empty())
            out.push_back(surface.expolygon);
    return out;
}

ExPolygons surface_expolygons_of_type(const SurfaceCollection &surfaces, const SurfaceType surface_type)
{
    ExPolygons out;
    out.reserve(surfaces.size());
    for (const Surface &surface : surfaces)
        if (!surface.empty() && surface.surface_type == surface_type)
            out.push_back(surface.expolygon);
    return out;
}

void require_same_union(const ExPolygons &actual, const ExPolygons &expected)
{
    const ExPolygons actual_union = union_ex(actual);
    const ExPolygons expected_union = union_ex(expected);
    INFO("actual area " << area_sum(actual_union) << ", expected area " << area_sum(expected_union));
    REQUIRE(area_sum(diff_ex(actual_union, expected_union)) <= area_tolerance());
    REQUIRE(area_sum(diff_ex(expected_union, actual_union)) <= area_tolerance());
}

void require_no_positive_overlap(const SurfaceCollection &surfaces)
{
    const ExPolygons areas = surface_expolygons(surfaces);
    for (size_t first_idx = 0; first_idx < areas.size(); ++first_idx) {
        for (size_t second_idx = first_idx + 1; second_idx < areas.size(); ++second_idx) {
            const ExPolygons overlap = intersection_ex(ExPolygons{areas[first_idx]}, ExPolygons{areas[second_idx]});
            INFO("surface pair " << first_idx << " / " << second_idx);
            CHECK(area_sum(overlap) <= area_tolerance());
        }
    }
}

const LayerRegionIsland &whole_region_island(const LayerSliceIsland &island)
{
    // The helper tests one-region fixtures most of the time, but some builders
    // may split region islands. Prefer the exact whole-island region set and
    // fall back to the first non-empty region island so failure output still
    // points at the produced surfaces instead of crashing on a null reference.
    const LayerRegionIsland *fallback = nullptr;
    const LayerRegionIsland *fallback_any_region_set = nullptr;
    for (const LayerRegionIsland &region_island : island.regions_islands()) {
        if (!region_island.fill_surfaces().empty() && fallback_any_region_set == nullptr)
            fallback_any_region_set = &region_island;
        if (region_island.regions() != island.regions())
            continue;
        if (!region_island.fill_surfaces().empty())
            return region_island;
        if (fallback == nullptr)
            fallback = &region_island;
    }

    if (fallback_any_region_set != nullptr)
        return *fallback_any_region_set;
    REQUIRE(fallback != nullptr);
    return *fallback;
}

void require_surface_area_contract(const LayerSliceIsland &island)
{
    // Surface-generation plugins are allowed to change types and split shapes,
    // but they must preserve the fill area as a non-overlapping partition. That
    // invariant is what the infill step depends on later.
    const SurfaceCollection &surfaces = whole_region_island(island).fill_surfaces();
    require_no_positive_overlap(surfaces);
    require_same_union(surface_expolygons(surfaces), island.infill_areas());
}

double surface_area(const LayerSliceIsland &island, const SurfaceType surface_type)
{
    return area_sum(surface_expolygons_of_type(whole_region_island(island).fill_surfaces(), surface_type));
}

void replace_surface_position_with_sparse_internal(Print &print, const SurfaceType position_flag)
{
    // This helper keeps the surface geometry intact and changes only one
    // position family. It lets the test exercise SolidShells' prerequisite
    // check directly: a typed object may legitimately have no top anchors or
    // no bottom anchors, and the plugin should then project only the anchors
    // that remain.
    bool replaced = false;
    for (PrintObject &object : print.objects())
        for (Layer &layer : object.layers())
            for (LayerSliceIsland &island : layer.islands())
                for (LayerRegionIsland &region_island : island.regions_islands())
                    for (Surface &surface : region_island.set_fill_surfaces()) {
                        if ((surface.surface_type & position_flag) == stNone)
                            continue;
                        surface.surface_type = stPosInternal | stDensSparse;
                        replaced = true;
                    }
    REQUIRE(replaced);
}

void set_region_area(LayerRegion &region, const ExPolygon &area)
{
    ExPolygons &region_slices = ApiInternal::LayerRegionAccess::slices_mutable(region);
    region_slices = ExPolygons{area};
    ApiInternal::LayerRegionAccess::surfaces_mutable(region).set(region_slices, stPosInternal | stDensSparse);
}

void replace_layer_island(Layer &layer, const ExPolygon &area)
{
    // Tests edit sliced layers directly to create controlled top/bottom cases.
    // After replacing the island geometry, fill_regions() rebuilds the region
    // membership that InitialTypedSurfaceBuilder expects from the slicing step.
    ApiInternal::LayerAccess::set_islands(layer, ExPolygons{area});
    set_region_area(layer.region(0), area);
    layer.island(0).fill_regions(layer);
}

void rebuild_island_overlap_graph(PrintObject &object)
{
    // Top/bottom shell detection uses upper_islands()/lower_islands(), which
    // are cached links. Any direct island edit in a test must rebuild the graph
    // before running surface-generation plugins.
    for (Layer &layer : object.layers())
        for (LayerSliceIsland &island : layer.islands()) {
            island.overlaps_above.clear();
            island.overlaps_below.clear();
        }

    for (size_t layer_idx = 1; layer_idx < object.layer_count(); ++layer_idx)
        Layer::build_up_down_graph(object.layer(layer_idx - 1), object.layer(layer_idx));
}

void run_perimeter_and_surface_steps(Print &print, const bool validate_surface_post = true)
{
    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepGeneratePerimeter::clear(print);
    Steps::StepGeneratePerimeter::run_step(orchestrator, print);
    Steps::StepSurfaceGeneration::clear(print);
    Steps::StepSurfaceGeneration::run_step(orchestrator, print);

    if (validate_surface_post) {
        std::string validation_error;
        const bool valid_surface_tree = Steps::StepSurfaceGeneration::validate_post(print, &validation_error);
        INFO("Surface-generation post validation: " << validation_error);
        REQUIRE(valid_surface_tree);
    }
}

void run_solid_shell_surface_case(PreparedPerimeterPrint &prepared)
{
    ScopedActivePlugins active({SIMPLE_PERIMETER_GENERATOR, INITIAL_TYPED_SURFACE_BUILDER, SOLID_SHELLS});
    run_perimeter_and_surface_steps(prepared.print);
}

void run_solid_shells_on_existing_surfaces(Print &print)
{
    // Some tests first create typed surfaces, then edit only their tags. Running
    // StepSurfaceGeneration without clear() preserves that prepared
    // input and exercises SolidShells as a standalone refinement pass.
    ScopedActivePlugins active({SOLID_SHELLS});
    Steps::StepSurfaceGeneration::run_step(Orchestrator::instance(), print);

    std::string validation_error;
    const bool valid_surface_tree = Steps::StepSurfaceGeneration::validate_post(print, &validation_error);
    INFO("Surface-generation post validation: " << validation_error);
    REQUIRE(valid_surface_tree);
}

void require_no_solid_shell_prerequisite_error(Orchestrator &orchestrator)
{
    const std::vector<Orchestrator::PluginMessage> messages = orchestrator.consume_plugin_messages();
    for (const Orchestrator::PluginMessage &message : messages) {
        INFO("Plugin message: " << message.message);
        const bool is_prerequisite_error =
            message.level == Orchestrator::PluginMessageLevel::Error &&
            message.plugin_id == SOLID_SHELLS &&
            message.message.find("requires typed fill surfaces") != std::string::npos;
        CHECK_FALSE(is_prerequisite_error);
    }
    orchestrator.reset_plugin_cancel();
}

} // namespace

TEST_CASE("SolidShells promotes sparse surface areas required by top and bottom shells",
          "[plugins][surface-generation][solid-shells]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    SECTION("vertical cube keeps a sparse core between configured solid shells")
    {
        // This mirrors a real cube slice: the post-slicing step owns the
        // upper/lower island graph, and no test helper rebuilds it afterward.
        // With three top and bottom solid layers, only the two internal layers
        // near each exposed side should become solid. A middle layer must stay
        // sparse and must not be classified as an unsupported bridge bottom.
        PreparedPerimeterPrint prepared;
        prepare_cube_print(prepared, perimeter_config({
            {"perimeters", "1"},
            {"raft_layers", "0"},
            {"top_solid_layers", "3"},
            {"bottom_solid_layers", "3"},
            {"top_solid_min_thickness", "0"},
            {"bottom_solid_min_thickness", "0"},
            {"solid_over_perimeters", "0"}
        }));
        PrintObject &object = prepared.print.object(0);
        REQUIRE(object.layer_count() > 7);

        {
            ScopedActivePlugins no_post_slicing_plugins({});
            Steps::StepPostSlicing::run_step(Orchestrator::instance(), prepared.print);
        }

        run_solid_shell_surface_case(prepared);

        const LayerSliceIsland &bottom_shell_island = object.layer(1).island(0);
        const LayerSliceIsland &core_island = object.layer(3).island(0);
        const LayerSliceIsland &top_shell_island = object.layer(layer_index_for_top(object) - 1).island(0);
        const SurfaceType bridge_bottom = stPosBottom | stDensSolid | stModBridge;

        require_same_union(surface_expolygons_of_type(whole_region_island(bottom_shell_island).fill_surfaces(),
                                                      stPosInternal | stDensSolid),
                           bottom_shell_island.infill_areas());
        require_same_union(surface_expolygons_of_type(whole_region_island(top_shell_island).fill_surfaces(),
                                                      stPosInternal | stDensSolid),
                           top_shell_island.infill_areas());

        CHECK(surface_area(core_island, bridge_bottom) <= area_tolerance());
        CHECK(surface_area(core_island, stPosInternal | stDensSolid) <= area_tolerance());
        require_same_union(surface_expolygons_of_type(whole_region_island(core_island).fill_surfaces(),
                                                      stPosInternal | stDensSparse),
                           core_island.infill_areas());
    }

    SECTION("top_solid_layers promotes the layer directly below a top surface")
    {
        // With top_solid_layers=2, the visible top layer itself is solid and
        // the first internal layer below it must become internal solid too.
        PreparedPerimeterPrint prepared;
        prepare_cube_print(prepared, perimeter_config({
            {"perimeters", "0"},
            {"top_solid_layers", "2"},
            {"bottom_solid_layers", "0"},
            {"top_solid_min_thickness", "0"},
            {"bottom_solid_min_thickness", "0"},
            {"solid_over_perimeters", "0"}
        }));
        PrintObject &object = prepared.print.object(0);
        REQUIRE(object.layer_count() > 2);
        const size_t layer_idx = layer_index_for_top(object) - 1;
        const ExPolygon area = rectangle_expolygon(-10., -10., 10., 10.);
        replace_layer_island(object.layer(0), area);
        replace_layer_island(object.layer(layer_idx - 1), area);
        replace_layer_island(object.layer(layer_idx), area);
        replace_layer_island(object.layer(layer_idx + 1), area);
        rebuild_island_overlap_graph(object);

        run_solid_shell_surface_case(prepared);

        const LayerSliceIsland &island = object.layer(layer_idx).island(0);
        require_surface_area_contract(island);
        require_same_union(surface_expolygons_of_type(whole_region_island(island).fill_surfaces(),
                                                      stPosInternal | stDensSolid),
                           island.infill_areas());
    }

    SECTION("top_solid_layers=1 leaves the layer below the top sparse")
    {
        // A value of 1 only covers the exposed top layer. The layer below is
        // not part of the requested shell and should stay sparse.
        PreparedPerimeterPrint prepared;
        prepare_cube_print(prepared, perimeter_config({
            {"perimeters", "0"},
            {"top_solid_layers", "1"},
            {"bottom_solid_layers", "0"},
            {"top_solid_min_thickness", "0"},
            {"bottom_solid_min_thickness", "0"},
            {"solid_over_perimeters", "0"}
        }));
        PrintObject &object = prepared.print.object(0);
        REQUIRE(object.layer_count() > 2);
        const size_t layer_idx = layer_index_for_top(object) - 1;
        const ExPolygon area = rectangle_expolygon(-10., -10., 10., 10.);
        replace_layer_island(object.layer(0), area);
        replace_layer_island(object.layer(layer_idx - 1), area);
        replace_layer_island(object.layer(layer_idx), area);
        replace_layer_island(object.layer(layer_idx + 1), area);
        rebuild_island_overlap_graph(object);

        run_solid_shell_surface_case(prepared);

        const LayerSliceIsland &island = object.layer(layer_idx).island(0);
        require_surface_area_contract(island);
        CHECK(surface_area(island, stPosInternal | stDensSolid) <= area_tolerance());
        require_same_union(surface_expolygons_of_type(whole_region_island(island).fill_surfaces(),
                                                      stPosInternal | stDensSparse),
                           island.infill_areas());
    }

    SECTION("top_solid_min_thickness can extend the top shell without extra layer count")
    {
        // The layer height is 1 mm. A 1.5 mm minimum top shell thickness makes
        // the layer below the top solid even though top_solid_layers=1.
        PreparedPerimeterPrint prepared;
        prepare_cube_print(prepared, perimeter_config({
            {"perimeters", "0"},
            {"top_solid_layers", "1"},
            {"bottom_solid_layers", "0"},
            {"top_solid_min_thickness", "1.5"},
            {"bottom_solid_min_thickness", "0"},
            {"solid_over_perimeters", "0"}
        }));
        PrintObject &object = prepared.print.object(0);
        REQUIRE(object.layer_count() > 2);
        const size_t layer_idx = layer_index_for_top(object) - 1;
        const ExPolygon area = rectangle_expolygon(-10., -10., 10., 10.);
        replace_layer_island(object.layer(0), area);
        replace_layer_island(object.layer(layer_idx - 1), area);
        replace_layer_island(object.layer(layer_idx), area);
        replace_layer_island(object.layer(layer_idx + 1), area);
        rebuild_island_overlap_graph(object);

        run_solid_shell_surface_case(prepared);

        const LayerSliceIsland &island = object.layer(layer_idx).island(0);
        require_surface_area_contract(island);
        require_same_union(surface_expolygons_of_type(whole_region_island(island).fill_surfaces(),
                                                      stPosInternal | stDensSolid),
                           island.infill_areas());
    }

    SECTION("bottom_solid_layers promotes the layer directly above a bottom surface")
    {
        // With bottom_solid_layers=2, the first object layer is bottom solid
        // and the next layer must become internal solid as part of the bottom
        // shell.
        PreparedPerimeterPrint prepared;
        prepare_cube_print(prepared, perimeter_config({
            {"perimeters", "0"},
            {"top_solid_layers", "0"},
            {"bottom_solid_layers", "2"},
            {"top_solid_min_thickness", "0"},
            {"bottom_solid_min_thickness", "0"},
            {"solid_over_perimeters", "0"}
        }));
        PrintObject &object = prepared.print.object(0);
        REQUIRE(object.layer_count() > 2);
        const ExPolygon area = rectangle_expolygon(-10., -10., 10., 10.);
        replace_layer_island(object.layer(0), area);
        replace_layer_island(object.layer(1), area);
        replace_layer_island(object.layer(2), area);
        replace_layer_island(object.layer(layer_index_for_top(object)), area);
        rebuild_island_overlap_graph(object);

        run_solid_shell_surface_case(prepared);

        const LayerSliceIsland &island = object.layer(1).island(0);
        require_surface_area_contract(island);
        require_same_union(surface_expolygons_of_type(whole_region_island(island).fill_surfaces(),
                                                      stPosInternal | stDensSolid),
                           island.infill_areas());
    }
}

TEST_CASE("SolidShells refuses to run before typed surfaces exist",
          "[plugins][surface-generation][solid-shells]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Orchestrator &orchestrator = Orchestrator::instance();
    orchestrator.reset_plugin_cancel();
    orchestrator.consume_plugin_messages();

    // SolidShells is a refinement plugin. Without InitialTypedSurfaceBuilder
    // before it, there are no top/bottom anchors to project through the object,
    // so the plugin must fail loudly instead of silently doing nothing useful.
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, perimeter_config({
        {"perimeters", "0"},
        {"top_solid_layers", "2"},
        {"bottom_solid_layers", "2"},
        {"top_solid_min_thickness", "0"},
        {"bottom_solid_min_thickness", "0"},
        {"solid_over_perimeters", "0"}
    }));

    bool stopped = false;
    std::string stop_message;
    try {
        ScopedActivePlugins active({SIMPLE_PERIMETER_GENERATOR, SOLID_SHELLS});
        run_perimeter_and_surface_steps(prepared.print, false);
    } catch (const RuntimeError &error) {
        stopped = true;
        stop_message = error.what();
    }

    CHECK(stopped);
    CHECK(stop_message.find(
              "STEP_SURFACE_GENERATION stopped after plugin 'surface.solid_shells' reported an error during run.") !=
          std::string::npos);

    const std::vector<Orchestrator::PluginMessage> messages = orchestrator.consume_plugin_messages();
    bool found_error = false;
    for (const Orchestrator::PluginMessage &message : messages) {
        if (message.level != Orchestrator::PluginMessageLevel::Error)
            continue;
        if (message.plugin_id != SOLID_SHELLS)
            continue;
        found_error = message.message.find("requires typed fill surfaces") != std::string::npos;
    }

    CHECK(found_error);
    orchestrator.reset_plugin_cancel();
}

TEST_CASE("SolidShells accepts typed surfaces when one exposed side is absent",
          "[plugins][surface-generation][solid-shells]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    struct MissingAnchorCase
    {
        const char *name;
        SurfaceType removed_position;
    };
    const MissingAnchorCase cases[] = {
        { "no top surfaces", stPosTop },
        { "no bottom surfaces", stPosBottom }
    };

    for (const MissingAnchorCase &test_case : cases) {
        CAPTURE(test_case.name);

        Orchestrator &orchestrator = Orchestrator::instance();
        orchestrator.reset_plugin_cancel();
        orchestrator.consume_plugin_messages();

        // A real sliced object can have no top anchors or no bottom anchors in
        // the typed fill-surface stream, for example with sloped geometry or a
        // later plugin that consumes one family. That is still a valid input for
        // SolidShells: it should project the anchors that exist and skip the
        // missing side without reporting a pipeline error.
        PreparedPerimeterPrint prepared;
        prepare_cube_print(prepared, perimeter_config({
            {"perimeters", "1"},
            {"top_solid_layers", "2"},
            {"bottom_solid_layers", "2"},
            {"top_solid_min_thickness", "0"},
            {"bottom_solid_min_thickness", "0"},
            {"solid_over_perimeters", "0"}
        }));

        {
            ScopedActivePlugins active({SIMPLE_PERIMETER_GENERATOR, INITIAL_TYPED_SURFACE_BUILDER});
            run_perimeter_and_surface_steps(prepared.print);
        }

        replace_surface_position_with_sparse_internal(prepared.print, test_case.removed_position);
        run_solid_shells_on_existing_surfaces(prepared.print);
        require_no_solid_shell_prerequisite_error(orchestrator);
    }
}

TEST_CASE("SolidShells keeps fully perimeter-covered shell candidates sparse",
          "[plugins][surface-generation][solid-shells]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    struct PerimeterCoverCase
    {
        int solid_over_perimeters;
        bool expect_projected_area_solid;
    };
    const PerimeterCoverCase cases[] = {
        { 0, true },
        { 1, false }
    };

    for (const PerimeterCoverCase &test_case : cases) {
        CAPTURE(test_case.solid_over_perimeters);

        PreparedPerimeterPrint prepared;
        prepare_cube_print(prepared, perimeter_config({
            {"perimeters", "1"},
            {"top_solid_layers", "2"},
            {"bottom_solid_layers", "0"},
            {"top_solid_min_thickness", "0"},
            {"bottom_solid_min_thickness", "0"},
            {"solid_over_perimeters", std::to_string(test_case.solid_over_perimeters)}
        }));
        PrintObject &object = prepared.print.object(0);
        REQUIRE(object.layer_count() > 2);
        const size_t top_idx = layer_index_for_top(object);
        const size_t current_idx = top_idx - 1;

        // The upper island is deliberately very narrow. One perimeter consumes
        // all of its free infill area, so its projection onto the layer below
        // is entirely "under perimeters". solid_over_perimeters=1 should leave
        // that projected area sparse, while 0 should allow it to become solid.
        const ExPolygon projected = rectangle_expolygon(-0.08, -8., 0.08, 8.);
        replace_layer_island(object.layer(0), rectangle_expolygon(-10., -10., 10., 10.));
        replace_layer_island(object.layer(top_idx), projected);
        replace_layer_island(object.layer(current_idx), rectangle_expolygon(-10., -10., 10., 10.));
        replace_layer_island(object.layer(current_idx - 1), rectangle_expolygon(-10., -10., 10., 10.));
        rebuild_island_overlap_graph(object);

        run_solid_shell_surface_case(prepared);

        const LayerSliceIsland &island = object.layer(current_idx).island(0);
        require_surface_area_contract(island);

        const ExPolygons projected_in_fill = intersection_ex(island.infill_areas(), ExPolygons{projected});
        REQUIRE_FALSE(projected_in_fill.empty());
        const ExPolygons solid = surface_expolygons_of_type(whole_region_island(island).fill_surfaces(),
                                                            stPosInternal | stDensSolid);
        const ExPolygons sparse = surface_expolygons_of_type(whole_region_island(island).fill_surfaces(),
                                                             stPosInternal | stDensSparse);

        if (test_case.expect_projected_area_solid) {
            require_same_union(intersection_ex(solid, projected_in_fill), projected_in_fill);
        } else {
            CHECK(area_sum(intersection_ex(solid, projected_in_fill)) <= area_tolerance());
            require_same_union(intersection_ex(sparse, projected_in_fill), projected_in_fill);
        }
    }
}
