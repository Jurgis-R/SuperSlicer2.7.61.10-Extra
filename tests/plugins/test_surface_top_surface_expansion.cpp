#include <catch2/catch.hpp>

#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/internal/LayerAccess.hpp"
#include "libslic3r/Api/internal/LayerRegionAccess.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepGeneratePerimeter.hpp"
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
        // Tests temporarily replace the global active plugin set. Restoring it
        // on scope exit keeps independent test files from affecting each other.
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
    // The fixtures use one region set. Keep the fallback to get useful failure
    // output if the surface-generation step unexpectedly creates a split.
    const LayerRegionIsland *fallback = nullptr;
    for (const LayerRegionIsland &region_island : island.regions_islands()) {
        if (!region_island.fill_surfaces().empty() && fallback == nullptr)
            fallback = &region_island;
        if (region_island.regions() == island.regions() && !region_island.fill_surfaces().empty())
            return region_island;
    }

    REQUIRE(fallback != nullptr);
    return *fallback;
}

void require_surface_area_contract(const LayerSliceIsland &island)
{
    // The plugin may change the classification boundaries, but it must not
    // create overlaps or lose fillable area. Later infill code assumes every
    // LayerRegionIsland fill-surface collection is a partition of infill_areas.
    const SurfaceCollection &surfaces = whole_region_island(island).fill_surfaces();
    require_no_positive_overlap(surfaces);
    require_same_union(surface_expolygons(surfaces), island.infill_areas());
}

void set_region_area(LayerRegion &region, const ExPolygon &area)
{
    ExPolygons &region_slices = ApiInternal::LayerRegionAccess::slices_mutable(region);
    region_slices = ExPolygons{area};
    ApiInternal::LayerRegionAccess::surfaces_mutable(region).set(region_slices, stPosInternal | stDensSparse);
}

void replace_layer_island(Layer &layer, const ExPolygon &area)
{
    // The tests build small synthetic ledges by editing the already-sliced
    // layers. fill_regions() must be called afterward so each island has the
    // region membership expected by the plugin API.
    ApiInternal::LayerAccess::set_islands(layer, ExPolygons{area});
    set_region_area(layer.region(0), area);
    layer.island(0).fill_regions(layer);
}

void rebuild_island_overlap_graph(PrintObject &object)
{
    // LayerIsland upper/lower links are cached. Direct layer edits invalidate
    // them, so rebuild the graph before running surface-generation plugins.
    for (Layer &layer : object.layers())
        for (LayerSliceIsland &island : layer.islands()) {
            island.overlaps_above.clear();
            island.overlaps_below.clear();
        }

    for (size_t layer_idx = 1; layer_idx < object.layer_count(); ++layer_idx)
        Layer::build_up_down_graph(object.layer(layer_idx - 1), object.layer(layer_idx));
}

void run_surface_pipeline(PreparedPerimeterPrint &prepared, const bool include_top_surface_expansion = true)
{
    // Most tests run the complete surface pipeline. A few sections deliberately
    // disable the new plugin to document the baseline problem it fixes.
    const std::initializer_list<const char *> active_plugins_with_expansion = {
        SIMPLE_PERIMETER_GENERATOR,
        INITIAL_TYPED_SURFACE_BUILDER,
        SOLID_SHELLS,
        TOP_SURFACE_EXPANSION
    };
    const std::initializer_list<const char *> active_plugins_without_expansion = {
        SIMPLE_PERIMETER_GENERATOR,
        INITIAL_TYPED_SURFACE_BUILDER,
        SOLID_SHELLS
    };
    ScopedActivePlugins active(include_top_surface_expansion ? active_plugins_with_expansion :
                                                              active_plugins_without_expansion);

    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepGeneratePerimeter::clear(prepared.print);
    Steps::StepGeneratePerimeter::run_step(orchestrator, prepared.print);
    Steps::StepSurfaceGeneration::clear(prepared.print);
    Steps::StepSurfaceGeneration::run_step(orchestrator, prepared.print);

    std::string validation_error;
    const bool valid_surface_tree = Steps::StepSurfaceGeneration::validate_post(prepared.print, &validation_error);
    INFO("Surface-generation post validation: " << validation_error);
    REQUIRE(valid_surface_tree);
}

DynamicPrintConfig top_expansion_config(std::initializer_list<std::pair<std::string, std::string>> overrides)
{
    DynamicPrintConfig config = perimeter_config({
        {"perimeters", "0"},
        {"external_infill_margin", "1"},
        {"top_solid_layers", "1"},
        {"top_solid_min_thickness", "0"},
        {"bottom_solid_layers", "0"},
        {"bottom_solid_min_thickness", "0"},
        {"solid_over_perimeters", "0"}
    });
    for (const std::pair<std::string, std::string> &entry : overrides)
        config.set_deserialize_strict(entry.first, entry.second);
    return config;
}

struct LedgeFixture
{
    PreparedPerimeterPrint prepared;
    ExPolygon full = rectangle_expolygon(-10., -10., 10., 10.);
    ExPolygon cover = rectangle_expolygon(-10., -10., 0., 10.);
    ExPolygons top_area;
    ExPolygons expanded_top_area;
    size_t target_idx = 0;
    size_t source_idx = 0;
};

void prepare_ledge_fixture(LedgeFixture &fixture,
                           const DynamicPrintConfig &config,
                           const ExPolygon &cover,
                           const double margin_mm)
{
    fixture.cover = cover;
    prepare_cube_print(fixture.prepared, config);

    PrintObject &object = fixture.prepared.print.object(0);
    REQUIRE(object.layer_count() > 5);
    const size_t top_idx = layer_index_for_top(object);
    fixture.source_idx = top_idx - 3;
    fixture.target_idx = fixture.source_idx - 1;

    // The source layer is a full island, but the layer above covers only the
    // left half. The source layer therefore has a right-half top surface, while
    // the target layer below is fully covered by the source and starts as plain
    // internal sparse infill. Every layer above the source repeats the same
    // left-half cover, so the cover itself does not become another nearby top
    // surface that would affect the minimum-thickness test.
    replace_layer_island(object.layer(0), fixture.full);
    replace_layer_island(object.layer(fixture.target_idx - 1), fixture.full);
    replace_layer_island(object.layer(fixture.target_idx), fixture.full);
    replace_layer_island(object.layer(fixture.source_idx), fixture.full);
    for (size_t layer_idx = fixture.source_idx + 1; layer_idx <= top_idx; ++layer_idx)
        replace_layer_island(object.layer(layer_idx), fixture.cover);
    rebuild_island_overlap_graph(object);

    fixture.top_area = diff_ex(ExPolygons{fixture.full}, ExPolygons{fixture.cover});
    fixture.expanded_top_area = intersection_ex(offset_ex(fixture.top_area, double(scale_i(margin_mm))),
                                                ExPolygons{fixture.full});
}

void prepare_ledge_fixture(LedgeFixture &fixture, const DynamicPrintConfig &config)
{
    prepare_ledge_fixture(fixture, config, fixture.cover, 1.);
}

} // namespace

TEST_CASE("TopSurfaceExpansion enlarges top surfaces without breaking the surface partition",
          "[plugins][surface-generation][top-surface-expansion]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    // The source layer has a top surface on the right half only. The plugin
    // should expand that top surface by external_infill_margin, keep the
    // original right-half top area inside the expanded result, and subtract the
    // expanded top from the sibling internal surfaces.
    LedgeFixture fixture;
    prepare_ledge_fixture(fixture, top_expansion_config({}));
    PrintObject &object = fixture.prepared.print.object(0);

    run_surface_pipeline(fixture.prepared);

    const LayerSliceIsland &source_island = object.layer(fixture.source_idx).island(0);
    const SurfaceCollection &surfaces = whole_region_island(source_island).fill_surfaces();
    const ExPolygons top = surface_expolygons_of_type(surfaces, stPosTop | stDensSolid);
    const ExPolygons sparse = surface_expolygons_of_type(surfaces, stPosInternal | stDensSparse);

    require_surface_area_contract(source_island);
    require_same_union(top, fixture.expanded_top_area);
    REQUIRE(area_sum(diff_ex(fixture.top_area, top)) <= area_tolerance());
    require_same_union(sparse, diff_ex(source_island.infill_areas(), fixture.expanded_top_area));
}

TEST_CASE("TopSurfaceExpansion carries the top margin into lower top shell layers",
          "[plugins][surface-generation][top-surface-expansion]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    SECTION("top_solid_layers transmits the margin to the layer directly below")
    {
        // top_solid_layers=2 means the visible source top surface and one
        // layer below it are part of the top shell. SolidShells already makes
        // the exact right-half top projection solid; TopSurfaceExpansion must
        // add the external_infill_margin ring around it.
        LedgeFixture fixture;
        prepare_ledge_fixture(fixture, top_expansion_config({
            {"top_solid_layers", "2"},
            {"top_solid_min_thickness", "0"}
        }));
        PrintObject &object = fixture.prepared.print.object(0);

        run_surface_pipeline(fixture.prepared);

        const LayerSliceIsland &target_island = object.layer(fixture.target_idx).island(0);
        const SurfaceCollection &surfaces = whole_region_island(target_island).fill_surfaces();
        const ExPolygons solid = surface_expolygons_of_type(surfaces, stPosInternal | stDensSolid);
        const ExPolygons sparse = surface_expolygons_of_type(surfaces, stPosInternal | stDensSparse);

        require_surface_area_contract(target_island);
        require_same_union(solid, fixture.expanded_top_area);
        require_same_union(sparse, diff_ex(target_island.infill_areas(), fixture.expanded_top_area));
    }

    SECTION("top_solid_layers=1 leaves the lower layer unchanged")
    {
        // A value of 1 only affects the layer that owns the visible top
        // surface. The layer below is not part of the top shell, so the margin
        // must not create internal solid surfaces there.
        LedgeFixture fixture;
        prepare_ledge_fixture(fixture, top_expansion_config({
            {"top_solid_layers", "1"},
            {"top_solid_min_thickness", "0"}
        }));
        PrintObject &object = fixture.prepared.print.object(0);

        run_surface_pipeline(fixture.prepared);

        const LayerSliceIsland &target_island = object.layer(fixture.target_idx).island(0);
        const SurfaceCollection &surfaces = whole_region_island(target_island).fill_surfaces();
        const ExPolygons solid = surface_expolygons_of_type(surfaces, stPosInternal | stDensSolid);
        const ExPolygons sparse = surface_expolygons_of_type(surfaces, stPosInternal | stDensSparse);

        require_surface_area_contract(target_island);
        CHECK(area_sum(solid) <= area_tolerance());
        require_same_union(sparse, target_island.infill_areas());
    }

    SECTION("top_solid_min_thickness can transmit the margin without extra layer count")
    {
        // The fixture uses 1 mm layers. With top_solid_layers=1 the lower layer
        // would normally remain sparse, but a 1.5 mm minimum top shell thickness
        // includes that layer and should carry the expanded margin into it.
        LedgeFixture fixture;
        const ExPolygon small_cube_footprint = rectangle_expolygon(-2., -2., 2., 2.);
        prepare_ledge_fixture(fixture,
                              top_expansion_config({
                                  {"external_infill_margin", "3"},
                                  {"top_solid_layers", "1"},
                                  {"top_solid_min_thickness", "1.5"}
                              }),
                              small_cube_footprint,
                              3.);
        PrintObject &object = fixture.prepared.print.object(0);

        run_surface_pipeline(fixture.prepared);

        const LayerSliceIsland &target_island = object.layer(fixture.target_idx).island(0);
        const SurfaceCollection &surfaces = whole_region_island(target_island).fill_surfaces();
        const ExPolygons solid = surface_expolygons_of_type(surfaces, stPosInternal | stDensSolid);
        const ExPolygons sparse = surface_expolygons_of_type(surfaces, stPosInternal | stDensSparse);

        require_surface_area_contract(target_island);
        require_same_union(solid, fixture.expanded_top_area);
        require_same_union(sparse, diff_ex(target_island.infill_areas(), fixture.expanded_top_area));
    }
}

TEST_CASE("TopSurfaceExpansion fills the big-cube top shell around a small cube",
          "[plugins][surface-generation][top-surface-expansion]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    const ExPolygon small_cube_footprint = rectangle_expolygon(-2., -2., 2., 2.);

    SECTION("without expansion the small cube leaves a sparse island center")
    {
        // The edited layer stack represents a small cube standing on a larger
        // cube. The top surface of the large cube is the ring around the small
        // cube footprint. SolidShells projects only that ring downward, so the
        // center under the small cube remains sparse without this plugin.
        LedgeFixture fixture;
        prepare_ledge_fixture(fixture,
                              top_expansion_config({
                                  {"external_infill_margin", "3"},
                                  {"top_solid_layers", "2"},
                                  {"top_solid_min_thickness", "0"}
                              }),
                              small_cube_footprint,
                              3.);
        PrintObject &object = fixture.prepared.print.object(0);

        run_surface_pipeline(fixture.prepared, false);

        const LayerSliceIsland &target_island = object.layer(fixture.target_idx).island(0);
        const SurfaceCollection &surfaces = whole_region_island(target_island).fill_surfaces();
        const ExPolygons solid = surface_expolygons_of_type(surfaces, stPosInternal | stDensSolid);
        const ExPolygons sparse = surface_expolygons_of_type(surfaces, stPosInternal | stDensSparse);

        require_surface_area_contract(target_island);
        require_same_union(solid, fixture.top_area);
        require_same_union(sparse, ExPolygons{small_cube_footprint});
    }

    SECTION("a large enough external_infill_margin makes every shell layer fully solid")
    {
        // With a 3 mm margin, the ring top surface of the large cube expands
        // across the whole small-cube footprint. The top layer of the large
        // cube therefore becomes the whole island, and the propagated top shell
        // below it also becomes entirely solid.
        LedgeFixture fixture;
        prepare_ledge_fixture(fixture,
                              top_expansion_config({
                                  {"external_infill_margin", "3"},
                                  {"top_solid_layers", "2"},
                                  {"top_solid_min_thickness", "0"}
                              }),
                              small_cube_footprint,
                              3.);
        PrintObject &object = fixture.prepared.print.object(0);

        run_surface_pipeline(fixture.prepared);

        const LayerSliceIsland &source_island = object.layer(fixture.source_idx).island(0);
        const SurfaceCollection &source_surfaces = whole_region_island(source_island).fill_surfaces();
        const ExPolygons source_top = surface_expolygons_of_type(source_surfaces, stPosTop | stDensSolid);
        require_surface_area_contract(source_island);
        require_same_union(source_top, source_island.infill_areas());

        const LayerSliceIsland &target_island = object.layer(fixture.target_idx).island(0);
        const SurfaceCollection &target_surfaces = whole_region_island(target_island).fill_surfaces();
        const ExPolygons target_solid = surface_expolygons_of_type(target_surfaces, stPosInternal | stDensSolid);
        const ExPolygons target_sparse = surface_expolygons_of_type(target_surfaces, stPosInternal | stDensSparse);

        require_surface_area_contract(target_island);
        require_same_union(target_solid, target_island.infill_areas());
        CHECK(area_sum(target_sparse) <= area_tolerance());
    }
}
