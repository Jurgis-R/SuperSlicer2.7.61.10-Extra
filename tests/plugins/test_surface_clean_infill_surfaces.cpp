#include <catch2/catch.hpp>

#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/internal/LayerAccess.hpp"
#include "libslic3r/Api/internal/LayerIslandAccess.hpp"
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
#include <cstdlib>
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
        // Surface-generation behavior depends on plugin order and active
        // plugin selection. Each test installs the minimal pipeline it needs
        // and restores the global test runtime afterward.
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

const LayerRegionIsland &whole_region_island(const LayerSliceIsland &island)
{
    for (const LayerRegionIsland &region_island : island.regions_islands()) {
        if (!region_island.fill_surfaces().empty())
            return region_island;
    }

    FAIL("No region island with fill surfaces");
    std::abort();
}

DynamicPrintConfig clean_surface_config(std::initializer_list<std::pair<std::string, std::string>> overrides)
{
    DynamicPrintConfig config = perimeter_config({
        {"perimeters", "0"},
        {"top_solid_layers", "0"},
        {"bottom_solid_layers", "0"},
        {"top_solid_min_thickness", "0"},
        {"bottom_solid_min_thickness", "0"},
        {"solid_over_perimeters", "0"},
        {"solid_infill_below_layer_area", "0"},
        {"solid_infill_below_area", "0"},
        {"solid_infill_below_width", "0"}
    });
    for (const std::pair<std::string, std::string> &entry : overrides)
        config.set_deserialize_strict(entry.first, entry.second);
    return config;
}

size_t editable_middle_layer_idx(const PrintObject &object)
{
    REQUIRE(object.layer_count() > 5);
    return layer_index_for_top(object) / 2;
}

void set_region_area(LayerRegion &region, const ExPolygon &area)
{
    ExPolygons &region_slices = ApiInternal::LayerRegionAccess::slices_mutable(region);
    region_slices = ExPolygons{area};
    ApiInternal::LayerRegionAccess::surfaces_mutable(region).set(region_slices, stPosInternal | stDensSparse);
}

void replace_layer_island(Layer &layer, const ExPolygon &area)
{
    // The tests edit sliced layers directly. fill_regions() rebuilds the
    // LayerSliceIsland-to-LayerRegion mapping consumed by the plugin API.
    ApiInternal::LayerAccess::set_islands(layer, ExPolygons{area});
    set_region_area(layer.region(0), area);
    layer.island(0).fill_regions(layer);
}

void rebuild_island_overlap_graph(PrintObject &object)
{
    // Surface type classification depends on cached upper/lower island links.
    // Direct layer edits invalidate those links, so rebuild them before the
    // surface-generation step runs.
    for (Layer &layer : object.layers())
        for (LayerSliceIsland &island : layer.islands()) {
            island.overlaps_above.clear();
            island.overlaps_below.clear();
        }

    for (size_t layer_idx = 1; layer_idx < object.layer_count(); ++layer_idx)
        Layer::build_up_down_graph(object.layer(layer_idx - 1), object.layer(layer_idx));
}

void prepare_middle_layer_island(PreparedPerimeterPrint &prepared)
{
    PrintObject &object = prepared.print.object(0);
    Layer &layer = object.layer(editable_middle_layer_idx(object));
    replace_layer_island(layer, rectangle_expolygon(-10., -10., 10., 10.));
    rebuild_island_overlap_graph(object);
}

void set_middle_infill_areas(PreparedPerimeterPrint &prepared, const ExPolygons &areas)
{
    // Some cases need fill areas that are hard to obtain through perimeter
    // offsets alone. Inject them after perimeter generation so the test can
    // focus on the surface-cleanup contract, then let surface generation build
    // fresh fill surfaces from those areas.
    PrintObject &object = prepared.print.object(0);
    Layer &layer = object.layer(editable_middle_layer_idx(object));
    ExPolygons copy = areas;
    ApiInternal::LayerIslandAccess::set_infill_areas(layer.island(0), std::move(copy));
}

void run_surface_cleanup_pipeline(PreparedPerimeterPrint &prepared)
{
    // Run the real perimeter and surface-generation steps with only the
    // plugins needed by this test. validate_post() checks the important global
    // invariant: surfaces must still be a partition of island.infill_areas().
    ScopedActivePlugins active({SIMPLE_PERIMETER_GENERATOR, INITIAL_TYPED_SURFACE_BUILDER, CLEAN_INFILL_SURFACES});
    Orchestrator &orchestrator = Orchestrator::instance();

    prepare_middle_layer_island(prepared);
    Steps::StepGeneratePerimeter::clear(prepared.print);
    Steps::StepGeneratePerimeter::run_step(orchestrator, prepared.print);
    Steps::StepSurfaceGeneration::clear(prepared.print);
    Steps::StepSurfaceGeneration::run_step(orchestrator, prepared.print);

    std::string validation_error;
    const bool valid_surface_tree = Steps::StepSurfaceGeneration::validate_post(prepared.print, &validation_error);
    INFO("Surface-generation post validation: " << validation_error);
    REQUIRE(valid_surface_tree);
}

void run_surface_cleanup_pipeline_with_middle_infill_areas(PreparedPerimeterPrint &prepared, const ExPolygons &areas)
{
    // This variant lets a test replace the generated infill areas with a
    // controlled shape before InitialTypedSurfaceBuilder creates surfaces.
    ScopedActivePlugins active({SIMPLE_PERIMETER_GENERATOR, INITIAL_TYPED_SURFACE_BUILDER, CLEAN_INFILL_SURFACES});
    Orchestrator &orchestrator = Orchestrator::instance();

    prepare_middle_layer_island(prepared);
    Steps::StepGeneratePerimeter::clear(prepared.print);
    Steps::StepGeneratePerimeter::run_step(orchestrator, prepared.print);
    set_middle_infill_areas(prepared, areas);
    Steps::StepSurfaceGeneration::clear(prepared.print);
    Steps::StepSurfaceGeneration::run_step(orchestrator, prepared.print);

    std::string validation_error;
    const bool valid_surface_tree = Steps::StepSurfaceGeneration::validate_post(prepared.print, &validation_error);
    INFO("Surface-generation post validation: " << validation_error);
    REQUIRE(valid_surface_tree);
}

const SurfaceCollection &middle_surfaces(const PreparedPerimeterPrint &prepared)
{
    const PrintObject &object = prepared.print.object(0);
    const LayerSliceIsland &island = object.layer(editable_middle_layer_idx(object)).island(0);
    return whole_region_island(island).fill_surfaces();
}

} // namespace

TEST_CASE("CleanInfillSurfaces promotes sparse surfaces when the layer area is below threshold",
          "[plugins][surface-generation][clean-infill-surfaces]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    // The whole editable layer is small compared to
    // solid_infill_below_layer_area. The cleanup plugin should therefore turn
    // every sparse surface on that layer into solid while preserving the area.
    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, clean_surface_config({{"solid_infill_below_layer_area", "100000"}}));

    run_surface_cleanup_pipeline(prepared);

    const SurfaceCollection &surfaces = middle_surfaces(prepared);
    const ExPolygons sparse = surface_expolygons_of_type(surfaces, stPosInternal | stDensSparse);
    const ExPolygons solid = surface_expolygons_of_type(surfaces, stPosInternal | stDensSolid);

    CHECK(area_sum(sparse) <= area_tolerance());
    REQUIRE(!solid.empty());
}

TEST_CASE("CleanInfillSurfaces promotes only infill areas below solid_infill_below_area",
          "[plugins][surface-generation][clean-infill-surfaces]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    // The layer has two independent infill areas. Only the small one is below
    // solid_infill_below_area, so it becomes solid and the larger area remains
    // sparse. This checks the per-ExPolygon behavior, not the whole-layer rule.
    const ExPolygon small = rectangle_expolygon(-9., -9., -7., -7.);
    const ExPolygon large = rectangle_expolygon(-5., -5., 9., 9.);

    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, clean_surface_config({{"solid_infill_below_area", "10"}}));
    run_surface_cleanup_pipeline_with_middle_infill_areas(prepared, ExPolygons{small, large});

    const SurfaceCollection &surfaces = middle_surfaces(prepared);
    require_same_union(surface_expolygons_of_type(surfaces, stPosInternal | stDensSolid), ExPolygons{small});
    require_same_union(surface_expolygons_of_type(surfaces, stPosInternal | stDensSparse), ExPolygons{large});
}

TEST_CASE("CleanInfillSurfaces turns sparse sections thinner than solid_infill_below_width into solid",
          "[plugins][surface-generation][clean-infill-surfaces]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    // A 0.4 mm wide fill island cannot survive the -width/2 then +width/2
    // sparse cleanup when solid_infill_below_width is 1 mm. The removed sparse
    // strip must be rebuilt as solid instead of disappearing.
    const ExPolygon thin = rectangle_expolygon(-9., -0.2, 9., 0.2);

    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, clean_surface_config({{"solid_infill_below_width", "1"}}));
    run_surface_cleanup_pipeline_with_middle_infill_areas(prepared, ExPolygons{thin});

    const SurfaceCollection &surfaces = middle_surfaces(prepared);
    CHECK(area_sum(surface_expolygons_of_type(surfaces, stPosInternal | stDensSparse)) <= area_tolerance());
    require_same_union(surface_expolygons_of_type(surfaces, stPosInternal | stDensSolid), ExPolygons{thin});
}

TEST_CASE("CleanInfillSurfaces rebuilds same-type fragments as merged surfaces",
          "[plugins][surface-generation][clean-infill-surfaces]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    // The two infill areas share a border and have the same surface type. The
    // final normalization pass should union them and publish one compact sparse
    // surface, proving cleanup does not leave accidental same-type splits.
    const ExPolygon left = rectangle_expolygon(-8., -5., 0., 5.);
    const ExPolygon right = rectangle_expolygon(0., -5., 8., 5.);
    const ExPolygons expected = union_ex(ExPolygons{left, right});

    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, clean_surface_config({}));
    run_surface_cleanup_pipeline_with_middle_infill_areas(prepared, ExPolygons{left, right});

    const SurfaceCollection &surfaces = middle_surfaces(prepared);
    const ExPolygons sparse = surface_expolygons_of_type(surfaces, stPosInternal | stDensSparse);
    REQUIRE(sparse.size() == expected.size());
    require_same_union(sparse, expected);
}
