#include <catch2/catch.hpp>

#include <cmath>
#include <initializer_list>
#include <memory>
#include <string>
#include <vector>

#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Fill/FillBase.hpp"
#include "libslic3r/Fill/FillRectilinear.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Plugins/Infill/Rectilinear/RectilinearFills.hpp"
#include "libslic3r/Plugins/Infill/Rectilinear/RectilinearWithPerimeter.hpp"
#include "libslic3r/Plugins/Infill/PatternPluginHelpers.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepGenerateInfill.hpp"
#include "libslic3r/Steps/StepGeneratePerimeter.hpp"
#include "libslic3r/Steps/StepSurfaceGeneration.hpp"
#include "libslic3r/Surface.hpp"

/*
Rectilinear-with-perimeter plugin tests
=======================================

The direct parity test compares the new storage-backed extrusion tree with the
legacy FillWithPerimeter tree on the same surface and recipe. The pipeline test
then activates the registered plugin and verifies that STEP_INFILL publishes
the perimeter-aware tree through the normal host path.
*/

using namespace Slic3r;
using namespace Slic3r::Test::PerimeterPluginTests;

namespace {

namespace recti = slic3r_api::Infill::RectilinearPatternsPlugin::recti;
namespace rect_fills = slic3r_api::Infill::RectilinearPatternsPlugin::fills;

ExPolygon make_surface(const std::string &name)
{
    if (name == "hole") {
        ExPolygon out(Polygon({ Point(-scale_i(20.), -scale_i(15.)), Point(scale_i(20.), -scale_i(15.)),
                                Point(scale_i(20.), scale_i(15.)), Point(-scale_i(20.), scale_i(15.)) }));
        out.holes.push_back(Polygon({ Point(-scale_i(6.), -scale_i(4.)), Point(-scale_i(6.), scale_i(4.)),
                                      Point(scale_i(6.), scale_i(4.)), Point(scale_i(6.), -scale_i(4.)) }));
        return out;
    }
    if (name == "concave")
        return ExPolygon(Polygon({ Point(0, 0), Point(scale_i(30.), 0), Point(scale_i(30.), scale_i(10.)),
                                   Point(scale_i(10.), scale_i(10.)), Point(scale_i(10.), scale_i(25.)),
                                   Point(0, scale_i(25.)) }));
    if (name == "two_islands")
        return ExPolygon(Polygon({ Point(0, 0), Point(scale_i(20.), 0), Point(scale_i(20.), scale_i(9.8)),
                                   Point(scale_i(30.), scale_i(9.8)), Point(scale_i(30.), 0),
                                   Point(scale_i(50.), 0), Point(scale_i(50.), scale_i(20.)),
                                   Point(scale_i(30.), scale_i(20.)), Point(scale_i(30.), scale_i(10.2)),
                                   Point(scale_i(20.), scale_i(10.2)), Point(scale_i(20.), scale_i(20.)),
                                   Point(0, scale_i(20.)) }));
    if (name == "acute")
        return ExPolygon(Polygon({ Point(0, 0), Point(scale_i(40.), 0), Point(scale_i(20.), scale_i(60.)) }));
    return ExPolygon(Polygon({ Point(0, 0), Point(scale_i(40.), 0), Point(scale_i(40.), scale_i(30.)),
                               Point(0, scale_i(30.)) }));
}

raw_infill_pattern_params make_raw_params(const Flow &flow, float density, double angle)
{
    raw_infill_pattern_params params = {};
    params.surface_type = RAW_SURFACE_TYPE_POS_INTERNAL | RAW_SURFACE_TYPE_DENS_SPARSE;
    params.extrusion_role = RAW_EXTRUSION_ROLE_INTERNAL_INFILL;
    params.flow.width = flow.scaled_width();
    params.flow.spacing = flow.scaled_spacing();
    params.flow.height = flow.scaled_height();
    params.flow.nozzle_diameter = scale_i(flow.nozzle_diameter());
    params.flow.spacing_ratio = flow.spacing_ratio();
    params.flow.mm3_per_mm = flow.mm3_per_mm();
    params.density = density;
    params.flow_mult = 0.85f;
    params.connection = C_INFILL_CONNECTION_CONNECTED;
    params.infill_flags = C_INFILL_FLAG_CAN_ANGLE_CROSS;
    params.spacing = 0.8;
    params.angle = angle;
    params.bridge_angle = -1.;
    params.layer_id = 3;
    params.z = 0.2;
    params.overlap = 0.;
    params.fill_resolution = scale_i(0.0125);
    params.anchor_length = 1000.f;
    params.anchor_length_max = 1000.f;
    return params;
}

std::unique_ptr<Fill> make_core_filler(const ExPolygon &surface,
                                       const raw_infill_pattern_params &raw,
                                       const Flow &flow)
{
    std::unique_ptr<Fill> filler(Fill::new_from_type(ipRectiWithPerimeter));
    REQUIRE(filler != nullptr);
    filler->set_bounding_box(get_extents(surface));
    filler->layer_id = raw.layer_id;
    filler->z = raw.z;
    filler->angle = float(raw.angle);
    filler->can_angle_cross = true;
    filler->link_max_length = raw.link_max_length;
    filler->loop_clipping = raw.loop_clipping;
    filler->overlap = raw.overlap;
    FillParams params;
    params.density = raw.density;
    params.flow_mult = raw.flow_mult;
    params.connection = InfillConnection::icConnected;
    params.bridge_offset = raw.bridge_offset;
    params.fill_resolution = raw.fill_resolution;
    params.anchor_length = raw.anchor_length;
    params.anchor_length_max = raw.anchor_length_max;
    params.dont_adjust = true;
    params.role = ExtrusionRole::InternalInfill;
    params.flow = flow;
    filler->init_spacing(raw.spacing, params);
    return filler;
}

void compare_tree(const ExtrusionEntity &core,
                  const slic3r_api::ExtrusionEntity &ported,
                  const std::string &path)
{
    INFO(path);
    INFO("core children " << core.child_count() << " ported children " << ported.child_count());
    INFO("core sortable " << core.can_sort() << " ported sortable " << ported.sortable());
    INFO("core reversible " << core.can_reverse() << " ported reversible " << ported.reversible());
    REQUIRE(core.is_leaf() == ported.is_leaf());
    REQUIRE(core.child_count() == ported.child_count());
    CHECK(core.can_sort() == ported.sortable());
    CHECK(core.can_reverse() == ported.reversible());

    if (!core.is_leaf()) {
        for (size_t idx = 0; idx < core.child_count(); ++idx)
            compare_tree(core.child(idx), ported.child(uint32_t(idx)), path + "." + std::to_string(idx));
        return;
    }

    Points core_points;
    core.collect_points(core_points);
    const std::vector<c_point> ported_points = ported.points();
    REQUIRE(core_points.size() == ported_points.size());
    for (size_t idx = 0; idx < core_points.size(); ++idx) {
        CHECK(core_points[idx].x() == ported_points[idx].x);
        CHECK(core_points[idx].y() == ported_points[idx].y);
    }

    const ExtrusionAttributes *core_attributes = core.get_property<ExtrusionAttributes>();
    const slic3r_api::EPropertyAttributes *ported_attributes = ported.get(slic3r_api::EPropertyAttributes::key);
    REQUIRE(core_attributes != nullptr);
    REQUIRE(ported_attributes != nullptr);
    CHECK(int32_t(core_attributes->role) == ported_attributes->extrusion_role());
    CHECK(core_attributes->mm3_per_mm ==
          Approx(ported_attributes->c_extrusion_property_attributes::mm3_per_mm).epsilon(1e-6));
    CHECK(core_attributes->width ==
          Approx(ported_attributes->c_extrusion_property_attributes::width).epsilon(1e-6));
    CHECK(core_attributes->height ==
          Approx(ported_attributes->c_extrusion_property_attributes::height).epsilon(1e-6));
}

// Match split outer islands by their first perimeter point because Clipper may
// enumerate equivalent islands in opposite orders across the two APIs.
void compare_split_tree(const ExtrusionEntity &core,
                        const slic3r_api::ExtrusionEntity &ported)
{
    INFO("split outer islands");
    REQUIRE_FALSE(core.is_leaf());
    REQUIRE_FALSE(ported.is_leaf());
    REQUIRE(core.child_count() == ported.child_count());
    CHECK(core.can_sort() == ported.sortable());
    CHECK(core.can_reverse() == ported.reversible());

    std::vector<bool> matched(ported.child_count(), false);
    for (size_t core_idx = 0; core_idx < core.child_count(); ++core_idx) {
        Points core_points;
        core.child(core_idx).collect_points(core_points);
        REQUIRE_FALSE(core_points.empty());
        const Point core_front = core_points.front();
        size_t ported_idx = ported.child_count();
        for (size_t candidate = 0; candidate < ported.child_count(); ++candidate) {
            if (matched[candidate])
                continue;
            const std::vector<c_point> ported_points = ported.child(uint32_t(candidate)).collect_points();
            REQUIRE_FALSE(ported_points.empty());
            const c_point ported_front = ported_points.front();
            if (core_front.x() == ported_front.x && core_front.y() == ported_front.y) {
                ported_idx = candidate;
                break;
            }
        }
        REQUIRE(ported_idx < ported.child_count());
        matched[ported_idx] = true;
        compare_tree(core.child(core_idx), ported.child(uint32_t(ported_idx)),
                     "island." + std::to_string(core_idx));
    }
}

size_t count_api_leaves(const slic3r_api::ExtrusionEntity &entity)
{
    if (entity.is_leaf())
        return entity.empty() ? 0 : 1;
    size_t count = 0;
    for (const slic3r_api::ExtrusionEntity child : entity.children())
        count += count_api_leaves(child);
    return count;
}

size_t count_api_loops(const slic3r_api::ExtrusionEntity &entity)
{
    size_t count = entity.is_loop() ? 1 : 0;
    for (const slic3r_api::ExtrusionEntity child : entity.children())
        count += count_api_loops(child);
    return count;
}

class ScopedActivePlugins
{
public:
    explicit ScopedActivePlugins(std::initializer_list<const char *> plugin_ids) :
        m_orchestrator(Orchestrator::instance())
    {
        m_previous_active_plugins.reserve(m_orchestrator.active_plugins().size());
        for (Plugin *plugin : m_orchestrator.active_plugins())
            m_previous_active_plugins.push_back(plugin);

        m_orchestrator.clear_active_plugins();
        for (const char *plugin_id : plugin_ids)
            REQUIRE(m_orchestrator.set_plugin_active(plugin_id, true));
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

DynamicPrintConfig pipeline_config()
{
    return perimeter_config({
        { "fill_pattern", "rectilinear" },
        { "fill_density", "20%" },
        { "top_solid_layers", "0" },
        { "bottom_solid_layers", "0" },
        { "solid_infill_every_layers", "0" },
        { "infill_every_layers", "1" }
    });
}

void run_until_infill(PreparedPerimeterPrint &prepared)
{
    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepGeneratePerimeter::clean_and_prepare(prepared.print);
    Steps::StepGeneratePerimeter::run_step(orchestrator, prepared.print);
    Steps::StepSurfaceGeneration::clean_and_prepare(prepared.print);
    Steps::StepSurfaceGeneration::run_step(orchestrator, prepared.print);
    Steps::StepGenerateInfill::clean_and_prepare(prepared.print);
    Steps::StepGenerateInfill::run_step(orchestrator, prepared.print);
}

} // namespace

TEST_CASE("Rectilinear with perimeter matches the legacy extrusion tree",
          "[plugins][infill][rectilinear]")
{
    const Flow flow = Flow::new_from_spacing(0.8f, 0.4f, 0.2f, 0.5f);
    struct ParityRecipe
    {
        float density;
        double angle;
    };
    const std::vector<ParityRecipe> recipes = {{ 0.55f, M_PI / 6. }, { 1.f, -M_PI / 4. }};
    const std::vector<std::string> surfaces = { "square", "hole", "concave", "two_islands", "acute" };

    for (const ParityRecipe &recipe : recipes) {
        INFO("density " << recipe.density << " angle " << recipe.angle);
        for (const std::string &surface_name : surfaces) {
            INFO(surface_name);
            const ExPolygon surface = make_surface(surface_name);
            const raw_infill_pattern_params raw = make_raw_params(flow, recipe.density, recipe.angle);

            std::unique_ptr<Fill> core_filler = make_core_filler(surface, raw, flow);
            FillParams core_params;
            core_params.density = raw.density;
            core_params.flow_mult = raw.flow_mult;
            core_params.connection = InfillConnection::icConnected;
            core_params.bridge_offset = raw.bridge_offset;
            core_params.fill_resolution = raw.fill_resolution;
            core_params.anchor_length = raw.anchor_length;
            core_params.anchor_length_max = raw.anchor_length_max;
            core_params.dont_adjust = true;
            core_params.role = ExtrusionRole::InternalInfill;
            core_params.flow = flow;
            Surface core_surface(stPosInternal | stDensSparse, surface);
            ExtrusionEntitiesPtr core_output;
            core_filler->fill_surface_extrusion(&core_surface, core_params, core_output);
            REQUIRE(core_output.size() == 1);
            if (surface_name == "two_islands")
                REQUIRE(core_output.front()->child_count() == 2);

            PluginStorage plugin_storage;
            storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
            rect_fills::FillRectilinear ported_filler;
            ported_filler.plugin_storage = storage;
            const BoundingBox bbox = get_extents(surface);
            rect_fills::BoundingBox ported_bbox;
            ported_bbox.min = rect_fills::Point(bbox.min.x(), bbox.min.y());
            ported_bbox.max = rect_fills::Point(bbox.max.x(), bbox.max.y());
            ported_bbox.defined = true;
            ported_filler.bounding_box = ported_bbox;
            ported_filler.layer_id = raw.layer_id;
            ported_filler.z = raw.z;
            ported_filler.angle = float(raw.angle);
            ported_filler.can_angle_cross = true;
            ported_filler.init_spacing(raw.spacing, raw);

            const slic3r_api::ExPolygon surface_view(
                reinterpret_cast<const expolygon_handle *>(&surface));
            slic3r_api::StoredExtrusionEntity ported_root =
                recti::RectiWithPerimeter::fill_with_perimeter(storage, surface_view, ported_filler, raw);
            REQUIRE_FALSE(ported_root.empty());
            if (surface_name == "two_islands")
                compare_split_tree(*core_output.front(), ported_root.readonly());
            else
                compare_tree(*core_output.front(), ported_root.readonly(), "root");
        }
    }
}

TEST_CASE("Rectilinear with perimeter registers as an infill pattern plugin",
          "[plugins][infill][rectilinear]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const Plugin *plugin = Orchestrator::instance().get_plugin("rectiwithperimeter");
    REQUIRE(plugin != nullptr);
    CHECK(plugin->get_step() == INFILL_PATTERN);
}

TEST_CASE("Rectilinear with perimeter generates extrusion through STEP_INFILL",
          "[plugins][infill][rectilinear]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    PreparedPerimeterPrint prepared;
    prepare_cube_print(prepared, pipeline_config());
    // The pattern intentionally contributes no config list. Activating it as
    // the only available provider makes STEP_INFILL exercise the host fallback
    // while keeping the stored test recipe a valid standard pattern value.
    ScopedActivePlugins active({
        SIMPLE_PERIMETER_GENERATOR,
        INITIAL_TYPED_SURFACE_BUILDER,
        "infill.generator.default",
        "rectiwithperimeter"
    });
    run_until_infill(prepared);

    size_t roots = 0;
    for (const PrintObject &object : prepared.print.objects())
        for (const Layer &layer : object.layers())
            for (const LayerSliceIsland &island : layer.islands())
                for (const LayerRegionIsland &region_island : island.regions_islands()) {
                    if (!region_island.has_extrusion(LayerRegionIsland::INFILLS))
                        continue;
                    for (const ExtrusionEntity *entity : region_island.extrusion(LayerRegionIsland::INFILLS).entities()) {
                        if (entity == nullptr)
                            continue;
                        ++roots;
                        const slic3r_api::ExtrusionEntity view(
                            reinterpret_cast<const extrusion_entity_handle *>(entity));
                        CHECK(count_api_leaves(view) > 1);
                        CHECK(count_api_loops(view) > 0);
                    }
                }

    CHECK(roots > 0);
}
