/* The Common support library rewrites, on the plugin API, values the native
   support code computes: GridSupportParams and the toolpath PathParams, the
   enum choices of the support settings, and FillWithPerimeter. These tests
   compare each one with the native code on the same inputs.

   - SupportParams runs on the configurations of the legacy-comparison harness
     (support_grid_test_helpers.hpp) and a few variants, after slicing.
   - The enum helpers run on every value of the native enum lists.
   - The perimeter-wrapped fill calls its inner pattern through the
     STEP_SUPPORT_TOOLPATH pattern service, as a toolpath plugin does, and its
     extrusion tree is compared node by node with the native one. The honeycomb
     plugin leaves its clipped pieces unconnected where the native filler joins them,
     so for it only the perimeters are compared and the inner fill is measured.

   Where the API keeps a native float as a scaled integer (flow width and
   spacing in c_flow), the derived values get a tolerance; each check says so. */
#include "support_grid_test_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "libslic3r/Api/host/ApiHostUtils.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_support_toolpath.h"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Fill/FillBase.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Plugins/Support/Common/PerimeterFill.hpp"
#include "libslic3r/Plugins/Support/Common/SupportParams.hpp"
#include "libslic3r/Plugins/Support/Common/SupportSettings.hpp"
#include "libslic3r/Plugins/Support/Grid/GridSupportParams.hpp"
#include "libslic3r/Steps/InfillPatternService.hpp"

using namespace Slic3r;
namespace SG = Slic3r::Test::SupportGrid;
namespace Common = slic3r_api::Support::Common;

namespace {

// One harness configuration with extra overrides.
struct ParamsCase
{
    const char *style;
    bool soluble;
    int interfaces;
    bool same_tool;
    std::vector<std::pair<std::string, std::string>> overrides;
    // Options disabled as a whole after the overrides.
    std::vector<std::string> disabled = {};
};

// The harness combinations plus variants exercising percent heights, the current extruder,
// percent XY gaps, the base patterns, the "auto" interface patterns, disabled layer limits and
// interface counts, and object extruders with different nozzles.
std::vector<ParamsCase> params_cases();
// Compare every c_flow field with a native Flow, converted like the host converts it.
void check_flow(const c_flow &actual, const Flow &expected);
// Serialized name of a native InfillPattern value.
std::string pattern_name(InfillPattern pattern);
// Native PathParams pattern choice for one interface, as LegacySupportPaths.cpp computes it.
InfillPattern native_auto_interface(bool soluble, double interface_density);
// Compare two extrusion trees node by node: shape, flags, loops, points and attributes.
void check_same_tree(const ExtrusionEntity &actual, const ExtrusionEntity &expected,
                     const std::string &node = "root");
// Paths and length of the inner fills of a perimeter-wrapped tree, native and API.
struct InnerFillStats { size_t native_paths = 0, api_paths = 0; double native_length = 0, api_length = 0; };
// Compare the area nodes and their perimeters exactly, and only measure the inner fills.
void check_perimeters_and_measure_fill(const ExtrusionEntity &actual, const ExtrusionEntity &expected,
                                       InnerFillStats &stats);
// Count the leaves of a tree and add their lengths (mm).
void measure_leaves(const ExtrusionEntity &entity, size_t &paths, double &length);
// Forward a pattern request of the test payload to the pattern service, as the host step does.
int32_t service_generate_pattern(const run_ctx_support_toolpath *ctx, infill_pattern_runtime_id pattern_id,
                                 const layer_handle *layer, const layer_island_handle *island,
                                 const layer_region_island_handle *region_island, const layer_region_handle *primary_region,
                                 const surface_handle *surface, const expolygon_collection_handle *no_overlap_areas,
                                 const raw_infill_pattern_params *params, extrusion_entity_handle *output);
// Recipe of the inner pattern equal to the native FillParams and Fill settings of the legacy toolpaths.
raw_infill_pattern_params inner_recipe(infill_pattern_runtime_id pattern_id, const Flow &flow, double spacing, float density,
                                       float angle, double z);
// c_flow of a native flow, as the host converts it.
c_flow to_api_flow(const Flow &flow);

std::vector<ParamsCase> params_cases()
{
    std::vector<ParamsCase> cases;
    for (const char *style : {"grid", "snug"})
        for (const bool soluble : {false, true})
            for (const int interfaces : {0, 3})
                for (const bool same_tool : {false, true})
                    cases.push_back({style, soluble, interfaces, same_tool, {}});
    cases.push_back({"grid", false, 3, false, {{"first_layer_height", "75%"}}});
    cases.push_back({"grid", false, 3, false, {{"support_material_extruder", "0"}, {"support_material_interface_extruder", "0"}}});
    cases.push_back({"grid", false, 3, false, {{"support_material_xy_spacing", "60%"}, {"support_material_bottom_interface_expansion", "50%"}}});
    cases.push_back({"grid", false, 3, false, {{"support_material_pattern", "honeycomb"}, {"support_material_with_sheath", "1"}}});
    cases.push_back({"snug", false, 3, false, {{"support_material_pattern", "rectilinear-grid"}, {"support_material_spacing", "0"}}});
    cases.push_back({"grid", false, 2, false, {{"support_material_top_interface_pattern", "auto"}, {"support_material_bottom_interface_pattern", "auto"}}});
    cases.push_back({"grid", true, 2, false, {{"support_material_top_interface_pattern", "hilbertcurve"}, {"support_material_bottom_interface_pattern", "auto"}}});
    cases.push_back({"grid", false, 2, false, {{"support_material_interface_spacing", "0"},
                                               {"support_material_angle", "30"}, {"support_material_interface_angle", "45"}}});
    cases.push_back({"grid", false, 3, false, {}, {"max_layer_height"}});
    cases.push_back({"grid", false, 3, false, {}, {"support_material_bottom_interface_layers"}});
    cases.push_back({"grid", false, 3, false, {{"support_material_bottom_interface_layers", "-1"}}});
    cases.push_back({"grid", false, 3, false, {{"nozzle_diameter", "0.6,0.4"}, {"first_layer_height", "75%"}, {"perimeter_extruder", "1"},
                                               {"infill_extruder", "2"}, {"solid_infill_extruder", "2"},
                                               {"support_material_extruder", "0"}, {"support_material_interface_extruder", "0"}}});
    cases.push_back({"grid", false, 3, false, {{"z_step", "0.04"}, {"layer_height", "0.15"}, {"min_layer_height", "0.08,0.05"},
                                               {"max_layer_height", "75%,0.3"}}});
    return cases;
}

void check_flow(const c_flow &actual, const Flow &expected)
{
    CHECK(actual.width == expected.scaled_width());
    CHECK(actual.spacing == expected.scaled_spacing());
    CHECK(actual.height == expected.scaled_height());
    CHECK(actual.nozzle_diameter == scale_i(expected.nozzle_diameter()));
    CHECK(actual.is_bridge == (expected.bridge() ? 1 : 0));
    CHECK(actual.spacing_ratio == expected.spacing_ratio());
    CHECK(actual.mm3_per_mm == expected.mm3_per_mm());
}

std::string pattern_name(InfillPattern pattern)
{
    // The static InfillPattern name map has no "supportbase", so the support patterns are listed here.
    switch (pattern) {
    case ipRectilinear: return "rectilinear";
    case ipMonotonic: return "monotonic";
    case ipConcentric: return "concentric";
    case ipSawtooth: return "sawtooth";
    case ipHilbertCurve: return "hilbertcurve";
    case ipSmooth: return "smooth";
    case ipSupportBase: return "supportbase";
    case ipHoneycomb: return "honeycomb";
    default: FAIL("Unexpected support pattern " << int(pattern)); return {};
    }
}

InfillPattern native_auto_interface(bool soluble, double interface_density)
{
    return soluble ? ipConcentric : interface_density > 0.95 ? ipRectilinear : ipSupportBase;
}

void check_same_tree(const ExtrusionEntity &actual, const ExtrusionEntity &expected,
                     const std::string &node)
{
    CAPTURE(node, expected.is_loop(), expected.is_collection(), expected.child_count());
    REQUIRE(actual.has_polyline() == expected.has_polyline());
    REQUIRE(actual.child_count() == expected.child_count());
    CHECK(actual.can_sort() == expected.can_sort());
    CHECK(actual.can_reverse() == expected.can_reverse());
    CHECK(actual.is_loop() == expected.is_loop());
    if (expected.has_polyline())
        CHECK(actual.polyline_ref().to_polyline().points == expected.polyline_ref().to_polyline().points);
    const ExtrusionAttributes *actual_attributes = actual.get_property<ExtrusionAttributes>();
    const ExtrusionAttributes *expected_attributes = expected.get_property<ExtrusionAttributes>();
    REQUIRE((actual_attributes == nullptr) == (expected_attributes == nullptr));
    if (expected_attributes != nullptr) {
        CHECK(actual_attributes->role == expected_attributes->role);
        CHECK(actual_attributes->mm3_per_mm == expected_attributes->mm3_per_mm);
        // Width and height go through the scaled integers of c_flow: allow one scaled unit,
        // unscaled(1) = SCALING_FACTOR mm.
        CHECK(std::abs(actual_attributes->width - expected_attributes->width) <= unscaled(coord_t(1)));
        CHECK(std::abs(actual_attributes->height - expected_attributes->height) <= unscaled(coord_t(1)));
    }
    for (size_t idx = 0; idx < expected.child_count(); ++idx)
        check_same_tree(actual.child(idx), expected.child(idx), node + "/" + std::to_string(idx));
}

void measure_leaves(const ExtrusionEntity &entity, size_t &paths, double &length)
{
    if (entity.has_polyline()) {
        ++paths;
        length += unscaled(entity.polyline_ref().to_polyline().length());
    }
    for (size_t idx = 0; idx < entity.child_count(); ++idx)
        measure_leaves(entity.child(idx), paths, length);
}

void check_perimeters_and_measure_fill(const ExtrusionEntity &actual, const ExtrusionEntity &expected,
                                       InnerFillStats &stats)
{
    // A single area is the root itself; several areas are fixed nodes under a sortable root.
    REQUIRE(actual.can_sort() == expected.can_sort());
    std::vector<std::pair<const ExtrusionEntity *, const ExtrusionEntity *>> areas;
    if (expected.can_sort()) {
        REQUIRE(actual.child_count() == expected.child_count());
        for (size_t idx = 0; idx < expected.child_count(); ++idx)
            areas.emplace_back(&actual.child(idx), &expected.child(idx));
    } else {
        areas.emplace_back(&actual, &expected);
    }
    for (const std::pair<const ExtrusionEntity *, const ExtrusionEntity *> &area : areas) {
        REQUIRE(area.first->child_count() >= 1);
        REQUIRE(area.second->child_count() >= 1);
        CHECK(area.first->can_sort() == area.second->can_sort());
        check_same_tree(area.first->child(0), area.second->child(0), "perimeters");
        for (size_t idx = 1; idx < area.second->child_count(); ++idx)
            measure_leaves(area.second->child(idx), stats.native_paths, stats.native_length);
        for (size_t idx = 1; idx < area.first->child_count(); ++idx)
            measure_leaves(area.first->child(idx), stats.api_paths, stats.api_length);
    }
}

int32_t service_generate_pattern(const run_ctx_support_toolpath *ctx, infill_pattern_runtime_id pattern_id,
                                 const layer_handle *layer, const layer_island_handle *island,
                                 const layer_region_island_handle *region_island, const layer_region_handle *primary_region,
                                 const surface_handle *surface, const expolygon_collection_handle *no_overlap_areas,
                                 const raw_infill_pattern_params *params, extrusion_entity_handle *output)
{
    run_ctx_infill_pattern payload = {};
    payload.print = ctx->print;
    payload.object = ctx->object;
    payload.layer = layer;
    payload.island = island;
    payload.region_island = region_island;
    payload.primary_region = primary_region;
    payload.surface = surface;
    payload.no_overlap_areas = no_overlap_areas;
    payload.params = params;
    payload.output = output;
    return static_cast<Steps::InfillPatternService *>(ctx->host_context)->generate_pattern(pattern_id, payload);
}

c_flow to_api_flow(const Flow &flow)
{
    c_flow out = {};
    out.width = flow.scaled_width();
    out.spacing = flow.scaled_spacing();
    out.height = flow.scaled_height();
    out.nozzle_diameter = scale_i(flow.nozzle_diameter());
    out.is_bridge = flow.bridge() ? 1 : 0;
    out.spacing_ratio = flow.spacing_ratio();
    out.mm3_per_mm = flow.mm3_per_mm();
    return out;
}

raw_infill_pattern_params inner_recipe(infill_pattern_runtime_id pattern_id, const Flow &flow, double spacing, float density,
                                       float angle, double z)
{
    raw_infill_pattern_params params = {};
    params.pattern_id = pattern_id;
    params.surface_type = raw_surface_type(stPosInternal | stDensSparse);
    params.extrusion_role = RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL;
    params.flow = to_api_flow(flow);
    params.density = density;
    params.flow_mult = 1.f;
    params.connection = C_INFILL_CONNECTION_CONNECTED;
    params.infill_flags = C_INFILL_FLAG_DONT_ADJUST;
    params.spacing = spacing;
    params.angle = angle;
    params.bridge_angle = -1.;
    params.z = z;
    params.layer_height = unscaled(flow.scaled_height());
    params.bridge_offset = -1;
    params.fill_resolution = scale_i(0.0125);
    params.anchor_length = 1000.f;
    params.anchor_length_max = 1000.f;
    // The legacy toolpaths center every pattern on a fixed 2 mm box around the origin.
    params.object_bounding_box.min = {-scale_i(1.), -scale_i(1.)};
    params.object_bounding_box.max = {scale_i(1.), scale_i(1.)};
    return params;
}

// One inner pattern of a perimeter-wrapped fill. exact: the pattern plugin reproduces the native inner
// Fill polylines. The honeycomb plugin keeps its clipped pieces apart where the native filler connects
// them along the border (accepted gap), so only its perimeters are compared exactly and its inner fill
// is measured.
struct InnerCase { const char *plugin_id; InfillPattern pattern; float density; float angle; bool exact; };

// Results gathered over several perimeter-wrapped fill comparisons.
struct PerimeterComparison
{
    size_t multi_area_trees = 0;
    InnerFillStats honeycomb;
};

// The test runtime leaves supportbase inactive; enable it for the sheath case and restore it.
struct ActiveSupportBase
{
    bool was_active = Orchestrator::instance().is_plugin_active("supportbase");
    ActiveSupportBase() { REQUIRE(Orchestrator::instance().set_plugin_active("supportbase", true)); }
    ~ActiveSupportBase() { Orchestrator::instance().set_plugin_active("supportbase", was_active); }
};

// A print and a pattern service reached through a STEP_SUPPORT_TOOLPATH payload, as a toolpath plugin sees it.
struct PatternServiceFixture
{
    PatternServiceFixture()
    {
        Test::init_print({Test::TestMesh::cube_20x20x20}, print, model);
        service = std::make_unique<Steps::InfillPatternService>(Orchestrator::instance(), print, 1,
                                                                Steps::InfillPatternService::SetupPatterns::Refuse);
        payload.print = reinterpret_cast<const print_handle *>(&print);
        payload.object = reinterpret_cast<const object_handle *>(&print.object(0));
        payload.host_context = service.get();
        payload.generate_pattern = &service_generate_pattern;
    }
    // The plugin runtime and its configuration caches must exist before the Print is built.
    bool runtime_ready = (Test::Plugins::ensure_plugin_test_runtime_initialized(), true);
    Print print;
    Model model;
    std::unique_ptr<Steps::InfillPatternService> service;
    run_ctx_support_toolpath payload = {};
};

// The first-layer flange (rectilinear, raft density) and the sheath around supportbase and honeycomb bases.
const std::vector<InnerCase> &inner_cases();
// Flow of the perimeter-wrapped fill comparisons.
Flow support_test_flow();
// Fill one surface natively and through the API and compare the trees (see InnerCase::exact).
void compare_perimeter_fill(PatternServiceFixture &fixture, const ExPolygon &surface, const InnerCase &inner,
                            PerimeterComparison &result);
// Copy of the native test_path() (ClipperUtils.cpp, not exported): a ring too small or too narrow to keep.
bool native_small_ring(const ClipperLib::Path &path);
// Rings the native perimeter intersection of FillWithPerimeter produces and its filter then removes.
size_t native_small_intersection_rings(const ExPolygon &surface, double spacing);

const std::vector<InnerCase> &inner_cases()
{
    static const std::vector<InnerCase> cases{{"rectilinear", ipRectilinear, 0.9f, float(M_PI / 2.), true},
                                              {"supportbase", ipSupportBase, 0.3f, 0.f, true},
                                              {"honeycomb", ipHoneycomb, 0.3f, 0.f, false}};
    return cases;
}

Flow support_test_flow()
{
    return Flow::new_from_width(0.44f, 0.4f, 0.2f, 1.f);
}

void compare_perimeter_fill(PatternServiceFixture &fixture, const ExPolygon &surface, const InnerCase &inner,
                            PerimeterComparison &result)
{
    const Flow flow = support_test_flow();
    const double spacing = flow.spacing();
    const double z = 0.2;
    const PrintRegionConfig region_config;
    // Native: the filler and parameters of LegacySupportPaths fill_paths().
    std::unique_ptr<FillWithPerimeter> native_fill(new FillWithPerimeter(Fill::new_from_type(inner.pattern)));
    native_fill->ratio_fill_inside = 0.2f;
    native_fill->set_bounding_box(BoundingBox(Point(-scale_i(1.), -scale_i(1.)), Point(scale_i(1.), scale_i(1.))));
    native_fill->z = z;
    native_fill->angle = inner.angle;
    native_fill->link_max_length = 0;
    FillParams fill_params;
    fill_params.density = inner.density;
    fill_params.dont_adjust = true;
    fill_params.config = &region_config;
    fill_params.flow = flow;
    fill_params.role = ExtrusionRole::SupportMaterial;
    native_fill->init_spacing(spacing, fill_params);
    const Surface native_surface(stPosInternal | stDensSparse, surface);
    ExtrusionEntitiesPtr native_out;
    native_fill->fill_surface_extrusion(&native_surface, fill_params, native_out);
    std::vector<std::unique_ptr<ExtrusionEntity>> native_roots;
    for (ExtrusionEntity *entity : native_out)
        native_roots.emplace_back(entity);

    // API: the same surface and recipe, the inner pattern through the pattern service.
    PluginStorage storage;
    storage_handle *storage_value = reinterpret_cast<storage_handle *>(&storage);
    const raw_infill_pattern_params recipe = inner_recipe(fixture.service->resolve_pattern_id(inner.plugin_id), flow, spacing,
                                                          inner.density, inner.angle, z);
    Common::PerimeterFillSettings settings;
    settings.spacing = spacing;
    settings.ratio_fill_inside = 0.2f;
    settings.role = RAW_EXTRUSION_ROLE_SUPPORT_MATERIAL;
    settings.flow = to_api_flow(flow);
    slic3r_api::StoredExtrusionEntity out(storage_value);
    const slic3r_api::ExPolygon area(reinterpret_cast<const expolygon_handle *>(&surface));
    Common::fill_with_perimeter(storage_value, slic3r_api::Surface(area, raw_surface_type(stPosInternal | stDensSparse)),
                                settings, Common::pattern_service_fill(fixture.payload, recipe.pattern_id, recipe, nullptr, nullptr, nullptr),
                                out.mutable_view());
    const ExtrusionEntity &api_out = *reinterpret_cast<const ExtrusionEntity *>(out.handle());
    REQUIRE(api_out.child_count() == native_roots.size());
    if (native_roots.empty())
        return;
    const ExtrusionEntity &native_root = *native_roots.front();
    if (inner.exact)
        check_same_tree(api_out.child(0), native_root);
    else
        check_perimeters_and_measure_fill(api_out.child(0), native_root, result.honeycomb);
    if (native_root.child_count() > 0 && !native_root.child(0).is_leaf() && native_root.can_sort())
        ++result.multi_area_trees;
}

bool native_small_ring(const ClipperLib::Path &path)
{
    double area = ClipperLib::Area(path);
    ClipperLib::cInt max_dist_sqr = 0;
    const ClipperLib::IntPoint centroid = ClipperLib::Centroid(path, area);
    area = std::abs(area);
    for (const ClipperLib::IntPoint &pt : path) {
        const ClipperLib::cInt dx = std::abs(pt.x() - centroid.x()) & 0x3FFFFFFF;
        const ClipperLib::cInt dy = std::abs(pt.y() - centroid.y()) & 0x3FFFFFFF;
        max_dist_sqr = max_dist_sqr | (dx * dx + dy * dy);
    }
    return area < (2 * SCALED_EPSILON) * std::sqrt(max_dist_sqr);
}

size_t native_small_intersection_rings(const ExPolygon &surface, double spacing)
{
    // The first two steps of FillWithPerimeter::fill_surface_extrusion(), then the first pass of
    // intersection_ex() (clipper_do_polytree), whose output remove_small_areas() filters.
    const coord_t offset_for_overlap = scale_d(spacing / 2) * ((1 - 0.) / 2);
    const ExPolygons shrunk = offset2_ex(ExPolygons{surface}, scale_d(-spacing) / 2 - offset_for_overlap, offset_for_overlap,
                                         ClipperLib::jtMiter, scale_d(spacing) * 10);
    const ExPolygons limit = offset_ex(surface, scale_d(-spacing / 2));
    ClipperLib::Clipper clipper;
    for (const ExPolygon &expolygon : shrunk) {
        clipper.AddPath(expolygon.contour.points, ClipperLib::ptSubject, true);
        for (const Polygon &hole : expolygon.holes)
            clipper.AddPath(hole.points, ClipperLib::ptSubject, true);
    }
    for (const ExPolygon &expolygon : limit) {
        clipper.AddPath(expolygon.contour.points, ClipperLib::ptClip, true);
        for (const Polygon &hole : expolygon.holes)
            clipper.AddPath(hole.points, ClipperLib::ptClip, true);
    }
    ClipperLib::Paths rings;
    clipper.Execute(ClipperLib::ctIntersection, rings, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
    return size_t(std::count_if(rings.begin(), rings.end(), native_small_ring));
}
} // namespace

TEST_CASE("Support params equal the native grid support parameters", "[plugins][support-common]")
{
    for (const ParamsCase &test : params_cases()) {
        CAPTURE(test.style, test.soluble, test.interfaces, test.same_tool, test.overrides.size(), test.disabled.size());
        DynamicPrintConfig config = SG::config(test.style, test.soluble, test.interfaces, test.same_tool);
        for (const std::pair<std::string, std::string> &item : test.overrides)
            config.set_deserialize_strict(item.first, item.second);
        for (const std::string &key : test.disabled) {
            REQUIRE(config.option(key) != nullptr);
            config.option(key)->set_enabled(false);
        }
        Print print;
        Model model;
        Test::init_print({SG::slab()}, print, model, config);
        Orchestrator &host = Orchestrator::instance();
        Steps::StepLayerHeightGeneration::prepare(print);
        Steps::StepLayerHeightGeneration::run_step(host, print);
        Steps::StepSlicing::run_step(host, print);
        const PrintObject &object = print.object(0);

        const SupportGrid::GridSupportParams native(object);
        const Common::SupportParams params(slic3r_api::Print(reinterpret_cast<const print_handle *>(&print)),
                                           slic3r_api::Object(reinterpret_cast<const object_handle *>(&object)));
        CHECK(params.first_layer_height == native.first_layer_height);
        CHECK(params.layer_height == native.layer_height);
        CHECK(params.max_support_height == native.max_support_height);
        CHECK(params.gap_above == native.gap_above);
        CHECK(params.gap_below == native.gap_below);
        CHECK(params.z_step == native.z_step);
        CHECK(params.soluble == native.soluble);
        CHECK(params.top_interfaces == native.top_interfaces);
        CHECK(params.bottom_interfaces == native.bottom_interfaces);
        check_flow(params.first_flow, native.first_flow);
        check_flow(params.base_flow, native.base_flow);
        check_flow(params.interface_flow, native.interface_flow);
        check_flow(params.bottom_flow, native.bottom_flow);
        CHECK(params.bottom_expansion.value == native.bottom_expansion.value);
        CHECK((params.bottom_expansion.percent != 0) == native.bottom_expansion.percent);
        CHECK(params.min_height == native.min_height);
        CHECK(params.gap_xy == native.gap_xy);
        CHECK(params.resolution == native.resolution);
        // Densities s / (gap + s) read the spacing s from c_flow, the native float spacing truncated to
        // a scaled unit, unscaled(1) = SCALING_FACTOR mm: the error is at most the derivative
        // gap / (gap + s)^2 times that unit.
        const PrintObjectConfig &object_config = object.config();
        const auto density_bound = [](double gap, double spacing) {
            return gap / ((gap + spacing) * (gap + spacing)) * unscaled(coord_t(1)) + 1e-12;
        };
        const double interface_gap = object_config.support_material_interface_layers.value == 0 ?
            object_config.support_material_spacing.value : object_config.support_material_interface_spacing.value;
        CHECK(std::abs(params.interface_density - native.interface_density) <=
              density_bound(interface_gap, native.interface_flow.spacing()));

        // Path settings, as the native PathParams of LegacySupportPaths.cpp derives them.
        const double native_density = std::min(1., native.base_flow.spacing() /
            (object_config.support_material_spacing.value + native.base_flow.spacing()));
        CHECK(std::abs(params.base_density - native_density) <=
              density_bound(object_config.support_material_spacing.value, native.base_flow.spacing()));
        const InfillPattern base_pattern = object_config.support_material_pattern.value == smpHoneycomb ? ipHoneycomb :
            native_density > 0.95 || object_config.support_material_with_sheath.value ? ipRectilinear : ipSupportBase;
        InfillPattern top_pattern = object_config.support_material_top_interface_pattern.value;
        InfillPattern bottom_pattern = object_config.support_material_bottom_interface_pattern.value;
        if (top_pattern == ipAuto)
            top_pattern = native_auto_interface(native.soluble, native.interface_density);
        if (bottom_pattern == ipAuto)
            bottom_pattern = top_pattern != ipHilbertCurve && top_pattern != ipSmooth && top_pattern != ipSawtooth ? top_pattern :
                native_auto_interface(native.soluble, native.interface_density);
        CHECK(params.base_pattern_id == pattern_name(base_pattern));
        CHECK(params.top_pattern_id == pattern_name(top_pattern));
        CHECK(params.bottom_pattern_id == pattern_name(bottom_pattern));
        CHECK(params.with_sheath == object_config.support_material_with_sheath.value);
        CHECK(params.interface_contact_loops == object_config.support_material_interface_contact_loops.value);
        CHECK(params.base_angle == Geometry::deg2rad(float(object_config.support_material_angle.value)));
        CHECK(params.interface_angle == Geometry::deg2rad(float(object_config.support_material_interface_angle.value)));
        CHECK(params.angle_height == object_config.support_material_angle_height.value);
        CHECK(params.interface_angle_increment == object_config.support_material_interface_angle_increment.value);
        CHECK(params.raft_first_layer_density == Approx(object_config.raft_first_layer_density.get_effective_value(1.)).epsilon(1e-12));
        CHECK(params.style == (std::string(test.style) == "grid" ? Common::SupportStyle::Grid : Common::SupportStyle::Snug));
    }
}

TEST_CASE("Support enum helpers read every serialized value", "[plugins][support-common]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const auto values = [](const char *key) {
        const ConfigOptionDef *def = PrintConfigDef::instance().get(key);
        REQUIRE(def != nullptr);
        REQUIRE(def->enum_def.get() != nullptr);
        return def->enum_def->values();
    };
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    const slic3r_api::Config view(ApiHost::to_config_handle(&config));

    for (const std::string &value : values("support_material_style")) {
        CAPTURE(value);
        config.set_deserialize_strict("support_material_style", value);
        const SupportMaterialStyle native = config.opt_enum<SupportMaterialStyle>("support_material_style");
        const Common::SupportStyle expected = native == smsGrid ? Common::SupportStyle::Grid :
            native == smsSnug ? Common::SupportStyle::Snug : native == smsOrganic ? Common::SupportStyle::Organic :
            Common::SupportStyle::Unknown;
        CHECK(expected != Common::SupportStyle::Unknown);
        CHECK(Common::support_style(view) == expected);
    }
    for (const std::string &value : values("support_material_contact_distance_type")) {
        CAPTURE(value);
        config.set_deserialize_strict("support_material_contact_distance_type", value);
        const SupportZDistanceType native = config.opt_enum<SupportZDistanceType>("support_material_contact_distance_type");
        const Common::ContactDistanceType expected = native == zdFilament ? Common::ContactDistanceType::Filament :
            native == zdPlane ? Common::ContactDistanceType::Plane : native == zdNone ? Common::ContactDistanceType::None :
            Common::ContactDistanceType::Unknown;
        CHECK(expected != Common::ContactDistanceType::Unknown);
        CHECK(Common::contact_distance_type(view) == expected);
    }
    for (const std::string &value : values("support_material_pattern")) {
        CAPTURE(value);
        config.set_deserialize_strict("support_material_pattern", value);
        const SupportMaterialPattern native = config.opt_enum<SupportMaterialPattern>("support_material_pattern");
        const Common::BasePattern expected = native == smpRectilinear ? Common::BasePattern::Rectilinear :
            native == smpRectilinearGrid ? Common::BasePattern::RectilinearGrid :
            native == smpHoneycomb ? Common::BasePattern::Honeycomb : Common::BasePattern::Unknown;
        CHECK(expected != Common::BasePattern::Unknown);
        const Common::BasePattern pattern = Common::base_pattern(view);
        CHECK(pattern == expected);
        // The base pattern id follows the native PathParams choice at both sides of the 0.95 density threshold.
        for (const double density : {0.3, 0.96})
            for (const bool sheath : {false, true}) {
                const InfillPattern native_id = native == smpHoneycomb ? ipHoneycomb :
                    density > 0.95 || sheath ? ipRectilinear : ipSupportBase;
                CHECK(Common::base_pattern_plugin_id(pattern, density, sheath) == pattern_name(native_id));
            }
    }
    CHECK(Common::parse_support_style("tree") == Common::SupportStyle::Unknown);
    CHECK(Common::parse_contact_distance_type("") == Common::ContactDistanceType::Unknown);
    CHECK(Common::parse_base_pattern("lightning") == Common::BasePattern::Unknown);

    // Interface patterns: every top value, then every bottom value after every top value, soluble or
    // not, on both sides of the 0.95 interface density threshold.
    const std::vector<std::string> top_values = values("support_material_top_interface_pattern");
    const std::vector<std::string> bottom_values = values("support_material_bottom_interface_pattern");
    for (const bool soluble : {false, true})
        for (const double density : {0.4, 0.96})
            for (const std::string &top : top_values) {
                CAPTURE(soluble, density, top);
                if (top == "concentricgapfill") {
                    // Listed in the option but absent from the native InfillPattern map, so no native
                    // config holds it; the helper passes it on for the pattern service to report.
                    CHECK(Common::top_interface_pattern_plugin_id(top, soluble, density) == top);
                    continue;
                }
                config.set_deserialize_strict("support_material_top_interface_pattern", top);
                InfillPattern native_top = config.opt_enum<InfillPattern>("support_material_top_interface_pattern");
                if (native_top == ipAuto)
                    native_top = native_auto_interface(soluble, density);
                const std::string top_id = Common::top_interface_pattern_plugin_id(top, soluble, density);
                CHECK(top_id == pattern_name(native_top));
                for (const std::string &bottom : bottom_values) {
                    CAPTURE(bottom);
                    config.set_deserialize_strict("support_material_bottom_interface_pattern", bottom);
                    InfillPattern native_bottom = config.opt_enum<InfillPattern>("support_material_bottom_interface_pattern");
                    if (native_bottom == ipAuto)
                        native_bottom = native_top != ipHilbertCurve && native_top != ipSmooth && native_top != ipSawtooth ?
                            native_top : native_auto_interface(soluble, density);
                    CHECK(Common::bottom_interface_pattern_plugin_id(bottom, top_id, soluble, density) == pattern_name(native_bottom));
                }
            }
}

TEST_CASE("Support plugins declare their keys from the Common library", "[plugins][support-common]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    const Common::UsedKeyList geometry = Common::support_geometry_keys();
    const Common::UsedKeyList path = Common::support_path_keys();
    const auto declares = [](const Plugin &plugin, const Common::UsedKeyList &list) {
        for (size_t idx = 0; idx < list.size; ++idx) {
            const std::string key = list.keys[idx].key;
            CAPTURE(plugin.get_id(), key);
            CHECK(std::any_of(plugin.get_used_config_keys().begin(), plugin.get_used_config_keys().end(),
                              [&key](const Plugin::UsedConfigKey &used) { return used.key == key; }));
        }
    };
    const Plugin *family = Orchestrator::instance().get_plugin("support.family.grid");
    REQUIRE(family != nullptr);
    declares(*family, geometry);
    CHECK(family->get_used_config_keys().size() == geometry.size);
    for (const char *id : {"support.toolpath.legacy_sparse", "support.toolpath.legacy_bottom", "support.toolpath.legacy_top"}) {
        const Plugin *toolpath = Orchestrator::instance().get_plugin(id);
        REQUIRE(toolpath != nullptr);
        declares(*toolpath, geometry);
        declares(*toolpath, path);
        CHECK(toolpath->get_used_config_keys().size() == geometry.size + path.size);
    }
    // A null buffer returns the count; a buffer receives the lists one after the other.
    std::vector<raw_used_config_key> keys(geometry.size + path.size);
    CHECK(Common::append_used_config_keys(nullptr, 0, geometry) == int32_t(geometry.size));
    const int32_t count = Common::append_used_config_keys(keys.data(), 0, geometry);
    CHECK(Common::append_used_config_keys(keys.data(), count, path) == int32_t(keys.size()));
    CHECK(std::string(keys.back().key) == path.keys[path.size - 1].key);
}

TEST_CASE("Perimeter-wrapped support fill equals the native FillWithPerimeter", "[plugins][support-common]")
{
    const ActiveSupportBase active_supportbase;
    PatternServiceFixture fixture;

    // A rectangle, an L with a hole, and two squares joined by a neck the perimeter inset splits.
    std::vector<ExPolygon> surfaces;
    surfaces.emplace_back(Polygon({Point(0, 0), Point(scale_i(10.), 0), Point(scale_i(10.), scale_i(6.)), Point(0, scale_i(6.))}));
    ExPolygon l_shape(Polygon({Point(0, 0), Point(scale_i(12.), 0), Point(scale_i(12.), scale_i(4.)), Point(scale_i(4.), scale_i(4.)),
                               Point(scale_i(4.), scale_i(12.)), Point(0, scale_i(12.))}));
    Polygon hole({Point(scale_i(1.), scale_i(1.)), Point(scale_i(1.), scale_i(2.5)), Point(scale_i(2.5), scale_i(2.5)), Point(scale_i(2.5), scale_i(1.))});
    hole.make_clockwise();
    l_shape.holes.push_back(hole);
    surfaces.push_back(l_shape);
    surfaces.emplace_back(Polygon({Point(0, 0), Point(scale_i(6.), 0), Point(scale_i(6.), scale_i(2.8)), Point(scale_i(6.4), scale_i(2.8)),
                                   Point(scale_i(6.4), 0), Point(scale_i(12.4), 0), Point(scale_i(12.4), scale_i(6.)),
                                   Point(scale_i(6.4), scale_i(6.)), Point(scale_i(6.4), scale_i(3.2)), Point(scale_i(6.), scale_i(3.2)),
                                   Point(scale_i(6.), scale_i(6.)), Point(0, scale_i(6.))}));

    PerimeterComparison result;
    for (const InnerCase &inner : inner_cases())
        for (size_t surface_idx = 0; surface_idx < surfaces.size(); ++surface_idx) {
            CAPTURE(inner.plugin_id, surface_idx);
            compare_perimeter_fill(fixture, surfaces[surface_idx], inner, result);
        }
    // The neck case must exercise the sortable root with one node per area.
    CHECK(result.multi_area_trees > 0);
    // Characterisation of the accepted honeycomb gap: the plugin leaves more, shorter paths.
    CAPTURE(result.honeycomb.native_paths, result.honeycomb.native_length, result.honeycomb.api_paths, result.honeycomb.api_length);
    CHECK(result.honeycomb.api_paths > result.honeycomb.native_paths);
    CHECK(result.honeycomb.api_length < result.honeycomb.native_length);
}

// The native intersection_ex() drops the rings too small or too narrow to keep before its union
// pass; the helper must drop the same ones. Seeded star shapes are searched for surfaces whose
// perimeter intersection leaves such rings, and those are compared with the native filler.
TEST_CASE("Perimeter-wrapped support fill drops the small intersection rings like native", "[plugins][support-common]")
{
    PatternServiceFixture fixture;
    const double spacing = support_test_flow().spacing();
    std::mt19937 rng(20261008);
    std::uniform_real_distribution<double> radius(0.3, 6.);
    std::vector<ExPolygon> residue_surfaces;
    size_t removed_rings = 0;
    for (size_t shape = 0; shape < 5000 && residue_surfaces.size() < 4; ++shape) {
        const size_t vertices = 5 + shape % 60;
        Polygon star;
        for (size_t idx = 0; idx < vertices; ++idx) {
            const double angle = 2. * M_PI * double(idx) / double(vertices);
            const double r = radius(rng);
            star.points.emplace_back(scale_i(r * std::cos(angle)), scale_i(r * std::sin(angle)));
        }
        star.make_counter_clockwise();
        const ExPolygon surface(star);
        if (const size_t removed = native_small_intersection_rings(surface, spacing); removed > 0) {
            residue_surfaces.push_back(surface);
            removed_rings += removed;
        }
    }
    CAPTURE(residue_surfaces.size(), removed_rings);
    REQUIRE_FALSE(residue_surfaces.empty());
    CHECK(removed_rings >= residue_surfaces.size());
    PerimeterComparison result;
    for (size_t surface_idx = 0; surface_idx < residue_surfaces.size(); ++surface_idx) {
        CAPTURE(surface_idx);
        compare_perimeter_fill(fixture, residue_surfaces[surface_idx], inner_cases().front(), result);
    }
}
// Evidence for the sheath gap: with the same recipe, the honeycomb plugin gives the pieces of the native
// unconnected honeycomb, ignores the connection field, and the native connected honeycomb joins them.
TEST_CASE("Honeycomb plugin pieces equal the native honeycomb without connection", "[plugins][support-common]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &host = Orchestrator::instance();
    REQUIRE(host.is_plugin_active("honeycomb"));
    Print print;
    Model model;
    Test::init_print({Test::TestMesh::cube_20x20x20}, print, model);
    Steps::InfillPatternService service(host, print, 1, Steps::InfillPatternService::SetupPatterns::Refuse);
    run_ctx_support_toolpath payload = {};
    payload.print = reinterpret_cast<const print_handle *>(&print);
    payload.object = reinterpret_cast<const object_handle *>(&print.object(0));
    payload.host_context = &service;
    payload.generate_pattern = &service_generate_pattern;

    const ExPolygon area(Polygon({Point(0, 0), Point(scale_i(10.), 0), Point(scale_i(10.), scale_i(6.)), Point(0, scale_i(6.))}));
    const Flow flow = Flow::new_from_width(0.44f, 0.4f, 0.2f, 1.f);
    const double spacing = flow.spacing();
    const float density = 0.3f;
    const PrintRegionConfig region_config;

    // Native honeycomb with the Fill members and FillParams the sheath gives its inner filler.
    const auto native_paths = [&](InfillConnection connection) {
        std::unique_ptr<Fill> fill(Fill::new_from_type(ipHoneycomb));
        fill->set_bounding_box(BoundingBox(Point(-scale_i(1.), -scale_i(1.)), Point(scale_i(1.), scale_i(1.))));
        fill->z = 0.2;
        fill->angle = 0.f;
        fill->link_max_length = 0;
        FillParams params;
        params.density = density;
        params.dont_adjust = true;
        params.flow = flow;
        params.role = ExtrusionRole::SupportMaterial;
        params.connection = connection;
        params.config = &region_config;
        fill->init_spacing(spacing, params);
        const Surface surface(stPosInternal | stDensSparse, area);
        return fill->fill_surface(&surface, params);
    };
    // Plugin honeycomb through the pattern service with the recipe of the perimeter-wrapped fill.
    const auto plugin_paths = [&](c_infill_connection connection) {
        raw_infill_pattern_params recipe = inner_recipe(service.resolve_pattern_id("honeycomb"), flow, spacing, density, 0.f, 0.2);
        recipe.connection = connection;
        PluginStorage storage;
        storage_handle *storage_value = reinterpret_cast<storage_handle *>(&storage);
        slic3r_api::StoredSurfaceCollection surfaces(storage_value);
        surfaces.append(slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&area)), raw_surface_type(stPosInternal | stDensSparse));
        ExtrusionEntityCollection output;
        REQUIRE(service_generate_pattern(&payload, recipe.pattern_id, nullptr, nullptr, nullptr, nullptr,
                                         surface_collection_at(surfaces.handle(), 0), nullptr, &recipe,
                                         reinterpret_cast<extrusion_entity_handle *>(&output)) == 1);
        Polylines lines;
        for (const ArcPolyline &line : output.as_polylines())
            lines.push_back(line.to_polyline());
        return lines;
    };
    const auto total_length = [](const Polylines &lines) {
        double length = 0;
        for (const Polyline &line : lines)
            length += unscaled(line.length());
        return length;
    };

    const Polylines native_connected = native_paths(InfillConnection::icConnected);
    const Polylines native_unconnected = native_paths(InfillConnection::icNotConnected);
    const Polylines plugin_connected = plugin_paths(C_INFILL_CONNECTION_CONNECTED);
    const Polylines plugin_unconnected = plugin_paths(C_INFILL_CONNECTION_NOT_CONNECTED);
    INFO("Honeycomb on a 10 x 6 mm rectangle: native connected " << native_connected.size() << " paths, "
         << total_length(native_connected) << " mm; native unconnected " << native_unconnected.size() << " paths, "
         << total_length(native_unconnected) << " mm; plugin " << plugin_connected.size() << " paths, "
         << total_length(plugin_connected) << " mm");
    // The plugin output does not depend on the connection field.
    CHECK(plugin_connected == plugin_unconnected);
    // It holds the same pieces as the native unconnected honeycomb (the native chains them in another order).
    CHECK(plugin_connected.size() == native_unconnected.size());
    CHECK(total_length(plugin_connected) == Approx(total_length(native_unconnected)).epsilon(1e-9));
    // Connecting joins pieces along the border: fewer, longer paths.
    CHECK(native_connected.size() < native_unconnected.size());
}
