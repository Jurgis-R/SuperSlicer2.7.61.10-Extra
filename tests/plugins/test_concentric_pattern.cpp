#include <catch2/catch.hpp>
#include <algorithm>
#include <cmath>
#include <memory>

#include "perimeter_test_helpers.hpp"
#include "plugin_test_helpers.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/Fill/FillBase.hpp"
#include "libslic3r/Plugins/Infill/ConcentricPattern.hpp"
#include "libslic3r/Surface.hpp"

// The concentric group is not sortable: within a component, every ring is printed after the ring around it.
// Loops are reversible and their seams will change, so loops are compared as rings, not from their start point.
// Components come from the API offset, whose component order is unspecified, so they are matched as a set.
using namespace Slic3r;
using namespace Slic3r::Test::PerimeterPluginTests;
namespace {
using Path = std::vector<std::pair<coord_t, coord_t>>;
using Paths = std::vector<Path>;

// Traverse children in publication order; closed paths retain their duplicate closing point.
void collect(const slic3r_api::ExtrusionEntity &entity, Paths &paths);
// Mirror the per-surface recipe independently of the native filler configuration.
raw_infill_pattern_params recipe(const ExPolygon &surface, float density, bool dont_adjust, float cap, coord_t clipping);
// Run the native FillConcentric on one recipe.
Paths native_paths(const ExPolygon &polygon, const raw_infill_pattern_params &raw, bool dont_adjust);
// Run the plugin on one recipe and read its published paths.
Paths api_paths(const ExPolygon &polygon, const raw_infill_pattern_params &raw);
// Split a path sequence into components: each starts with its outermost loop and its other loops lie inside it.
std::vector<Paths> components(const Paths &paths);
// The ring of an unclipped closed path, rotated to its smallest point, direction kept.
Path canonical_loop(const Path &closed);
// The sorted rings of a component, which identify it independently of order and seams.
Paths loop_set(const Paths &component);
// Whether the ring of one path encloses the start of another.
bool encloses(const Path &outer, const Path &inner);
// Whether every pair of loops is nested, so that the outside-in order is unique.
bool totally_nested(const Paths &component);
// Compare the paths printed with loop clipping with the same paths printed without it.
void require_clipped(const Paths &unclipped, const Paths &clipped, coord_t clipping);
double path_length(const Path &path);

void collect(const slic3r_api::ExtrusionEntity &entity, Paths &paths)
{
    if (entity.is_leaf()) {
        Path &path = paths.emplace_back();
        for (const c_point point : entity.points())
            path.emplace_back(point.x, point.y);
    } else
        for (const slic3r_api::ExtrusionEntity child : entity.children())
            collect(child, paths);
}

raw_infill_pattern_params recipe(const ExPolygon &surface, float density, bool dont_adjust, float cap, coord_t clipping)
{
    raw_infill_pattern_params raw = {};
    raw.spacing = 0.45;
    raw.density = density;
    raw.max_sparse_infill_spacing = cap;
    raw.loop_clipping = clipping;
    raw.fill_resolution = scale_i(0.0125);
    raw.infill_flags = dont_adjust ? C_INFILL_FLAG_DONT_ADJUST : 0;
    raw.flow_mult = 0.7f;
    raw.flow.width = scale_i(0.45);
    raw.flow.height = scale_i(0.2);
    raw.flow.mm3_per_mm = 0.08;
    raw.extrusion_role = RAW_EXTRUSION_ROLE_INTERNAL_INFILL;
    const BoundingBox box = get_extents(surface);
    raw.object_bounding_box = {{box.min.x(), box.min.y()}, {box.max.x(), box.max.y()}};
    return raw;
}

Paths native_paths(const ExPolygon &polygon, const raw_infill_pattern_params &raw, bool dont_adjust)
{
    const std::unique_ptr<Fill> filler(Fill::new_from_type(ipConcentric));
    filler->set_bounding_box(get_extents(polygon));
    filler->angle = 0.f;
    filler->layer_id = 0;
    filler->loop_clipping = raw.loop_clipping;
    static const PrintRegionConfig config;
    FillParams params;
    params.config = &config;
    params.density = raw.density;
    params.dont_adjust = dont_adjust;
    params.max_sparse_infill_spacing = raw.max_sparse_infill_spacing;
    params.fill_resolution = raw.fill_resolution;
    filler->init_spacing(raw.spacing, params);
    Surface surface(stPosInternal | stDensSparse, polygon);
    Paths result;
    for (const Polyline &line : filler->fill_surface(&surface, params)) {
        Path &path = result.emplace_back();
        for (const Point &point : line.points)
            path.emplace_back(point.x(), point.y());
    }
    return result;
}

Paths api_paths(const ExPolygon &polygon, const raw_infill_pattern_params &raw)
{
    Surface surface(stPosInternal | stDensSparse, polygon);
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    slic3r_api::StoredExtrusionEntity output(handle);
    run_ctx_infill_pattern ctx = {};
    ctx.surface = reinterpret_cast<const surface_handle *>(&surface);
    ctx.params = &raw;
    ctx.output = output.mutable_handle();
    slic3r_api::Infill::fill_concentric_surface(ctx, handle, nullptr, {});
    Paths result;
    collect(output.readonly(), result);
    return result;
}

std::vector<Paths> components(const Paths &paths)
{
    std::vector<Paths> result;
    Polygon outer;
    for (const Path &path : paths) {
        const Point start(path.front().first, path.front().second);
        if (result.empty() || !outer.contains(start)) {
            result.emplace_back();
            outer.points.clear();
            for (const std::pair<coord_t, coord_t> &point : path)
                outer.points.emplace_back(point.first, point.second);
        }
        result.back().push_back(path);
    }
    return result;
}

Path canonical_loop(const Path &closed)
{
    Path ring(closed.begin(), closed.end());
    if (ring.size() > 1 && ring.front() == ring.back())
        ring.pop_back();
    std::rotate(ring.begin(), std::min_element(ring.begin(), ring.end()), ring.end());
    return ring;
}

Paths loop_set(const Paths &component)
{
    Paths result;
    for (const Path &path : component)
        result.push_back(canonical_loop(path));
    std::sort(result.begin(), result.end());
    return result;
}

bool encloses(const Path &outer, const Path &inner)
{
    Polygon ring;
    for (const std::pair<coord_t, coord_t> &point : canonical_loop(outer))
        ring.points.emplace_back(point.first, point.second);
    return ring.contains(Point(inner.front().first, inner.front().second));
}

bool totally_nested(const Paths &component)
{
    for (size_t first = 0; first < component.size(); ++first)
        for (size_t second = first + 1; second < component.size(); ++second)
            if (!encloses(component[first], component[second]) && !encloses(component[second], component[first]))
                return false;
    return true;
}

void require_clipped(const Paths &unclipped, const Paths &clipped, coord_t clipping)
{
    // Clipping only shortens each loop from its end: the seams and the order stay those of the
    // unclipped run, and a loop no longer than the clipping disappears.
    size_t next = 0;
    for (size_t index = 0; index < unclipped.size(); ++index) {
        const Path &loop = unclipped[index];
        CAPTURE(index);
        const bool kept = next < clipped.size() && clipped[next].front() == loop.front() &&
                          clipped[next].size() <= loop.size();
        if (!kept) {
            CHECK(path_length(loop) <= double(clipping) + 5.);
            continue;
        }
        const Path &path = clipped[next++];
        for (size_t point = 0; point + 1 < path.size(); ++point)
            REQUIRE(path[point] == loop[point]);
        CHECK(path_length(path) == Approx(path_length(loop) - double(clipping)).margin(5.));
    }
    CHECK(next == clipped.size());
}

double path_length(const Path &path)
{
    double length = 0.;
    for (size_t index = 1; index < path.size(); ++index)
        length += std::hypot(double(path[index].first - path[index - 1].first),
                             double(path[index].second - path[index - 1].second));
    return length;
}
}

TEST_CASE("Concentric plugin prints the native loops of each component outside-in", "[plugins][infill][concentric]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    const ExPolygons narrow = union_ex(ExPolygons{rectangle_expolygon(-10., -5., -4., 5.),
        rectangle_expolygon(-4., -0.1, 4., 0.1), rectangle_expolygon(4., -5., 10., 5.)});
    REQUIRE(narrow.size() == 1);
    // A wider bridge keeps one component that splits into two branches deeper in the onion.
    const ExPolygons dumbbell = union_ex(ExPolygons{rectangle_expolygon(-12., -5., -4., 5.),
        rectangle_expolygon(-4., -1., 4., 1.), rectangle_expolygon(4., -5., 12., 5.)});
    REQUIRE(dumbbell.size() == 1);
    const ExPolygon lshape(Polygon{Point(0, 0), Point(scale_i(30.), 0), Point(scale_i(30.), scale_i(10.)),
        Point(scale_i(10.), scale_i(10.)), Point(scale_i(10.), scale_i(25.)), Point(0, scale_i(25.))});
    const ExPolygons nested = union_ex(ExPolygons{rectangle_with_hole_expolygon(), rectangle_expolygon(-2., -2., 2., 2.)});
    REQUIRE(nested.size() == 2);
    // Two holes whose grown rings merge, so one hole ring encloses both smaller ones.
    ExPolygon two_holes = rectangle_expolygon(-15., -10., 15., 10.);
    for (const ExPolygon &hole : {rectangle_expolygon(-8., -3., -1.5, 3.), rectangle_expolygon(1.5, -3., 8., 3.)}) {
        Polygon ring = hole.contour;
        ring.make_clockwise();
        two_holes.holes.push_back(ring);
    }
    ExPolygons cases{rectangle_expolygon(-19., -19., 19., 19.), rectangle_with_hole_expolygon(), lshape, narrow.front(),
                     dumbbell.front(), two_holes};
    cases.insert(cases.end(), nested.begin(), nested.end());
    for (const ExPolygon &polygon : cases)
        for (float density : {0.15f, 0.5f, 1.f})
            for (bool dont_adjust : {true, false})
                for (float cap : {0.f, 0.8f}) {
                    CAPTURE(polygon.holes.size(), polygon.area(), density, dont_adjust, cap);
                    const raw_infill_pattern_params raw = recipe(polygon, density, dont_adjust, cap, 0);
                    const Paths native = native_paths(polygon, raw, dont_adjust);
                    const Paths actual = api_paths(polygon, raw);
                    REQUIRE_FALSE(native.empty());
                    REQUIRE(actual.size() == native.size());

                    std::vector<Paths> native_components = components(native);
                    std::vector<Paths> actual_components = components(actual);
                    REQUIRE(actual_components.size() == native_components.size());
                    const auto by_loops = [](const Paths &first, const Paths &second) {
                        return loop_set(first) < loop_set(second);
                    };
                    std::sort(native_components.begin(), native_components.end(), by_loops);
                    std::sort(actual_components.begin(), actual_components.end(), by_loops);
                    for (size_t component = 0; component < native_components.size(); ++component) {
                        CAPTURE(component);
                        const Paths &expected = native_components[component];
                        const Paths &printed = actual_components[component];
                        REQUIRE(loop_set(printed) == loop_set(expected));
                        // A ring never follows a ring it encloses.
                        for (size_t first = 0; first < printed.size(); ++first)
                            for (size_t second = first + 1; second < printed.size(); ++second) {
                                CAPTURE(first, second);
                                CHECK_FALSE(encloses(printed[second], printed[first]));
                            }
                        // Without siblings the outside-in order is unique, so it is the native one.
                        if (totally_nested(expected))
                            for (size_t index = 0; index < expected.size(); ++index) {
                                CAPTURE(index);
                                CHECK(canonical_loop(printed[index]) == canonical_loop(expected[index]));
                            }
                    }

                    const coord_t clipping = scale_i(0.1);
                    require_clipped(actual, api_paths(polygon, recipe(polygon, density, dont_adjust, cap, clipping)),
                                    clipping);
                }
}

TEST_CASE("Concentric groups keep order and classical loops under Arachne", "[plugins][infill][concentric]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Surface surface(stPosInternal | stDensSparse, rectangle_expolygon(-10., -10., 10., 10.));
    for (const bool monotonic : {false, true}) {
        Paths reference;
        for (const bool arachne : {false, true}) {
            CAPTURE(monotonic, arachne);
            raw_infill_pattern_params raw = recipe(surface.expolygon, 0.5f, true, 0.f, 0);
            raw.infill_flags |= monotonic ? C_INFILL_FLAG_MONOTONIC : 0;
            raw.infill_flags |= arachne ? C_INFILL_FLAG_USE_ARACHNE : 0;
            PluginStorage storage;
            storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
            slic3r_api::StoredExtrusionEntity output(handle);
            run_ctx_infill_pattern ctx = {};
            ctx.surface = reinterpret_cast<const surface_handle *>(&surface);
            ctx.params = &raw;
            ctx.output = output.mutable_handle();
            slic3r_api::Infill::fill_concentric_surface(ctx, handle, nullptr, {});
            REQUIRE_FALSE(output.empty());
            const slic3r_api::ExtrusionEntity group = *output.readonly().children().begin();
            CHECK_FALSE(group.sortable());
            CHECK_FALSE(group.reversible());
            for (const slic3r_api::ExtrusionEntity path : group.children()) {
                CHECK(path.is_loop());
                CHECK(path.reversible() == !monotonic);
                const slic3r_api::EPropertyAttributes *attributes = path.get(slic3r_api::EPropertyAttributes::key);
                REQUIRE(attributes != nullptr);
                const c_extrusion_property_attributes &flow = *attributes;
                CHECK(flow.mm3_per_mm == Approx(raw.flow.mm3_per_mm * raw.flow_mult));
                CHECK(attributes->extrusion_role() == RAW_EXTRUSION_ROLE_INTERNAL_INFILL);
            }
            Paths actual;
            collect(output.readonly(), actual);
            if (!arachne)
                reference = actual;
            else
                CHECK(actual == reference);
        }
    }
}

TEST_CASE("Concentric cancellation discards the completed unpublished group", "[plugins][infill][concentric][cancellation]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Surface surface(stPosInternal | stDensSparse, rectangle_with_hole_expolygon());
    const raw_infill_pattern_params raw = recipe(surface.expolygon, 0.5f, true, 0.f, 0);
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    slic3r_api::StoredExtrusionEntity output(handle);
    const size_t initial = storage.size();
    run_ctx_infill_pattern ctx = {};
    ctx.surface = reinterpret_cast<const surface_handle *>(&surface);
    ctx.params = &raw;
    ctx.output = output.mutable_handle();
    struct Probe { PluginStorage *storage; const extrusion_entity_handle *output; bool ready = false; } probe{&storage, output.handle()};
    plugin_run_context run = {};
    run.host_context = reinterpret_cast<plugin_host_context *>(&probe);
    run.is_cancelled = [](plugin_host_context *host) {
        Probe &probe = *reinterpret_cast<Probe *>(host);
        // Inspect a locked pointer snapshot; this single-threaded probe keeps the temporary entities alive.
        for (const ExtrusionEntity *owned : probe.storage->stored_extrusions()) {
            const slic3r_api::ExtrusionEntity entity(reinterpret_cast<const extrusion_entity_handle *>(owned));
            if (entity.handle() != probe.output && !entity.is_leaf() && !entity.empty()) {
                probe.ready = true;
                return 1;
            }
        }
        return 0;
    };
    CHECK_THROWS_AS(slic3r_api::Infill::fill_concentric_surface(ctx, handle, &run, {}), slic3r_api::PluginCancelled);
    CHECK(probe.ready);
    CHECK(output.empty());
    CHECK(storage.size() == initial);
}
