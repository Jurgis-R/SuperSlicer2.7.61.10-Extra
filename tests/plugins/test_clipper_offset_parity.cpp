/*
 * Clipper offset parity
 * ======================
 *
 * This file compares the public `slic3r_api::clipper_offset` operation with
 * the legacy `Slic3r::offset_ex` overloads.  Every API result is compared with
 * a legacy result made from the same input paths; the ExPolygon and Polygon
 * API variants are never compared with one another.
 *
 * The canonical form ignores island order, hole order, and the starting
 * vertex of each ring.  It preserves ring orientation and every coordinate,
 * so strict comparisons detect any geometric difference rather than merely a
 * different container traversal.  Inputs that overlap or touch are
 * documentary because the legacy multi-ExPolygon offset requires disjoint
 * inputs.  Other documented entries record measured strict divergences and
 * emit a warning instead of failing.
 *
 * Polygon offsets do not exercise delta zero: the legacy `offset_ex(Polygons)`
 * path asserts that the offset is non-zero.  Round-join limits are already in
 * scaled coordinate units for both implementations.
 *
 * Documented divergences
 * ----------------------
 * | case | variant | delta | join | cause |
 * |------|---------|-------|------|-------|
 * | overlapping holed squares | all | all | all | overlapping input paths are outside the legacy multi-ExPolygon contract |
 * | squares sharing an edge | all | all | all | touching input paths are outside the legacy multi-ExPolygon contract |
 * | squares sharing a vertex | all | all | all | touching input paths are outside the legacy multi-ExPolygon contract |
 * | holed deterministic cases listed below | all | negative matrix values | round | grouped and per-ring round offsets choose different integer arc vertices |
 * | random disjoint holed rectangles | all | negative sampled values | round | same grouped versus per-ring arc rounding as the deterministic holed cases |
 */

#include <catch2/catch.hpp>

#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <initializer_list>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Polygon.hpp"
#include "clipper_offset_test_helpers.hpp"

namespace {

using namespace Slic3r;

struct OffsetParams {
    double delta_mm;
    clipper_join_type_t join;
    double limit;
};

using namespace clipper_offset_test;

enum class BenchmarkMethod { Legacy, GenericApi };
enum class BenchmarkLevel { Prepared, EndToEnd };
enum class BenchmarkInputForm { ExPolygons, Polygons };
enum class BenchmarkCollectionMethod { Legacy, GenericApi };
enum class BenchmarkProfile { Fast, Full };

struct BenchmarkCenter {
    double x;
    double y;
};

struct BenchmarkFixture {
    Slic3r::ExPolygon expolygon;
    std::vector<BenchmarkCenter> centers;
};

struct BenchmarkCollectionFixture {
    Slic3r::ExPolygons expolygons;
    Slic3r::Polygons polygons;
    std::vector<BenchmarkCenter> outer_centers;
    std::vector<std::vector<BenchmarkCenter>> hole_centers;
    size_t holes_per_expolygon = 0;
};

struct BenchmarkSingleCase {
    size_t hole_count;
    size_t points_per_hole;
};

struct BenchmarkOutputStats {
    size_t islands = 0;
    size_t holes = 0;
    size_t points = 0;
    double area_mm2 = 0.;
    bool valid = true;
};

struct BenchmarkRun {
    double elapsed_ms = 0.;
    BenchmarkOutputStats output;
    Slic3r::ExPolygons native_output;
};

struct BenchmarkAggregate {
    std::vector<double> samples;
    size_t batch_calls = 1;
    double median_ms = 0.;
    double min_ms = 0.;
    double q1_ms = 0.;
    double q3_ms = 0.;
};

struct BenchmarkMethodSummary {
    BenchmarkAggregate timing;
    BenchmarkOutputStats output;
    std::string geometry_status;
};

struct BenchmarkPreparedGeneric {
    Slic3r::PluginStorage plugin_storage;
    storage_handle *storage = nullptr;
    slic3r_api::StoredExPolygonCollection collection;
    std::unique_ptr<slic3r_api::ClipperOperand> operand;
    size_t baseline_allocations = 0;

    explicit BenchmarkPreparedGeneric(const Slic3r::ExPolygon &input);
    explicit BenchmarkPreparedGeneric(const Slic3r::ExPolygons &inputs);
};

struct BenchmarkPreparedGenericPolygons {
    Slic3r::PluginStorage plugin_storage;
    storage_handle *storage = nullptr;
    slic3r_api::StoredPolygonCollection collection;
    std::unique_ptr<slic3r_api::ClipperOperand> operand;
    size_t baseline_allocations = 0;

    explicit BenchmarkPreparedGenericPolygons(const Slic3r::Polygons &inputs);
};

// Run every strict and documentary comparison for one named input family.
void run_case(const std::string &name,
              const std::vector<Slic3r::ExPolygon> &inputs,
              bool documentary);

// Build the common offset matrix and append the critical deltas for a case.
std::vector<OffsetParams> offset_matrix(std::initializer_list<double> extra_deltas_mm);

// Offset copied ExPolygon inputs through the public API and materialize them.
Slic3r::ExPolygons api_offset_expolygons(storage_handle *storage,
                                         const std::vector<Slic3r::ExPolygon> &inputs,
                                         const OffsetParams &params);

// Offset copied flat Polygon inputs through the public API and materialize them.
Slic3r::ExPolygons api_offset_polygons(storage_handle *storage,
                                       const Slic3r::Polygons &inputs,
                                       const OffsetParams &params);

// Identify strict mismatches that have been measured and intentionally documented.
bool is_documented(const std::string &name,
                   const std::string &variant,
                   double delta_mm,
                   clipper_join_type_t join,
                   double limit);

// Build the deterministic hexagonal hole placement used by the benchmark.
std::vector<BenchmarkCenter> benchmark_centers();

// Build hexagonal centers up to a requested radial distance.
std::vector<BenchmarkCenter> benchmark_centers_in_radius(double max_radius);

// Build one closed circle with project-scaled integer coordinates.
Slic3r::Polygon benchmark_circle(double center_x,
                                 double center_y,
                                 double radius,
                                 size_t point_count,
                                 bool clockwise);

// Build one benchmark ExPolygon from a stable center subset and resolution.
BenchmarkFixture benchmark_fixture(const std::vector<BenchmarkCenter> &centers,
                                   size_t hole_count,
                                   size_t points_per_hole);

// Build eight disjoint holed islands with sixteen holes per island.
BenchmarkCollectionFixture benchmark_collection_fixture(
    const std::vector<BenchmarkCenter> &local_centers,
    size_t points_per_hole);

// Validate the generated fixture before any timed operation.
void validate_benchmark_fixture(const BenchmarkFixture &fixture,
                                size_t expected_holes,
                                size_t points_per_hole);

// Validate the multi-island and flattened polygon benchmark inputs.
void validate_benchmark_collection_fixture(const BenchmarkCollectionFixture &fixture,
                                           size_t points_per_hole);

// Return a stable method label for benchmark output.
const char *benchmark_method_name(BenchmarkMethod method);

// Return a stable measurement-level label for benchmark output.
const char *benchmark_level_name(BenchmarkLevel level);

// Return a stable input-form label for collection benchmark output.
const char *benchmark_input_form_name(BenchmarkInputForm form);

// Return a stable method label for collection benchmark output.
const char *benchmark_collection_method_name(BenchmarkCollectionMethod method);

// Collect output topology, point count, area and validity.
BenchmarkOutputStats benchmark_output_stats(const Slic3r::ExPolygons &expolygons);

// Classify output equality against the legacy reference.
std::string benchmark_geometry_status(const Slic3r::ExPolygons &reference,
                                      const Slic3r::ExPolygons &candidate);

// Run one prepared legacy offset without timing result destruction.
BenchmarkRun benchmark_prepared_legacy(const Slic3r::ExPolygon &input,
                                       const OffsetParams &params);

// Run one prepared generic API offset without native reconstruction in timing.
BenchmarkRun benchmark_prepared_generic(BenchmarkPreparedGeneric &prepared,
                                        const OffsetParams &params);

// Run one end-to-end legacy offset on the native input.
BenchmarkRun benchmark_end_to_end_legacy(const Slic3r::ExPolygon &input,
                                         const OffsetParams &params);

// Run the current generic API helper including copies and native reconstruction.
BenchmarkRun benchmark_end_to_end_generic(const Slic3r::ExPolygons &inputs,
                                           const OffsetParams &params);

// Run the current generic API polygon helper including copies and reconstruction.
BenchmarkRun benchmark_end_to_end_generic_polygons(const Slic3r::Polygons &inputs,
                                                  const OffsetParams &params);

// Run a prepared legacy multi-ExPolygon offset.
BenchmarkRun benchmark_prepared_legacy_expolygons(const Slic3r::ExPolygons &inputs,
                                                  const OffsetParams &params);

// Run a prepared legacy flat-Polygon offset.
BenchmarkRun benchmark_prepared_legacy_polygons(const Slic3r::Polygons &inputs,
                                                const OffsetParams &params);

// Run a prepared generic flat-Polygon API offset.
BenchmarkRun benchmark_prepared_generic_polygons(BenchmarkPreparedGenericPolygons &prepared,
                                                 const OffsetParams &params);

// Run a complete legacy multi-ExPolygon offset.
BenchmarkRun benchmark_end_to_end_legacy_expolygons(const Slic3r::ExPolygons &inputs,
                                                    const OffsetParams &params);

// Dispatch one collection benchmark method at one measurement level.
BenchmarkRun benchmark_collection_run_method(BenchmarkInputForm form,
                                             BenchmarkCollectionMethod method,
                                             BenchmarkLevel level,
                                             const BenchmarkCollectionFixture &fixture,
                                             const OffsetParams &params,
                                             BenchmarkPreparedGeneric *prepared_expolygons,
                                             BenchmarkPreparedGenericPolygons *prepared_polygons);

// Dispatch one benchmark method at one measurement level.
BenchmarkRun benchmark_run_method(BenchmarkMethod method,
                                  BenchmarkLevel level,
                                  const BenchmarkFixture &fixture,
                                  const Slic3r::ExPolygons &inputs,
                                  const OffsetParams &params,
                                  BenchmarkPreparedGeneric *prepared_generic);

// Select a small batch only when the pilot call is too short to time reliably.
size_t benchmark_batch_calls(double pilot_ms);

// Aggregate configured per-call measurements.
BenchmarkAggregate benchmark_aggregate(std::vector<double> samples, size_t batch_calls);

// Validate one benchmark output against its matching legacy form.
std::string validate_benchmark_run(const BenchmarkRun &run,
                                   const Slic3r::ExPolygons &legacy_reference);

// Return a stable benchmark profile label.
const char *benchmark_profile_name(BenchmarkProfile profile);

// Return the number of warmups selected by the benchmark profile.
size_t benchmark_warmup_count(BenchmarkProfile profile);

// Return the number of measured samples selected by the benchmark profile.
size_t benchmark_sample_count(BenchmarkProfile profile);

// Build the benchmark delta and join matrix selected by the profile.
std::vector<OffsetParams> benchmark_matrix(BenchmarkProfile profile);

// Build the single-ExPolygon matrix selected for one fixture size.
std::vector<OffsetParams> benchmark_single_matrix(BenchmarkProfile profile,
                                                   size_t hole_count,
                                                   size_t points_per_hole);

// Build the collection matrix selected by the profile.
std::vector<OffsetParams> benchmark_collection_matrix(BenchmarkProfile profile);

// Build the single-ExPolygon fixture sizes selected by the profile.
std::vector<BenchmarkSingleCase> benchmark_single_cases(BenchmarkProfile profile);

// Build the collection resolutions selected by the profile.
std::vector<size_t> benchmark_collection_point_resolutions(BenchmarkProfile profile);

// Emit one structured timing line.
void print_benchmark_row(size_t hole_count,
                         size_t points_per_hole,
                         const OffsetParams &params,
                         BenchmarkLevel level,
                         BenchmarkMethod method,
                         BenchmarkProfile profile,
                         const BenchmarkMethodSummary &summary);

// Emit one method/reference median ratio line.
void print_benchmark_ratio(size_t hole_count,
                           size_t points_per_hole,
                           const OffsetParams &params,
                           BenchmarkLevel level,
                           BenchmarkMethod method,
                           BenchmarkProfile profile,
                           const BenchmarkMethodSummary &method_summary,
                           const BenchmarkMethodSummary &legacy_summary);

// Emit one structured collection benchmark timing line.
void print_collection_benchmark_row(const BenchmarkCollectionFixture &fixture,
                                    BenchmarkInputForm form,
                                    const OffsetParams &params,
                                    BenchmarkLevel level,
                                    BenchmarkCollectionMethod method,
                                    BenchmarkProfile profile,
                                    const BenchmarkMethodSummary &summary,
                                    size_t points_per_hole);

// Emit one collection method/reference median ratio line.
void print_collection_benchmark_ratio(const BenchmarkCollectionFixture &fixture,
                                      BenchmarkInputForm form,
                                      const OffsetParams &params,
                                      BenchmarkLevel level,
                                      BenchmarkCollectionMethod method,
                                      BenchmarkProfile profile,
                                      const BenchmarkMethodSummary &method_summary,
                                      const BenchmarkMethodSummary &legacy_summary,
                                      size_t points_per_hole);

// Execute all warmups, samples and validations for one configuration.
void run_benchmark_configuration(const BenchmarkFixture &fixture,
                                 const Slic3r::ExPolygons &inputs,
                                 size_t hole_count,
                                 size_t points_per_hole,
                                 const OffsetParams &params,
                                 BenchmarkProfile profile);

// Execute all collection-form warmups, samples and validations.
void run_collection_benchmark_configuration(const BenchmarkCollectionFixture &fixture,
                                            const OffsetParams &params,
                                            size_t points_per_hole,
                                            BenchmarkProfile profile);

// Execute the selected single-ExPolygon benchmark profile.
void run_single_benchmark(BenchmarkProfile profile);

// Execute the selected collection benchmark profile.
void run_collection_benchmark(BenchmarkProfile profile);

void run_case(const std::string &name,
              const std::vector<Slic3r::ExPolygon> &inputs,
              bool documentary)
{
    Slic3r::PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    std::vector<OffsetParams> matrix;
    if (name == "Square with hole")
        matrix = offset_matrix({-1.9, 1.9});
    else if (name == "Two close holes")
        matrix = offset_matrix({-0.4, -0.6});
    else if (name == "Holed square inside a hole")
        matrix = offset_matrix({-2.4, 2.4, 4.9});
    else if (name == "Close disjoint squares")
        matrix = offset_matrix({0.25, 0.3});
    else if (name == "Hole near the contour")
        matrix = offset_matrix({-0.2, -0.4});
    else if (name == "Vanishing hole")
        matrix = offset_matrix({0.9, 1.1});
    else if (name == "Dumbbell")
        matrix = offset_matrix({-0.05, -0.1, -0.15});
    else if (name == "Collapsing square" || name == "Empty collection" || name == "Empty ExPolygon")
        matrix = offset_matrix({-1.5, -5.});
    else
        matrix = offset_matrix({});
    std::vector<Slic3r::ExPolygon> ordered_inputs(inputs);
    std::vector<size_t> ordered_indices;
    for (size_t index = 0; index < inputs.size(); ++index)
        ordered_indices.push_back(index);

    const size_t order_count = inputs.size() < 2 ? 1 : inputs.size();
    for (size_t order = 0; order < order_count; ++order) {
        if (order > 0) {
            std::rotate(ordered_inputs.begin(), ordered_inputs.begin() + 1, ordered_inputs.end());
            std::rotate(ordered_indices.begin(), ordered_indices.begin() + 1, ordered_indices.end());
        }

        std::ostringstream order_info;
        order_info << name << " order=";
        if (ordered_indices.empty())
            order_info << "empty";
        else
            for (size_t index : ordered_indices)
                order_info << index;

        for (const OffsetParams &params : matrix) {
            const double delta = scale_d(params.delta_mm);
            const ClipperLib::JoinType join = legacy_join(params.join);
            std::ostringstream info;
            info << order_info.str() << " variant=expolygons delta_mm=" << params.delta_mm
                 << " join=" << join_name(params.join) << " limit=" << params.limit;

            const Slic3r::ExPolygons api = api_offset_expolygons(storage, ordered_inputs, params);
            const Slic3r::ExPolygons legacy = Slic3r::offset_ex(ordered_inputs, delta, join, params.limit);
            compare(info.str(), api, legacy,
                    documentary || is_documented(name, "expolygons", params.delta_mm, params.join, params.limit));

            if (ordered_inputs.size() == 1) {
                std::ostringstream single_info;
                single_info << order_info.str() << " variant=single-expolygon delta_mm=" << params.delta_mm
                            << " join=" << join_name(params.join) << " limit=" << params.limit;
                const Slic3r::ExPolygons single_legacy =
                    Slic3r::offset_ex(ordered_inputs.front(), delta, join, params.limit);
                compare(single_info.str(), api, single_legacy,
                        documentary || is_documented(name, "single-expolygon", params.delta_mm,
                                                     params.join, params.limit));
            }

            if (params.delta_mm != 0.) {
                const Slic3r::Polygons polygons = needs_manual_polygon_conversion(ordered_inputs) ?
                    flatten_polygons(ordered_inputs) : Slic3r::to_polygons(ordered_inputs);
                std::ostringstream polygon_info;
                polygon_info << order_info.str() << " variant=polygons delta_mm=" << params.delta_mm
                             << " join=" << join_name(params.join) << " limit=" << params.limit;
                const Slic3r::ExPolygons api_polygons = api_offset_polygons(storage, polygons, params);
                const Slic3r::ExPolygons legacy_polygons =
                    Slic3r::offset_ex(polygons, delta, join, params.limit);
                compare(polygon_info.str(), api_polygons, legacy_polygons,
                        documentary || is_documented(name, "polygons", params.delta_mm, params.join,
                                                     params.limit));
            }
        }
    }
}

std::vector<OffsetParams> offset_matrix(std::initializer_list<double> extra_deltas_mm)
{
    const double round_limit = scale_d(0.005);
    std::vector<double> deltas{0., -0.1, 0.1, -0.225, 0.225, -1., 1., -5., 5.};
    for (const double delta : extra_deltas_mm)
        if (std::find(deltas.begin(), deltas.end(), delta) == deltas.end())
            deltas.push_back(delta);

    std::vector<OffsetParams> result;
    for (const double delta : deltas) {
        result.push_back({delta, CLIPPER_JOIN_MITER, 1.2});
        result.push_back({delta, CLIPPER_JOIN_MITER, 3.});
        result.push_back({delta, CLIPPER_JOIN_MITER, 10.});
        result.push_back({delta, CLIPPER_JOIN_SQUARE, 3.});
        result.push_back({delta, CLIPPER_JOIN_ROUND, round_limit});
    }
    return result;
}

Slic3r::ExPolygons api_offset_expolygons(storage_handle *storage,
                                         const std::vector<Slic3r::ExPolygon> &inputs,
                                         const OffsetParams &params)
{
    slic3r_api::StoredExPolygonCollection collection(storage);
    for (const Slic3r::ExPolygon &input : inputs) {
        slic3r_api::StoredExPolygon stored(storage);
        stored.copy_from(view(input));
        collection.push_back_move(std::move(stored));
    }

    slic3r_api::ClipperOperand operand(storage, collection.readonly());
    slic3r_api::ClipperOperand offset =
        slic3r_api::clipper_offset(operand, scale_d(params.delta_mm), params.join, params.limit);
    slic3r_api::StoredExPolygonCollection result = offset.to_expolygon_collection();
    return to_native(result.readonly());
}

Slic3r::ExPolygons api_offset_polygons(storage_handle *storage,
                                       const Slic3r::Polygons &inputs,
                                       const OffsetParams &params)
{
    slic3r_api::StoredPolygonCollection collection(storage);
    for (const Slic3r::Polygon &input : inputs) {
        slic3r_api::StoredPolygon stored(storage);
        stored.copy_from(view(input));
        collection.push_back_move(std::move(stored));
    }

    slic3r_api::ClipperOperand operand(storage, collection.readonly());
    slic3r_api::ClipperOperand offset =
        slic3r_api::clipper_offset(operand, scale_d(params.delta_mm), params.join, params.limit);
    slic3r_api::StoredExPolygonCollection result = offset.to_expolygon_collection();
    return to_native(result.readonly());
}

bool is_documented(const std::string &name,
                   const std::string &variant,
                   double delta_mm,
                   clipper_join_type_t join,
                   double limit)
{
    (void)variant;
    (void)limit;
    if (name == "Overlapping holed squares" ||
        name == "Squares sharing an edge" ||
        name == "Squares sharing a vertex")
        return true;
    if (delta_mm < 0. && join == CLIPPER_JOIN_ROUND &&
        (name == "Square with hole" ||
         name == "Two close holes" ||
         name == "Holed square inside a hole" ||
         name == "Hole near the contour" ||
         name == "Vanishing hole" ||
         name == "Rotated holed square" ||
         name == "Three nesting levels" ||
         name == "Negative coordinates" ||
         name == "Large positive coordinates" ||
         name.find("Shifted start vertex ") == 0 ||
         name == "Random disjoint holed rectangles"))
        return true;
    return false;
}

std::vector<BenchmarkCenter> benchmark_centers_in_radius(double max_radius)
{
    const double spacing = 8.;
    const double row_step = spacing * std::sqrt(3.) / 2.;
    std::vector<BenchmarkCenter> candidates;
    for (int row = -20; row <= 20; ++row) {
        const double y = row * row_step;
        const double x_shift = (row & 1) == 0 ? 0. : spacing / 2.;
        for (int column = -20; column <= 20; ++column) {
            const double x = column * spacing + x_shift;
            if (std::sqrt(x * x + y * y) <= max_radius)
                candidates.push_back({x, y});
        }
    }

    std::sort(candidates.begin(), candidates.end(), [](const BenchmarkCenter &first,
                                                       const BenchmarkCenter &second) {
        const double first_radius = first.x * first.x + first.y * first.y;
        const double second_radius = second.x * second.x + second.y * second.y;
        if (first_radius != second_radius)
            return first_radius < second_radius;
        if (first.x != second.x)
            return first.x < second.x;
        return first.y < second.y;
    });
    return candidates;
}

std::vector<BenchmarkCenter> benchmark_centers()
{
    return benchmark_centers_in_radius(55.);
}

Slic3r::Polygon benchmark_circle(double center_x,
                                 double center_y,
                                 double radius,
                                 size_t point_count,
                                 bool clockwise)
{
    const double pi = std::acos(-1.);
    Slic3r::Polygon result;
    result.points.reserve(point_count);
    for (size_t point = 0; point < point_count; ++point) {
        const double angle = 2. * pi * static_cast<double>(point) / static_cast<double>(point_count);
        result.points.emplace_back(scale_i(center_x + radius * std::cos(angle)),
                                   scale_i(center_y + radius * std::sin(angle)));
    }
    if (clockwise)
        result.make_clockwise();
    else
        result.make_counter_clockwise();
    return result;
}

BenchmarkFixture benchmark_fixture(const std::vector<BenchmarkCenter> &centers,
                                   size_t hole_count,
                                   size_t points_per_hole)
{
    BenchmarkFixture result;
    result.centers.assign(centers.begin(), centers.begin() + hole_count);
    result.expolygon = Slic3r::ExPolygon(benchmark_circle(0., 0., 60., 256, false));
    for (const BenchmarkCenter &center : result.centers)
        result.expolygon.holes.push_back(benchmark_circle(center.x, center.y, 2., points_per_hole, true));
    return result;
}

BenchmarkCollectionFixture benchmark_collection_fixture(
    const std::vector<BenchmarkCenter> &local_centers,
    size_t points_per_hole)
{
    BenchmarkCollectionFixture result;
    result.holes_per_expolygon = 16;
    result.outer_centers = {
        {-120., -40.}, {-40., -40.}, {40., -40.}, {120., -40.},
        {-120., 40.}, {-40., 40.}, {40., 40.}, {120., 40.}
    };
    result.expolygons.reserve(result.outer_centers.size());
    result.hole_centers.reserve(result.outer_centers.size());
    for (const BenchmarkCenter &outer_center : result.outer_centers) {
        Slic3r::ExPolygon expolygon(benchmark_circle(
            outer_center.x, outer_center.y, 30., 256, false));
        std::vector<BenchmarkCenter> hole_centers;
        hole_centers.reserve(result.holes_per_expolygon);
        for (size_t hole_index = 0; hole_index < result.holes_per_expolygon; ++hole_index) {
            const BenchmarkCenter &local_center = local_centers[hole_index];
            const BenchmarkCenter center{
                outer_center.x + local_center.x,
                outer_center.y + local_center.y};
            hole_centers.push_back(center);
            expolygon.holes.push_back(benchmark_circle(center.x, center.y, 2., points_per_hole, true));
        }
        result.hole_centers.push_back(std::move(hole_centers));
        result.expolygons.push_back(std::move(expolygon));
    }
    result.polygons = Slic3r::to_polygons(result.expolygons);
    return result;
}

void validate_benchmark_fixture(const BenchmarkFixture &fixture,
                                size_t expected_holes,
                                size_t points_per_hole)
{
    const double scale = scale_d(1.);
    CHECK(fixture.expolygon.contour.points.size() == 256);
    CHECK(fixture.expolygon.holes.size() == expected_holes);
    CHECK(fixture.expolygon.is_valid());
    CHECK(fixture.expolygon.contour.is_counter_clockwise());

    for (size_t point = 0; point < fixture.expolygon.contour.points.size(); ++point) {
        const Slic3r::Point &current = fixture.expolygon.contour.points[point];
        const Slic3r::Point &next = fixture.expolygon.contour.points[(point + 1) % fixture.expolygon.contour.points.size()];
        CHECK(current != next);
        const double x = static_cast<double>(current.x()) / scale;
        const double y = static_cast<double>(current.y()) / scale;
        CHECK(std::sqrt(x * x + y * y) <= 60.001);
    }

    for (size_t first = 0; first < fixture.centers.size(); ++first) {
        for (size_t second = first + 1; second < fixture.centers.size(); ++second) {
            const double dx = fixture.centers[first].x - fixture.centers[second].x;
            const double dy = fixture.centers[first].y - fixture.centers[second].y;
            CHECK(std::sqrt(dx * dx + dy * dy) >= 8. - 1e-9);
        }
    }

    for (size_t hole_index = 0; hole_index < fixture.expolygon.holes.size(); ++hole_index) {
        const Slic3r::Polygon &hole = fixture.expolygon.holes[hole_index];
        const BenchmarkCenter &center = fixture.centers[hole_index];
        CHECK(hole.points.size() == points_per_hole);
        CHECK(hole.is_clockwise());
        CHECK(fixture.expolygon.contour.contains(hole.front()));
        CHECK(std::sqrt(center.x * center.x + center.y * center.y) + 2. <= 57.);
        for (size_t point = 0; point < hole.points.size(); ++point) {
            const Slic3r::Point &current = hole.points[point];
            const Slic3r::Point &next = hole.points[(point + 1) % hole.points.size()];
            CHECK(current != next);
            const double x = static_cast<double>(current.x()) / scale - center.x;
            const double y = static_cast<double>(current.y()) / scale - center.y;
            CHECK(std::sqrt(x * x + y * y) <= 2.001);
        }
    }
}

void validate_benchmark_collection_fixture(const BenchmarkCollectionFixture &fixture,
                                           size_t points_per_hole)
{
    CHECK(fixture.expolygons.size() == 8);
    CHECK(fixture.outer_centers.size() == fixture.expolygons.size());
    CHECK(fixture.hole_centers.size() == fixture.expolygons.size());
    CHECK(fixture.holes_per_expolygon == 16);
    CHECK(fixture.polygons.size() == fixture.expolygons.size() * (1 + fixture.holes_per_expolygon));

    for (size_t island = 0; island < fixture.expolygons.size(); ++island) {
        const Slic3r::ExPolygon &expolygon = fixture.expolygons[island];
        CHECK(expolygon.contour.points.size() == 256);
        CHECK(expolygon.holes.size() == fixture.holes_per_expolygon);
        CHECK(expolygon.is_valid());
        CHECK(expolygon.contour.is_counter_clockwise());
        for (size_t hole_index = 0; hole_index < expolygon.holes.size(); ++hole_index) {
            const Slic3r::Polygon &hole = expolygon.holes[hole_index];
            const BenchmarkCenter &center = fixture.hole_centers[island][hole_index];
            const BenchmarkCenter &outer_center = fixture.outer_centers[island];
            const double local_x = center.x - outer_center.x;
            const double local_y = center.y - outer_center.y;
            CHECK(hole.points.size() == points_per_hole);
            CHECK(hole.is_clockwise());
            CHECK(expolygon.contour.contains(hole.front()));
            CHECK(std::sqrt(local_x * local_x + local_y * local_y) + 2. <= 27.001);
        }
    }

    for (size_t first = 0; first < fixture.outer_centers.size(); ++first) {
        for (size_t second = first + 1; second < fixture.outer_centers.size(); ++second) {
            const double dx = fixture.outer_centers[first].x - fixture.outer_centers[second].x;
            const double dy = fixture.outer_centers[first].y - fixture.outer_centers[second].y;
            CHECK(std::sqrt(dx * dx + dy * dy) >= 60.);
        }
    }
}

const char *benchmark_profile_name(BenchmarkProfile profile)
{
    return profile == BenchmarkProfile::Fast ? "fast" : "full";
}

size_t benchmark_warmup_count(BenchmarkProfile profile)
{
    return profile == BenchmarkProfile::Fast ? 1 : 2;
}

size_t benchmark_sample_count(BenchmarkProfile profile)
{
    return profile == BenchmarkProfile::Fast ? 3 : 9;
}

std::vector<OffsetParams> benchmark_matrix(BenchmarkProfile profile)
{
    const double round_limit = scale_d(0.005);
    std::vector<OffsetParams> result;
    if (profile == BenchmarkProfile::Fast) {
        result = {
            {0., CLIPPER_JOIN_MITER, 3.},
            {-2.1, CLIPPER_JOIN_MITER, 3.}
        };
        return result;
    }

    const std::vector<double> deltas{0., -0.225, 0.225, -1.9, 1.9, -2.1, 2.1};
    result.reserve(deltas.size() * 5);
    for (const double delta : deltas) {
        result.push_back({delta, CLIPPER_JOIN_MITER, 1.2});
        result.push_back({delta, CLIPPER_JOIN_MITER, 3.});
        result.push_back({delta, CLIPPER_JOIN_MITER, 10.});
        result.push_back({delta, CLIPPER_JOIN_SQUARE, 3.});
        result.push_back({delta, CLIPPER_JOIN_ROUND, round_limit});
    }
    return result;
}

std::vector<OffsetParams> benchmark_single_matrix(BenchmarkProfile profile,
                                                  size_t hole_count,
                                                  size_t points_per_hole)
{
    std::vector<OffsetParams> result = benchmark_matrix(profile);
    if (profile == BenchmarkProfile::Full)
        return result;

    const double round_limit = scale_d(0.005);
    if (hole_count == 0) {
        result.push_back({-0.225, CLIPPER_JOIN_MITER, 3.});
        result.push_back({0.225, CLIPPER_JOIN_MITER, 3.});
        result.push_back({2.1, CLIPPER_JOIN_MITER, 1.2});
    } else if (hole_count == 8 && points_per_hole == 64) {
        result.push_back({2.1, CLIPPER_JOIN_MITER, 3.});
        result.push_back({2.1, CLIPPER_JOIN_SQUARE, 3.});
        result.push_back({2.1, CLIPPER_JOIN_ROUND, round_limit});
    }
    return result;
}

std::vector<OffsetParams> benchmark_collection_matrix(BenchmarkProfile profile)
{
    std::vector<OffsetParams> result = benchmark_matrix(profile);
    if (profile == BenchmarkProfile::Fast)
        result.push_back({0.225, CLIPPER_JOIN_MITER, 3.});
    return result;
}

std::vector<BenchmarkSingleCase> benchmark_single_cases(BenchmarkProfile profile)
{
    if (profile == BenchmarkProfile::Fast)
        return {{0, 64}, {8, 64}, {32, 128}, {128, 256}};

    const std::vector<size_t> hole_counts{0, 8, 32, 128};
    const std::vector<size_t> point_resolutions{64, 128, 256};
    std::vector<BenchmarkSingleCase> result;
    result.reserve(hole_counts.size() * point_resolutions.size());
    for (const size_t hole_count : hole_counts)
        for (const size_t points_per_hole : point_resolutions)
            result.push_back({hole_count, points_per_hole});
    return result;
}

std::vector<size_t> benchmark_collection_point_resolutions(BenchmarkProfile profile)
{
    if (profile == BenchmarkProfile::Fast)
        return {64};
    return {64, 128, 256};
}

const char *benchmark_method_name(BenchmarkMethod method)
{
    switch (method) {
    case BenchmarkMethod::Legacy:
        return "L";
    case BenchmarkMethod::GenericApi:
        return "G";
    }
    return "?";
}

const char *benchmark_level_name(BenchmarkLevel level)
{
    return level == BenchmarkLevel::Prepared ? "A" : "B";
}

const char *benchmark_input_form_name(BenchmarkInputForm form)
{
    return form == BenchmarkInputForm::ExPolygons ? "expolygons" : "polygons";
}

const char *benchmark_collection_method_name(BenchmarkCollectionMethod method)
{
    return method == BenchmarkCollectionMethod::Legacy ? "L" : "G";
}

BenchmarkOutputStats benchmark_output_stats(const Slic3r::ExPolygons &expolygons)
{
    const double scale = scale_d(1.);
    BenchmarkOutputStats result;
    result.islands = expolygons.size();
    for (const Slic3r::ExPolygon &expolygon : expolygons) {
        result.holes += expolygon.holes.size();
        result.points += expolygon.contour.points.size();
        for (const Slic3r::Polygon &hole : expolygon.holes)
            result.points += hole.points.size();
        result.area_mm2 += std::abs(expolygon.area()) / (scale * scale);
        result.valid = result.valid && expolygon.is_valid();
    }
    return result;
}

std::string benchmark_geometry_status(const Slic3r::ExPolygons &reference,
                                      const Slic3r::ExPolygons &candidate)
{
    if (canonical(reference) == canonical(candidate))
        return "exact";
    if (!benchmark_output_stats(candidate).valid)
        return "invalid";

    std::vector<size_t> reference_holes;
    std::vector<size_t> candidate_holes;
    for (const Slic3r::ExPolygon &expolygon : reference)
        reference_holes.push_back(expolygon.holes.size());
    for (const Slic3r::ExPolygon &expolygon : candidate)
        candidate_holes.push_back(expolygon.holes.size());
    std::sort(reference_holes.begin(), reference_holes.end());
    std::sort(candidate_holes.begin(), candidate_holes.end());
    return reference_holes == candidate_holes ? "coordinates" : "topology";
}

BenchmarkPreparedGeneric::BenchmarkPreparedGeneric(const Slic3r::ExPolygon &input) :
    BenchmarkPreparedGeneric(Slic3r::ExPolygons{input})
{
}

BenchmarkPreparedGeneric::BenchmarkPreparedGeneric(const Slic3r::ExPolygons &inputs) :
    plugin_storage(),
    storage(reinterpret_cast<storage_handle *>(&plugin_storage)),
    collection(storage)
{
    for (const Slic3r::ExPolygon &input : inputs) {
        slic3r_api::StoredExPolygon stored(storage);
        stored.copy_from(view(input));
        collection.push_back_move(std::move(stored));
    }
    operand.reset(new slic3r_api::ClipperOperand(storage, collection.readonly()));
    baseline_allocations = plugin_storage.size();
}

BenchmarkPreparedGenericPolygons::BenchmarkPreparedGenericPolygons(const Slic3r::Polygons &inputs) :
    plugin_storage(),
    storage(reinterpret_cast<storage_handle *>(&plugin_storage)),
    collection(storage)
{
    for (const Slic3r::Polygon &input : inputs) {
        slic3r_api::StoredPolygon stored(storage);
        stored.copy_from(view(input));
        collection.push_back_move(std::move(stored));
    }
    operand.reset(new slic3r_api::ClipperOperand(storage, collection.readonly()));
    baseline_allocations = plugin_storage.size();
}

BenchmarkRun benchmark_prepared_legacy(const Slic3r::ExPolygon &input,
                                       const OffsetParams &params)
{
    const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    Slic3r::ExPolygons output =
        Slic3r::offset_ex(input, scale_d(params.delta_mm), legacy_join(params.join), params.limit);
    const std::chrono::steady_clock::time_point finish = std::chrono::steady_clock::now();
    BenchmarkRun result;
    result.elapsed_ms = std::chrono::duration<double, std::milli>(finish - start).count();
    result.output = benchmark_output_stats(output);
    result.native_output = std::move(output);
    return result;
}

BenchmarkRun benchmark_prepared_legacy_expolygons(const Slic3r::ExPolygons &inputs,
                                                  const OffsetParams &params)
{
    const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    Slic3r::ExPolygons output =
        Slic3r::offset_ex(inputs, scale_d(params.delta_mm), legacy_join(params.join), params.limit);
    const std::chrono::steady_clock::time_point finish = std::chrono::steady_clock::now();
    BenchmarkRun result;
    result.elapsed_ms = std::chrono::duration<double, std::milli>(finish - start).count();
    result.output = benchmark_output_stats(output);
    result.native_output = std::move(output);
    return result;
}

BenchmarkRun benchmark_prepared_legacy_polygons(const Slic3r::Polygons &inputs,
                                                const OffsetParams &params)
{
    const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    Slic3r::ExPolygons output =
        Slic3r::offset_ex(inputs, scale_d(params.delta_mm), legacy_join(params.join), params.limit);
    const std::chrono::steady_clock::time_point finish = std::chrono::steady_clock::now();
    BenchmarkRun result;
    result.elapsed_ms = std::chrono::duration<double, std::milli>(finish - start).count();
    result.output = benchmark_output_stats(output);
    result.native_output = std::move(output);
    return result;
}

BenchmarkRun benchmark_prepared_generic(BenchmarkPreparedGeneric &prepared,
                                        const OffsetParams &params)
{
    Slic3r::ExPolygons output;
    const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    {
        slic3r_api::ClipperOperand offset = slic3r_api::clipper_offset(
            *prepared.operand, scale_d(params.delta_mm), params.join, params.limit,
            CLIPPER_END_CLOSED_POLYGON);
        slic3r_api::StoredExPolygonCollection collection = offset.to_expolygon_collection();
        const std::chrono::steady_clock::time_point finish = std::chrono::steady_clock::now();
        output = to_native(collection.readonly());
        BenchmarkRun result;
        result.elapsed_ms = std::chrono::duration<double, std::milli>(finish - start).count();
        result.output = benchmark_output_stats(output);
        result.native_output = std::move(output);
        return result;
    }
}

BenchmarkRun benchmark_prepared_generic_polygons(BenchmarkPreparedGenericPolygons &prepared,
                                                 const OffsetParams &params)
{
    Slic3r::ExPolygons output;
    const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    {
        slic3r_api::ClipperOperand offset = slic3r_api::clipper_offset(
            *prepared.operand, scale_d(params.delta_mm), params.join, params.limit,
            CLIPPER_END_CLOSED_POLYGON);
        slic3r_api::StoredExPolygonCollection collection = offset.to_expolygon_collection();
        const std::chrono::steady_clock::time_point finish = std::chrono::steady_clock::now();
        output = to_native(collection.readonly());
        BenchmarkRun result;
        result.elapsed_ms = std::chrono::duration<double, std::milli>(finish - start).count();
        result.output = benchmark_output_stats(output);
        result.native_output = std::move(output);
        return result;
    }
}

BenchmarkRun benchmark_end_to_end_legacy(const Slic3r::ExPolygon &input,
                                         const OffsetParams &params)
{
    return benchmark_prepared_legacy(input, params);
}

BenchmarkRun benchmark_end_to_end_generic(const Slic3r::ExPolygons &inputs,
                                          const OffsetParams &params)
{
    Slic3r::PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    Slic3r::ExPolygons output = api_offset_expolygons(storage, inputs, params);
    const std::chrono::steady_clock::time_point finish = std::chrono::steady_clock::now();
    CHECK(plugin_storage.size() == 0);
    BenchmarkRun result;
    result.elapsed_ms = std::chrono::duration<double, std::milli>(finish - start).count();
    result.output = benchmark_output_stats(output);
    result.native_output = std::move(output);
    return result;
}

BenchmarkRun benchmark_end_to_end_generic_polygons(const Slic3r::Polygons &inputs,
                                                  const OffsetParams &params)
{
    Slic3r::PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    Slic3r::ExPolygons output = api_offset_polygons(storage, inputs, params);
    const std::chrono::steady_clock::time_point finish = std::chrono::steady_clock::now();
    CHECK(plugin_storage.size() == 0);
    BenchmarkRun result;
    result.elapsed_ms = std::chrono::duration<double, std::milli>(finish - start).count();
    result.output = benchmark_output_stats(output);
    result.native_output = std::move(output);
    return result;
}

BenchmarkRun benchmark_end_to_end_legacy_expolygons(const Slic3r::ExPolygons &inputs,
                                                    const OffsetParams &params)
{
    return benchmark_prepared_legacy_expolygons(inputs, params);
}

BenchmarkRun benchmark_collection_run_method(BenchmarkInputForm form,
                                             BenchmarkCollectionMethod method,
                                             BenchmarkLevel level,
                                             const BenchmarkCollectionFixture &fixture,
                                             const OffsetParams &params,
                                             BenchmarkPreparedGeneric *prepared_expolygons,
                                             BenchmarkPreparedGenericPolygons *prepared_polygons)
{
    if (form == BenchmarkInputForm::ExPolygons) {
        if (level == BenchmarkLevel::Prepared) {
            if (method == BenchmarkCollectionMethod::Legacy)
                return benchmark_prepared_legacy_expolygons(fixture.expolygons, params);
            assert(prepared_expolygons != nullptr);
            return benchmark_prepared_generic(*prepared_expolygons, params);
        }
        if (method == BenchmarkCollectionMethod::Legacy)
            return benchmark_end_to_end_legacy_expolygons(fixture.expolygons, params);
        return benchmark_end_to_end_generic(fixture.expolygons, params);
    }

    if (level == BenchmarkLevel::Prepared) {
        if (method == BenchmarkCollectionMethod::Legacy)
            return benchmark_prepared_legacy_polygons(fixture.polygons, params);
        assert(prepared_polygons != nullptr);
        return benchmark_prepared_generic_polygons(*prepared_polygons, params);
    }
    if (method == BenchmarkCollectionMethod::Legacy)
        return benchmark_prepared_legacy_polygons(fixture.polygons, params);
    return benchmark_end_to_end_generic_polygons(fixture.polygons, params);
}

BenchmarkRun benchmark_run_method(BenchmarkMethod method,
                                  BenchmarkLevel level,
                                  const BenchmarkFixture &fixture,
                                  const Slic3r::ExPolygons &inputs,
                                  const OffsetParams &params,
                                  BenchmarkPreparedGeneric *prepared_generic)
{
    if (level == BenchmarkLevel::Prepared) {
        if (method == BenchmarkMethod::Legacy)
            return benchmark_prepared_legacy(fixture.expolygon, params);
        assert(prepared_generic != nullptr);
        return benchmark_prepared_generic(*prepared_generic, params);
    }

    if (method == BenchmarkMethod::Legacy)
        return benchmark_end_to_end_legacy(fixture.expolygon, params);
    return benchmark_end_to_end_generic(inputs, params);
}

size_t benchmark_batch_calls(double pilot_ms)
{
    if (pilot_ms >= 0.05)
        return 1;
    const double safe_ms = std::max(pilot_ms, 0.000001);
    const size_t calls = static_cast<size_t>(std::ceil(0.5 / safe_ms));
    return std::min<size_t>(8, std::max<size_t>(2, calls));
}

BenchmarkAggregate benchmark_aggregate(std::vector<double> samples, size_t batch_calls)
{
    assert(!samples.empty());
    std::sort(samples.begin(), samples.end());
    BenchmarkAggregate result;
    result.samples = std::move(samples);
    result.batch_calls = batch_calls;
    result.min_ms = result.samples.front();
    result.median_ms = result.samples[result.samples.size() / 2];
    result.q1_ms = result.samples[result.samples.size() / 4];
    result.q3_ms = result.samples[(result.samples.size() * 3) / 4];
    return result;
}

std::string validate_benchmark_run(const BenchmarkRun &run,
                                   const Slic3r::ExPolygons &legacy_reference)
{
    CHECK(run.output.valid);
    return benchmark_geometry_status(legacy_reference, run.native_output);
}

void print_benchmark_row(size_t hole_count,
                         size_t points_per_hole,
                         const OffsetParams &params,
                         BenchmarkLevel level,
                         BenchmarkMethod method,
                         BenchmarkProfile profile,
                         const BenchmarkMethodSummary &summary)
{
    const size_t total_input_points = 256 + hole_count * points_per_hole;
    std::cout << std::fixed << std::setprecision(6)
              << "holes=" << hole_count
              << " outer_points=256"
              << " points_per_hole=" << points_per_hole
              << " total_input_points=" << total_input_points
              << " delta_mm=" << params.delta_mm
              << " join=" << join_name(params.join)
              << " limit=" << params.limit
              << " profile=" << benchmark_profile_name(profile)
              << " measurement_level=" << benchmark_level_name(level)
              << " method=" << benchmark_method_name(method)
              << " samples=" << summary.timing.samples.size()
              << " batch_calls=" << summary.timing.batch_calls
              << " median_ms=" << summary.timing.median_ms
              << " min_ms=" << summary.timing.min_ms
              << " q1_ms=" << summary.timing.q1_ms
              << " q3_ms=" << summary.timing.q3_ms
              << " output_islands=" << summary.output.islands
              << " output_holes=" << summary.output.holes
              << " output_points=" << summary.output.points
              << " output_area_mm2=" << summary.output.area_mm2
              << " output_valid=" << (summary.output.valid ? "true" : "false")
              << " geometry_status=" << summary.geometry_status << '\n';
}

void print_benchmark_ratio(size_t hole_count,
                           size_t points_per_hole,
                           const OffsetParams &params,
                           BenchmarkLevel level,
                           BenchmarkMethod method,
                           BenchmarkProfile profile,
                           const BenchmarkMethodSummary &method_summary,
                           const BenchmarkMethodSummary &legacy_summary)
{
    if (method == BenchmarkMethod::Legacy)
        return;
    const double ratio = legacy_summary.timing.median_ms == 0. ? 0. :
        method_summary.timing.median_ms / legacy_summary.timing.median_ms;
    std::cout << std::fixed << std::setprecision(6)
              << "ratio holes=" << hole_count
              << " points_per_hole=" << points_per_hole
              << " delta_mm=" << params.delta_mm
              << " join=" << join_name(params.join)
              << " limit=" << params.limit
              << " profile=" << benchmark_profile_name(profile)
              << " measurement_level=" << benchmark_level_name(level)
              << " method=" << benchmark_method_name(method) << "/L"
              << " median_ratio=" << ratio << '\n';
}

void run_benchmark_configuration(const BenchmarkFixture &fixture,
                                 const Slic3r::ExPolygons &inputs,
                                 size_t hole_count,
                                 size_t points_per_hole,
                                 const OffsetParams &params,
                                 BenchmarkProfile profile)
{
    const Slic3r::ExPolygons legacy_reference = Slic3r::offset_ex(
        fixture.expolygon, scale_d(params.delta_mm), legacy_join(params.join), params.limit);
    CHECK(benchmark_output_stats(legacy_reference).valid);

    BenchmarkPreparedGeneric prepared_generic(fixture.expolygon);
    const std::array<BenchmarkMethod, 2> method_orders[2] = {
        {BenchmarkMethod::Legacy, BenchmarkMethod::GenericApi},
        {BenchmarkMethod::GenericApi, BenchmarkMethod::Legacy}
    };

    for (const BenchmarkLevel level : {BenchmarkLevel::Prepared, BenchmarkLevel::EndToEnd}) {
        std::array<BenchmarkMethodSummary, 2> summaries;
        std::array<size_t, 2> batch_calls{};
        for (size_t method_index = 0; method_index < 2; ++method_index) {
            const BenchmarkMethod method = static_cast<BenchmarkMethod>(method_index);
            BenchmarkRun pilot = benchmark_run_method(method, level, fixture, inputs, params,
                                                       &prepared_generic);
            summaries[method_index].output = pilot.output;
            summaries[method_index].geometry_status = validate_benchmark_run(pilot, legacy_reference);
            batch_calls[method_index] = benchmark_batch_calls(pilot.elapsed_ms);
            if (level == BenchmarkLevel::Prepared && method == BenchmarkMethod::GenericApi)
                CHECK(prepared_generic.plugin_storage.size() == prepared_generic.baseline_allocations);
        }

        for (size_t warmup = 0; warmup < benchmark_warmup_count(profile); ++warmup) {
            const std::array<BenchmarkMethod, 2> &order = method_orders[warmup % 2];
            for (const BenchmarkMethod method : order) {
                const size_t method_index = static_cast<size_t>(method);
                BenchmarkRun run = benchmark_run_method(method, level, fixture, inputs, params,
                                                        &prepared_generic);
                const std::string status = validate_benchmark_run(run, legacy_reference);
                CHECK(status == summaries[method_index].geometry_status);
                if (level == BenchmarkLevel::Prepared && method == BenchmarkMethod::GenericApi)
                    CHECK(prepared_generic.plugin_storage.size() == prepared_generic.baseline_allocations);
            }
        }

        for (size_t sample = 0; sample < benchmark_sample_count(profile); ++sample) {
            const std::array<BenchmarkMethod, 2> &order = method_orders[sample % 2];
            for (const BenchmarkMethod method : order) {
                const size_t method_index = static_cast<size_t>(method);
                double elapsed_ms = 0.;
                for (size_t repetition = 0; repetition < batch_calls[method_index]; ++repetition) {
                    BenchmarkRun run = benchmark_run_method(method, level, fixture, inputs, params,
                                                            &prepared_generic);
                    elapsed_ms += run.elapsed_ms;
                    const std::string status = validate_benchmark_run(run, legacy_reference);
                    CHECK(status == summaries[method_index].geometry_status);
                    if (level == BenchmarkLevel::Prepared && method == BenchmarkMethod::GenericApi)
                        CHECK(prepared_generic.plugin_storage.size() == prepared_generic.baseline_allocations);
                }
                summaries[method_index].timing.samples.push_back(
                    elapsed_ms / static_cast<double>(batch_calls[method_index]));
            }
        }

        for (size_t method_index = 0; method_index < 2; ++method_index)
            summaries[method_index].timing = benchmark_aggregate(
                std::move(summaries[method_index].timing.samples), batch_calls[method_index]);

        for (size_t method_index = 0; method_index < 2; ++method_index) {
            const BenchmarkMethod method = static_cast<BenchmarkMethod>(method_index);
            print_benchmark_row(hole_count, points_per_hole, params, level, method, profile,
                                summaries[method_index]);
            print_benchmark_ratio(hole_count, points_per_hole, params, level, method, profile,
                                  summaries[method_index], summaries[0]);
        }
    }

    CHECK(prepared_generic.plugin_storage.size() == prepared_generic.baseline_allocations);
}

void print_collection_benchmark_row(const BenchmarkCollectionFixture &fixture,
                                    BenchmarkInputForm form,
                                    const OffsetParams &params,
                                    BenchmarkLevel level,
                                    BenchmarkCollectionMethod method,
                                    BenchmarkProfile profile,
                                    const BenchmarkMethodSummary &summary,
                                    size_t points_per_hole)
{
    size_t total_input_points = 0;
    if (form == BenchmarkInputForm::ExPolygons) {
        for (const Slic3r::ExPolygon &expolygon : fixture.expolygons) {
            total_input_points += expolygon.contour.points.size();
            for (const Slic3r::Polygon &hole : expolygon.holes)
                total_input_points += hole.points.size();
        }
    } else {
        for (const Slic3r::Polygon &polygon : fixture.polygons)
            total_input_points += polygon.points.size();
    }
    std::cout << std::fixed << std::setprecision(6)
              << "input_form=" << benchmark_input_form_name(form)
              << " expolygons=" << fixture.expolygons.size()
              << " holes_per_expolygon=" << fixture.holes_per_expolygon
              << " total_holes=" << fixture.expolygons.size() * fixture.holes_per_expolygon
              << " outer_points=256"
              << " points_per_hole=" << points_per_hole
              << " total_input_points=" << total_input_points
              << " delta_mm=" << params.delta_mm
              << " join=" << join_name(params.join)
              << " limit=" << params.limit
              << " profile=" << benchmark_profile_name(profile)
              << " measurement_level=" << benchmark_level_name(level)
              << " method=" << benchmark_collection_method_name(method)
              << " samples=" << summary.timing.samples.size()
              << " batch_calls=" << summary.timing.batch_calls
              << " median_ms=" << summary.timing.median_ms
              << " min_ms=" << summary.timing.min_ms
              << " q1_ms=" << summary.timing.q1_ms
              << " q3_ms=" << summary.timing.q3_ms
              << " output_islands=" << summary.output.islands
              << " output_holes=" << summary.output.holes
              << " output_points=" << summary.output.points
              << " output_area_mm2=" << summary.output.area_mm2
              << " output_valid=" << (summary.output.valid ? "true" : "false")
              << " geometry_status=" << summary.geometry_status << '\n';
}

void print_collection_benchmark_ratio(const BenchmarkCollectionFixture &fixture,
                                      BenchmarkInputForm form,
                                      const OffsetParams &params,
                                      BenchmarkLevel level,
                                      BenchmarkCollectionMethod method,
                                      BenchmarkProfile profile,
                                      const BenchmarkMethodSummary &method_summary,
                                      const BenchmarkMethodSummary &legacy_summary,
                                      size_t points_per_hole)
{
    if (method == BenchmarkCollectionMethod::Legacy)
        return;
    const double ratio = legacy_summary.timing.median_ms == 0. ? 0. :
        method_summary.timing.median_ms / legacy_summary.timing.median_ms;
    std::cout << std::fixed << std::setprecision(6)
              << "input_form=" << benchmark_input_form_name(form)
              << " expolygons=" << fixture.expolygons.size()
              << " holes_per_expolygon=" << fixture.holes_per_expolygon
              << " points_per_hole=" << points_per_hole
              << " delta_mm=" << params.delta_mm
              << " join=" << join_name(params.join)
              << " limit=" << params.limit
              << " profile=" << benchmark_profile_name(profile)
              << " measurement_level=" << benchmark_level_name(level)
              << " method=" << benchmark_collection_method_name(method) << "/L"
              << " median_ratio=" << ratio << '\n';
}

void run_collection_benchmark_configuration(const BenchmarkCollectionFixture &fixture,
                                            const OffsetParams &params,
                                            size_t points_per_hole,
                                            BenchmarkProfile profile)
{
    const std::array<BenchmarkInputForm, 2> input_forms{
        BenchmarkInputForm::ExPolygons, BenchmarkInputForm::Polygons};
    const std::array<BenchmarkCollectionMethod, 2> method_orders[2] = {
        {BenchmarkCollectionMethod::Legacy, BenchmarkCollectionMethod::GenericApi},
        {BenchmarkCollectionMethod::GenericApi, BenchmarkCollectionMethod::Legacy}
    };
    BenchmarkPreparedGeneric prepared_expolygons(fixture.expolygons);
    BenchmarkPreparedGenericPolygons prepared_polygons(fixture.polygons);

    for (const BenchmarkInputForm form : input_forms) {
        if (form == BenchmarkInputForm::Polygons && params.delta_mm == 0.)
            continue;
        const Slic3r::ExPolygons legacy_reference = form == BenchmarkInputForm::ExPolygons ?
            Slic3r::offset_ex(fixture.expolygons, scale_d(params.delta_mm), legacy_join(params.join), params.limit) :
            Slic3r::offset_ex(fixture.polygons, scale_d(params.delta_mm), legacy_join(params.join), params.limit);
        CHECK(benchmark_output_stats(legacy_reference).valid);

        for (const BenchmarkLevel level : {BenchmarkLevel::Prepared, BenchmarkLevel::EndToEnd}) {
            std::array<BenchmarkMethodSummary, 2> summaries;
            std::array<size_t, 2> batch_calls{};
            for (size_t method_index = 0; method_index < 2; ++method_index) {
                const BenchmarkCollectionMethod method = static_cast<BenchmarkCollectionMethod>(method_index);
                BenchmarkRun pilot = benchmark_collection_run_method(
                    form, method, level, fixture, params, &prepared_expolygons, &prepared_polygons);
                summaries[method_index].output = pilot.output;
                summaries[method_index].geometry_status = validate_benchmark_run(pilot, legacy_reference);
                batch_calls[method_index] = benchmark_batch_calls(pilot.elapsed_ms);
            }

            for (size_t warmup = 0; warmup < benchmark_warmup_count(profile); ++warmup) {
                const std::array<BenchmarkCollectionMethod, 2> &order = method_orders[warmup % 2];
                for (const BenchmarkCollectionMethod method : order) {
                    const size_t method_index = static_cast<size_t>(method);
                    BenchmarkRun run = benchmark_collection_run_method(
                        form, method, level, fixture, params, &prepared_expolygons, &prepared_polygons);
                    const std::string status = validate_benchmark_run(run, legacy_reference);
                    CHECK(status == summaries[method_index].geometry_status);
                }
            }

            for (size_t sample = 0; sample < benchmark_sample_count(profile); ++sample) {
                const std::array<BenchmarkCollectionMethod, 2> &order = method_orders[sample % 2];
                for (const BenchmarkCollectionMethod method : order) {
                    const size_t method_index = static_cast<size_t>(method);
                    double elapsed_ms = 0.;
                    for (size_t repetition = 0; repetition < batch_calls[method_index]; ++repetition) {
                        BenchmarkRun run = benchmark_collection_run_method(
                            form, method, level, fixture, params, &prepared_expolygons, &prepared_polygons);
                        elapsed_ms += run.elapsed_ms;
                        const std::string status = validate_benchmark_run(run, legacy_reference);
                        CHECK(status == summaries[method_index].geometry_status);
                    }
                    summaries[method_index].timing.samples.push_back(
                        elapsed_ms / static_cast<double>(batch_calls[method_index]));
                }
            }

            for (size_t method_index = 0; method_index < 2; ++method_index)
                summaries[method_index].timing = benchmark_aggregate(
                    std::move(summaries[method_index].timing.samples), batch_calls[method_index]);

            for (size_t method_index = 0; method_index < 2; ++method_index) {
                const BenchmarkCollectionMethod method =
                    static_cast<BenchmarkCollectionMethod>(method_index);
                print_collection_benchmark_row(fixture, form, params, level, method,
                                               profile, summaries[method_index], points_per_hole);
                print_collection_benchmark_ratio(fixture, form, params, level, method,
                                                 profile, summaries[method_index], summaries[0],
                                                 points_per_hole);
            }
        }
    }

    CHECK(prepared_expolygons.plugin_storage.size() == prepared_expolygons.baseline_allocations);
    CHECK(prepared_polygons.plugin_storage.size() == prepared_polygons.baseline_allocations);
}

void run_single_benchmark(BenchmarkProfile profile)
{
    const std::vector<BenchmarkCenter> centers = benchmark_centers();
    REQUIRE(centers.size() >= 128);
    const std::vector<BenchmarkSingleCase> cases = benchmark_single_cases(profile);

    std::cout << "clipper_offset_benchmark format=key=value profile="
                 << benchmark_profile_name(profile) << " methods=L,G,N levels=A,B\n";
    for (const BenchmarkSingleCase &benchmark_case : cases) {
        const BenchmarkFixture fixture = benchmark_fixture(
            centers, benchmark_case.hole_count, benchmark_case.points_per_hole);
        validate_benchmark_fixture(fixture, benchmark_case.hole_count, benchmark_case.points_per_hole);
        const Slic3r::ExPolygons inputs{fixture.expolygon};
        const std::vector<OffsetParams> matrix = benchmark_single_matrix(
            profile, benchmark_case.hole_count, benchmark_case.points_per_hole);
        for (const OffsetParams &params : matrix)
            run_benchmark_configuration(fixture, inputs, benchmark_case.hole_count,
                                        benchmark_case.points_per_hole, params, profile);
    }
}

void run_collection_benchmark(BenchmarkProfile profile)
{
    const std::vector<BenchmarkCenter> local_centers = benchmark_centers_in_radius(25.);
    REQUIRE(local_centers.size() >= 16);
    const std::vector<size_t> point_resolutions = benchmark_collection_point_resolutions(profile);
    const std::vector<OffsetParams> matrix = benchmark_collection_matrix(profile);

    std::cout << "clipper_offset_collection_benchmark format=key=value profile="
                 << benchmark_profile_name(profile) << " methods=L,G "
                    "input_forms=expolygons,polygons\n";
    for (const size_t points_per_hole : point_resolutions) {
        const BenchmarkCollectionFixture fixture =
            benchmark_collection_fixture(local_centers, points_per_hole);
        validate_benchmark_collection_fixture(fixture, points_per_hole);
        for (const OffsetParams &params : matrix)
            run_collection_benchmark_configuration(fixture, params, points_per_hole, profile);
    }
}

} // namespace

TEST_CASE("Clipper offset parity: square", "[plugins][clipper][clipper-offset]")
{
    run_case("Square", {Slic3r::ExPolygon(rectangle_mm(0., 0., 20., 20.))}, false);
}

TEST_CASE("Clipper offset parity: square with hole", "[plugins][clipper][clipper-offset]")
{
    const Slic3r::ExPolygon square = square_with_holes(
        rectangle_mm(0., 0., 20., 20.), {rectangle_mm(8., 8., 12., 12.)});
    run_case("Square with hole", {square}, false);
}

TEST_CASE("Clipper offset parity: two close holes", "[plugins][clipper][clipper-offset]")
{
    const Slic3r::ExPolygon square = square_with_holes(
        rectangle_mm(0., 0., 30., 30.), {rectangle_mm(10., 10., 14., 14.), rectangle_mm(15., 15., 19., 19.)});
    run_case("Two close holes", {square}, false);
}

TEST_CASE("Clipper offset parity: overlapping holed squares", "[plugins][clipper][clipper-offset]")
{
    const Slic3r::ExPolygon first = square_with_holes(
        rectangle_mm(0., 0., 20., 20.), {rectangle_mm(5., 5., 9., 9.)});
    const Slic3r::ExPolygon second = square_with_holes(
        rectangle_mm(12., 12., 32., 32.), {rectangle_mm(23., 23., 27., 27.)});
    run_case("Overlapping holed squares", {first, second}, true);
}

TEST_CASE("Clipper offset parity: holed square inside a hole", "[plugins][clipper][clipper-offset]")
{
    const Slic3r::ExPolygon outer = square_with_holes(
        rectangle_mm(0., 0., 40., 40.), {rectangle_mm(5., 5., 35., 35.)});
    const Slic3r::ExPolygon inner = square_with_holes(
        rectangle_mm(10., 10., 30., 30.), {rectangle_mm(17., 17., 23., 23.)});
    run_case("Holed square inside a hole", {outer, inner}, false);
}

TEST_CASE("Clipper offset parity: squares sharing an edge", "[plugins][clipper][clipper-offset]")
{
    run_case("Squares sharing an edge",
             {Slic3r::ExPolygon(rectangle_mm(0., 0., 10., 10.)),
              Slic3r::ExPolygon(rectangle_mm(10., 0., 20., 10.))},
             true);
}

TEST_CASE("Clipper offset parity: squares sharing a vertex", "[plugins][clipper][clipper-offset]")
{
    run_case("Squares sharing a vertex",
             {Slic3r::ExPolygon(rectangle_mm(0., 0., 10., 10.)),
              Slic3r::ExPolygon(rectangle_mm(10., 10., 20., 20.))},
             true);
}

TEST_CASE("Clipper offset parity: close disjoint squares", "[plugins][clipper][clipper-offset]")
{
    run_case("Close disjoint squares",
             {Slic3r::ExPolygon(rectangle_mm(0., 0., 10., 10.)),
              Slic3r::ExPolygon(rectangle_mm(10.5, 0., 20.5, 10.))},
             false);
}

TEST_CASE("Clipper offset parity: hole near the contour", "[plugins][clipper][clipper-offset]")
{
    const Slic3r::ExPolygon square = square_with_holes(
        rectangle_mm(0., 0., 20., 20.), {rectangle_mm(0.3, 5., 5., 15.)});
    run_case("Hole near the contour", {square}, false);
}

TEST_CASE("Clipper offset parity: vanishing hole", "[plugins][clipper][clipper-offset]")
{
    const Slic3r::ExPolygon square = square_with_holes(
        rectangle_mm(0., 0., 20., 20.), {rectangle_mm(9., 9., 11., 11.)});
    run_case("Vanishing hole", {square}, false);
}

TEST_CASE("Clipper offset parity: dumbbell", "[plugins][clipper][clipper-offset]")
{
    const Slic3r::ExPolygons united = union_ex(Slic3r::ExPolygons{
        Slic3r::ExPolygon(rectangle_mm(0., 0., 8., 8.)),
        Slic3r::ExPolygon(rectangle_mm(8., 3.9, 12., 4.1)),
        Slic3r::ExPolygon(rectangle_mm(12., 0., 20., 8.))});
    REQUIRE(united.size() == 1);
    run_case("Dumbbell", {united.front()}, false);
}

TEST_CASE("Clipper offset parity: collapsing and empty", "[plugins][clipper][clipper-offset]")
{
    run_case("Collapsing square", {Slic3r::ExPolygon(rectangle_mm(0., 0., 2., 2.))}, false);
    run_case("Empty collection", {}, false);
    run_case("Empty ExPolygon", {Slic3r::ExPolygon()}, false);
}

TEST_CASE("Clipper offset parity: sharp and concave", "[plugins][clipper][clipper-offset]")
{
    Slic3r::Polygon triangle({Slic3r::Point(scale_i(0.), scale_i(0.)),
                              Slic3r::Point(scale_i(20.), scale_i(0.)),
                              Slic3r::Point(scale_i(20.), scale_i(5.36))});
    triangle.make_counter_clockwise();
    run_case("Sharp triangle", {Slic3r::ExPolygon(triangle)}, false);

    const Slic3r::ExPolygon outer(rectangle_mm(0., 0., 20., 20.));
    const Slic3r::Polygon removed = rectangle_mm(10., 10., 20., 20.);
    const Slic3r::ExPolygons l_shape = diff_ex(outer, removed);
    REQUIRE(l_shape.size() == 1);
    run_case("Concave L", {l_shape.front()}, false);
}

TEST_CASE("Clipper offset parity: rotated holed square", "[plugins][clipper][clipper-offset]")
{
    const Slic3r::ExPolygon square = square_with_holes(
        rectangle_mm(0., 0., 20., 20.), {rectangle_mm(8., 8., 12., 12.)});
    run_case("Rotated holed square", {rotated(square, 30. * M_PI / 180.)}, false);
}

TEST_CASE("Clipper offset parity: three nesting levels", "[plugins][clipper][clipper-offset]")
{
    const Slic3r::ExPolygon first = square_with_holes(
        rectangle_mm(0., 0., 40., 40.), {rectangle_mm(5., 5., 35., 35.)});
    const Slic3r::ExPolygon second = square_with_holes(
        rectangle_mm(10., 10., 30., 30.), {rectangle_mm(15., 15., 25., 25.)});
    const Slic3r::ExPolygon third(rectangle_mm(18., 18., 22., 22.));
    run_case("Three nesting levels", {first, second, third}, false);
}

TEST_CASE("Clipper offset parity: large and negative coordinates", "[plugins][clipper][clipper-offset]")
{
    const Slic3r::ExPolygon square = square_with_holes(
        rectangle_mm(0., 0., 20., 20.), {rectangle_mm(8., 8., 12., 12.)});
    Slic3r::ExPolygon negative = square;
    negative.translate(Slic3r::Point(scale_i(-500.), scale_i(-500.)));
    Slic3r::ExPolygon positive = square;
    positive.translate(Slic3r::Point(scale_i(480.), scale_i(480.)));
    run_case("Negative coordinates", {negative}, false);
    run_case("Large positive coordinates", {positive}, false);
}

TEST_CASE("Clipper offset parity: shifted start vertex", "[plugins][clipper][clipper-offset]")
{
    const Slic3r::ExPolygon square = square_with_holes(
        rectangle_mm(0., 0., 20., 20.), {rectangle_mm(8., 8., 12., 12.)});
    for (size_t shift = 1; shift <= 3; ++shift)
        run_case("Shifted start vertex " + std::to_string(shift), {start_shifted(square, shift)}, false);
}

TEST_CASE("Random disjoint holed rectangles", "[plugins][clipper][clipper-offset]")
{
    const unsigned int seed = 0x0ff5e7u;
    std::mt19937 rng(seed);
    const std::vector<double> deltas{-0.1, 0.1, -0.225, 0.225, -0.5, 0.5, -1., 1., -3., 3.};
    const std::vector<OffsetParams> joins{
        {0., CLIPPER_JOIN_MITER, 1.2},
        {0., CLIPPER_JOIN_MITER, 3.},
         {0., CLIPPER_JOIN_MITER, 10.},
         {0., CLIPPER_JOIN_SQUARE, 3.},
         {0., CLIPPER_JOIN_ROUND, scale_d(0.005)}};
    std::uniform_int_distribution<size_t> delta_index_distribution(0, deltas.size() - 1);
    std::uniform_int_distribution<size_t> join_index_distribution(0, joins.size() - 1);

    for (int iteration = 0; iteration < 300; ++iteration) {
        const std::vector<Slic3r::ExPolygon> inputs = random_disjoint_holed_rectangles(rng);

        const double delta_mm = deltas[delta_index_distribution(rng)];
        OffsetParams params = joins[join_index_distribution(rng)];
        params.delta_mm = delta_mm;
        Slic3r::PluginStorage plugin_storage;
        storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
        const Slic3r::ExPolygons input_description(inputs.begin(), inputs.end());
        INFO("seed=" << seed << " iteration=" << iteration << " delta_mm=" << delta_mm
             << " join=" << join_name(params.join) << " limit=" << params.limit
             << " inputs=" << describe(input_description));

        const Slic3r::ExPolygons api = api_offset_expolygons(storage, inputs, params);
        const Slic3r::ExPolygons legacy =
            Slic3r::offset_ex(inputs, scale_d(delta_mm), legacy_join(params.join), params.limit);
        const bool random_documented = is_documented("Random disjoint holed rectangles", "expolygons",
                                                     delta_mm, params.join, params.limit);
        compare("Random disjoint holed rectangles variant=expolygons", api, legacy, random_documented);

        const Slic3r::Polygons polygons = Slic3r::to_polygons(inputs);
        const Slic3r::ExPolygons api_polygons = api_offset_polygons(storage, polygons, params);
        const Slic3r::ExPolygons legacy_polygons =
            Slic3r::offset_ex(polygons, scale_d(delta_mm), legacy_join(params.join), params.limit);
        const bool random_polygon_documented = is_documented("Random disjoint holed rectangles", "polygons",
                                                              delta_mm, params.join, params.limit);
        compare("Random disjoint holed rectangles variant=polygons", api_polygons, legacy_polygons,
                random_polygon_documented);
    }
}

TEST_CASE("Clipper offset performance benchmark (fast)", "[.][clipper-offset-benchmark]")
{
    run_single_benchmark(BenchmarkProfile::Fast);
}

TEST_CASE("Clipper offset performance benchmark (full)", "[.][clipper-offset-benchmark-full]")
{
    run_single_benchmark(BenchmarkProfile::Full);
}

TEST_CASE("Clipper offset collection performance benchmark (fast)", "[.][clipper-offset-benchmark-collections]")
{
    run_collection_benchmark(BenchmarkProfile::Fast);
}

TEST_CASE("Clipper offset collection performance benchmark (full)", "[.][clipper-offset-benchmark-collections-full]")
{
    run_collection_benchmark(BenchmarkProfile::Full);
}
