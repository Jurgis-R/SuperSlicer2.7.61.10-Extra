/*
 * Clipper offset strategies
 * =========================
 *
 * Every ClipperOffset emits a small self-intersecting loop at each vertex where
 * the offset turns against the ring (a "spike": convex vertices of a shrinking
 * ring, concave vertices of a growing one).  The clean-up union resolves them.
 * Offsetting all rings with one ClipperOffset resolves every spike in a single
 * union whose active edge list spans the whole shape; offsetting ring by ring
 * resolves them in tiny unions but pays one Clipper pass per ring.
 *
 * This benchmark compares implementations of one offset step, and of the
 * two-step offset2, on large shapes:
 *
 * | strategy | description |
 * |----------|-------------|
 * | legacy | `Slic3r::offset_ex` / `offset2_ex` on ExPolygons |
 * | api | public `clipper_offset` / `clipper_offset2` on a prepared operand, implementing adaptive |
 * | grouped | one ClipperOffset over all rings, positive fill, bridge check (the API for a lone ring) |
 * | per_ring | every ring offset and cleaned alone, then one positive union |
 * | spike_split | rings with a spike cleaned alone, the others offset raw, then one positive union |
 * | *_no_check | same strategy without the zero-width bridge check |
 * | adaptive | grouped for a single ring, spike_split otherwise, always checked: the API |
 *
 * Measured and dropped: grouped for a negative delta and per_ring for a
 * positive one (never better than spike_split), and spike_split leaving rings
 * with fewer than eight spikes raw (their raw spikes recreate the bridges).
 *
 * Every ring strategy but the `_no_check` ones ends with the API zero-width
 * bridge check.  Without it, overlapping raw spike loops (grouped, or rings
 * left raw with spikes) and exact collapses (a feature exactly twice the offset
 * wide) leave bridges: the contacts report shows both.  The report
 * prints the legacy time and, for every other strategy, its time ratio to
 * legacy followed by flags: `A` same rings as the API, `a` same area as the
 * API, `L` same rings as legacy, `-` none of them; a `_no_check` strategy adds
 * `=` when it matches its checked twin and `!` otherwise.  The contacts report
 * runs every ring strategy on the shapes whose grouped offset leaves
 * zero-width bridges.  Run the timing in Release:
 * `[clipper-offset-strategies]`; `[clipper-offset-strategies-quick]` runs each
 * strategy once, without timing, to check the flags in Debug.
 */

#include <catch2/catch.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Polygon.hpp"
#include "clipper_offset_test_helpers.hpp"
// Last: it includes clipper.hpp, which must follow clipper_z.hpp from the headers above.
#include "libslic3r/Api/host/ClipperShapes.hpp"

namespace {

using namespace Slic3r;
using namespace clipper_offset_test;

enum class Strategy {
    Legacy, Api, Grouped, GroupedNoCheck, PerRing, PerRingNoCheck, SpikeSplit, SpikeSplitNoCheck, Adaptive
};

const std::array<Strategy, 9> all_strategies = {
    Strategy::Legacy, Strategy::Api, Strategy::Grouped, Strategy::GroupedNoCheck, Strategy::PerRing,
    Strategy::PerRingNoCheck, Strategy::SpikeSplit, Strategy::SpikeSplitNoCheck, Strategy::Adaptive};

struct Fixture {
    std::string name;
    Slic3r::ExPolygons inputs;
};

// One offset step when two_steps is false, offset2(delta1, delta2) otherwise.
struct Operation {
    double delta1_mm;
    double delta2_mm;
    bool two_steps;
};

struct JoinChoice {
    clipper_join_type_t join;
    double limit;
};

struct StepParams {
    double delta;
    ClipperLib::JoinType join;
    double limit;
};

// Everything one strategy needs: the native input and the prepared API operand.
struct Run {
    const Fixture *fixture;
    const slic3r_api::ClipperOperand *operand;
    Operation operation;
    JoinChoice join;
};

struct Row {
    std::string family;
    clipper_join_type_t join;
    std::vector<double> median_ms;
};

// Large shapes stressing grouped and per-ring clean-up differently.
std::vector<Fixture> benchmark_fixtures();

// Disc with a grid of circular holes, in millimetres.
Slic3r::ExPolygon holed_disc(double radius, double pitch, double hole_radius, size_t hole_points);

// Regular polygon approximating a circle, in millimetres.
Slic3r::Polygon circle_mm(double x, double y, double radius, size_t points, bool clockwise);

// Grid of separate circular islands without holes.
Slic3r::ExPolygons dots(size_t columns, size_t rows, double pitch, double radius, size_t points);

// Square with square holes on every other cell of a grid.
Slic3r::ExPolygon checkerboard(size_t cells, double pitch, double size, double margin);

// Single offsets and offset2 pairs measured on every fixture.
std::vector<Operation> benchmark_operations();

// Run the whole matrix, printing one row per fixture, operation and join.
void run_benchmark(size_t batches, double target_batch_ms);

// Median time per call for every strategy, strategies interleaved per batch.
std::vector<double> time_strategies(const Run &run, size_t batches, double target_batch_ms);

// Run one strategy and materialize its result as native ExPolygons.
Slic3r::ExPolygons run_strategy(Strategy strategy, const Run &run);

// Offset rings once with a ring strategy, ending with the bridge check.
ClipperLib::PolyTree offset_rings(const ClipperLib::Paths &rings, const StepParams &step, Strategy strategy);

// One ClipperOffset over all rings with a positive-fill clean-up.
ClipperLib::PolyTree grouped_offset(const ClipperLib::Paths &rings, const StepParams &step, bool check_bridges);

// Whether the offset of a ring emits at least one spike loop.
bool has_spikes(const ClipperLib::Path &ring, double delta);

// Unite offset paths with a positive fill, then optionally split zero-width bridges.
ClipperLib::PolyTree positive_union(const ClipperLib::Paths &paths, bool check_bridges);

// Rerun a non-zero union when two edges of the tree overlap, as the API does.
ClipperLib::PolyTree without_bridges(ClipperLib::PolyTree tree);

// Apply the join limit and the shortest edge rule shared with ClipperUtils.
void configure_offsetter(ClipperLib::ClipperOffset &offsetter, const StepParams &step);

// Contours and holes of ExPolygons as Clipper paths.
ClipperLib::Paths ring_paths(const Slic3r::ExPolygons &expolygons);

// Flags comparing a result with the API and legacy results.
std::string flags(const Slic3r::ExPolygons &result, const Slic3r::ExPolygons &api, const Slic3r::ExPolygons &legacy);

// Index of the checked strategy a _no_check strategy is measured against, or 0.
size_t checked_twin(Strategy strategy);

// Print, for each shape with known bridges, whether every ring strategy
// matches legacy and leaves zero-width bridges.
void run_contacts_report();

// Print the geometric mean of every strategy ratio per operation family and join.
void print_summary(const std::vector<Row> &rows);

const char *strategy_name(Strategy strategy);

std::vector<Fixture> benchmark_fixtures()
{
    Slic3r::ExPolygons islands;
    for (size_t island = 0; island < 8; ++island) {
        Slic3r::ExPolygon disc = holed_disc(25., 6., 1.5, 32);
        disc.translate(Slic3r::Point(scale_i(60. * double(island)), 0));
        islands.push_back(std::move(disc));
    }
    return {
        {"disc_300_holes_64pts", {holed_disc(100., 10., 2., 64)}},
        {"disc_1250_holes_32pts", {holed_disc(100., 5., 1.5, 32)}},
        {"8_islands_50_holes", islands},
        {"400_dots_32pts", dots(20, 20, 5., 1.5, 32)},
        {"checkerboard_200_holes", {checkerboard(20, 9., 6., 5.)}},
        {"comb_200_teeth", {Slic3r::ExPolygon(comb_mm(1.1, 200, 6.))}},
        {"star_2000_vertices", {Slic3r::ExPolygon(star_mm(0., 0., 100., 60., 1000))}},
    };
}

Slic3r::ExPolygon holed_disc(double radius, double pitch, double hole_radius, size_t hole_points)
{
    Slic3r::ExPolygon disc(circle_mm(0., 0., radius, 720, false));
    for (double x = -radius; x <= radius; x += pitch)
        for (double y = -radius; y <= radius; y += pitch)
            if (std::hypot(x, y) + hole_radius + 1. < radius)
                disc.holes.push_back(circle_mm(x, y, hole_radius, hole_points, true));
    return disc;
}

Slic3r::Polygon circle_mm(double x, double y, double radius, size_t points, bool clockwise)
{
    Slic3r::Polygon polygon;
    for (size_t index = 0; index < points; ++index) {
        const double angle = 2. * PI * double(index) / double(points);
        polygon.points.emplace_back(scale_i(x + radius * std::cos(angle)), scale_i(y + radius * std::sin(angle)));
    }
    if (clockwise)
        polygon.make_clockwise();
    else
        polygon.make_counter_clockwise();
    return polygon;
}

Slic3r::ExPolygons dots(size_t columns, size_t rows, double pitch, double radius, size_t points)
{
    Slic3r::ExPolygons result;
    for (size_t row = 0; row < rows; ++row)
        for (size_t column = 0; column < columns; ++column)
            result.emplace_back(circle_mm(pitch * double(column), pitch * double(row), radius, points, false));
    return result;
}

Slic3r::ExPolygon checkerboard(size_t cells, double pitch, double size, double margin)
{
    Slic3r::Polygons holes;
    for (size_t row = 0; row < cells; ++row)
        for (size_t column = 0; column < cells; ++column)
            if ((row + column) % 2 == 0) {
                const double x = margin + pitch * double(column);
                const double y = margin + pitch * double(row);
                holes.push_back(rectangle_mm(x, y, x + size, y + size));
            }
    const double extent = 2. * margin + pitch * double(cells - 1) + size;
    return with_holes(rectangle_mm(0., 0., extent, extent), holes);
}

std::vector<Operation> benchmark_operations()
{
    return {
        {-0.2, 0., false}, {0.2, 0., false}, {-1., 0., false}, {1., 0., false},
        {-0.2, 0.2, true}, {0.2, -0.2, true}, {-1., 1., true}, {1., -1., true},
    };
}

void run_benchmark(size_t batches, double target_batch_ms)
{
    const std::array<JoinChoice, 2> joins = {{{CLIPPER_JOIN_MITER, 3.}, {CLIPPER_JOIN_ROUND, scale_d(0.005)}}};
    std::cout << std::left << std::setw(24) << "fixture" << std::setw(14) << "operation" << std::setw(7) << "join"
              << std::right << std::setw(11) << "legacy_ms";
    for (size_t index = 1; index < all_strategies.size(); ++index)
        std::cout << std::setw(15) << strategy_name(all_strategies[index]);
    std::cout << '\n';

    std::vector<Row> rows;
    for (const Fixture &fixture : benchmark_fixtures()) {
        Slic3r::PluginStorage plugin_storage;
        storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
        slic3r_api::StoredExPolygonCollection collection(storage);
        for (const Slic3r::ExPolygon &input : fixture.inputs) {
            slic3r_api::StoredExPolygon stored(storage);
            stored.copy_from(view(input));
            collection.push_back_move(std::move(stored));
        }
        const slic3r_api::ClipperOperand operand(storage, collection.readonly());

        for (const Operation &operation : benchmark_operations())
            for (const JoinChoice &join : joins) {
                const Run run{&fixture, &operand, operation, join};
                std::vector<Slic3r::ExPolygons> results;
                for (Strategy strategy : all_strategies)
                    results.push_back(run_strategy(strategy, run));
                // Zero batches only checks the flags: every time then reads 1 ms.
                const std::vector<double> median_ms = batches == 0 ?
                    std::vector<double>(all_strategies.size(), 1.) : time_strategies(run, batches, target_batch_ms);

                std::ostringstream label;
                if (operation.two_steps)
                    label << '(' << operation.delta1_mm << ',' << operation.delta2_mm << ')';
                else
                    label << operation.delta1_mm;
                std::cout << std::left << std::setw(24) << fixture.name << std::setw(14) << label.str()
                          << std::setw(7) << join_name(join.join) << std::right << std::fixed << std::setprecision(2)
                          << std::setw(11) << median_ms[0];
                for (size_t index = 1; index < all_strategies.size(); ++index) {
                    std::ostringstream cell;
                    cell << std::fixed << std::setprecision(2) << median_ms[index] / median_ms[0] << ' '
                         << flags(results[index], results[1], results[0]);
                    const size_t twin = checked_twin(all_strategies[index]);
                    if (twin != 0)
                        cell << (canonical(results[index]) == canonical(results[twin]) ? '=' : '!');
                    std::cout << std::setw(15) << cell.str();
                }
                std::cout << std::defaultfloat << '\n';

                const std::string family = operation.two_steps ? "offset2" :
                                           operation.delta1_mm < 0. ? "shrink" : "grow";
                rows.push_back({family, join.join, median_ms});
            }
    }
    print_summary(rows);
}

std::vector<double> time_strategies(const Run &run, size_t batches, double target_batch_ms)
{
    // Calibrate each strategy so one batch lasts about target_batch_ms.
    std::vector<size_t> calls(all_strategies.size(), 1);
    for (size_t index = 0; index < all_strategies.size(); ++index) {
        const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
        run_strategy(all_strategies[index], run);
        const std::chrono::duration<double, std::milli> elapsed = std::chrono::steady_clock::now() - start;
        calls[index] = std::max<size_t>(1, size_t(target_batch_ms / std::max(elapsed.count(), 1e-3)));
    }

    std::vector<std::vector<double>> samples(all_strategies.size());
    for (size_t batch = 0; batch < batches; ++batch)
        for (size_t index = 0; index < all_strategies.size(); ++index) {
            const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
            for (size_t call = 0; call < calls[index]; ++call)
                run_strategy(all_strategies[index], run);
            const std::chrono::duration<double, std::milli> elapsed = std::chrono::steady_clock::now() - start;
            samples[index].push_back(elapsed.count() / double(calls[index]));
        }

    std::vector<double> medians;
    for (std::vector<double> &strategy_samples : samples) {
        std::sort(strategy_samples.begin(), strategy_samples.end());
        medians.push_back(strategy_samples[strategy_samples.size() / 2]);
    }
    return medians;
}

Slic3r::ExPolygons run_strategy(Strategy strategy, const Run &run)
{
    const double delta1 = scale_d(run.operation.delta1_mm);
    const double delta2 = scale_d(run.operation.delta2_mm);
    const ClipperLib::JoinType join = legacy_join(run.join.join);
    if (strategy == Strategy::Legacy)
        return run.operation.two_steps ?
            Slic3r::offset2_ex(run.fixture->inputs, delta1, delta2, join, run.join.limit) :
            Slic3r::offset_ex(run.fixture->inputs, delta1, join, run.join.limit);
    if (strategy == Strategy::Api) {
        const slic3r_api::ClipperOperand result = run.operation.two_steps ?
            slic3r_api::clipper_offset2(*run.operand, delta1, delta2, run.join.join, run.join.limit) :
            slic3r_api::clipper_offset(*run.operand, delta1, run.join.join, run.join.limit);
        const slic3r_api::StoredExPolygonCollection collection = result.to_expolygon_collection();
        return to_native(collection.readonly());
    }

    ClipperLib::PolyTree tree = offset_rings(ring_paths(run.fixture->inputs), {delta1, join, run.join.limit}, strategy);
    if (run.operation.two_steps) {
        ClipperLib::Paths intermediate;
        ClipperLib::PolyTreeToPaths(std::move(tree), intermediate);
        tree = offset_rings(intermediate, {delta2, join, run.join.limit}, strategy);
    }
    Slic3r::ExPolygons result;
    Slic3r::traverse_pt(&tree, &result);
    return result;
}

ClipperLib::PolyTree offset_rings(const ClipperLib::Paths &rings, const StepParams &step, Strategy strategy)
{
    const bool check_bridges = strategy != Strategy::GroupedNoCheck && strategy != Strategy::PerRingNoCheck &&
                               strategy != Strategy::SpikeSplitNoCheck;
    // A lone ring needs no merge union: its own clean-up and the bridge check
    // are cheaper than cleaning it and uniting it again.
    if (strategy == Strategy::Grouped || strategy == Strategy::GroupedNoCheck ||
        (strategy == Strategy::Adaptive && rings.size() == 1))
        return grouped_offset(rings, step, check_bridges);

    const bool spike_split = strategy == Strategy::SpikeSplit || strategy == Strategy::SpikeSplitNoCheck ||
                             strategy == Strategy::Adaptive;
    ClipperLib::ClipperOffset offsetter;
    configure_offsetter(offsetter, step);
    offsetter.ShrinkWithPositiveFill = true;
    ClipperLib::Paths offset_paths;
    ClipperLib::Paths ring_output;
    for (const ClipperLib::Path &ring : rings) {
        // Execute reorients a lone clockwise ring, so it is offset with the
        // opposite sign and reversed back, as ClipperUtils raw_offset does.
        const bool counter_clockwise = ClipperLib::Orientation(ring);
        const double delta = counter_clockwise ? step.delta : -step.delta;
        offsetter.Clear();
        offsetter.AddPath(ring, step.join, ClipperLib::etClosedPolygon);
        if (!spike_split || has_spikes(ring, step.delta))
            offsetter.Execute(ring_output, delta);
        else
            offsetter.ExecuteRaw(ring_output, delta);
        for (ClipperLib::Path &path : ring_output) {
            if (!counter_clockwise)
                std::reverse(path.begin(), path.end());
            offset_paths.push_back(std::move(path));
        }
    }
    return positive_union(offset_paths, check_bridges);
}

ClipperLib::PolyTree grouped_offset(const ClipperLib::Paths &rings, const StepParams &step, bool check_bridges)
{
    ClipperLib::ClipperOffset offsetter;
    configure_offsetter(offsetter, step);
    offsetter.ShrinkWithPositiveFill = true;
    offsetter.AddPaths(rings, step.join, ClipperLib::etClosedPolygon);
    ClipperLib::PolyTree tree;
    offsetter.Execute(tree, step.delta);
    return check_bridges ? without_bridges(std::move(tree)) : std::move(tree);
}

bool has_spikes(const ClipperLib::Path &ring, double delta)
{
    // Same test as ClipperOffset::OffsetPoint: nearly collinear vertices emit
    // one point, the others a spike when the turn opposes the offset.
    const size_t count = ring.size();
    for (size_t index = 0; index < count; ++index) {
        const ClipperLib::IntPoint &previous = ring[(index + count - 1) % count];
        const ClipperLib::IntPoint &point = ring[index];
        const ClipperLib::IntPoint &next = ring[(index + 1) % count];
        const double ax = double(point.x() - previous.x());
        const double ay = double(point.y() - previous.y());
        const double bx = double(next.x() - point.x());
        const double by = double(next.y() - point.y());
        const double lengths = std::hypot(ax, ay) * std::hypot(bx, by);
        if (lengths == 0.)
            continue;
        const double sin_a = (ax * by - ay * bx) / lengths;
        const double cos_a = (ax * bx + ay * by) / lengths;
        if (std::abs(sin_a * delta) < 1. && cos_a > 0.)
            continue;
        if (sin_a * delta < 0.)
            return true;
    }
    return false;
}

ClipperLib::PolyTree positive_union(const ClipperLib::Paths &paths, bool check_bridges)
{
    ClipperLib::Clipper clipper;
    clipper.AddPaths(paths, ClipperLib::ptSubject, true);
    ClipperLib::PolyTree tree;
    clipper.Execute(ClipperLib::ctUnion, tree, ClipperLib::pftPositive, ClipperLib::pftPositive);
    return check_bridges ? without_bridges(std::move(tree)) : std::move(tree);
}

ClipperLib::PolyTree without_bridges(ClipperLib::PolyTree tree)
{
    if (!Slic3r::ApiClipper::has_overlapping_edges(tree))
        return tree;
    ClipperLib::Paths rings;
    ClipperLib::PolyTreeToPaths(std::move(tree), rings);
    ClipperLib::Clipper clipper;
    clipper.AddPaths(rings, ClipperLib::ptSubject, true);
    ClipperLib::PolyTree united;
    clipper.Execute(ClipperLib::ctUnion, united, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
    return united;
}

void configure_offsetter(ClipperLib::ClipperOffset &offsetter, const StepParams &step)
{
    if (step.join == ClipperLib::jtRound)
        offsetter.ArcTolerance = step.limit;
    else
        offsetter.MiterLimit = step.limit;
    offsetter.ShortestEdgeLength = std::abs(step.delta * Slic3r::ClipperOffsetShortestEdgeFactor);
}

ClipperLib::Paths ring_paths(const Slic3r::ExPolygons &expolygons)
{
    ClipperLib::Paths paths;
    for (const Slic3r::ExPolygon &expolygon : expolygons) {
        paths.emplace_back(expolygon.contour.points);
        for (const Slic3r::Polygon &hole : expolygon.holes)
            paths.emplace_back(hole.points);
    }
    return paths;
}

std::string flags(const Slic3r::ExPolygons &result, const Slic3r::ExPolygons &api, const Slic3r::ExPolygons &legacy)
{
    const CanonicalShape shape = canonical(result);
    std::string text;
    if (shape == canonical(api))
        text += 'A';
    else if (symmetric_area_mm2(result, api) < 1e-6)
        text += 'a';
    if (shape == canonical(legacy))
        text += 'L';
    return text.empty() ? "-" : text;
}

size_t checked_twin(Strategy strategy)
{
    const Strategy twin = strategy == Strategy::GroupedNoCheck ? Strategy::Grouped :
                          strategy == Strategy::PerRingNoCheck ? Strategy::PerRing :
                          strategy == Strategy::SpikeSplitNoCheck ? Strategy::SpikeSplit : Strategy::Legacy;
    return size_t(std::find(all_strategies.begin(), all_strategies.end(), twin) - all_strategies.begin());
}

void run_contacts_report()
{
    struct ContactShape {
        std::string name;
        Slic3r::ExPolygon input;
        double delta_mm;
        clipper_join_type_t join;
        double limit;
    };
    const std::vector<ContactShape> shapes = {
        {"88_square", with_holes(rectangle_mm(105., 19., 118., 34.),
                                 {rectangle_mm(110., 26., 112., 31.), rectangle_mm(110., 21., 115., 24.)}),
         -1., CLIPPER_JOIN_SQUARE, 3.},
        {"14_miter3", with_holes(rectangle_mm(46., 2., 73., 12.),
                                 {rectangle_mm(61., 4., 68., 10.), rectangle_mm(49., 5., 53., 9.)}),
         -1., CLIPPER_JOIN_MITER, 3.},
        {"244_miter1.2", with_holes(rectangle_mm(85., 21., 104., 37.),
                                    {rectangle_mm(93., 27., 97., 34.), rectangle_mm(91., 22., 98., 26.)}),
         -0.5, CLIPPER_JOIN_MITER, 1.2},
        {"two_close_holes_square", with_holes(rectangle_mm(0., 0., 30., 30.),
                                              {rectangle_mm(10., 10., 14., 14.), rectangle_mm(15., 15., 19., 19.)}),
         -5., CLIPPER_JOIN_SQUARE, 3.},
        {"14_grow_back", with_holes(rectangle_mm(46., 2., 73., 12.),
                                    {rectangle_mm(61.5, 4.5, 67.5, 9.5), rectangle_mm(49., 5., 53., 9.)}),
         -0.5, CLIPPER_JOIN_MITER, 3.},
        // The 2 mm base bar collapses exactly to a line under the -1 mm offset.
        {"holed_comb_collapse", with_holes(comb_mm(1.2, 6, 5.), {rectangle_mm(2., 0.8, 9., 1.6)}),
         -1., CLIPPER_JOIN_ROUND, scale_d(0.02)},
    };
    std::cout << "\nContacts: L same rings as legacy offset_ex(ExPolygon), B zero-width bridge left\n"
              << std::left << std::setw(24) << "shape" << std::right;
    for (Strategy strategy : all_strategies)
        std::cout << std::setw(15) << strategy_name(strategy);
    std::cout << '\n';
    for (const ContactShape &shape : shapes) {
        const Fixture fixture{shape.name, {shape.input}};
        Slic3r::PluginStorage plugin_storage;
        storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
        slic3r_api::StoredExPolygonCollection collection(storage);
        slic3r_api::StoredExPolygon stored(storage);
        stored.copy_from(view(shape.input));
        collection.push_back_move(std::move(stored));
        const slic3r_api::ClipperOperand operand(storage, collection.readonly());
        const Run run{&fixture, &operand, {shape.delta_mm, 0., false}, {shape.join, shape.limit}};
        const Slic3r::ExPolygons legacy = Slic3r::offset_ex(shape.input, scale_d(shape.delta_mm),
                                                            legacy_join(shape.join), shape.limit);
        std::cout << std::left << std::setw(24) << shape.name << std::right;
        for (Strategy strategy : all_strategies) {
            const Slic3r::ExPolygons result = run_strategy(strategy, run);
            std::string cell = canonical(result) == canonical(legacy) ? "L" : "-";
            if (find_contacts(result).self_touching_rings != 0 || !find_contacts(result).points.empty())
                cell += 'B';
            std::cout << std::setw(15) << cell;
        }
        std::cout << '\n';
    }
}

void print_summary(const std::vector<Row> &rows)
{
    std::cout << "\nGeometric mean of time / legacy time\n" << std::left << std::setw(10) << "family"
              << std::setw(7) << "join" << std::right;
    for (size_t index = 1; index < all_strategies.size(); ++index)
        std::cout << std::setw(15) << strategy_name(all_strategies[index]);
    std::cout << '\n';
    for (const char *family : {"shrink", "grow", "offset2"})
        for (clipper_join_type_t join : {CLIPPER_JOIN_MITER, CLIPPER_JOIN_ROUND}) {
            std::vector<double> log_sums(all_strategies.size(), 0.);
            size_t count = 0;
            for (const Row &row : rows)
                if (row.family == family && row.join == join) {
                    for (size_t index = 1; index < all_strategies.size(); ++index)
                        log_sums[index] += std::log(row.median_ms[index] / row.median_ms[0]);
                    ++count;
                }
            std::cout << std::left << std::setw(10) << family << std::setw(7) << join_name(join) << std::right
                      << std::fixed << std::setprecision(2);
            for (size_t index = 1; index < all_strategies.size(); ++index)
                std::cout << std::setw(15) << std::exp(log_sums[index] / double(count));
            std::cout << std::defaultfloat << '\n';
        }
}

const char *strategy_name(Strategy strategy)
{
    switch (strategy) {
    case Strategy::Legacy:
        return "legacy";
    case Strategy::Api:
        return "api";
    case Strategy::Grouped:
        return "grouped";
    case Strategy::GroupedNoCheck:
        return "grouped_no_ck";
    case Strategy::PerRing:
        return "per_ring";
    case Strategy::PerRingNoCheck:
        return "per_ring_no_ck";
    case Strategy::SpikeSplit:
        return "spike_split";
    case Strategy::SpikeSplitNoCheck:
        return "spike_no_check";
    case Strategy::Adaptive:
        return "adaptive";
    }
    return "";
}

} // namespace

TEST_CASE("Clipper offset strategies benchmark", "[.][clipper-offset-strategies]")
{
    run_benchmark(7, 40.);
}

TEST_CASE("Clipper offset strategies quick check", "[.][clipper-offset-strategies-quick]")
{
    run_benchmark(0, 0.);
}

TEST_CASE("Clipper offset strategies contacts report", "[.][clipper-offset-strategies-contacts]")
{
    run_contacts_report();
}
