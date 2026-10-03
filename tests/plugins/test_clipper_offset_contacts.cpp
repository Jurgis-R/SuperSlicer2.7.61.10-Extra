/*
 * Clipper offset contacts
 * =======================
 *
 * A single `ClipperOffset` over every ring of an ExPolygon and the legacy
 * `offset_ex(ExPolygon)`, which offsets the contour and each hole separately and
 * subtracts the grown holes, cover the same area.  When a shrink makes rings
 * meet exactly along an edge they report a different topology: the grouped
 * offset keeps a zero-width bridge (an edge walked twice in opposite
 * directions, or a hole sharing an edge with its contour), the legacy path
 * splits the island or opens the hole there.
 *
 * The bridges come from the single clean-up union of all raw offset rings
 * inside `ClipperOffset::Execute`, whatever its fill rule.  A strictly simple
 * clean-up union was measured and changes nothing, because the contacts are
 * shared edges, not shared vertices.  Any later union of the cleaned rings
 * removes them.
 *
 * `slic3r_api::clipper_offset` implements `positive_fill_checked_union`.  This
 * file rebuilds it and the alternatives below directly with ClipperLib; the
 * contacts report counts vertices lying on another ring, or on a non-adjacent
 * edge of their own ring.  A bridge always shows as two collinear edges
 * overlapping along a positive length, which is cheap to detect by sorting the
 * edges along their support lines:
 *
 * | variant | passes | description |
 * |---------|--------|-------------|
 * | grouped_offset | offset | one ClipperOffset over all rings, PolyTree output, the former API |
 * | positive_fill_offset | offset | same, cleaned with a positive-fill union instead of the bounding rectangle |
 * | union_after | offset + union | API offset to Paths, then a non-zero union |
 * | checked_union_after | offset (+ union) | API offset, non-zero union only when two edges overlap |
 * | positive_fill_checked_union | offset (+ union) | positive-fill offset, then the same checked union: the API |
 * | ring_difference | ring offsets + difference | legacy ring offsets, difference straight to a PolyTree |
 * | legacy_offset_ex | ring offsets + difference + union | `Slic3r::offset_ex(ExPolygon)` |
 *
 * The `[plugins]` tests check that the API matches its rebuild, that every
 * variant covers the legacy area on the divergent cases, and that the
 * positive-fill clean-up covers the same area as the current one on concave
 * shapes for every join type.  The hidden `[clipper-offset-contacts-report]`
 * test prints topology and timing per variant and writes one SVG per case,
 * `clipper_offset_contacts_<case>.svg`, in the working directory: the input
 * followed by one panel per variant, with contact vertices in red.
 */

#include <catch2/catch.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/SVG.hpp"

namespace {

using namespace Slic3r;

using CanonicalRing = std::vector<std::pair<coord_t, coord_t>>;

struct CanonicalExPolygon {
    CanonicalRing contour;
    std::vector<CanonicalRing> holes;
};

using CanonicalShape = std::vector<CanonicalExPolygon>;

struct ContactCase {
    std::string name;
    Slic3r::ExPolygon input;
    double delta_mm;
    clipper_join_type_t join;
    double limit;
    size_t calls_per_batch;
};

enum class Variant {
    GroupedOffset,
    PositiveFillOffset,
    UnionAfter,
    CheckedUnionAfter,
    PositiveFillCheckedUnion,
    RingDifference,
    Legacy
};

const std::array<Variant, 7> all_variants = {
    Variant::GroupedOffset, Variant::PositiveFillOffset, Variant::UnionAfter, Variant::CheckedUnionAfter,
    Variant::PositiveFillCheckedUnion, Variant::RingDifference, Variant::Legacy};

// One edge placed on its support line: the reduced direction and the line
// offset identify the line, the projections bound the edge along it.
struct LineEdge {
    int64_t direction_x;
    int64_t direction_y;
    int64_t offset;
    int64_t begin;
    int64_t end;
};

// Vertices lying on another ring, or on a non-adjacent edge of their own ring.
struct Contacts {
    Slic3r::Points points;
    // Rings touching themselves: zero-width bridges inside one ring.
    size_t self_touching_rings = 0;
};

struct Topology {
    size_t islands = 0;
    size_t holes = 0;
    size_t contacts = 0;
    size_t self_touching_rings = 0;
};

struct VariantTiming {
    double median_us = 0.;
    double min_us = 0.;
};

// The four measured divergences between the API and the legacy offset.
std::vector<ContactCase> divergence_cases();

// Checkerboard holes growing until diagonal neighbours touch at their corners.
ContactCase checkerboard_case(const std::string &name, double delta_mm, size_t calls_per_batch);

// Concave shapes shrunk with every join type, for clean-up area equivalence.
std::vector<ContactCase> concave_cases();

// Counter-clockwise star centred on (x, y) with alternating radii in millimetres.
Slic3r::Polygon star_mm(double x, double y, double outer_radius, double inner_radius, size_t tips);

// Counter-clockwise comb: a base bar with teeth of equal width and gap.
Slic3r::Polygon comb_mm(double tooth_width, size_t teeth, double tooth_height);

// Build a counter-clockwise rectangle from millimetre bounds.
Slic3r::Polygon rectangle_mm(double min_x, double min_y, double max_x, double max_y);

// Build an ExPolygon from a contour and clockwise copies of the holes.
Slic3r::ExPolygon with_holes(const Slic3r::Polygon &contour, const Slic3r::Polygons &holes);

// Offset one case through the public API and materialize the result.
Slic3r::ExPolygons api_offset(const ContactCase &contact_case);

// Convert an API ExPolygon collection back to native ExPolygons.
Slic3r::ExPolygons to_native(const slic3r_api::ExPolygonCollection &collection);

// Run one variant on one case.
Slic3r::ExPolygons run_variant(Variant variant, const ContactCase &contact_case);

// One ClipperOffset over every ring as a PolyTree, optionally cleaned with a
// positive-fill union of the offset paths.
Slic3r::ExPolygons grouped_offset(const ContactCase &contact_case, bool positive_fill);

// One ClipperOffset over every ring to Paths, then a non-zero union.
Slic3r::ExPolygons grouped_offset_then_union(const ContactCase &contact_case);

// One ClipperOffset over every ring as a PolyTree, optionally cleaned with a
// positive fill, then a non-zero union of its rings only when two of their
// edges overlap.
Slic3r::ExPolygons grouped_offset_then_checked_union(const ContactCase &contact_case, bool positive_fill);

// Whether two edges of the tree overlap along a positive length.
bool has_overlapping_edges(const ClipperLib::PolyTree &tree);

// Append the edges of a node contour and of all its descendants.
void collect_edges(const ClipperLib::PolyNode &node, std::vector<LineEdge> &edges);

// Offset the contour and every hole separately, subtract into a PolyTree.
Slic3r::ExPolygons ring_difference(const ContactCase &contact_case);

// Apply the join limit and the shortest edge rule shared with ClipperUtils.
void configure_offsetter(ClipperLib::ClipperOffset &offsetter, const ContactCase &contact_case);

// Contour and holes of the input as Clipper paths, in API order.
ClipperLib::Paths ring_paths(const Slic3r::ExPolygon &expolygon);

// Collect a PolyTree into ExPolygons exactly like the API conversion.
Slic3r::ExPolygons expolygons_from_tree(const ClipperLib::PolyTree &tree);

ClipperLib::JoinType native_join(clipper_join_type_t join);

// Island, hole and contact counts of a result.
Topology topology(const Slic3r::ExPolygons &expolygons);

// Area covered by exactly one of the two results, in square millimetres.
double symmetric_area_mm2(const Slic3r::ExPolygons &first, const Slic3r::ExPolygons &second);

// Order-, start- and hole-order-independent form preserving every coordinate.
CanonicalShape canonical(const Slic3r::ExPolygons &expolygons);
CanonicalRing canonical_ring(const Slic3r::Polygon &polygon);
bool operator==(const CanonicalExPolygon &first, const CanonicalExPolygon &second);
bool operator<(const CanonicalExPolygon &first, const CanonicalExPolygon &second);

// Median and minimum time per call, variants interleaved batch by batch.
std::vector<VariantTiming> time_variants(const ContactCase &contact_case, size_t batches);

// Print one table row per variant for a case, and the rings of small results.
void print_report(const ContactCase &contact_case,
                  const std::vector<Slic3r::ExPolygons> &results,
                  const std::vector<VariantTiming> &timings);

// Print every ring of a result in millimetres.
void print_rings(const Slic3r::ExPolygons &expolygons);

// Draw the input and every variant result side by side.
void write_svg(const ContactCase &contact_case, const std::vector<Slic3r::ExPolygons> &results);

// Vertices of a result touching another ring or a non-adjacent edge of their own.
Contacts find_contacts(const Slic3r::ExPolygons &expolygons);

// Whether a point lies on the closed segment [a, b].
bool on_segment(const Slic3r::Point &point, const Slic3r::Point &a, const Slic3r::Point &b);

const char *variant_name(Variant variant);

std::vector<ContactCase> divergence_cases()
{
    return {
        {"88_hole_touching_contour_square",
         with_holes(rectangle_mm(105., 19., 118., 34.),
                    {rectangle_mm(110., 26., 112., 31.), rectangle_mm(110., 21., 115., 24.)}),
         -1., CLIPPER_JOIN_SQUARE, 3., 50},
        {"14_hole_splitting_island_miter3",
         with_holes(rectangle_mm(46., 2., 73., 12.),
                    {rectangle_mm(61., 4., 68., 10.), rectangle_mm(49., 5., 53., 9.)}),
         -1., CLIPPER_JOIN_MITER, 3., 50},
        {"244_holes_merging_contour_miter1.2",
         with_holes(rectangle_mm(85., 21., 104., 37.),
                    {rectangle_mm(93., 27., 97., 34.), rectangle_mm(91., 22., 98., 26.)}),
         -0.5, CLIPPER_JOIN_MITER, 1.2, 50},
        {"two_close_holes_square",
         with_holes(rectangle_mm(0., 0., 30., 30.),
                    {rectangle_mm(10., 10., 14., 14.), rectangle_mm(15., 15., 19., 19.)}),
         -5., CLIPPER_JOIN_SQUARE, 3., 50},
    };
}

ContactCase checkerboard_case(const std::string &name, double delta_mm, size_t calls_per_batch)
{
    // 6 mm holes on a 9 mm pitch, one cell out of two: growing them by 1.5 mm
    // with sharp miter corners makes diagonal neighbours meet exactly at their
    // corners.
    const size_t cells = 20;
    const double pitch = 9.;
    const double size = 6.;
    const double margin = 5.;
    Slic3r::Polygons holes;
    for (size_t row = 0; row < cells; ++row)
        for (size_t column = 0; column < cells; ++column)
            if ((row + column) % 2 == 0) {
                const double x = margin + pitch * column;
                const double y = margin + pitch * row;
                holes.push_back(rectangle_mm(x, y, x + size, y + size));
            }
    const double extent = 2. * margin + pitch * (cells - 1) + size;
    return {name, with_holes(rectangle_mm(0., 0., extent, extent), holes), delta_mm, CLIPPER_JOIN_MITER, 3.,
            calls_per_batch};
}

std::vector<ContactCase> concave_cases()
{
    std::vector<std::pair<std::string, Slic3r::ExPolygon>> shapes = {
        {"star", Slic3r::ExPolygon(star_mm(0., 0., 20., 8., 7))},
        {"holed_star", with_holes(star_mm(0., 0., 20., 8., 7), {rectangle_mm(-3., -2., 3., 2.)})},
        {"comb", Slic3r::ExPolygon(comb_mm(1., 8, 6.))},
        {"holed_comb", with_holes(comb_mm(1.2, 6, 5.), {rectangle_mm(2., 0.8, 9., 1.6)})},
    };
    for (const ContactCase &divergence : divergence_cases())
        shapes.emplace_back(divergence.name, divergence.input);

    const std::vector<std::pair<clipper_join_type_t, double>> joins = {
        {CLIPPER_JOIN_SQUARE, 3.}, {CLIPPER_JOIN_MITER, 3.}, {CLIPPER_JOIN_MITER, 1.2},
        {CLIPPER_JOIN_ROUND, scale_d(0.02)}};
    std::vector<ContactCase> cases;
    for (const std::pair<std::string, Slic3r::ExPolygon> &shape : shapes)
        for (const std::pair<clipper_join_type_t, double> &join : joins)
            for (double delta_mm : {0., -0.3, -0.6, -1., -2.5, -5.})
                cases.push_back({shape.first, shape.second, delta_mm, join.first, join.second, 1});
    return cases;
}

Slic3r::Polygon star_mm(double x, double y, double outer_radius, double inner_radius, size_t tips)
{
    Slic3r::Polygon polygon;
    for (size_t index = 0; index < 2 * tips; ++index) {
        const double angle = PI * double(index) / double(tips);
        const double radius = index % 2 == 0 ? outer_radius : inner_radius;
        polygon.points.emplace_back(scale_i(x + radius * std::cos(angle)), scale_i(y + radius * std::sin(angle)));
    }
    return polygon;
}

Slic3r::Polygon comb_mm(double tooth_width, size_t teeth, double tooth_height)
{
    // Base bar from y = 0 to y = 2, teeth rising from its top edge.
    const double width = tooth_width * double(2 * teeth - 1);
    Slic3r::Polygon polygon;
    polygon.points.emplace_back(scale_i(0.), scale_i(0.));
    polygon.points.emplace_back(scale_i(width), scale_i(0.));
    for (size_t tooth = teeth; tooth > 0; --tooth) {
        const double right = tooth_width * double(2 * tooth - 1);
        const double left = right - tooth_width;
        if (tooth != teeth)
            polygon.points.emplace_back(scale_i(right), scale_i(2.));
        polygon.points.emplace_back(scale_i(right), scale_i(2. + tooth_height));
        polygon.points.emplace_back(scale_i(left), scale_i(2. + tooth_height));
        if (tooth != 1)
            polygon.points.emplace_back(scale_i(left), scale_i(2.));
    }
    return polygon;
}

Slic3r::Polygon rectangle_mm(double min_x, double min_y, double max_x, double max_y)
{
    Slic3r::Polygon polygon({
        Slic3r::Point(scale_i(min_x), scale_i(min_y)),
        Slic3r::Point(scale_i(max_x), scale_i(min_y)),
        Slic3r::Point(scale_i(max_x), scale_i(max_y)),
        Slic3r::Point(scale_i(min_x), scale_i(max_y))
    });
    polygon.make_counter_clockwise();
    return polygon;
}

Slic3r::ExPolygon with_holes(const Slic3r::Polygon &contour, const Slic3r::Polygons &holes)
{
    Slic3r::ExPolygon result(contour);
    for (const Slic3r::Polygon &hole : holes) {
        Slic3r::Polygon oriented_hole(hole);
        oriented_hole.make_clockwise();
        result.holes.push_back(std::move(oriented_hole));
    }
    return result;
}

Slic3r::ExPolygons api_offset(const ContactCase &contact_case)
{
    Slic3r::PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    slic3r_api::StoredExPolygonCollection collection(storage);
    slic3r_api::StoredExPolygon stored(storage);
    stored.copy_from(slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&contact_case.input)));
    collection.push_back_move(std::move(stored));

    slic3r_api::ClipperOperand operand(storage, collection.readonly());
    slic3r_api::ClipperOperand offset = slic3r_api::clipper_offset(
        operand, scale_d(contact_case.delta_mm), contact_case.join, contact_case.limit);
    slic3r_api::StoredExPolygonCollection result = offset.to_expolygon_collection();
    return to_native(result.readonly());
}

Slic3r::ExPolygons to_native(const slic3r_api::ExPolygonCollection &collection)
{
    Slic3r::ExPolygons result;
    result.reserve(collection.size());
    for (const slic3r_api::ExPolygon source : collection) {
        Slic3r::ExPolygon native;
        for (const c_point point : source.contour().points())
            native.contour.points.emplace_back(point.x, point.y);
        for (const slic3r_api::Polygon source_hole : source.holes()) {
            Slic3r::Polygon hole;
            for (const c_point point : source_hole.points())
                hole.points.emplace_back(point.x, point.y);
            native.holes.push_back(std::move(hole));
        }
        result.push_back(std::move(native));
    }
    return result;
}

Slic3r::ExPolygons run_variant(Variant variant, const ContactCase &contact_case)
{
    switch (variant) {
    case Variant::GroupedOffset:
        return grouped_offset(contact_case, false);
    case Variant::PositiveFillOffset:
        return grouped_offset(contact_case, true);
    case Variant::UnionAfter:
        return grouped_offset_then_union(contact_case);
    case Variant::CheckedUnionAfter:
        return grouped_offset_then_checked_union(contact_case, false);
    case Variant::PositiveFillCheckedUnion:
        return grouped_offset_then_checked_union(contact_case, true);
    case Variant::RingDifference:
        return ring_difference(contact_case);
    case Variant::Legacy:
        return Slic3r::offset_ex(contact_case.input, scale_d(contact_case.delta_mm), native_join(contact_case.join),
                                 contact_case.limit);
    }
    return {};
}

Slic3r::ExPolygons grouped_offset(const ContactCase &contact_case, bool positive_fill)
{
    ClipperLib::ClipperOffset offsetter;
    configure_offsetter(offsetter, contact_case);
    offsetter.ShrinkWithPositiveFill = positive_fill;
    offsetter.AddPaths(ring_paths(contact_case.input), native_join(contact_case.join), ClipperLib::etClosedPolygon);
    ClipperLib::PolyTree tree;
    offsetter.Execute(tree, scale_d(contact_case.delta_mm));
    return expolygons_from_tree(tree);
}

Slic3r::ExPolygons grouped_offset_then_union(const ContactCase &contact_case)
{
    ClipperLib::ClipperOffset offsetter;
    configure_offsetter(offsetter, contact_case);
    offsetter.AddPaths(ring_paths(contact_case.input), native_join(contact_case.join), ClipperLib::etClosedPolygon);
    ClipperLib::Paths offset;
    offsetter.Execute(offset, scale_d(contact_case.delta_mm));

    ClipperLib::Clipper clipper;
    clipper.AddPaths(offset, ClipperLib::ptSubject, true);
    ClipperLib::PolyTree tree;
    clipper.Execute(ClipperLib::ctUnion, tree, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
    return expolygons_from_tree(tree);
}

Slic3r::ExPolygons grouped_offset_then_checked_union(const ContactCase &contact_case, bool positive_fill)
{
    ClipperLib::ClipperOffset offsetter;
    configure_offsetter(offsetter, contact_case);
    offsetter.ShrinkWithPositiveFill = positive_fill;
    offsetter.AddPaths(ring_paths(contact_case.input), native_join(contact_case.join), ClipperLib::etClosedPolygon);
    ClipperLib::PolyTree tree;
    offsetter.Execute(tree, scale_d(contact_case.delta_mm));
    if (!has_overlapping_edges(tree))
        return expolygons_from_tree(tree);

    ClipperLib::Paths rings;
    ClipperLib::PolyTreeToPaths(std::move(tree), rings);
    ClipperLib::Clipper clipper;
    clipper.AddPaths(rings, ClipperLib::ptSubject, true);
    ClipperLib::PolyTree cleaned;
    clipper.Execute(ClipperLib::ctUnion, cleaned, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
    return expolygons_from_tree(cleaned);
}

bool has_overlapping_edges(const ClipperLib::PolyTree &tree)
{
    std::vector<LineEdge> edges;
    for (const ClipperLib::PolyNode *child : tree.Childs)
        collect_edges(*child, edges);
    std::sort(edges.begin(), edges.end(), [](const LineEdge &first, const LineEdge &second) {
        return std::tie(first.direction_x, first.direction_y, first.offset, first.begin) <
               std::tie(second.direction_x, second.direction_y, second.offset, second.begin);
    });
    // Edges of one line are sorted by their start: one overlaps an earlier edge
    // when it starts before the furthest end reached so far on that line.
    int64_t furthest_end = 0;
    for (size_t index = 0; index < edges.size(); ++index) {
        const LineEdge &current = edges[index];
        const bool same_line = index > 0 && current.direction_x == edges[index - 1].direction_x &&
                               current.direction_y == edges[index - 1].direction_y &&
                               current.offset == edges[index - 1].offset;
        if (same_line && current.begin < furthest_end)
            return true;
        furthest_end = same_line ? std::max(furthest_end, current.end) : current.end;
    }
    return false;
}

void collect_edges(const ClipperLib::PolyNode &node, std::vector<LineEdge> &edges)
{
    const ClipperLib::Path &contour = node.Contour;
    for (size_t index = 0; index < contour.size(); ++index) {
        const ClipperLib::IntPoint &a = contour[index];
        const ClipperLib::IntPoint &b = contour[(index + 1) % contour.size()];
        int64_t direction_x = b.x() - a.x();
        int64_t direction_y = b.y() - a.y();
        const int64_t divisor = std::gcd(direction_x, direction_y);
        if (divisor == 0)
            continue;
        // Orient every direction the same way so opposite edges share a line.
        direction_x /= divisor;
        direction_y /= divisor;
        if (direction_x < 0 || (direction_x == 0 && direction_y < 0)) {
            direction_x = -direction_x;
            direction_y = -direction_y;
        }
        const int64_t projection_a = direction_x * a.x() + direction_y * a.y();
        const int64_t projection_b = direction_x * b.x() + direction_y * b.y();
        edges.push_back({direction_x, direction_y, direction_x * a.y() - direction_y * a.x(),
                         std::min(projection_a, projection_b), std::max(projection_a, projection_b)});
    }
    for (const ClipperLib::PolyNode *child : node.Childs)
        collect_edges(*child, edges);
}

Slic3r::ExPolygons ring_difference(const ContactCase &contact_case)
{
    const ClipperLib::JoinType join = native_join(contact_case.join);
    const double delta = scale_d(contact_case.delta_mm);
    ClipperLib::ClipperOffset offsetter;
    configure_offsetter(offsetter, contact_case);
    ClipperLib::Clipper clipper;

    ClipperLib::Paths offset;
    offsetter.AddPath(contact_case.input.contour.points, join, ClipperLib::etClosedPolygon);
    offsetter.Execute(offset, delta);
    clipper.AddPaths(offset, ClipperLib::ptSubject, true);
    for (const Slic3r::Polygon &hole : contact_case.input.holes) {
        // Execute reorients the clockwise hole, so it grows with the opposite sign.
        offsetter.Clear();
        offsetter.AddPath(hole.points, join, ClipperLib::etClosedPolygon);
        offsetter.Execute(offset, -delta);
        clipper.AddPaths(offset, ClipperLib::ptClip, true);
    }

    ClipperLib::PolyTree tree;
    clipper.Execute(ClipperLib::ctDifference, tree, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
    return expolygons_from_tree(tree);
}

void configure_offsetter(ClipperLib::ClipperOffset &offsetter, const ContactCase &contact_case)
{
    if (contact_case.join == CLIPPER_JOIN_ROUND)
        offsetter.ArcTolerance = contact_case.limit;
    else
        offsetter.MiterLimit = contact_case.limit;
    offsetter.ShortestEdgeLength = std::abs(scale_d(contact_case.delta_mm) * Slic3r::ClipperOffsetShortestEdgeFactor);
}

ClipperLib::Paths ring_paths(const Slic3r::ExPolygon &expolygon)
{
    ClipperLib::Paths paths;
    paths.reserve(expolygon.holes.size() + 1);
    paths.emplace_back(expolygon.contour.points);
    for (const Slic3r::Polygon &hole : expolygon.holes)
        paths.emplace_back(hole.points);
    return paths;
}

Slic3r::ExPolygons expolygons_from_tree(const ClipperLib::PolyTree &tree)
{
    Slic3r::ExPolygons result;
    Slic3r::traverse_pt(&tree, &result);
    return result;
}

ClipperLib::JoinType native_join(clipper_join_type_t join)
{
    switch (join) {
    case CLIPPER_JOIN_SQUARE:
        return ClipperLib::jtSquare;
    case CLIPPER_JOIN_ROUND:
        return ClipperLib::jtRound;
    case CLIPPER_JOIN_MITER:
        return ClipperLib::jtMiter;
    }
    return ClipperLib::jtMiter;
}

Topology topology(const Slic3r::ExPolygons &expolygons)
{
    Topology result;
    result.islands = expolygons.size();
    for (const Slic3r::ExPolygon &expolygon : expolygons)
        result.holes += expolygon.holes.size();
    const Contacts contacts = find_contacts(expolygons);
    result.contacts = contacts.points.size();
    result.self_touching_rings = contacts.self_touching_rings;
    return result;
}

Contacts find_contacts(const Slic3r::ExPolygons &expolygons)
{
    std::vector<const Slic3r::Polygon *> rings;
    for (const Slic3r::ExPolygon &expolygon : expolygons) {
        rings.push_back(&expolygon.contour);
        for (const Slic3r::Polygon &hole : expolygon.holes)
            rings.push_back(&hole);
    }

    Contacts result;
    for (const Slic3r::Polygon *ring : rings) {
        bool self_touching = false;
        const size_t count = ring->points.size();
        for (size_t vertex = 0; vertex < count; ++vertex)
            for (const Slic3r::Polygon *other : rings) {
                const size_t other_count = other->points.size();
                for (size_t edge = 0; edge < other_count; ++edge) {
                    const size_t next = (edge + 1) % other_count;
                    // A vertex always lies on its two own edges.
                    if (other == ring && (edge == vertex || next == vertex))
                        continue;
                    if (on_segment(ring->points[vertex], other->points[edge], other->points[next])) {
                        result.points.push_back(ring->points[vertex]);
                        self_touching |= other == ring;
                    }
                }
            }
        if (self_touching)
            ++result.self_touching_rings;
    }
    std::sort(result.points.begin(), result.points.end());
    result.points.erase(std::unique(result.points.begin(), result.points.end()), result.points.end());
    return result;
}

bool on_segment(const Slic3r::Point &point, const Slic3r::Point &a, const Slic3r::Point &b)
{
    const int64_t cross = int64_t(b.x() - a.x()) * int64_t(point.y() - a.y()) -
                          int64_t(b.y() - a.y()) * int64_t(point.x() - a.x());
    return cross == 0 && std::min(a.x(), b.x()) <= point.x() && point.x() <= std::max(a.x(), b.x()) &&
           std::min(a.y(), b.y()) <= point.y() && point.y() <= std::max(a.y(), b.y());
}

double symmetric_area_mm2(const Slic3r::ExPolygons &first, const Slic3r::ExPolygons &second)
{
    const Slic3r::Polygons first_polygons = Slic3r::to_polygons(first);
    const Slic3r::Polygons second_polygons = Slic3r::to_polygons(second);
    double area = 0.;
    for (const Slic3r::ExPolygon &expolygon : Slic3r::diff_ex(first_polygons, second_polygons))
        area += std::abs(expolygon.area());
    for (const Slic3r::ExPolygon &expolygon : Slic3r::diff_ex(second_polygons, first_polygons))
        area += std::abs(expolygon.area());
    const double scale = scale_d(1.);
    return area / (scale * scale);
}

CanonicalShape canonical(const Slic3r::ExPolygons &expolygons)
{
    CanonicalShape result;
    result.reserve(expolygons.size());
    for (const Slic3r::ExPolygon &expolygon : expolygons) {
        CanonicalExPolygon canonical_expolygon;
        canonical_expolygon.contour = canonical_ring(expolygon.contour);
        for (const Slic3r::Polygon &hole : expolygon.holes)
            canonical_expolygon.holes.push_back(canonical_ring(hole));
        std::sort(canonical_expolygon.holes.begin(), canonical_expolygon.holes.end());
        result.push_back(std::move(canonical_expolygon));
    }
    std::sort(result.begin(), result.end());
    return result;
}

CanonicalRing canonical_ring(const Slic3r::Polygon &polygon)
{
    CanonicalRing result;
    if (polygon.points.empty())
        return result;

    const size_t first = size_t(std::min_element(polygon.points.begin(), polygon.points.end()) - polygon.points.begin());
    result.reserve(polygon.points.size());
    for (size_t offset = 0; offset < polygon.points.size(); ++offset) {
        const Slic3r::Point &point = polygon.points[(first + offset) % polygon.points.size()];
        result.emplace_back(point.x(), point.y());
    }
    return result;
}

bool operator==(const CanonicalExPolygon &first, const CanonicalExPolygon &second)
{
    return first.contour == second.contour && first.holes == second.holes;
}

bool operator<(const CanonicalExPolygon &first, const CanonicalExPolygon &second)
{
    return std::tie(first.contour, first.holes) < std::tie(second.contour, second.holes);
}

std::vector<VariantTiming> time_variants(const ContactCase &contact_case, size_t batches)
{
    std::vector<std::vector<double>> samples(all_variants.size());
    for (size_t batch = 0; batch < batches; ++batch)
        for (size_t variant = 0; variant < all_variants.size(); ++variant) {
            const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
            for (size_t call = 0; call < contact_case.calls_per_batch; ++call)
                run_variant(all_variants[variant], contact_case);
            const std::chrono::duration<double, std::micro> elapsed = std::chrono::steady_clock::now() - start;
            samples[variant].push_back(elapsed.count() / double(contact_case.calls_per_batch));
        }

    std::vector<VariantTiming> timings(all_variants.size());
    for (size_t variant = 0; variant < all_variants.size(); ++variant) {
        std::vector<double> &variant_samples = samples[variant];
        std::sort(variant_samples.begin(), variant_samples.end());
        timings[variant].median_us = variant_samples[variant_samples.size() / 2];
        timings[variant].min_us = variant_samples.front();
    }
    return timings;
}

void print_report(const ContactCase &contact_case,
                  const std::vector<Slic3r::ExPolygons> &results,
                  const std::vector<VariantTiming> &timings)
{
    const Slic3r::ExPolygons &legacy = results.back();
    const CanonicalShape legacy_canonical = canonical(legacy);
    const double legacy_median = timings.back().median_us;
    std::cout << "\n== " << contact_case.name << " (delta " << contact_case.delta_mm << " mm, "
              << contact_case.input.holes.size() << " holes)\n"
              << std::left << std::setw(30) << "variant" << std::right << std::setw(8) << "islands"
              << std::setw(7) << "holes" << std::setw(10) << "contacts" << std::setw(8) << "bridges"
              << std::setw(8) << "legacy" << std::setw(12) << "area_mm2" << std::setw(12) << "median_us"
              << std::setw(10) << "min_us" << std::setw(9) << "ratio" << '\n';
    for (size_t variant = 0; variant < all_variants.size(); ++variant) {
        const Topology shape = topology(results[variant]);
        const bool exact = canonical(results[variant]) == legacy_canonical;
        std::cout << std::left << std::setw(30) << variant_name(all_variants[variant]) << std::right
                  << std::setw(8) << shape.islands << std::setw(7) << shape.holes << std::setw(10) << shape.contacts
                  << std::setw(8) << shape.self_touching_rings << std::setw(8) << (exact ? "same" : "diff")
                  << std::setw(12) << std::setprecision(3) << symmetric_area_mm2(results[variant], legacy)
                  << std::setw(12) << std::fixed << std::setprecision(1) << timings[variant].median_us
                  << std::setw(10) << timings[variant].min_us << std::setw(9) << std::setprecision(2)
                  << timings[variant].median_us / legacy_median << std::defaultfloat << '\n';
    }
    if (contact_case.input.holes.size() > 2)
        return;
    for (size_t variant = 0; variant < all_variants.size(); ++variant) {
        std::cout << "  " << variant_name(all_variants[variant]) << ":\n";
        print_rings(results[variant]);
    }
}

void print_rings(const Slic3r::ExPolygons &expolygons)
{
    auto print_ring = [](const char *kind, const Slic3r::Polygon &ring) {
        std::cout << "    " << kind << std::setprecision(10);
        for (const Slic3r::Point &point : ring.points)
            std::cout << " (" << unscaled(point.x()) << ',' << unscaled(point.y()) << ')';
        std::cout << '\n';
    };
    for (const Slic3r::ExPolygon &expolygon : expolygons) {
        print_ring("contour", expolygon.contour);
        for (const Slic3r::Polygon &hole : expolygon.holes)
            print_ring("  hole", hole);
    }
}

void write_svg(const ContactCase &contact_case, const std::vector<Slic3r::ExPolygons> &results)
{
    const Slic3r::BoundingBox input_box = Slic3r::get_extents(contact_case.input);
    const coord_t step = input_box.size().x() + scale_i(4.);
    const coord_t label_y = input_box.max.y() + scale_i(1.5);
    Slic3r::BoundingBox canvas = input_box;
    canvas.merge(Slic3r::Point(input_box.max.x() + step * coord_t(results.size()), label_y + scale_i(1.)));

    Slic3r::SVG svg("clipper_offset_contacts_" + contact_case.name + ".svg", canvas, scale_i(3.));
    svg.draw(contact_case.input, "lightgrey");
    svg.draw_outline(contact_case.input, "black", "blue", scale_i(0.05));
    svg.draw_text(Slic3r::Point(input_box.min.x(), label_y), "input", "black");
    for (size_t variant = 0; variant < results.size(); ++variant) {
        const Slic3r::Point shift(step * coord_t(variant + 1), 0);
        Slic3r::ExPolygons panel = results[variant];
        for (Slic3r::ExPolygon &expolygon : panel)
            expolygon.translate(shift);
        svg.draw(panel, "lightskyblue");
        svg.draw_outline(panel, "black", "blue", scale_i(0.05));
        svg.draw(find_contacts(panel).points, "red", scale_i(0.3));
        svg.draw_text(Slic3r::Point(input_box.min.x() + shift.x(), label_y), variant_name(all_variants[variant]),
                      "black");
    }
    svg.Close();
}

const char *variant_name(Variant variant)
{
    switch (variant) {
    case Variant::GroupedOffset:
        return "grouped_offset";
    case Variant::PositiveFillOffset:
        return "positive_fill_offset";
    case Variant::UnionAfter:
        return "union_after";
    case Variant::CheckedUnionAfter:
        return "checked_union_after";
    case Variant::PositiveFillCheckedUnion:
        return "positive_fill_checked_union";
    case Variant::RingDifference:
        return "ring_difference";
    case Variant::Legacy:
        return "legacy_offset_ex";
    }
    return "";
}

} // namespace

TEST_CASE("Clipper offset contacts: variants cover the legacy area", "[plugins][clipper][clipper-offset-contacts]")
{
    for (const ContactCase &contact_case : divergence_cases()) {
        INFO(contact_case.name);
        CHECK(canonical(run_variant(Variant::PositiveFillCheckedUnion, contact_case)) ==
              canonical(api_offset(contact_case)));
        const Slic3r::ExPolygons legacy = run_variant(Variant::Legacy, contact_case);
        for (Variant variant : all_variants) {
            INFO(variant_name(variant));
            CHECK(symmetric_area_mm2(run_variant(variant, contact_case), legacy) == Approx(0.).margin(1e-9));
        }
        // A union in the normal orientation restores the legacy rings exactly.
        for (Variant variant : {Variant::UnionAfter, Variant::CheckedUnionAfter, Variant::PositiveFillCheckedUnion,
                                Variant::RingDifference}) {
            INFO(variant_name(variant));
            CHECK(canonical(run_variant(variant, contact_case)) == canonical(legacy));
        }
    }
}

TEST_CASE("Clipper offset contacts: positive-fill clean-up covers the same area",
          "[plugins][clipper][clipper-offset-contacts]")
{
    for (const ContactCase &contact_case : concave_cases()) {
        INFO(contact_case.name << " delta " << contact_case.delta_mm << " join " << int(contact_case.join)
                               << " limit " << contact_case.limit);
        CHECK(symmetric_area_mm2(run_variant(Variant::PositiveFillOffset, contact_case),
                                 run_variant(Variant::GroupedOffset, contact_case)) == Approx(0.).margin(1e-9));
        CHECK(canonical(run_variant(Variant::PositiveFillCheckedUnion, contact_case)) ==
              canonical(api_offset(contact_case)));
    }
}

TEST_CASE("Clipper offset contacts: topology and timing report", "[.][clipper-offset-contacts-report]")
{
    std::vector<ContactCase> cases = divergence_cases();
    cases.push_back(checkerboard_case("checkerboard_touching_corners", -1.5, 2));
    cases.push_back(checkerboard_case("checkerboard_no_contact", -1., 2));
    for (const ContactCase &contact_case : cases) {
        std::vector<Slic3r::ExPolygons> results;
        for (Variant variant : all_variants)
            results.push_back(run_variant(variant, contact_case));
        print_report(contact_case, results, time_variants(contact_case, 41));
        write_svg(contact_case, results);
    }
}
