///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_Api_plugin_cpp_OrderingEngine_hpp_
#define slic3r_Api_plugin_cpp_OrderingEngine_hpp_

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_geometry.h"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"

namespace slic3r_api { namespace Ordering {

/*
Geometry-only interface for arranging independent printable items.

The engine deliberately knows nothing about PrintingExtrusion, extrusion
trees, seams or reversals. A caller describes every legal way to enter and
leave an item with OrderingCandidate values, asks an OrderingEngine for a
sequence, then applies that sequence to its own data structures.

Entry and exit positions are estimates. Later processing may move a seam, add
a seam gap or append a wipe without invalidating the ordering contract. The
engine only tries to reduce the estimated transition cost between items.
*/
struct OrderingCandidate
{
    // Approximate position at which printing this candidate begins. Coordinates
    // use the scaled slicer coordinate system carried by c_point.
    c_point estimated_entry = {};

    // Approximate position left after printing the candidate. A closed loop
    // normally uses the same point for entry and exit.
    c_point estimated_exit = {};

    // Dimensionless weight applied to the XY distance travelled to this
    // candidate. Zero makes the candidate's transition cost independent of
    // distance and leaves only penalty_mm.
    double distance_ratio = 1.0;

    // Intrinsic cost of selecting this candidate, expressed in millimetres so
    // it can be added to the weighted transition distance.
    double penalty_mm = 0.0;
};

/*
One independently reorderable item.

Multiple candidates describe alternatives for the same item, not duplicate
items. For example, a reversible open path exposes front-to-back and
back-to-front candidates, while a loop may expose one candidate per possible
seam. Every item must contain at least one candidate.
*/
struct OrderingItem
{
    std::vector<OrderingCandidate> candidates;
};

/*
One selected item in its new sequence position.

item_index always refers to the original input vector. selected_candidate is a
copy of the entry/exit estimate selected by the engine, allowing the caller to
sort nested content without retaining pointers into the request.
*/
struct OrderedItem
{
    uint32_t item_index = 0;
    OrderingCandidate selected_candidate;
};

/*
Replaceable ordering strategy used by PrintingPlan ordering plugins.

Implementations must return each input item exactly once. They must not mutate
the request. The transition cost for a destination candidate is:

    distance_xy_mm * distance_ratio + penalty_mm

When initial_position is absent, implementations choose a deterministic first
item without charging travel from an invented machine position.
*/
class OrderingEngine
{
public:
    /* Destroy an implementation through the public strategy interface. */
    virtual ~OrderingEngine() = default;

    /*
    Return a complete permutation of items and the candidate selected for each.

    Implementations should reject malformed requests with std::invalid_argument
    rather than returning a partial sequence. An empty request returns an empty
    result.
    */
    virtual std::vector<OrderedItem> order(
        const std::vector<OrderingItem> &items,
        std::optional<c_point> initial_position = std::nullopt) const = 0;
};

/*
Order and orient the polylines of a collection in place with the provided
engine.

Each polyline becomes one item exposing its two directions as candidates, so the
engine may start it from either end. The collection keeps the same polylines:
this helper never joins two of them. That property is what makes it safe on
clipped geometry, where the pieces are separated by holes or voids: a joined
path would extrude a straight segment across the gap. Travels between the
ordered paths are inserted later, for example by STEP_ORDERING.

Polylines with fewer than two points carry no extrusion; they are not routed and
keep their relative order after the routed ones. A single routable polyline is
left in place because ordering it cannot change anything.

initial_position is forwarded to OrderingEngine::order(), so a caller may chain
several collections. The returned optional is the exit point of the last routed
polyline, which is the point to pass as the next collection's initial_position.
It is initial_position itself when nothing was routed.

The engine is expected to return each item exactly once, as documented by
OrderingEngine::order(). Malformed permutations are rejected with
std::runtime_error instead of silently corrupting the printed sequence.
*/
inline std::optional<c_point> connect_infill(StoredPolylineCollection &to_order,
                                             const OrderingEngine &engine,
                                             std::optional<c_point> initial_position = std::nullopt)
{
    if (!to_order.valid())
        return initial_position;

    const uint32_t count = to_order.size();
    if (count < 2)
        return initial_position;

    std::vector<OrderingItem> items;
    std::vector<uint32_t> routed_indices;
    std::vector<c_point> fronts(count);
    std::vector<c_point> backs(count);
    std::vector<uint32_t> degenerates;
    items.reserve(count);
    routed_indices.reserve(count);
    degenerates.reserve(count);
    for (uint32_t index = 0; index < count; ++index) {
        const Polyline polyline = to_order.at(index);
        if (polyline.size() < 2) {
            degenerates.push_back(index);
            continue;
        }

        fronts[index] = polyline.front();
        backs[index] = polyline.back();
        OrderingItem item;
        item.candidates.push_back(OrderingCandidate{fronts[index], backs[index]});
        item.candidates.push_back(OrderingCandidate{backs[index], fronts[index]});
        items.emplace_back(std::move(item));
        routed_indices.push_back(index);
    }

    if (items.size() < 2)
        return initial_position;

    const std::vector<OrderedItem> ordered = engine.order(items, initial_position);
    if (ordered.size() != items.size())
        throw std::runtime_error("connect_infill: the ordering engine returned an incomplete permutation.");

    // target[slot] is the original polyline index placed at that slot. The
    // routed polylines come first, then the untouched degenerate ones.
    std::vector<uint32_t> target;
    std::vector<uint8_t> reverse_original(count, uint8_t(0));
    std::vector<uint8_t> selected(items.size(), uint8_t(0));
    target.reserve(count);
    for (const OrderedItem &selection : ordered) {
        if (selection.item_index >= items.size() || selected[selection.item_index] != 0)
            throw std::runtime_error("connect_infill: the ordering engine returned an invalid permutation.");
        selected[selection.item_index] = 1;

        const uint32_t original = routed_indices[selection.item_index];
        target.push_back(original);
        const c_point entry = selection.selected_candidate.estimated_entry;
        // Reversing on an equal entry/exit is unnecessary: the polyline starts
        // and ends on the selected point already.
        if ((entry.x != fronts[original].x || entry.y != fronts[original].y) &&
            entry.x == backs[original].x && entry.y == backs[original].y)
            reverse_original[original] = 1;
    }
    for (const uint32_t original : degenerates)
        target.push_back(original);

    // Apply the permutation in place. One scratch polyline is appended so two
    // elements can be swapped with polyline_move(), then it is removed again.
    polyline_collection_handle *collection = to_order.mutable_handle();
    polylines_emplace_back(collection);
    std::vector<uint32_t> occupant(count);
    std::vector<uint32_t> position(count);
    for (uint32_t index = 0; index < count; ++index) {
        occupant[index] = index;
        position[index] = index;
    }
    for (uint32_t slot = 0; slot < count; ++slot) {
        const uint32_t current = position[target[slot]];
        if (current == slot)
            continue;

        polyline_handle *scratch = polylines_at(collection, count);
        polyline_handle *destination = polylines_at(collection, slot);
        polyline_handle *source = polylines_at(collection, current);
        polyline_move(scratch, destination);
        polyline_move(destination, source);
        polyline_move(source, scratch);

        const uint32_t moved_out = occupant[slot];
        const uint32_t moved_in = occupant[current];
        occupant[slot] = moved_in;
        position[moved_in] = slot;
        occupant[current] = moved_out;
        position[moved_out] = current;
    }
    polylines_pop_back(collection);

    for (uint32_t slot = 0; slot < count; ++slot)
        if (reverse_original[target[slot]] != 0)
            multipoint_reverse(polyline_as_multipoint(polylines_at(collection, slot)));

    return ordered.back().selected_candidate.estimated_exit;
}

}} // namespace slic3r_api::Ordering

#endif // slic3r_Api_plugin_cpp_OrderingEngine_hpp_
