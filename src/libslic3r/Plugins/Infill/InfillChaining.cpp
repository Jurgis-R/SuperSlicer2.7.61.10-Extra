///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ Copyright (c) Prusa Research 2017 - 2022 Vojtěch Bubník
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

// Own API/work conversions and path ordering; keep chaining tie handling synchronized with libslic3r/ShortestPath.cpp.

#include "InfillPaths.hpp"

#include <array>
#include <stdexcept>

#include "libslic3r/Api/plugin/cpp/KDTreeIndirect.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"

namespace slic3r_api { namespace Infill { namespace Paths {
namespace {

using work::Polyline;
using work::Polylines;
using work::Vec2d;
using work::to_vec2d;
constexpr size_t invalid = std::numeric_limits<size_t>::max();

struct EndPoint
{
    std::array<double, 2> position;
    EndPoint *candidate = nullptr;
    double distance = std::numeric_limits<double>::max();
    size_t heap_index = invalid;
    size_t chain = 0;
    EndPoint *next = nullptr;
};

struct Edge
{
    Vec2d first, last;
    size_t source;
    void flip() { std::swap(first, last); }
};
struct Cost
{
    double normal = 0.;
    double flipped = 0.;
};
using Span = std::pair<size_t, size_t>;

// Whole paths remain independent; endpoint links describe travel order, not extruded connectors.
Polylines greedy_chain(Polylines lines, const plugin_run_context *run_ctx);
// Two exchanges shorten travel while retaining each source path and its possible reversal.
void improve(std::vector<Edge> &edges, const plugin_run_context *run_ctx);
// Search native span permutations and reversal bits without changing equal-cost choices.
std::pair<double, size_t> minimum_crossover(const std::vector<Edge> &edges, const std::array<Span, 3> &spans,
                                           const std::array<Cost, 3> &costs, double current);
// Endpoint costs use double precision before heap tie comparisons.
double distance(const EndPoint &a, const EndPoint &b);
// Prefix cost differences retain both normal and flipped travel totals.
Cost operator-(const Cost &a, const Cost &b);

double distance(const EndPoint &a, const EndPoint &b)
{
    return Vec2d(a.position[0] - b.position[0], a.position[1] - b.position[1]).norm();
}

// Equal costs must keep the native mutable heap's exchange order.
class Queue
{
public:
    explicit Queue(size_t count) { m_items.reserve(count); }
    EndPoint *top() const { assert(!m_items.empty()); return m_items.front(); }
    bool empty() const { return m_items.empty(); }
    void push(EndPoint *point)
    {
        point->heap_index = m_items.size();
        m_items.push_back(point);
        up(m_items.size() - 1);
    }
    void pop()
    {
        assert(!m_items.empty());
        m_items.front()->heap_index = invalid;
        if (m_items.size() == 1) {
            m_items.clear();
            return;
        }
        m_items.front() = m_items.back();
        m_items.pop_back();
        m_items.front()->heap_index = 0;
        down(0);
    }
    void remove(size_t index)
    {
        assert(index < m_items.size());
        m_items[index]->heap_index = invalid;
        if (index + 1 == m_items.size()) {
            m_items.pop_back();
            return;
        }
        m_items[index] = m_items.back();
        m_items[index]->heap_index = index;
        m_items.pop_back();
        down(index);
        up(index);
    }
    void update(EndPoint *point)
    {
        if (point->heap_index != invalid)
            remove(point->heap_index);
        push(point);
    }
private:
    void exchange(size_t a, size_t b)
    {
        std::swap(m_items[a], m_items[b]);
        m_items[a]->heap_index = a;
        m_items[b]->heap_index = b;
    }
    void up(size_t child)
    {
        while (child > 0) {
            const size_t parent = (child - 1) >> 1;
            if (!(m_items[parent]->distance < m_items[child]->distance))
                exchange(parent, child);
            child = parent;
        }
    }
    void down(size_t parent)
    {
        for (size_t child = parent * 2 + 1; child < m_items.size(); child = parent * 2 + 1) {
            if (child + 1 < m_items.size() && !(m_items[child]->distance < m_items[child + 1]->distance))
                ++child;
            if (m_items[parent]->distance < m_items[child]->distance)
                return;
            exchange(parent, child);
            parent = child;
        }
    }
    std::vector<EndPoint *> m_items;
};

struct Chain
{
    size_t count = 0;
    double cost = 0.;
    double flipped_cost = 0.;
    EndPoint *begin = nullptr;
    EndPoint *end = nullptr;
    size_t equivalent = 0;

    void flip(std::vector<EndPoint> &points)
    {
        EndPoint *point = begin;
        EndPoint *previous = nullptr;
        do {
            EndPoint *opposite = &points[size_t(point - points.data()) ^ 1];
            EndPoint *following = opposite->next;
            opposite->next = previous;
            if (previous)
                previous->next = opposite;
            previous = point;
            point = following;
        } while (point);
        previous->next = nullptr;
        std::swap(cost, flipped_cost);
        EndPoint *new_begin = &points[size_t(begin - points.data()) ^ 1];
        EndPoint *new_end = &points[size_t(end - points.data()) ^ 1];
        std::swap(begin->chain, new_begin->chain);
        std::swap(end->chain, new_end->chain);
        begin = new_begin;
        end = new_end;
    }
};

Polylines greedy_chain(Polylines lines, const plugin_run_context *run_ctx)
{
    if (lines.size() < 2)
        return lines;
    std::vector<EndPoint> points;
    points.reserve(lines.size() * 2);
    for (const Polyline &line : lines) {
        points.push_back({{double(line.front().x()), double(line.front().y())}});
        points.push_back({{double(line.back().x()), double(line.back().y())}});
    }
    const auto coordinate = [&points](size_t index, size_t dimension) { return points[index].position[dimension]; };
    KDTreeIndirect<2, double, decltype(coordinate)> tree(coordinate, points.size());
    std::vector<Chain> chains;
    chains.reserve(lines.size());
    chains.emplace_back();
    const auto equivalent = [&chains](size_t id) {
        size_t last = id;
        while (last != chains[last].equivalent)
            last = chains[last].equivalent;
        chains[id].equivalent = last;
        return last;
    };
    Queue queue(points.size() * 2);
    for (size_t index = 0; index < points.size(); ++index) {
        EndPoint &point = points[index];
        const size_t nearest = find_closest_point(tree, point.position, [index](size_t other) { return (index ^ other) > 1; });
        point.candidate = &points[nearest];
        point.distance = distance(point, points[nearest]);
        queue.push(&point);
    }
    const auto update = [&](EndPoint &point) {
        const size_t index = size_t(&point - points.data());
        point.candidate = nullptr;
        const size_t chain = equivalent(std::max(point.chain, points[index ^ 1].chain));
        const size_t nearest = find_closest_point(tree, point.position, [&](size_t other) {
            if ((index ^ other) <= 1 || (points[other].chain > 0 && points[other ^ 1].chain > 0))
                return false;
            const size_t other_chain = equivalent(std::max(points[other].chain, points[other ^ 1].chain));
            return chain != other_chain || chain == 0;
        });
        assert(nearest < points.size());
        point.candidate = &points[nearest];
        point.distance = distance(point, points[nearest]);
        if (point.chain > 0)
            point.distance += chains[chain].flipped_cost - chains[chain].cost;
        if (points[nearest].chain > 0) {
            const Chain &other = chains[equivalent(points[nearest].chain)];
            point.distance += other.flipped_cost - other.cost;
        }
        queue.update(&point);
    };
    EndPoint *first = nullptr;
    size_t remaining = lines.size() - 1;
    for (size_t iterations = lines.size() * 16; iterations > 0; --iterations) {
        throw_if_cancelled(run_ctx);
        EndPoint *a = queue.top();
        EndPoint *opposite_a = &points[size_t(a - points.data()) ^ 1];
        EndPoint *b = a->candidate;
        EndPoint *opposite_b = &points[size_t(b - points.data()) ^ 1];
        const bool flip_a = a->chain > 0;
        const bool flip_b = b->chain > 0;
        const size_t id_a = equivalent((flip_a ? a : opposite_a)->chain);
        const size_t id_b = equivalent((flip_b ? b : opposite_b)->chain);
        double cost = distance(*a, *b);
        if (flip_a)
            cost += chains[id_a].flipped_cost - chains[id_a].cost;
        if (flip_b)
            cost += chains[id_b].flipped_cost - chains[id_b].cost;
        if ((b->chain > 0 && opposite_b->chain > 0) || (id_a == id_b && id_a != 0) ||
            std::abs(cost - a->distance) > SCALED_EPSILON) {
            update(*a);
            continue;
        }
        queue.pop();
        queue.remove(b->heap_index);
        a->candidate = nullptr;
        Chain *chain_a = id_a == 0 ? nullptr : &chains[id_a];
        Chain *chain_b = id_b == 0 ? nullptr : &chains[id_b];
        if (flip_a)
            chain_a->flip(points);
        if (flip_b)
            chain_b->flip(points);
        size_t id;
        if (id_a == 0 && id_b == 0) {
            id = chains.size();
            chains.emplace_back();
        } else if (id_a == 0 || id_b == 0)
            id = std::max(id_a, id_b);
        else {
            id = std::min(id_a, id_b);
            chains[id_a].equivalent = id;
            chains[id_b].equivalent = id;
        }
        Chain combined;
        combined.begin = chain_a == nullptr ? opposite_a : chain_a->begin == a ? chain_a->end : chain_a->begin;
        combined.end = chain_b == nullptr ? opposite_b : chain_b->begin == b ? chain_b->end : chain_b->begin;
        combined.cost = (chain_a ? chain_a->cost : 0.) + (chain_b ? chain_b->cost : 0.) + distance(*a, *b);
        combined.flipped_cost = (chain_a ? chain_a->flipped_cost : 0.) + (chain_b ? chain_b->flipped_cost : 0.) + distance(*opposite_a, *opposite_b);
        combined.count = (chain_a ? chain_a->count : 1) + (chain_b ? chain_b->count : 1);
        combined.equivalent = id;
        chains[id] = combined;
        if (combined.begin != opposite_a && opposite_a->heap_index != invalid)
            queue.remove(opposite_a->heap_index);
        if (combined.end != opposite_b && opposite_b->heap_index != invalid)
            queue.remove(opposite_b->heap_index);
        a->next = b;
        b->next = a;
        a->chain = b->chain = opposite_a->chain = opposite_b->chain = id;
        combined.begin->chain = combined.end->chain = 0;
        if (--remaining == 0) {
            do {
                first = queue.top();
                queue.pop();
            } while (first->next != nullptr);
            break;
        }
        update(points[size_t(combined.begin - points.data()) ^ 1]);
        update(points[size_t(combined.end - points.data()) ^ 1]);
        if (flip_a)
            update(*combined.begin);
        if (flip_b)
            update(*combined.end);
    }
    if (first == nullptr)
        throw std::runtime_error("Adaptive Cubic polyline chaining did not converge.");
    Polylines output;
    output.reserve(lines.size());
    do {
        const size_t index = size_t(first - points.data());
        output.emplace_back(std::move(lines[index >> 1]));
        if ((index & 1) != 0)
            output.back().reverse();
        first = points[index ^ 1].next;
    } while (first != nullptr);
    assert(output.size() == lines.size());
    return output;
}

Cost operator-(const Cost &a, const Cost &b) { return {a.normal - b.normal, a.flipped - b.flipped}; }

std::pair<double, size_t> minimum_crossover(const std::vector<Edge> &edges, const std::array<Span, 3> &spans,
                                           const std::array<Cost, 3> &costs, double current)
{
    const auto evaluate = [&](const std::array<int, 3> &order, size_t bits) {
        std::array<double, 3> internal_costs;
        std::array<Vec2d, 3> beginnings, endings;
        for (int index = 0; index < 3; ++index) {
            const Span span = spans[size_t(order[size_t(index)])];
            const Cost &c = costs[size_t(order[size_t(index)])];
            const bool reversed = (bits & (size_t(1) << (2 * index))) != 0;
            const bool flipped = (bits & (size_t(2) << (2 * index))) != 0;
            if (span.second == span.first + 1 && reversed)
                return std::numeric_limits<double>::max();
            internal_costs[size_t(index)] = flipped ? c.flipped : c.normal;
            const Vec2d begin = reversed ? (flipped ? edges[span.second - 1].first : edges[span.second - 1].last) :
                                          (flipped ? edges[span.first].last : edges[span.first].first);
            const Vec2d end = reversed ? (flipped ? edges[span.first].last : edges[span.first].first) :
                                        (flipped ? edges[span.second - 1].first : edges[span.second - 1].last);
            beginnings[size_t(index)] = begin;
            endings[size_t(index)] = end;
        }
        return internal_costs[0] + internal_costs[1] + internal_costs[2] +
               (beginnings[1] - endings[0]).norm() + (beginnings[2] - endings[1]).norm();
    };
    double best = current;
    size_t best_flip = 0;
    const std::array<std::array<int, 3>, 3> orders{{{{0, 1, 2}}, {{0, 2, 1}}, {{1, 0, 2}}}};
    for (size_t bits = 0; bits < 64; ++bits)
        for (size_t order = 0; order < orders.size(); ++order) {
            const double cost = bits == 0 && order == 0 ? current : evaluate(orders[order], bits);
            if (cost < best) {
                best = cost;
                best_flip = bits + (order << 6);
            }
        }
    return {best, best_flip};
}

void improve(std::vector<Edge> &edges, const plugin_run_context *run_ctx)
{
    std::vector<Cost> costs(edges.size());
    std::vector<Edge> temporary(edges);
    std::vector<std::pair<double, size_t>> lengths(edges.size() - 1);
    std::vector<bool> tried(edges.size());
    for (size_t iteration = 0; iteration < std::min(edges.size(), size_t(100)); ++iteration) {
        throw_if_cancelled(run_ctx);
        for (size_t i = 1; i < edges.size(); ++i) {
            costs[i] = costs[i - 1];
            const double length = (edges[i].first - edges[i - 1].last).norm();
            costs[i].normal += length;
            costs[i].flipped += (edges[i].last - edges[i - 1].first).norm();
            lengths[i - 1] = {length, i};
        }
        std::sort(lengths.begin(), lengths.end(), [](const std::pair<double, size_t> &a, const std::pair<double, size_t> &b) { return a.first > b.first; });
        std::fill(tried.begin(), tried.end(), false);
        size_t first = invalid, second = invalid, flip = 0;
        for (const std::pair<double, size_t> &candidate : lengths) {
            const size_t index = candidate.second;
            tried[index] = true;
            double minimum = costs.back().normal;
            size_t best_index = invalid, best_flip = 0;
            for (size_t j = 1; j < edges.size(); ++j) {
                if (tried[j])
                    continue;
                const size_t a = std::min(index, j), b = std::max(index, j);
                const std::pair<double, size_t> choice = minimum_crossover(edges,
                    {{{0, a}, {a, b}, {b, edges.size()}}},
                    {{costs[a - 1], costs[b - 1] - costs[a], costs.back() - costs[b]}}, costs.back().normal);
                if (choice.second > 0 && choice.first < minimum) {
                    minimum = choice.first;
                    best_index = j;
                    best_flip = choice.second;
                }
            }
            if (minimum < costs.back().normal) {
                first = std::min(index, best_index);
                second = std::max(index, best_index);
                flip = best_flip;
                break;
            }
        }
        if (flip == 0)
            break;
        std::array<Span, 3> spans{{{0, first}, {first, second}, {second, edges.size()}}};
        if ((flip >> 6) == 1)
            std::swap(spans[1], spans[2]);
        else if ((flip >> 6) == 2)
            std::swap(spans[0], spans[1]);
        size_t output = 0;
        for (size_t i = 0; i < spans.size(); ++i) {
            const bool reversed = (flip & (size_t(1) << (2 * i))) != 0;
            const bool flipped = (flip & (size_t(2) << (2 * i))) != 0;
            const Span span = spans[i];
            for (size_t j = span.first; j < span.second; ++j) {
                Edge edge = edges[reversed ? span.second - 1 - (j - span.first) : j];
                if (reversed != flipped)
                    edge.flip();
                temporary[output++] = edge;
            }
        }
        edges.swap(temporary);
    }
}

} // namespace

StoredPolylineCollection store_polylines(storage_handle *storage, const work::Polylines &lines,
                                         const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    StoredPolylineCollection result(storage);
    for (const work::Polyline &line : lines) {
        throw_if_cancelled(run_ctx);
        StoredPolyline path(storage);
        for (size_t index = 0; index < line.size(); ++index) {
            if ((index & 1023u) == 0)
                throw_if_cancelled(run_ctx);
            path.push_back(c_point{line[index].x(), line[index].y()});
        }
        result.push_back_move(std::move(path));
    }
    return result;
}

work::Polylines read_polylines(const PolylineCollection &lines, const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    work::Polylines result;
    result.reserve(lines.size());
    for (const slic3r_api::Polyline line : lines) {
        throw_if_cancelled(run_ctx);
        const multipoint_const_view points = line.view();
        work::Polyline &path = result.emplace_back();
        path.points.reserve(points.size);
        for (uint32_t index = 0; index < points.size; ++index) {
            if ((index & 1023u) == 0)
                throw_if_cancelled(run_ctx);
            path.points.emplace_back(points.array[index]);
        }
    }
    return result;
}

work::Polylines chain_polylines(work::Polylines lines, const plugin_run_context *run_ctx)
{
    lines = greedy_chain(std::move(lines), run_ctx);
    if (lines.size() < 2)
        return lines;
    std::vector<Edge> edges;
    edges.reserve(lines.size());
    for (size_t i = 0; i < lines.size(); ++i)
        edges.push_back({to_vec2d(lines[i].front()), to_vec2d(lines[i].back()), i});
    improve(edges, run_ctx);
    Polylines output;
    output.reserve(lines.size());
    for (const Edge &edge : edges) {
        output.emplace_back(std::move(lines[edge.source]));
        if (edge.last == to_vec2d(output.back().front()))
            output.back().reverse();
    }
    return output;
}

}}}
