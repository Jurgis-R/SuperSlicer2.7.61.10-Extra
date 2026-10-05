///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ Copyright (c) Prusa Research 2020 - 2021 Lukáš Matěna, Vojtěch Bubník
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

// Keep T-joint and hook decisions synchronized with connect_lines_using_hooks() in libslic3r/Fill/FillAdaptive.cpp.

#include "AdaptivePaths.hpp"

#include <numeric>
#include <optional>
#include <type_traits>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"

namespace slic3r_api { namespace Infill { namespace AdaptiveCubic {
namespace {

using work::Point;
using work::Points;
using work::Polyline;
using work::Polylines;
using work::Vec2d;
using work::to_point;
using work::to_vec2d;

double cross2(const Vec2d &a, const Vec2d &b) { return a.x() * b.y() - a.y() * b.x(); }

struct Line
{
    Point a, b;
    Vec2d vector() const { return to_vec2d(b - a); }
    void translate(const Point &v) { a += v; b += v; }
    bool intersection(const Line &other, Point &point) const
    {
        const Vec2d v1 = vector();
        const Vec2d v2 = other.vector();
        const double denominator = cross2(v1, v2);
        if (std::abs(denominator) < EPSILON)
            return false;
        const Vec2d difference = to_vec2d(a - other.a);
        const double t1 = cross2(v2, difference) / denominator;
        const double t2 = cross2(v1, difference) / denominator;
        if (t1 < 0. || t1 > 1. || t2 < 0. || t2 > 1.)
            return false;
        const Vec2d result = to_vec2d(a) + t1 * v1;
        if constexpr (std::is_same<coord_t, int64_t>::value)
            point = Point(coord_t(std::round(result.x())), coord_t(std::round(result.y())));
        else
            point = to_point(result);
        return true;
    }
};

struct Intersection
{
    const Line *closest_line;
    const Line *intersect_line;
    Polyline *polyline;
    Point point;
    bool front;
    bool left;
    bool used = false;

    Intersection(const Line &closest, const Line &source, Polyline &path, Point p, bool first)
        : closest_line(&closest), intersect_line(&source), polyline(&path), point(p), front(first)
    {
        left = cross2(closest.vector(), p == source.a ? source.vector() : -1. * source.vector()) > 0.;
    }
    bool fresh() const { return !used && !polyline->empty(); }
    std::optional<Line> other_hook() const
    {
        const Points &points = polyline->points;
        if (points.size() < 3)
            return {};
        return front ? Line{points[1], points[2]} : Line{points[points.size() - 2], points[points.size() - 3]};
    }
    bool other_intersection(const Line &line, Point &p) const
    {
        const std::optional<Line> other = other_hook();
        return other && other->intersection(line, p);
    }
};

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
// With coord_t, bgi::intersects can throw "bad numeric conversion: positive overflow"; keep float coordinates.
using TreePoint = bg::model::point<float, 2, bg::cs::cartesian>;
using TreeSegment = bg::model::segment<TreePoint>;
using TreeItem = std::pair<TreeSegment, size_t>;
using Tree = bgi::rtree<TreeItem, bgi::rstar<16, 4>>;

TreePoint tree_point(const Point &p) { return TreePoint(float(p.x()), float(p.y())); }
TreeSegment tree_segment(const Line &line) { return {tree_point(line.a), tree_point(line.b)}; }

Line offset_line(Line line, const Intersection &intersection, double offset)
{
    line.translate(to_point(work::perp(intersection.closest_line->vector().normalized()) *
                            (intersection.left ? offset : -offset)));
    const Point extension = to_point(double(coord_t(offset * 1.16)) * line.vector().normalized());
    line.a -= extension;
    line.b += extension;
    return line;
}

void add_hook(const Intersection &intersection, double offset, double hook_length, double trim,
              const Tree &tree, const std::vector<Line> &source)
{
    if (hook_length < SCALED_EPSILON)
        return;
    Point start;
    const bool found = intersection.intersect_line->intersection(
        offset_line(*intersection.closest_line, intersection, offset), start);
    assert(found);
    const std::optional<Line> other = intersection.other_hook();
    const Vec2d direction = intersection.closest_line->vector().normalized();
    const Point extension = to_point((hook_length + 1.16 * trim) * direction);
    const auto filter = [&intersection, &source](const TreeItem &item) {
        return item.second != size_t(intersection.intersect_line - source.data());
    };
    const auto maximum_length = [&](const Vec2d &dir, const std::vector<TreeItem> &hits,
                                     bool self_intersection, const Point &self_point) {
        double length = hook_length;
        for (const TreeItem &hit : hits) {
            const Vec2d origin(bg::get<0, 0>(hit.first), bg::get<0, 1>(hit.first));
            const Vec2d vector = Vec2d(bg::get<1, 0>(hit.first), bg::get<1, 1>(hit.first)) - origin;
            const double denominator = cross2(dir, vector);
            assert(std::abs(denominator) > EPSILON);
            double t = cross2(origin - to_vec2d(start), vector) / denominator;
            if (hit.second < source.size())
                t -= trim * std::abs(cross2(dir, vector.normalized()));
            length = std::min(length, t);
        }
        if (self_intersection)
            length = std::min(length, (to_vec2d(self_point) - to_vec2d(start)).dot(dir) -
                                     trim * std::abs(cross2(dir, other->vector().normalized())));
        return std::max(0., length);
    };
    std::vector<TreeItem> hits;
    Line forward{start, start + extension};
    tree.query(bgi::intersects(tree_segment(forward)) && bgi::satisfies(filter), std::back_inserter(hits));
    Point self_point;
    bool self = other && other->intersection(forward, self_point);
    const double forward_length = maximum_length(direction, hits, self, self_point);
    double backward_length = 0.;
    if (forward_length < hook_length - SCALED_EPSILON) {
        hits.clear();
        Line backward{start, start - extension};
        tree.query(bgi::intersects(tree_segment(backward)) && bgi::satisfies(filter), std::back_inserter(hits));
        self = other && other->intersection(backward, self_point);
        backward_length = maximum_length(-1. * direction, hits, self, self_point);
    }
    const Point end = start + to_point((forward_length > backward_length ? forward_length : -backward_length) * direction);
    Points &points = intersection.polyline->points;
    if (intersection.front) {
        points.front() = start;
        points.emplace(points.begin(), end);
    } else {
        points.back() = start;
        points.emplace_back(end);
    }
}

} // namespace

work::Polylines connect_hooks(work::Polylines lines, const ExPolygon &boundary, double spacing,
                              double hook_length, double hook_length_max, const plugin_run_context *run_ctx)
{
    Tree tree;
    size_t index = 0;
    const float offset = float(scale_d(spacing) * 0.81);
    const float trim = float(scale_d(spacing) * 0.5 * 0.75);
    std::vector<TreeItem> closest;
    std::vector<std::pair<const Polyline *, const Polyline *>> touching;
    for (Polyline &polyline : lines) {
        throw_if_cancelled(run_ctx);
        assert(polyline.size() == 2);
        if (&polyline != lines.data()) {
            const auto collinear = [&](const Point &p, const Point &opposite) -> std::pair<Polyline *, bool> {
                closest.clear();
                tree.query(bgi::nearest(tree_point(p), 1), std::back_inserter(closest));
                if (closest.empty())
                    return {nullptr, false};
                Polyline *other = &lines[closest.front().second];
                const double front = to_vec2d(other->front() - p).squaredNorm();
                const double back = to_vec2d(other->back() - p).squaredNorm();
                const double nearest = std::min(front, back);
                if (nearest < 1200. * 1200. &&
                    std::min(to_vec2d(other->front() - opposite).squaredNorm(),
                             to_vec2d(other->back() - opposite).squaredNorm()) > nearest) {
                    const double parallel = to_vec2d(opposite - p).normalized().dot(to_vec2d(other->back() - other->front()).normalized());
                    if (std::abs(parallel) > 0.99f) {
                        tree.remove(closest.front());
                        return {other, nearest == front};
                    }
                    touching.emplace_back(std::min(&polyline, other), std::max(&polyline, other));
                }
                return {nullptr, false};
            };
            const std::pair<Polyline *, bool> front = collinear(polyline.front(), polyline.back());
            const std::pair<Polyline *, bool> back = collinear(polyline.back(), polyline.front());
            assert(!front.first || !back.first || front.first != back.first);
            if (front.first) {
                polyline.points.front() = front.second ? front.first->back() : front.first->front();
                front.first->points.clear();
            }
            if (back.first) {
                polyline.points.back() = back.second ? back.first->back() : back.first->front();
                back.first->points.clear();
            }
        }
        tree.insert({tree_segment({polyline.front(), polyline.back()}), index++});
    }

    std::vector<Line> source;
    source.reserve(lines.size());
    for (const Polyline &line : lines)
        source.push_back(line.empty() ? Line{Point(0, 0), Point(0, 0)} : Line{line.front(), line.back()});
    std::sort(touching.begin(), touching.end());
    touching.erase(std::unique(touching.begin(), touching.end()), touching.end());
    std::vector<Intersection> intersections;
    const double drop_both = offset * (2. / std::cos(PI / 6.) + 0.5) + SCALED_EPSILON;
    const double drop_single = offset * (1. / std::cos(PI / 6.) + 1.5) + SCALED_EPSILON;
    for (size_t line_index = 0; line_index < lines.size(); ++line_index) {
        throw_if_cancelled(run_ctx);
        Polyline &line = lines[line_index];
        if (line.empty())
            continue;
        const auto tjoint = [&](const Point &point) -> std::optional<size_t> {
            const auto eligible = [&](const TreeItem &item) {
                if (item.second == line_index)
                    return false;
                const Line &candidate = source[item.second];
                const Vec2d v = candidate.vector();
                const double t = to_vec2d(point - candidate.a).dot(v);
                return v.squaredNorm() > 0. && t > SCALED_EPSILON && t < v.squaredNorm() - SCALED_EPSILON;
            };
            closest.clear();
            tree.query(bgi::nearest(tree_point(point), 1) && bgi::satisfies(eligible), std::back_inserter(closest));
            if (closest.empty())
                return {};
            const size_t other_index = closest.front().second;
            const Polyline &other = lines[other_index];
            if (other.size() < 2 || work::line_alg::distance_to_squared(
                work::Linef(to_vec2d(other.front()), to_vec2d(other.back())), to_vec2d(point)) > 1000. * 1000.)
                return {};
            const std::pair<const Polyline *, const Polyline *> pair{std::min(&line, &lines[other_index]),
                                                                     std::max(&line, &lines[other_index])};
            return std::binary_search(touching.begin(), touching.end(), pair) ? std::nullopt : std::optional<size_t>(other_index);
        };
        const std::optional<size_t> front = tjoint(line.front());
        const std::optional<size_t> back = tjoint(line.back());
        const int joints = int(front.has_value()) + int(back.has_value());
        if (joints == 0)
            continue;
        const double threshold = joints == 1 ? drop_single : drop_both;
        if (line.length() < threshold) {
            line.points.clear();
        } else if (line.length() > threshold + offset) {
            if (front)
                intersections.emplace_back(source[*front], source[line_index], line, line.front(), true);
            if (back)
                intersections.emplace_back(source[*back], source[line_index], line, line.back(), false);
        } else {
            if (front)
                line.points.front() += to_point((trim * 1.155) * to_vec2d(line.back() - line.front()).normalized());
            if (back)
                line.points.back() += to_point((trim * 1.155) * to_vec2d(line.front() - line.back()).normalized());
        }
    }
    for (size_t i = 0; i < intersections.size();) {
        if (lines[size_t(intersections[i].closest_line - source.data())].empty()) {
            intersections[i] = intersections.back();
            intersections.pop_back();
        } else
            ++i;
    }
    std::sort(intersections.begin(), intersections.end(), [](const Intersection &a, const Intersection &b) {
        return a.closest_line == b.closest_line ? int(a.left) < int(b.left) : a.closest_line < b.closest_line;
    });
    std::vector<size_t> merged(lines.size());
    std::iota(merged.begin(), merged.end(), 0);
    const auto insert_boundary = [&](const Polygon &polygon) {
        Point previous(polygon.back());
        const multipoint_const_view points = polygon.view();
        for (uint32_t i = 0; i < points.size; ++i) {
            const Point point(points.array[i]);
            tree.insert({tree_segment({previous, point}), index++});
            previous = point;
        }
    };
    insert_boundary(boundary.contour());
    for (const Polygon hole : boundary.holes())
        insert_boundary(hole);
    const auto representative = [&](size_t i) {
        size_t last = i;
        while (merged[last] != last)
            last = merged[last];
        merged[i] = last;
        return last;
    };
    const auto update = [&](Intersection &intersection) {
        intersection.polyline = &lines[representative(size_t(intersection.polyline - lines.data()))];
        if (intersection.fresh())
            intersection.front = intersection.polyline->front() == intersection.point;
    };
    for (std::vector<std::pair<const Polyline *, const Polyline *>>::reverse_iterator it = touching.rbegin(); it != touching.rend(); ++it) {
        Polyline &first = lines[size_t(it->first - lines.data())];
        Polyline &second = lines[representative(size_t(it->second - lines.data()))];
        if (&first == &second || first.empty() || second.empty())
            continue;
        const double d11 = to_vec2d(first.front() - second.front()).squaredNorm();
        const double d12 = to_vec2d(first.front() - second.back()).squaredNorm();
        const double d21 = to_vec2d(first.back() - second.front()).squaredNorm();
        const double d22 = to_vec2d(first.back() - second.back()).squaredNorm();
        if (std::min(d11, d12) < std::min(d21, d22)) {
            first.reverse();
            if (d12 == std::min(d11, d12))
                second.reverse();
        } else if (d22 == std::min(d21, d22))
            second.reverse();
        first.points.back() = Point((first.back().x() + second.front().x()) / 2,
                                    (first.back().y() + second.front().y()) / 2);
        first.points.insert(first.points.end(), second.points.begin() + 1, second.points.end());
        second.points.clear();
        merged[size_t(&second - lines.data())] = size_t(&first - lines.data());
    }

    std::vector<std::pair<Intersection *, double>> group;
    for (size_t begin = 0; begin < intersections.size();) {
        throw_if_cancelled(run_ctx);
        group.clear();
        const Intersection &initial = intersections[begin];
        const Vec2d direction = initial.closest_line->vector();
        size_t end = begin;
        for (; end < intersections.size() && intersections[end].closest_line == initial.closest_line &&
               intersections[end].left == initial.left; ++end)
            group.emplace_back(&intersections[end], direction.dot(to_vec2d(intersections[end].point)));
        begin = end;
        std::sort(group.begin(), group.end(), [](const std::pair<Intersection *, double> &a,
                                                const std::pair<Intersection *, double> &b) { return a.second < b.second; });
        for (size_t i = 0; i < group.size(); ++i) {
            Intersection &first = *group[i].first;
            update(first);
            if (!first.fresh())
                continue;
            if (group.size() == 1) {
                add_hook(first, offset, hook_length, trim, tree, source);
                first.used = true;
                continue;
            }
            if (i > 0)
                update(*group[i - 1].first);
            if (i + 1 < group.size())
                update(*group[i + 1].first);
            const bool next = i == 0 ? true : i + 1 == group.size() ? false :
                group[i - 1].first->fresh() && group[i + 1].first->fresh() ?
                    group[i + 1].second - group[i].second < group[i].second - group[i - 1].second :
                    group[i + 1].first->fresh();
            Intersection &nearest = *group[next ? i + 1 : i - 1].first;
            const Line offset_segment = offset_line({first.point, nearest.point}, first, offset);
            Point a, b;
            bool connect = nearest.fresh() && first.intersect_line->intersection(offset_segment, a) &&
                           nearest.intersect_line->intersection(offset_segment, b);
            connect = connect && to_vec2d(b - a).squaredNorm() <= hook_length_max * hook_length_max;
            if (connect) {
                closest.clear();
                tree.query(bgi::intersects(tree_segment({a, b})) && bgi::satisfies([&](const TreeItem &item) {
                    return item.second != size_t(first.intersect_line - source.data()) &&
                           item.second != size_t(nearest.intersect_line - source.data());
                }), std::back_inserter(closest));
                connect = closest.empty();
            }
            if (connect) {
                Points &first_points = first.polyline->points;
                Points &second_points = nearest.polyline->points;
                if (first.polyline == nearest.polyline) {
                    if (!first.front)
                        std::swap(a, b);
                    first_points.front() = a;
                    first_points.back() = b;
                    first_points.emplace(first_points.begin(), b);
                } else {
                    Line trimming{a, b};
                    trimming.translate(to_point(work::perp(first.closest_line->vector().normalized()) * (first.left ? trim : -trim)));
                    Point start, finish;
                    const bool trim_start = first_points.size() == 3 && first.other_intersection(trimming, start);
                    const bool trim_end = second_points.size() == 3 && nearest.other_intersection(trimming, finish);
                    first_points.reserve(first_points.size() + second_points.size());
                    if (first.front)
                        std::reverse(first_points.begin(), first_points.end());
                    if (trim_start)
                        first_points.front() = start;
                    first_points.back() = a;
                    first_points.push_back(b);
                    if (nearest.front)
                        first_points.insert(first_points.end(), second_points.begin() + 1, second_points.end());
                    else
                        first_points.insert(first_points.end(), second_points.rbegin() + 1, second_points.rend());
                    if (trim_end)
                        first_points.back() = finish;
                    if (first.polyline < nearest.polyline) {
                        second_points.clear();
                        merged[size_t(nearest.polyline - lines.data())] = size_t(first.polyline - lines.data());
                    } else {
                        second_points = std::move(first_points);
                        first_points.clear();
                        merged[size_t(first.polyline - lines.data())] = size_t(nearest.polyline - lines.data());
                    }
                }
                nearest.used = true;
            } else
                add_hook(first, offset, hook_length, trim, tree, source);
            first.used = true;
        }
    }
    Polylines output;
    for (Polyline &line : lines)
        if (!line.empty())
            output.emplace_back(std::move(line));
    return output;
}

}}}
