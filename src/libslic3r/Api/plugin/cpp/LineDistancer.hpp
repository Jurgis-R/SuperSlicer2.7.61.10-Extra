///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_plugin_cpp_LineDistancer_hpp_
#define slic3r_Api_plugin_cpp_LineDistancer_hpp_

/*
LineDistancer
=============

LineDistancer copies the boundary segments of an `ExPolygonCollection` and
answers nearest-distance queries against those segments. Both outer contours
and holes are included. A query is measured against the finite segments, not
against their infinite supporting lines, so the closest point may be a segment
endpoint.

The stored coordinates use the same scaled units as `c_point`. The returned
distance and the radius argument use those units as well; convert to or from
millimeters at the API boundary when necessary.

    LineDistancer boundaries(areas);
    double distance = boundaries.distance_from_lines(point);
    std::vector<size_t> nearby = boundaries.all_lines_in_radius(point, radius);

`distance_from_lines()` returns positive distance outside the stored areas. If
`signed_distance` is true, it returns a negative distance for a point inside an
outer contour and outside its holes. An empty distancer returns positive
infinity. `all_lines_in_radius()` returns the indices of matching stored
segments in insertion order and returns no indices for a negative radius.

An owned balanced AABB index prunes unrelated segments. Construction from areas
builds it automatically. append_segment() invalidates it; queries then use a
linear scan until build_index() is called explicitly. Const queries never mutate
state and may run concurrently after preparation, with no concurrent mutations.
Copies and moves contain only values and indices, never pointers into another
helper. Index construction costs O(E log E); overlapping boxes may still require
O(E) query work. Returned indices always follow segment insertion order.
The source polygons are copied during construction, so later changes to the
input views do not change an existing LineDistancer.
*/

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <numeric>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"

namespace slic3r_api {

class LineDistancer
{
public:
    LineDistancer() = default;

    explicit LineDistancer(const ExPolygonCollection &areas) {
        for (const ExPolygon area : areas) append(area);
        build_index();
    }

    bool empty() const { return m_segments.empty(); }

    // Added open segments contribute distances, but never define an interior.
    // The caller must rebuild before sharing indexed queries with other threads.
    void append_segment(c_point a, c_point b) {
        m_segments.push_back({a, b, invalid_index});
        m_index_valid = false;
    }

    // Own all tree data. Failure leaves the exhaustive query path available.
    void build_index() {
        m_index_valid = false;
        m_nodes.clear();
        m_order.resize(m_segments.size());
        std::iota(m_order.begin(), m_order.end(), size_t(0));
        if (!m_order.empty()) build_node(0, m_order.size());
        m_index_valid = true;
    }

    // Conservative broad phase: box overlap, not exact segment/box intersection.
    // Bounds are closed, in scaled coordinates. Inverted boxes return no indices.
    std::vector<size_t> all_lines_in_box(c_point min, c_point max) const {
        return query_box({double(min.x), double(min.y), double(max.x), double(max.y)});
    }

    double distance_from_lines(c_point point, bool signed_distance = false) const {
        if (m_segments.empty()) return std::numeric_limits<double>::infinity();
        double best = std::numeric_limits<double>::max();
        if (m_index_valid && !m_nodes.empty()) nearest(0, point, best);
        else for (const Segment &segment : m_segments)
            best = std::min(best, segment_distance_squared(point, segment));
        const double distance = std::sqrt(best);
        return signed_distance && contains(point) ? -distance : distance;
    }

    std::vector<size_t> all_lines_in_radius(c_point point, double radius) const {
        if (!(radius >= 0.)) return {};
        const Box box{double(point.x)-radius, double(point.y)-radius,
                      double(point.x)+radius, double(point.y)+radius};
        std::vector<size_t> out = query_box(box);
        const double squared = radius * radius;
        out.erase(std::remove_if(out.begin(), out.end(), [&](size_t idx) {
            return segment_distance_squared(point, m_segments[idx]) > squared;
        }), out.end());
        return out;
    }

private:
    static constexpr size_t invalid_index = size_t(-1);
    struct Segment { c_point a, b; size_t ring; };
    struct RingInfo { size_t area; bool hole; };
    struct Box { double min_x, min_y, max_x, max_y; };
    struct Node {
        Box box;
        size_t begin, end;
        size_t left = invalid_index, right = invalid_index;
    };

    // Floating bounds avoid overflow in midpoint and distance arithmetic.
    static Box bounds(const Segment &segment) {
        return {double(std::min(segment.a.x, segment.b.x)), double(std::min(segment.a.y, segment.b.y)),
                double(std::max(segment.a.x, segment.b.x)), double(std::max(segment.a.y, segment.b.y))};
    }
    static bool overlaps(const Box &a, const Box &b) {
        return a.min_x <= b.max_x && a.max_x >= b.min_x &&
               a.min_y <= b.max_y && a.max_y >= b.min_y;
    }
    static double box_distance(const Box &box, c_point p) {
        const double dx = std::max({box.min_x-double(p.x), 0., double(p.x)-box.max_x});
        const double dy = std::max({box.min_y-double(p.y), 0., double(p.y)-box.max_y});
        return dx*dx + dy*dy;
    }

    // Median partition bounds the depth even for identical or degenerate edges.
    size_t build_node(size_t begin, size_t end) {
        Box box = bounds(m_segments[m_order[begin]]);
        for (size_t i=begin+1; i<end; ++i) {
            const Box next = bounds(m_segments[m_order[i]]);
            box.min_x=std::min(box.min_x,next.min_x); box.min_y=std::min(box.min_y,next.min_y);
            box.max_x=std::max(box.max_x,next.max_x); box.max_y=std::max(box.max_y,next.max_y);
        }
        const size_t idx=m_nodes.size();
        m_nodes.push_back({box,begin,end});
        if (end-begin > 8) {
            const bool x=box.max_x-box.min_x >= box.max_y-box.min_y;
            const size_t middle=begin+(end-begin)/2;
            std::nth_element(m_order.begin()+begin,m_order.begin()+middle,m_order.begin()+end,
                [&](size_t lhs,size_t rhs) {
                    const Segment &a=m_segments[lhs], &b=m_segments[rhs];
                    const double ac=x?double(a.a.x)+a.b.x:double(a.a.y)+a.b.y;
                    const double bc=x?double(b.a.x)+b.b.x:double(b.a.y)+b.b.y;
                    return ac==bc?lhs<rhs:ac<bc;
                });
            const size_t left=build_node(begin,middle), right=build_node(middle,end);
            m_nodes[idx].left=left; m_nodes[idx].right=right;
        }
        return idx;
    }

    // Query-local traversal preserves concurrent const access and stable indices.
    void collect(size_t idx,const Box &box,std::vector<size_t> &out) const {
        const Node &node=m_nodes[idx];
        if (!overlaps(node.box,box)) return;
        if (node.left!=invalid_index) {
            collect(node.left,box,out); collect(node.right,box,out);
        } else for (size_t i=node.begin;i<node.end;++i)
            if (overlaps(bounds(m_segments[m_order[i]]),box)) out.push_back(m_order[i]);
    }
    std::vector<size_t> query_box(const Box &box) const {
        std::vector<size_t> out;
        if (!(box.min_x<=box.max_x && box.min_y<=box.max_y)) return out;
        if (m_index_valid && !m_nodes.empty()) {
            collect(0,box,out);
            std::sort(out.begin(),out.end());
        } else for (size_t i=0;i<m_segments.size();++i)
            if (overlaps(bounds(m_segments[i]),box)) out.push_back(i);
        return out;
    }
    void nearest(size_t idx,c_point p,double &best) const {
        const Node &node=m_nodes[idx];
        if (box_distance(node.box,p)>best) return;
        if (node.left==invalid_index) {
            for (size_t i=node.begin;i<node.end;++i)
                best=std::min(best,segment_distance_squared(p,m_segments[m_order[i]]));
        } else {
            const bool left_first=box_distance(m_nodes[node.left].box,p)<=box_distance(m_nodes[node.right].box,p);
            nearest(left_first?node.left:node.right,p,best);
            nearest(left_first?node.right:node.left,p,best);
        }
    }

    // Keep polygon identity separate from index order: holes subtract only from
    // their own area, and overlapping ExPolygons form a union, not an XOR.
    void append(const ExPolygon &area) {
        const size_t area_idx=m_area_count++;
        append_ring(area.contour().points(),area_idx,false);
        for (const Polygon hole:area.holes()) append_ring(hole.points(),area_idx,true);
    }
    void append_ring(const std::vector<c_point> &points,size_t area,bool hole) {
        if (points.size()<2) return;
        const size_t ring=m_rings.size();
        m_rings.push_back({area,hole});
        for (size_t i=1;i<points.size();++i) m_segments.push_back({points[i-1],points[i],ring});
        if (points.front().x!=points.back().x || points.front().y!=points.back().y)
            m_segments.push_back({points.back(),points.front(),ring});
    }
    static double segment_distance_squared(c_point point,const Segment &segment) {
        const double ax=double(segment.a.x), ay=double(segment.a.y);
        const double dx=double(segment.b.x)-ax, dy=double(segment.b.y)-ay;
        const double length=dx*dx+dy*dy;
        const double t=length>0?std::clamp(((double(point.x)-ax)*dx+(double(point.y)-ay)*dy)/length,0.,1.):0.;
        const double x=double(point.x)-ax-t*dx, y=double(point.y)-ay-t*dy;
        return x*x+y*y;
    }
    bool contains(c_point point) const {
        const std::vector<size_t> candidates=query_box({double(point.x),double(point.y),
            std::numeric_limits<double>::infinity(),double(point.y)});
        std::map<size_t,bool> parity;
        for (size_t idx:candidates) {
            const Segment &s=m_segments[idx];
            if (s.ring==invalid_index || (s.a.y>point.y)==(s.b.y>point.y)) continue;
            const double x=double(s.a.x)+(double(s.b.x)-s.a.x)*(double(point.y)-s.a.y)/(double(s.b.y)-s.a.y);
            if (double(point.x)<x) parity[s.ring]=!parity[s.ring];
        }
        std::map<size_t,std::pair<bool,bool>> areas;
        for (const std::pair<const size_t,bool> &entry:parity) {
            if (!entry.second) continue;
            const RingInfo &ring=m_rings[entry.first];
            if (ring.hole) areas[ring.area].second=true;
            else areas[ring.area].first=true;
        }
        for (const std::pair<const size_t,std::pair<bool,bool>> &area:areas)
            if (area.second.first && !area.second.second) return true;
        return false;
    }

    std::vector<Segment> m_segments;
    std::vector<RingInfo> m_rings;
    size_t m_area_count=0;
    std::vector<Node> m_nodes;
    std::vector<size_t> m_order;
    bool m_index_valid=false;
};

} // namespace slic3r_api

#endif // slic3r_Api_plugin_cpp_LineDistancer_hpp_
