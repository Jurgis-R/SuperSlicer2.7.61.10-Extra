///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ Copyright (c) 2021 Ultimaker B.V.
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

// Keep branch propagation synchronized with Fill/Lightning/TreeNode.cpp; clones isolate pruning from published trees.
#include "LightningTreeNode.hpp"
#include "libslic3r/Api/plugin/c/slic3r_utils.h"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"

namespace slic3r_api { namespace Infill { namespace Lightning {

NodePtr Node::add_child(const work::Point &point)
{
    assert(point != location);
    NodePtr child = std::make_shared<Node>(point);
    attach(child);
    return child;
}

void Node::attach(const NodePtr &child)
{
    children.push_back(child);
    child->parent = shared_from_this();
    child->root = false;
}

bool Node::has_offspring(const NodePtr &node) const
{
    if (node.get() == this)
        return true;
    for (const NodePtr &child : children)
        if (child->has_offspring(node))
            return true;
    return false;
}

void Node::visit(const std::function<void(const NodePtr &)> &visitor)
{
    visitor(shared_from_this());
    for (const NodePtr &child : children)
        child->visit(visitor);
}

coord_t Node::weighted_distance(const work::Point &point, coord_t radius) const
{
    const size_t valence = size_t(!root) + children.size();
    return coord_t(location.distance_to(point)) - (valence > 0 && valence < 4 ? 4 * radius : 0);
}

NodePtr Node::closest(const work::Point &point)
{
    NodePtr result = shared_from_this();
    coord_t distance = coord_t(location.distance_to(point));
    for (const NodePtr &child : children) {
        const NodePtr candidate = child->closest(point);
        const coord_t next = coord_t(candidate->location.distance_to(point));
        if (next < distance) {
            distance = next;
            result = candidate;
        }
    }
    return result;
}

void Node::reroot(const NodePtr &new_parent)
{
    if (!root) {
        const NodePtr old_parent = parent.lock();
        old_parent->reroot(shared_from_this());
        children.push_back(old_parent);
    }
    if (new_parent) {
        children.erase(std::remove(children.begin(), children.end(), new_parent), children.end());
        root = false;
        parent = new_parent;
    } else {
        root = true;
        parent.reset();
    }
}

work::BoundingBox Node::bounds() const
{
    work::BoundingBox box;
    box.merge(location);
    for (const NodePtr &child : children) {
        const work::BoundingBox child_bounds = child->bounds();
        box.merge(child_bounds.min);
        box.merge(child_bounds.max);
    }
    return box;
}

NodePtr Node::clone(const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    NodePtr copy = std::make_shared<Node>(location);
    copy->root = root;
    if (root)
        copy->grounding = grounding.value_or(location);
    for (const NodePtr &child : children) {
        NodePtr cloned = child->clone(run_ctx);
        cloned->parent = copy;
        copy->children.push_back(std::move(cloned));
    }
    return copy;
}

coord_t Node::prune(coord_t distance, const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    if (distance <= 0)
        return 0;
    coord_t maximum = 0;
    for (size_t index = 0; index < children.size();) {
        NodePtr &child = children[index];
        const coord_t pruned = child->prune(distance, run_ctx);
        if (pruned >= distance) {
            maximum = std::max(maximum, pruned);
            ++index;
        } else {
            const work::Point vector = location - child->location;
            const coord_t length = coord_t(work::to_vec2d(vector).norm());
            if (pruned + length <= distance) {
                maximum = std::max(maximum, pruned + length);
                children.erase(children.begin() + ptrdiff_t(index));
            } else {
                child->location += work::to_point(double(distance - pruned) * work::to_vec2d(vector).normalized());
                maximum = std::max(maximum, distance);
                ++index;
            }
        }
    }
    return maximum;
}

Node::Junction Node::straighten(coord_t magnitude, const work::Point &above, coord_t distance,
                                int64_t maximum_squared_distance, const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    if (children.size() == 1) {
        NodePtr child = children.front();
        const Junction below = child->straighten(magnitude, above, distance + coord_t(location.distance_to(child->location)),
                                                  maximum_squared_distance, run_ctx);
        if (above != below.point) {
            const work::Point vector = below.point - above;
            const int64_t divisor = std::max(int64_t(1), int64_t(below.distance));
            const work::Point destination(above.x() + vector.x() * int64_t(distance) / divisor,
                                          above.y() + vector.y() * int64_t(distance) / divisor);
            if (work::to_vec2d(destination - location).squaredNorm() <= double(magnitude) * magnitude)
                location = destination;
            else
                location += work::to_point(double(magnitude) * work::to_vec2d(destination - location).normalized());
        }
        child = children.front();
        const NodePtr ancestor = parent.lock();
        if (ancestor && work::to_vec2d(child->location - ancestor->location).squaredNorm() < maximum_squared_distance &&
            distance_to_segment(ancestor->location, child->location, location) < 100.) {
            child->parent = parent;
            for (NodePtr &sibling : ancestor->children)
                if (sibling.get() == this) {
                    sibling = child;
                    break;
                }
        }
        return below;
    }
    work::Point movement = work::to_point(1000. * work::to_vec2d(above - location).normalized());
    bool prevent = false;
    for (const NodePtr &child : children) {
        const Junction below = child->straighten(magnitude, location, coord_t(location.distance_to(child->location)),
                                                  maximum_squared_distance, run_ctx);
        movement += work::to_point(1000. * work::to_vec2d(below.point - location).normalized());
        prevent = prevent || below.distance < magnitude;
    }
    if (movement != work::Point(0, 0) && !children.empty() && !root && !prevent) {
        const coord_t limit = magnitude * 3 / 4;
        const coord_t length = coord_t(movement.norm());
        if (length > limit)
            movement = movement * (double(limit) / double(length));
        location += movement;
    }
    return {distance, location};
}

bool Node::realign(const work::Polygons &outlines, const grid::Grid &locator,
                   std::vector<NodePtr> &parts, const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    if (outlines.empty())
        return false;
    if (inside(outlines, location)) {
        bool reground = false;
        children.erase(std::remove_if(children.begin(), children.end(), [&](const NodePtr &child) {
            bool connected = child->realign(outlines, locator, parts, run_ctx);
            work::Point hit;
            if (connected && boundary_intersection(child->location, location, locator, hit, locator.resolution() * 2)) {
                child->grounding.reset();
                child->parent.reset();
                child->root = true;
                parts.push_back(child);
                reground = true;
                connected = false;
            }
            return !connected;
        }), children.end());
        if (reground)
            grounding.reset();
        return true;
    }
    for (const NodePtr &child : children)
        if (child->realign(outlines, locator, parts, run_ctx)) {
            child->grounding = location;
            child->parent.reset();
            child->root = true;
            parts.push_back(child);
        }
    children.clear();
    return false;
}

void Node::propagate(std::vector<NodePtr> &next, const work::Polygons &outlines, const grid::Grid &locator,
                     coord_t prune_length, coord_t straightening_distance, const plugin_run_context *run_ctx) const
{
    NodePtr copy = clone(run_ctx);
    copy->prune(prune_length, run_ctx);
    copy->straighten(straightening_distance, copy->location, 0,
        int64_t(locator_cell_size / 2) * (locator_cell_size / 2), run_ctx);
    if (copy->realign(outlines, locator, next, run_ctx))
        next.push_back(std::move(copy));
}

void Node::convert(size_t path, work::Polylines &output, const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    if (children.empty()) {
        output[path].points.push_back(location);
        return;
    }
    const size_t first = size_t(slic3r_safe_rand(int32_t(children.size() - 1)));
    children[first]->convert(path, output, run_ctx);
    output[path].points.push_back(location);
    for (size_t offset = 1; offset < children.size(); ++offset) {
        output.emplace_back();
        const size_t next = output.size() - 1;
        children[(first + offset) % children.size()]->convert(next, output, run_ctx);
        output[next].points.push_back(location);
    }
}

work::Polylines Node::paths(coord_t overlap, const plugin_run_context *run_ctx) const
{
    work::Polylines output(1);
    convert(0, output, run_ctx);
    for (size_t index = 0; index < output.size();) {
        work::Polyline &path = output[index];
        if (path.size() > 1) {
            coord_t remaining = overlap;
            work::Point a = path.back();
            for (ptrdiff_t point = ptrdiff_t(path.size()) - 2; point >= 0; --point) {
                const work::Point b = path.points[size_t(point)];
                const work::Point vector = b - a;
                const coord_t length = coord_t(work::to_vec2d(vector).norm());
                // Realignment may attach coincident vertices; they cannot consume junction overlap.
                if (length == 0) {
                    path.points.pop_back();
                    a = b;
                    continue;
                }
                if (length >= remaining) {
                    path.points.back() = a + work::to_point((double(remaining) / length) * work::to_vec2d(vector));
                    break;
                }
                remaining -= length;
                path.points.pop_back();
                a = b;
            }
        }
        if (path.size() <= 1) {
            path = std::move(output.back());
            output.pop_back();
        } else
            ++index;
    }
    return output;
}

}}}
