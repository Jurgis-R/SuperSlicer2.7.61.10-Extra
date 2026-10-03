///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ Copyright (c) 2021 Ultimaker B.V.
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#ifndef plugins_lightning_tree_node_hpp_
#define plugins_lightning_tree_node_hpp_
#include <memory>
#include <optional>
#include <functional>
#include "LightningGeometry.hpp"

namespace slic3r_api { namespace Infill { namespace Lightning {
class Node;
using NodePtr = std::shared_ptr<Node>;

// Keep rerooting, pruning and branch conversion synchronized with Fill/Lightning/TreeNode.cpp.
class Node : public std::enable_shared_from_this<Node>
{
public:
    explicit Node(work::Point point, std::optional<work::Point> ground = {}) : location(point), grounding(ground) {}
    work::Point location;
    std::optional<work::Point> grounding;
    // Roots, locators and reconnection jobs share nodes; weak parents break ownership cycles.
    std::weak_ptr<Node> parent;
    std::vector<NodePtr> children;
    bool root = true;

    NodePtr add_child(const work::Point &point);
    void attach(const NodePtr &child);
    bool has_offspring(const NodePtr &node) const;
    void visit(const std::function<void(const NodePtr &)> &visitor);
    coord_t weighted_distance(const work::Point &point, coord_t radius) const;
    NodePtr closest(const work::Point &point);
    // Reversing ancestry must update weak parents and strong child links together.
    void reroot(const NodePtr &new_parent = {});
    work::BoundingBox bounds() const;
    // Clone before pruning so the upper layer remains immutable when shared by surface workers.
    void propagate(std::vector<NodePtr> &next, const work::Polygons &outlines, const grid::Grid &locator,
                    coord_t prune_length, coord_t straightening_distance, const plugin_run_context *run_ctx) const;
    // Host randomness selects branch order; junction overlap trims paths without mutating nodes.
    work::Polylines paths(coord_t overlap, const plugin_run_context *run_ctx = nullptr) const;

private:
    struct Junction { coord_t distance; work::Point point; };
    NodePtr clone(const plugin_run_context *run_ctx) const;
    // Leaf-first pruning spends the physical layer delta across consecutive branch segments.
    coord_t prune(coord_t distance, const plugin_run_context *run_ctx);
    // Junction movement is bounded while nearly collinear short chains may collapse.
    Junction straighten(coord_t magnitude, const work::Point &above, coord_t distance,
                         int64_t maximum_squared_distance, const plugin_run_context *run_ctx);
    // Disconnected interior fragments become separate roots requiring new grounding below.
    bool realign(const work::Polygons &outlines, const grid::Grid &locator,
                  std::vector<NodePtr> &parts, const plugin_run_context *run_ctx);
    void convert(size_t path, work::Polylines &output, const plugin_run_context *run_ctx) const;
};
}}}
#endif
