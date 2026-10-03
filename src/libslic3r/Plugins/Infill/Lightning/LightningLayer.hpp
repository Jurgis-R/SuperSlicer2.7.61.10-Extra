///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ Copyright (c) 2021 Ultimaker B.V.
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

#ifndef plugins_lightning_layer_hpp_
#define plugins_lightning_layer_hpp_
#include <unordered_map>
#include "LightningTreeNode.hpp"

namespace slic3r_api { namespace Infill { namespace Lightning {
using NodeGrid = std::unordered_multimap<work::Point, std::weak_ptr<Node>, PointHash>;

// Keep anchoring and root reconnection synchronized with Fill/Lightning/LightningLayer.cpp.
class LayerTree
{
public:
    std::vector<NodePtr> roots;
    void generate(const work::Polygons &overhang, const work::Polygons &outlines, const grid::Grid &locator,
                   coord_t radius, coord_t wall_radius, storage_handle *scratch, const plugin_run_context *run_ctx);
    void reconnect(const std::vector<NodePtr> &previous, const work::Polygons &outlines, const grid::Grid &locator,
                    coord_t radius, coord_t wall_radius, const plugin_run_context *run_ctx);
    work::Polylines paths(coord_t overlap, const plugin_run_context *run_ctx = nullptr) const;

private:
    struct Ground
    {
        NodePtr node;
        work::Point boundary;
        work::Point point() const { return node ? node->location : boundary; }
    };
    void populate(NodeGrid &nodes, const work::BoundingBox &bounds);
    Ground best_ground(const work::Point &point, const work::Polygons &outlines,
                        const grid::Grid &locator, coord_t radius, coord_t wall_radius,
                        const NodeGrid &nodes, const NodePtr &exclude, const plugin_run_context *run_ctx);
};
}}}
#endif
