///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Shared seam candidate preparation
=================================
This private header describes the object-local candidate material shared by
the built-in Cost, Aligned and Contiguous seam providers. Cost may merge
candidates by LayerRegionIsland, while the cross-layer providers keep each
external perimeter separate and use stable traversal identities for ties.
*/

#ifndef slic3r_Plugins_SeamPlacer_SeamCandidatePreparation_hpp_
#define slic3r_Plugins_SeamPlacer_SeamCandidatePreparation_hpp_

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_plugin_types.h"

struct storage_handle;

namespace slic3r_api {
class ExtrusionEntity;
class LayerRegion;
class LayerRegionIsland;

namespace SeamPlacement {
class SeamPlacementConstraints;

namespace CandidatePreparation {

struct Candidate {
    c_point point{};
    double angle = 0;
    double overhang = 0;
    double embedded = 0;
    double width = 0;
    double nozzle = 0;
    bool blocked = false;
    bool enforced = false;
    bool boundary = false;
    size_t stable_order = 0;
};

/* One atomic perimeter loop discovered in an extrusion tree. distance_only
   summarizes all effective roles in the marked subtree; providers use it to
   keep technical loops as projection targets without treating them as
   seam-candidate sources. */
struct CollectedLoop {
    const extrusion_entity_handle *loop = nullptr;
    bool distance_only = false;
    bool has_external_geometry = false;
    double external_width = 0;
};

/* Collect every non-empty atomic perimeter loop while inheriting attributes
   downward and summarizing descendant metadata back onto the loop identity. */
std::vector<CollectedLoop> collect_perimeter_loops(
    const ExtrusionEntity &root, raw_extrusion_role inherited_role);

/* Generate ordered geometric, painted-constraint, and boundary candidates for
   one external perimeter loop. */
std::vector<Candidate> prepare_loop_candidates(
    storage_handle *storage, const SeamPlacementConstraints &constraints,
    const LayerRegionIsland &island, const ExtrusionEntity &loop,
    const LayerRegion &region, c_point shift, double width, double nozzle);

/* Add legacy overhang, embedding, and neighboring-region boundary attributes. */
void calculate_layer_attributes(const LayerRegionIsland &island,
                               std::vector<Candidate> &candidates);

/* Apply the common blocker/enforcer/boundary/overhang/embedding policy. */
void filter_candidates(std::vector<Candidate> &candidates);

/* Project onto real line and arc segments; strict ties retain traversal order. */
std::pair<c_point, double> project_onto_loop(const ExtrusionEntity &entity,
                                             c_point target);

}} // namespace SeamPlacement::CandidatePreparation
} // namespace slic3r_api

#endif
