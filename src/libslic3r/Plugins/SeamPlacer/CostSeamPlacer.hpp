///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
CostSeamPlacer: immutable cost-based seam placement
==================================================
The built-in provider prepares one object-coordinate candidate set per source
LayerRegionIsland and binds every perimeter loop in the PrintingPlan to its
source island and instance shift. Sessions retain immutable prepared candidate
tables plus borrowed plan, loop, region and native configuration identities;
seam_travel_cost is read for each placement call. Requests may run concurrently
only while those borrowed identities and their native configuration are not
mutated.
*/

#ifndef slic3r_Plugins_SeamPlacer_CostSeamPlacer_hpp_
#define slic3r_Plugins_SeamPlacer_CostSeamPlacer_hpp_

#include <map>
#include <memory>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_plugin_types.h"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"

namespace slic3r_api {
class SeamPlacerSession;

namespace SeamPlacement { namespace CostPlugin {

struct SeamCandidate {
    c_point point;
    double static_cost;
};

using RegionIslandKey = const layer_region_island_handle *;

struct LoopBinding {
    RegionIslandKey island;
    c_point instance_shift;
};

std::unique_ptr<SeamPlacerSession> make_cost_seam_session(storage_handle *storage);
void register_cost_seam_placer_plugin(orchestrator_handle *orchestrator);

}} // namespace SeamPlacement::CostPlugin
} // namespace slic3r_api

#endif
