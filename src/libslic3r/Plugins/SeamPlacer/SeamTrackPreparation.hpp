///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Shared seam track preparation
=============================
This private helper maps actual external perimeter loops onto their source
LayerRegionIsland, LayerIsland graph node and object instance. It also records
bindings for internal and technical loops. Providers own all selection policy:
Aligned builds and smooths chains, while Contiguous propagates finalized seams
from directly linked lower islands.

The returned structure borrows host handles from the Print and PrintingPlan.
Candidate coordinates and finalized provider targets use object coordinates;
instance_shift converts them to the platter coordinates stored by extrusion
geometry. Preparation is complete before a session is published, so consumers
may read the result concurrently while the source plan remains alive.
*/

#ifndef slic3r_Plugins_SeamPlacer_SeamTrackPreparation_hpp_
#define slic3r_Plugins_SeamPlacer_SeamTrackPreparation_hpp_

#include <cstddef>
#include <map>
#include <set>
#include <vector>

#include "SeamCandidatePreparation.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"

struct storage_handle;

namespace slic3r_api { namespace SeamPlacement {

class SeamPlacementConstraints;

namespace TrackPreparation {

using RegionIslandKey = const layer_region_island_handle *;

struct StableTrackId {
    // These indices come from semantic traversal order, never handle values.
    size_t object = 0;
    size_t instance = 0;
    size_t layer = 0;
    size_t island = 0;
    size_t region_island = 0;
    size_t perimeter = 0;
};

struct Track {
    // Geometry and candidates use object coordinates; placement adds shift.
    const extrusion_entity_handle *loop = nullptr;
    RegionIslandKey region_island = nullptr;
    const layer_island_handle *layer_island = nullptr;
    c_point instance_shift{};
    coord_t slice_z = 0;
    StableTrackId stable_id;
    std::vector<CandidatePreparation::Candidate> candidates;
};

struct LoopBinding {
    // Internal loops omit external_track and select the nearest island track.
    RegionIslandKey region_island = nullptr;
    c_point instance_shift{};
    bool distance_only = false;
    size_t external_track = size_t(-1);
};

struct LayerIslandNode {
    // Direct graph links are borrowed and valid for the initialized session.
    StableTrackId stable_id;
    size_t layer = 0;
    std::vector<const layer_island_handle *> lower;
    std::vector<const layer_island_handle *> upper;
};

struct Data {
    // All containers become immutable before the provider session is exposed.
    std::set<const print_region_handle *> regions;
    std::vector<Track> tracks;
    std::map<const extrusion_entity_handle *, LoopBinding> bindings;
    std::map<const layer_island_handle *, LayerIslandNode> islands;
};

/* Build immutable loop bindings, external tracks and direct graph links. The
   supplied constraints must already be initialized for the same regions. */
Data prepare(const PrintingPlan &plan,
             const std::vector<PrintRegion> &regions,
             storage_handle *storage,
             const SeamPlacementConstraints &constraints,
             const char *provider_name,
             const plugin_run_context *run_context = nullptr);

/* Stable traversal identity is the only ordering key; borrowed pointer values
   and graph insertion order never decide a tie. */
bool stable_track_less(const Track &left, const Track &right);

/* Compare object-instance translations without relying on handle identity. */
bool same_shift(c_point left, c_point right);

} // namespace TrackPreparation
}} // namespace slic3r_api::SeamPlacement

#endif
