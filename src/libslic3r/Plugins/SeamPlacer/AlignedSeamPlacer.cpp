///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "AlignedSeamPlacer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

#include "SeamCandidatePreparation.hpp"
#include "SeamPlacementConstraints.hpp"
#include "SeamCostModel.hpp"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Api/plugin/cpp/SeamPlacerViews.hpp"

/*
Aligned seam placement
======================
Initialization chooses one local anchor for every actual external perimeter,
then grows deterministic chains over adjacent LayerIsland graph links. Only a
chain with at least six tracks is spline-smoothed; all other tracks retain the
local anchor selected from their own filtered candidates. The published state
contains values and borrowed loop identities only and is never mutated by
place_seam(), so parallel placement calls are safe while host geometry lives.
*/

namespace slic3r_api { namespace SeamPlacement { namespace AlignedPlugin {

namespace CP = CandidatePreparation;

namespace {

constexpr double kCandidateCostTolerance = 0.3;
constexpr double kDistanceFactor = 4.0;
constexpr size_t kMinimumChainLength = 6;
constexpr double kSharpAngle = 55.0 * PI / 180.0;
const char *const kPluginId = "seam_placer.aligned";
const char *const kGroupId = "seam_placer_plugin";
const char *const kDependencies[] = { nullptr };

using RegionIslandKey = const layer_region_island_handle *;

struct StableTrackId {
    size_t object = 0;
    size_t instance = 0;
    size_t layer = 0;
    size_t island = 0;
    size_t region_island = 0;
    size_t perimeter = 0;
};

struct Track {
    const extrusion_entity_handle *loop = nullptr;
    RegionIslandKey region_island = nullptr;
    const layer_island_handle *layer_island = nullptr;
    c_point instance_shift{};
    coord_t slice_z = 0;
    StableTrackId stable_id;
    std::vector<CP::Candidate> candidates;
};

struct LoopBinding {
    RegionIslandKey region_island = nullptr;
    c_point instance_shift{};
    bool distance_only = false;
    size_t external_track = size_t(-1);
};

struct LayerIslandNode {
    StableTrackId stable_id;
    size_t layer = 0;
    std::vector<const layer_island_handle *> lower;
    std::vector<const layer_island_handle *> upper;
};

struct AlignedData {
    std::set<const print_region_handle *> regions;
    std::vector<Track> tracks;
    std::map<const extrusion_entity_handle *, LoopBinding> bindings;
    std::map<const layer_island_handle *, LayerIslandNode> islands;
};

struct TrackState {
    size_t local_candidate = size_t(-1);
    size_t chain_candidate = size_t(-1);
    double local_cost = std::numeric_limits<double>::infinity();
    c_point final_point{};
    bool assigned = false;
};

struct Continuation {
    size_t track = size_t(-1);
    size_t candidate = size_t(-1);
    double distance = std::numeric_limits<double>::infinity();
    double cost = std::numeric_limits<double>::infinity();
};

class AlignedSession;
class AlignedSeamPlacer;

/* Find the LayerIsland containing a source LayerRegionIsland. */
LayerIsland parent_island(const Layer &layer,
                          const LayerRegionIsland &region_island);

/* Compute the provider's local score without visibility or travel. */
double local_cost(const CP::Candidate &candidate);
/* Compare track identities without using borrowed handle values or graph order. */
bool stable_track_less(const Track &left, const Track &right);
/* Compare complete candidate identities for deterministic final ties. */
bool stable_candidate_less(const Track &left_track, const CP::Candidate &left,
                           const Track &right_track, const CP::Candidate &right);
/* Select the local best candidate of every independently filtered perimeter. */
void prepare_local_anchors(AlignedData &data, std::vector<TrackState> &states);
/* Find one eligible continuation on a linked island at the adjacent level. */
Continuation find_continuation(const AlignedData &data,
                               const std::vector<TrackState> &states,
                               size_t source_track, size_t source_candidate,
                               bool upward);
/* Walk one graph direction, reserving only the selected branch. */
std::vector<std::pair<size_t, size_t>> walk_chain(
    const AlignedData &data, std::vector<TrackState> &states,
    size_t seed_track, size_t seed_candidate, bool upward);
/* Smooth a long chain and preserve local anchors if fitting is not finite. */
void smooth_chain(const AlignedData &data,
                  const std::vector<std::pair<size_t, size_t>> &chain,
                  std::vector<TrackState> &states);
/* Seed chains by local quality and stable traversal identity. */
void build_aligned_targets(const AlignedData &data,
                           std::vector<TrackState> &states);
/* Prepare Aligned-owned tracks, bindings, and layer-island graph nodes. */
AlignedData prepare_aligned_data(const PrintingPlan &plan,
                                 const std::vector<PrintRegion> &regions,
                                 storage_handle *storage);
/* Verify that the request is a real non-empty closed geometry loop. */
void validate_loop(const ExtrusionEntity &loop);
/* Compare shifts as values so separate object instances never share a track. */
bool same_shift(c_point left, c_point right);

class AlignedSession final : public SeamPlacerSession
{
public:
    explicit AlignedSession(storage_handle *storage);
    void initialize(const PrintingPlan &plan,
                    const std::vector<PrintRegion> &regions) override;
    float get_priority(const ExtrusionEntity &, const LayerRegion &) const override;
    c_point place_seam(const ExtrusionEntity &loop, const LayerRegion &region,
                       c_point start_position) const override;

private:
    storage_handle *m_storage;
    bool m_initialization_started = false;
    AlignedData m_data;
    std::vector<TrackState> m_states;
};

class AlignedSeamPlacer final : public PluginBase
{
public:
    static AlignedSeamPlacer &instance(orchestrator_handle *orchestrator);
    explicit AlignedSeamPlacer(orchestrator_handle *orchestrator);

private:
    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    const char *exclusive_group_impl() const noexcept override;
    const char *exclusive_group_label_impl() const noexcept override;
    const char *exclusive_group_tooltip_impl() const noexcept override;
    slicing_step_t step_impl() const noexcept override;
    const char *const *dependencies_impl() const noexcept override;
    int32_t priority_impl() const noexcept override;
    void run_impl(const plugin_run_context *run_context) const override;
};

LayerIsland parent_island(const Layer &layer,
                          const LayerRegionIsland &region_island)
{
    for (uint32_t island_index = 0; island_index < layer.island_count();
         ++island_index) {
        const LayerIsland candidate = layer.island(island_index);
        for (uint32_t region_island_index = 0;
             region_island_index < candidate.region_island_count();
             ++region_island_index) {
            const LayerRegionIsland child =
                candidate.region_island(region_island_index);
            if (child.handle() == region_island.handle()) return candidate;
        }
    }
    return LayerIsland();
}

double local_cost(const CP::Candidate &candidate)
{
    const double overhang_penalty = candidate.overhang > 0 && candidate.width > 0 ?
        std::clamp(2.0 * (candidate.overhang - candidate.width / 8.0) /
                       candidate.width, 0.0, 3.0) : 0.0;
    return overhang_penalty + 0.6 *
        Slic3r::SeamCostModel::compute_angle_penalty(float(candidate.angle));
}

bool stable_track_less(const Track &left, const Track &right)
{
    return std::tie(left.stable_id.object, left.stable_id.instance,
                    left.stable_id.layer, left.stable_id.island,
                    left.stable_id.region_island, left.stable_id.perimeter) <
           std::tie(right.stable_id.object, right.stable_id.instance,
                    right.stable_id.layer, right.stable_id.island,
                    right.stable_id.region_island, right.stable_id.perimeter);
}

bool stable_candidate_less(const Track &left_track, const CP::Candidate &left,
                           const Track &right_track, const CP::Candidate &right)
{
    if (stable_track_less(left_track, right_track)) return true;
    if (stable_track_less(right_track, left_track)) return false;
    return left.stable_order < right.stable_order;
}

void prepare_local_anchors(AlignedData &data, std::vector<TrackState> &states)
{
    states.resize(data.tracks.size());
    for (size_t track_index = 0; track_index < data.tracks.size(); ++track_index) {
        Track &track = data.tracks[track_index];
        CP::filter_candidates(track.candidates);
        if (track.candidates.empty())
            throw std::runtime_error("Aligned seam filtering removed every perimeter candidate.");
        TrackState &state = states[track_index];
        for (size_t candidate_index = 0;
             candidate_index < track.candidates.size(); ++candidate_index) {
            const double cost = local_cost(track.candidates[candidate_index]);
            if (cost < state.local_cost) {
                state.local_cost = cost;
                state.local_candidate = candidate_index;
            }
        }
        state.chain_candidate = state.local_candidate;
        state.final_point = track.candidates[state.local_candidate].point;
    }
}

Continuation find_continuation(const AlignedData &data,
                               const std::vector<TrackState> &states,
                               size_t source_track, size_t source_candidate,
                               bool upward)
{
    Continuation best;
    const Track &source = data.tracks[source_track];
    const CP::Candidate &source_point = source.candidates[source_candidate];
    const std::map<const layer_island_handle *, LayerIslandNode>::const_iterator source_node =
        data.islands.find(source.layer_island);
    if (source_node == data.islands.end()) return best;
    const std::vector<const layer_island_handle *> &links = upward ?
        source_node->second.upper : source_node->second.lower;
    const size_t expected_layer = upward ? source_node->second.layer + 1 :
        (source_node->second.layer == 0 ? size_t(-1) : source_node->second.layer - 1);

    for (const layer_island_handle *linked_handle : links) {
        const std::map<const layer_island_handle *, LayerIslandNode>::const_iterator linked =
            data.islands.find(linked_handle);
        if (linked == data.islands.end() || linked->second.layer != expected_layer)
            continue;
        for (size_t track_index = 0; track_index < data.tracks.size(); ++track_index) {
            const Track &target = data.tracks[track_index];
            if (states[track_index].assigned || target.layer_island != linked_handle ||
                target.stable_id.object != source.stable_id.object ||
                target.stable_id.instance != source.stable_id.instance)
                continue;
            for (size_t candidate_index = 0;
                 candidate_index < target.candidates.size(); ++candidate_index) {
                const CP::Candidate &candidate = target.candidates[candidate_index];
                const double cost = local_cost(candidate);
                if (cost > states[track_index].local_cost + kCandidateCostTolerance)
                    continue;
                const double distance = unscaled(std::hypot(
                    double(candidate.point.x) - source_point.point.x,
                    double(candidate.point.y) - source_point.point.y));
                const double scale = std::max({source_point.width, source_point.nozzle,
                                               candidate.width, candidate.nozzle});
                if (!std::isfinite(distance) || scale <= 0 ||
                    distance > kDistanceFactor * scale)
                    continue;
                const bool better = distance < best.distance ||
                    (distance == best.distance && (cost < best.cost ||
                    (cost == best.cost && (best.track == size_t(-1) ||
                    stable_candidate_less(target, candidate,
                        data.tracks[best.track],
                        data.tracks[best.track].candidates[best.candidate])))));
                if (better) {
                    best.track = track_index;
                    best.candidate = candidate_index;
                    best.distance = distance;
                    best.cost = cost;
                }
            }
        }
    }
    return best;
}

std::vector<std::pair<size_t, size_t>> walk_chain(
    const AlignedData &data, std::vector<TrackState> &states,
    size_t seed_track, size_t seed_candidate, bool upward)
{
    std::vector<std::pair<size_t, size_t>> result;
    size_t source_track = seed_track;
    size_t source_candidate = seed_candidate;
    while (true) {
        const Continuation next = find_continuation(
            data, states, source_track, source_candidate, upward);
        if (next.track == size_t(-1)) break;
        states[next.track].assigned = true;
        states[next.track].chain_candidate = next.candidate;
        result.push_back({next.track, next.candidate});
        source_track = next.track;
        source_candidate = next.candidate;
    }
    return result;
}

void smooth_chain(const AlignedData &data,
                  const std::vector<std::pair<size_t, size_t>> &chain,
                  std::vector<TrackState> &states)
{
    if (chain.size() < kMinimumChainLength) return;
    std::vector<Slic3r::SeamCostModel::WeightedSplinePoint> observations;
    observations.reserve(chain.size());
    for (const std::pair<size_t, size_t> &entry : chain) {
        const Track &track = data.tracks[entry.first];
        const CP::Candidate &candidate = track.candidates[entry.second];
        double weight = 1.0 / (0.1 +
            Slic3r::SeamCostModel::compute_angle_penalty(float(candidate.angle)));
        if (candidate.enforced) weight += 3.0;
        observations.push_back({unscaled(candidate.point.x),
                                unscaled(candidate.point.y),
                                unscaled(track.slice_z), weight});
    }
    std::vector<c_vec2d> fitted;
    if (!Slic3r::SeamCostModel::fit_weighted_cubic_bspline(
            observations, fitted))
        return;
    std::vector<c_point> smoothed;
    smoothed.reserve(chain.size());
    for (size_t index = 0; index < chain.size(); ++index) {
        const size_t track_index = chain[index].first;
        const CP::Candidate &candidate =
            data.tracks[track_index].candidates[chain[index].second];
        double blend = std::min(1.0, std::pow(std::abs(candidate.angle) /
                                              kSharpAngle, 3.0));
        if (candidate.enforced) blend = std::max(0.4, blend);
        const double x = blend * unscaled(candidate.point.x) +
                         (1.0 - blend) * fitted[index].x;
        const double y = blend * unscaled(candidate.point.y) +
                         (1.0 - blend) * fitted[index].y;
        if (!std::isfinite(x) || !std::isfinite(y)) return;
        smoothed.push_back({coord_t(scale_d(x)), coord_t(scale_d(y))});
    }
    for (size_t index = 0; index < chain.size(); ++index)
        states[chain[index].first].final_point = smoothed[index];
}

void build_aligned_targets(const AlignedData &data,
                           std::vector<TrackState> &states)
{
    std::vector<size_t> seeds(data.tracks.size());
    for (size_t index = 0; index < seeds.size(); ++index) seeds[index] = index;
    std::sort(seeds.begin(), seeds.end(), [&](size_t left, size_t right) {
        if (states[left].local_cost != states[right].local_cost)
            return states[left].local_cost < states[right].local_cost;
        return stable_track_less(data.tracks[left], data.tracks[right]);
    });
    for (const size_t seed : seeds) {
        if (states[seed].assigned) continue;
        states[seed].assigned = true;
        const size_t candidate = states[seed].local_candidate;
        std::vector<std::pair<size_t, size_t>> upper =
            walk_chain(data, states, seed, candidate, true);
        std::vector<std::pair<size_t, size_t>> lower =
            walk_chain(data, states, seed, candidate, false);
        std::vector<std::pair<size_t, size_t>> chain;
        chain.reserve(lower.size() + 1 + upper.size());
        for (std::vector<std::pair<size_t, size_t>>::reverse_iterator iterator =
                 lower.rbegin(); iterator != lower.rend(); ++iterator)
            chain.push_back(*iterator);
        chain.push_back({seed, candidate});
        chain.insert(chain.end(), upper.begin(), upper.end());
        smooth_chain(data, chain, states);
    }
}

AlignedData prepare_aligned_data(const PrintingPlan &plan,
                                 const std::vector<PrintRegion> &regions,
                                 storage_handle *storage)
{
    if (!plan.valid() || storage == nullptr)
        throw std::invalid_argument(
            "Aligned seam placement needs a valid plan and temporary storage.");
    AlignedData data;
    SeamPlacementConstraints constraints;
    constraints.initialize_for_regions(plan, storage, regions);
    for (const PrintRegion &region : regions)
        if (region.valid()) data.regions.insert(region.handle());

    std::map<const object_handle *, size_t> object_orders;
    std::set<const object_handle *> indexed_objects;
    std::map<std::pair<RegionIslandKey, size_t>, size_t> perimeter_orders;
    std::map<const extrusion_entity_handle *, LoopBinding> raw_bindings;
    size_t next_object_order = 0;

    for (uint32_t group_index = 0; group_index < plan.group_count();
         ++group_index) {
        const PrintingGroup group = plan.group(group_index);
        for (uint32_t layer_group_index = 0;
             layer_group_index < group.layer_group_count();
             ++layer_group_index) {
            const PrintingLayerGroup layer_group =
                group.layer_group(layer_group_index);
            for (uint32_t tool_index = 0;
                 tool_index < layer_group.tool_group_count(); ++tool_index) {
                const PrintingToolGroup tool =
                    layer_group.tool_group(tool_index);
                for (uint32_t extrusion_index = 0;
                     extrusion_index < tool.extrusion_count();
                     ++extrusion_index) {
                    const PrintingExtrusion extrusion =
                        tool.extrusion(extrusion_index);
                    const LayerRegionIsland region_island =
                        extrusion.region_island();
                    const std::vector<CP::CollectedLoop> collected =
                        CP::collect_perimeter_loops(
                            extrusion.root(), extrusion.role());
                    const bool has_non_distance_only_loop = std::any_of(
                        collected.begin(), collected.end(),
                        [](const CP::CollectedLoop &loop) {
                            return !loop.distance_only;
                        });
                    if (!region_island.valid() ||
                        region_island.region_count() == 0) {
                        if (!collected.empty())
                            throw std::runtime_error(
                                "Aligned seam placement found a perimeter loop with an invalid source LayerRegionIsland.");
                        continue;
                    }
                    bool assigned = false;
                    for (uint32_t index = 0;
                         index < region_island.region_count(); ++index) {
                        const PrintRegion print_region =
                            region_island.region(index).print_region();
                        if (print_region.valid() &&
                            data.regions.count(print_region.handle()) != 0) {
                            assigned = true;
                            break;
                        }
                    }
                    if (!assigned) continue;

                    const c_point shift = plan.object_instance_shift(extrusion);
                    if (!has_non_distance_only_loop) {
                        /* Technical-only loops need only a binding and their
                           own projection; skip parent-island and graph setup. */
                        for (const CP::CollectedLoop &loop : collected)
                            raw_bindings.emplace(
                                loop.loop,
                                LoopBinding{region_island.handle(), shift,
                                            loop.distance_only, size_t(-1)});
                        continue;
                    }

                    const Layer layer = region_island.region(0).layer();
                    const Object object = layer.object();
                    const std::map<const object_handle *, size_t>::iterator
                        known_object = object_orders.find(object.handle());
                    const size_t object_order =
                        known_object == object_orders.end() ?
                            next_object_order++ : known_object->second;
                    if (known_object == object_orders.end())
                        object_orders.emplace(object.handle(), object_order);

                    if (indexed_objects.insert(object.handle()).second) {
                        for (uint32_t layer_index = 0;
                             layer_index < object.layer_count(); ++layer_index) {
                            const Layer object_layer = object.layer(layer_index);
                            for (uint32_t island_index = 0;
                                 island_index < object_layer.island_count();
                                 ++island_index) {
                                const LayerIsland island =
                                    object_layer.island(island_index);
                                LayerIslandNode node;
                                node.stable_id.object = object_order;
                                node.stable_id.layer = layer_index;
                                node.stable_id.island = island_index;
                                node.layer = layer_index;
                                for (uint32_t index = 0;
                                     index < island.lower_island_count();
                                     ++index)
                                    if (island.lower_island(index).valid())
                                        node.lower.push_back(
                                            island.lower_island(index).handle());
                                for (uint32_t index = 0;
                                     index < island.upper_island_count();
                                     ++index)
                                    if (island.upper_island(index).valid())
                                        node.upper.push_back(
                                            island.upper_island(index).handle());
                                data.islands.emplace(island.handle(),
                                                     std::move(node));
                            }
                        }
                    }

                    size_t layer_index = size_t(-1);
                    size_t island_index = size_t(-1);
                    size_t region_island_index = size_t(-1);
                    const LayerIsland parent =
                        parent_island(layer, region_island);
                    for (uint32_t index = 0; index < object.layer_count();
                         ++index)
                        if (object.layer(index).handle() == layer.handle()) {
                            layer_index = index;
                            break;
                        }
                    for (uint32_t index = 0; index < layer.island_count();
                         ++index)
                        if (layer.island(index).handle() == parent.handle()) {
                            island_index = index;
                            break;
                        }
                    if (parent.valid())
                        for (uint32_t index = 0;
                             index < parent.region_island_count(); ++index)
                            if (parent.region_island(index).handle() ==
                                region_island.handle()) {
                                region_island_index = index;
                                break;
                            }
                    if (!parent.valid() || layer_index == size_t(-1) ||
                        island_index == size_t(-1) ||
                        region_island_index == size_t(-1))
                        throw std::runtime_error(
                            "Aligned seam placement cannot locate a source LayerRegionIsland in object traversal.");

                    const size_t instance_index =
                        printing_plan_object_instance_idx(plan, extrusion);
                    if (instance_index == size_t(-1))
                        throw std::runtime_error(
                            "Aligned seam placement found an unknown object instance.");
                    for (const CP::CollectedLoop &loop : collected)
                        raw_bindings.emplace(
                            loop.loop,
                            LoopBinding{region_island.handle(), shift,
                                        loop.distance_only, size_t(-1)});

                    const LayerRegion region = region_island.region(0);
                    const c_flow external_flow = region.flow(
                        RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
                    const double fallback_width = unscaled(external_flow.width);
                    const double nozzle =
                        unscaled(external_flow.nozzle_diameter);
                    for (const CP::CollectedLoop &source : collected) {
                        if (source.distance_only || !source.has_external_geometry)
                            continue;
                        const ExtrusionEntity loop(source.loop);
                        Track track;
                        track.loop = source.loop;
                        track.region_island = region_island.handle();
                        track.layer_island = parent.handle();
                        track.instance_shift = shift;
                        track.slice_z = layer.slice_z();
                        track.stable_id.object = object_order;
                        track.stable_id.instance = instance_index;
                        track.stable_id.layer = layer_index;
                        track.stable_id.island = island_index;
                        track.stable_id.region_island = region_island_index;
                        track.stable_id.perimeter = perimeter_orders[
                            {region_island.handle(), instance_index}]++;
                        const double width = source.external_width > 0 ?
                            source.external_width : fallback_width;
                        track.candidates = CP::prepare_loop_candidates(
                            storage, constraints, region_island, loop, region,
                            shift, width, nozzle);
                        CP::calculate_layer_attributes(region_island,
                                                       track.candidates);
                        if (track.candidates.empty())
                            throw std::runtime_error(
                                "Aligned seam placement found an external perimeter without candidates.");
                        const size_t track_index = data.tracks.size();
                        data.tracks.push_back(std::move(track));
                        data.bindings[source.loop] = {
                            region_island.handle(), shift, false, track_index};
                    }
                }
            }
        }
    }
    for (const std::pair<const extrusion_entity_handle *const, LoopBinding>
             &entry : raw_bindings)
        data.bindings.emplace(entry.first, entry.second);
    return data;
}

void validate_loop(const ExtrusionEntity &loop)
{
    if (!loop.valid() || !loop.is_loop() || loop.empty())
        throw std::invalid_argument(
            "Aligned seam placement needs a non-empty closed loop.");
    const std::vector<c_point> points = loop.collect_points();
    if (points.size() < 2 || points.front().x != points.back().x ||
        points.front().y != points.back().y)
        throw std::invalid_argument(
            "Aligned seam placement rejects open or invalid loop geometry.");
}

bool same_shift(c_point left, c_point right)
{
    return left.x == right.x && left.y == right.y;
}

AlignedSession::AlignedSession(storage_handle *storage) : m_storage(storage) {}

void AlignedSession::initialize(const PrintingPlan &plan,
                                const std::vector<PrintRegion> &regions)
{
    if (m_initialization_started)
        throw std::logic_error(
            "Aligned seam placement sessions may only be initialized once.");
    m_initialization_started = true;
    m_data = prepare_aligned_data(plan, regions, m_storage);
    prepare_local_anchors(m_data, m_states);
    build_aligned_targets(m_data, m_states);
    m_storage = nullptr;
}

float AlignedSession::get_priority(const ExtrusionEntity &,
                                   const LayerRegion &) const
{
    return 50.0f;
}

c_point AlignedSession::place_seam(const ExtrusionEntity &loop,
                                    const LayerRegion &region,
                                    c_point start_position) const
{
    validate_loop(loop);
    if (!region.valid())
        throw std::invalid_argument(
            "Aligned seam placement needs a valid LayerRegion.");
    const PrintRegion print_region = region.print_region();
    if (!print_region.valid() || m_data.regions.count(print_region.handle()) == 0)
        throw std::invalid_argument(
            "Aligned seam LayerRegion was not assigned to this provider session.");
    const std::map<const extrusion_entity_handle *, LoopBinding>::const_iterator binding =
        m_data.bindings.find(loop.handle());
    if (binding == m_data.bindings.end())
        throw std::invalid_argument(
            "Aligned seam loop was not present in the initialized plan.");
    if (binding->second.distance_only) {
        const std::pair<c_point, double> projection =
            CP::project_onto_loop(loop, start_position);
        if (!std::isfinite(projection.second))
            throw std::runtime_error(
                "Aligned seam could not project the approach position onto a distance-only loop.");
        return projection.first;
    }

    if (binding->second.external_track != size_t(-1)) {
        const size_t track_index = binding->second.external_track;
        const c_point local = m_states[track_index].final_point;
        const c_point target = {local.x + binding->second.instance_shift.x,
                                local.y + binding->second.instance_shift.y};
        const std::pair<c_point, double> projection =
            CP::project_onto_loop(loop, target);
        if (!std::isfinite(projection.second))
            throw std::runtime_error(
                "Aligned seam could not project onto external loop geometry.");
        return projection.first;
    }

    c_point best{};
    double best_distance = std::numeric_limits<double>::infinity();
    size_t best_track = size_t(-1);
    for (size_t track_index = 0; track_index < m_data.tracks.size(); ++track_index) {
        const Track &track = m_data.tracks[track_index];
        if (track.region_island != binding->second.region_island ||
            !same_shift(track.instance_shift, binding->second.instance_shift))
            continue;
        const c_point local = m_states[track_index].final_point;
        const c_point target = {local.x + binding->second.instance_shift.x,
                                local.y + binding->second.instance_shift.y};
        const std::pair<c_point, double> projection =
            CP::project_onto_loop(loop, target);
        if (projection.second < best_distance ||
            (projection.second == best_distance && best_track != size_t(-1) &&
             stable_track_less(track, m_data.tracks[best_track]))) {
            best = projection.first;
            best_distance = projection.second;
            best_track = track_index;
        }
    }
    if (best_track == size_t(-1) || !std::isfinite(best_distance))
        throw std::runtime_error(
            "Aligned seam source island has no aligned external track.");
    return best;
}

AlignedSeamPlacer &AlignedSeamPlacer::instance(orchestrator_handle *orchestrator)
{
    static AlignedSeamPlacer plugin(orchestrator);
    return plugin;
}

AlignedSeamPlacer::AlignedSeamPlacer(orchestrator_handle *orchestrator) :
    PluginBase(orchestrator)
{}

const char *AlignedSeamPlacer::id_impl() const noexcept { return kPluginId; }
const char *AlignedSeamPlacer::name_impl() const noexcept { return "Aligned"; }
const char *AlignedSeamPlacer::description_impl() const noexcept
{
    return "Aligns locally suitable seam positions across connected layers.";
}
const char *AlignedSeamPlacer::exclusive_group_impl() const noexcept { return kGroupId; }
const char *AlignedSeamPlacer::exclusive_group_label_impl() const noexcept { return "Seam placement"; }
const char *AlignedSeamPlacer::exclusive_group_tooltip_impl() const noexcept
{
    return "Selects the service used to choose the starting point of closed extrusion loops.";
}
slicing_step_t AlignedSeamPlacer::step_impl() const noexcept { return SEAM_PLACER; }
const char *const *AlignedSeamPlacer::dependencies_impl() const noexcept { return kDependencies; }
int32_t AlignedSeamPlacer::priority_impl() const noexcept { return 950; }

void AlignedSeamPlacer::run_impl(const plugin_run_context *run_context) const
{
    run_ctx_seam_placer *context = plugin_ctx_as_seam_placer(run_context);
    if (context == nullptr || context->plan == nullptr ||
        (context->region_count != 0 && context->regions == nullptr))
        throw std::invalid_argument(
            "Aligned seam placement received an invalid run context.");
    std::vector<PrintRegion> regions;
    regions.reserve(context->region_count);
    for (uint32_t index = 0; index < context->region_count; ++index)
        regions.emplace_back(context->regions[index]);
    context->instance = make_seam_placer_instance(
        std::unique_ptr<SeamPlacerSession>(
            new AlignedSession(run_context->plugin_storage)),
        PrintingPlan(context->plan), regions);
}

} // namespace

std::unique_ptr<SeamPlacerSession> make_aligned_seam_session(storage_handle *storage)
{
    return std::unique_ptr<SeamPlacerSession>(new AlignedSession(storage));
}

void register_aligned_seam_placer_plugin(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(
        orchestrator, AlignedSeamPlacer::instance(orchestrator).c_instance());
}

}}} // namespace slic3r_api::SeamPlacement::AlignedPlugin
