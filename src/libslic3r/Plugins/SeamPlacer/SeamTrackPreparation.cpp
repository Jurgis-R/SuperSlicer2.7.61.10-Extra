///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "SeamTrackPreparation.hpp"
#include "SeamInitializationCancellation.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>

#include "SeamPlacementConstraints.hpp"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"

/*
Shared seam track preparation
=============================
The plan traversal below is the common structural front end for providers that
relate seams between layers. It deliberately stops before choosing any seam:
each provider receives the same loop candidates, stable identities and direct
LayerIsland links, then applies its own chaining policy.
*/

namespace slic3r_api { namespace SeamPlacement { namespace TrackPreparation {

namespace CP = CandidatePreparation;

namespace {

/* Find the LayerIsland which owns one LayerRegionIsland. */
LayerIsland parent_island(const Layer &layer,
                          const LayerRegionIsland &region_island);
/* Prefix structural preparation errors with the requesting provider name. */
std::runtime_error preparation_error(const char *provider_name,
                                     const char *message);

LayerIsland parent_island(const Layer &layer,
                          const LayerRegionIsland &region_island)
{
    for (uint32_t island_index = 0; island_index < layer.island_count();
         ++island_index) {
        const LayerIsland candidate = layer.island(island_index);
        for (uint32_t region_island_index = 0;
             region_island_index < candidate.region_island_count();
             ++region_island_index)
            if (candidate.region_island(region_island_index).handle() ==
                region_island.handle())
                return candidate;
    }
    return LayerIsland();
}

std::runtime_error preparation_error(const char *provider_name,
                                     const char *message)
{
    return std::runtime_error(std::string(provider_name) +
                              " seam placement " + message);
}

} // namespace

bool stable_track_less(const Track &left, const Track &right)
{
    return std::tie(left.stable_id.object, left.stable_id.instance,
                    left.stable_id.layer, left.stable_id.island,
                    left.stable_id.region_island, left.stable_id.perimeter) <
           std::tie(right.stable_id.object, right.stable_id.instance,
                    right.stable_id.layer, right.stable_id.island,
                    right.stable_id.region_island, right.stable_id.perimeter);
}

bool same_shift(c_point left, c_point right)
{
    return left.x == right.x && left.y == right.y;
}

Data prepare(const PrintingPlan &plan,
             const std::vector<PrintRegion> &regions,
             storage_handle *storage,
             const SeamPlacementConstraints &constraints,
             const char *provider_name,
             const plugin_run_context *run_context)
{
    InitializationCancellation::check(run_context);
    if (!plan.valid() || storage == nullptr || provider_name == nullptr)
        throw std::invalid_argument(
            "Seam track preparation needs a valid plan, storage, and provider name.");

    Data data;
    for (const PrintRegion &region : regions)
        if (region.valid()) data.regions.insert(region.handle());

    std::map<const object_handle *, size_t> object_orders;
    std::set<const object_handle *> indexed_objects;
    std::map<std::pair<RegionIslandKey, size_t>, size_t> perimeter_orders;
    std::map<const extrusion_entity_handle *, LoopBinding> raw_bindings;
    size_t next_object_order = 0;

    /* Follow PrintingPlan order for discovery, but assign explicit object,
       layer and island indices so later tie-breaking does not depend on handle
       addresses or the order of graph links. */
    for (uint32_t group_index = 0; group_index < plan.group_count();
         ++group_index) {
        InitializationCancellation::check(run_context);
        const ConstPrintingGroup group = plan.group(group_index);
        for (uint32_t layer_group_index = 0;
             layer_group_index < group.layer_group_count();
             ++layer_group_index) {
            const ConstPrintingLayerGroup layer_group =
                group.layer_group(layer_group_index);
            for (uint32_t tool_index = 0;
                 tool_index < layer_group.tool_group_count(); ++tool_index) {
                const ConstPrintingToolGroup tool =
                    layer_group.tool_group(tool_index);
                for (uint32_t extrusion_index = 0;
                     extrusion_index < tool.extrusion_count();
                     ++extrusion_index) {
                    InitializationCancellation::check_periodic(
                        run_context, extrusion_index);
                    const ConstPrintingExtrusion extrusion =
                        tool.extrusion(extrusion_index);
                    const LayerRegionIsland region_island =
                        extrusion.region_island();
                    const std::vector<CP::CollectedLoop> collected =
                        CP::collect_perimeter_loops(
                            extrusion.root(), extrusion.role(), run_context);
                    const bool has_scored_loop = std::any_of(
                        collected.begin(), collected.end(),
                        [](const CP::CollectedLoop &loop) {
                            return !loop.distance_only;
                        });
                    if (!region_island.valid() ||
                        region_island.region_count() == 0) {
                        if (!collected.empty())
                            throw preparation_error(
                                provider_name,
                                "found a perimeter loop with an invalid source LayerRegionIsland.");
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

                    const c_point shift =
                        plan.object_instance_shift(extrusion);
                    if (!has_scored_loop) {
                        /* Technical loops do not need layer topology or scored
                           candidates; placement projects the approach point. */
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

                    /* Index the source object's graph once. Both upward and
                       downward links are retained because individual policies
                       choose their own traversal direction. */
                    if (indexed_objects.insert(object.handle()).second) {
                        for (uint32_t layer_index = 0;
                             layer_index < object.layer_count(); ++layer_index) {
                            InitializationCancellation::check_periodic(
                                run_context, layer_index);
                            const Layer object_layer = object.layer(layer_index);
                            for (uint32_t island_index = 0;
                                 island_index < object_layer.island_count();
                                 ++island_index) {
                                InitializationCancellation::check_periodic(
                                    run_context, island_index);
                                const LayerIsland island =
                                    object_layer.island(island_index);
                                LayerIslandNode node;
                                node.stable_id.object = object_order;
                                node.stable_id.layer = layer_index;
                                node.stable_id.island = island_index;
                                node.layer = layer_index;
                                for (uint32_t index = 0;
                                     index < island.lower_island_count();
                                     ++index) {
                                    InitializationCancellation::check_periodic(
                                        run_context, index);
                                    if (island.lower_island(index).valid())
                                        node.lower.push_back(
                                            island.lower_island(index).handle());
                                }
                                for (uint32_t index = 0;
                                     index < island.upper_island_count();
                                     ++index) {
                                    InitializationCancellation::check_periodic(
                                        run_context, index);
                                    if (island.upper_island(index).valid())
                                        node.upper.push_back(
                                            island.upper_island(index).handle());
                                }
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
                         ++index) {
                        InitializationCancellation::check_periodic(
                            run_context, index);
                        if (object.layer(index).handle() == layer.handle()) {
                            layer_index = index;
                            break;
                        }
                    }
                    for (uint32_t index = 0; index < layer.island_count();
                         ++index) {
                        InitializationCancellation::check_periodic(
                            run_context, index);
                        if (layer.island(index).handle() == parent.handle()) {
                            island_index = index;
                            break;
                        }
                    }
                    if (parent.valid()) {
                        for (uint32_t index = 0;
                             index < parent.region_island_count(); ++index) {
                            InitializationCancellation::check_periodic(
                                run_context, index);
                            if (parent.region_island(index).handle() ==
                                region_island.handle()) {
                                region_island_index = index;
                                break;
                            }
                        }
                    }
                    if (!parent.valid() || layer_index == size_t(-1) ||
                        island_index == size_t(-1) ||
                        region_island_index == size_t(-1))
                        throw preparation_error(
                            provider_name,
                            "cannot locate a source LayerRegionIsland in object traversal.");

                    const size_t instance_index =
                        printing_plan_object_instance_idx(plan, extrusion);
                    if (instance_index == size_t(-1))
                        throw preparation_error(
                            provider_name,
                            "found an unknown object instance.");
                    for (const CP::CollectedLoop &loop : collected)
                        raw_bindings.emplace(
                            loop.loop,
                            LoopBinding{region_island.handle(), shift,
                                        loop.distance_only, size_t(-1)});

                    const LayerRegion region = region_island.region(0);
                    const c_flow external_flow = region.flow(
                        RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
                    const double fallback_width =
                        unscaled(external_flow.width);
                    const double nozzle =
                        unscaled(external_flow.nozzle_diameter);
                    for (const CP::CollectedLoop &source : collected) {
                        if (source.distance_only ||
                            !source.has_external_geometry)
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
                            shift, width, nozzle, run_context);
                        CP::calculate_layer_attributes(region_island,
                                                       track.candidates,
                                                       run_context);
                        if (track.candidates.empty())
                            throw preparation_error(
                                provider_name,
                                "found an external perimeter without candidates.");
                        const size_t track_index = data.tracks.size();
                        data.tracks.push_back(std::move(track));
                        data.bindings[source.loop] = {
                            region_island.handle(), shift, false, track_index};
                    }
                }
            }
        }
    }

    /* External tracks overwrite their provisional raw binding; internal and
       technical loops retain the island-only fallback recorded above. */
    for (const std::pair<const extrusion_entity_handle *const, LoopBinding>
             &entry : raw_bindings)
        data.bindings.emplace(entry.first, entry.second);
    return data;
}

}}} // namespace slic3r_api::SeamPlacement::TrackPreparation
