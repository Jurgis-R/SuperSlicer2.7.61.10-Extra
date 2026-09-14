///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "SeamPlacerRouter.hpp"

#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_seam_placer.h"
#include "libslic3r/Api/plugin/cpp/OrchestratorViews.hpp"
#include "libslic3r/Api/plugin/cpp/SeamPlacerViews.hpp"
#include "libslic3r/Plugins/Ordering/ExtrusionTreeOrderingGeometry.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"

/*
SeamPlacerRouter.cpp
====================

The selector for seam placement belongs to PrintRegion configuration, but an
ordering worker receives only a loop from the transformed PrintingPlan. This
file joins those two views before parallel ordering starts. Sessions are
grouped by provider to avoid repeating expensive initialization, and each loop
stores the winning LayerRegion together with shared ownership of that session.
*/

namespace slic3r_api { namespace SeamPlacement {
namespace {

const char *const k_seam_placer_group = "seam_placer_plugin";

struct ProviderBatch;
struct LoopRoute;

/* Enumerate unique PrintRegions in stable object/region order. */
std::vector<PrintRegion> collect_print_regions(const Print &print, const PrintingPlan &plan);

/* Group regions by the active provider selected from their configurations. */
std::vector<ProviderBatch> group_regions_by_provider(
    orchestrator_handle *orchestrator,
    const std::vector<PrintRegion> &regions);

/* Execute each provider once and map every assigned region to its session. */
std::map<const print_region_handle *, std::shared_ptr<SeamPlacer>> create_sessions(
    orchestrator_handle *orchestrator,
    const Print &print,
    const PrintingPlan &plan,
    std::vector<ProviderBatch> &batches);

/* Resolve all atomic loops below entity; a loop remains one indivisible unit. */
void build_entity_routes(
    storage_handle *scratch_storage,
    const PrintingPlan &plan,
    const PrintingExtrusion &extrusion,
    const ExtrusionEntity &entity,
    const std::map<const print_region_handle *, std::shared_ptr<SeamPlacer>> &sessions,
    std::map<const extrusion_entity_handle *, LoopRoute> &routes);

/* Walk every extrusion root and build the immutable loop routing table. */
std::map<const extrusion_entity_handle *, LoopRoute> build_plan_routes(
    storage_handle *scratch_storage,
    const PrintingPlan &plan,
    const std::map<const print_region_handle *, std::shared_ptr<SeamPlacer>> &sessions);

struct ProviderBatch
{
    PluginView provider;
    std::vector<PrintRegion> regions;
    std::vector<const print_region_handle *> raw_regions;
};

struct LoopRoute
{
    LayerRegion region;
    std::shared_ptr<SeamPlacer> session;
};

std::vector<PrintRegion> collect_print_regions(const Print &print, const PrintingPlan &plan)
{
    std::vector<PrintRegion> result;
    std::map<const print_region_handle *, bool> seen;
    for (uint32_t object_idx = 0; object_idx < print.object_count(); ++object_idx) {
        const Object object = print.object(object_idx);
        for (uint32_t region_idx = 0; region_idx < object.print_region_count(); ++region_idx) {
            const PrintRegion region = object.print_region(region_idx);
            if (region.valid() && seen.emplace(region.handle(), true).second)
                result.push_back(region);
        }
    }
    for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
        const PrintingGroup group = plan.group(group_idx);
        for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx) {
            const PrintingLayerGroup layer = group.layer_group(layer_idx);
            for (uint32_t tool_idx = 0; tool_idx < layer.tool_group_count(); ++tool_idx) {
                const PrintingToolGroup tool = layer.tool_group(tool_idx);
                for (uint32_t idx = 0; idx < tool.extrusion_count(); ++idx) {
                    const LayerRegionIsland island = tool.extrusion(idx).region_island();
                    if (!island.valid()) continue;
                    for (uint32_t region_idx = 0; region_idx < island.region_count(); ++region_idx) {
                        const PrintRegion region = island.region(region_idx).print_region();
                        if (region.valid() && seen.emplace(region.handle(), true).second)
                            result.push_back(region);
                    }
                }
            }
        }
    }
    return result;
}

std::vector<ProviderBatch> group_regions_by_provider(
    orchestrator_handle *orchestrator,
    const std::vector<PrintRegion> &regions)
{
    OrchestratorView orchestrator_view(orchestrator);
    std::vector<ProviderBatch> batches;
    for (size_t region_idx = 0; region_idx < regions.size(); ++region_idx) {
        const PrintRegion &region = regions[region_idx];
        const PluginView provider = orchestrator_view.select_plugin(
            region.config().handle(), SEAM_PLACER, k_seam_placer_group);
        if (!provider.valid()) {
            const std::string option_key = Slic3r::Steps::plugin_exclusive_group_option_key(
                *reinterpret_cast<const Slic3r::Orchestrator *>(orchestrator),
                SEAM_PLACER, k_seam_placer_group);
            const ConfigOption option = region.config().get(option_key.c_str());
            const std::string selected = option.valid() ?
                std::to_string(option.get_int()) : "<missing>";
            throw std::runtime_error(
                "PrintRegion #" + std::to_string(region_idx) + " has selector '" +
                option_key + "' = '" + selected +
                "', which does not identify an active provider from '" +
                k_seam_placer_group + "'.");
        }

        ProviderBatch *batch = nullptr;
        for (ProviderBatch &candidate : batches) {
            if (candidate.provider.handle() == provider.handle()) {
                batch = &candidate;
                break;
            }
        }
        if (batch == nullptr) {
            batches.push_back(ProviderBatch{});
            batch = &batches.back();
            batch->provider = provider;
        }
        batch->regions.push_back(region);
    }
    return batches;
}

std::map<const print_region_handle *, std::shared_ptr<SeamPlacer>> create_sessions(
    orchestrator_handle *orchestrator,
    const Print &print,
    const PrintingPlan &plan,
    std::vector<ProviderBatch> &batches)
{
    OrchestratorView orchestrator_view(orchestrator);
    std::map<const print_region_handle *, std::shared_ptr<SeamPlacer>> result;
    for (ProviderBatch &batch : batches) {
        if (batch.regions.size() > std::numeric_limits<uint32_t>::max())
            throw std::runtime_error("A seam placer provider received too many PrintRegions.");
        batch.raw_regions.reserve(batch.regions.size());
        for (const PrintRegion &region : batch.regions)
            batch.raw_regions.push_back(region.handle());

        run_ctx_seam_placer context = {};
        context.plan = plan.handle();
        context.regions = batch.raw_regions.empty() ? nullptr : batch.raw_regions.data();
        context.region_count = uint32_t(batch.raw_regions.size());
        const raw_plugin_execution_status status = orchestrator_view.execute(
            batch.provider, print.handle(), context);
        if (status != RAW_PLUGIN_EXECUTION_SUCCESS)
            throw std::runtime_error(
                "SEAM_PLACER provider '" + batch.provider.id() +
                "' could not initialize its regional session.");

        std::shared_ptr<SeamPlacer> session;
        try {
            session = std::make_shared<SeamPlacer>(context.instance);
        } catch (const std::exception &exception) {
            throw std::runtime_error(
                "SEAM_PLACER provider '" + batch.provider.id() +
                "' published an invalid session: " + exception.what());
        }
        for (const PrintRegion &region : batch.regions)
            result.emplace(region.handle(), session);
    }
    return result;
}

void build_entity_routes(
    storage_handle *scratch_storage,
    const PrintingPlan &plan,
    const PrintingExtrusion &extrusion,
    const ExtrusionEntity &entity,
    const std::map<const print_region_handle *, std::shared_ptr<SeamPlacer>> &sessions,
    std::map<const extrusion_entity_handle *, LoopRoute> &routes)
{
    if (entity.empty())
        return;
    if (entity.is_loop()) {
        const std::vector<LayerRegion> candidates =
            Ordering::TreeOrderingGeometry::resolve_source_layer_regions(
                scratch_storage, plan, extrusion, entity);
        LayerRegion selected = candidates.front();
        const print_region_handle *selected_print_region = selected.print_region().handle();
        const std::map<const print_region_handle *, std::shared_ptr<SeamPlacer>>::const_iterator first =
            sessions.find(selected_print_region);
        if (first == sessions.end())
            throw std::runtime_error(
                "A loop candidate region has no initialized SEAM_PLACER session.");
        std::shared_ptr<SeamPlacer> selected_session = first->second;

        if (candidates.size() > 1) {
            float best_priority = selected_session->get_priority(entity, selected);
            for (size_t candidate_idx = 1; candidate_idx < candidates.size(); ++candidate_idx) {
                const LayerRegion candidate = candidates[candidate_idx];
                const print_region_handle *print_region = candidate.print_region().handle();
                const std::map<const print_region_handle *, std::shared_ptr<SeamPlacer>>::const_iterator found =
                    sessions.find(print_region);
                if (found == sessions.end())
                    throw std::runtime_error(
                        "A loop candidate region has no initialized SEAM_PLACER session.");
                const float priority = found->second->get_priority(entity, candidate);
                // Strict comparison preserves source-island order on exact ties.
                if (priority > best_priority) {
                    best_priority = priority;
                    selected = candidate;
                    selected_session = found->second;
                }
            }
        }

        if (!routes.emplace(entity.handle(), LoopRoute{selected, selected_session}).second)
            throw std::runtime_error(
                "An atomic loop appears more than once in the PrintingPlan.");
        return;
    }

    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx)
        build_entity_routes(
            scratch_storage, plan, extrusion, entity.child(child_idx), sessions, routes);
}

std::map<const extrusion_entity_handle *, LoopRoute> build_plan_routes(
    storage_handle *scratch_storage,
    const PrintingPlan &plan,
    const std::map<const print_region_handle *, std::shared_ptr<SeamPlacer>> &sessions)
{
    std::map<const extrusion_entity_handle *, LoopRoute> result;
    for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
        const PrintingGroup group = plan.group(group_idx);
        for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx) {
            const PrintingLayerGroup layer = group.layer_group(layer_idx);
            for (uint32_t tool_idx = 0; tool_idx < layer.tool_group_count(); ++tool_idx) {
                const PrintingToolGroup tool = layer.tool_group(tool_idx);
                for (uint32_t extrusion_idx = 0; extrusion_idx < tool.extrusion_count(); ++extrusion_idx) {
                    const PrintingExtrusion extrusion = tool.extrusion(extrusion_idx);
                    build_entity_routes(
                        scratch_storage, plan, extrusion, extrusion.root(), sessions, result);
                }
            }
        }
    }
    return result;
}

} // namespace

struct SeamPlacerRouter::Impl
{
    std::vector<ProviderBatch> batches;
    std::map<const print_region_handle *, std::shared_ptr<SeamPlacer>> sessions;
    std::map<const extrusion_entity_handle *, LoopRoute> routes;
};

SeamPlacerRouter::SeamPlacerRouter(orchestrator_handle *orchestrator,
                                   const Print &print,
                                   const PrintingPlan &plan,
                                   storage_handle *scratch_storage) :
    m_impl(new Impl())
{
    if (orchestrator == nullptr || !print.valid() || !plan.valid() || scratch_storage == nullptr)
        throw std::invalid_argument(
            "SeamPlacerRouter needs an orchestrator, Print, PrintingPlan, and scratch storage.");
    const std::vector<PrintRegion> regions = collect_print_regions(print, plan);
    m_impl->batches = group_regions_by_provider(orchestrator, regions);
    m_impl->sessions = create_sessions(orchestrator, print, plan, m_impl->batches);
    m_impl->routes = build_plan_routes(scratch_storage, plan, m_impl->sessions);
}

SeamPlacerRouter::~SeamPlacerRouter() = default;

c_point SeamPlacerRouter::place_seam(const ExtrusionEntity &loop,
                                     const c_point start_position) const
{
    if (!loop.is_loop())
        throw std::invalid_argument(
            "SeamPlacerRouter needs a non-empty fixed continuous closed loop.");
    const std::map<const extrusion_entity_handle *, LoopRoute>::const_iterator route =
        m_impl->routes.find(loop.handle());
    if (route == m_impl->routes.end())
        throw std::runtime_error(
            "The loop has no precomputed regional SEAM_PLACER route.");
    return route->second.session->place_seam(
        loop, route->second.region, start_position);
}

}} // namespace slic3r_api::SeamPlacement
