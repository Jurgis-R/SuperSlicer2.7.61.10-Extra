
#include "StepPostPerimeterGeneration.hpp"

#include <utility>

#include "libslic3r/Api/internal/LayerIslandAccess.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_post_perimeter.h"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"

#include "StepRunner.hpp"

namespace Slic3r::Steps::StepPostPerimeterGeneration {
namespace {

LayerSliceIsland *to_layer_island(const layer_island_handle *handle)
{
    return const_cast<LayerSliceIsland *>(reinterpret_cast<const LayerSliceIsland *>(handle));
}

LayerRegionIsland *to_layer_region_island(const layer_region_island_handle *handle)
{
    return const_cast<LayerRegionIsland *>(reinterpret_cast<const LayerRegionIsland *>(handle));
}

const ExPolygons *to_expolygons(const expolygon_collection_handle *handle)
{
    return reinterpret_cast<const ExPolygons *>(handle);
}

ExtrusionRole bucket_role_from_raw(raw_extrusion_role role)
{
    if ((role & RAW_EXTRUSION_ROLE_THIN) != 0 || role == RAW_EXTRUSION_ROLE_GAP_FILL)
        return LayerRegionIsland::GAP_FILLS;
    if ((role & RAW_EXTRUSION_ROLE_INFILL) != 0)
        return LayerRegionIsland::INFILLS;
    if ((role & RAW_EXTRUSION_ROLE_IRONING) != 0)
        return LayerRegionIsland::IRONINGS;
    if ((role & RAW_EXTRUSION_ROLE_MILL) != 0)
        return LayerRegionIsland::MILLS;
    if ((role & RAW_EXTRUSION_ROLE_SUPPORT) != 0)
        return (role & RAW_EXTRUSION_ROLE_EXTERNAL) != 0 ? LayerRegionIsland::SUPPORT_INTERFACE :
                                                           LayerRegionIsland::SUPPORT;
    return LayerRegionIsland::PERIMETERS;
}

extrusion_entity_handle *get_region_island_mutable_extrusion_callback(
    const layer_region_island_handle *region_island_handle,
    raw_extrusion_role role)
{
    LayerRegionIsland *region_island = to_layer_region_island(region_island_handle);
    if (region_island == nullptr)
        return nullptr;

    // Post-perimeter plugins edit buckets produced by earlier steps. Missing
    // buckets stay absent, so a plugin can safely probe optional extrusion
    // classes without creating empty LayerRegionIsland entries.
    const ExtrusionRole bucket_role = bucket_role_from_raw(role);
    if (!region_island->has_extrusion(bucket_role))
        return nullptr;

    ExtrusionEntityCollection &collection = region_island->mutable_extrusion(bucket_role);
    return reinterpret_cast<extrusion_entity_handle *>(&collection);
}

int32_t set_island_fill_areas_callback(const layer_island_handle *island_handle,
                                       const expolygon_collection_handle *areas_handle)
{
    LayerSliceIsland *island = to_layer_island(island_handle);
    if (island == nullptr)
        return 0;

    ExPolygons areas = areas_handle == nullptr ? ExPolygons{} : *to_expolygons(areas_handle);
    ensure_valid(areas);
    ApiInternal::LayerIslandAccess::set_infill_areas(*island, std::move(areas));
    return 1;
}

int32_t set_island_fill_free_areas_callback(const layer_island_handle *island_handle,
                                            const expolygon_collection_handle *areas_handle)
{
    LayerSliceIsland *island = to_layer_island(island_handle);
    if (island == nullptr)
        return 0;

    ExPolygons areas = areas_handle == nullptr ? ExPolygons{} : *to_expolygons(areas_handle);
    ensure_valid(areas);
    ApiInternal::LayerIslandAccess::infill_free_areas_mutable(*island) = std::move(areas);
    return 1;
}

} // namespace

void clear(Print &) {}
void prepare(Print &) {}

bool validate_pre(const Print &, std::string &)
{
    return true;
}

bool validate_post(const Print &, std::string &)
{
    return true;
}

void run_step(Orchestrator &orchestrator, Print &print)
{
    Detail::run_object_step_plugins(orchestrator,
                                    print,
                                    STEP_POST_PERIMETER,
                                    &print.full_print_config(),
                                    print.objects().size(),
                                    [&print](const size_t object_idx) {
        run_ctx_post_perimeter_generation context = {};
        context.print = reinterpret_cast<const print_handle *>(&print);
        context.object = reinterpret_cast<const object_handle *>(&print.object(object_idx));
        context.get_region_island_mutable_extrusion = &get_region_island_mutable_extrusion_callback;
        context.set_island_fill_areas = &set_island_fill_areas_callback;
        context.set_island_fill_free_areas = &set_island_fill_free_areas_callback;
        return context;
    });
}

} // namespace Slic3r::Steps::StepPostPerimeterGeneration
