///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "OverhangSlopeFill.hpp"

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <iterator>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_post_slicing.h"
#include "libslic3r/Api/plugin/cpp/PrintHelpers.hpp"
#include "libslic3r/Api/plugin/cpp/Views.hpp"

/*
Overhang slope fill
===================

This STEP_POST_SLICING plugin adds material under overhangs so that each layer
hangs at most `overhangs_fill_slope` over the layer below. MaxOverhangThreshold
enforces the same kind of contract by removing the unsupported part of an
overhang; this plugin enforces it by building support from below instead. It
runs first (priority -100 versus 0), so with a fill slope lower than the max
slope, the cut pass finds nothing left to remove where the fill succeeded.

Contract for one layer L and the layer above it (upper):

    added(L) = diff(offset(upper, -slope), slices(L) + bridgeable(upper))

Every point of upper outside the bridgeable areas is then within `slope` of
layer L. The added area always stays inside the footprint of upper.

Layers are processed from the top to the bottom. The geometry added on L is
part of `upper` when L-1 is processed, so the fill propagates down until it
merges with the object or vanishes, forming an inverted cone under a floating
overhang. Bridgeable overhangs are never filled; as the fill never enters
them, the layers below them do not see anything to support either. This is
why no "upper layers" look-ahead is needed here, unlike MaxOverhangThreshold.

The normal call flow is:

    OverhangSlopeFill::inilialize_impl()
    `-- create plugin settings, UI line and GUI rule

    OverhangSlopeFill::run_impl()
    |-- no region enables overhangs_fill_slope? finish unchanged
    `-- for each layer L from the second-to-last down to the first
        `-- process_layer()
            |-- enabled_region_slopes(): regions of L with a slope > 0
            |-- filtered_overhangs(): upper minus L, without tiny noise
            |-- for each enabled region r
            |   |-- bridgeable_overhangs(): overhang areas r accepts as bridges
            |   |-- pieces = added(L) computed with the slope of r
            |   `-- keep a piece for r when touched_region() says it touches
            |       only r, or when it touches zero/several regions and r is
            |       the default region (default_region_idx())
            `-- publish_additions(): union the kept pieces into their regions
                and rebuild the layer slices and islands

Region ownership of the added material
--------------------------------------
LayerRegion slices only contain printed material: a modifier volume is clipped
to its parent part during slicing, so the empty space under an overhang has no
region. The v1 heuristic therefore reads the slope of the region the piece
grows from: a piece touching exactly one region extends that region, and
other pieces (floating, or touching several regions) go to the default region
of the layer range, i.e. the model part without any modifier.

Accepted limitations (v1):
- Overhang parts narrower than twice the slope vanish in the erosion and are
  not filled. Supports generated later are expected to hold them.
- A modifier carrying the setting does not stop the fill exactly at its
  boundary: a piece touching material inside the modifier may extend outside
  of it laterally.

Planned v2: slice the modifier meshes at each layer Z (object slicing layer
ranges + Volume::mesh().slice_to_expolygons()) without clipping them to their
parent part. These masks give the region domain in empty space, so the added
material can be assigned to the modifier region exactly and stop at its
boundary.
*/

namespace slic3r_api { namespace OverhangSlopeFillPlugin {

namespace {

const char *k_overhang_slope_fill_id = "overhang_slope_fill";
const char *k_no_dependencies[] = {nullptr};
const char *k_slope_key = "overhangs_fill_slope";
const char *k_bridge_threshold_key = "overhangs_fill_bridge_threshold";
const raw_used_config_key k_used_config_keys[] = {
    { k_slope_key, RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT },
    { k_bridge_threshold_key, RAW_CO_FLOAT, RAW_CONTAINER_TYPE_REGION, RAW_PRESET_TYPE_FFF_PRINT },
    { "bridge_precision", RAW_CO_FLOAT_OR_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "nozzle_diameter", RAW_CO_VECTOR_FLOAT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE }
};
constexpr size_t k_defined_config_key_count = 2;

// touched_region() results that are not a region index.
constexpr int32_t k_no_region = -1;
constexpr int32_t k_several_regions = -2;

// One region of the current layer that enables the fill, with its slope
// already converted to a scaled distance.
struct RegionSlope
{
    uint32_t region_idx = 0;
    coord_t distance = 0;
};

// Number of progress units: every layer except the top one can receive
// material.
uint32_t layer_work_count(const Object &object);

// Largest nozzle diameter used by the object, the reference of percent
// slopes. Falls back to 0.4 mm when no extruder is known.
coord_t max_nozzle_diameter(const Print &print, const Object &object, const plugin_run_context *run_ctx);

// Return true when at least one print region of the object enables the fill,
// so the whole pass can be skipped for objects that do not use it.
bool object_has_enabled_slope(const Object &object, coord_t max_nz_diam);

// Add material to layer `layer_idx` under the overhangs of the layer above,
// then publish it. Does nothing when no region of the layer enables the fill.
void process_layer(storage_handle *storage,
                   const run_ctx_post_slicing &ctx,
                   const plugin_run_context *run_ctx,
                   const Print &print,
                   const Object &object,
                   uint32_t layer_idx,
                   coord_t max_nz_diam);

// Regions of `layer` whose slope is > 0, in region order.
std::vector<RegionSlope> enabled_region_slopes(const Layer &layer, coord_t max_nz_diam);

// Areas of `upper` not resting on `layer`. Tiny islands are removed because
// they are noise for the bridge detector.
StoredExPolygonCollection filtered_overhangs(storage_handle *storage,
                                             const Layer &layer,
                                             const Layer &upper,
                                             coord_t max_nz_diam,
                                             const plugin_run_context *run_ctx);

// Region of the model part without modifier in the layer range containing
// slice_z. This region receives the pieces without a unique neighbour region.
uint32_t default_region_idx(const Object &object, coord_t slice_z);

// Union of the bridge coverage of every overhang, using the bridge flow and
// the bridge length limit of one region. Empty when the region sets the limit
// to 0, which means "fill under bridges too".
ClipperOperand bridgeable_overhangs(storage_handle *storage,
                                    const Print &print,
                                    const Layer &layer,
                                    const LayerRegion &region,
                                    const StoredExPolygonCollection &overhangs,
                                    uint32_t upper_layer_idx,
                                    const plugin_run_context *run_ctx);

// Index of the only region of `layer` touched by `piece`, or k_no_region /
// k_several_regions.
int32_t touched_region(storage_handle *storage, const Layer &layer, const ExPolygon &piece);

// Merge the kept pieces into their regions and rebuild the layer caches.
// Pieces already claimed by a lower region index are removed first, so raw
// region slices stay disjoint.
void publish_additions(storage_handle *storage,
                       const run_ctx_post_slicing &ctx,
                       const plugin_run_context *run_ctx,
                       const Layer &layer,
                       uint32_t layer_idx,
                       const std::vector<StoredExPolygonCollection> &additions);

uint32_t layer_work_count(const Object &object)
{
    const uint32_t layer_count = object.layer_count();
    return layer_count > 1 ? layer_count - 1 : 0;
}

coord_t max_nozzle_diameter(const Print &print, const Object &object, const plugin_run_context *run_ctx)
{
    coord_t max_nz_diam = 0;
    for (extruder_index_t extr_id : object_extruders(print, object)) {
        throw_if_cancelled(run_ctx);
        max_nz_diam = std::max(max_nz_diam, scale_i(print.config().get("nozzle_diameter").get_float(extr_id)));
    }
    return max_nz_diam == 0 ? scale_i(0.4) : max_nz_diam;
}

bool object_has_enabled_slope(const Object &object, const coord_t max_nz_diam)
{
    for (uint32_t region_idx = 0; region_idx < object.print_region_count(); ++region_idx) {
        const Config config = object.print_region(region_idx).config();
        if (scale_i(config.get(k_slope_key).get_effective_value(unscaled(max_nz_diam))) > 0)
            return true;
    }
    return false;
}

void process_layer(storage_handle *storage,
                   const run_ctx_post_slicing &ctx,
                   const plugin_run_context *run_ctx,
                   const Print &print,
                   const Object &object,
                   const uint32_t layer_idx,
                   const coord_t max_nz_diam)
{
    const Layer layer = object.layer(layer_idx);
    const Layer upper = object.layer(layer_idx + 1);
    const std::vector<RegionSlope> slopes = enabled_region_slopes(layer, max_nz_diam);
    if (slopes.empty())
        return;

    throw_if_cancelled(run_ctx);
    const StoredExPolygonCollection overhangs = filtered_overhangs(storage, layer, upper, max_nz_diam, run_ctx);
    if (overhangs.empty())
        return;

    ClipperContext clipper(storage);
    const uint32_t default_region = default_region_idx(object, layer.slice_z());
    std::vector<StoredExPolygonCollection> additions;
    additions.reserve(layer.region_count());
    for (uint32_t region_idx = 0; region_idx < layer.region_count(); ++region_idx)
        additions.emplace_back(storage);

    // Each enabled region computes the pieces with its own slope and bridge
    // limit, but only keeps the pieces it owns. A piece computed with a larger
    // slope is contained in the piece computed with a smaller one, so a piece
    // touching only region r cannot be kept by another region with the same
    // unique-touch rule.
    for (const RegionSlope &slope : slopes) {
        throw_if_cancelled(run_ctx);
        const LayerRegion region = layer.region(slope.region_idx);
        ClipperOperand bridged = bridgeable_overhangs(storage, print, layer, region, overhangs, layer_idx + 1,
                                                      run_ctx);
        throw_if_cancelled(run_ctx);
        ClipperOperand blocked = clipper_union2(clipper(layer.slices()), bridged);
        throw_if_cancelled(run_ctx);
        ClipperOperand eroded_upper = clipper_offset(clipper(upper.slices()), -double(slope.distance));
        throw_if_cancelled(run_ctx);
        // The safety offset absorbs slivers along the boundary of L; the gap it
        // leaves is closed again by the union in publish_additions().
        const StoredExPolygonCollection pieces = clipper_diff_with_safety_offset(eroded_upper, blocked)
                                                     .to_expolygon_collection();
        for (ExPolygon piece : pieces) {
            throw_if_cancelled(run_ctx);
            const int32_t owner = touched_region(storage, layer, piece);
            if (owner == int32_t(slope.region_idx) || (owner < 0 && slope.region_idx == default_region))
                additions[slope.region_idx].push_back(piece);
        }
    }

    throw_if_cancelled(run_ctx);
    publish_additions(storage, ctx, run_ctx, layer, layer_idx, additions);
}

std::vector<RegionSlope> enabled_region_slopes(const Layer &layer, const coord_t max_nz_diam)
{
    std::vector<RegionSlope> slopes;
    for (uint32_t region_idx = 0; region_idx < layer.region_count(); ++region_idx) {
        const Config config = layer.region(region_idx).print_region().config();
        const coord_t distance = scale_i(config.get(k_slope_key).get_effective_value(unscaled(max_nz_diam)));
        if (distance > 0)
            slopes.push_back(RegionSlope{region_idx, distance});
    }
    return slopes;
}

StoredExPolygonCollection filtered_overhangs(storage_handle *storage,
                                             const Layer &layer,
                                             const Layer &upper,
                                             const coord_t max_nz_diam,
                                             const plugin_run_context *run_ctx)
{
    ClipperContext clipper(storage);
    ClipperOperand overhangs = clipper_diff_with_safety_offset(clipper(upper.slices()), clipper(layer.slices()));
    throw_if_cancelled(run_ctx);
    if (overhangs.empty())
        return overhangs.to_expolygon_collection();
    // Same noise filter as MaxOverhangThreshold: islands thinner than half a
    // nozzle disappear in the opening, the rest is restored to its exact shape
    // by the intersection.
    ClipperOperand opened = clipper_offset2(overhangs, double(-max_nz_diam / 2), double(max_nz_diam),
                                            CLIPPER_JOIN_MITER, 5.);
    throw_if_cancelled(run_ctx);
    StoredExPolygonCollection out = clipper_intersection(overhangs, opened).to_expolygon_collection();
    throw_if_cancelled(run_ctx);
    return out;
}

uint32_t default_region_idx(const Object &object, const coord_t slice_z)
{
    // Find the layer range containing slice_z. Shared boundaries belong to the
    // upper range, as in STEP_SLICING.
    const uint32_t range_count = object_slicing_layer_range_count(object);
    int32_t selected_range = -1;
    for (uint32_t range_idx = 0; range_idx < range_count; ++range_idx) {
        const SlicingLayerRange range = object_slicing_layer_range(object, range_idx);
        if (range.z_min() <= slice_z && slice_z <= range.z_max()) {
            selected_range = int32_t(range_idx);
            if (slice_z < range.z_max())
                break;
        }
    }
    if (selected_range < 0)
        return 0;

    // Model parts have no parent entry; modifiers point to the part they
    // override. The first model part is the region "without any modifier".
    const SlicingLayerRange range = object_slicing_layer_range(object, uint32_t(selected_range));
    for (uint32_t volume_region_idx = 0; volume_region_idx < range.volume_region_count(); ++volume_region_idx) {
        const SlicingVolumeRegion volume_region = range.volume_region(volume_region_idx);
        if (volume_region.parent() >= 0 || slicing_volume_region_get_volume(volume_region.handle()) == nullptr)
            continue;
        if (volume_region.volume().is_model_part() && volume_region.layer_region_idx() >= 0)
            return uint32_t(volume_region.layer_region_idx());
    }
    return 0;
}

ClipperOperand bridgeable_overhangs(storage_handle *storage,
                                    const Print &print,
                                    const Layer &layer,
                                    const LayerRegion &region,
                                    const StoredExPolygonCollection &overhangs,
                                    const uint32_t upper_layer_idx,
                                    const plugin_run_context *run_ctx)
{
    ClipperContext clipper(storage);
    ClipperOperand bridged = clipper.empty();
    const ConfigOption bridge_threshold = region.print_region().config().get(k_bridge_threshold_key);
    // An enabled limit of 0 means bridges are filled like any other overhang.
    // A disabled option means "accept every bridge length".
    if (bridge_threshold.is_enabled() && bridge_threshold.get_float() == 0)
        return bridged;

    const c_flow bridge_flow = region.flow(RAW_EXTRUSION_ROLE_INFILL | RAW_EXTRUSION_ROLE_SOLID |
                                           RAW_EXTRUSION_ROLE_BRIDGE);
    const coord_t precision = scale_i(
        print.config().get("bridge_precision").get_effective_value(unscaled(bridge_flow.spacing)));
    for (ExPolygon overhang : overhangs) {
        throw_if_cancelled(run_ctx);
        // The bridge detector is a service plugin. The C++ wrapper owns the
        // service instance only; the input geometry stays owned by this plugin
        // and by the host layer.
        bridge_detector_create_input input = {};
        input.expolygon = overhang.handle();
        input.lower_slices = layer.slices().handle();
        input.spacing = bridge_flow.spacing;
        input.precision = precision;
        input.layer_id = upper_layer_idx;
        BridgeDetector detector(orchestrator_create_bridge_detector(nullptr, &input));
        detector.set_max_bridge_length(bridge_threshold.is_enabled() ?
                                           scale_d(std::max(0., bridge_threshold.get_float())) :
                                           -1.);
        throw_if_cancelled(run_ctx);
        if (!detector.detect_angle())
            continue;
        StoredPolygonCollection coverage(storage);
        throw_if_cancelled(run_ctx);
        if (detector.coverage(coverage) > 0) {
            bridged += clipper_union(clipper(coverage));
            throw_if_cancelled(run_ctx);
        }
    }
    return clipper_union(bridged);
}

int32_t touched_region(storage_handle *storage, const Layer &layer, const ExPolygon &piece)
{
    // Pieces are separated from L by the safety offset of their diff, so the
    // contact test grows them by a distance well above that offset but far
    // below any printable width.
    const coord_t touch_distance = scale_i(0.01);
    ClipperContext clipper(storage);
    ClipperOperand grown_piece = clipper_offset(clipper(piece), double(touch_distance));

    // LayerRegion::bounding_box() describes the typed surfaces, which do not
    // exist yet at post-slicing, so it cannot be used to reject regions early.
    int32_t touched = k_no_region;
    for (uint32_t region_idx = 0; region_idx < layer.region_count(); ++region_idx) {
        const ExPolygonCollection slices = layer.region(region_idx).slices();
        if (slices.empty())
            continue;
        if (clipper_intersection(grown_piece, clipper(slices)).empty())
            continue;
        if (touched != k_no_region)
            return k_several_regions;
        touched = int32_t(region_idx);
    }
    return touched;
}

void publish_additions(storage_handle *storage,
                       const run_ctx_post_slicing &ctx,
                       const plugin_run_context *run_ctx,
                       const Layer &layer,
                       const uint32_t layer_idx,
                       const std::vector<StoredExPolygonCollection> &additions)
{
    // Build every replacement before touching the layer: publishing rebuilds
    // the layer, which invalidates the views used to compute the replacements.
    ClipperContext clipper(storage);
    ClipperOperand claimed = clipper.empty();
    std::vector<uint32_t> changed_region_indices;
    std::vector<StoredExPolygonCollection> replacement_region_slices;
    for (uint32_t region_idx = 0; region_idx < additions.size(); ++region_idx) {
        if (additions[region_idx].empty())
            continue;
        throw_if_cancelled(run_ctx);
        // A fallback piece of the default region may contain a piece kept by
        // another region; the first claim wins.
        ClipperOperand added = clipper_diff(clipper(additions[region_idx]), claimed);
        throw_if_cancelled(run_ctx);
        if (added.empty())
            continue;
        ClipperOperand merged = clipper(layer.region(region_idx).slices());
        merged += added;
        replacement_region_slices.push_back(clipper_union_with_safety_offset(merged).to_expolygon_collection());
        changed_region_indices.push_back(region_idx);
        throw_if_cancelled(run_ctx);
        claimed = clipper_union2(claimed, added);
        throw_if_cancelled(run_ctx);
    }
    if (changed_region_indices.empty())
        return;

    // Commit every replacement and rebuild the layer-level caches without a
    // cancellation point between those consistency updates.
    layer_handle *mut_layer = ctx.object_borrow_mutable_layer(ctx.object, layer_idx);
    for (size_t replacement_idx = 0; replacement_idx < changed_region_indices.size(); ++replacement_idx) {
        layer_region_handle *mut_region = layer_get_region_mutable(mut_layer,
                                                                    changed_region_indices[replacement_idx]);
        expolygon_collection_handle *mut_slices = ctx.layer_region_borrow_mutable_slices(mut_region);
        expolygons_move(mut_slices, replacement_region_slices[replacement_idx].mutable_handle());
    }
    ctx.layer_recompute_slices_and_islands_from_layer_region(mut_layer);
}

} // namespace

void OverhangSlopeFill::run_impl(const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    const run_ctx_post_slicing *ctx = plugin_ctx_as_post_slicing(run_ctx);
    if (ctx == nullptr || ctx->print == nullptr || ctx->object == nullptr)
        return;

    const Print print(ctx->print);
    const Object object(ctx->object);
    storage_handle *storage = run_ctx->plugin_storage;
    const coord_t max_nz_diam = max_nozzle_diameter(print, object, run_ctx);
    if (!object_has_enabled_slope(object, max_nz_diam)) {
        progress().increment_by(static_cast<int32_t>(layer_work_count(object)));
        progress().finish_run();
        return;
    }

    // Top-down: the material added on a layer must itself be supported by the
    // layer below, which is processed next.
    for (uint32_t layer_idx = layer_work_count(object); layer_idx > 0; --layer_idx) {
        throw_if_cancelled(run_ctx);
        process_layer(storage, *ctx, run_ctx, print, object, layer_idx - 1, max_nz_diam);
        progress().increment();
    }
    progress().finish_run();

    assert(storage_size(storage) == 0);
    storage_clear(storage);
}

OverhangSlopeFill &OverhangSlopeFill::instance(orchestrator_handle *orch)
{
    static OverhangSlopeFill s_instance(orch);
    return s_instance;
}

const char *OverhangSlopeFill::id_impl() const noexcept { return k_overhang_slope_fill_id; }

const char *OverhangSlopeFill::name_impl() const noexcept { return "Overhang slope fill"; }

const char *OverhangSlopeFill::description_impl() const noexcept
{
    return "Add material under overhangs so that each layer respects a maximum overhang slope.";
}

slicing_step_t OverhangSlopeFill::step_impl() const noexcept { return STEP_POST_SLICING; }

const char *const *OverhangSlopeFill::dependencies_impl() const noexcept { return k_no_dependencies; }

// Runs before MaxOverhangThreshold (priority 0), so the cut pass sees the
// filled geometry, and before VaseMultiIslandConnector (priority 1000).
int32_t OverhangSlopeFill::priority_impl() const noexcept { return -100; }

int32_t OverhangSlopeFill::used_config_keys(raw_used_config_key *keys) const noexcept
{
    if (keys != nullptr)
        std::copy(std::begin(k_used_config_keys), std::end(k_used_config_keys), keys);
    return int32_t(std::size(k_used_config_keys));
}

int32_t OverhangSlopeFill::defined_config_keys(const char **keys) const noexcept
{
    if (keys != nullptr)
        for (size_t idx = 0; idx < k_defined_config_key_count; ++idx)
            keys[idx] = k_used_config_keys[idx].key;
    return int32_t(k_defined_config_key_count);
}

const char *OverhangSlopeFill::progress_message_format_impl() const noexcept
{
    return "Overhang slope fill: %u / %u layers";
}

const char *OverhangSlopeFill::print_ui_fragment() noexcept
{
    return "page:Slicing\n"
           "group:Modifying slices\n"
           "line:Overhangs fill\n"
           "setting:overhangs_fill_slope\n"
           "setting:overhangs_fill_bridge_threshold\n"
           "end_line\n";
}

void OverhangSlopeFill::inilialize_impl(storage_handle *) const
{
    raw_config_option_def def = raw_config_option_def_init();
    def.opt_key = k_slope_key;
    def.type = RAW_CO_FLOAT_OR_PERCENT;
    def.container_type = RAW_CONTAINER_TYPE_REGION;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.label = "Overhangs fill slope";
    def.full_label = "Overhangs fill slope";
    def.category = RAW_OPTION_CATEGORY_SLICING;
    def.invalidates_step = STEP_SLICING;
    def.tooltip = ("Maximum overhang distance per layer. Where a layer hangs by more than this value over the layer"
                   " below, material is added below it, down to the bed or to the object, so the overhang is"
                   " supported by the part itself."
                   " Detected bridgeable areas are not filled if 'overhangs_fill_bridge_threshold' allows it."
                   "\nThe added material uses the settings of the region it grows from."
                   "\nCan be a % of the highest nozzle diameter."
                   "\nSet to 0 to disable.");
    def.sidetext = "mm or %";
    def.ratio_over = "nozzle_diameter";
    def.has_min = true;
    def.min_value = 0;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "0";
    orchestrator_create_option_def(m_orchestrator, &def);

    def = raw_config_option_def_init();
    def.opt_key = k_bridge_threshold_key;
    def.type = RAW_CO_FLOAT;
    def.container_type = RAW_CONTAINER_TYPE_REGION;
    def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINT;
    def.printer_technology = RAW_PT_FFF;
    def.label = "Bridge max length";
    def.full_label = "Overhangs fill bridge max length";
    def.category = RAW_OPTION_CATEGORY_SLICING;
    def.invalidates_step = STEP_SLICING;
    def.tooltip = ("Maximum distance for bridges. Bridgeable areas shorter than this are not filled by"
                   " 'overhangs_fill_slope'."
                   "\nIf disabled, accept all distances."
                   "\nSet to 0 to ignore bridges and fill them too.");
    def.sidetext = "mm";
    def.has_min = true;
    def.min_value = 0;
    def.can_be_disabled = true;
    def.mode = RAW_CONFIG_OPTION_MODE_EXPERT | RAW_CONFIG_OPTION_MODE_SUSI;
    def.default_serialized_value = "!0";
    orchestrator_create_option_def(m_orchestrator, &def);

    orchestrator_add_ui_fragment(m_orchestrator,
                                 "print.ui",
                                 k_overhang_slope_fill_id,
                                 OverhangSlopeFill::print_ui_fragment(),
                                 1);

    GuiRule::activate({ k_bridge_threshold_key })
        .when_non_zero(k_slope_key)
        .add_to(m_orchestrator);
}

void OverhangSlopeFill::setup_run_impl(const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    const run_ctx_post_slicing *ctx = plugin_ctx_as_post_slicing(run_ctx);
    if (ctx == nullptr || ctx->object == nullptr)
        return;

    progress().add_max(layer_work_count(Object(ctx->object)));
}

void register_overhang_slope_fill_plugin(orchestrator_handle *orch)
{
    orchestrator_register_plugin(orch, OverhangSlopeFill::instance(orch).c_instance());
}

}} // namespace slic3r_api::OverhangSlopeFillPlugin
