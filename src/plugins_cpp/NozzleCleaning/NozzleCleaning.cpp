///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Per-tool nozzle-cleaning plan editor
====================================

This STEP_EXTRUSION_EDIT plugin scans the final PrintingPlan in firmware order.
Each tool owns fresh layer, volume, retraction and tool-return counters for one
run. A threshold only makes cleaning due; the selected transition mode decides
which later plan boundary may consume that due state. Scripts are stored as
ordinary extrusion custom-G-code properties so the selected firmware processes
placeholders and restores its normal state tracking.

Generated scripts carry a namespaced dynamic marker. The next run removes only
marked insertions and unwraps their ordered leaves before rebuilding counters,
which keeps repeated pipeline execution idempotent without retaining state in
the plugin singleton.
*/

#include "NozzleCleaning.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_config_def.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_extrusion_edit.h"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionTreeVisitors.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingExtrusionScopeViews.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Api/plugin/cpp/properties/ExtrusionProperties.hpp"
#include "libslic3r/Api/plugin/cpp/properties/PrintingExtrusionScopeProperty.hpp"

namespace slic3r_api { namespace NozzleCleaningPlugin {
namespace {

const char *const k_no_dependencies[] = { nullptr };
const char *const k_generated_property_name =
    "superslicer.nozzle_cleaning.generated";

const raw_used_config_key k_used_config_keys[] = {
    { "nozzle_cleaning_gcode", RAW_CO_STRING, RAW_CONTAINER_TYPE_PROJECT, RAW_PRESET_TYPE_FFF_PRINTER },
    { "nozzle_cleaning_interval_layers", RAW_CO_INT, RAW_CONTAINER_TYPE_PROJECT, RAW_PRESET_TYPE_FFF_PRINTER },
    { "nozzle_cleaning_interval_volume", RAW_CO_FLOAT, RAW_CONTAINER_TYPE_PROJECT, RAW_PRESET_TYPE_FFF_PRINTER },
    { "nozzle_cleaning_interval_retractions", RAW_CO_INT, RAW_CONTAINER_TYPE_PROJECT, RAW_PRESET_TYPE_FFF_PRINTER },
    { "nozzle_cleaning_interval_toolchanges", RAW_CO_INT, RAW_CONTAINER_TYPE_PROJECT, RAW_PRESET_TYPE_FFF_PRINTER },
    { "nozzle_cleaning_when", RAW_CO_ENUM, RAW_CONTAINER_TYPE_PROJECT, RAW_PRESET_TYPE_FFF_PRINTER }
};

const char *const k_defined_config_keys[] = {
    "nozzle_cleaning_gcode",
    "nozzle_cleaning_interval_layers",
    "nozzle_cleaning_interval_volume",
    "nozzle_cleaning_interval_retractions",
    "nozzle_cleaning_interval_toolchanges",
    "nozzle_cleaning_when"
};

// Values are serialized in the printer preset in this order. Each later value
// includes the occasions of the earlier modes.
enum class CleaningOpportunity : int32_t
{
    Toolchange = 0,
    Object = 1,
    Island = 2,
    Retraction = 3
};

struct CleaningSettings
{
    // One shared configuration is copied here; runtime counters are kept per tool.
    std::string script;
    int64_t interval_layers = 0;
    double interval_volume = 0.0;
    int64_t interval_retractions = 0;
    int64_t interval_toolchanges = 0;
    CleaningOpportunity when = CleaningOpportunity::Retraction;
};

struct ToolCounters
{
    // A tool's counters start only when that tool first appears in plan order.
    bool initialized = false;
    // Remaining work before each independently enabled trigger becomes due.
    int64_t layers_remaining = 0;
    double volume_remaining = 0.0;
    int64_t retractions_remaining = 0;
    int64_t toolchanges_remaining = 0;
    // A return may be noticed before the next printable tree gives us a safe
    // place to store its script. Keep that occasion pending until then.
    bool pending_toolchange_cleaning = false;
    // These two flags describe physical E-axis state, not interval counters.
    // They survive cleaning so partial retract commands still form one cycle.
    bool retracted = false;
    bool retraction_cycle_pending = false;
    // True only after this tool deposited material since the last clean in this
    // layer. The layer interval consumes this once, when the layer group ends.
    bool extruded_in_current_layer = false;
};

/* Island identity must include both the printed object instance and geometry:
   distinct objects can reuse the same source LayerIsland handles. */
struct IslandIdentity
{
    uint32_t object_instance_id = SLIC3R_PRINTING_OBJECT_INSTANCE_ID_NONE;
    const layer_island_handle *island = nullptr;

    bool operator==(const IslandIdentity &other) const
    {
        return object_instance_id == other.object_instance_id && island == other.island;
    }
};

struct CleaningInsertion
{
    // The script either wraps a specific execution node or follows all existing
    // events in a reserved scope phase when no Unretract node exists.
    enum class Position
    {
        BeforeEntity,
        AppendToPhase
    };

    MutableExtrusionEntity anchor;
    Position position = Position::BeforeEntity;
    extruder_index_t extruder_id = EXTRUDER_ID_UNKNOWN;
};

struct ToolEventInsertion
{
    // Empty tool visits have no extrusion tree, so their script is attached to
    // the tool group's before-events, after any tool-selection event.
    printing_tool_group_handle *tool_group = nullptr;
    extruder_index_t extruder_id = EXTRUDER_ID_UNKNOWN;
};

struct EntryOpportunity
{
    // An allowed object/island/tool transition was observed for this tree.
    bool active = false;
    // Due was already true at the transition. Keep this snapshot while the
    // visitor walks travel/retraction events to the safe insertion point.
    bool due_when_opened = false;
    CleaningInsertion insertion;
};

using IslandOwnerMap = std::unordered_map<
    const layer_region_island_handle *, const layer_island_handle *>;

/* Read printer-preset values and normalize malformed numeric profiles safely. */
CleaningSettings load_settings(const Config &config);

/* Restore plugin-owned extrusion trees before examining the current plan. */
void remove_generated_insertions(
    PrintingPlan &plan,
    const PluginPropertyKey<NozzleCleaningGeneratedProperty> &generated_key);

/* Recursively remove generated leaves and report whether this root is a wrapper. */
bool clean_generated_tree(
    MutableExtrusionEntity entity,
    const PluginPropertyKey<NozzleCleaningGeneratedProperty> &generated_key);

/* Recognize the two-child wrapper produced by emplace_ordered_leaf(). */
bool is_generated_wrapper(
    const ExtrusionEntity &entity,
    const PluginPropertyKey<NozzleCleaningGeneratedProperty> &generated_key,
    uint32_t *generated_child_index);

/* Move a wrapper's original content back into its parent without cloning it. */
void unwrap_generated_child(
    MutableExtrusionEntity parent,
    uint32_t wrapper_index,
    MutableExtrusionEntity wrapper,
    const PluginPropertyKey<NozzleCleaningGeneratedProperty> &generated_key);

/* Build a lookup from each region-specific island to its physical LayerIsland. */
IslandOwnerMap island_owners_for_layer_group(const PrintingLayerGroup &layer_group,
                                              const plugin_run_context *run_ctx);

/* Find the first prepared scope in one extrusion tree. */
MutableExtrusionEntity first_scope_in_tree(
    MutableExtrusionEntity entity,
    const PluginPropertyKey<PrintingExtrusionScopeProperty> &scope_key,
    const plugin_run_context *run_ctx);

/* Find one semantic Unretract in the named before phase. */
MutableExtrusionEntity first_unretract_in_tree(
    MutableExtrusionEntity entity,
    const plugin_run_context *run_ctx);

/* Find the earliest depositing leaf used when no incoming scope phase exists. */
MutableExtrusionEntity first_printable_leaf(
    MutableExtrusionEntity entity,
    const plugin_run_context *run_ctx);

/* Create a transition anchor that runs after start-filament and before resumption. */
EntryOpportunity make_entry_opportunity(
    MutableExtrusionEntity extrusion_root,
    extruder_index_t extruder_id,
    const PluginPropertyKey<PrintingExtrusionScopeProperty> &scope_key,
    bool due_when_opened,
    const plugin_run_context *run_ctx);

/* Store one script on an entity or append a standalone event to a phase. */
void apply_insertion(
    storage_handle *storage,
    const CleaningInsertion &insertion,
    const CleaningSettings &settings,
    const PluginPropertyKey<NozzleCleaningGeneratedProperty> &generated_key);

/* Store a deferred cleaning on an empty tool visit after its selection event. */
void apply_tool_event_insertion(
    storage_handle *storage,
    const ToolEventInsertion &insertion,
    const CleaningSettings &settings,
    const PluginPropertyKey<NozzleCleaningGeneratedProperty> &generated_key);

/* Detect a previous generated event without treating user scripts as owned. */
bool has_generated_tool_event(
    const PrintingScopeEvents &events,
    extruder_index_t extruder_id,
    const PluginPropertyKey<NozzleCleaningGeneratedProperty> &generated_key);

/* Search one event tree for this plugin's script for the selected tool. */
bool entity_has_generated_tool_script(
    const ExtrusionEntity &entity,
    extruder_index_t extruder_id,
    const PluginPropertyKey<NozzleCleaningGeneratedProperty> &generated_key);

/* Enable transition types allowed by the configured hierarchy. */
bool allows_toolchange(CleaningOpportunity when);
bool allows_object(CleaningOpportunity when);
bool allows_island(CleaningOpportunity when);

/* Initialize one tool's independent counters on its first activation. */
void initialize_tool(ToolCounters &tool, const CleaningSettings &settings);

/* Test whether at least one enabled trigger has expired; configured zeroes are
   disabled and therefore never make the tool due. */
bool cleaning_is_due(const ToolCounters &tool, const CleaningSettings &settings);

/* Reload every interval and clear this layer's post-clean extrusion marker.
   Preserve retract state because cleaning G-code does not itself restore E. */
void reset_tool_counters_after_cleaning(ToolCounters &tool,
                                        const CleaningSettings &settings);

/* Decrement a signed interval without allowing an integer underflow. */
void decrement_interval(int64_t &remaining);

/* Process all groups, layers, tools and extrusion roots in serializer order. */
void process_plan(
    const plugin_run_context *run_ctx,
    const CleaningSettings &settings,
    PrintingPlan &plan,
    storage_handle *storage,
    const PluginPropertyKey<PrintingExtrusionScopeProperty> &scope_key,
    const PluginPropertyKey<NozzleCleaningGeneratedProperty> &generated_key);

/* Count material, retraction transitions, and transition opportunities in one tree. */
class CleaningTreeVisitor final : public ExtrusionTreeVisitor<>
{
public:
    CleaningTreeVisitor(const CleaningSettings &settings,
                        ToolCounters &tool,
                        extruder_index_t extruder_id,
                        EntryOpportunity &entry_opportunity,
                        std::vector<CleaningInsertion> &insertions,
                        const plugin_run_context *run_ctx);

protected:
    void enter_node(MutableExtrusionEntity entity) override;
    void visit_leaf(MutableExtrusionEntity entity) override;
    void leave_node(MutableExtrusionEntity entity) override;

private:
    /* Resolve a due entry transition or completed retract cycle at one node. */
    void process_unretract(MutableExtrusionEntity entity,
                           bool completed_retraction_cycle);

    /* Store one before-entity insertion and reset exactly this tool's counters. */
    void clean_before_entity(MutableExtrusionEntity entity);

    /* Store one after-phase insertion when a transition has no Unretract node. */
    void clean_after_phase(MutableExtrusionEntity phase);

    const CleaningSettings &m_settings;
    ToolCounters &m_tool;
    extruder_index_t m_extruder_id;
    EntryOpportunity &m_entry_opportunity;
    std::vector<CleaningInsertion> &m_insertions;
    const plugin_run_context *m_run_ctx;
};

CleaningSettings load_settings(const Config &config)
{
    CleaningSettings settings;
    // Profiles are hand-editable, so clamp negative counters and reject a
    // non-finite volume instead of allowing it to make cleaning never due.
    settings.script = config.string_or_default("nozzle_cleaning_gcode", std::string());
    settings.interval_layers = std::max<int64_t>(0,
        config.int_or_default("nozzle_cleaning_interval_layers", 0));
    settings.interval_retractions = std::max<int64_t>(0,
        config.int_or_default("nozzle_cleaning_interval_retractions", 0));
    settings.interval_toolchanges = std::max<int64_t>(0,
        config.int_or_default("nozzle_cleaning_interval_toolchanges", 0));
    const double volume = config.float_or_default("nozzle_cleaning_interval_volume", 0.0);
    settings.interval_volume = std::isfinite(volume) ? std::max(0.0, volume) : 0.0;
    const int32_t when = config.enum_or_default(
        "nozzle_cleaning_when", int32_t(CleaningOpportunity::Retraction));
    // An unknown serialized enum value falls back to the safest, most flexible
    // opportunity mode rather than disabling all cleaning opportunities.
    if (when >= int32_t(CleaningOpportunity::Toolchange) &&
        when <= int32_t(CleaningOpportunity::Retraction))
        settings.when = static_cast<CleaningOpportunity>(when);
    return settings;
}

void remove_generated_insertions(
    PrintingPlan &plan,
    const PluginPropertyKey<NozzleCleaningGeneratedProperty> &generated_key)
{
    // Plans may be edited more than once before the host rebuilds its scope
    // events. Remove only trees carrying our private marker; user scripts and
    // other plugins' wrappers must remain untouched.
    for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
        PrintingGroup group = plan.group(group_idx);
        for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx) {
            PrintingLayerGroup layer = group.layer_group(layer_idx);
            for (uint32_t tool_idx = 0; tool_idx < layer.tool_group_count(); ++tool_idx) {
                PrintingToolGroup tool = layer.tool_group(tool_idx);
                for (uint32_t extrusion_idx = 0;
                     extrusion_idx < tool.extrusion_count(); ++extrusion_idx) {
                    PrintingExtrusion extrusion = tool.extrusion(extrusion_idx);
                    MutableExtrusionEntity root = extrusion.mutable_root();
                    if (!clean_generated_tree(root, generated_key))
                        continue;

                    // A before/after insertion at the extrusion root temporarily
                    // wrapped its old root. Restore a clone of that original
                    // content because the plan owns this top-level handle.
                    uint32_t generated_child_index = 0;
                    if (!is_generated_wrapper(root.readonly(), generated_key,
                                              &generated_child_index))
                        throw std::runtime_error(
                            "A marked nozzle-cleaning root lost its wrapper layout.");
                    const uint32_t content_index = generated_child_index == 0 ? 1u : 0u;
                    if (!extrusion.set_root_clone(root.child(content_index)))
                        throw std::runtime_error(
                            "Unable to restore an extrusion wrapped by nozzle cleaning.");
                }
            }
        }
    }
}

bool clean_generated_tree(
    MutableExtrusionEntity entity,
    const PluginPropertyKey<NozzleCleaningGeneratedProperty> &generated_key)
{
    // emplace_ordered_leaf() keeps its outer handle and puts the new event beside
    // the old content. Detect that exact two-child shape before descending, then
    // ask the parent to replace the wrapper with its original content.
    if (is_generated_wrapper(entity.readonly(), generated_key, nullptr)) {
        uint32_t generated_child_index = 0;
        is_generated_wrapper(entity.readonly(), generated_key, &generated_child_index);
        const uint32_t content_index = generated_child_index == 0 ? 1u : 0u;
        MutableExtrusionEntity content = entity.child_mutable(content_index);
        if (clean_generated_tree(content, generated_key))
            unwrap_generated_child(entity, content_index, content, generated_key);
        return true;
    }

    if (generated_key.has(entity)) {
        // Directly attached scripts sit on the original path or E-axis event.
        // Remove only our custom-G-code payload and marker; preserve its path,
        // axis request, inherited attributes and all unrelated properties.
        entity.remove(EPropertyCustomGcode::key);
        generated_key.remove(entity);
    }

    uint32_t child_idx = 0;
    while (child_idx < entity.child_count()) {
        MutableExtrusionEntity child = entity.child_mutable(child_idx);
        const bool was_generated = generated_key.has(child);
        if (clean_generated_tree(child, generated_key)) {
            unwrap_generated_child(entity, child_idx, child, generated_key);
            ++child_idx;
            continue;
        }

        if (was_generated && child.segment_count() == 0 && child.child_count() == 0 &&
            child.property_count() == 0) {
            // Standalone generated event leaves contain no source content after
            // their script is removed, so delete the empty leaf from the tree.
            if (!entity.remove_child(child_idx))
                throw std::runtime_error("Unable to remove an empty nozzle-cleaning event.");
            continue;
        }
        ++child_idx;
    }
    return false;
}

bool is_generated_wrapper(
    const ExtrusionEntity &entity,
    const PluginPropertyKey<NozzleCleaningGeneratedProperty> &generated_key,
    uint32_t *generated_child_index)
{
    // Only remove a wrapper if it matches the shape produced by our ordered
    // insertion: a fixed two-child parent with one marked script leaf and one
    // preserved source-content child.
    if (entity.child_count() != 2)
        return false;

    for (uint32_t child_idx = 0; child_idx < 2; ++child_idx) {
        const ExtrusionEntity child = entity.child(child_idx);
        if (!generated_key.has(child) ||
            child.get(EPropertyCustomGcode::key) == nullptr ||
            child.segment_count() != 0 || child.child_count() != 0)
            continue;
        // Check ownership before querying parent flags: unrelated transition
        // trees can contain wrapped native loops whose path-only geometry views
        // are not valid until their edit phases have finished.
        if (entity.segment_count() != 0 || entity.property_count() != 0 ||
            entity.sortable() || entity.reversible())
            return false;
        if (generated_child_index != nullptr)
            *generated_child_index = child_idx;
        return true;
    }
    return false;
}

void unwrap_generated_child(
    MutableExtrusionEntity parent,
    const uint32_t wrapper_index,
    MutableExtrusionEntity wrapper,
    const PluginPropertyKey<NozzleCleaningGeneratedProperty> &generated_key)
{
    // Move the untouched content out first. Removing the wrapper afterward
    // avoids cloning nested geometry and preserves its property-owned data.
    uint32_t generated_child_index = 0;
    if (!is_generated_wrapper(wrapper.readonly(), generated_key,
                              &generated_child_index))
        throw std::runtime_error("A nozzle-cleaning insertion wrapper is malformed.");
    const uint32_t content_index = generated_child_index == 0 ? 1u : 0u;
    if (parent.move_child_from(wrapper_index, wrapper, content_index) == EXTRUSION_INDEX_INVALID ||
        !parent.remove_child(wrapper_index + 1))
        throw std::runtime_error("Unable to restore content below a nozzle-cleaning insertion.");
}

IslandOwnerMap island_owners_for_layer_group(
    const PrintingLayerGroup &layer_group,
    const plugin_run_context *run_ctx)
{
    IslandOwnerMap owners;
    // Each PrintingExtrusion names its source LayerRegionIsland, but island
    // transitions are physical LayerIsland changes. Build this reverse lookup
    // once per layer group so several regions of one island compare equal.
    for (uint32_t layer_idx = 0; layer_idx < layer_group.layer_count(); ++layer_idx) {
        throw_if_cancelled(run_ctx);
        const Layer layer = layer_group.layer(layer_idx);
        if (!layer.valid())
            continue;
        for (uint32_t island_idx = 0; island_idx < layer.island_count(); ++island_idx) {
            throw_if_cancelled(run_ctx);
            const LayerIsland island = layer.island(island_idx);
            for (uint32_t region_idx = 0;
                 region_idx < island.region_island_count(); ++region_idx) {
                const LayerRegionIsland region_island = island.region_island(region_idx);
                if (region_island.valid())
                    owners[region_island.handle()] = island.handle();
            }
        }
    }
    return owners;
}

MutableExtrusionEntity first_scope_in_tree(
    MutableExtrusionEntity entity,
    const PluginPropertyKey<PrintingExtrusionScopeProperty> &scope_key,
    const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    // Scope roots are marked directly and never nest. Stop at the first marker
    // so this lookup returns the entry scope, not a later scope inside content.
    if (is_printing_extrusion_scope(entity.readonly(), scope_key))
        return entity;
    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
        MutableExtrusionEntity scope = first_scope_in_tree(
            entity.child_mutable(child_idx), scope_key, run_ctx);
        if (scope.valid())
            return scope;
    }
    return MutableExtrusionEntity();
}

MutableExtrusionEntity first_unretract_in_tree(
    MutableExtrusionEntity entity,
    const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    // The custom script must run after retract/wipe travel but before E is
    // restored. EPropertyExtrusionAxis gives that boundary a firmware-neutral
    // identity even when the actual command is G10/G11 or explicit E motion.
    const EPropertyExtrusionAxis *axis = entity.get(EPropertyExtrusionAxis::key);
    if (axis != nullptr && axis->operation == C_EXTRUSION_AXIS_OPERATION_UNRETRACT)
        return entity;
    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
        MutableExtrusionEntity unretract = first_unretract_in_tree(
            entity.child_mutable(child_idx), run_ctx);
        if (unretract.valid())
            return unretract;
    }
    return MutableExtrusionEntity();
}

class FirstPrintableLeafVisitor final : public ExtrusionTreeConstVisitor<>
{
public:
    explicit FirstPrintableLeafVisitor(const plugin_run_context *run_ctx) :
        m_run_ctx(run_ctx)
    {}

    MutableExtrusionEntity first(MutableExtrusionEntity root)
    {
        m_first = MutableExtrusionEntity();
        traverse(root.readonly());
        return m_first;
    }

protected:
    void visit_leaf(ExtrusionEntity entity) override
    {
        // The first depositing path is a safe fallback when no transition scope
        // reserves a before-phase. A path is selected once, after inherited
        // attributes have identified material flow and excluded process moves.
        if (m_first.valid() || entity.segment_count() == 0)
            return;
        throw_if_cancelled(m_run_ctx);
        const EPropertyAttributes *attributes = current_property(EPropertyAttributes::key);
        if (attributes == nullptr || attributes->mm3_per_mm() <= 0.0)
            return;
        const int32_t role = attributes->extrusion_role();
        if (RAW_EXTRUSION_ROLE_IS_TRAVEL(role) || RAW_EXTRUSION_ROLE_IS_WIPE(role) ||
            RAW_EXTRUSION_ROLE_IS_RETRACT(role) || RAW_EXTRUSION_ROLE_IS_UNRETRACT(role))
            return;
        m_first = MutableExtrusionEntity(
            const_cast<extrusion_entity_handle *>(entity.handle()));
    }

private:
    const plugin_run_context *m_run_ctx;
    MutableExtrusionEntity m_first;
};

MutableExtrusionEntity first_printable_leaf(
    MutableExtrusionEntity entity,
    const plugin_run_context *run_ctx)
{
    FirstPrintableLeafVisitor visitor(run_ctx);
    return visitor.first(entity);
}

EntryOpportunity make_entry_opportunity(
    MutableExtrusionEntity extrusion_root,
    const extruder_index_t extruder_id,
    const PluginPropertyKey<PrintingExtrusionScopeProperty> &scope_key,
    const bool due_when_opened,
    const plugin_run_context *run_ctx)
{
    EntryOpportunity opportunity;
    // Prefer the named incoming phase: its order is travel, preparation/start
    // scripts, then Unretract/content. This keeps cleaning after setup but before
    // extrusion resumes without relying on private child indexes.
    const MutableExtrusionEntity scope_root = first_scope_in_tree(
        extrusion_root, scope_key, run_ctx);
    if (scope_root.valid()) {
        const PrintingExtrusionScopeView scope(scope_root, scope_key);
        MutableExtrusionEntity before = scope.before();
        if (before.valid()) {
            MutableExtrusionEntity unretract = first_unretract_in_tree(before, run_ctx);
            opportunity.active = true;
            opportunity.due_when_opened = due_when_opened;
            opportunity.insertion.anchor = unretract.valid() ? unretract : before;
            opportunity.insertion.position = unretract.valid() ?
                CleaningInsertion::Position::BeforeEntity :
                CleaningInsertion::Position::AppendToPhase;
            opportunity.insertion.extruder_id = extruder_id;
            return opportunity;
        }
    }

    // Without an incoming phase, attach to the first material path. Its tree
    // position is still after preceding travel/events and before deposited flow.
    const MutableExtrusionEntity first_path = first_printable_leaf(extrusion_root, run_ctx);
    if (!first_path.valid())
        return opportunity;
    opportunity.active = true;
    opportunity.due_when_opened = due_when_opened;
    opportunity.insertion.anchor = first_path;
    opportunity.insertion.position = CleaningInsertion::Position::BeforeEntity;
    opportunity.insertion.extruder_id = extruder_id;
    return opportunity;
}

bool allows_toolchange(const CleaningOpportunity when)
{
    // Every mode includes tool changes; the enum adds progressively later
    // opportunities as the user selects object, island or retraction.
    return when == CleaningOpportunity::Toolchange ||
           when == CleaningOpportunity::Object ||
           when == CleaningOpportunity::Island ||
           when == CleaningOpportunity::Retraction;
}

bool allows_object(const CleaningOpportunity when)
{
    // Object mode also accepts tool changes; later modes inherit both.
    return when == CleaningOpportunity::Object ||
           when == CleaningOpportunity::Island ||
           when == CleaningOpportunity::Retraction;
}

bool allows_island(const CleaningOpportunity when)
{
    // Island mode inherits object/tool opportunities; retraction mode inherits
    // all three and adds completed retract cycles.
    return when == CleaningOpportunity::Island ||
           when == CleaningOpportunity::Retraction;
}

void initialize_tool(ToolCounters &tool, const CleaningSettings &settings)
{
    // Do not count the first selection as a return. It simply establishes this
    // tool's own starting values, independently from tools seen earlier.
    tool.initialized = true;
    tool.layers_remaining = settings.interval_layers;
    tool.volume_remaining = settings.interval_volume;
    tool.retractions_remaining = settings.interval_retractions;
    tool.toolchanges_remaining = settings.interval_toolchanges;
}

bool cleaning_is_due(const ToolCounters &tool, const CleaningSettings &settings)
{
    // The triggers combine with OR: crossing any enabled threshold requests a
    // clean, while an interval of zero is intentionally excluded.
    return (settings.interval_layers > 0 && tool.layers_remaining <= 0) ||
           (settings.interval_volume > 0.0 && tool.volume_remaining <= 0.0) ||
           (settings.interval_retractions > 0 && tool.retractions_remaining <= 0) ||
           (settings.interval_toolchanges > 0 && tool.toolchanges_remaining <= 0);
}

void reset_tool_counters_after_cleaning(ToolCounters &tool,
                                        const CleaningSettings &settings)
{
    // Reset every interval for this tool, not just the trigger that expired.
    // Retraction flags describe actual machine state and must survive a clean.
    tool.layers_remaining = settings.interval_layers;
    tool.volume_remaining = settings.interval_volume;
    tool.retractions_remaining = settings.interval_retractions;
    tool.toolchanges_remaining = settings.interval_toolchanges;
    tool.pending_toolchange_cleaning = false;
    tool.extruded_in_current_layer = false;
}

void decrement_interval(int64_t &remaining)
{
    // A due counter may continue below zero while waiting for its allowed
    // opportunity. Saturate rather than overflow after an unusually long wait.
    if (remaining > (std::numeric_limits<int64_t>::min)())
        --remaining;
}

CleaningTreeVisitor::CleaningTreeVisitor(
    const CleaningSettings &settings,
    ToolCounters &tool,
    const extruder_index_t extruder_id,
    EntryOpportunity &entry_opportunity,
    std::vector<CleaningInsertion> &insertions,
    const plugin_run_context *run_ctx) :
    m_settings(settings),
    m_tool(tool),
    m_extruder_id(extruder_id),
    m_entry_opportunity(entry_opportunity),
    m_insertions(insertions),
    m_run_ctx(run_ctx)
{}

void CleaningTreeVisitor::enter_node(MutableExtrusionEntity entity)
{
    throw_if_cancelled(m_run_ctx);
    // Tree entry is plan execution order. Process axis events here because one
    // event may live on a parent; inherited properties must not count it once
    // per descendant leaf.
    const EPropertyExtrusionAxis *axis = entity.get(EPropertyExtrusionAxis::key);
    const bool is_unretract = axis != nullptr &&
        axis->operation == C_EXTRUSION_AXIS_OPERATION_UNRETRACT;
    const bool completed_cycle = is_unretract && m_tool.retracted &&
        m_tool.retraction_cycle_pending;

    // Closing a cycle is the last safe retraction opportunity: retracting wipe
    // work and travel have already run, while the physical E restore has not.
    if (is_unretract)
        process_unretract(entity, completed_cycle);

    // Several partial RETRACT_TO requests can belong to one wipe sequence. Mark
    // the tool retracted on the first request and consume only one interval unit
    // until a matching Unretract closes that cycle.
    if (axis != nullptr && axis->operation == C_EXTRUSION_AXIS_OPERATION_RETRACT_TO &&
        axis->value > 0.0 && !m_tool.retracted) {
        m_tool.retracted = true;
        m_tool.retraction_cycle_pending = true;
        if (m_settings.interval_retractions > 0)
            decrement_interval(m_tool.retractions_remaining);
    }

    // If this is the selected transition anchor, queue the script before the
    // node. A matching Unretract is handled above so its cycle can be coalesced
    // with the object/island/tool transition instead of adding a second script.
    if (m_entry_opportunity.active &&
        m_entry_opportunity.insertion.position == CleaningInsertion::Position::BeforeEntity &&
        entity.handle() == m_entry_opportunity.insertion.anchor.handle() && !is_unretract) {
        if (m_entry_opportunity.due_when_opened || cleaning_is_due(m_tool, m_settings))
            clean_before_entity(entity);
    }
}

void CleaningTreeVisitor::process_unretract(
    MutableExtrusionEntity entity,
    const bool completed_retraction_cycle)
{
    throw_if_cancelled(m_run_ctx);
    // A boundary opened at extrusion entry may point exactly at this Unretract.
    // Independently, retraction mode permits a clean at the end of any complete
    // cycle. These conditions are combined before queuing, so one Unretract can
    // never receive two scripts for the same transition.
    const bool is_entry_anchor = m_entry_opportunity.active &&
        m_entry_opportunity.insertion.position == CleaningInsertion::Position::BeforeEntity &&
        entity.handle() == m_entry_opportunity.insertion.anchor.handle();
    const bool retraction_opportunity =
        m_settings.when == CleaningOpportunity::Retraction && completed_retraction_cycle;
    const bool entry_transition_due = is_entry_anchor &&
        (m_entry_opportunity.due_when_opened || cleaning_is_due(m_tool, m_settings));

    if ((entry_transition_due ||
         (retraction_opportunity && cleaning_is_due(m_tool, m_settings))))
        clean_before_entity(entity);

    // The semantic event now restores E. Keep this physical state update even
    // when no cleaning script was due at this cycle.
    if (completed_retraction_cycle) {
        m_tool.retracted = false;
        m_tool.retraction_cycle_pending = false;
    }
}

void CleaningTreeVisitor::visit_leaf(MutableExtrusionEntity entity)
{
    throw_if_cancelled(m_run_ctx);
    // Only leaves contain one local path to measure. current_property() resolves
    // flow attributes inherited from ancestors without counting the same tree
    // geometry at both parent and child levels.
    if (entity.segment_count() == 0)
        return;
    const EPropertyAttributes *attributes = current_property(EPropertyAttributes::key);
    if (attributes == nullptr || attributes->mm3_per_mm() <= 0.0)
        return;
    const int32_t role = attributes->extrusion_role();
    if (RAW_EXTRUSION_ROLE_IS_TRAVEL(role) || RAW_EXTRUSION_ROLE_IS_WIPE(role) ||
        RAW_EXTRUSION_ROLE_IS_RETRACT(role) || RAW_EXTRUSION_ROLE_IS_UNRETRACT(role))
        return;

    // local_length() includes the true arc length when this path contains arcs.
    // mm3_per_mm is already the planned effective flow, with no extrusion
    // multiplier applied.
    const double length_mm = unscaled(entity.local_length());
    const double volume_mm3 = length_mm * attributes->mm3_per_mm();
    if (!std::isfinite(volume_mm3) || volume_mm3 <= 0.0)
        return;

    if (m_settings.interval_volume > 0.0)
        m_tool.volume_remaining -= volume_mm3;
    // A layer counts only if this tool deposited after its most recent clean.
    m_tool.extruded_in_current_layer = true;
}

void CleaningTreeVisitor::leave_node(MutableExtrusionEntity entity)
{
    // If the incoming phase contains no Unretract, append after its existing
    // start/preparation events when that phase finishes traversal.
    if (!m_entry_opportunity.active ||
        m_entry_opportunity.insertion.position != CleaningInsertion::Position::AppendToPhase ||
        entity.handle() != m_entry_opportunity.insertion.anchor.handle())
        return;
    throw_if_cancelled(m_run_ctx);
    if (m_entry_opportunity.due_when_opened || cleaning_is_due(m_tool, m_settings))
        clean_after_phase(entity);
}

void CleaningTreeVisitor::clean_before_entity(MutableExtrusionEntity entity)
{
    // Do not add G-code here: save the anchor in the insertion queue because
    // restructuring the tree during traversal could invalidate the visitor.
    // process_plan() later passes this anchor to apply_insertion(). Reset now
    // because later nodes in this same ordered tree run after the planned clean.
    CleaningInsertion insertion;
    insertion.anchor = entity;
    insertion.position = CleaningInsertion::Position::BeforeEntity;
    insertion.extruder_id = m_extruder_id;
    m_insertions.push_back(insertion);
    m_entry_opportunity.active = false;
    reset_tool_counters_after_cleaning(m_tool, m_settings);
}

void CleaningTreeVisitor::clean_after_phase(MutableExtrusionEntity phase)
{
    // This phase has no Unretract anchor, so queue an append after its existing
    // preparation events. process_plan() later calls apply_insertion(); resetting
    // now makes any following deposited path start a fresh layer interval.
    CleaningInsertion insertion;
    insertion.anchor = phase;
    insertion.position = CleaningInsertion::Position::AppendToPhase;
    insertion.extruder_id = m_extruder_id;
    m_insertions.push_back(insertion);
    m_entry_opportunity.active = false;
    reset_tool_counters_after_cleaning(m_tool, m_settings);
}

void apply_insertion(
    storage_handle *storage,
    const CleaningInsertion &insertion,
    const CleaningSettings &settings,
    const PluginPropertyKey<NozzleCleaningGeneratedProperty> &generated_key)
{
    if (!insertion.anchor.valid() || settings.script.empty())
        return;

    if (insertion.position == CleaningInsertion::Position::AppendToPhase) {
        // Keep start/preparation scripts already in the phase first, then add a
        // separate event. This avoids inheriting the script onto every child.
        StoredExtrusionEntity event(storage);
        event.script_gcode(settings.script, GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM,
                           insertion.extruder_id);
        generated_key.get_or_add(event.mutable_view()).generated = 1;
        MutableExtrusionEntity phase = insertion.anchor;
        if (phase.append_child_move(event.mutable_view()) == EXTRUSION_INDEX_INVALID)
            throw std::runtime_error("Unable to append nozzle-cleaning G-code to a transition phase.");
        return;
    }

    MutableExtrusionEntity anchor = insertion.anchor;
    if (!anchor.has(EPropertyCustomGcode::key) && anchor.child_count() == 0) {
        // A leaf can own the custom script directly; the firmware writes it on
        // node entry, immediately before that leaf's E-axis operation or path.
        anchor.script_gcode(settings.script, GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM,
                            insertion.extruder_id);
        generated_key.get_or_add(anchor).generated = 1;
        return;
    }

    // Preserve any script/content already on this node by making an ordered
    // script leaf before it. The dynamic marker lets the next run unwrap only
    // this plugin-owned wrapper.
    MutableExtrusionEntity event = anchor.emplace_ordered_leaf(
        OrderedLeafPosition::Before,
        ExistingPropertyPlacement::MoveWithExistingContent);
    if (!event.valid())
        throw std::runtime_error("Unable to insert nozzle-cleaning G-code before its transition.");
    event.script_gcode(settings.script, GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM,
                       insertion.extruder_id);
    generated_key.get_or_add(event).generated = 1;
}

void process_plan(
    const plugin_run_context *run_ctx,
    const CleaningSettings &settings,
    PrintingPlan &plan,
    storage_handle *storage,
    const PluginPropertyKey<PrintingExtrusionScopeProperty> &scope_key,
    const PluginPropertyKey<NozzleCleaningGeneratedProperty> &generated_key)
{
    // Remove marked tree insertions from an earlier execution before inspecting
    // this plan. This gives the counter scan the original, unmodified work.
    remove_generated_insertions(plan, generated_key);

    // An empty script can never produce a clean. If every interval is zero,
    // clean old tree events above and skip all counter and identity work.
    if (settings.script.empty() ||
        (settings.interval_layers == 0 && settings.interval_volume == 0.0 &&
         settings.interval_retractions == 0 && settings.interval_toolchanges == 0))
        return;

    // This state belongs to this run only. Each map entry is one tool's private
    // set of counters; the two previous identities track global plan order.
    std::unordered_map<extruder_index_t, ToolCounters> tools;
    std::vector<CleaningInsertion> insertions;
    std::vector<ToolEventInsertion> tool_event_insertions;
    extruder_index_t active_extruder = EXTRUDER_ID_UNKNOWN;
    std::optional<uint32_t> previous_object_instance;
    std::optional<IslandIdentity> previous_island;

    // Match PrintingPlanFileWriter's nesting order: group, layer, tool, then
    // each extrusion tree. Tool groups are visited even when they are empty.
    for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
        throw_if_cancelled(run_ctx);
        PrintingGroup group = plan.group(group_idx);
        for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx) {
            throw_if_cancelled(run_ctx);
            PrintingLayerGroup layer = group.layer_group(layer_idx);
            // Convert source region-island handles to physical island identity
            // once for this layer group. Several regions may share one island.
            const IslandOwnerMap island_owners = island_owners_for_layer_group(layer, run_ctx);

            for (uint32_t tool_idx = 0; tool_idx < layer.tool_group_count(); ++tool_idx) {
                throw_if_cancelled(run_ctx);
                PrintingToolGroup tool_group = layer.tool_group(tool_idx);
                const extruder_index_t extruder_id = tool_group.extruder_id();
                if (extruder_id == EXTRUDER_ID_UNKNOWN)
                    continue;

                // operator[] creates a fresh state the first time this tool is
                // observed. Later visits retrieve the same per-tool counters.
                ToolCounters &tool = tools[extruder_id];
                const bool first_activation = !tool.initialized;
                if (first_activation)
                    initialize_tool(tool, settings);

                // A tool-change interval counts returns, not initial selection:
                // the incoming tool must have been used before, and a different
                // tool must have been active immediately before this group.
                if (!first_activation && active_extruder != EXTRUDER_ID_UNKNOWN &&
                    active_extruder != extruder_id) {
                    if (settings.interval_toolchanges > 0)
                        decrement_interval(tool.toolchanges_remaining);
                    if (allows_toolchange(settings.when) && cleaning_is_due(tool, settings))
                        tool.pending_toolchange_cleaning = true;
                }
                active_extruder = extruder_id;

                // If a previous run already left its marked clean in this
                // group's event list, it will execute before this visit. Reset
                // this tool's intervals at that point rather than queueing a
                // duplicate. The marker means "the script is already stored in
                // this plan", not "a clean is merely due". Empty visits get
                // this event from apply_tool_event_insertion(); this check makes
                // direct repeated runs idempotent. The reset changes all four
                // interval counters plus current-layer activity, but leaves the
                // actual E-axis retract state untouched.
                if (has_generated_tool_event(tool_group.events(), extruder_id,
                                             generated_key))
                    reset_tool_counters_after_cleaning(tool, settings);

                // Compare each printable root's catalog identity with the last
                // owned root. Catalog IDs distinguish two printed instances of
                // the same source object.
                for (uint32_t extrusion_idx = 0;
                     extrusion_idx < tool_group.extrusion_count(); ++extrusion_idx) {
                    throw_if_cancelled(run_ctx);
                    PrintingExtrusion extrusion = tool_group.extrusion(extrusion_idx);
                    const uint32_t object_instance_id = extrusion.printing_catalog_id();
                    bool object_changed = false;
                    bool island_changed = false;
                    // NONE and out-of-range IDs have no reliable object owner.
                    // Ignore them rather than inventing a transition from an
                    // auxiliary tree such as print-level generated geometry.
                    if (object_instance_id != SLIC3R_PRINTING_OBJECT_INSTANCE_ID_NONE &&
                        object_instance_id < plan.object_instance_count()) {
                        object_changed = previous_object_instance.has_value() &&
                            *previous_object_instance != object_instance_id;
                        previous_object_instance = object_instance_id;

                        // Region islands are only subdivisions. Resolve their
                        // owning LayerIsland so two regions on one island do
                        // not create a false island transition.
                        const LayerRegionIsland region_island = extrusion.region_island();
                        if (region_island.valid()) {
                            const IslandOwnerMap::const_iterator owner =
                                island_owners.find(region_island.handle());
                            if (owner != island_owners.end()) {
                                const IslandIdentity current_island{
                                    object_instance_id, owner->second};
                                // Include the object instance in the comparison:
                                // two instances can share the same source island.
                                island_changed = previous_island.has_value() &&
                                    !(*previous_island == current_island);
                                previous_island = current_island;
                            }
                        }
                    }

                    // A transition may satisfy several modes at once. Start
                    // with a due tool return, then OR in allowed object/island
                    // transitions. One bool deliberately represents the whole
                    // boundary, so it cannot schedule one script per cause.
                    bool boundary_due = tool.pending_toolchange_cleaning;
                    boundary_due = boundary_due ||
                        (object_changed && allows_object(settings.when) &&
                         cleaning_is_due(tool, settings));
                    boundary_due = boundary_due ||
                        (island_changed && allows_island(settings.when) &&
                         cleaning_is_due(tool, settings));

                    MutableExtrusionEntity root = extrusion.mutable_root();
                    EntryOpportunity entry_opportunity;
                    if (boundary_due) {
                        // Remember a safe position in this tree. The visitor may
                        // wait through travel/retraction until it reaches that
                        // anchor, where it queues the script and resets counters.
                        entry_opportunity = make_entry_opportunity(
                            root, extruder_id, scope_key, true, run_ctx);
                        if (entry_opportunity.active) {
                            // The outstanding tool-change opportunity is now
                            // carried by entry_opportunity until its anchor.
                            tool.pending_toolchange_cleaning = false;
                        }
                    }

                    // One visitor both measures material/retraction work and
                    // detects the exact in-tree point where a clean is allowed.
                    CleaningTreeVisitor visitor(settings, tool, extruder_id,
                        entry_opportunity, insertions, run_ctx);
                    visitor.traverse(root);
                }

                // If this was a due tool return but none of its roots had a
                // printable anchor, the tool-change event is still an allowed
                // opportunity. Put the script after selection in the group's
                // before-events instead of carrying the due request elsewhere.
                if (tool.pending_toolchange_cleaning && cleaning_is_due(tool, settings)) {
                    tool_event_insertions.push_back(
                        ToolEventInsertion{tool_group.mutable_handle(), extruder_id});
                    reset_tool_counters_after_cleaning(tool, settings);
                }
            }

            // A layer interval advances once per PrintingLayerGroup and tool,
            // only if that tool deposited material since its last clean. Clear
            // the activity bit even when the interval trigger is disabled.
            for (std::unordered_map<extruder_index_t, ToolCounters>::iterator it = tools.begin();
                 it != tools.end(); ++it) {
                ToolCounters &tool = it->second;
                if (tool.extruded_in_current_layer && settings.interval_layers > 0)
                    decrement_interval(tool.layers_remaining);
                tool.extruded_in_current_layer = false;
            }
        }
    }

    // Apply tree edits only after traversal has finished, so every borrowed
    // entity handle collected above remains valid throughout the scan.
    for (const CleaningInsertion &insertion : insertions) {
        throw_if_cancelled(run_ctx);
        apply_insertion(storage, insertion, settings, generated_key);
    }
    // Empty-visit scripts target separate tool event vectors and are appended
    // after all extrusion-tree changes have completed.
    for (const ToolEventInsertion &insertion : tool_event_insertions) {
        throw_if_cancelled(run_ctx);
        apply_tool_event_insertion(storage, insertion, settings, generated_key);
    }
}

void apply_tool_event_insertion(
    storage_handle *storage,
    const ToolEventInsertion &insertion,
    const CleaningSettings &settings,
    const PluginPropertyKey<NozzleCleaningGeneratedProperty> &generated_key)
{
    if (storage == nullptr || insertion.tool_group == nullptr || settings.script.empty())
        throw std::invalid_argument("An empty tool-visit cleaning needs valid storage and ownership.");

    // There is no extrusion tree to anchor on for an empty visit. Append a
    // normal custom-G-code event to the tool group's before-list; that list is
    // serialized after existing selection commands and before the next group.
    StoredExtrusionEntity event(storage);
    event.script_gcode(settings.script, GCODE_SCRIPT_TYPE_EXTRUSION_CUSTOM,
                       insertion.extruder_id);
    generated_key.get_or_add(event.mutable_view()).generated = 1;
    PrintingToolGroup tool_group(insertion.tool_group);
    if (!tool_group.events().append_before_move(event.mutable_view()).valid())
        throw std::runtime_error("Unable to append nozzle cleaning to an empty tool visit.");
}

bool has_generated_tool_event(
    const PrintingScopeEvents &events,
    const extruder_index_t extruder_id,
    const PluginPropertyKey<NozzleCleaningGeneratedProperty> &generated_key)
{
    // Empty-visit cleaning is appended only to before-events. An after-event
    // would run after this visit's material and therefore must not reset its
    // counters at tool-group entry.
    return events.has_before() && entity_has_generated_tool_script(
        events.before(), extruder_id, generated_key);
}

bool entity_has_generated_tool_script(
    const ExtrusionEntity &entity,
    const extruder_index_t extruder_id,
    const PluginPropertyKey<NozzleCleaningGeneratedProperty> &generated_key)
{
    // The marker establishes ownership; the extruder check ensures an event for
    // a different tool never resets this tool's independent counter set.
    if (generated_key.has(entity)) {
        const EPropertyCustomGcode *script = entity.get(EPropertyCustomGcode::key);
        if (script != nullptr && script->processing_extruder_id == extruder_id)
            return true;
    }
    for (uint32_t child_idx = 0; child_idx < entity.child_count(); ++child_idx)
        if (entity_has_generated_tool_script(entity.child(child_idx), extruder_id,
                                             generated_key))
            return true;
    return false;
}

} // namespace

NozzleCleaning &NozzleCleaning::instance(orchestrator_handle *orchestrator)
{
    static NozzleCleaning plugin(orchestrator);
    return plugin;
}

NozzleCleaning::NozzleCleaning(orchestrator_handle *orchestrator) :
    PluginBase(orchestrator),
    m_generated_property(PluginPropertyKey<NozzleCleaningGeneratedProperty>::register_dynamic(
        orchestrator, k_generated_property_name))
{}

const char *NozzleCleaning::id_impl() const noexcept
{
    return "nozzle_cleaning";
}

const char *NozzleCleaning::name_impl() const noexcept
{
    return "Nozzle cleaning";
}

const char *NozzleCleaning::description_impl() const noexcept
{
    return "Insert a nozzle-cleaning script when per-tool intervals expire.";
}

slicing_step_t NozzleCleaning::step_impl() const noexcept
{
    return STEP_EXTRUSION_EDIT;
}

const char *const *NozzleCleaning::dependencies_impl() const noexcept
{
    return k_no_dependencies;
}

int32_t NozzleCleaning::priority_impl() const noexcept
{
    // Settings and per-print scripts are present by priority 100; feature role
    // scripts run at 1000 and therefore see this event in its final path order.
    return 200;
}

int32_t NozzleCleaning::used_config_keys(raw_used_config_key *keys) const noexcept
{
    constexpr int32_t count = int32_t(sizeof(k_used_config_keys) / sizeof(k_used_config_keys[0]));
    if (keys != nullptr)
        for (int32_t idx = 0; idx < count; ++idx)
            keys[idx] = k_used_config_keys[idx];
    return count;
}

int32_t NozzleCleaning::defined_config_keys(const char **keys) const noexcept
{
    constexpr int32_t count = int32_t(sizeof(k_defined_config_keys) / sizeof(k_defined_config_keys[0]));
    if (keys != nullptr)
        for (int32_t idx = 0; idx < count; ++idx)
            keys[idx] = k_defined_config_keys[idx];
    return count;
}

const char *NozzleCleaning::progress_message_format_impl() const noexcept
{
    return "Preparing nozzle cleaning: %u / %u insertions";
}

const char *NozzleCleaning::print_ui_fragment() noexcept
{
    return "page:Custom G-code\n"
           "group:Nozzle cleaning\n"
           "setting:nozzle_cleaning_gcode\n"
           "setting:nozzle_cleaning_interval_layers\n"
           "setting:nozzle_cleaning_interval_volume\n"
           "setting:nozzle_cleaning_interval_retractions\n"
           "setting:nozzle_cleaning_interval_toolchanges\n"
           "setting:nozzle_cleaning_when\n"
           "end_group\n";
}

void NozzleCleaning::inilialize_impl(storage_handle *storage) const
{
    (void)storage;
    // All six options belong to the FFF printer preset and invalidate the plan
    // edit step because changing any value can change script placement/content.
    // The index order matches k_defined_config_keys and the switch below.
    for (size_t idx = 0; idx < sizeof(k_defined_config_keys) / sizeof(k_defined_config_keys[0]); ++idx) {
        raw_config_option_def def = raw_config_option_def_init();
        def.opt_key = k_defined_config_keys[idx];
        def.container_type = RAW_CONTAINER_TYPE_PROJECT;
        def.option_preset_type = RAW_PRESET_TYPE_FFF_PRINTER;
        def.printer_technology = RAW_PT_FFF;
        def.invalidates_step = STEP_EXTRUSION_EDIT;
        def.category = RAW_OPTION_CATEGORY_CUSTOMGCODE;
        def.level = RAW_OPTION_LEVEL_EXPERT;
        def.mode = RAW_CONFIG_OPTION_MODE_ADV_EXP | RAW_CONFIG_OPTION_MODE_SUSI;
        def.tooltip = "Per-tool interval. Zero disables this trigger; any enabled trigger can request cleaning.";

        // Keep each option definition beside its own type, UI metadata and
        // default. The enum's order is also the CleaningOpportunity value used
        // when the serialized printer preset is read by load_settings().
        switch (idx) {
        case 0:
            def.type = RAW_CO_STRING;
            def.label = "Nozzle cleaning G-code";
            def.full_label = "Nozzle cleaning G-code";
            def.tooltip = "Script executed for the tool being cleaned with standard extrusion custom-G-code placeholders. Leave the machine in a state compatible with the print transition that follows.";
            def.multiline = 1;
            def.full_width = 1;
            def.is_code = 1;
            def.height = 120;
            def.default_serialized_value = "";
            break;
        case 1:
            def.type = RAW_CO_INT;
            def.label = "Layers between cleanings";
            def.sidetext = "layers";
            def.tooltip = "Per-tool count of PrintingPlan layer groups where this tool deposited material. A layer is counted after it finishes.";
            def.has_min = 1;
            def.min_value = 0;
            def.default_serialized_value = "0";
            break;
        case 2:
            def.type = RAW_CO_FLOAT;
            def.label = "Volume between cleanings";
            def.sidetext = "mm³";
            def.tooltip = "Per-tool planned depositing volume, calculated from actual path length and effective mm³/mm. Travels, retractions and wipe motions are excluded.";
            def.has_min = 1;
            def.min_value = 0.0;
            def.precision = 1;
            def.default_serialized_value = "0";
            break;
        case 3:
            def.type = RAW_CO_INT;
            def.label = "Retractions between cleanings";
            def.sidetext = "retractions";
            def.tooltip = "Per-tool count of complete retract/restore cycles. Partial retract commands and retracting wipe segments within one cycle count once.";
            def.has_min = 1;
            def.min_value = 0;
            def.default_serialized_value = "0";
            break;
        case 4:
            def.type = RAW_CO_INT;
            def.label = "Tool changes between cleanings";
            def.sidetext = "tool changes";
            def.tooltip = "Per-tool count of returns after another tool was selected. The first activation initializes the counters and is not counted.";
            def.has_min = 1;
            def.min_value = 0;
            def.default_serialized_value = "0";
            break;
        case 5: {
            static const key_value_string_pair_t choices[] = {
                { "toolchange", "Tool change" },
                { "object", "Object or tool change" },
                { "island", "Island, object or tool change" },
                { "retraction", "Retraction, island, object or tool change" }
            };
            def.type = RAW_CO_ENUM;
            def.gui_type = RAW_GUI_TYPE_SELECT_CLOSE;
            def.label = "Clean when due";
            def.full_label = "Earliest transition allowed to clean the nozzle";
            def.tooltip = "A due cleaning waits for the first allowed transition: tool change; object or tool change; island, object or tool change; or retraction, island, object or tool change. One transition inserts at most one script.";
            def.enum_def.value_label_pairs.items = choices;
            def.enum_def.value_label_pairs.count = uint32_t(sizeof(choices) / sizeof(choices[0]));
            def.default_serialized_value = "retraction";
            break;
        }
        default:
            continue;
        }
        orchestrator_create_option_def(m_orchestrator, &def);
    }

    orchestrator_add_ui_fragment(m_orchestrator, "printer_fff.ui", "nozzle_cleaning",
                                 print_ui_fragment(), 0);
}

void NozzleCleaning::run_impl(const plugin_run_context *run_ctx) const
{
    throw_if_cancelled(run_ctx);
    // STEP_EXTRUSION_EDIT provides both the print configuration and the final
    // mutable PrintingPlan. Reject an incomplete payload before taking views.
    const run_ctx_extrusion_edition *context =
        plugin_ctx_as_extrusion_edition(run_ctx);
    if (context == nullptr || context->print == nullptr || context->plan == nullptr)
        throw std::runtime_error("Nozzle cleaning received an incomplete extrusion-edit context.");

    const Print print(context->print);
    PrintingPlan plan(context->plan);
    const CleaningSettings settings = load_settings(print.config());
    // Dynamic property IDs are scoped to this orchestrator. Reacquire the shared
    // transition key for the scope view and pass our owned-insertions key into
    // the single run-local plan traversal.
    const PluginPropertyKey<PrintingExtrusionScopeProperty> scope_key =
        printing_extrusion_scope_property_key(m_orchestrator);
    process_plan(run_ctx, settings, plan, run_ctx->plugin_storage,
                 scope_key, m_generated_property);
}

void register_nozzle_cleaning_plugin(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(orchestrator,
        NozzleCleaning::instance(orchestrator).c_instance());
}

}} // namespace slic3r_api::NozzleCleaningPlugin

#ifdef NOZZLE_CLEANING_PLUGIN_DLL
#include "libslic3r/Api/plugin/c/slic3r_plugin_register_version.h"

extern "C" SLIC3R_PLUGIN_API void register_plugin(orchestrator_handle *orchestrator)
{
    slic3r_api::NozzleCleaningPlugin::register_nozzle_cleaning_plugin(orchestrator);
}
#endif // NOZZLE_CLEANING_PLUGIN_DLL
