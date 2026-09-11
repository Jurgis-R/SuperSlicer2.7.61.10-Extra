#include "StepGenerateGcode.hpp"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <utility>
#include <vector>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/host/GCodeScriptProcessor.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_gcode.h"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_gcode_firmware.h"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeFirmwareViews.hpp"
#include "libslic3r/ExtrusionProperty.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/libslic3r.h"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"

/*
STEP_GCODE bridge
=================

The G-code step is now a plugin boundary. Earlier pipeline steps build a
PrintingPlan, then this step creates the selected firmware session and invokes
the active STEP_GCODE writer. The writer receives the source Print, mutable
PrintingPlan, borrowed firmware session, and resolved final output path.

Before crossing that plugin boundary, the step validates the process speed of
every geometric leaf and explicit E-only extrusion command, including those in
before/after scope events. This is the last common point where the host can
describe an invalid tree with its complete PrintingPlan location; a firmware
session sees only one borrowed tree at a time and cannot recover that context.

File format decisions still belong to the selected plugin, so external writers
can replace the built-in implementation without changing the pipeline runner.
*/

namespace Slic3r::Steps::StepGenerateGcode {

namespace {

constexpr size_t MaximumReportedSpeedErrors = 10;

/* One direct EPropertySpeed declaration encountered from a root to a leaf. */
struct SpeedDeclaration
{
    std::vector<size_t> child_path;
    float value = -1.f;
};

/*
State inherited while walking one extrusion tree.

Positive finite speeds replace the effective value. Non-positive values are
the public inheritance sentinel and therefore leave it unchanged. Non-finite
values are retained separately: even if another node provides a usable speed,
the malformed declaration must still be reported for every geometric leaf
whose path contains it.
*/
struct SpeedTraversalState
{
    std::optional<float> effective_speed;
    std::optional<uint16_t> effective_role;
    std::vector<SpeedDeclaration> declarations;
    bool has_non_finite_speed = false;
};

/*
Location and source context for one independently serialized extrusion tree.

The firmware resets inherited requested state before every PrintingExtrusion
and every before/after event root. Keeping the root path here lets the same
walker validate both kinds while optional pointers describe only context that
actually exists at that scope.
*/
struct PrintingTreeLocation
{
    std::string root_path;
    const Printing::PrintingLayerGroup *layer_group = nullptr;
    const Printing::PrintingToolGroup *tool_group = nullptr;
    const Printing::PrintingExtrusion *extrusion = nullptr;
};

/*
Accumulate every failure count while retaining only a bounded number of full
diagnostics. A damaged plan may contain thousands of related leaves; the total
still tells the developer the blast radius without creating an unusable GUI
message or log entry.
*/
struct SpeedValidationReport
{
    size_t error_count = 0;
    size_t speed_error_count = 0;
    size_t extrusion_event_speed_error_count = 0;
    size_t event_speed_error_count = 0;
    size_t missing_root_count = 0;
    std::vector<std::string> details;

    void add_speed_error(std::string detail)
    {
        ++error_count;
        ++speed_error_count;
        if (details.size() < MaximumReportedSpeedErrors)
            details.push_back(std::move(detail));
    }

    void add_extrusion_event_speed_error(std::string detail)
    {
        ++error_count;
        ++extrusion_event_speed_error_count;
        if (details.size() < MaximumReportedSpeedErrors)
            details.push_back(std::move(detail));
    }

    void add_event_speed_error(std::string detail)
    {
        ++error_count;
        ++event_speed_error_count;
        if (details.size() < MaximumReportedSpeedErrors)
            details.push_back(std::move(detail));
    }

    void add_missing_root(std::string detail)
    {
        ++error_count;
        ++missing_root_count;
        if (details.size() < MaximumReportedSpeedErrors)
            details.push_back(std::move(detail));
    }
};

/* Return a readable value while preserving NaN and infinity signs. */
std::string speed_value_text(float value);

/* Format the zero-based path used by PrintingPlan and the debugger. */
std::string extrusion_tree_path(const PrintingTreeLocation &location,
                                const std::vector<size_t> &child_path);

/* Describe the concrete tree node without relying on RTTI class names. */
const char *extrusion_entity_type(const ExtrusionEntity &entity);

/* Find a source PrintObject's zero-based position in Print::objects(). */
std::optional<size_t> print_object_index(const Print &print, const PrintObject *object);

/* Append plan and optional source-object metadata shared by all failures. */
void append_plan_source_context(std::ostringstream &out,
                                const Print &print,
                                const PrintingTreeLocation &location);

/*
Build the full diagnostic for one geometric leaf. The source pointers stored in
PrintingPlan are optional in synthetic tests, so every source-derived field has
an explicit unavailable form instead of dereferencing a missing context.
*/
std::string geometric_speed_error(const Print &print,
                                  const ExtrusionEntity &entity,
                                  const PrintingTreeLocation &location,
                                  const std::vector<size_t> &child_path,
                                  const SpeedTraversalState &state,
                                  bool special_extrusion_event);

/* Validate one tree recursively while preserving inherited process state. */
void validate_extrusion_tree_speed(const Print &print,
                                   const ExtrusionEntity &entity,
                                   const PrintingTreeLocation &location,
                                   std::vector<size_t> &child_path,
                                   const SpeedTraversalState &parent_state,
                                   SpeedValidationReport &report);

/* Validate the non-empty before/after roots owned by one plan scope. */
void validate_scope_event_speeds(const Print &print,
                                 const Printing::PrintingScopeEvents &events,
                                 const std::string &scope_path,
                                 const Printing::PrintingLayerGroup *layer_group,
                                 const Printing::PrintingToolGroup *tool_group,
                                 SpeedValidationReport &report);

/* Convert the bounded report into the public validate_pre() diagnostic. */
std::string speed_validation_message(const SpeedValidationReport &report);

/*
Own the raw firmware session returned through the plugin ABI.

The provider allocates the session, but STEP_GCODE may fail before or during
file publication. Keeping ownership in this stack object guarantees that the
provider destructor is called once on every path.
*/
class FirmwareInstanceOwner
{
public:
    FirmwareInstanceOwner() = default;
    ~FirmwareInstanceOwner();

    FirmwareInstanceOwner(const FirmwareInstanceOwner &) = delete;
    FirmwareInstanceOwner &operator=(const FirmwareInstanceOwner &) = delete;

    raw_gcode_firmware_instance *mutable_instance();
    const raw_gcode_firmware_instance *instance() const;

private:
    raw_gcode_firmware_instance m_instance = {};
};

// Ask the selected provider for one session and validate it before the
// STEP_GCODE writer can observe the borrowed instance.
void create_firmware_session(Orchestrator &orchestrator,
                             Print &print,
                             const raw_gcode_script_processor *script_processor,
                             FirmwareInstanceOwner &owner);

std::string speed_value_text(const float value)
{
    if (std::isnan(value))
        return "NaN";
    if (std::isinf(value))
        return value > 0.f ? "+infinity" : "-infinity";

    std::ostringstream out;
    out << std::setprecision(std::numeric_limits<float>::max_digits10) << value;
    return out.str();
}

std::string extrusion_tree_path(const PrintingTreeLocation &location,
                                const std::vector<size_t> &child_path)
{
    std::ostringstream out;
    out << location.root_path;
    for (const size_t child_idx : child_path)
        out << ".children[" << child_idx << "]";
    return out.str();
}

const char *extrusion_entity_type(const ExtrusionEntity &entity)
{
    if (entity.child_count() > 0 && entity.is_loop())
        return "loop";
    if (entity.child_count() > 0 && entity.is_continuous())
        return "multipath";
    if (entity.child_count() > 0)
        return "collection";
    if (entity.is_nop())
        return "nop";
    if (entity.polyline_or_null() != nullptr)
        return "path";
    return "empty";
}

std::optional<size_t> print_object_index(const Print &print, const PrintObject *object)
{
    if (object == nullptr)
        return std::nullopt;
    for (size_t object_idx = 0; object_idx < print.objects().size(); ++object_idx)
        if (&print.objects()[object_idx] == object)
            return object_idx;
    return std::nullopt;
}

void append_plan_source_context(std::ostringstream &out,
                                const Print &print,
                                const PrintingTreeLocation &location)
{
    out << "Plan context: print_z=";
    if (location.layer_group != nullptr)
        out << std::setprecision(12) << unscaled(location.layer_group->print_z) << " mm";
    else
        out << "unavailable";
    out << ", extruder_id=";
    if (location.tool_group != nullptr)
        out << location.tool_group->extruder_id;
    else
        out << "unavailable";
    out << ", source_role=";
    if (location.extrusion != nullptr)
        out << role_to_code(location.extrusion->sregion_island_role);
    else
        out << "unavailable (scope event)";
    out << "\n";

    if (location.extrusion == nullptr) {
        out << "Source context: unavailable for a before/after scope event\n";
        return;
    }

    const Printing::PrintingExtrusion &extrusion = *location.extrusion;
    const LayerRegionIsland *region_island = extrusion.region_island;
    const LayerRegion *source_region = region_island != nullptr && !region_island->regions().empty() ?
        *region_island->regions().begin() : nullptr;
    const Layer *source_layer = source_region != nullptr ? source_region->layer() : nullptr;
    const PrintObject *source_object = source_layer != nullptr ? source_layer->object() : nullptr;
    const ModelObject *model_object = source_object != nullptr ? source_object->model_object() : nullptr;
    const std::optional<size_t> source_object_idx = print_object_index(print, source_object);

    out << "Source context: ";
    if (region_island == nullptr) {
        out << "region_island=unavailable, layer=unavailable, object=unavailable";
    } else {
        out << "region_count=" << region_island->regions().size()
            << ", island_extruder_id=" << region_island->extruder_id();
        if (source_layer != nullptr)
            out << ", layer_id=" << source_layer->id();
        else
            out << ", layer=unavailable";
        if (source_object != nullptr) {
            out << ", print_object_index=";
            if (source_object_idx)
                out << *source_object_idx;
            else
                out << "unavailable";
            out << ", object_name="
                << std::quoted(model_object != nullptr ? model_object->name : std::string("<unnamed>"));
        } else {
            out << ", object=unavailable";
        }
    }
    out << ", object_instance_idx=" << extrusion.object_instance_idx;
    if (source_object != nullptr && extrusion.object_instance_idx >= source_object->instances().size())
        out << " (invalid; object has " << source_object->instances().size() << " instances)";
    out << "\n";
}

std::string geometric_speed_error(const Print &print,
                                  const ExtrusionEntity &entity,
                                  const PrintingTreeLocation &location,
                                  const std::vector<size_t> &child_path,
                                  const SpeedTraversalState &state,
                                  const bool special_extrusion_event)
{
    std::ostringstream out;
    out << "PrintingPlan location: " << extrusion_tree_path(location, child_path) << "\n";
    append_plan_source_context(out, print, location);

    const ArcPolyline *polyline = entity.polyline_or_null();
    const size_t point_count = polyline != nullptr ? polyline->size() : 0;
    const size_t segment_count = point_count > 0 ? point_count - 1 : 0;
    out << "Entity: id=" << entity.get_id() << ", type=" << extrusion_entity_type(entity)
        << ", points=" << point_count << ", segments=" << segment_count
        << ", length=" << std::setprecision(12) << unscaled(entity.length()) << " mm";
    if (polyline != nullptr && !polyline->empty()) {
        out << ", start=(" << unscaled(polyline->front().x()) << ", "
            << unscaled(polyline->front().y());
        if (polyline->has_z_offset())
            out << ", z_offset=" << unscaled(polyline->z_offset(0));
        out << ") mm, end=(" << unscaled(polyline->back().x()) << ", "
            << unscaled(polyline->back().y());
        if (polyline->has_z_offset())
            out << ", z_offset=" << unscaled(polyline->z_offset(point_count - 1));
        out << ") mm, contains_arcs=" << (polyline->has_arc() ? "yes" : "no");
    }
    if (state.effective_role)
        out << ", effective_role=" << *state.effective_role;
    else
        out << ", effective_role=unavailable";
    if (special_extrusion_event)
        out << ", movement=E-only special extrusion";
    else
        out << ", movement=geometric";
    out << "\n";

    out << "Speed declarations:";
    if (state.declarations.empty()) {
        out << " none on the leaf or its parents\n";
    } else {
        out << "\n";
        for (const SpeedDeclaration &declaration : state.declarations)
            out << "  - " << extrusion_tree_path(location, declaration.child_path)
                << ": speed_mm_per_s=" << speed_value_text(declaration.value)
                << (std::isfinite(declaration.value) && declaration.value <= 0.f ? " (inherit)" : "")
                << "\n";
    }

    if (state.has_non_finite_speed)
        out << "Reason: a speed declaration on this path is NaN or infinite."
               " Every explicit speed must be finite.\n";
    else if (state.declarations.empty())
        out << "Reason: neither this "
            << (special_extrusion_event ? "E-only event" : "geometric leaf")
            << " nor any parent declares a speed.\n";
    else
        out << "Reason: every speed declaration on this path requests inheritance,"
               " but no parent provides a finite positive speed.\n";
    return out.str();
}

void validate_extrusion_tree_speed(const Print &print,
                                   const ExtrusionEntity &entity,
                                   const PrintingTreeLocation &location,
                                   std::vector<size_t> &child_path,
                                   const SpeedTraversalState &parent_state,
                                   SpeedValidationReport &report)
{
    SpeedTraversalState state = parent_state;
    if (const ExtrusionPropertySpeed *speed = entity.get_property<ExtrusionPropertySpeed>()) {
        state.declarations.push_back(SpeedDeclaration{child_path, speed->speed_mm_per_s});
        if (!std::isfinite(speed->speed_mm_per_s)) {
            state.has_non_finite_speed = true;
        } else if (speed->speed_mm_per_s > 0.f) {
            state.effective_speed = speed->speed_mm_per_s;
        }
    }
    if (const ExtrusionAttributes *attributes = entity.get_property<ExtrusionAttributes>())
        state.effective_role = attributes->role;

    for (size_t child_idx = 0; child_idx < entity.child_count(); ++child_idx) {
        child_path.push_back(child_idx);
        validate_extrusion_tree_speed(
            print, entity.child(child_idx), location, child_path, state, report);
        child_path.pop_back();
    }

    const ExtrusionPropertySpecialCommand *special_command =
        entity.get_property<ExtrusionPropertySpecialCommand>();
    const bool special_extrusion_event = special_command != nullptr &&
        special_command->command_code() == ExtrusionPropertySpecialCommand::Code::EXTRUSION &&
        special_command->extra_data != 0.0;
    const ArcPolyline *polyline = entity.polyline_or_null();
    const bool geometric_movement = entity.child_count() == 0 &&
        polyline != nullptr && polyline->size() >= 2;

    // Direct special-command properties execute when their node is entered,
    // including on container nodes. Ordinary geometry executes only on a leaf.
    // Retraction-axis events are omitted here because their speed may legally
    // come from retract_speed/deretract_speed or native firmware retraction.
    if (!geometric_movement && !special_extrusion_event)
        return;
    if (!state.has_non_finite_speed && state.effective_speed &&
        std::isfinite(*state.effective_speed) && *state.effective_speed > 0.f)
        return;

    const std::string detail = geometric_speed_error(
        print, entity, location, child_path, state, special_extrusion_event);
    if (location.extrusion == nullptr)
        report.add_event_speed_error(detail);
    else if (!geometric_movement)
        report.add_extrusion_event_speed_error(detail);
    else
        report.add_speed_error(detail);
}

void validate_scope_event_speeds(const Print &print,
                                 const Printing::PrintingScopeEvents &events,
                                 const std::string &scope_path,
                                 const Printing::PrintingLayerGroup *layer_group,
                                 const Printing::PrintingToolGroup *tool_group,
                                 SpeedValidationReport &report)
{
    // The file writer invokes write_event() once per non-empty root. Validate
    // each root independently because the firmware's tree writer resets all
    // inherited requested properties between those calls.
    const std::pair<const char *, const ExtrusionEntity *> roots[] = {
        {"before", events.has_before() ? &events.before() : nullptr},
        {"after", events.has_after() ? &events.after() : nullptr}
    };
    for (const std::pair<const char *, const ExtrusionEntity *> &root : roots) {
        if (root.second == nullptr)
            continue;
        const PrintingTreeLocation location{
            scope_path + ".events." + root.first,
            layer_group,
            tool_group,
            nullptr
        };
        std::vector<size_t> child_path;
        validate_extrusion_tree_speed(
            print, *root.second, location, child_path, SpeedTraversalState{}, report);
    }
}

std::string speed_validation_message(const SpeedValidationReport &report)
{
    std::ostringstream out;
    out << "STEP_GCODE cannot generate G-code because the PrintingPlan contains "
        << report.error_count << " validation " << (report.error_count == 1 ? "error" : "errors") << ".\n";
    if (report.speed_error_count > 0)
        out << report.speed_error_count << " geometric extrusion "
            << (report.speed_error_count == 1 ? "leaf has" : "leaves have")
            << " no valid effective speed. A geometric leaf must define or inherit a finite speed"
               " greater than zero.\n";
    if (report.extrusion_event_speed_error_count > 0)
        out << report.extrusion_event_speed_error_count << " E-only special extrusion "
            << (report.extrusion_event_speed_error_count == 1 ? "event has" : "events have")
            << " no valid effective speed. Such an event must define or inherit a finite speed"
               " greater than zero.\n";
    if (report.event_speed_error_count > 0)
        out << report.event_speed_error_count << " movement-producing before/after event "
            << (report.event_speed_error_count == 1 ? "entity has" : "entities have")
            << " no valid effective speed. Each event tree is serialized with independent"
               " inherited process state.\n";
    if (report.missing_root_count > 0)
        out << report.missing_root_count << " PrintingExtrusion "
            << (report.missing_root_count == 1 ? "has" : "entries have") << " no root.\n";
    out << "All PrintingPlan indices below are zero-based.\n";
    for (size_t detail_idx = 0; detail_idx < report.details.size(); ++detail_idx)
        out << "\n[" << detail_idx + 1 << "/" << report.error_count << "]\n"
            << report.details[detail_idx];
    if (report.error_count > report.details.size())
        out << "\n" << report.error_count - report.details.size()
            << " additional validation errors were omitted.\n";
    return out.str();
}

FirmwareInstanceOwner::~FirmwareInstanceOwner()
{
    // A provider may publish owning fields before producing an invalid table.
    // Destroy whenever the required pointers are available so validation
    // failures do not leak that partially published session.
    if (m_instance.session != nullptr && m_instance.vtable != nullptr &&
        m_instance.vtable->struct_size >=
            offsetof(raw_gcode_firmware_vtable, destroy) + sizeof(gcode_firmware_destroy_fn) &&
        m_instance.vtable->destroy != nullptr)
        m_instance.vtable->destroy(m_instance.session);
}

raw_gcode_firmware_instance *FirmwareInstanceOwner::mutable_instance()
{
    return &m_instance;
}

const raw_gcode_firmware_instance *FirmwareInstanceOwner::instance() const
{
    return &m_instance;
}

void create_firmware_session(Orchestrator &orchestrator,
                             Print &print,
                             const raw_gcode_script_processor *script_processor,
                             FirmwareInstanceOwner &owner)
{
    Plugin *firmware_plugin = selected_or_active_plugin_for_step(
        orchestrator, GCODE_FIRMWARE, &print.full_print_config());
    if (firmware_plugin == nullptr)
        throw RuntimeError("No active G-code firmware plugin is available.");

    plugin_host_context host_context = orchestrator.prepare_plugin_host_context(
        GCODE_FIRMWARE, firmware_plugin, &print);
    plugin_run_context run_context = orchestrator.prepare_plugin_run_context(
        GCODE_FIRMWARE, firmware_plugin, &host_context);
    run_ctx_gcode_firmware payload = {};
    payload.print = reinterpret_cast<const print_handle *>(&print);
    payload.script_processor = script_processor;
    run_context.data = &payload;

    firmware_plugin->setup(run_context, 1);
    firmware_plugin->setup_run(run_context);
    firmware_plugin->run(run_context);

    // Transfer ownership before checking cancellation or table completeness;
    // the provider may allocate a session before reporting its error.
    *owner.mutable_instance() = payload.instance;
    if (run_context.is_cancelled != nullptr && run_context.is_cancelled(run_context.host_context))
        throw RuntimeError("G-code firmware plugin failed while creating its session.");

    // Constructing the borrowed view validates struct sizes and every required
    // callback without invoking firmware code.
    const slic3r_api::GCodeFirmwareView validated(owner.instance());
    (void)validated;
}

} // namespace

void clean_and_prepare(Print &) {}

bool validate_pre(const Print &print, std::string *error)
{
    if (error != nullptr)
        error->clear();

    const Printing::PrintingPlan *plan = print.printing_plan();
    if (plan == nullptr) {
        if (error != nullptr)
            *error = "STEP_GCODE requires the PrintingPlan produced by STEP_ORDERING, but the Print has no plan.";
        return false;
    }

    SpeedValidationReport report;
    validate_scope_event_speeds(print, plan->events, "plan", nullptr, nullptr, report);
    for (size_t group_idx = 0; group_idx < plan->groups.size(); ++group_idx) {
        const Printing::PrintingGroup &group = plan->groups[group_idx];
        const std::string group_path = "groups[" + std::to_string(group_idx) + "]";
        validate_scope_event_speeds(
            print, group.events, group_path, nullptr, nullptr, report);
        for (size_t layer_idx = 0; layer_idx < group.layers.size(); ++layer_idx) {
            const Printing::PrintingLayerGroup &layer_group = group.layers[layer_idx];
            const std::string layer_path = group_path + ".layers[" +
                std::to_string(layer_idx) + "]";
            validate_scope_event_speeds(
                print, layer_group.events, layer_path, &layer_group, nullptr, report);
            for (size_t tool_group_idx = 0; tool_group_idx < layer_group.tool_groups.size(); ++tool_group_idx) {
                const Printing::PrintingToolGroup &tool_group = layer_group.tool_groups[tool_group_idx];
                const std::string tool_group_path = layer_path + ".tool_groups[" +
                    std::to_string(tool_group_idx) + "]";
                validate_scope_event_speeds(
                    print, tool_group.events, tool_group_path,
                    &layer_group, &tool_group, report);
                for (size_t extrusion_idx = 0; extrusion_idx < tool_group.extrusions.size(); ++extrusion_idx) {
                    const Printing::PrintingExtrusion &extrusion = tool_group.extrusions[extrusion_idx];
                    const PrintingTreeLocation location{
                        tool_group_path + ".extrusions[" +
                            std::to_string(extrusion_idx) + "].root",
                        &layer_group,
                        &tool_group,
                        &extrusion
                    };
                    if (extrusion.root == nullptr) {
                        std::ostringstream detail;
                        detail << "PrintingPlan location: " << extrusion_tree_path(location, {}) << "\n";
                        append_plan_source_context(detail, print, location);
                        detail << "Entity: unavailable because the PrintingExtrusion has no root.\n"
                               << "Speed declarations: unavailable because there is no extrusion tree.\n"
                               << "Reason: the PrintingExtrusion has no root extrusion tree.\n";
                        report.add_missing_root(detail.str());
                        continue;
                    }

                    std::vector<size_t> child_path;
                    validate_extrusion_tree_speed(print, *extrusion.root, location, child_path,
                                                  SpeedTraversalState{}, report);
                }
            }
        }
    }

    if (report.error_count == 0)
        return true;
    if (error != nullptr)
        *error = speed_validation_message(report);
    return false;
}

bool validate_post(const Print &, std::string *)
{
    return true;
}

void run_step(Orchestrator &orchestrator, Print &print, const std::string &path)
{
    if (path.empty())
        throw RuntimeError("G-code generation plugin needs a non-empty output path.");

    // Run the host-owned validation before constructing plugins or touching a
    // staging file. RuntimeError is required here instead of the debug-only
    // validate_or_report helper so Release builds expose the same diagnosis.
    std::string validation_error;
    if (!validate_pre(print, &validation_error))
        throw RuntimeError(validation_error);

    Plugin *plugin = selected_or_active_plugin_for_step(orchestrator, STEP_GCODE, &print.full_print_config());
    if (plugin == nullptr)
        throw RuntimeError("No active G-code generation plugin is available.");

    // The G-code step consumes the PrintingPlan created by STEP_ORDERING. If a
    // caller runs this step directly in a test, mutable_printing_plan() still
    // gives the plugin a valid empty plan instead of a null handle.
    Printing::PrintingPlan &plan = print.mutable_printing_plan();

    // The host parser outlives the borrowed firmware view and is destroyed
    // only after the firmware session has released every reference to it.
    GCodeScriptProcessor scripts(print, orchestrator);
    FirmwareInstanceOwner firmware;
    create_firmware_session(orchestrator, print, scripts.c_processor(), firmware);
    run_ctx_generate_gcode payload = {};
    payload.print = reinterpret_cast<const print_handle *>(&print);
    payload.plan = reinterpret_cast<printing_plan_handle *>(&plan);
    payload.firmware = firmware.instance();
    payload.output_path = path.c_str();

    plugin_host_context host_context = orchestrator.prepare_plugin_host_context(STEP_GCODE, plugin, &print);
    plugin_run_context run_context = orchestrator.prepare_plugin_run_context(STEP_GCODE, plugin, &host_context);
    run_context.data = &payload;

    plugin->setup(run_context, 1);
    plugin->setup_run(run_context);
    plugin->run(run_context);

    // PluginBase reports C++ exceptions through report_error() instead of
    // letting them cross the C ABI. report_error() requests cancellation; the
    // host step turns that request back into a normal export failure.
    if (run_context.is_cancelled != nullptr && run_context.is_cancelled(run_context.host_context))
        throw RuntimeError("G-code generation plugin failed.");
}

} // namespace Slic3r::Steps::StepGenerateGcode
