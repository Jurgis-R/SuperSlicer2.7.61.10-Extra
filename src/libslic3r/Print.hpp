///|/ Copyright (c) Prusa Research 2016 - 2023 Lukáš Matěna @lukasmatena, Enrico Turri @enricoturri1966, Vojtěch Bubník @bubnikv, Tomáš Mészáros @tamasmeszaros, Pavel Mikuš @Godrak, Lukáš Hejl @hejllukas, Filip Sykala @Jony01, Oleksandra Iushchenko @YuSanka, Vojtěch Král @vojtechkral
///|/ Copyright (c) BambuStudio 2023 manch1n @manch1n
///|/ Copyright (c) SuperSlicer 2022 Remi Durand @supermerill
///|/ Copyright (c) 2019 Bryan Smith
///|/ Copyright (c) 2017 Eyal Soha @eyal0
///|/ Copyright (c) Slic3r 2013 - 2016 Alessandro Ranellucci @alranel
///|/ Copyright (c) 2017 Joseph Lenox @lordofhyphens
///|/
///|/ ported from lib/Slic3r/Print.pm:
///|/ Copyright (c) Prusa Research 2016 - 2018 Vojtěch Bubník @bubnikv, Tomáš Mészáros @tamasmeszaros
///|/ Copyright (c) Slic3r 2011 - 2016 Alessandro Ranellucci @alranel
///|/ Copyright (c) 2012 - 2013 Mark Hindess
///|/ Copyright (c) 2013 Devin Grady
///|/ Copyright (c) 2012 - 2013 Mike Sheldrake @mesheldrake
///|/ Copyright (c) 2012 Henrik Brix Andersen @henrikbrixandersen
///|/ Copyright (c) 2012 Michael Moon
///|/ Copyright (c) 2011 Richard Goodwin
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Print_hpp_
#define slic3r_Print_hpp_

#include <atomic>
#include <cstdint>
#include <ctime>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "DataTreeFwd.hpp"
#include "ExtrusionEntityCollection.hpp"
#include "Flow.hpp"
#include "GCode/ThumbnailData.hpp"
#include "GCode/ToolOrdering.hpp"
#include "GCode/WipeTower.hpp"
#include "libslic3r.h"
#include "Point.hpp"
#include "PrintBase.hpp"
#include "Config/FFFPrintConfig.hpp"
#include "PrintRecords.hpp"
#include "PrintSteps.hpp"
#include "Steps/PipelineProgress.hpp"

namespace Slic3r {

class GCodeGenerator;
class Plugin;
struct GCodeProcessorResult;
class WipeTower2;
struct ConflictResult;
namespace Printing {
struct PrintingPlan;
}
namespace ApiInternal {
struct PrintAccess;
}
namespace Steps {
enum class Checkpoint : uint8_t;
}

struct WipeTowerData
{
    // Following section will be consumed by the GCodeGenerator.
    // Tool ordering of a non-sequential print has to be known to calculate the wipe tower.
    // Cache it here, so it does not need to be recalculated during the G-code generation.
    Print                                                *print;
    // Cache of tool changes per print layer.
    std::unique_ptr<std::vector<WipeTower::ToolChangeResult>> priming;
    std::vector<std::vector<WipeTower::ToolChangeResult>> tool_changes;
    std::unique_ptr<WipeTower::ToolChangeResult>          final_purge;
    std::vector<std::pair<float, std::vector<float>>>     used_filament_until_layer;
    int                                                   number_of_toolchanges;

    // Depth of the wipe tower to pass to GLCanvas3D for exact bounding box:
    float                                                 depth;
    std::vector<std::pair<float, float>>                  z_and_depth_pairs;
    float                                                 brim_width;
    float                                                 height;

    // Data needed to generate fake extrusions for conflict checking.
    float                                                 width;
    float                                                 first_layer_height;
    float                                                 cone_angle;
    Vec2d                                                 position;
    float                                                 rotation_angle;

    void clear() {
        priming.reset(nullptr);
        tool_changes.clear();
        final_purge.reset(nullptr);
        used_filament_until_layer.clear();
        number_of_toolchanges = -1;
        depth = 0.f;
        z_and_depth_pairs.clear();
        brim_width = 0.f;
        height = 0.f;
        width = 0.f;
        first_layer_height = 0.f;
        cone_angle = 0.f;
        position = Vec2d::Zero();
        rotation_angle = 0.f;
    }

private:
	// Only allow the WipeTowerData to be instantiated internally by Print, 
	// as this WipeTowerData shares reference to Print::m_tool_ordering.
	friend class Print;
	WipeTowerData(Print *print) : print(print) { clear(); }
	WipeTowerData(const WipeTowerData & /* rhs */) = delete;
	WipeTowerData &operator=(const WipeTowerData & /* rhs */) = delete;
};

struct PrintStatistics
{
    PrintStatistics() { clear(); }
    // PrintEstimatedStatistics::ETimeMode::Normal -> time
    std::map<uint8_t, double>       estimated_print_time;
    std::map<uint8_t, std::string>  estimated_print_time_str;
    double                          total_used_filament;
    std::vector<std::pair<size_t, double>> color_extruderid_to_used_filament; // id -> mm (length)
    double                          total_extruded_volume;
    double                          total_cost;
    int                             total_toolchanges;
    double                          total_weight;
    std::vector<std::pair<size_t, double>> color_extruderid_to_used_weight;
    double                          total_wipe_tower_cost;
    double                          total_wipe_tower_filament;
    double                          total_wipe_tower_filament_weight;
    std::vector<extruder_index_t>   printing_extruders;
    extruder_index_t                initial_extruder_id;
    std::string                     initial_filament_type;
    std::string                     printing_filament_types;
    std::map<extruder_index_t, double> filament_stats; // extruder id -> volume in mm3
    std::vector<std::pair<coord_t, float>> _layer_area_stats; // print_z to area

    std::atomic_bool is_computing_gcode;

    // Config with the filled in print statistics.
    DynamicConfig           config() const;
    // Config with the statistics keys populated with placeholder strings.
    static DynamicConfig    placeholders();
    // Replace the print statistics placeholders in the path.
    std::string             finalize_output_path(const std::string &path_in) const;

    void clear() {
        estimated_print_time.clear();
        estimated_print_time_str.clear();
        total_used_filament    = 0.;
        total_extruded_volume  = 0.;
        total_cost             = 0.;
        total_toolchanges      = 0;
        total_weight           = 0.;
        total_wipe_tower_cost  = 0.;
        total_wipe_tower_filament = 0.;
        total_wipe_tower_filament_weight = 0.;
        initial_extruder_id    = EXTRUDER_ID_UNKNOWN;
        initial_filament_type.clear();
        printing_filament_types.clear();
        filament_stats.clear();
        printing_extruders.clear();
        color_extruderid_to_used_filament.clear();
        color_extruderid_to_used_weight.clear();
        _layer_area_stats.clear();
        is_computing_gcode = false;
    }

    static const std::string FilamentUsedG;
    static const std::string FilamentUsedGMask;
    static const std::string TotalFilamentUsedG;
    static const std::string TotalFilamentUsedGMask;
    static const std::string TotalFilamentUsedGValueMask;
    static const std::string FilamentUsedCm3;
    static const std::string FilamentUsedCm3Mask;
    static const std::string FilamentUsedMm;
    static const std::string FilamentUsedMmMask;
    static const std::string FilamentCost;
    static const std::string FilamentCostMask;
    static const std::string TotalFilamentCost;
    static const std::string TotalFilamentCostMask;
    static const std::string TotalFilamentCostValueMask;
    static const std::string TotalFilamentUsedWipeTower;
    static const std::string TotalFilamentUsedWipeTowerValueMask;
};

struct ConflictResult
{
    std::string _objName1;
    std::string _objName2;
    double      _height;
    const void* _obj1; // nullptr means wipe tower
    const void* _obj2;
    int         layer = -1;
    ConflictResult(const std::string& objName1, const std::string& objName2, double height, const void* obj1, const void* obj2)
        : _objName1(objName1), _objName2(objName2), _height(height), _obj1(obj1), _obj2(obj2)
    {}
    ConflictResult(const std::string& objName1, const std::string& objName2, double height, const void* obj1, const void* obj2, int layer_id)
        : _objName1(objName1), _objName2(objName2), _height(height), _obj1(obj1), _obj2(obj2), layer(layer_id)
    {}
    ConflictResult() = default;
};

using ConflictResultOpt = std::optional<ConflictResult>;

// The complete print tray with possibly multiple objects.
class Print : public PrintBase
{
private: // Prevents erroneous use by other classes.
    // Bool indicates if supports of PrintObject are top-level contour.
    typedef std::pair<PrintObject *, bool>         PrintObjectInfo;

public:
    Print();
    ~Print() override;

    PrinterTechnology	technology() const noexcept override { return ptFFF; }

    // Methods, which change the state of Print / PrintObject / PrintRegion.
    // The following methods are synchronized with process() and Orchestrator::export_gcode(),
    // so that slicing and exporting may be called from a background thread.
    // In case the following methods need to modify data processed by slicing or exporting,
    // a cancellation callback is executed to stop the background processing before the operation.
    void                clear() override;
    bool                empty() const override { return m_objects.empty(); }
    // List of existing PrintObject IDs, to remove notifications for non-existent IDs.
    std::vector<ObjectID> print_object_ids() const override;

    ApplyStatus         apply(const Model &model, DynamicPrintConfig config) override;
    void set_task(const TaskParams &params) override;
    void process() override;
    void finalize() override;
    void                cleanup() override;

    // methods for handling state
    // Returns true if a print step is done, or if an object step is done on all objects.
    bool                is_step_done(slicing_step_t step) const;
    PrintStateBase::StateWithTimeStamp step_state_with_timestamp(slicing_step_t step) const;
    PrintStateBase::StateWithWarnings  step_state_with_warnings(slicing_step_t step) const;
    // Returns true if the last step was finished with success.
    bool                finished() const override { return this->is_step_done(psGCodeExport); }

    bool                has_infinite_skirt() const;
    bool                has_skirt() const;
    bool                has_brim() const;

    // Returns an empty string if valid, otherwise returns an error message.
    std::pair<PrintValidationError, std::string> validate(std::vector<std::string>* warnings = nullptr) const override;
    Flow                brim_flow(extruder_index_t extruder_id, const PrintObjectConfig &brim_config) const;
    Flow                skirt_flow(extruder_index_t extruder_id, bool first_layer=false) const;
    coord_t             get_min_first_layer_height() const;
    coord_t             get_object_first_layer_height(const PrintObject& object) const;

    // get the extruders of these objects
    std::set<extruder_index_t> object_extruders(const PrintObjectPtrs &objects, coord_t z = -1) const;
    // get all extruders from the list of objects in this print ( same as print.object_extruders(print.objects()) )
    std::set<extruder_index_t> object_extruders(coord_t z = -1) const;
    std::set<extruder_index_t> support_material_extruders(coord_t z = -1) const;
    // all extruder to print layers that extrude at this z.
    std::set<extruder_index_t> extruders(coord_t z = -1) const;
    // Effective first-layer bed temperature across every tool used at Z=0.
    int32_t first_layer_bed_temperature() const;
    double              max_allowed_layer_height() const;
    bool                has_support_material() const;
    // Make sure the background processing has no access to this model_object during this call!
    void                auto_assign_extruders(ModelObject* model_object) const;

    const PrintConfig&          config() const { return m_config; }
    const PrintRecordStore&     records() const { return m_records; }
    PrintRecordStore&           records() { return m_records; }
    const PrintObjectConfig&    default_object_config() const { return m_default_object_config; }
    const PrintRegionConfig&    default_region_config() const { return m_default_region_config; }

    PrintObjectCRefs            objects() const { return make_ref_view<PrintObject>(m_objects); }
    PrintObjectRefs             objects() { return make_ref_view<PrintObject>(m_objects); }
    const PrintObject&          object(size_t idx) const { return *m_objects[idx]; }
    PrintObject&                object(size_t idx) { return *m_objects[idx]; }
    const PrintObject *get_print_object_by_model_object_id(ObjectID object_id) const;
    // PrintObject by its ObjectID, to be used to uniquely bind slicing warnings to their source PrintObjects
    // in the notification center.
    const PrintObject *get_object(ObjectID object_id) const;
    // How many of PrintObject::copies() over all print objects are there?
    // If zero, then the print is empty and the print shall not be executed.
    uint16_t                    num_object_instances() const;
    // Sort the PrintObjects by their increasing Z, likely useful for avoiding colisions on Deltas during sequential prints.
    std::vector<const PrintInstance*> sort_object_instances_by_max_z() const;
    // Sort the PrintObjects by their increasing Y, likely useful for avoiding colisions on printer with a x-bar during sequential prints.
    std::vector<const PrintInstance*> sort_object_instances_by_max_y() const;
    // Produce a vector of PrintObjects in the order of their respective ModelObjects in print.model().
    std::vector<const PrintInstance*> sort_object_instances_by_model_order() const;

    [[deprecated("brim/skirt are stored in auxiliary layers; this accessor materializes a compatibility cache.")]]
    const std::optional<ExtrusionEntityCollection>& skirt_first_layer() const;
    [[deprecated("brim/skirt are stored in auxiliary layers; this accessor materializes a compatibility cache.")]]
    const ExtrusionEntityCollection& skirt() const;
    [[deprecated("brim/skirt are stored in auxiliary layers; this accessor materializes a compatibility cache.")]]
    const ExtrusionEntityCollection& brim() const;
    // Convex hull of the 1st layer extrusions, for bed leveling and placing the initial purge line.
    // It encompasses the object extrusions, support extrusions, skirt, brim, wipe tower.
    // It does NOT encompass user extrusions generated by custom G-code,
    // therefore it does NOT encompass the initial purge line.
    // It does NOT encompass MMU/MMU2 starting (wipe) areas.
    const Polygon&                   first_layer_convex_hull() const { return m_first_layer_convex_hull; }

    const PrintStatistics&      print_statistics() const { return m_print_statistics; }
    PrintStatistics&            print_statistics() { return m_print_statistics; }
    const std::optional<ConflictResult>& conflict_result() const { return m_conflict_result; }
    std::time_t                 timestamp_last_change() const { return m_timestamp_last_change; }

    // Wipe tower support.
    bool                        has_wipe_tower() const;
    const WipeTowerData&        wipe_tower_data(const ConfigBase* config, double nozzle_diameter) const;
    const WipeTowerData&        wipe_tower_data() const { return wipe_tower_data(&this->m_config,0); }
    const WipeTower2*           wipe_tower2() const { return m_wipe_tower2.get(); }
    const std::vector<ToolOrdering> &tool_orderings() const { return m_tool_orderings; }
    const Printing::PrintingPlan *printing_plan() const { return m_printing_plan.get(); }
    Printing::PrintingPlan &mutable_printing_plan();
    void reset_printing_plan();
    // Control whether Checkpoint::Ordering is available: when enabled, a deep
    // copy of the plan is kept after STEP_ORDERING so plan editors can replay
    // without re-ordering. Disabling it releases the copy.
    void set_keep_ordering_checkpoint(bool keep);
    bool keep_ordering_checkpoint() const { return m_keep_ordering_checkpoint; }
    // Store, query, restore or release the Checkpoint::Ordering plan copy.
    // Restoring clones the copy so it can be restored again later.
    bool save_ordering_checkpoint();
    bool has_ordering_checkpoint() const;
    bool restore_ordering_checkpoint();
    void clear_ordering_checkpoint();
    // Path of the finished text stream used by GCodeReader and GCodeViewer.
    // The producer owns the file; Print owns only this copied UTF-8 path.
    const std::string &gcode_reader_source_path() const { return m_gcode_reader_source_path; }
    void set_gcode_reader_source_path(std::string path);
    void reset_gcode_reader_source_path();
    const PrintObject *auxiliary_object() const { return m_auxiliary_object.get(); }
    PrintObject &mutable_auxiliary_object();
    void reset_auxiliary_object();

    std::string                 output_filename(const std::string &filename_base = std::string()) const override;

    // Intern a complete snapshot. Concurrent registrations/readers are allowed;
    // apply() and clear() must run with workers stopped. Does not activate it.
    const PrintRegion &register_print_region(const PrintRegionConfig &config);
    // Complete regional overrides using this Print's current defaults. Invalid
    // keys/types return nullptr without modifying the registry.
    const PrintRegion *register_print_region_config(const ConfigBase &config);
    // Rebuild only the active catalogue in deterministic object order.
    void rebuild_active_print_regions();

    size_t                      num_print_regions() const throw() { return m_print_regions.size(); }
    PrintRegionCRefs            print_regions() const  { return make_ref_view<PrintRegion, PrintRegionPtrs>(m_print_regions); }
    const PrintRegion&          print_region(size_t idx) const  { return *m_print_regions[idx]; }

    const Polygons& get_sequential_print_clearance_contours() const { return m_sequential_print_clearance_contours; }
//TODO: decide to use this one or the printconfig one.
    static bool sequential_print_horizontal_clearance_valid(const Print& print, Polygons* polygons = nullptr);

    //put this in public to be accessible for tests, it was in private before.
    bool                invalidate_state_by_config_options(const ConfigOptionResolver& new_config, const std::vector<t_config_option_key> &opt_keys);

    // Control thread only: callers hold state_mutex, which the GUI cancel
    // callback expects before it can stop and wait for the worker. Worker-side
    // paths must use request_replay_from_worker() instead.
    void                request_replay_from(Steps::Checkpoint checkpoint);
    // Worker/idle only: refresh scheduling and historical milestones without self-cancellation.
    void                request_replay_from_worker(Steps::Checkpoint checkpoint);
    // Latest checkpoint whose data is valid: the checkpoint just before the
    // first requested step, moved back to Checkpoint::Print when its plan
    // (Ordering copy or live Plan) is missing.
    Steps::Checkpoint   current_checkpoint() const;
    // Pipeline-only: request every step after current_checkpoint() and put the
    // live plan in the state that checkpoint describes (restored Ordering copy,
    // or released plan for Checkpoint::Print and earlier).
    void                prepare_checkpoint_resume();
    /*
    One successful plugin setup() that may still hold caches for this Print. The
    entry is kept until a clear callback succeeds, so a failed cleanup can be
    retried before the owners it borrowed are replaced. setup_order is the host's
    monotonic setup counter, used to unwind participants in reverse order.
    */
    struct PluginCleanupParticipant
    {
        Plugin *plugin = nullptr;
        // Pipeline step whose setup created the state; also its cleanup boundary.
        slicing_step_t step = STEP_NONE;
        size_t setup_order = 0;
    };
    // Record one plugin setup so it can receive a clear callback later.
    void track_plugin_setup(Plugin &plugin, slicing_step_t step);
    // Ask every tracked plugin at or after first_invalid_step to release the
    // state it prepared. Returns false when at least one callback failed; failed
    // entries stay tracked for a retry.
    bool clear_plugin_state(slicing_step_t first_invalid_step);
    // Borrowed list used by the orchestrator to unwind plugins before their owners are replaced.
    const std::vector<PluginCleanupParticipant> &plugin_cleanup_participants() const
        { return m_plugin_cleanup_participants; }
    // Drop one participant after its clear callback succeeded.
    void erase_plugin_cleanup_participant(Plugin *plugin, slicing_step_t step);
    // Pipeline step currently running, or STEP_NONE outside the pipeline.
    slicing_step_t      running_step() const { return m_running_step; }
    void                set_running_step(slicing_step_t step) { m_running_step = step; }
    // Checkpoint freeze contract: data owned by the steps up to
    // last_editor_step may be edited only by those steps, or outside the
    // pipeline (apply, tests). Host API debug asserts use this, for example
    // STEP_POST_SLICING for slices and STEP_FINALIZE_SLICING for the data tree.
    bool                running_step_may_edit(slicing_step_t last_editor_step) const;
    // Worker-owned or stopped-worker primitives; UI invalidation must use request_replay_from().
    void                mark_step_for_execution(slicing_step_t step);
    void                mark_step_and_dependents_for_execution(slicing_step_t step);
    bool                should_execute_step(slicing_step_t step) const;
    void                mark_step_executed(slicing_step_t step);
    void                reset_step_execution_plan_all();

    Steps::PipelineProgress       &pipeline_progress()       { return m_pipeline_progress; }
    const Steps::PipelineProgress &pipeline_progress() const { return m_pipeline_progress; }

    // Deprecated: legacy PrintState invalidation used by the historical object
    // and print steps. New plugin-pipeline options should mark execution with
    // invalidates_step and the methods above instead of extending this linear
    // propagation model.
    bool                invalidate_step(slicing_step_t step);
    bool                invalidate_steps(std::initializer_list<slicing_step_t> steps);
    bool                invalidate_all_steps();

    // Legacy mutable facade used by G-code processing for warnings, step state
    // and statistics. It is unrelated to PrintStatus, which only delivers
    // progress and control notifications.
    class StatusMonitor
    {
    private:
        Print& print;

    public:
        StatusMonitor(Print &print_mutable) : print(print_mutable) {}

        // need this extra method because active_step_add_warning is protected and so need the friend status, and Gcode has it.
        void active_step_add_warning(PrintStateBase::WarningLevel warning_level, const std::string &message, int message_id = 0)
        {
            print.active_step_add_warning(warning_level, message, message_id);
        }
        PrintStatistics &stats() { return print.m_print_statistics; }
        bool             set_started(slicing_step_t step) { return print.set_started(step); }
        PrintStateBase::TimeStamp set_done(slicing_step_t step) { return print.set_done(step); }
        
    };

protected:
private:
    bool                set_started(slicing_step_t step);
    PrintStateBase::TimeStamp set_done(slicing_step_t step);
    void                active_step_add_warning(PrintStateBase::WarningLevel warning_level, const std::string &message, int message_id = 0);

    [[deprecated("Skirt/brim is generated by STEP_SKIRT_BRIM plugins into auxiliary layers.")]]
    void                _make_skirt_brim();
    [[deprecated("Skirt/brim is generated by STEP_SKIRT_BRIM plugins into auxiliary layers.")]]
    void                _make_skirt(const PrintObjectPtrs &objects, ExtrusionEntityCollection &out, std::optional<ExtrusionEntityCollection> &out_first_layer);
    //void                _make_wipe_tower();
    void                finalize_first_layer_convex_hull();
    void                alert_when_supports_needed();
    // Non-cancelling scheduling for a checkpoint resume. Only the stored Ordering
    // copy is released here; the live plan stays until plugin cleanup completed.
    void                request_steps_after(Steps::Checkpoint checkpoint);
    // Caller holds state_mutex and owns execution, or has already stopped the worker.
    bool                invalidate_checkpoint_milestones(Steps::Checkpoint checkpoint);

    // Islands of objects and their supports extruded at the 1st layer.
    Polygons            first_layer_islands() const;
    // Return 4 wipe tower corners in the world coordinates (shifted and rotated), including the wipe tower brim.
    Points              first_layer_wipe_tower_corners() const;

    // Returns true if any of the print_objects has print_object_step valid.
    // That means data shared by all print objects of the print_objects span may still use the shared data.
    // Otherwise the shared data shall be released.
    // Unguarded variant, thus it shall only be called from main thread with background processing stopped.
    static bool         is_shared_print_object_step_valid_unguarded(SpanOfConstPtrs<PrintObject> print_objects, PrintObjectStep print_object_step);

    PrintConfig                             m_config;
    // Variable metadata shared by plugins and host producers for this Print.
    PrintRecordStore                        m_records;
    PrintObjectConfig                       m_default_object_config;
    PrintRegionConfig                       m_default_region_config;
    PrintObjectUPtrs                        m_objects;
    // Hidden owner for print-level auxiliary layers such as global skirt, brim
    // or wipe-tower helper geometry. It is not part of objects(), so normal
    // model slicing, validation and GUI object counts keep seeing only real
    // model-derived PrintObjects.
    std::unique_ptr<PrintObject>             m_auxiliary_object;
    // print regions are stored in PrintObjectRegions, here it's a shortcut
    // Historical ownership is separate from the active catalogue below.
    std::mutex m_region_registry_mutex;
    std::map<size_t, std::vector<std::unique_ptr<const PrintRegion>>> m_region_registry;
    PrintRegionPtrs                         m_print_regions;

    /*
    Deprecated read-through caches for old G-code and tests that still expect
    print.brim()/skirt() to return an ExtrusionEntityCollection reference.
    They are rebuilt from auxiliary layers on every accessor call and must
    never be used as the canonical storage for new brim/skirt output.
    */
    mutable std::optional<ExtrusionEntityCollection> m_legacy_skirt_first_layer_cache;
    mutable ExtrusionEntityCollection               m_legacy_skirt_cache;
    mutable ExtrusionEntityCollection               m_legacy_brim_cache;
    // Convex hull of the 1st layer extrusions.
    // It encompasses the object extrusions, support extrusions, skirt, brim, wipe tower.
    // It does NOT encompass user extrusions generated by custom G-code,
    // therefore it does NOT encompass the initial purge line.
    // It does NOT encompass MMU/MMU2 starting (wipe) areas.
    Polygon                                 m_first_layer_convex_hull;
    Points                                  m_skirt_convex_hull;

    // Following section will be consumed by the GCodeGenerator.
    std::vector<ToolOrdering>               m_tool_orderings;
    std::unique_ptr<Printing::PrintingPlan> m_printing_plan;
    // Checkpoint::Ordering copy. It owns its plan trees and metadata but
    // borrows source objects, layers and region-islands from this Print.
    std::unique_ptr<Printing::PrintingPlan> m_ordering_checkpoint;
    bool                                    m_keep_ordering_checkpoint = true;
    // Written by the pipeline thread between steps only; read by host API
    // debug checks while the step runs.
    slicing_step_t                          m_running_step = STEP_NONE;
    std::string                             m_gcode_reader_source_path;
    mutable std::mutex                      m_wipe_tower_data_mutex;
    WipeTowerData                           m_wipe_tower_data {this};
    std::unique_ptr<WipeTower2>             m_wipe_tower2;

    // Estimated print time, filament consumed.
    PrintStatistics                         m_print_statistics;
    // time of last change, used by the gui to see if it needs to be updated
    std::time_t                             m_timestamp_last_change;

    // Cache to store sequential print clearance contours
    Polygons m_sequential_print_clearance_contours;

    // Steps that the next StepPipeline::run() should execute. The set is owned
    // by Print because config changes arrive here, while StepPipeline only
    // knows how to consume the plan in its fixed written order.
    std::set<slicing_step_t> m_steps_to_execute;
    std::vector<PluginCleanupParticipant>     m_plugin_cleanup_participants;
    // Tracked caches may belong to valid in-flight work; their presence alone must not trigger UI cleanup.
    std::atomic_bool                         m_plugin_cleanup_pending { false };
    size_t                                    m_next_plugin_setup_order = 0;

    Steps::PipelineProgress                    m_pipeline_progress;

    // To allow GCode to set the Print's GCodeExport step status.
    //friend class GCodeGenerator;
    // To allow GCodeProcessor to emit warnings.
    //friend class GCodeProcessor;
    // Allow PrintObject to access m_mutex and m_cancel_callback.
    friend class PrintObject;
    friend struct ApiInternal::PrintAccess;

    std::optional<ConflictResult> m_conflict_result;
};

//for testing purpose (in printobject)
ExPolygons dense_fill_fit_to_size(const ExPolygon &polygon_to_cover,
    const ExPolygon& growing_area, const coord_t offset, float coverage);

} /* slic3r_Print_hpp_ */

#endif
