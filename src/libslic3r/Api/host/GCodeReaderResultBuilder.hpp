///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_host_GCodeReaderResultBuilder_hpp_
#define slic3r_Api_host_GCodeReaderResultBuilder_hpp_

#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "libslic3r/Api/plugin/cpp/gcode/GCodeStateTracker.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"

/*
Reader events to G-code Viewer data
===================================

GCodeReaderResultBuilder is the host-side timeline consumer. A Reader parses
syntax and publishes normalized semantic events; this class turns those
events into the existing GCodeProcessorResult consumed by GCodeViewer.

    Print settings --------> initial machine state and viewer metadata
    final PrintingPlan ----> source-island areas stored in PrintStatistics
    source chunks ---------> line-end index used by the source-code window
    Reader event + state --> moves, temperatures, fan, time and material
                                |
                                v
                         GCodeProcessorResult

The builder never parses G-code text and never knows a firmware dialect. It
uses only validated Reader events and the complete state after each event.
This keeps calculations such as filament volume and nominal duration in the
host, where every Reader obtains the same viewer behavior.
Consumed volume is recorded per logical tool and extrusion role. In single-
extruder multi-material (SEMM), logical tools share one physical extruder and
one retraction debt: positive E repays earlier negative E before consumption.

Event arrays and source chunks are borrowed for one synchronous call. The
builder copies only derived values into its result. One instance represents
one input stream and must be consumed in stream order.
*/

namespace Slic3r {

class Print;
namespace Printing { struct PrintingPlan; }

class GCodeReaderResultBuilder
{
public:
    // Seed the timeline from a host-owned initial state. Print and plan are
    // either both present for a local export or both null for a standalone
    // viewer. The builder borrows them only during construction. For a local
    // export, construction also replaces PrintStatistics::_layer_area_stats
    // with the areas of the source islands still referenced by the final plan.
    GCodeReaderResultBuilder(Print *print,
                             const Printing::PrintingPlan *plan,
                             std::string reader_plugin_id,
                             const slic3r_api::GCodeStateTracker &initial_state);

    // Index newline positions while the same chunks are lent to the Reader.
    // The chunk is not retained after this call.
    void consume_chunk(std::string_view chunk);

    // Consume one event after the Reader service validated and applied it.
    // post_state is complete; event.items contains only this action's delta.
    void consume_event(const raw_gcode_reader_event &event,
                       const slic3r_api::GCodeStateTracker &post_state);

    // Complete the final unterminated line and calculated statistics, then
    // move the finished viewer result out of the builder.
    GCodeProcessorResult finish(const slic3r_api::GCodeStateTracker &final_state);

private:
    // Seed metadata and SEMM accounting from the local print, or standalone defaults.
    void initialize_result(Print *print,
                           const Printing::PrintingPlan *plan,
                           const std::string &reader_plugin_id);
    // Dispatch semantic events; each listener handles one viewer concern.
    void process_state_update(const raw_gcode_reader_event &event,
                              const slic3r_api::GCodeStateTracker &state);
    // Store the visible transition between two known logical tools.
    void process_tool_change(const raw_gcode_reader_event &event,
                             const slic3r_api::GCodeStateTracker &state,
                             uint32_t new_tool);
    // Validate one zero-based Reader instance ID and prepare its viewer legend
    // entry. UINT32_MAX is outside every object and is represented in a
    // MoveVertex by GCodeProcessorResult::OBJECT_INSTANCE_ID_NONE. Reader
    // results therefore do not contain an artificial outside entry.
    void process_object_change(uint32_t object_instance_id);
    void process_linear_move(const raw_gcode_reader_event &event,
                             const slic3r_api::GCodeStateTracker &state);
    void process_arc_move(const raw_gcode_reader_event &event,
                          const slic3r_api::GCodeStateTracker &state);
    void process_position_change(const raw_gcode_reader_event &event,
                                 const slic3r_api::GCodeStateTracker &state);
    void process_temperature(const raw_gcode_reader_event &event,
                             const slic3r_api::GCodeStateTracker &state);
    void process_marker(const raw_gcode_reader_event &event,
                        const slic3r_api::GCodeStateTracker &state);
    void process_diagnostic(const raw_gcode_reader_event &event,
                            const slic3r_api::GCodeStateTracker &state);
    void process_invalidation(const raw_gcode_reader_event &event,
                              const slic3r_api::GCodeStateTracker &state);

    // Discard retraction debt when its tool E state is no longer trustworthy.
    void invalidate_derived_state(const raw_gcode_reader_event &event,
                                  const slic3r_api::GCodeStateTracker &state);

    // Store one linearized endpoint and accumulate consumption by role and logical tool.
    void append_move(const raw_gcode_reader_event &event,
                     const slic3r_api::GCodeStateTracker &state,
                     const Vec3d &start,
                     const Vec3d &end,
                     double start_e,
                     double end_e,
                     bool internal_only);
    // Extend logical-tool metadata and allocate its independent or shared debt slot.
    void ensure_tool(uint32_t tool);
    // Rebuild the compact vectors expected by the viewer from running totals.
    void finalize_statistics();

    GCodeProcessorResult m_result;
    slic3r_api::GCodeStateTracker m_previous_state;
    std::array<float, static_cast<size_t>(EMoveType::Count)> m_move_times{};
    std::array<float, static_cast<size_t>(GCodeExtrusionRole::Count)> m_role_times{};
    std::vector<float> m_layer_times;
    std::vector<double> m_color_volumes;
    // Debt is filament length in mm. SEMM uses only slot 0; otherwise each
    // logical tool has its own slot. Shared debt follows the active tool's
    // E-known state. E changes without a known active tool discard all debt.
    std::vector<double> m_retracted_filament;
    bool m_single_extruder_multi_material = false;
    uint64_t m_source_size = 0;
    float m_total_time = 0.0f;
    uint32_t m_current_color = 0;
    bool m_has_object_instance_catalog = false;
    bool m_source_ends_with_newline = true;
    bool m_finished = false;
};

} // namespace Slic3r

#endif // slic3r_Api_host_GCodeReaderResultBuilder_hpp_
