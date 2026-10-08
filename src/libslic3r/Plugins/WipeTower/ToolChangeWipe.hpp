///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
/* The tool-change service reports one purge for every change between consecutive
   tool visits, including empty visits and layer/group boundaries, never before the
   first visit. It does not emit tool selections: downstream CreateToolChange does.
   A square matrix sized for the configured tools supplies A->B volume, floored by
   B's minimal purge. Without that matrix, A's unload plus B's load components are
   used when both exist, still floored by B's minimum; without either source the
   larger of A's and B's minima is used. Multi-nozzle volumes are not suppressed.
   The volume of the eligible object geometry that B prints first is removed from
   the purge, down to B's minimum: every leaf of an object with wipe_into_objects,
   or sparse infill of a wipe_into_infill region whose perimeters are already
   printed. Nothing is moved or reassigned to another tool, and nothing is
   measured when either filament is soluble.
   Generation writes rounded-flow connected X zigzags in scaled local coordinates.
   The first four purge lines ramp linearly from starting to main speed; a zero
   start disables the ramp. filament_max_wipe_tower_speed is a percentage of the
   incoming purge speed, or a fallback by filament type when it is not positive;
   ramming, the SEMM load and unload and the tower maintenance keep their own
   speeds.

   Outgoing part: on a single-extruder multi-material (SEMM) printer, and for tools
   with filament_multitool_ramming on a multi-nozzle printer, the request also rams
   the old filament A at the end of A's visit, at the start of the same band (the
   ramming volume is added to the purge volume). SEMM then unloads the filament to
   its parking position (cooling tube, cooling moves, skinnydip, toolchange
   temperature) and loads the new one before its purge; the cooling and loading
   moves travel along the band so the filament spreads instead of forming a blob.
   When B opens its layer, EDIT
   adds an empty visit of A at the start of that layer so the ramming is printed at
   the change Z. SEMM priming adds, before the first visit of the print, one empty
   visit per other used tool so each filament is loaded and purged once; when at
   least one change was reported, a last floating request unloads the filament
   left at the end of the print, above the tower top when the tower stopped
   lower. A print without any change reports nothing, so its end G-code unloads. */
#ifndef slic3r_Plugins_WipeTower_ToolChangeWipe_hpp_
#define slic3r_Plugins_WipeTower_ToolChangeWipe_hpp_

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"

namespace slic3r_api::WipeTower {
// Register the non-exclusive tool-change wipe service.
void register_tool_change_wipe_plugin(orchestrator_handle *orchestrator);
}
#endif
