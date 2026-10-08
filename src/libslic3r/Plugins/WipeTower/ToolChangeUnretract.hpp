///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
/* wipe_tower.toolchange_unretract.default rewrites the semantic tool-change unretraction of a
   visit that starts on the wipe tower, so the new filament is restored while wiping the start
   of the tower path.

   The chain stays split on purpose, and the retraction providers never learn about the tower:
   CreateTransitionScope marks the tool-change boundary of the first scope of the incoming
   visit, CreateRetraction writes there a point-less UNRETRACT event carrying
   retract_restart_extra_toolchange and toolchange = 1, and this plugin replaces that event with
   an in-place partial unretraction followed by a geometric Wipe leaf whose unretract request
   the firmware spreads over the path. DefaultLayerEntryState and the travel providers then aim
   the travel at the first wipe point, which is the first point of the tower content, so the
   tower content continues where the wipe ends.

   The wipe follows the first printable leaf of the tower content with whole round trips that
   end at its first point, so a leaf shorter than half the requested length is traversed several
   times. Its length comes from the wiped filament volume divided by the leaf flow, and its XY
   speed is the deretraction speed scaled by that length, capped by the fastest sparse-infill or
   support speed of the tower configuration. Firmware retraction and single-extruder
   multi-material keep the original in-place unretraction: G10/G11 cannot restore a partial
   amount, and the SEMM swap has no tool-change unretraction to rewrite. */
#ifndef slic3r_Plugins_WipeTower_ToolChangeUnretract_hpp_
#define slic3r_Plugins_WipeTower_ToolChangeUnretract_hpp_

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"

namespace slic3r_api { namespace WipeTower { namespace ToolChangeUnretractPlugin {
// Register the non-exclusive tool-change unretraction wipe plugin.
void register_tool_change_unretract_plugin(orchestrator_handle *orchestrator);
}}} // namespace slic3r_api::WipeTower::ToolChangeUnretractPlugin
#endif
