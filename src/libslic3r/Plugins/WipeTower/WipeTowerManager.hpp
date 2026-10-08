///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/* The built-in manager chooses config-derived global/instance towers, reserves
   local bands and tower layers, generates maintenance, and publishes fixed-order
   service/maintenance roots into the host-owned plan after all service calls.
   Every generated request is bracketed by one 100% speed-ratio save and restore,
   which also cover its tool change.
   A floating request above the tower top gets its own band layer there, without
   maintenance, and never grows the tower's depth or cone.
   Forced contour visits stop at the owner's last tool-change layer in the
   unmodified plan (including changes across layers). Above it, an absent contour
   tool is not added; maintenance uses the first tower visit's tool. Only towers
   with printed layers receive synthetic catalog entries. Print-ordered owner
   groups may share a top layer or stack above it when source Z falls; inserted
   roots carry the difference from their anchor group's Z as a Z offset. */
#ifndef slic3r_Plugins_WipeTower_WipeTowerManager_hpp_
#define slic3r_Plugins_WipeTower_WipeTowerManager_hpp_

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"

namespace slic3r_api::WipeTower {
// Register the default exclusive whole-plan wipe-tower manager.
void register_wipe_tower_manager_plugin(orchestrator_handle *orchestrator);
}

#endif
