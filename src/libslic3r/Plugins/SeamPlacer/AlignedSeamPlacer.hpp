///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Aligned seam provider
=====================
The provider prepares immutable per-external-perimeter anchors, follows only
LayerIsland upper/lower overlap links, and smooths chains of at least six
adjacent layers. Placement projects the prepared object-local target onto the
requested real line/arc loop after restoring its instance shift.
*/

#ifndef slic3r_Plugins_SeamPlacer_AlignedSeamPlacer_hpp_
#define slic3r_Plugins_SeamPlacer_AlignedSeamPlacer_hpp_

#include <memory>

struct orchestrator_handle;
struct storage_handle;

namespace slic3r_api {
class SeamPlacerSession;

namespace SeamPlacement { namespace AlignedPlugin {

std::unique_ptr<SeamPlacerSession> make_aligned_seam_session(storage_handle *storage);
void register_aligned_seam_placer_plugin(orchestrator_handle *orchestrator);

}} // namespace SeamPlacement::AlignedPlugin
} // namespace slic3r_api

#endif
