///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Contiguous seam provider
========================
The provider propagates each finalized external seam onto directly linked
LayerIslands of the next object layer. Local enforcers and shared candidate
constraints may interrupt the propagation. All targets are finalized during
initialization, leaving placement immutable and safe for parallel callers.
*/

#ifndef slic3r_Plugins_SeamPlacer_ContiguousSeamPlacer_hpp_
#define slic3r_Plugins_SeamPlacer_ContiguousSeamPlacer_hpp_

#include <memory>

struct orchestrator_handle;
struct storage_handle;

namespace slic3r_api {
class SeamPlacerSession;

namespace SeamPlacement { namespace ContiguousPlugin {

/* Construct one uninitialized session for native tests and host registration. */
std::unique_ptr<SeamPlacerSession> make_contiguous_seam_session(
    storage_handle *storage);
/* Register the built-in provider with the shared SeamPlacer exclusive group. */
void register_contiguous_seam_placer_plugin(orchestrator_handle *orchestrator);

}} // namespace SeamPlacement::ContiguousPlugin
} // namespace slic3r_api

#endif
