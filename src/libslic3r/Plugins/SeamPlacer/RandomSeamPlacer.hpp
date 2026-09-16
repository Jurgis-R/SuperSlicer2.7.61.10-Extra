///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
RandomSeamPlacer: registration and session ownership
==================================================
This header exposes the two entry points of the built-in Random seam provider.
Registration installs its metadata and callbacks in the orchestrator; it does
not create a session or take ownership of a printing plan.

To use the factory directly, include SeamPlacerViews.hpp for the complete session
type. Pass the returned unique_ptr to make_seam_placer_instance(), together with
the host-owned plan and assigned PrintRegions. The adapter initializes the session
before publishing it through the C service and destroys it if initialization fails.
The resulting SeamPlacer wrapper owns the published session and releases it on
destruction. Callers must not free the session or its caches a second time.

Storage is borrowed for initialization and must remain alive until that call
finishes. The optional entropy callback is used only for objects whose seed is
zero; it permits deterministic tests without changing the production random
source. Initialization releases temporary storage allocations and the callback.
Published sessions retain immutable seam coordinates indexed by borrowed loop
and region identities. The host must keep those identities alive; new loops or
changed geometry/configuration require a new session. Concurrent seam requests
only read the cache and never alter the host's extrusion tree.

The normal execution path is:

    register_random_seam_placer_plugin()
    `-- RandomProvider::run_impl()
        `-- make_random_seam_session()
            `-- RandomSession::initialize()
                |-- collect_loops()
                |-- make_domain() and random_seam()
                `-- align_loops()

After initialization, RandomSession::place_seam() only looks up the stored
point. It does not advance the generator or inspect mutable geometry.
*/

#ifndef slic3r_RandomSeamPlacer_hpp_
#define slic3r_RandomSeamPlacer_hpp_
#include "libslic3r/Api/plugin/c/slic3r_plugin_types.h"
#include <cstdint>
#include <functional>
#include <memory>

namespace slic3r_api {
class SeamPlacerSession;
namespace SeamPlacement { namespace RandomPlugin {
/* Own an unpublished session. initialize() borrows storage temporarily; the
   optional entropy source is called once per zero-seeded object, only during
   initialization. An empty source uses the platform random device. This seam
   factory also allows deterministic validation of seed-zero behavior. */
std::unique_ptr<SeamPlacerSession> make_random_seam_session(
    storage_handle *storage, std::function<uint64_t()> entropy = {});
/* Register process-wide metadata and callbacks; it creates no plan session. */
void register_random_seam_placer_plugin(orchestrator_handle *orchestrator);
}}}
#endif
