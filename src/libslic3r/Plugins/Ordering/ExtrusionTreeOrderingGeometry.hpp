///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_Plugins_Ordering_ExtrusionTreeOrderingGeometry_hpp_
#define slic3r_Plugins_Ordering_ExtrusionTreeOrderingGeometry_hpp_

#include <vector>

#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"

namespace slic3r_api { namespace Ordering { namespace TreeOrderingGeometry {

/*
Shared geometry mutations used by built-in extrusion-tree ordering providers.

These operations preserve existing entity handles whenever possible. Loop
rotation accepts seams on lines and arcs, splits only the segment containing
the seam, and preserves segment Z offsets. The caller owns the policy deciding
which seam, direction, or ordering should be selected.
*/

/* Reverse every local path and child order so the complete tree runs backward. */
void reverse_tree(MutableExtrusionEntity entity);

/* Rotate a fixed closed loop so its first and last point equal seam. */
void rotate_loop_to_seam(storage_handle *scratch_storage,
                         MutableExtrusionEntity loop,
                         c_point seam);

/* Remove sorting and reversal permissions from an entity and all descendants. */
void disable_ordering_flags_recursively(MutableExtrusionEntity entity);

/*
Resolve the first source LayerRegion touched by a PrintingExtrusion.

The geometric test uses the unshifted source extrusion retained by
LayerRegionIsland, not the instance-shifted plan clone. This keeps both the
path and region slices in the source object's coordinate system. Region order
supplies the deterministic result when a path crosses several regions. The
scratch storage owns only temporary Clipper results and may be cleared after
this call; the returned LayerRegion remains borrowed from the source island.
*/
LayerRegion resolve_source_layer_region(storage_handle *scratch_storage,
                                        const PrintingExtrusion &extrusion);

/*
Resolve every source LayerRegion touched by one atomic loop from the plan.

loop uses platter coordinates, while the region slices use object coordinates;
the PrintingExtrusion instance shift is removed only for this test. Results
preserve source-island order and remain borrowed after scratch data is freed.
*/
std::vector<LayerRegion> resolve_source_layer_regions(
    storage_handle *scratch_storage,
    const PrintingPlan &plan,
    const PrintingExtrusion &extrusion,
    const ExtrusionEntity &loop);

}}} // namespace slic3r_api::Ordering::TreeOrderingGeometry

#endif // slic3r_Plugins_Ordering_ExtrusionTreeOrderingGeometry_hpp_
