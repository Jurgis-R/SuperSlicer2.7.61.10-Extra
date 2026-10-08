///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/ Copyright (c) Prusa Research 2017 - 2019 Lukáš Matěna @lukasmatena, Vojtěch Bubník @bubnikv
///|/ Copyright (c) 2019 Thomas Moore
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_PrintExtents_hpp_
#define slic3r_PrintExtents_hpp_

/* Extent queries project skirt/brim and object paths onto XY, including line
   widths and instance translations. They return value-owned bounding boxes. */


#include "libslic3r.h"

namespace Slic3r {

class Print;
class PrintObject;
class BoundingBoxf;

// Returns a bounding box of a projection of the brim and skirt.
BoundingBoxf get_print_extrusions_extents(const Print &print);

// Returns a bounding box of a projection of the object extrusions at z <= max_print_z.
BoundingBoxf get_print_object_extrusions_extents(const PrintObject &print_object, const coord_t max_print_z);


};

#endif /* slic3r_PrintExtents_hpp_ */
