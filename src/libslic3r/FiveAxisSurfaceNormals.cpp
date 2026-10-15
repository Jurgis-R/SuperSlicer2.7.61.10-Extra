/// Copyright (c) 2026 Jurgis-R
/// SuperSlicer is released under the terms of the AGPLv3 or higher.

#include "FiveAxisSurfaceNormals.hpp"

#include "Model.hpp"
#include "Print.hpp"

namespace Slic3r {

FiveAxisValidationResult transform_five_axis_print_volume_mesh(
    const PrintObject &print_object,
    const ModelVolume &volume,
    indexed_triangle_set &output)
{
    // This is the transform composition used by PrintObject::slice_volumes()
    // when slicing each ModelVolume into the centered print-object frame.
    return transform_five_axis_surface_mesh(
        volume.mesh().its,
        print_object.trafo_centered() * volume.get_matrix(),
        output);
}

} // namespace Slic3r
