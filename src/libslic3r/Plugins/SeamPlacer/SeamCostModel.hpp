///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Cost seam model
===============
This private implementation surface contains the scalar and geometric costs
used by the built-in Cost seam provider. The model deliberately lives beside
the provider: it has no dependency on the legacy GCode seam placer and is not
part of the plugin ABI.
*/

#ifndef libslic3r_Plugins_SeamPlacer_SeamCostModel_hpp_
#define libslic3r_Plugins_SeamPlacer_SeamCostModel_hpp_

#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_geometry.h"

namespace slic3r_api {
class Object;
}

namespace Slic3r {
namespace SeamCostModel {

float gauss(float value, float mean_x_coord, float mean_value, float falloff_speed);
float compute_angle_penalty(float ccw_angle);
std::vector<float> calculate_cost_angles(const std::vector<c_point> &points, float min_arm_length);
std::vector<float> calculate_cost_visibility(const slic3r_api::Object &object,
                                             const std::vector<c_vec3f> &points);

} // namespace SeamCostModel
} // namespace Slic3r

#endif
