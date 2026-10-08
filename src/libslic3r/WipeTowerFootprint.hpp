///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/* Generic tower envelopes shared by validation and display. Dimensions/origins
   are millimeters, rotation is degrees, and polygons use scaled platter XY.
   Instance origins use ModelInstance translation, never its rotation or the
   centered PrintInstance shift. Owners are borrowed; a null object is global. */
#ifndef slic3r_WipeTowerFootprint_hpp_
#define slic3r_WipeTowerFootprint_hpp_

#include <vector>
#include "Polygon.hpp"

namespace Slic3r {
class Print;
class PrintObject;
class Model;
class ModelObject;
class ModelInstance;
class DynamicPrintConfig;

struct WipeTowerFootprint {
    Vec2d origin;       // Platter translation in millimeters, applied after local rotation.
    double width;      // Square side in millimeters.
    double brim_width; // Resolved nonnegative brim in millimeters.
    double rotation;   // Degrees about the tower's local origin.
    Polygon polygon;   // Owned scaled platter-coordinate envelope.
    const ModelObject *object = nullptr;
    const ModelInstance *instance = nullptr;
    const PrintObject *print_object = nullptr;
};

// Build a scaled, rotated and translated square from nonnegative millimeter width/brim and a finite pose.
Polygon wipe_tower_footprint(const Vec2d &origin, double width, double brim_width, double rotation);
// Enumerate configured global and printable-instance towers from effective Print settings.
std::vector<WipeTowerFootprint> wipe_tower_footprints(const Print &print);
// Enumerate unsliced Model towers using preset defaults plus each object's overrides.
std::vector<WipeTowerFootprint> wipe_tower_footprints(const Model &model, const DynamicPrintConfig &config);
}
#endif
