///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/ Copyright (c) Prusa Research 2019 - 2021 Vojtěch Bubník @bubnikv, Lukáš Matěna @lukasmatena, Enrico Turri @enricoturri1966
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_ThumbnailData_hpp_
#define slic3r_ThumbnailData_hpp_

#include <array>
#include <functional>
#include <vector>

#include "libslic3r/Point.hpp"

namespace Slic3r {

struct ThumbnailData
{
    unsigned int width;
    unsigned int height;
    std::vector<unsigned char> pixels;

    ThumbnailData() { reset(); }
    void set(unsigned int w, unsigned int h);
    void reset();

    bool is_valid() const;
};

using ThumbnailsList = std::vector<ThumbnailData>;

struct ThumbnailsParams
{
    const Vec2ds    sizes;
    bool            printable_only;
    bool            parts_only;
    bool            show_bed;
    bool            transparent_background;
    // Extended G-code plugin requests. Existing aggregate initializers keep
    // the legacy isometric/orthographic and filament-color defaults.
    unsigned char   camera { 0 };
    unsigned char   projection { 0 };
    unsigned char   framing { 0 };
    unsigned char   color_mode { 2 };
    std::array<unsigned char, 4> uniform_color { 1, 138, 255, 255 };
    std::array<unsigned char, 4> background_color { 255, 255, 255, 255 };
    Vec3d           camera_position { Vec3d::Zero() };
    Vec3d           camera_target { Vec3d::Zero() };
    Vec3d           camera_up { Vec3d(0., 0., 1.) };
    double          custom_scale { 1.0 };
};

// Normalize native profile colors before calling a renderer. Explicit service
// requests already carry their colors and must not be overwritten with a profile.
class ConfigBase;
void apply_thumbnail_colors(ThumbnailsParams &params, const ConfigBase &config);

typedef std::function<ThumbnailsList(const ThumbnailsParams&)> ThumbnailsGeneratorCallback;

} // namespace Slic3r

#endif // slic3r_ThumbnailData_hpp_
