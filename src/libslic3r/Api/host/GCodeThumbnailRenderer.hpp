///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_GCodeThumbnailRenderer_hpp_
#define slic3r_GCodeThumbnailRenderer_hpp_

#include <string>

#include "libslic3r/Api/plugin/c/slic3r_gcode_thumbnail.h"
#include "libslic3r/GCode/ThumbnailData.hpp"

namespace Slic3r {

class Print;

/* One export owns this synchronous bridge. It translates ABI render requests
   to the GUI callback and retains the resulting pixel vector until the next
   call. It never compresses, frames, places or writes thumbnail data. */
class GCodeThumbnailRenderer
{
public:
    GCodeThumbnailRenderer(const Print &print, ThumbnailsGeneratorCallback callback);
    GCodeThumbnailRenderer(const GCodeThumbnailRenderer &) = delete;
    GCodeThumbnailRenderer &operator=(const GCodeThumbnailRenderer &) = delete;

    const raw_gcode_thumbnail_renderer *c_renderer() const { return &m_renderer; }

private:
    static void render(void *opaque,
                       const raw_gcode_thumbnail_render_request *request,
                       raw_gcode_thumbnail_render_result *result) noexcept;

    const Print &m_print;
    ThumbnailsGeneratorCallback m_callback;
    ThumbnailData m_image;
    std::string m_error;
    raw_gcode_thumbnail_renderer m_renderer;
};

} // namespace Slic3r

#endif // slic3r_GCodeThumbnailRenderer_hpp_
