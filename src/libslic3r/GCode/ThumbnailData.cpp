///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/ Copyright (c) Prusa Research 2019 - 2020 Lukáš Matěna @lukasmatena, Enrico Turri @enricoturri1966
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "ThumbnailData.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include <charconv>

namespace Slic3r {

void apply_thumbnail_colors(ThumbnailsParams &params, const ConfigBase &config)
{
    const ConfigOption *mode = config.option("thumbnail_color_mode");
    params.color_mode = static_cast<unsigned char>(mode ? mode->get_int() : 2);
    params.uniform_color = {1, 138, 255, 255};
    const ConfigOptionString *color = config.option<ConfigOptionString>("thumbnails_color");
    if (color && color->value.size() == 7 && color->value.front() == '#') {
        unsigned int rgb = 0;
        const char *begin = color->value.data() + 1;
        const std::from_chars_result result = std::from_chars(begin, begin + 6, rgb, 16);
        if (result.ec == std::errc() && result.ptr == begin + 6)
            params.uniform_color = {static_cast<unsigned char>(rgb >> 16),
                                    static_cast<unsigned char>(rgb >> 8),
                                    static_cast<unsigned char>(rgb), 255};
    }
}


void ThumbnailData::set(unsigned int w, unsigned int h)
{
    if ((w == 0) || (h == 0))
        return;

    if ((width != w) || (height != h))
    {
        width = w;
        height = h;
        // defaults to white texture
        pixels = std::vector<unsigned char>(width * height * 4, 255);
    }
}

void ThumbnailData::reset()
{
    width = 0;
    height = 0;
    pixels.clear();
}

bool ThumbnailData::is_valid() const
{
    return (width != 0) && (height != 0) && ((unsigned int)pixels.size() == 4 * width * height);
}

} // namespace Slic3r
