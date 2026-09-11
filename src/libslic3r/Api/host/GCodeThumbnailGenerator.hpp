///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_GCodeThumbnailGenerator_hpp_
#define slic3r_GCodeThumbnailGenerator_hpp_
#include <string>
#include "libslic3r/GCode/ThumbnailData.hpp"
#include "libslic3r/Api/plugin/c/slic3r_gcode_firmware.h"
namespace Slic3r {
class Print;
// One export owns this bridge until after its firmware session is destroyed.
// Renders synchronously through the caller's callback; never chooses placement.
class GCodeThumbnailGenerator {
public:
    GCodeThumbnailGenerator(const Print &print, ThumbnailsGeneratorCallback callback);
    GCodeThumbnailGenerator(const GCodeThumbnailGenerator &) = delete;
    GCodeThumbnailGenerator &operator=(const GCodeThumbnailGenerator &) = delete;
    const raw_gcode_thumbnail_generator *c_generator() const { return &m_service; }
private:
    static void generate(void *, const raw_gcode_thumbnail_size *, uint32_t, uint32_t,
                         uint8_t, uint8_t, raw_gcode_firmware_result *) noexcept;
    const Print &m_print;
    ThumbnailsGeneratorCallback m_callback;
    std::string m_text, m_error;
    raw_gcode_thumbnail_generator m_service;
};
}
#endif
