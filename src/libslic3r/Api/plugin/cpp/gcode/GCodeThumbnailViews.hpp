///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_GCodeThumbnailViews_hpp_
#define slic3r_GCodeThumbnailViews_hpp_
#include <string>
#include <vector>
#include <stdexcept>
#include "libslic3r/Api/plugin/c/slic3r_gcode_firmware.h"
namespace slic3r_api {
// A non-owning service view. Call only during its export, on the serial writer
// thread; returned strings are copies and survive subsequent service calls.
class GCodeThumbnailGeneratorView {
public:
    explicit GCodeThumbnailGeneratorView(const raw_gcode_thumbnail_generator *service = nullptr) : m_service(service) {}
    std::string generate(const std::vector<raw_gcode_thumbnail_size> &sizes,
                         uint32_t format, bool with_bed, bool with_format_tag) const {
        if (m_service == nullptr) return {};
        if (m_service->struct_size < sizeof(raw_gcode_thumbnail_generator) || !m_service->generate)
            throw std::invalid_argument("Invalid thumbnail service.");
        raw_gcode_firmware_result result{};
        result.struct_size = sizeof(result);
        m_service->generate(m_service->context, sizes.data(), uint32_t(sizes.size()),
                            format, with_bed, with_format_tag, &result);
        if (result.status != RAW_GCODE_FIRMWARE_STATUS_SUCCESS)
            throw std::runtime_error(result.error_message ? result.error_message : "Thumbnail generation failed.");
        if (result.text_size && !result.text) throw std::runtime_error("Missing thumbnail text.");
        return result.text_size ? std::string(result.text, size_t(result.text_size)) : std::string();
    }
private:
    const raw_gcode_thumbnail_generator *m_service;
};
}
#endif
