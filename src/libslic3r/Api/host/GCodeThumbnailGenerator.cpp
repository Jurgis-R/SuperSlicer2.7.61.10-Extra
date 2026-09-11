///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
// Host thumbnail bridge: render through the export callback, compress to the
// requested format and retain each textual result until the next serial call.
// Exceptions become C results; the caller owns placement and file publication.
#include "GCodeThumbnailGenerator.hpp"
#include "libslic3r/GCode/Thumbnails.hpp"
#include "libslic3r/Print.hpp"
#include <stdexcept>
namespace Slic3r {
// The raw table borrows this stable owner; copying it would invalidate context.
GCodeThumbnailGenerator::GCodeThumbnailGenerator(const Print &print, ThumbnailsGeneratorCallback callback) :
    m_print(print), m_callback(std::move(callback)), m_service{sizeof(m_service), this, &generate} {}

// Exceptions, including cancellation, become a failed result at the C boundary.
// The writer aborts publication; the host Print retains its cancellation state.
void GCodeThumbnailGenerator::generate(void *opaque, const raw_gcode_thumbnail_size *sizes,
    uint32_t count, uint32_t format, uint8_t with_bed, uint8_t with_tag,
    raw_gcode_firmware_result *result) noexcept
{
    if (!result || result->struct_size < sizeof(*result)) return;
    result->text = nullptr; result->text_size = 0; result->error_message = nullptr;
    result->status = RAW_GCODE_FIRMWARE_STATUS_ERROR;
    if (!opaque) return;
    GCodeThumbnailGenerator &owner = *static_cast<GCodeThumbnailGenerator *>(opaque);
    owner.m_text.clear(); owner.m_error.clear();
    try {
        if ((count && !sizes) || format > 3 || with_bed > 1 || with_tag > 1)
            throw std::invalid_argument("Invalid thumbnail parameters.");
        std::vector<Vec2d> dimensions;
        dimensions.reserve(count);
        for (uint32_t idx = 0; idx < count; ++idx) {
            if (sizes[idx].width == 0 || sizes[idx].height == 0)
                throw std::invalid_argument("Thumbnail dimensions must be positive.");
            dimensions.emplace_back(sizes[idx].width, sizes[idx].height);
        }
        owner.m_print.throw_if_canceled();
        GCodeThumbnails::export_thumbnails_to_file(owner.m_callback, dimensions, with_bed != 0,
            static_cast<GCodeThumbnailsFormat>(format), with_tag != 0,
            [&](const char *text) { owner.m_text += text; },
            [&]() { owner.m_print.throw_if_canceled(); });
        result->text = owner.m_text.data(); result->text_size = owner.m_text.size();
        result->status = RAW_GCODE_FIRMWARE_STATUS_SUCCESS;
    } catch (const std::exception &error) { owner.m_error = error.what(); }
      catch (...) { owner.m_error = "Thumbnail generation failed."; }
    if (result->status != RAW_GCODE_FIRMWARE_STATUS_SUCCESS)
        result->error_message = owner.m_error.c_str();
}
}
