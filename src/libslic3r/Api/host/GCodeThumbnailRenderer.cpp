///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "GCodeThumbnailRenderer.hpp"

#include <stdexcept>

#include "libslic3r/Api/plugin/cpp/gcode/GCodeThumbnailViews.hpp"
#include "libslic3r/Print.hpp"

/*
Host thumbnail renderer bridge
==============================

The plugin-visible callback remains independent from wxWidgets and OpenGL.
It converts one validated request to ThumbnailsParams, invokes the export's
GUI callback synchronously, then exposes the first exact-size RGBA8 image as a
borrowed view. A missing GUI callback is an explicit SKIPPED result so optional
thumbnail profiles can continue without special exceptions.
*/

namespace Slic3r {

GCodeThumbnailRenderer::GCodeThumbnailRenderer(
    const Print &print,
    ThumbnailsGeneratorCallback callback) :
    m_print(print), m_callback(std::move(callback)),
    m_renderer{sizeof(m_renderer), this, &render}
{}

void GCodeThumbnailRenderer::render(
    void *opaque,
    const raw_gcode_thumbnail_render_request *request,
    raw_gcode_thumbnail_render_result *result) noexcept
{
    if (result == nullptr || result->struct_size < sizeof(raw_gcode_thumbnail_render_result))
        return;
    result->status = RAW_GCODE_THUMBNAIL_STATUS_UNSET;
    result->image = {};
    result->image.struct_size = sizeof(result->image);
    result->error_message = nullptr;
    if (opaque == nullptr || request == nullptr ||
        request->struct_size < sizeof(raw_gcode_thumbnail_render_request)) {
        result->status = RAW_GCODE_THUMBNAIL_STATUS_INVALID_ARGUMENT;
        return;
    }

    GCodeThumbnailRenderer &owner = *static_cast<GCodeThumbnailRenderer *>(opaque);
    owner.m_image.reset();
    owner.m_error.clear();
    try {
        slic3r_api::validate_gcode_thumbnail_render_request(*request);
        if (!owner.m_callback) {
            result->status = RAW_GCODE_THUMBNAIL_STATUS_SKIPPED;
            return;
        }

        // Camera fields are reserved; this host currently renders automatic views only.
        const ThumbnailsParams params = {
            {Vec2d(request->width, request->height)},
            request->printable_only != 0,
            request->parts_only != 0,
            request->show_bed != 0,
            request->background == RAW_GCODE_THUMBNAIL_BACKGROUND_TRANSPARENT,
            uint8_t(RAW_GCODE_THUMBNAIL_CAMERA_ISOMETRIC),
            uint8_t(RAW_GCODE_THUMBNAIL_PROJECTION_ORTHOGRAPHIC),
            uint8_t(RAW_GCODE_THUMBNAIL_FRAMING_AUTO),
            uint8_t(request->color_mode),
            {request->uniform_color.red, request->uniform_color.green,
             request->uniform_color.blue, request->uniform_color.alpha},
            {request->background_color.red, request->background_color.green,
             request->background_color.blue, request->background_color.alpha},
            Vec3d(1.0, -1.0, 1.0),
            Vec3d(0.0, 0.0, 0.0),
            Vec3d(0.0, 0.0, 1.0),
            1.0
        };
        owner.m_print.throw_if_canceled();
        ThumbnailsList images = owner.m_callback(params);
        owner.m_print.throw_if_canceled();
        if (images.empty() || !images.front().is_valid())
            throw std::runtime_error("The graphical renderer returned no thumbnail image.");
        owner.m_image = std::move(images.front());
        if (owner.m_image.width != request->width || owner.m_image.height != request->height)
            throw std::runtime_error("The graphical renderer returned unexpected dimensions.");

        result->image.pixels = owner.m_image.pixels.data();
        result->image.pixels_size = owner.m_image.pixels.size();
        result->image.width = owner.m_image.width;
        result->image.height = owner.m_image.height;
        result->image.stride_bytes = owner.m_image.width * 4u;
        result->image.pixel_format = RAW_GCODE_THUMBNAIL_PIXEL_FORMAT_RGBA8;
        result->image.origin = RAW_GCODE_THUMBNAIL_ORIGIN_BOTTOM_LEFT;
        result->status = RAW_GCODE_THUMBNAIL_STATUS_SUCCESS;
    } catch (const std::exception &error) {
        owner.m_error = error.what();
        result->status = RAW_GCODE_THUMBNAIL_STATUS_ERROR;
        result->error_message = owner.m_error.c_str();
    } catch (...) {
        owner.m_error = "Thumbnail rendering failed with an unknown exception.";
        result->status = RAW_GCODE_THUMBNAIL_STATUS_ERROR;
        result->error_message = owner.m_error.c_str();
    }
}

} // namespace Slic3r
