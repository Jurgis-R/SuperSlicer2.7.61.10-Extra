///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_gcode_thumbnail_h_
#define slic3r_gcode_thumbnail_h_

#define SLIC3R_PLUGIN_API_GCODE_THUMBNAIL_MAJOR 1u
#define SLIC3R_PLUGIN_API_GCODE_THUMBNAIL_MINOR 0u

#include <stdint.h>

#include "slic3r_data_tree.h"
#include "slic3r_def.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
G-code thumbnail service API
============================

The host renderer and the encoder deliberately live on opposite sides of this
contract. The host owns the OpenGL scene and returns one borrowed RGBA8 image;
the selected GCODE_THUMBNAIL provider copies or consumes that image during the
same callback and returns a complete byte block ready for the output file.

Every pointer returned through a result remains valid only until the next call
on the same renderer or encoder session. Callers must copy data they need to
retain. No allocation crosses the ABI and plugins must never free host pixels.
*/

typedef enum raw_gcode_thumbnail_status {
    RAW_GCODE_THUMBNAIL_STATUS_UNSET = 0,
    RAW_GCODE_THUMBNAIL_STATUS_SUCCESS,
    RAW_GCODE_THUMBNAIL_STATUS_SKIPPED,
    RAW_GCODE_THUMBNAIL_STATUS_INVALID_ARGUMENT,
    RAW_GCODE_THUMBNAIL_STATUS_ERROR
} raw_gcode_thumbnail_status;

typedef enum raw_gcode_thumbnail_pixel_format {
    RAW_GCODE_THUMBNAIL_PIXEL_FORMAT_RGBA8 = 0
} raw_gcode_thumbnail_pixel_format;

typedef enum raw_gcode_thumbnail_origin {
    RAW_GCODE_THUMBNAIL_ORIGIN_BOTTOM_LEFT = 0,
    RAW_GCODE_THUMBNAIL_ORIGIN_TOP_LEFT = 1
} raw_gcode_thumbnail_origin;

typedef enum raw_gcode_thumbnail_camera {
    RAW_GCODE_THUMBNAIL_CAMERA_ISOMETRIC = 0,
    RAW_GCODE_THUMBNAIL_CAMERA_FRONT,
    RAW_GCODE_THUMBNAIL_CAMERA_REAR,
    RAW_GCODE_THUMBNAIL_CAMERA_LEFT,
    RAW_GCODE_THUMBNAIL_CAMERA_RIGHT,
    RAW_GCODE_THUMBNAIL_CAMERA_TOP,
    RAW_GCODE_THUMBNAIL_CAMERA_CUSTOM
} raw_gcode_thumbnail_camera;

typedef enum raw_gcode_thumbnail_projection {
    RAW_GCODE_THUMBNAIL_PROJECTION_ORTHOGRAPHIC = 0,
    RAW_GCODE_THUMBNAIL_PROJECTION_PERSPECTIVE = 1
} raw_gcode_thumbnail_projection;

typedef enum raw_gcode_thumbnail_framing {
    RAW_GCODE_THUMBNAIL_FRAMING_AUTO = 0,
    RAW_GCODE_THUMBNAIL_FRAMING_CUSTOM = 1
} raw_gcode_thumbnail_framing;

typedef enum raw_gcode_thumbnail_color_mode {
    RAW_GCODE_THUMBNAIL_COLOR_UNIFORM = 0,
    RAW_GCODE_THUMBNAIL_COLOR_BY_EXTRUDER = 1,
    RAW_GCODE_THUMBNAIL_COLOR_BY_FILAMENT = 2
} raw_gcode_thumbnail_color_mode;

typedef enum raw_gcode_thumbnail_background {
    RAW_GCODE_THUMBNAIL_BACKGROUND_TRANSPARENT = 0,
    RAW_GCODE_THUMBNAIL_BACKGROUND_SOLID = 1
} raw_gcode_thumbnail_background;

typedef enum raw_gcode_thumbnail_placement {
    RAW_GCODE_THUMBNAIL_FILE_PREFIX = 0,
    RAW_GCODE_THUMBNAIL_BEFORE_START_GCODE = 1,
    RAW_GCODE_THUMBNAIL_FILE_SUFFIX = 2
} raw_gcode_thumbnail_placement;

typedef struct raw_gcode_thumbnail_color {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    uint8_t alpha;
} raw_gcode_thumbnail_color;

/* One rendering request. The current host ignores camera, projection, framing,
   vectors and scale: it always uses isometric orthographic automatic framing.
   These fields are reserved for future renderers. Reserved fields must be 0. */
typedef struct raw_gcode_thumbnail_render_request {
    uint32_t struct_size;
    uint32_t width;
    uint32_t height;
    raw_gcode_thumbnail_camera camera;
    raw_gcode_thumbnail_projection projection;
    raw_gcode_thumbnail_framing framing;
    raw_gcode_thumbnail_color_mode color_mode;
    raw_gcode_thumbnail_background background;
    uint8_t show_bed;
    uint8_t printable_only;
    uint8_t parts_only;
    uint8_t with_format_tag;
    raw_gcode_thumbnail_color uniform_color;
    raw_gcode_thumbnail_color background_color;
    c_vec3d camera_position;
    c_vec3d camera_target;
    c_vec3d camera_up;
    double custom_scale;
    uint32_t reserved[4];
} raw_gcode_thumbnail_render_request;

/* Borrowed pixel view. stride_bytes may exceed width * 4. origin identifies
   which visual row is stored first; pixels are never writable by the plugin. */
typedef struct raw_gcode_thumbnail_image {
    uint32_t struct_size;
    const uint8_t *pixels;
    uint64_t pixels_size;
    uint32_t width;
    uint32_t height;
    uint32_t stride_bytes;
    raw_gcode_thumbnail_pixel_format pixel_format;
    raw_gcode_thumbnail_origin origin;
} raw_gcode_thumbnail_image;

typedef struct raw_gcode_thumbnail_render_result {
    uint32_t struct_size;
    raw_gcode_thumbnail_status status;
    raw_gcode_thumbnail_image image;
    const char *error_message;
} raw_gcode_thumbnail_render_result;

typedef void (*gcode_thumbnail_render_fn)(
    void *context,
    const raw_gcode_thumbnail_render_request *request,
    raw_gcode_thumbnail_render_result *result);

/* The renderer is borrowed for one export. render() is synchronous and the
   returned pixels remain valid until the next render call on this object. */
typedef struct raw_gcode_thumbnail_renderer {
    uint32_t struct_size;
    void *context;
    gcode_thumbnail_render_fn render;
} raw_gcode_thumbnail_renderer;

typedef struct raw_gcode_thumbnail_encode_result {
    uint32_t struct_size;
    raw_gcode_thumbnail_status status;
    const uint8_t *bytes;
    uint64_t bytes_size;
    const char *error_message;
} raw_gcode_thumbnail_encode_result;

typedef void (*gcode_thumbnail_encoder_destroy_fn)(void *session);
typedef void (*gcode_thumbnail_encode_fn)(
    void *session,
    const raw_gcode_thumbnail_render_request *request,
    raw_gcode_thumbnail_encode_result *result);

typedef struct raw_gcode_thumbnail_encoder_vtable {
    uint32_t struct_size;
    gcode_thumbnail_encoder_destroy_fn destroy;
    gcode_thumbnail_encode_fn encode;
} raw_gcode_thumbnail_encoder_vtable;

typedef struct raw_gcode_thumbnail_encoder_instance {
    uint32_t struct_size;
    void *session;
    const raw_gcode_thumbnail_encoder_vtable *vtable;
} raw_gcode_thumbnail_encoder_instance;

#ifdef __cplusplus
}
#endif

#endif // slic3r_gcode_thumbnail_h_
