///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
/* Plain C value types shared by geometry and mesh queries. No host helpers or
   C++ definitions are included; layout is independent of the coordinate scale. */
#ifndef slic3r_geometry_types_h_
#define slic3r_geometry_types_h_
#define SLIC3R_PLUGIN_API_GEOMETRY_TYPES_MAJOR 1u
#define SLIC3R_PLUGIN_API_GEOMETRY_TYPES_MINOR 0u
#include <stdint.h>
#ifndef SLIC3R_HOST_API
#if defined(_WIN32) && defined(SLIC3R_HOST_EXPORTS)
#define SLIC3R_HOST_API __declspec(dllexport)
#else
#define SLIC3R_HOST_API
#endif
#endif
typedef struct c_vec3f
{
    float x;
    float y;
    float z;
} c_vec3f;

typedef struct c_vec2d
{
    double x;
    double y;
} c_vec2d;

typedef struct c_vec3d
{
    double x;
    double y;
    double z;
} c_vec3d;

/*
4x4 transform matrix in row-major order:
    value[row * 4 + column]

It mirrors Eigen::Transform3d::matrix(), but keeps the ABI strictly C.
The matrix values are unscaled floating-point 3D coordinates. Do not use
coord_t here: coord_t is for scaled 2D/height coordinates in the slicer space.
*/
typedef struct c_matrix4d
{
    double value[16];
} c_matrix4d;

#endif
