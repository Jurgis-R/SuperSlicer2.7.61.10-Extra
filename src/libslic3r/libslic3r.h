///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ Copyright (c) Prusa Research 2016 - 2023 Tomáš Mészáros @tamasmeszaros, Vojtěch Bubník @bubnikv, Oleksandra Iushchenko @YuSanka, Lukáš Matěna @lukasmatena, Pavel Mikuš @Godrak, Filip Sykala @Jony01, Lukáš Hejl @hejllukas, Enrico Turri @enricoturri1966, Vojtěch Král @vojtechkral
///|/ Copyright (c) Slic3r 2013 - 2016 Alessandro Ranellucci @alranel
///|/ Copyright (c) 2016 Miro Hrončok @hroncok
///|/ Copyright (c) 2014 Kamil Kwolek
///|/
///|/ ported from xs/src/libslic3r/libslic3r.h:
///|/ Copyright (c) Prusa Research 2016 - 2019 Vojtěch Král @vojtechkral, Vojtěch Bubník @bubnikv, Enrico Turri @enricoturri1966
///|/ Copyright (c) Slic3r 2013 - 2016 Alessandro Ranellucci @alranel
///|/ Copyright (c) 2016 Miro Hrončok @hroncok
///|/ Copyright (c) 2014 Kamil Kwolek
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef _libslic3r_h_
#define _libslic3r_h_

#include "libslic3r_version.h"

// this needs to be included early for MSVC (listing it in Build.PL is not enough)
#include <array>

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <exception>

#ifdef _WIN32
// On MSVC, std::deque degenerates to a list of pointers, which defeats its purpose of reducing allocator load and memory fragmentation.
// https://github.com/microsoft/STL/issues/147#issuecomment-1090148740
// Thus it is recommended to use boost::container::deque instead.
#include <boost/container/deque.hpp>
#endif // _WIN32

#include "Technologies.hpp"
#include "Api/plugin/c/slic3r_def.h"

//for creating circles (for brim_ear)
#define POLY_SIDES 24
#define PI 3.141592653589793238
// When extruding a closed loop, the loop is interrupted and shortened a bit to reduce the seam.
//static constexpr double LOOP_CLIPPING_LENGTH_OVER_NOZZLE_DIAMETER = 0.15; now seam_gap
// Maximum perimeter length for the loop to apply the small perimeter speed. 
//#define                 SMALL_PERIMETER_LENGTH  ((6.5 / SCALING_FACTOR) * 2 * PI)
static constexpr double INSET_OVERLAP_TOLERANCE = 0.4;

inline uint16_t operator "" _u(unsigned long long value)
{
    return static_cast<uint16_t>(value);
}

#ifndef UNUSED
#define UNUSED(x) (void)(x)
#endif /* UNUSED */

// Write slices as SVG images into out directory during the 2D processing of the slices.
// #define SLIC3R_DEBUG_SLICE_PROCESSING

namespace Slic3r {

class Semver;

//using defines from slic3r_def.h
using ::coord_t;
using ::coordf_t;
using ::distf_t;
using ::distsqrf_t;
using ::coord_index_t;
using ::lengthsqr_t;
using ::extruder_index_t;

using ::EPSILON;
using ::SCALED_EPSILON;

using ::scale_i;
using ::scale_d;
using ::scale_to_layer_coord;
using ::unscaled;
using ::coord_sqr;
using ::coord_int_sqr;
using ::coord_index;

extern Semver SEMVER;

enum Axis { 
	X=0,
	Y,
	Z,
	E,
	F,
	NUM_AXES,
	// For the GCodeReader to mark a parsed axis, which is not in "XYZEF", it was parsed correctly.
	UNKNOWN_AXIS = NUM_AXES,
	NUM_AXES_WITH_UNKNOWN,
};

// from PrintConfig.hpp, but also used in extrusionentity & polyline
enum class ArcFittingType {
    Disabled,
    Bambu,
    ArcWelder
};

#ifdef _DEBUG
#define _DEBUGINFO
    #define release_assert(X) assert(X)
#else
#ifdef _RELWITHDEBINFO
#define _DEBUGINFO
inline void release_assert(bool valid) {
    if (!valid)
        throw new std::exception();
}
#endif
//error if release, as it's purely a debug thingy that need to be cleaned
#endif

#ifdef _DEBUGINFO
#ifdef WIN32
#define UNOPTIMIZE __pragma(optimize("", off))
#else
//#define UNOPTIMIZE _Pragma("optimize(\"\", off)")
#endif
#endif

} // namespace Slic3r

#endif // _libslic3r_h_
