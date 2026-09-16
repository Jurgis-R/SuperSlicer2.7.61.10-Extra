///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ Copyright (c) Prusa Research 2019 - 2022 Pavel Mikuš @Godrak, Tomáš Mészáros @tamasmeszaros, Vojtěch Bubník @bubnikv, Lukáš Matěna @lukasmatena
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
// Compatibility facade: native callers retain the historic traversal tolerance.
// The implementation is shared with the standalone plugin SDK utility.
#ifndef slic3r_KDTreeIndirect_hpp_
#define slic3r_KDTreeIndirect_hpp_
#include "libslic3r.h"
#include "Api/plugin/cpp/KDTreeIndirect.hpp"
namespace Slic3r {
using slic3r_api::VisitorReturnMask;
using slic3r_api::find_closest_point;
using slic3r_api::find_closest_points;
using slic3r_api::find_nearby_points;
template<size_t D, typename C, typename F>
class KDTreeIndirect : public slic3r_api::KDTreeIndirect<D, C, F> {
    using Base = slic3r_api::KDTreeIndirect<D, C, F>;
    static constexpr double tolerance = std::is_floating_point<C>::value ? EPSILON : SCALED_EPSILON;
public:
    explicit KDTreeIndirect(F f) : Base(std::move(f), size_t(0), tolerance) {}
    KDTreeIndirect(F f, size_t count) : Base(std::move(f), count, tolerance) {}
    KDTreeIndirect(F f, std::vector<size_t> indices) : Base(std::move(f), std::move(indices), tolerance) {}
};
}
#endif
