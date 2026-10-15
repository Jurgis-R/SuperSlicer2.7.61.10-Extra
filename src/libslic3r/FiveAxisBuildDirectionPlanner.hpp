/// Copyright (c) 2026 Jurgis-R
/// SuperSlicer is released under the terms of the AGPLv3 or higher.
#ifndef slic3r_FiveAxisBuildDirectionPlanner_hpp_
#define slic3r_FiveAxisBuildDirectionPlanner_hpp_

#include "FiveAxisKinematics.hpp"
#include "TriangleMesh.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace Slic3r {

struct FiveAxisBuildDirectionCandidate {
    FiveAxisVec3 build_up { 0.0, 0.0, 1.0 };
    double unsupported_area_ratio { 1.0 };
    double projected_height_mm { 0.0 };
    double score { std::numeric_limits<double>::infinity() };
    bool valid { false };
    std::string error;
};

struct FiveAxisBuildDirectionConfig {
    // Maximum overhang angle measured from vertical. Faces beyond this angle
    // are a crude unsupported-area proxy, not a support-generation simulation.
    double max_overhang_from_vertical_deg { 45.0 };
    double height_penalty_weight { 0.05 };
};

inline FiveAxisBuildDirectionCandidate score_five_axis_build_direction(
    const indexed_triangle_set &mesh,
    FiveAxisVec3 build_up,
    const FiveAxisBuildDirectionConfig &config = {})
{
    FiveAxisBuildDirectionCandidate result;
    if (!five_axis_finite(build_up) || !five_axis_normalize(build_up)) {
        result.error = "Build direction must be finite and non-zero.";
        return result;
    }
    if (!std::isfinite(config.max_overhang_from_vertical_deg) ||
        config.max_overhang_from_vertical_deg <= 0.0 ||
        config.max_overhang_from_vertical_deg >= 90.0 ||
        !std::isfinite(config.height_penalty_weight) ||
        config.height_penalty_weight < 0.0) {
        result.error = "Build-direction scoring settings are invalid.";
        return result;
    }
    if (mesh.vertices.empty() || mesh.indices.empty()) {
        result.error = "Cannot score an empty mesh.";
        return result;
    }

    double total_area = 0.0;
    double unsupported_area = 0.0;
    for (const Vec3i &face : mesh.indices) {
        if (face.x() < 0 || face.y() < 0 || face.z() < 0 ||
            std::size_t(face.x()) >= mesh.vertices.size() ||
            std::size_t(face.y()) >= mesh.vertices.size() ||
            std::size_t(face.z()) >= mesh.vertices.size()) {
            result.error = "Mesh contains an out-of-range triangle index.";
            return result;
        }
        const Vec3f &a = mesh.vertices[std::size_t(face.x())];
        const Vec3f &b = mesh.vertices[std::size_t(face.y())];
        const Vec3f &c = mesh.vertices[std::size_t(face.z())];
        const Vec3d cross = (b.cast<double>() - a.cast<double>()).cross(
                            c.cast<double>() - a.cast<double>());
        const double twice_area = cross.norm();
        if (!std::isfinite(twice_area)) {
            result.error = "Mesh contains non-finite triangle geometry.";
            return result;
        }
        if (twice_area <= 1e-12)
            continue;
        const double area = 0.5 * twice_area;
        const double normal_dot_up = (cross / twice_area).dot(
            Eigen::Vector3d(build_up.x, build_up.y, build_up.z));
        total_area += area;
        if (normal_dot_up < -std::sin(config.max_overhang_from_vertical_deg * 3.14159265358979323846 / 180.0))
            unsupported_area += area;
    }
    if (total_area <= 0.0 || !std::isfinite(total_area)) {
        result.error = "Mesh has no measurable triangle area.";
        return result;
    }

    double min_projection = std::numeric_limits<double>::infinity();
    double max_projection = -std::numeric_limits<double>::infinity();
    for (const Vec3f &vertex : mesh.vertices) {
        const double projection = double(vertex.x()) * build_up.x +
                                  double(vertex.y()) * build_up.y +
                                  double(vertex.z()) * build_up.z;
        if (!std::isfinite(projection)) {
            result.error = "Mesh contains non-finite vertex coordinates.";
            return result;
        }
        min_projection = std::min(min_projection, projection);
        max_projection = std::max(max_projection, projection);
    }

    result.build_up = build_up;
    result.unsupported_area_ratio = unsupported_area / total_area;
    result.projected_height_mm = max_projection - min_projection;
    result.score = result.unsupported_area_ratio +
                   config.height_penalty_weight * result.projected_height_mm;
    result.valid = true;
    return result;
}

// Generate a deterministic, approximately uniform full-sphere candidate set.
// The six cardinal directions are included explicitly; remaining samples use
// a Fibonacci sphere. This is candidate generation only, not a claim that each
// direction is reachable or collision-free on the configured machine.
inline std::vector<FiveAxisVec3> generate_five_axis_build_direction_candidates(
    std::size_t sample_count = 42)
{
    std::vector<FiveAxisVec3> candidates;
    if (sample_count < 6)
        sample_count = 6;
    candidates.reserve(sample_count + 6);
    candidates.push_back({ 1.0, 0.0, 0.0 });
    candidates.push_back({ -1.0, 0.0, 0.0 });
    candidates.push_back({ 0.0, 1.0, 0.0 });
    candidates.push_back({ 0.0, -1.0, 0.0 });
    candidates.push_back({ 0.0, 0.0, 1.0 });
    candidates.push_back({ 0.0, 0.0, -1.0 });

    const double golden_angle = 2.3999632297286533222;
    const std::size_t remaining = sample_count - 6;
    for (std::size_t i = 0; i < remaining; ++i) {
        // Midpoint sampling avoids placing additional samples exactly at poles.
        const double y = 1.0 - 2.0 * (double(i) + 0.5) / double(remaining);
        const double radius = std::sqrt(std::max(0.0, 1.0 - y * y));
        const double angle = golden_angle * double(i);
        candidates.push_back({ radius * std::cos(angle), y,
                               radius * std::sin(angle) });
    }
    return candidates;
}

// Scores candidates independently and returns the lowest-scoring valid one.
// Candidate directions must be expressed in the same frame as mesh vertices.
// This is an early geometric heuristic only: it does not model supports,
// tool accessibility, machine reachability, fixtures, or collisions.
inline FiveAxisBuildDirectionCandidate choose_five_axis_build_direction(
    const indexed_triangle_set &mesh,
    const std::vector<FiveAxisVec3> &candidates,
    const FiveAxisBuildDirectionConfig &config = {})
{
    FiveAxisBuildDirectionCandidate best;
    if (candidates.empty()) {
        best.error = "No build-direction candidates were supplied.";
        return best;
    }
    std::string last_error = "No candidate build direction was valid.";
    for (const FiveAxisVec3 &direction : candidates) {
        FiveAxisBuildDirectionCandidate scored =
            score_five_axis_build_direction(mesh, direction, config);
        if (!scored.valid) {
            last_error = scored.error;
            continue;
        }
        if (!best.valid || scored.score < best.score)
            best = std::move(scored);
    }
    if (!best.valid)
        best.error = last_error;
    return best;
}

 
// End-to-end geometry-only convenience entry point: generate candidates,
// score them, and return the best valid direction without requiring a
// caller-provided direction list.
inline FiveAxisBuildDirectionCandidate choose_five_axis_build_direction(
    const indexed_triangle_set &mesh,
    const FiveAxisBuildDirectionConfig &config = {},
    std::size_t sample_count = 42)
{
    const std::vector<FiveAxisVec3> candidates =
        generate_five_axis_build_direction_candidates(sample_count);
    return choose_five_axis_build_direction(mesh, candidates, config);
}

} // namespace Slic3r
#endif
