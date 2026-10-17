/// Copyright (c) 2026 Jurgis-R
/// SuperSlicer is released under the terms of the AGPLv3 or higher.
#ifndef slic3r_FiveAxisKinematics_hpp_
#define slic3r_FiveAxisKinematics_hpp_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>

namespace Slic3r {

struct FiveAxisVec3 {
    double x { 0.0 }, y { 0.0 }, z { 0.0 };
};

inline FiveAxisVec3 operator+(const FiveAxisVec3 &a, const FiveAxisVec3 &b)
{ return { a.x + b.x, a.y + b.y, a.z + b.z }; }
inline FiveAxisVec3 operator-(const FiveAxisVec3 &a, const FiveAxisVec3 &b)
{ return { a.x - b.x, a.y - b.y, a.z - b.z }; }
inline FiveAxisVec3 operator*(const FiveAxisVec3 &a, double s)
{ return { a.x * s, a.y * s, a.z * s }; }
inline double five_axis_dot(const FiveAxisVec3 &a, const FiveAxisVec3 &b)
{ return a.x * b.x + a.y * b.y + a.z * b.z; }
inline FiveAxisVec3 five_axis_cross(const FiveAxisVec3 &a, const FiveAxisVec3 &b)
{
    return { a.y * b.z - a.z * b.y,
             a.z * b.x - a.x * b.z,
             a.x * b.y - a.y * b.x };
}
inline double five_axis_norm(const FiveAxisVec3 &v)
{ return std::sqrt(five_axis_dot(v, v)); }
inline bool five_axis_finite(const FiveAxisVec3 &v)
{ return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }
inline bool five_axis_normalize(FiveAxisVec3 &v)
{
    const double n = five_axis_norm(v);
    if (!std::isfinite(n) || n <= 1e-12) return false;
    v = v * (1.0 / n);
    return true;
}

// Rodrigues rotation about a normalized axis. The axis is expressed in the
// parent frame; joint 2's axis is therefore evaluated after joint 1's rotation.
inline FiveAxisVec3 five_axis_rotate(const FiveAxisVec3 &value,
                                     const FiveAxisVec3 &axis,
                                     double angle_deg)
{
    const double radians = angle_deg * 3.14159265358979323846 / 180.0;
    const double c = std::cos(radians), s = std::sin(radians);
    return value * c + five_axis_cross(axis, value) * s +
           axis * (five_axis_dot(axis, value) * (1.0 - c));
}

enum class FiveAxisRotaryMount {
    RotaryBed,  // The part/surface normal rotates with the two rotary joints.
    RotaryHead  // The tool axis rotates toward the desired surface normal.
};

struct FiveAxisKinematicsConfig {
    FiveAxisRotaryMount mount { FiveAxisRotaryMount::RotaryBed };
    FiveAxisVec3 rotary_1_axis { 1.0, 0.0, 0.0 };
    FiveAxisVec3 rotary_2_axis { 0.0, 1.0, 0.0 }; // in joint-1 child frame
    // For a rotary bed this is the first joint's pivot in machine coordinates.
    // For a rotary head it is the offset from the XYZ carriage reference to
    // the first joint pivot, in carriage coordinates.
    FiveAxisVec3 rotary_1_pivot { 0.0, 0.0, 0.0 };
    // Second rotary pivot, in the first joint's child frame.
    FiveAxisVec3 rotary_2_pivot { 0.0, 0.0, 0.0 };
    // Tool-tip offset in the second joint's child frame (head mount only).
    FiveAxisVec3 tool_tip_offset { 0.0, 0.0, 0.0 };
    double rotary_1_min_deg { -180.0 };
    double rotary_1_max_deg { 180.0 };
    double rotary_2_min_deg { -90.0 };
    double rotary_2_max_deg { 90.0 };
    double orientation_tolerance { 1e-5 };
    std::size_t max_iterations { 120 };
};

struct FiveAxisJointSolution {
    bool valid { false };
    double rotary_1_deg { 0.0 };
    double rotary_2_deg { 0.0 };
    double orientation_error { 0.0 };
    std::string error;
};

inline bool validate_five_axis_kinematics_config(
    const FiveAxisKinematicsConfig &config, std::string *error = nullptr)
{
    auto fail = [error](const char *message) {
        if (error != nullptr) *error = message;
        return false;
    };
    FiveAxisVec3 axis1 = config.rotary_1_axis;
    FiveAxisVec3 axis2 = config.rotary_2_axis;
    if (!five_axis_finite(axis1) || !five_axis_finite(axis2) ||
        !five_axis_normalize(axis1) || !five_axis_normalize(axis2))
        return fail("Rotary joint axes must be finite non-zero vectors.");
    if (!five_axis_finite(config.rotary_1_pivot) ||
        !five_axis_finite(config.rotary_2_pivot) ||
        !five_axis_finite(config.tool_tip_offset))
        return fail("Rotary pivots and tool-tip offset must be finite.");
    if (!std::isfinite(config.rotary_1_min_deg) ||
        !std::isfinite(config.rotary_1_max_deg) ||
        !std::isfinite(config.rotary_2_min_deg) ||
        !std::isfinite(config.rotary_2_max_deg) ||
        config.rotary_1_min_deg >= config.rotary_1_max_deg ||
        config.rotary_2_min_deg >= config.rotary_2_max_deg)
        return fail("Rotary joint limits must be finite increasing ranges.");
    if (!std::isfinite(config.orientation_tolerance) ||
        config.orientation_tolerance <= 0.0 ||
        config.orientation_tolerance >= 0.1 ||
        config.max_iterations == 0 || config.max_iterations > 10000)
        return fail("IK tolerance or iteration budget is invalid.");
    if (error != nullptr) error->clear();
    return true;
}

inline FiveAxisVec3 five_axis_predict_normal(
    const FiveAxisKinematicsConfig &config,
    double rotary_1_deg, double rotary_2_deg,
    const FiveAxisVec3 &desired_part_normal)
{
    const FiveAxisVec3 axis1 = config.rotary_1_axis *
        (1.0 / five_axis_norm(config.rotary_1_axis));
    const FiveAxisVec3 axis2 = config.rotary_2_axis *
        (1.0 / five_axis_norm(config.rotary_2_axis));
    if (config.mount == FiveAxisRotaryMount::RotaryBed) {
        const FiveAxisVec3 after_joint2 =
            five_axis_rotate(desired_part_normal, axis2, rotary_2_deg);
        return five_axis_rotate(after_joint2, axis1, rotary_1_deg);
    }
    const FiveAxisVec3 tool_axis { 0.0, 0.0, 1.0 };
    const FiveAxisVec3 after_joint2 = five_axis_rotate(tool_axis, axis2, rotary_2_deg);
    return five_axis_rotate(after_joint2, axis1, rotary_1_deg);
}

// Numerically solves the two rotary angles for a desired surface normal. This
// solves orientation only: point coordinates must separately be transformed
// with five_axis_transform_bed_point() or five_axis_compensate_head_position().
inline FiveAxisJointSolution solve_five_axis_orientation(
    const FiveAxisKinematicsConfig &config,
    FiveAxisVec3 desired_surface_normal,
    double initial_rotary_1_deg = 0.0,
    double initial_rotary_2_deg = 0.0)
{
    FiveAxisJointSolution result;
    if (!validate_five_axis_kinematics_config(config, &result.error))
        return result;
    if (!five_axis_finite(desired_surface_normal) ||
        !five_axis_normalize(desired_surface_normal) ||
        !std::isfinite(initial_rotary_1_deg) ||
        !std::isfinite(initial_rotary_2_deg)) {
        result.error = "Desired normal or initial rotary angles are invalid.";
        return result;
    }

    const FiveAxisVec3 target = config.mount == FiveAxisRotaryMount::RotaryBed
        ? FiveAxisVec3{ 0.0, 0.0, 1.0 } : desired_surface_normal;
    double a = std::max(config.rotary_1_min_deg,
                        std::min(config.rotary_1_max_deg, initial_rotary_1_deg));
    double b = std::max(config.rotary_2_min_deg,
                        std::min(config.rotary_2_max_deg, initial_rotary_2_deg));
    const double epsilon = 1e-3;
    const double damping = 1e-6;

    auto predicted = [&](double aa, double bb) {
        if (config.mount == FiveAxisRotaryMount::RotaryBed)
            return five_axis_predict_normal(config, aa, bb, desired_surface_normal);
        const FiveAxisVec3 axis1 = config.rotary_1_axis *
            (1.0 / five_axis_norm(config.rotary_1_axis));
        const FiveAxisVec3 axis2 = config.rotary_2_axis *
            (1.0 / five_axis_norm(config.rotary_2_axis));
        return five_axis_rotate(five_axis_rotate(FiveAxisVec3{0.0, 0.0, 1.0},
                                                  axis2, bb), axis1, aa);
    };

    for (std::size_t iteration = 0; iteration < config.max_iterations; ++iteration) {
        const FiveAxisVec3 p = predicted(a, b);
        const FiveAxisVec3 residual = target - p;
        const double error = five_axis_norm(residual);
        result.orientation_error = error;
        if (error <= config.orientation_tolerance) {
            result.valid = true;
            result.rotary_1_deg = a;
            result.rotary_2_deg = b;
            result.error.clear();
            return result;
        }

        const FiveAxisVec3 ja = (predicted(std::min(config.rotary_1_max_deg, a + epsilon), b) - p) *
            (1.0 / std::max(epsilon, std::min(config.rotary_1_max_deg, a + epsilon) - a));
        const FiveAxisVec3 jb = (predicted(a, std::min(config.rotary_2_max_deg, b + epsilon)) - p) *
            (1.0 / std::max(epsilon, std::min(config.rotary_2_max_deg, b + epsilon) - b));
        const double h11 = five_axis_dot(ja, ja) + damping;
        const double h22 = five_axis_dot(jb, jb) + damping;
        const double h12 = five_axis_dot(ja, jb);
        const double g1 = five_axis_dot(ja, residual);
        const double g2 = five_axis_dot(jb, residual);
        const double determinant = h11 * h22 - h12 * h12;
        if (!std::isfinite(determinant) || determinant <= 1e-14) {
            result.error = "Inverse kinematics is singular at the current pose.";
            return result;
        }
        const double da = std::max(-15.0, std::min(15.0, (g1 * h22 - g2 * h12) / determinant));
        const double db = std::max(-15.0, std::min(15.0, (g2 * h11 - g1 * h12) / determinant));
        const double next_a = std::max(config.rotary_1_min_deg,
                                       std::min(config.rotary_1_max_deg, a + da));
        const double next_b = std::max(config.rotary_2_min_deg,
                                       std::min(config.rotary_2_max_deg, b + db));
        if (std::abs(next_a - a) + std::abs(next_b - b) <= 1e-10)
            break;
        a = next_a;
        b = next_b;
    }
    result.rotary_1_deg = a;
    result.rotary_2_deg = b;
    result.orientation_error = five_axis_norm(target - predicted(a, b));
    result.error = "Inverse kinematics did not converge within the configured limits.";
    return result;
}

// Transform a point expressed in the bed's first-joint child frame into machine
// coordinates. The point is rotated about joint 2, then joint 1.
inline FiveAxisVec3 five_axis_transform_bed_point(
    const FiveAxisKinematicsConfig &config, const FiveAxisVec3 &point,
    double rotary_1_deg, double rotary_2_deg)
{
    const FiveAxisVec3 axis1 = config.rotary_1_axis *
        (1.0 / five_axis_norm(config.rotary_1_axis));
    const FiveAxisVec3 axis2 = config.rotary_2_axis *
        (1.0 / five_axis_norm(config.rotary_2_axis));
    // The input point and both pivots are expressed in the zero-angle
    // machine frame. Rotate about joint 2 first, then rotate the resulting
    // point about joint 1. Subtract joint 1's pivot before the outer rotation
    // and add it back afterwards; otherwise a non-zero pivot translates even
    // at zero angles and produces incorrect carriage coordinates.
    const FiveAxisVec3 about_joint2 = config.rotary_2_pivot +
        five_axis_rotate(point - config.rotary_2_pivot, axis2, rotary_2_deg);
    return config.rotary_1_pivot +
        five_axis_rotate(about_joint2 - config.rotary_1_pivot, axis1, rotary_1_deg);
}

// Convert a desired tool-tip machine position to the XYZ carriage reference
// for a serial rotary-head chain. This compensates for the configured joint
// pivots and tool-tip offset; it does not account for machine-specific limits
// or collision envelopes.
inline FiveAxisVec3 five_axis_compensate_head_position(
    const FiveAxisKinematicsConfig &config, const FiveAxisVec3 &tool_tip_machine,
    double rotary_1_deg, double rotary_2_deg)
{
    const FiveAxisVec3 axis1 = config.rotary_1_axis *
        (1.0 / five_axis_norm(config.rotary_1_axis));
    const FiveAxisVec3 axis2 = config.rotary_2_axis *
        (1.0 / five_axis_norm(config.rotary_2_axis));
    const FiveAxisVec3 tool_offset_after_joint2 = config.rotary_2_pivot +
        five_axis_rotate(config.tool_tip_offset - config.rotary_2_pivot,
                         axis2, rotary_2_deg);
    // Rotate the second-joint/tool-tip offset about joint 1's pivot.
    // The pivot must be subtracted before rotation and added back afterwards;
    // otherwise the pivot is incorrectly treated as an extra tool offset.
    const FiveAxisVec3 head_offset = config.rotary_1_pivot +
        five_axis_rotate(tool_offset_after_joint2 - config.rotary_1_pivot,
                         axis1, rotary_1_deg);
    return tool_tip_machine - head_offset;
}

} // namespace Slic3r

#endif // slic3r_FiveAxisKinematics_hpp_
