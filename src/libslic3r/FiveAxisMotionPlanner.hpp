/// Copyright (c) 2026 Jurgis-R
/// SuperSlicer is released under the terms of the AGPLv3 or higher.
#ifndef slic3r_FiveAxisMotionPlanner_hpp_
#define slic3r_FiveAxisMotionPlanner_hpp_

#include "FiveAxisEngine.hpp"
#include "FiveAxisKinematics.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace Slic3r {

// Work-coordinate sample from a geometry/toolpath stage. The normal is the
// requested tool-axis direction; it is not a machine rotary-angle pair.
struct FiveAxisToolPose {
    FiveAxisVec3 point;
    FiveAxisVec3 normal { 0.0, 0.0, 1.0 };
    double extrusion { 0.0 };
    double feed_mm_min { 0.0 };
};

// Axis-aligned machine-space obstacle. Supply conservative boxes for fixtures,
// the bed, printed geometry, and other forbidden regions. This is a sampled
// clearance guard, not an exact mesh/swept-volume collision proof.
struct FiveAxisAabb {
    FiveAxisVec3 min;
    FiveAxisVec3 max;
};

struct FiveAxisCollisionConfig {
    std::vector<FiveAxisAabb> obstacles;
    double tool_clearance_radius_mm { 0.0 };
    double max_sample_spacing_mm { 1.0 };
    std::size_t max_samples_per_segment { 100000 };
};

inline bool five_axis_point_inside_expanded_aabb(
    const FiveAxisVec3 &p, const FiveAxisAabb &box, double radius)
{
    return p.x >= box.min.x - radius && p.x <= box.max.x + radius &&
           p.y >= box.min.y - radius && p.y <= box.max.y + radius &&
           p.z >= box.min.z - radius && p.z <= box.max.z + radius;
}

inline FiveAxisValidationResult validate_five_axis_collision_config(
    const FiveAxisCollisionConfig &config)
{
    if (!std::isfinite(config.tool_clearance_radius_mm) ||
        config.tool_clearance_radius_mm < 0.0 ||
        !std::isfinite(config.max_sample_spacing_mm) ||
        config.max_sample_spacing_mm <= 0.0 ||
        config.max_samples_per_segment == 0)
        return { false, "Collision clearance must be non-negative and sample spacing positive." };
    for (const FiveAxisAabb &box : config.obstacles) {
        if (!five_axis_finite(box.min) || !five_axis_finite(box.max) ||
            box.min.x > box.max.x || box.min.y > box.max.y || box.min.z > box.max.z)
            return { false, "Collision obstacle has invalid or non-finite bounds." };
    }
    return { true, {} };
}

// Checks the interpolated XYZ tool-reference path against expanded obstacle
// boxes. It is intentionally conservative and only as accurate as the supplied
// obstacle geometry and chosen sample spacing.
inline FiveAxisValidationResult validate_five_axis_path_clearance(
    const FiveAxisCollisionConfig &config,
    const std::vector<FiveAxisCommand> &path)
{
    const auto config_result = validate_five_axis_collision_config(config);
    if (!config_result.valid)
        return config_result;
    if (path.empty())
        return { false, "Cannot collision-check an empty path." };

    for (std::size_t i = 0; i < path.size(); ++i) {
        const FiveAxisCommand &a = path[i];
        if (!five_axis_finite_command(a))
            return { false, "Collision check received an invalid motion command." };
        if (i == 0) {
            for (const FiveAxisAabb &box : config.obstacles)
                if (five_axis_point_inside_expanded_aabb(
                        {a.x, a.y, a.z}, box, config.tool_clearance_radius_mm))
                    return { false, "Tool reference intersects a configured obstacle." };
            continue;
        }

        const FiveAxisCommand &b = path[i - 1];
        const FiveAxisVec3 start { b.x, b.y, b.z };
        const FiveAxisVec3 delta { a.x - b.x, a.y - b.y, a.z - b.z };
        const double distance = five_axis_norm(delta);
        const double requested_steps = std::ceil(distance / config.max_sample_spacing_mm);
        if (!std::isfinite(requested_steps) ||
            requested_steps > static_cast<double>(config.max_samples_per_segment))
            return { false, "Collision segment exceeds the configured sampling budget." };
        const std::size_t steps = std::max<std::size_t>(
            1, static_cast<std::size_t>(requested_steps));
        for (std::size_t sample = 1; sample <= steps; ++sample) {
            const FiveAxisVec3 p = start + delta * (static_cast<double>(sample) / steps);
            for (const FiveAxisAabb &box : config.obstacles)
                if (five_axis_point_inside_expanded_aabb(
                        p, box, config.tool_clearance_radius_mm))
                    return { false, "Interpolated tool path intersects a configured obstacle." };
        }
    }
    return { true, {} };
}

// Rotary-head carriage XYZ alone is not a collision envelope: the pivots and
// tool shaft sweep through space while the rotary joints move. Sample the
// carriage-to-joint1, joint1-to-joint2, and joint2-to-tip segments at interpolated
// XYZ/rotary poses. This is conservative only for the configured line-segment
// model and clearance radius; bulky head geometry must be represented by a
// larger clearance radius or additional obstacle/envelope data.
inline FiveAxisValidationResult validate_five_axis_head_swept_clearance(
    const FiveAxisKinematicsConfig &kinematics,
    const FiveAxisCollisionConfig &config,
    const std::vector<FiveAxisCommand> &path)
{
    const auto config_result = validate_five_axis_collision_config(config);
    if (!config_result.valid)
        return config_result;
    std::string kinematics_error;
    if (!validate_five_axis_kinematics_config(kinematics, &kinematics_error))
        return { false, "Invalid kinematics for swept-head collision check: " + kinematics_error };
    if (path.empty())
        return { false, "Cannot collision-check an empty swept-head path." };

    const FiveAxisVec3 axis1 = kinematics.rotary_1_axis *
        (1.0 / five_axis_norm(kinematics.rotary_1_axis));
    const FiveAxisVec3 axis2 = kinematics.rotary_2_axis *
        (1.0 / five_axis_norm(kinematics.rotary_2_axis));
    std::size_t sample_budget = 0;
    auto check_point = [&](const FiveAxisVec3 &p) -> bool {
        for (const FiveAxisAabb &box : config.obstacles)
            if (five_axis_point_inside_expanded_aabb(p, box, config.tool_clearance_radius_mm))
                return false;
        return true;
    };
    auto head_points = [&](const FiveAxisCommand &pose, FiveAxisVec3 &carriage,
                           FiveAxisVec3 &joint1, FiveAxisVec3 &joint2,
                           FiveAxisVec3 &tip) {
        carriage = { pose.x, pose.y, pose.z };
        joint1 = carriage + kinematics.rotary_1_pivot;
        joint2 = carriage + kinematics.rotary_1_pivot +
            five_axis_rotate(kinematics.rotary_2_pivot - kinematics.rotary_1_pivot,
                             axis1, pose.rotary_1_deg);
        tip = carriage + kinematics.rotary_1_pivot +
            five_axis_rotate(
                kinematics.rotary_2_pivot +
                    five_axis_rotate(kinematics.tool_tip_offset - kinematics.rotary_2_pivot,
                                     axis2, pose.rotary_2_deg) -
                    kinematics.rotary_1_pivot,
                axis1, pose.rotary_1_deg);
    };
    auto check_segment = [&](const FiveAxisVec3 &a, const FiveAxisVec3 &b) -> bool {
        const double distance = five_axis_norm(b - a);
        const double requested = std::ceil(distance / config.max_sample_spacing_mm);
        if (!std::isfinite(requested) ||
            requested > static_cast<double>(config.max_samples_per_segment))
            return false;
        const std::size_t steps = std::max<std::size_t>(1, static_cast<std::size_t>(requested));
        if (sample_budget > config.max_samples_per_segment - std::min(
                config.max_samples_per_segment, steps))
            return false;
        sample_budget += steps;
        for (std::size_t i = 0; i <= steps; ++i) {
            const FiveAxisVec3 p = a + (b - a) * (static_cast<double>(i) / steps);
            if (!check_point(p))
                return false;
        }
        return true;
    };

    FiveAxisCommand previous = path.front();
    if (!five_axis_finite_command(previous))
        return { false, "Swept-head collision check received an invalid motion command." };
    FiveAxisVec3 carriage, joint1, joint2, tip;
    head_points(previous, carriage, joint1, joint2, tip);
    if (!check_segment(carriage, joint1) || !check_segment(joint1, joint2) ||
        !check_segment(joint2, tip))
        return { false, "Rotary-head assembly intersects an obstacle or exceeds the sampling budget." };

    for (std::size_t i = 1; i < path.size(); ++i) {
        const FiveAxisCommand &next = path[i];
        if (!five_axis_finite_command(next))
            return { false, "Swept-head collision check received an invalid motion command." };
        const double linear_distance = five_axis_norm(
            FiveAxisVec3{next.x - previous.x, next.y - previous.y, next.z - previous.z});
        const double angular_delta = std::max(
            std::abs(next.rotary_1_deg - previous.rotary_1_deg),
            std::abs(next.rotary_2_deg - previous.rotary_2_deg));
        const double lever = five_axis_norm(kinematics.rotary_1_pivot) +
            five_axis_norm(kinematics.rotary_2_pivot) +
            five_axis_norm(kinematics.tool_tip_offset);
        const double swept_distance = linear_distance +
            lever * angular_delta * 3.14159265358979323846 / 180.0;
        const double requested = std::ceil(swept_distance / config.max_sample_spacing_mm);
        if (!std::isfinite(requested) ||
            requested > static_cast<double>(config.max_samples_per_segment))
            return { false, "Rotary-head swept segment exceeds the configured sampling budget." };
        const std::size_t steps = std::max<std::size_t>(1, static_cast<std::size_t>(requested));
        if (sample_budget > config.max_samples_per_segment - std::min(
                config.max_samples_per_segment, steps))
            return { false, "Rotary-head path exceeds the configured total sampling budget." };
        sample_budget += steps;
        for (std::size_t sample = 1; sample <= steps; ++sample) {
            const double t = static_cast<double>(sample) / steps;
            FiveAxisCommand pose = previous;
            pose.x = previous.x + (next.x - previous.x) * t;
            pose.y = previous.y + (next.y - previous.y) * t;
            pose.z = previous.z + (next.z - previous.z) * t;
            pose.rotary_1_deg = previous.rotary_1_deg +
                (next.rotary_1_deg - previous.rotary_1_deg) * t;
            pose.rotary_2_deg = previous.rotary_2_deg +
                (next.rotary_2_deg - previous.rotary_2_deg) * t;
            head_points(pose, carriage, joint1, joint2, tip);
            if (!check_segment(carriage, joint1) || !check_segment(joint1, joint2) ||
                !check_segment(joint2, tip))
                return { false, "Interpolated rotary-head sweep intersects an obstacle or exceeds the sampling budget." };
        }
        previous = next;
    }
    return { true, {} };
}

inline FiveAxisValidationResult validate_five_axis_machine_swept_clearance(
    const FiveAxisKinematicsConfig &kinematics,
    const FiveAxisCollisionConfig &config,
    const std::vector<FiveAxisCommand> &path)
{
    if (kinematics.mount == FiveAxisRotaryMount::RotaryHead)
        return validate_five_axis_head_swept_clearance(kinematics, config, path);
    return validate_five_axis_path_clearance(config, path);
}

// Converts a tool-pose polyline into machine coordinates. Rotary solutions are
// seeded from the preceding point to reduce discontinuous branch jumps. This
// planner deliberately fails closed if any pose is unreachable, the complete
// path violates output limits, or the configured clearance check fails.
inline FiveAxisValidationResult plan_five_axis_path(
    const FiveAxisKinematicsConfig &kinematics,
    const FiveAxisOutputConfig &output_config,
    const FiveAxisCollisionConfig &collision_config,
    const std::vector<FiveAxisToolPose> &poses,
    std::vector<FiveAxisCommand> &commands,
    double initial_rotary_1_deg = 0.0,
    double initial_rotary_2_deg = 0.0)
{
    commands.clear();
    if (poses.size() < 2)
        return { false, "A planned five-axis toolpath requires at least two poses." };
    std::string kinematics_error;
    if (!validate_five_axis_kinematics_config(kinematics, &kinematics_error))
        return { false, kinematics_error };
    const auto output_validation = validate_five_axis_output_config(output_config);
    if (!output_validation.valid)
        return output_validation;
    const auto collision_validation = validate_five_axis_collision_config(collision_config);
    if (!collision_validation.valid)
        return collision_validation;

    std::vector<FiveAxisCommand> candidate;
    candidate.reserve(poses.size());
    double previous_a = initial_rotary_1_deg;
    double previous_b = initial_rotary_2_deg;
    for (const FiveAxisToolPose &pose : poses) {
        if (!five_axis_finite(pose.point) || !five_axis_finite(pose.normal) ||
            !std::isfinite(pose.extrusion) || !std::isfinite(pose.feed_mm_min) ||
            pose.feed_mm_min <= 0.0)
            return { false, "Toolpath contains an invalid point, normal, extrusion, or feedrate." };

        const FiveAxisJointSolution joints = solve_five_axis_orientation(
            kinematics, pose.normal, previous_a, previous_b);
        if (!joints.valid)
            return { false, "Toolpath pose has no valid rotary solution: " + joints.error };

        FiveAxisVec3 machine_point;
        if (kinematics.mount == FiveAxisRotaryMount::RotaryBed)
            machine_point = five_axis_transform_bed_point(
                kinematics, pose.point, joints.rotary_1_deg, joints.rotary_2_deg);
        else
            machine_point = five_axis_compensate_head_position(
                kinematics, pose.point, joints.rotary_1_deg, joints.rotary_2_deg);

        // Apply the active SuperSlicer tool's carriage-frame offset after
        // rotary transforms; this offset must not rotate with the workpiece.
        machine_point = machine_point + kinematics.machine_frame_tool_offset;
        if (!five_axis_finite(machine_point))
            return { false, "Kinematic point transform produced non-finite coordinates." };

        candidate.push_back({ machine_point.x, machine_point.y, machine_point.z,
                              pose.extrusion, joints.rotary_1_deg,
                              joints.rotary_2_deg, pose.feed_mm_min });
        previous_a = joints.rotary_1_deg;
        previous_b = joints.rotary_2_deg;
    }

    const auto feed_limit = limit_five_axis_rotary_feedrates(output_config, candidate);
    if (!feed_limit.valid)
        return feed_limit;
    const auto path_validation = validate_five_axis_path(output_config, candidate);
    if (!path_validation.valid)
        return path_validation;
    const auto clearance = validate_five_axis_machine_swept_clearance(
        kinematics, collision_config, candidate);
    if (!clearance.valid)
        return clearance;
    commands.swap(candidate);
    return { true, {} };
}

} // namespace Slic3r

#endif // slic3r_FiveAxisMotionPlanner_hpp_
