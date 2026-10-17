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

    const auto path_validation = validate_five_axis_path(output_config, candidate);
    if (!path_validation.valid)
        return path_validation;
    const auto clearance = validate_five_axis_path_clearance(collision_config, candidate);
    if (!clearance.valid)
        return clearance;
    commands.swap(candidate);
    return { true, {} };
}

} // namespace Slic3r

#endif // slic3r_FiveAxisMotionPlanner_hpp_
