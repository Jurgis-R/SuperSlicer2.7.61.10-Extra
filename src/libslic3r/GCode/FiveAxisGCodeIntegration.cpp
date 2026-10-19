/// Five-axis G-code travel integration for SuperSlicer.
#include "../GCode.hpp"

#include "../Exception.hpp"
#include "../FiveAxisConfig.hpp"
#include "../FiveAxisKinematics.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace Slic3r {

namespace {

FiveAxisVec3 transform_work_point(
    const FiveAxisMachineProfile &profile,
    const Vec3d &work_point,
    double rotary_1_deg,
    double rotary_2_deg)
{
    const FiveAxisVec3 point{work_point.x(), work_point.y(), work_point.z()};
    FiveAxisVec3 machine;
    if (profile.kinematics.mount == FiveAxisRotaryMount::RotaryBed)
        machine = five_axis_transform_bed_point(
            profile.kinematics, point, rotary_1_deg, rotary_2_deg);
    else
        machine = five_axis_compensate_head_position(
            profile.kinematics, point, rotary_1_deg, rotary_2_deg);
    return machine + profile.kinematics.machine_frame_tool_offset;
}

FiveAxisCommand command_from_point(
    const FiveAxisVec3 &point, double rotary_1_deg,
    double rotary_2_deg, double feed_mm_min)
{
    FiveAxisCommand command;
    command.x = point.x;
    command.y = point.y;
    command.z = point.z;
    command.e = 0.0;
    command.rotary_1_deg = rotary_1_deg;
    command.rotary_2_deg = rotary_2_deg;
    command.feed_mm_min = feed_mm_min;
    return command;
}

FiveAxisMachineProfile load_profile_or_throw(
    const FullPrintConfig &config, const GCodeWriter &writer)
{
    FiveAxisMachineProfile profile;
    const FiveAxisValidationResult loaded = load_five_axis_machine_profile(config, profile);
    if (!loaded.valid)
        throw Slic3r::RuntimeError(std::string("Invalid five-axis machine profile: ") + loaded.error);
    if (!profile.enabled)
        throw Slic3r::RuntimeError("Five-axis travel was requested while the profile is disabled.");
    if (!is_five_axis_postprocessor_registered(profile.postprocessor_id))
        throw Slic3r::RuntimeError("The configured five-axis controller has no registered writer.");
    const Vec2d offset = writer.current_tool_offset();
    profile.kinematics.machine_frame_tool_offset = {-offset.x(), -offset.y(), 0.0};
    std::string kinematics_error;
    if (!validate_five_axis_kinematics_config(profile.kinematics, &kinematics_error))
        throw Slic3r::RuntimeError(
            std::string("Five-axis kinematics with the active tool offset is invalid: ") + kinematics_error);
    return profile;
}

double five_axis_travel_speed(const GCodeWriter &writer, const FullPrintConfig &config)
{
    double speed = config.get_computed_value("travel_speed");
    if (!std::isfinite(speed) || speed <= 0.0)
        speed = writer.gcode_config().travel_speed.value;
    if (!std::isfinite(speed) || speed <= 0.0)
        throw Slic3r::RuntimeError("Five-axis travel has an invalid feedrate.");
    return speed;
}

void validate_five_axis_travel(
    const FiveAxisMachineProfile &profile,
    const std::vector<FiveAxisCommand> &route,
    const char *kind)
{
    const FiveAxisValidationResult motion = validate_five_axis_path(profile.output, route);
    if (!motion.valid)
        throw Slic3r::RuntimeError(std::string("Five-axis ") + kind +
            " violates motion limits: " + motion.error);
    const FiveAxisValidationResult envelope = validate_five_axis_machine_envelope(profile, route);
    if (!envelope.valid)
        throw Slic3r::RuntimeError(std::string("Five-axis ") + kind +
            " violates the machine envelope: " + envelope.error);
    const FiveAxisValidationResult collision = validate_five_axis_machine_swept_clearance(
        profile.kinematics, profile.collision, route);
    if (!collision.valid)
        throw Slic3r::RuntimeError(std::string("Five-axis ") + kind +
            " fails configured clearance checks: " + collision.error);
}

} // namespace

std::string GCodeGenerator::_five_axis_write_travel_to(
    Polyline &travel, const std::string &comment)
{
    if (travel.empty())
        return {};

    const FiveAxisMachineProfile profile = load_profile_or_throw(m_config, m_writer);
    const double speed = five_axis_travel_speed(m_writer, m_config);
    const unsigned travel_acceleration =
        (unsigned)(m_config.travel_acceleration.value + 0.5);
    // Klipper does not support a separate travel-acceleration setting in
    // GCodeWriter: set_travel_acceleration() temporarily changes the active
    // acceleration. Restore it after this travel so the next extrusion does
    // not accidentally run at travel acceleration.
    const uint32_t acceleration_before_travel = m_writer.get_acceleration();
    m_writer.set_travel_acceleration(travel_acceleration);
    const double configured_clearance = m_config.opt_float("five_axis_transition_clearance");
    const double pending_lift = m_writer.get_extra_lift();
    const double current_lift = m_writer.get_lift();
    if (!std::isfinite(configured_clearance) || configured_clearance <= 0.0 ||
        !std::isfinite(pending_lift) || pending_lift < 0.0 ||
        !std::isfinite(current_lift) || current_lift < 0.0)
        throw Slic3r::RuntimeError("Five-axis travel has invalid clearance or lift state.");

    const Vec3d current = m_writer.get_position();
    const double rotary_1 = m_five_axis_rotary_state_initialized
        ? m_five_axis_rotary_state_1_deg : 0.0;
    const double rotary_2 = m_five_axis_rotary_state_initialized
        ? m_five_axis_rotary_state_2_deg : 0.0;
    const double base_z = m_layer != nullptr ? m_layer->print_z : current.z();

    // The first point is the already-current point for a well-formed
    // multi-point travel. Transform every subsequent waypoint before emitting.
    const std::size_t begin = travel.size() > 1 ? 1 : 0;
    std::vector<FiveAxisVec3> targets;
    targets.reserve(travel.size() - begin);
    for (std::size_t i = begin; i < travel.size(); ++i) {
        const Point &point = travel.points[i];
        const Vec3d work_point(
            unscaled<double>(point.x()) + m_origin.x(),
            unscaled<double>(point.y()) + m_origin.y(),
            base_z);
        const FiveAxisVec3 machine = transform_work_point(
            profile, work_point, rotary_1, rotary_2);
        if (!five_axis_finite(machine))
            throw Slic3r::RuntimeError("Five-axis travel transform produced a non-finite point.");
        targets.push_back(machine);
    }
    if (targets.empty())
        return {};

    double highest_z = current.z();
    for (const FiveAxisVec3 &target : targets)
        highest_z = std::max(highest_z, target.z);
    const double clearance = std::max({configured_clearance, pending_lift, current_lift});
    if (!std::isfinite(m_next_lift_min))
        throw Slic3r::RuntimeError("Five-axis travel has a non-finite minimum-lift target.");
    const double safe_z = std::max({
        highest_z, m_next_lift_min, current.z() + pending_lift
    }) + clearance;
    if (!std::isfinite(safe_z))
        throw Slic3r::RuntimeError("Five-axis travel clearance produced non-finite Z.");

    FiveAxisCommand current_pose = command_from_point(
        {current.x(), current.y(), current.z()}, rotary_1, rotary_2, speed * 60.0);
    FiveAxisCommand raised = current_pose;
    raised.z = safe_z;

    std::vector<FiveAxisCommand> route;
    route.reserve(targets.size() + 3);
    route.push_back(current_pose);
    route.push_back(raised);
    // Retain slicer-planned detours, but traverse them at a safe fixed height.
    for (const FiveAxisVec3 &target : targets) {
        FiveAxisCommand waypoint = raised;
        waypoint.x = target.x;
        waypoint.y = target.y;
        route.push_back(waypoint);
    }
    route.push_back(command_from_point(targets.back(), rotary_1, rotary_2, speed * 60.0));

    validate_five_axis_travel(profile, route, "travel");
    const auto emit = [&](const FiveAxisCommand &pose) {
        return m_writer.travel_to_xyz_rotary(
            Vec3d(pose.x, pose.y, pose.z), speed, comment,
            profile.output.rotary_1_word, pose.rotary_1_deg,
            profile.output.rotary_2_word, pose.rotary_2_deg,
            profile.output.precision);
    };
    std::string output;
    for (std::size_t i = 1; i < route.size(); ++i) {
        const FiveAxisCommand &previous = route[i - 1];
        const FiveAxisCommand &pose = route[i];
        const double linear_delta = std::hypot(
            std::hypot(pose.x - previous.x, pose.y - previous.y),
            pose.z - previous.z);
        const bool rotary_changed =
            std::abs(pose.rotary_1_deg - previous.rotary_1_deg) > 1e-9 ||
            std::abs(pose.rotary_2_deg - previous.rotary_2_deg) > 1e-9;
        if (linear_delta <= 1e-9 && !rotary_changed)
            continue;
        const std::string move = emit(pose);
        if (move.empty())
            throw Slic3r::RuntimeError("Five-axis writer rejected a preflighted travel waypoint.");
        output += move;
    }

    m_writer.set_extra_lift(0.0);
    m_writer.set_lift(0.0);
    m_next_lift_min = 0.0;
    this->set_last_pos(travel.back());
    // On Klipper, restore the acceleration that was active before travel.
    // Do not call set_travel_acceleration() here: it would overwrite this
    // restored value because Klipper has no independent travel acceleration.
    m_writer.set_acceleration(acceleration_before_travel);
    return output;
}

std::string GCodeGenerator::_five_axis_generate_travel_gcode(
    const Points3 &travel, const std::string &comment)
{
    if (travel.empty())
        return {};

    const FiveAxisMachineProfile profile = load_profile_or_throw(m_config, m_writer);
    const double speed = five_axis_travel_speed(m_writer, m_config);
    const unsigned travel_acceleration =
        (unsigned)(m_config.travel_acceleration.value + 0.5);
    // Klipper does not support a separate travel-acceleration setting in
    // GCodeWriter: set_travel_acceleration() temporarily changes the active
    // acceleration. Restore it after this travel so the next extrusion does
    // not accidentally run at travel acceleration.
    const uint32_t acceleration_before_travel = m_writer.get_acceleration();
    m_writer.set_travel_acceleration(travel_acceleration);
    const double configured_clearance = m_config.opt_float("five_axis_transition_clearance");
    const double pending_lift = m_writer.get_extra_lift();
    const double current_lift = m_writer.get_lift();
    if (!std::isfinite(configured_clearance) || configured_clearance <= 0.0 ||
        !std::isfinite(pending_lift) || pending_lift < 0.0 ||
        !std::isfinite(current_lift) || current_lift < 0.0 ||
        !std::isfinite(m_next_lift_min))
        throw Slic3r::RuntimeError("Five-axis XYZ travel clearance or lift state is invalid.");

    const double rotary_1 = m_five_axis_rotary_state_initialized
        ? m_five_axis_rotary_state_1_deg : 0.0;
    const double rotary_2 = m_five_axis_rotary_state_initialized
        ? m_five_axis_rotary_state_2_deg : 0.0;
    const Vec3d current = m_writer.get_position();
    std::vector<FiveAxisVec3> targets;
    targets.reserve(travel.size());
    double highest_z = current.z();
    for (const Vec3crd &point : travel) {
        const Vec3d work(
            unscaled<double>(point.x()) + m_origin.x(),
            unscaled<double>(point.y()) + m_origin.y(),
            unscaled<double>(point.z()));
        const FiveAxisVec3 machine = transform_work_point(profile, work, rotary_1, rotary_2);
        if (!five_axis_finite(machine))
            throw Slic3r::RuntimeError("Five-axis XYZ travel transform produced a non-finite point.");
        targets.push_back(machine);
        highest_z = std::max(highest_z, machine.z);
    }

    const double clearance = std::max({configured_clearance, pending_lift, current_lift});
    const double safe_z = std::max({
        highest_z, m_next_lift_min, current.z() + pending_lift
    }) + clearance;
    if (!std::isfinite(safe_z))
        throw Slic3r::RuntimeError("Five-axis XYZ travel clearance produced non-finite Z.");

    std::vector<FiveAxisCommand> route;
    route.reserve(targets.size() + 3);
    route.push_back(command_from_point(
        {current.x(), current.y(), current.z()}, rotary_1, rotary_2, speed * 60.0));
    FiveAxisCommand raised = route.front();
    raised.z = safe_z;
    route.push_back(raised);
    for (const FiveAxisVec3 &target : targets) {
        FiveAxisCommand waypoint = raised;
        waypoint.x = target.x;
        waypoint.y = target.y;
        route.push_back(waypoint);
    }
    route.push_back(command_from_point(targets.back(), rotary_1, rotary_2, speed * 60.0));

    validate_five_axis_travel(profile, route, "XYZ waypoint travel");
    m_writer.set_travel_acceleration((unsigned)(m_config.travel_acceleration.value + 0.5));
    const auto emit = [&](const FiveAxisCommand &pose) {
        return m_writer.travel_to_xyz_rotary(
            Vec3d(pose.x, pose.y, pose.z), speed, comment,
            profile.output.rotary_1_word, pose.rotary_1_deg,
            profile.output.rotary_2_word, pose.rotary_2_deg,
            profile.output.precision);
    };
    std::string output;
    for (std::size_t i = 1; i < route.size(); ++i) {
        const FiveAxisCommand &previous = route[i - 1];
        const FiveAxisCommand &pose = route[i];
        const double linear_delta = std::hypot(
            std::hypot(pose.x - previous.x, pose.y - previous.y),
            pose.z - previous.z);
        const bool rotary_changed =
            std::abs(pose.rotary_1_deg - previous.rotary_1_deg) > 1e-9 ||
            std::abs(pose.rotary_2_deg - previous.rotary_2_deg) > 1e-9;
        if (linear_delta <= 1e-9 && !rotary_changed)
            continue;
        const std::string move = emit(pose);
        if (move.empty())
            throw Slic3r::RuntimeError("Five-axis writer rejected a preflighted XYZ waypoint.");
        output += move;
    }
    this->set_last_pos(travel.back().head<2>());
    m_writer.set_extra_lift(0.0);
    m_writer.set_lift(0.0);
    m_next_lift_min = 0.0;
    // On Klipper, restore the acceleration that was active before travel.
    // Do not call set_travel_acceleration() here: it would overwrite this
    // restored value because Klipper has no independent travel acceleration.
    m_writer.set_acceleration(acceleration_before_travel);
    return output;
}

} // namespace Slic3r
