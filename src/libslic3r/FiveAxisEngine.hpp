/// Copyright (c) 2026 Jurgis-R
/// SuperSlicer is released under the terms of the AGPLv3 or higher.
#ifndef slic3r_FiveAxisEngine_hpp_
#define slic3r_FiveAxisEngine_hpp_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace Slic3r {

// Output-stage representation. XYZ and E are already in the machine's active
// coordinate/extrusion modes; rotary angles must be produced by a kinematics
// backend. Keeping this boundary explicit prevents the output formatter from
// pretending that a surface normal is itself a machine-axis solution.
struct FiveAxisCommand {
    double x { 0.0 };
    double y { 0.0 };
    double z { 0.0 };
    double e { 0.0 };
    double rotary_1_deg { 0.0 };
    double rotary_2_deg { 0.0 };
    double feed_mm_min { 0.0 };
};

struct FiveAxisOutputConfig {
    char rotary_1_word { 'A' };
    char rotary_2_word { 'B' };
    double rotary_1_min_deg { -180.0 };
    double rotary_1_max_deg { 180.0 };
    double rotary_2_min_deg { -90.0 };
    double rotary_2_max_deg { 90.0 };
    double max_rotary_rate_deg_s { 30.0 };
    double max_rotary_delta_deg_per_mm { 20.0 };
    int precision { 3 };
    bool supports_simultaneous_rotary { true };
};

struct FiveAxisValidationResult {
    bool valid { false };
    std::string error;
};

inline bool five_axis_finite_command(const FiveAxisCommand &c)
{
    return std::isfinite(c.x) && std::isfinite(c.y) && std::isfinite(c.z) &&
           std::isfinite(c.e) && std::isfinite(c.rotary_1_deg) &&
           std::isfinite(c.rotary_2_deg) && std::isfinite(c.feed_mm_min) &&
           c.feed_mm_min > 0.0;
}

inline FiveAxisValidationResult validate_five_axis_output_config(
    const FiveAxisOutputConfig &config)
{
    const auto valid_word = [](char c) {
        return c >= 'A' && c <= 'Z' && c != 'X' && c != 'Y' && c != 'Z' &&
               c != 'E' && c != 'F' && c != 'G';
    };
    if (!valid_word(config.rotary_1_word) || !valid_word(config.rotary_2_word) ||
        config.rotary_1_word == config.rotary_2_word)
        return { false, "Rotary axis words must be distinct, valid G-code letters." };
    if (!std::isfinite(config.rotary_1_min_deg) ||
        !std::isfinite(config.rotary_1_max_deg) ||
        !std::isfinite(config.rotary_2_min_deg) ||
        !std::isfinite(config.rotary_2_max_deg) ||
        config.rotary_1_min_deg >= config.rotary_1_max_deg ||
        config.rotary_2_min_deg >= config.rotary_2_max_deg)
        return { false, "Rotary axis limits must be finite increasing ranges." };
    if (!std::isfinite(config.max_rotary_rate_deg_s) ||
        config.max_rotary_rate_deg_s <= 0.0 ||
        !std::isfinite(config.max_rotary_delta_deg_per_mm) ||
        config.max_rotary_delta_deg_per_mm <= 0.0)
        return { false, "Rotary rate and angular change limits must be positive and finite." };
    if (config.precision < 0 || config.precision > 8)
        return { false, "G-code precision must be between 0 and 8 decimal places." };
    return { true, {} };
}

// Validates an entire candidate before the caller emits any part of it.
// This is a motion-limit guard, not a geometric collision test.
inline FiveAxisValidationResult validate_five_axis_path(
    const FiveAxisOutputConfig &config,
    const std::vector<FiveAxisCommand> &path)
{
    const auto config_result = validate_five_axis_output_config(config);
    if (!config_result.valid)
        return config_result;
    if (path.size() < 2)
        return { false, "A five-axis path needs at least two commands." };

    for (std::size_t i = 0; i < path.size(); ++i) {
        const FiveAxisCommand &p = path[i];
        if (!five_axis_finite_command(p))
            return { false, "Path contains non-finite coordinates or a non-positive feedrate." };
        if (p.rotary_1_deg < config.rotary_1_min_deg ||
            p.rotary_1_deg > config.rotary_1_max_deg ||
            p.rotary_2_deg < config.rotary_2_min_deg ||
            p.rotary_2_deg > config.rotary_2_max_deg)
            return { false, "Path exceeds a configured rotary-axis limit." };
        if (i == 0)
            continue;

        const FiveAxisCommand &prev = path[i - 1];
        const double dx = p.x - prev.x;
        const double dy = p.y - prev.y;
        const double dz = p.z - prev.z;
        const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        const double da = std::abs(p.rotary_1_deg - prev.rotary_1_deg);
        const double db = std::abs(p.rotary_2_deg - prev.rotary_2_deg);
        if (!config.supports_simultaneous_rotary && da > 1e-9 && db > 1e-9)
            return { false, "Machine configuration forbids simultaneous motion of both rotary axes." };

        if (distance <= 1e-9) {
            if (da > 1e-9 || db > 1e-9)
                return { false, "Rotary-only moves are not accepted by this output stage." };
            continue;
        }

        const double max_delta = std::max(da, db);
        if (max_delta / distance > config.max_rotary_delta_deg_per_mm + 1e-9)
            return { false, "Path exceeds the configured rotary-angle change per millimetre." };

        // Use the slower endpoint feed as a conservative estimate for the
        // segment duration; the final motion planner may apply stricter limits.
        const double feed_mm_min = std::min(prev.feed_mm_min, p.feed_mm_min);
        const double duration_s = distance / (feed_mm_min / 60.0);
        if (duration_s <= 0.0 ||
            max_delta / duration_s > config.max_rotary_rate_deg_s + 1e-9)
            return { false, "Path exceeds the configured rotary-axis speed limit." };
    }
    return { true, {} };
}

// Formats only after whole-path validation succeeds. This function does not
// emit setup commands, change XYZ/E coordinate modes, solve inverse kinematics,
// or prove nozzle/tool clearance. The caller owns those responsibilities.
inline bool emit_five_axis_linear_gcode(
    const FiveAxisOutputConfig &config,
    const std::vector<FiveAxisCommand> &path,
    std::string &output,
    std::string *error = nullptr)
{
    output.clear();
    const FiveAxisValidationResult validation = validate_five_axis_path(config, path);
    if (!validation.valid) {
        if (error != nullptr)
            *error = validation.error;
        return false;
    }

    std::ostringstream stream;
    stream << std::fixed << std::setprecision(config.precision);
    for (const FiveAxisCommand &p : path) {
        stream << "G1 X" << p.x << " Y" << p.y << " Z" << p.z
               << ' ' << config.rotary_1_word << p.rotary_1_deg
               << ' ' << config.rotary_2_word << p.rotary_2_deg
               << " E" << p.e << " F" << p.feed_mm_min << '\n';
    }
    output = stream.str();
    if (error != nullptr)
        error->clear();
    return true;
}

} // namespace Slic3r

#endif // slic3r_FiveAxisEngine_hpp_
