/// Copyright (c) 2026 Jurgis-R
/// SuperSlicer is released under the terms of the AGPLv3 or higher.
#ifndef slic3r_FiveAxisConfig_hpp_
#define slic3r_FiveAxisConfig_hpp_

#include "FiveAxisMachineProfile.hpp"
#include "PrintConfig.hpp"

#include <sstream>
#include <string>
#include <vector>

namespace Slic3r {

// Convert persisted printer-profile options into the validated engine schema.
// Output is assigned only when every setting parses and validates.
inline FiveAxisValidationResult load_five_axis_machine_profile(
    const DynamicPrintConfig &config, FiveAxisMachineProfile &profile)
{
    FiveAxisMachineProfile candidate;
    candidate.enabled = config.opt_bool("five_axis_enabled");
    candidate.name = config.opt_string("five_axis_machine_name");
    candidate.postprocessor_id = config.opt_string("five_axis_postprocessor_id");

    const std::string mount = config.opt_string("five_axis_mount");
    if (mount == "rotary_bed")
        candidate.kinematics.mount = FiveAxisRotaryMount::RotaryBed;
    else if (mount == "rotary_head")
        candidate.kinematics.mount = FiveAxisRotaryMount::RotaryHead;
    else
        return { false, "Five-axis mount must be rotary_bed or rotary_head." };

    const auto f = [&config](const char *key) { return config.opt_float(key); };
    const auto s = [&config](const char *key) -> const std::string& { return config.opt_string(key); };
    auto &k = candidate.kinematics;
    k.rotary_1_axis = { f("five_axis_rotary_1_axis_x"), f("five_axis_rotary_1_axis_y"), f("five_axis_rotary_1_axis_z") };
    k.rotary_2_axis = { f("five_axis_rotary_2_axis_x"), f("five_axis_rotary_2_axis_y"), f("five_axis_rotary_2_axis_z") };
    k.rotary_1_pivot = { f("five_axis_rotary_1_pivot_x"), f("five_axis_rotary_1_pivot_y"), f("five_axis_rotary_1_pivot_z") };
    k.rotary_2_pivot = { f("five_axis_rotary_2_pivot_x"), f("five_axis_rotary_2_pivot_y"), f("five_axis_rotary_2_pivot_z") };
    k.tool_tip_offset = { f("five_axis_tool_tip_offset_x"), f("five_axis_tool_tip_offset_y"), f("five_axis_tool_tip_offset_z") };
    k.rotary_1_min_deg = f("five_axis_rotary_1_min");
    k.rotary_1_max_deg = f("five_axis_rotary_1_max");
    k.rotary_2_min_deg = f("five_axis_rotary_2_min");
    k.rotary_2_max_deg = f("five_axis_rotary_2_max");

    auto &o = candidate.output;
    const std::string word1 = s("five_axis_rotary_1_word");
    const std::string word2 = s("five_axis_rotary_2_word");
    if (word1.size() != 1 || word2.size() != 1)
        return { false, "Each five-axis rotary output word must be one character." };
    o.rotary_1_word = word1[0];
    o.rotary_2_word = word2[0];
    o.rotary_1_min_deg = k.rotary_1_min_deg;
    o.rotary_1_max_deg = k.rotary_1_max_deg;
    o.rotary_2_min_deg = k.rotary_2_min_deg;
    o.rotary_2_max_deg = k.rotary_2_max_deg;
    o.max_rotary_rate_deg_s = f("five_axis_max_rotary_rate");
    o.max_rotary_delta_deg_per_mm = f("five_axis_max_rotary_delta_per_mm");
    o.supports_simultaneous_rotary = config.opt_bool("five_axis_simultaneous_rotary");
    o.precision = config.opt_int("five_axis_output_precision");

    candidate.machine_min = { f("five_axis_machine_min_x"), f("five_axis_machine_min_y"), f("five_axis_machine_min_z") };
    candidate.machine_max = { f("five_axis_machine_max_x"), f("five_axis_machine_max_y"), f("five_axis_machine_max_z") };
    candidate.collision.tool_clearance_radius_mm = f("five_axis_tool_clearance_radius");
    candidate.collision.max_sample_spacing_mm = f("five_axis_collision_sample_spacing");

    // Obstacle boxes are encoded as semicolon-separated minX,minY,minZ,maxX,maxY,maxZ groups.
    // Empty means no configured static obstacle boxes; this is not a claim that the machine is obstacle-free.
    const std::string boxes = s("five_axis_obstacle_boxes");
    std::stringstream groups(boxes);
    std::string group;
    while (std::getline(groups, group, ';')) {
        if (group.empty())
            continue;
        std::stringstream values(group);
        std::vector<double> v;
        std::string token;
        while (std::getline(values, token, ',')) {
            std::stringstream number(token);
            double value = 0.0;
            number >> value;
            number >> std::ws;
            if (!number || !number.eof() || !std::isfinite(value))
                return { false, "Malformed value in five-axis obstacle box list." };
            v.push_back(value);
        }
        if (v.size() != 6)
            return { false, "Each five-axis obstacle box must contain exactly six comma-separated numbers." };
        candidate.collision.obstacles.push_back({ { v[0], v[1], v[2] }, { v[3], v[4], v[5] } });
    }

    const FiveAxisValidationResult validation = validate_five_axis_machine_profile(candidate);
    if (!validation.valid)
        return validation;
    profile = candidate;
    return { true, {} };
}

} // namespace Slic3r

#endif // slic3r_FiveAxisConfig_hpp_
