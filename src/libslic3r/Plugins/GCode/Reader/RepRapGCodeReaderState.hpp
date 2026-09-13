///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Plugins_GCode_Reader_RepRapGCodeReaderState_hpp_
#define slic3r_Plugins_GCode_Reader_RepRapGCodeReaderState_hpp_

#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <vector>

#include "libslic3r/Api/plugin/cpp/gcode/GCodeStateTracker.hpp"

/*
RepRapFirmware private Reader state
===================================

The generic Reader state describes portable, currently effective values. A
RepRapFirmware writer additionally owns persistent registers and relationships
which have no portable meaning, such as separate print/travel accelerations
and one-to-many tool mappings. This file is the private C++ contract shared by
the built-in RepRap Reader and firmware; none of its keys belong to the public
plugin API.

    RepRap firmware state -> private scalar keys -> generic opaque tracker
                                                   |
                                                   v
    RepRap firmware state <- private scalar keys <- RepRap Reader model

All values still cross the Reader ABI as finite doubles. Scalar keys use target
zero unless documented otherwise. Tool values use the logical tool id as
target, and pressure advance uses the physical drive id. Mapping lists use one
count key plus indexed slot types, allowing the generic (type,target) map to
represent several devices for one tool without duplicate keys.

Private key reference
---------------------

  Key family                         target             value / unit
  PRINT/TRAVEL_ACCELERATION          0                  mm/s^2, >= 0
  MAX_ACCELERATION_{X,Y,Z,E}         0                  mm/s^2, >= 0
  MAX_FEEDRATE_{X,Y,Z,E}             0                  mm/s, >= 0
  MIN_EXTRUDING_FEEDRATE             0                  mm/s, >= 0
  MACHINE/JOB_JERK_{X,Y,Z,E}         0                  mm/s, >= 0
  PRESSURE_ADVANCE                   physical drive     seconds, >= 0
  TOOL_STANDBY_TEMPERATURE           logical tool       degrees C, finite
  TOOL_OFFSET_{X,Y,Z}                logical tool       millimetres, finite
  TOOL_RETRACTED                     logical tool       exact boolean 0/1
  TOOL_{DRIVE,HEATER,FAN}_COUNT      logical tool       exact uint32 count
  TOOL_*_SLOT_BEGIN + n              logical tool       exact uint32 device id

An absent scalar means unknown, not zero. An absent mapping count means the
same empty mapping as count zero; encoding always writes the count explicitly.
The tracker owns its copied `(type,target,value)` entries. RepRapReaderState is
an independent C++ value and retains no pointer into the tracker.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Reader {

constexpr uint32_t REPRAP_READER_ID = 0u;
constexpr uint32_t REPRAP_MAPPING_SLOT_COUNT = 65536u;

enum RepRapGCodeReaderItem : uint32_t {
    RRF_ITEM_PRINT_ACCELERATION = RAW_GCODE_READER_ITEM_CUSTOM_BEGIN,
    RRF_ITEM_TRAVEL_ACCELERATION,
    RRF_ITEM_MAX_ACCELERATION_X,
    RRF_ITEM_MAX_ACCELERATION_Y,
    RRF_ITEM_MAX_ACCELERATION_Z,
    RRF_ITEM_MAX_ACCELERATION_E,
    RRF_ITEM_MAX_FEEDRATE_X,
    RRF_ITEM_MAX_FEEDRATE_Y,
    RRF_ITEM_MAX_FEEDRATE_Z,
    RRF_ITEM_MAX_FEEDRATE_E,
    RRF_ITEM_MIN_EXTRUDING_FEEDRATE,
    RRF_ITEM_MACHINE_JERK_X,
    RRF_ITEM_MACHINE_JERK_Y,
    RRF_ITEM_MACHINE_JERK_Z,
    RRF_ITEM_MACHINE_JERK_E,
    RRF_ITEM_JOB_JERK_X,
    RRF_ITEM_JOB_JERK_Y,
    RRF_ITEM_JOB_JERK_Z,
    RRF_ITEM_JOB_JERK_E,
    RRF_ITEM_PRESSURE_ADVANCE,
    RRF_ITEM_TOOL_STANDBY_TEMPERATURE,
    RRF_ITEM_TOOL_OFFSET_X,
    RRF_ITEM_TOOL_OFFSET_Y,
    RRF_ITEM_TOOL_OFFSET_Z,
    RRF_ITEM_TOOL_RETRACTED,
    RRF_ITEM_TOOL_DRIVE_COUNT,
    RRF_ITEM_TOOL_HEATER_COUNT,
    RRF_ITEM_TOOL_FAN_COUNT,

    RRF_ITEM_TOOL_DRIVE_SLOT_BEGIN = RAW_GCODE_READER_ITEM_CUSTOM_BEGIN + 0x00010000u,
    RRF_ITEM_TOOL_HEATER_SLOT_BEGIN = RAW_GCODE_READER_ITEM_CUSTOM_BEGIN + 0x00020000u,
    RRF_ITEM_TOOL_FAN_SLOT_BEGIN = RAW_GCODE_READER_ITEM_CUSTOM_BEGIN + 0x00030000u
};

struct RepRapToolState {
    std::vector<uint32_t> drives;
    std::vector<uint32_t> heaters;
    std::vector<uint32_t> fans;
    std::optional<double> standby_temperature;
    std::array<std::optional<double>, 3> offset;
    std::optional<bool> retracted;
};

class RepRapGCodeReaderState
{
public:
    // Decode and validate every known RRF key without modifying the tracker.
    // Unknown custom keys are deliberately ignored and remain tracker-owned.
    static RepRapGCodeReaderState from_tracker(const GCodeStateTracker &tracker);

    // Validate the semantic domains which the generic tracker intentionally
    // cannot know. Limits, accelerations, jerk and pressure advance are
    // non-negative; temperatures and offsets may be signed but remain finite.
    void validate() const;

    // Replace only the known RRF key ranges. Other providers' or future keys
    // stay untouched so a newer state can pass through an older implementation.
    void write_to(GCodeStateTracker &tracker) const;

    std::optional<double> print_acceleration;
    std::optional<double> travel_acceleration;
    std::array<std::optional<double>, 4> max_acceleration;
    std::array<std::optional<double>, 4> max_feedrate;
    std::optional<double> min_extruding_feedrate;
    std::array<std::optional<double>, 4> machine_jerk;
    std::array<std::optional<double>, 4> job_jerk;
    std::map<uint32_t, double> pressure_advance;
    std::map<uint32_t, RepRapToolState> tools;
};

}}} // namespace slic3r_api::GCodeGeneration::Reader

#endif // slic3r_Plugins_GCode_Reader_RepRapGCodeReaderState_hpp_
