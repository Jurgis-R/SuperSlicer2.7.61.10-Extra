///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Plugins_GCode_Firmware_RepRapGCodeFirmware_hpp_
#define slic3r_Plugins_GCode_Firmware_RepRapGCodeFirmware_hpp_

#include "libslic3r/Api/plugin/cpp/gcode/DefaultGCodeFirmwareSession.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/MachineEnvelope.hpp"
#include "libslic3r/Plugins/GCode/Reader/RepRapGCodeReaderState.hpp"

/*
RepRapFirmware session
======================

RepRapFirmware reuses the generic independent print/travel motion states but
encodes tool heaters, fans and pressure advance with Duet-compatible commands.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Firmware {

class RepRapGCodeFirmwareSession : public DefaultGCodeFirmwareSession
{
public:
    using DefaultGCodeFirmwareSession::DefaultGCodeFirmwareSession;
    std::string reader_plugin_id() const override { return "gcode.reader.reprap"; }

protected:
    void setup_firmware(const Config &config) override;
    void append_reader_private_state(
        std::vector<raw_gcode_reader_event_item> &items) const override;
    void synchronize_reader_private_state(const GCodeStateTracker &state) override;
    std::string resolve_empty_script(gcode_script_type script_type,
                                     const Config *producer_config) const override;
    std::string encode_machine_envelope(const MachineEnvelope &envelope) const override;
    std::string encode_tool_temperature(uint16_t tool_id,
                                        int16_t temperature,
                                        bool wait) const override;
    std::string encode_fan(uint16_t tool_id, double speed_percent) const override;
    std::string encode_pressure_advance(uint16_t tool_id,
                                        double pressure_advance) const override;
    std::string encode_acceleration(uint32_t acceleration, bool travel) const override;
    std::string encode_firmware_retraction(bool retract) const override;
    std::string encode_object_definitions(const std::vector<ObjectLabel> &objects) const override;
    std::string encode_object_start(const ObjectLabel &object) const override;
    std::string encode_object_end(const ObjectLabel &object) const override;

private:
    // Encoder methods are logically const because they do not alter the
    // PrintingPlan request. They do record the exact RRF registers represented
    // by their output so a following custom script starts from a complete state.
    mutable Reader::RepRapGCodeReaderState m_reader_state;
};

}}} // namespace slic3r_api::GCodeGeneration::Firmware

#endif // slic3r_Plugins_GCode_Firmware_RepRapGCodeFirmware_hpp_
