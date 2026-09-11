///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_plugin_cpp_gcode_SingleAccelerationRegisterGCodeFirmwareSession_hpp_
#define slic3r_Api_plugin_cpp_gcode_SingleAccelerationRegisterGCodeFirmwareSession_hpp_

#include <optional>

#include "DefaultGCodeFirmwareSession.hpp"

/*
Single acceleration-register firmware session
==============================================

Some firmware dialects expose one physical acceleration command for both
print and travel moves. This specialization keeps Gantry's two logical
requests, but acknowledges only the category represented by the latest
command. Switching movement category therefore restores that category's
requested acceleration even when both requested values happen to be equal.
*/

namespace slic3r_api { namespace GCodeGeneration {

// Reuses the standard PrintingPlan traversal while adapting only the relation
// between the independent print and travel acceleration histories.
class SingleAccelerationRegisterGCodeFirmwareSession :
    public DefaultGCodeFirmwareSession
{
public:
    using DefaultGCodeFirmwareSession::DefaultGCodeFirmwareSession;

protected:
    void setup_firmware(const Config &config) override;
    std::string write_acceleration(PreparedMove::Kind kind) override;

private:
    // A shared register may use travel acceleration only when the following
    // print move has a known value to restore. This fallback is available only
    // when machine limits participate in host-side policy.
    std::optional<uint32_t> m_print_acceleration_fallback;
};

}} // namespace slic3r_api::GCodeGeneration

#endif // slic3r_Api_plugin_cpp_gcode_SingleAccelerationRegisterGCodeFirmwareSession_hpp_
