///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Plugins_GCode_Reader_KlipperGCodeReader_hpp_
#define slic3r_Plugins_GCode_Reader_KlipperGCodeReader_hpp_

#include "BuiltinGCodeReaderBase.hpp"

/*
Klipper G-code reader
=====================

Klipper combines standard G/M commands with named textual commands. This
provider resolves those names, including configured extruder sections when a
Print is available, before delegating ordinary modal G-code to the base class.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Reader {

class KlipperGCodeReader final : public BuiltinGCodeReaderBase
{
public:
    explicit KlipperGCodeReader(orchestrator_handle *orchestrator);

private:
    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    uint32_t probe_impl(std::string_view prefix) const override;
    bool consume_text_command(const plugin_run_context &run_context,
                              const run_ctx_gcode_reader &context,
                              GCodeStateTracker &state,
                              const GCodeLineParser::GCodeLine &line,
                              raw_gcode_reader_source source) const override;
    bool consume_numeric_command(const plugin_run_context &run_context,
                                 const run_ctx_gcode_reader &context,
                                 GCodeStateTracker &state,
                                 const GCodeLineParser::GCodeLine &line,
                                 char letter,
                                 uint32_t number,
                                 raw_gcode_reader_source source) const override;
    bool is_neutral_command(char letter, uint32_t number) const override;
    bool is_marker_command(char letter, uint32_t number) const override;
};

}}} // namespace slic3r_api::GCodeGeneration::Reader

#endif // slic3r_Plugins_GCode_Reader_KlipperGCodeReader_hpp_
