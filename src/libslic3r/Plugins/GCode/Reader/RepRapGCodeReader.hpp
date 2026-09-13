///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Plugins_GCode_Reader_RepRapGCodeReader_hpp_
#define slic3r_Plugins_GCode_Reader_RepRapGCodeReader_hpp_

#include "BuiltinGCodeReaderBase.hpp"

/*
RepRapFirmware G-code reader
============================

RepRapFirmware reuses several standard command numbers with distinct parameter
semantics. This provider owns those interpretations while inheriting movement,
modal coordinates and the normalized event protocol from the shared parser.

Command coverage
----------------

Fully parsed into portable and/or private state:
  G10/G11, M106/M107, M116, M201, M203, M204, M205, M563, M566,
  M568, M572 and T-1, plus the common movement and modal commands.

Recognized without a represented state change:
  G4, M900 and M906.

Not parsed:
  RepRapFirmware expressions, macros, text-valued peripheral configuration,
  remote-board dotted device identifiers and commands not listed above. Such
  commands invalidate known state because their effects cannot be bounded.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Reader {

class RepRapGCodeReader final : public BuiltinGCodeReaderBase
{
public:
    explicit RepRapGCodeReader(orchestrator_handle *orchestrator);

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
    void validate_private_state(const GCodeStateTracker &state) const override;
};

}}} // namespace slic3r_api::GCodeGeneration::Reader

#endif // slic3r_Plugins_GCode_Reader_RepRapGCodeReader_hpp_
