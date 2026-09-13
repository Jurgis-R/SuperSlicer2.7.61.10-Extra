///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Plugins_GCode_Reader_Marlin2GCodeReader_hpp_
#define slic3r_Plugins_GCode_Reader_Marlin2GCodeReader_hpp_

#include "MarlinGCodeReaderBase.hpp"

/*
Marlin 2 G-code reader
======================

Marlin 2 gives M204 separate print, retract and travel parameters. A command
containing only P or only T identifies one portable acceleration value. A
combined command is accepted but does not pretend that either logical register
is the one current acceleration; exact register preservation requires a future
private Reader state.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Reader {

class Marlin2GCodeReader : public MarlinGCodeReaderBase
{
public:
    explicit Marlin2GCodeReader(orchestrator_handle *orchestrator);

protected:
    void process_M204(const plugin_run_context &run_context,
                      const run_ctx_gcode_reader &context,
                      GCodeStateTracker &state,
                      const GCodeLineParser::GCodeLine &line,
                      raw_gcode_reader_source source) const override;

private:
    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    uint32_t probe_impl(std::string_view prefix) const override;
};

}}} // namespace slic3r_api::GCodeGeneration::Reader

#endif // slic3r_Plugins_GCode_Reader_Marlin2GCodeReader_hpp_
