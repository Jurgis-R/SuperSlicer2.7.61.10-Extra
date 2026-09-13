///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Plugins_GCode_Reader_Marlin1GCodeReader_hpp_
#define slic3r_Plugins_GCode_Reader_Marlin1GCodeReader_hpp_

#include "MarlinGCodeReaderBase.hpp"

/*
Marlin 1 G-code reader
======================

Marlin 1 uses M204 S as the acceleration register shared by printing and
travel. The Reader publishes that physical value through the portable current
acceleration item. More detailed logical acceleration histories are outside
this portable-state pass.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Reader {

class Marlin1GCodeReader final : public MarlinGCodeReaderBase
{
public:
    explicit Marlin1GCodeReader(orchestrator_handle *orchestrator);

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

#endif // slic3r_Plugins_GCode_Reader_Marlin1GCodeReader_hpp_
