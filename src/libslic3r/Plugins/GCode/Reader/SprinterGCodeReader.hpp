///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Plugins_GCode_Reader_SprinterGCodeReader_hpp_
#define slic3r_Plugins_GCode_Reader_SprinterGCodeReader_hpp_

#include "MarlinGCodeReaderBase.hpp"

/*
Sprinter G-code reader
======================

Sprinter uses M204 P as one physical acceleration register. M572 is a known
pressure-advance command without a portable state slot, so the Marlin-family
base accepts it without discarding unrelated state.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Reader {

class SprinterGCodeReader final : public MarlinGCodeReaderBase
{
public:
    explicit SprinterGCodeReader(orchestrator_handle *orchestrator);

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

#endif // slic3r_Plugins_GCode_Reader_SprinterGCodeReader_hpp_
