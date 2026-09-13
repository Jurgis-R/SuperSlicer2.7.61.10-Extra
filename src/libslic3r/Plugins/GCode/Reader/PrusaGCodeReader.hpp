///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Plugins_GCode_Reader_PrusaGCodeReader_hpp_
#define slic3r_Plugins_GCode_Reader_PrusaGCodeReader_hpp_

#include "Marlin2GCodeReader.hpp"

/*
Prusa G-code reader
===================

Prusa firmware currently uses Marlin 2 movement and state commands, including
M204 P/T. Its distinct provider identity lets firmware selection and external
file probing choose Prusa explicitly. M601 is already represented as a marker
by the shared Marlin-family base.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Reader {

class PrusaGCodeReader final : public Marlin2GCodeReader
{
public:
    explicit PrusaGCodeReader(orchestrator_handle *orchestrator);

private:
    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    uint32_t probe_impl(std::string_view prefix) const override;
};

}}} // namespace slic3r_api::GCodeGeneration::Reader

#endif // slic3r_Plugins_GCode_Reader_PrusaGCodeReader_hpp_
