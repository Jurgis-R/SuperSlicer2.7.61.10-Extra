///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "PrusaGCodeReader.hpp"

/*
Prusa Reader provider
=====================

The implementation inherits Marlin 2 parsing and contributes only its stable
service identity and high-confidence firmware marker. This keeps the current
small difference explicit and gives later Prusa commands a dedicated class.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Reader {

PrusaGCodeReader::PrusaGCodeReader(orchestrator_handle *orchestrator) :
    Marlin2GCodeReader(orchestrator)
{}

const char *PrusaGCodeReader::id_impl() const noexcept
{
    return "gcode.reader.prusa";
}

const char *PrusaGCodeReader::name_impl() const noexcept
{
    return "Prusa G-code reader";
}

const char *PrusaGCodeReader::description_impl() const noexcept
{
    return "Reads Prusa firmware G-code into normalized machine-state events.";
}

uint32_t PrusaGCodeReader::probe_impl(std::string_view prefix) const
{
    return probe_marker(prefix, "prusa");
}

}}} // namespace slic3r_api::GCodeGeneration::Reader
