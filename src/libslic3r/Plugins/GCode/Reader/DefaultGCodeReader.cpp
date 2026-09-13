///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "DefaultGCodeReader.hpp"

#include "KlipperGCodeReader.hpp"
#include "MarlinGCodeReader.hpp"
#include "RepRapGCodeReader.hpp"

/*
Built-in G-code reader registration
===================================

The default Reader is a deliberately conservative concrete use of the shared
modal parser. This file also owns registration so the loader has one entry
point while each dialect keeps its implementation isolated in its own file.

    register_builtin_gcode_reader_plugins()
        +-- DefaultGCodeReader
        +-- MarlinGCodeReader
        +-- RepRapGCodeReader
        `-- KlipperGCodeReader

All four objects are process-lived and immutable. Per-run state belongs to the
base parser, allowing simultaneous probes and reads without shared mutation.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Reader {

DefaultGCodeReader::DefaultGCodeReader(orchestrator_handle *orchestrator) :
    BuiltinGCodeReaderBase(orchestrator)
{}

const char *DefaultGCodeReader::id_impl() const noexcept
{
    return "gcode.reader.default";
}

const char *DefaultGCodeReader::name_impl() const noexcept
{
    return "Default G-code reader";
}

const char *DefaultGCodeReader::description_impl() const noexcept
{
    return "Reads common Marlin-style G-code into normalized machine-state events.";
}

uint32_t DefaultGCodeReader::probe_impl(std::string_view) const
{
    // The default Reader is selected only as the service fallback. Claiming a
    // generic producer comment would hide a more appropriate external Reader.
    return 0;
}

void register_builtin_gcode_reader_plugins(orchestrator_handle *orchestrator)
{
    static DefaultGCodeReader default_reader(orchestrator);
    static MarlinGCodeReader marlin_reader(orchestrator);
    static RepRapGCodeReader reprap_reader(orchestrator);
    static KlipperGCodeReader klipper_reader(orchestrator);
    orchestrator_register_plugin(orchestrator, default_reader.c_instance());
    orchestrator_register_plugin(orchestrator, marlin_reader.c_instance());
    orchestrator_register_plugin(orchestrator, reprap_reader.c_instance());
    orchestrator_register_plugin(orchestrator, klipper_reader.c_instance());
}

}}} // namespace slic3r_api::GCodeGeneration::Reader
