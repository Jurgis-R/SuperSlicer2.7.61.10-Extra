///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Plugins_GCode_Reader_DefaultGCodeReader_hpp_
#define slic3r_Plugins_GCode_Reader_DefaultGCodeReader_hpp_

struct orchestrator_handle;

/*
Built-in G-code reader registration
===================================

This small public header exposes only registration. Parsing remains private to
DefaultGCodeReader.cpp and is reached later through the generic GCODE_READER
service selected by the Orchestrator.

    PluginLoader -> register_default_gcode_reader_plugin()
                 -> Orchestrator stores provider
                 -> read_gcode() selects and executes provider

The registered object is process-lived and stateless. Per-stream parser state
is created on the stack for each run, allowing concurrent Reader executions.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Reader {

// Register the built-in Marlin-like provider. The Orchestrator borrows the
// process-lived static plugin instance; this function transfers no ownership.
// Selection occurs later from the exclusive Reader provider group.
void register_default_gcode_reader_plugin(orchestrator_handle *orchestrator);

}}} // namespace slic3r_api::GCodeGeneration::Reader

#endif // slic3r_Plugins_GCode_Reader_DefaultGCodeReader_hpp_
