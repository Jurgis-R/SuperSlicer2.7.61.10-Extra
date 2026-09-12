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
service by its stable id, or as the fallback after no active Reader recognizes
an imported file.

    PluginLoader -> register_default_gcode_reader_plugin()
                 -> Orchestrator stores provider
                 -> firmware requests "gcode.reader.default"
                 -> read_gcode() validates and executes that exact provider

The registered object is process-lived and stateless. Per-stream parser state
is created on the stack for each run, allowing concurrent Reader executions.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Reader {

// Register the built-in Marlin-like provider. The Orchestrator borrows the
// process-lived static plugin instance; this function transfers no ownership.
// It does not join an exclusive group: every Reader may stay active so the host
// can probe all dialects and execute a specifically requested one.
void register_default_gcode_reader_plugin(orchestrator_handle *orchestrator);

}}} // namespace slic3r_api::GCodeGeneration::Reader

#endif // slic3r_Plugins_GCode_Reader_DefaultGCodeReader_hpp_
