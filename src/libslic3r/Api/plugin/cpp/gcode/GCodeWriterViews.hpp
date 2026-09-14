///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_plugin_cpp_gcode_GCodeWriterViews_hpp_
#define slic3r_Api_plugin_cpp_gcode_GCodeWriterViews_hpp_

#include <stdexcept>
#include <string>
#include <string_view>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_gcode.h"

/*
STEP_GCODE writer helpers
=========================

A writer may produce an artifact which GCodeReader cannot consume directly,
such as binary output or a device stream. It therefore publishes the path of a
separate text representation after that file is complete. The host copies the
path but continues to borrow the file itself from its producer.
*/

namespace slic3r_api {

// Publish the finished text source used to build the G-code preview. This call
// does not transfer ownership of the file and may be made only once per run.
inline void publish_gcode_reader_source_path(const run_ctx_generate_gcode &context,
                                             std::string_view path)
{
    if (path.empty())
        throw std::invalid_argument("The G-code Reader source path must not be empty.");
    if (context.publish_reader_source_path == nullptr)
        throw std::runtime_error("The STEP_GCODE host did not provide a Reader source callback.");

    const std::string owned_path(path);
    if (!context.publish_reader_source_path(context.reader_source_context,
                                            owned_path.c_str()))
        throw std::runtime_error("The host rejected the G-code Reader source path.");
}

} // namespace slic3r_api

#endif // slic3r_Api_plugin_cpp_gcode_GCodeWriterViews_hpp_
