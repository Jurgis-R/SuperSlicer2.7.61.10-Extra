///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "StepReadGeneratedGcode.hpp"

#include <array>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>

#include <boost/nowide/fstream.hpp>

#include "libslic3r/Api/host/GCodeReaderService.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Print.hpp"

/*
Generated G-code preview implementation
=======================================

Only a reusable fixed-size buffer is retained while parsing. Every returned
string_view remains valid until the Reader requests the next chunk, matching
the Reader callback contract and allowing multi-gigabyte files to be processed
without loading them completely into memory.
*/

namespace Slic3r::Steps::StepReadGeneratedGcode {

void run(Orchestrator &orchestrator,
         Print &print,
         std::string_view reader_plugin_id,
         GCodeProcessorResult &result)
{
    const std::string source_path = print.gcode_reader_source_path();
    if (source_path.empty())
        throw std::runtime_error("The G-code writer did not publish a Reader source path.");

    boost::nowide::ifstream stream(source_path.c_str(), std::ios::in | std::ios::binary);
    if (!stream)
        throw std::runtime_error("Cannot open the G-code Reader source file '" + source_path + "'.");

    std::array<char, 64u * 1024u> buffer = {};
    bool finished = false;
    GCodeReaderChunkSource source = [&stream, &buffer, &finished]() -> std::string_view {
        if (finished)
            return {};

        stream.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
        const std::streamsize count = stream.gcount();
        if (count > 0)
            return std::string_view(buffer.data(), static_cast<size_t>(count));
        if (stream.bad() || !stream.eof())
            throw std::runtime_error("Failed while reading the G-code Reader source file.");

        finished = true;
        return {};
    };

    GCodeReaderService reader(orchestrator, &print);
    GCodeProcessorResult built = reader.read_for_viewer(reader_plugin_id, std::move(source));
    built.filename = source_path;
    result = std::move(built);
}

} // namespace Slic3r::Steps::StepReadGeneratedGcode
