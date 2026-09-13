///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Plugins_GCode_Reader_KlipperGCodeReader_hpp_
#define slic3r_Plugins_GCode_Reader_KlipperGCodeReader_hpp_

#include "BuiltinGCodeReaderBase.hpp"

/*
Klipper G-code reader
=====================

Klipper combines standard G/M commands with named textual commands. Both
dispatchers call their parent first, then route only Klipper additions to
virtual handlers. Configured extruder names use the Print when available;
external files accept only conventional names.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Reader {

class KlipperGCodeReader : public BuiltinGCodeReaderBase
{
public:
    explicit KlipperGCodeReader(orchestrator_handle *orchestrator);

protected:
    // Chain textual dispatch through the parent before trying Klipper's named
    // commands. True is returned only after a process_*() handler runs.
    bool consume_text_command(const plugin_run_context &run_context,
                              const run_ctx_gcode_reader &context,
                              GCodeStateTracker &state,
                              const GCodeLineParser::GCodeLine &line) const override;
    // Chain portable numeric commands first, then add Klipper M204.
    bool consume_numeric_command(const plugin_run_context &run_context,
                                 const run_ctx_gcode_reader &context,
                                 GCodeStateTracker &state,
                                 const GCodeLineParser::GCodeLine &line,
                                 char letter,
                                 uint32_t number) const override;
    // Resolve EXTRUDER=name against the Print configuration or conventional
    // external-file names and publish the resulting active tool.
    virtual void process_ACTIVATE_EXTRUDER(
        const plugin_run_context &run_context,
        const run_ctx_gcode_reader &context,
        GCodeStateTracker &state,
        const GCodeLineParser::GCodeLine &line) const;
    // Preserve a valid pressure-advance command as a known state update. Its
    // value has no portable Reader item yet.
    virtual void process_SET_PRESSURE_ADVANCE(
        const plugin_run_context &run_context,
        const run_ctx_gcode_reader &context,
        GCodeStateTracker &state,
        const GCodeLineParser::GCodeLine &line) const;
    // Publish Klipper PAUSE as a timeline marker.
    virtual void process_PAUSE(const plugin_run_context &run_context,
                               const run_ctx_gcode_reader &context,
                               GCodeStateTracker &state) const;
    // Resolve Klipper's S/P/T forms to the single portable acceleration active
    // after the command.
    virtual void process_M204(const plugin_run_context &run_context,
                              const run_ctx_gcode_reader &context,
                              GCodeStateTracker &state,
                              const GCodeLineParser::GCodeLine &line) const;
    bool is_neutral_command(char letter, uint32_t number) const override;
    bool is_marker_command(char letter, uint32_t number) const override;

private:
    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    uint32_t probe_impl(std::string_view prefix) const override;
};

}}} // namespace slic3r_api::GCodeGeneration::Reader

#endif // slic3r_Plugins_GCode_Reader_KlipperGCodeReader_hpp_
