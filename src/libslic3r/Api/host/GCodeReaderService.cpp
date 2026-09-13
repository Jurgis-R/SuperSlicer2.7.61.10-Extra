///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "GCodeReaderService.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <boost/log/trivial.hpp>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_gcode_reader.h"
#include "libslic3r/Print.hpp"

/*
Host G-code reader service implementation
==========================================

The plugin-side GCodeReaderService owns the complete synchronous read bridge:
it supplies chunks, validates callbacks and copies the final state. This host
facade only adds automatic probing for files whose producing firmware is not
already known.

    known Reader id --> plugin helper --> exact Reader plugin
    unknown dialect --> host probes ----^
*/

namespace Slic3r {

slic3r_api::GCodeStateTracker read_gcode(
    Orchestrator &orchestrator,
    Print *print,
    std::string_view reader_plugin_id,
    const raw_gcode_reader_state &initial_state,
    GCodeReaderChunkSource source)
{
    return slic3r_api::GCodeReaderService(
        reinterpret_cast<orchestrator_handle *>(&orchestrator),
        reinterpret_cast<const print_handle *>(print)).read(
            reader_plugin_id, initial_state, std::move(source));
}

slic3r_api::GCodeStateTracker read_gcode(
    Orchestrator &orchestrator,
    Print *print,
    std::string_view reader_plugin_id,
    const raw_gcode_reader_state &initial_state,
    std::string_view text)
{
    return slic3r_api::GCodeReaderService(
        reinterpret_cast<orchestrator_handle *>(&orchestrator),
        reinterpret_cast<const print_handle *>(print)).read(
            reader_plugin_id, initial_state, text);
}

namespace {

class ProbeInvocation
{
public:
    ProbeInvocation(std::string_view prefix, const Print *print);
    run_ctx_gcode_reader context();
    uint32_t confidence() const;

private:
    static void next_callback(void *opaque, raw_gcode_reader_chunk *chunk,
                              raw_gcode_reader_result *result) noexcept;

    static void probe_callback(void *opaque, uint32_t confidence,
                               raw_gcode_reader_result *result) noexcept;

    std::string_view m_prefix;
    const Print *m_print;
    uint32_t m_confidence = 0;
    bool m_supplied = false;
    bool m_reported = false;
};

ProbeInvocation::ProbeInvocation(std::string_view prefix, const Print *print)
    : m_prefix(prefix), m_print(print)
{}

run_ctx_gcode_reader ProbeInvocation::context()
{
    run_ctx_gcode_reader value = {};
    value.operation = RAW_GCODE_READER_OPERATION_PROBE;
    value.print = reinterpret_cast<const print_handle *>(m_print);
    value.callback_context = this;
    value.get_next_chunk = &ProbeInvocation::next_callback;
    value.emit_probe = &ProbeInvocation::probe_callback;
    return value;
}

uint32_t ProbeInvocation::confidence() const
{
    if (!m_reported)
        throw std::runtime_error("The G-code reader probe returned without a confidence result.");
    return m_confidence;
}

void ProbeInvocation::next_callback(void *opaque, raw_gcode_reader_chunk *chunk,
                                    raw_gcode_reader_result *result) noexcept
{
    ProbeInvocation *invocation = static_cast<ProbeInvocation *>(opaque);
    if (invocation == nullptr || chunk == nullptr || result == nullptr)
        return;
    *chunk = {};
    if (!invocation->m_supplied) {
        invocation->m_supplied = true;
        if (!invocation->m_prefix.empty()) {
            chunk->bytes = invocation->m_prefix.data();
            chunk->byte_count = static_cast<uint64_t>(invocation->m_prefix.size());
        }
    }
    result->status = RAW_GCODE_READER_STATUS_SUCCESS;
    result->error_message = nullptr;
}

void ProbeInvocation::probe_callback(void *opaque, uint32_t confidence,
                                     raw_gcode_reader_result *result) noexcept
{
    ProbeInvocation *invocation = static_cast<ProbeInvocation *>(opaque);
    if (invocation == nullptr || result == nullptr)
        return;
    if (invocation->m_reported || confidence > 100) {
        result->status = RAW_GCODE_READER_STATUS_INVALID_ARGUMENT;
        result->error_message = "A G-code reader probe published an invalid confidence result.";
        return;
    }
    invocation->m_reported = true;
    invocation->m_confidence = confidence;
    result->status = RAW_GCODE_READER_STATUS_SUCCESS;
    result->error_message = nullptr;
}

} // namespace

std::string detect_gcode_reader(Orchestrator &orchestrator, Print *print,
                                std::string_view prefix)
{
    const std::string_view bounded = prefix.substr(0, GCODE_READER_PROBE_PREFIX_SIZE);
    Plugin *winner = nullptr;
    uint32_t winner_confidence = 0;
    const std::vector<Plugin *> readers = orchestrator.get_active_plugins_for_step(GCODE_READER);
    for (Plugin *plugin : readers) {
        ProbeInvocation invocation(bounded, print);
        run_ctx_gcode_reader context = invocation.context();
        raw_plugin_run_payload payload = {};
        payload.data = &context;
        // Probes are independent candidates. Their diagnostics are retained,
        // but a provider failure must not cancel or suppress later probes.
        if (orchestrator.execute_plugin(*plugin, print, &payload, 1, false) !=
            RAW_PLUGIN_EXECUTION_SUCCESS)
            continue;
        uint32_t confidence = 0;
        try {
            confidence = invocation.confidence();
        } catch (const std::exception &error) {
            // A malformed probe is local to its provider. Keep its diagnostic
            // visible while allowing the remaining Readers to compete.
            BOOST_LOG_TRIVIAL(warning) << "G-code Reader probe '" << plugin->get_id()
                                       << "' failed: " << error.what();
            continue;
        }
        if (confidence == 0)
            continue;
        if (winner == nullptr || confidence > winner_confidence ||
            (confidence == winner_confidence &&
             (plugin->get_priority() < winner->get_priority() ||
              (plugin->get_priority() == winner->get_priority() && plugin->get_id() < winner->get_id())))) {
            winner = plugin;
            winner_confidence = confidence;
        }
    }
    if (winner != nullptr)
        return winner->get_id();

    Plugin *fallback = orchestrator.get_plugin(DEFAULT_GCODE_READER_PLUGIN_ID);
    if (fallback == nullptr || fallback->get_step() != GCODE_READER ||
        !orchestrator.is_plugin_active(fallback))
        throw std::runtime_error("No probe recognized the G-code and the default Reader is unavailable.");
    return DEFAULT_GCODE_READER_PLUGIN_ID;
}

GCodeReaderService::GCodeReaderService(Orchestrator &orchestrator, Print *print)
    : m_orchestrator(orchestrator), m_print(print),
      m_reader(reinterpret_cast<orchestrator_handle *>(&orchestrator),
               reinterpret_cast<const print_handle *>(print))
{}

slic3r_api::GCodeStateTracker GCodeReaderService::read(
    std::string_view reader_plugin_id,
    const raw_gcode_reader_state &initial_state,
    GCodeReaderChunkSource source) const
{
    return m_reader.read(reader_plugin_id, initial_state, std::move(source));
}

slic3r_api::GCodeStateTracker GCodeReaderService::read(
    std::string_view reader_plugin_id,
    const raw_gcode_reader_state &initial_state,
    std::string_view text) const
{
    return m_reader.read(reader_plugin_id, initial_state, text);
}

std::string GCodeReaderService::detect(std::string_view prefix) const
{
    return detect_gcode_reader(m_orchestrator, m_print, prefix);
}

std::string GCodeReaderService::validated_reader_id(std::string_view reader_plugin_id) const
{
    return m_reader.validated_reader_id(reader_plugin_id);
}

} // namespace Slic3r
