///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "GCodeReaderService.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <optional>
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

ReaderInvocation is the temporary bridge used during one plugin run. It owns
the input callback, validates the order of all provider callbacks and copies
the final state. Its static C callbacks catch every exception so neither the
host source nor the state validator can unwind through the plugin ABI.

    GCodeReaderChunkSource
             |
             v
      ReaderInvocation::get_next_chunk_callback
             |
             v
       provider read_impl
          |          |
          | events   | final state
          v          v
      m_event_state  m_final_state

m_event_state is intentionally independent from m_final_state. Replaying each
event validates the event stream as it arrives. The separately copied final
state also includes state changes, such as physical line advancement, which do
not need an action event.
*/

namespace Slic3r {
namespace {

class ReaderInvocation
{
public:
    // Copy the borrowed initial state and take ownership of the source callable
    // for one synchronous execution. Input bytes remain owned by that source.
    ReaderInvocation(GCodeReaderChunkSource source,
                     const raw_gcode_reader_state &initial_state);

    // Build the borrowed C payload whose callback_context points back to this
    // invocation. The payload must not outlive the ReaderInvocation object.
    run_ctx_gcode_reader context();
    // Move the host-owned final state out after successful plugin execution.
    // A successful provider which omitted the final callback violates the API.
    slic3r_api::GCodeStateTracker take_final_state();
    // Return the last callback diagnostic copied into host-owned storage.
    const std::string &error() const { return m_error; }

private:
    // C ABI entry points. They validate pointers, translate exceptions into a
    // raw result and never permit C++ unwinding through provider code.
    static void get_next_chunk_callback(void *opaque,
                                        raw_gcode_reader_chunk *chunk,
                                        raw_gcode_reader_result *result) noexcept;
    static void emit_event_callback(void *opaque,
                                    const raw_gcode_reader_event *event,
                                    raw_gcode_reader_result *result) noexcept;
    static void emit_final_state_callback(void *opaque,
                                          const raw_gcode_reader_state *state,
                                          raw_gcode_reader_result *result) noexcept;
    // Implement the ordered host side of the three Reader callbacks.
    void get_next_chunk(raw_gcode_reader_chunk &chunk);
    void emit_event(const raw_gcode_reader_event &event);
    void emit_final_state(const raw_gcode_reader_state &state);
    // Keep the diagnostic alive in this invocation and expose it through the
    // borrowed result channel. A null result cannot receive the diagnostic.
    void fail(raw_gcode_reader_result *result,
              raw_gcode_reader_status status,
              const std::string &message) noexcept;
    // Mark a non-null callback result as accepted and clear stale diagnostics.
    static void initialize_success(raw_gcode_reader_result *result) noexcept;
    // Source bytes are borrowed through m_active_chunk until the next pull.
    GCodeReaderChunkSource m_source;
    // Initial and replay states validate host input and provider events. Only
    // m_final_state is returned to the service caller.
    slic3r_api::GCodeStateTracker m_initial_state;
    slic3r_api::GCodeStateTracker m_event_state;
    std::optional<slic3r_api::GCodeStateTracker> m_final_state;
    std::string_view m_active_chunk;
    std::string m_error;
    uint64_t m_supplied_bytes = 0;
    bool m_end_reached = false;
};

ReaderInvocation::ReaderInvocation(GCodeReaderChunkSource source,
                                   const raw_gcode_reader_state &initial_state)
    : m_source(std::move(source)), m_initial_state(initial_state),
      m_event_state(initial_state)
{
    if (!m_source)
        throw std::invalid_argument("G-code reader requires an input chunk source.");
}

run_ctx_gcode_reader ReaderInvocation::context()
{
    run_ctx_gcode_reader value = {};
    value.operation = RAW_GCODE_READER_OPERATION_READ;
    value.initial_state = &m_initial_state.state();
    value.callback_context = this;
    value.get_next_chunk = &ReaderInvocation::get_next_chunk_callback;
    value.emit_event = &ReaderInvocation::emit_event_callback;
    value.emit_final_state = &ReaderInvocation::emit_final_state_callback;
    return value;
}

slic3r_api::GCodeStateTracker ReaderInvocation::take_final_state()
{
    if (!m_final_state)
        throw std::runtime_error("The G-code reader returned successfully without a final state.");
    return std::move(*m_final_state);
}

void ReaderInvocation::get_next_chunk(raw_gcode_reader_chunk &chunk)
{
    if (m_final_state)
        throw std::logic_error("The G-code reader requested input after publishing its final state.");
    chunk = {};
    if (m_end_reached)
        return;

    // Keep the view in the invocation so its lifetime boundary is visible: a
    // subsequent source call may reuse the storage behind the previous view.
    m_active_chunk = m_source();
    if (m_active_chunk.empty()) {
        m_end_reached = true;
        return;
    }
    if (m_active_chunk.size() > std::numeric_limits<uint64_t>::max() - m_supplied_bytes)
        throw std::overflow_error("G-code reader input exceeds the uint64_t source range.");
    m_supplied_bytes += static_cast<uint64_t>(m_active_chunk.size());
    chunk.bytes = m_active_chunk.data();
    chunk.byte_count = static_cast<uint64_t>(m_active_chunk.size());
}

void ReaderInvocation::emit_event(const raw_gcode_reader_event &event)
{
    if (m_final_state)
        throw std::logic_error("The G-code reader emitted an event after its final state.");
    if (event.source.end > m_supplied_bytes)
        throw std::invalid_argument("A G-code reader event refers to bytes not supplied by the host.");

    // Replay independently from the provider. apply() validates the complete
    // event before mutating this host-owned state. The separately copied final
    // state remains authoritative for state changes which intentionally have
    // no action event.
    m_event_state.apply(event);
}

void ReaderInvocation::emit_final_state(const raw_gcode_reader_state &state)
{
    if (!m_end_reached)
        throw std::logic_error("The G-code reader published its final state before reaching end-of-stream.");
    if (m_final_state)
        throw std::logic_error("The G-code reader published more than one final state.");

    // Empty and comment-only lines advance the provider's line cursor without
    // producing action events, so the final state is authoritative rather than
    // required to equal the host's event replay byte-for-byte.
    m_final_state.emplace(state);
}

void ReaderInvocation::initialize_success(raw_gcode_reader_result *result) noexcept
{
    if (result == nullptr)
        return;
    result->status = RAW_GCODE_READER_STATUS_SUCCESS;
    result->error_message = nullptr;
}

void ReaderInvocation::fail(raw_gcode_reader_result *result,
                            raw_gcode_reader_status status,
                            const std::string &message) noexcept
{
    m_error = message;
    if (result == nullptr)
        return;
    result->status = status;
    result->error_message = m_error.c_str();
}

void ReaderInvocation::get_next_chunk_callback(void *opaque,
                                               raw_gcode_reader_chunk *chunk,
                                               raw_gcode_reader_result *result) noexcept
{
    ReaderInvocation *invocation = static_cast<ReaderInvocation *>(opaque);
    if (invocation == nullptr || chunk == nullptr) {
        if (invocation != nullptr)
            invocation->fail(result, RAW_GCODE_READER_STATUS_INVALID_ARGUMENT,
                             "G-code reader requested a chunk with an invalid output structure.");
        return;
    }
    try {
        invocation->get_next_chunk(*chunk);
        initialize_success(result);
    } catch (const std::invalid_argument &error) {
        invocation->fail(result, RAW_GCODE_READER_STATUS_INVALID_ARGUMENT, error.what());
    } catch (const std::exception &error) {
        invocation->fail(result, RAW_GCODE_READER_STATUS_ERROR, error.what());
    } catch (...) {
        invocation->fail(result, RAW_GCODE_READER_STATUS_ERROR,
                         "Unknown exception while supplying G-code input.");
    }
}

void ReaderInvocation::emit_event_callback(void *opaque,
                                           const raw_gcode_reader_event *event,
                                           raw_gcode_reader_result *result) noexcept
{
    ReaderInvocation *invocation = static_cast<ReaderInvocation *>(opaque);
    if (invocation == nullptr || event == nullptr) {
        if (invocation != nullptr)
            invocation->fail(result, RAW_GCODE_READER_STATUS_INVALID_ARGUMENT,
                             "G-code reader emitted a null event.");
        return;
    }
    try {
        invocation->emit_event(*event);
        initialize_success(result);
    } catch (const std::invalid_argument &error) {
        invocation->fail(result, RAW_GCODE_READER_STATUS_INVALID_ARGUMENT, error.what());
    } catch (const std::exception &error) {
        invocation->fail(result, RAW_GCODE_READER_STATUS_ERROR, error.what());
    } catch (...) {
        invocation->fail(result, RAW_GCODE_READER_STATUS_ERROR,
                         "Unknown exception while receiving a G-code reader event.");
    }
}

void ReaderInvocation::emit_final_state_callback(void *opaque,
                                                 const raw_gcode_reader_state *state,
                                                 raw_gcode_reader_result *result) noexcept
{
    ReaderInvocation *invocation = static_cast<ReaderInvocation *>(opaque);
    if (invocation == nullptr || state == nullptr) {
        if (invocation != nullptr)
            invocation->fail(result, RAW_GCODE_READER_STATUS_INVALID_ARGUMENT,
                             "G-code reader emitted a null final state.");
        return;
    }
    try {
        invocation->emit_final_state(*state);
        initialize_success(result);
    } catch (const std::invalid_argument &error) {
        invocation->fail(result, RAW_GCODE_READER_STATUS_INVALID_ARGUMENT, error.what());
    } catch (const std::exception &error) {
        invocation->fail(result, RAW_GCODE_READER_STATUS_ERROR, error.what());
    } catch (...) {
        invocation->fail(result, RAW_GCODE_READER_STATUS_ERROR,
                         "Unknown exception while receiving the G-code reader final state.");
    }
}

} // namespace

slic3r_api::GCodeStateTracker read_gcode(
    Orchestrator &orchestrator,
    Print *print,
    std::string_view reader_plugin_id,
    const raw_gcode_reader_state &initial_state,
    GCodeReaderChunkSource source)
{
    const std::string id = reader_plugin_id.empty() ? DEFAULT_GCODE_READER_PLUGIN_ID :
        std::string(reader_plugin_id);
    Plugin *plugin = orchestrator.get_plugin(id);
    if (plugin == nullptr)
        throw std::runtime_error("G-code reader plugin '" + id + "' is not loaded.");
    if (plugin->get_step() != GCODE_READER)
        throw std::runtime_error("Plugin '" + id + "' is not a G-code reader.");
    if (!orchestrator.is_plugin_active(plugin))
        throw std::runtime_error("G-code reader plugin '" + id + "' is inactive.");

    // invocation outlives execute_plugin(), therefore callback_context and all
    // borrowed diagnostics remain valid for every provider callback.
    ReaderInvocation invocation(std::move(source), initial_state);
    run_ctx_gcode_reader context = invocation.context();
    raw_plugin_run_payload payload = {};
    payload.data = &context;
    const raw_plugin_execution_status status = orchestrator.execute_plugin(*plugin, print, &payload, 1);
    if (status != RAW_PLUGIN_EXECUTION_SUCCESS) {
        const std::string message = invocation.error().empty() ?
            "The selected G-code reader provider failed." : invocation.error();
        throw std::runtime_error(message);
    }
    return invocation.take_final_state();
}

slic3r_api::GCodeStateTracker read_gcode(
    Orchestrator &orchestrator,
    Print *print,
    std::string_view reader_plugin_id,
    const raw_gcode_reader_state &initial_state,
    std::string_view text)
{
    // Adapt the in-memory text to the same pull protocol as streamed files.
    // The second call is the explicit end marker required by the contract.
    bool supplied = false;
    GCodeReaderChunkSource source = [text, &supplied]() mutable -> std::string_view {
        if (supplied)
            return {};
        supplied = true;
        return text;
    };
    return read_gcode(orchestrator, print, reader_plugin_id, initial_state, std::move(source));
}

namespace {

class ProbeInvocation
{
public:
    explicit ProbeInvocation(std::string_view prefix);
    run_ctx_gcode_reader context();
    uint32_t confidence() const;

private:
    static void next_callback(void *opaque, raw_gcode_reader_chunk *chunk,
                              raw_gcode_reader_result *result) noexcept;

    static void probe_callback(void *opaque, uint32_t confidence,
                               raw_gcode_reader_result *result) noexcept;

    std::string_view m_prefix;
    uint32_t m_confidence = 0;
    bool m_supplied = false;
    bool m_reported = false;
};

ProbeInvocation::ProbeInvocation(std::string_view prefix) : m_prefix(prefix) {}

run_ctx_gcode_reader ProbeInvocation::context()
{
    run_ctx_gcode_reader value = {};
    value.operation = RAW_GCODE_READER_OPERATION_PROBE;
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
        ProbeInvocation invocation(bounded);
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
      m_service{this, &GCodeReaderService::read_callback}
{}

slic3r_api::GCodeStateTracker GCodeReaderService::read(
    std::string_view reader_plugin_id,
    const raw_gcode_reader_state &initial_state,
    GCodeReaderChunkSource source) const
{
    return read_gcode(m_orchestrator, m_print, reader_plugin_id, initial_state,
                      std::move(source));
}

slic3r_api::GCodeStateTracker GCodeReaderService::read(
    std::string_view reader_plugin_id,
    const raw_gcode_reader_state &initial_state,
    std::string_view text) const
{
    return read_gcode(m_orchestrator, m_print, reader_plugin_id, initial_state, text);
}

std::string GCodeReaderService::detect(std::string_view prefix) const
{
    return detect_gcode_reader(m_orchestrator, m_print, prefix);
}

std::string GCodeReaderService::validated_reader_id(std::string_view reader_plugin_id) const
{
    const std::string id = reader_plugin_id.empty() ? DEFAULT_GCODE_READER_PLUGIN_ID :
        std::string(reader_plugin_id);
    Plugin *plugin = m_orchestrator.get_plugin(id);
    if (plugin == nullptr)
        throw std::runtime_error("G-code reader plugin '" + id + "' is not loaded.");
    if (plugin->get_step() != GCODE_READER)
        throw std::runtime_error("Plugin '" + id + "' is not a G-code reader.");
    if (!m_orchestrator.is_plugin_active(plugin))
        throw std::runtime_error("G-code reader plugin '" + id + "' is inactive.");
    return id;
}

void GCodeReaderService::read_from_callbacks(
    const char *reader_plugin_id,
    const raw_gcode_reader_state &initial_state,
    void *source_context,
    gcode_reader_get_next_chunk_fn get_next_chunk,
    void *final_state_context,
    gcode_reader_emit_final_state_fn receive_final_state)
{
    if (get_next_chunk == nullptr || receive_final_state == nullptr)
        throw std::invalid_argument("G-code Reader service received incomplete callbacks.");
    GCodeReaderChunkSource source = [source_context, get_next_chunk, this]() -> std::string_view {
        raw_gcode_reader_chunk chunk = {};
        raw_gcode_reader_result result = {};
        result.status = RAW_GCODE_READER_STATUS_UNSET;
        get_next_chunk(source_context, &chunk, &result);
        if (result.status != RAW_GCODE_READER_STATUS_SUCCESS)
            throw std::runtime_error(result.error_message != nullptr ? result.error_message :
                                     "G-code Reader service input callback failed.");
        if (chunk.bytes == nullptr && chunk.byte_count == 0)
            return {};
        if (chunk.bytes == nullptr || chunk.byte_count == 0 ||
            chunk.byte_count > uint64_t(std::numeric_limits<size_t>::max()))
            throw std::invalid_argument("G-code Reader service input callback returned an invalid chunk.");
        return std::string_view(chunk.bytes, static_cast<size_t>(chunk.byte_count));
    };
    const slic3r_api::GCodeStateTracker state = read_gcode(
        m_orchestrator, m_print, reader_plugin_id != nullptr ? reader_plugin_id : "",
        initial_state, std::move(source));
    raw_gcode_reader_result result = {};
    result.status = RAW_GCODE_READER_STATUS_UNSET;
    receive_final_state(final_state_context, &state.state(), &result);
    if (result.status != RAW_GCODE_READER_STATUS_SUCCESS)
        throw std::runtime_error(result.error_message != nullptr ? result.error_message :
                                 "G-code Reader service final-state callback failed.");
}

void GCodeReaderService::read_callback(
    void *service_context, const char *reader_plugin_id,
    const raw_gcode_reader_state *initial_state, void *source_context,
    gcode_reader_get_next_chunk_fn get_next_chunk, void *final_state_context,
    gcode_reader_emit_final_state_fn receive_final_state,
    raw_gcode_reader_result *result) noexcept
{
    GCodeReaderService *service = static_cast<GCodeReaderService *>(service_context);
    if (result == nullptr)
        return;
    if (service == nullptr || initial_state == nullptr) {
        result->status = RAW_GCODE_READER_STATUS_INVALID_ARGUMENT;
        result->error_message = "G-code Reader service received an invalid context or initial state.";
        return;
    }
    const std::lock_guard<std::mutex> lock(service->m_mutex);
    try {
        service->read_from_callbacks(reader_plugin_id, *initial_state, source_context,
                                     get_next_chunk, final_state_context,
                                     receive_final_state);
        result->status = RAW_GCODE_READER_STATUS_SUCCESS;
        result->error_message = nullptr;
    } catch (const std::exception &error) {
        service->m_error = error.what();
        result->status = RAW_GCODE_READER_STATUS_ERROR;
        result->error_message = service->m_error.c_str();
    } catch (...) {
        service->m_error = "Unknown error while executing the requested G-code Reader.";
        result->status = RAW_GCODE_READER_STATUS_ERROR;
        result->error_message = service->m_error.c_str();
    }
}

} // namespace Slic3r
