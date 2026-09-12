///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "GCodeReaderService.hpp"

#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_gcode_reader.h"
#include "libslic3r/Print.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"

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
    const raw_gcode_reader_state &initial_state,
    GCodeReaderChunkSource source)
{
    // Service providers use a generated exclusive selector. Without a Print,
    // selection falls back to the first active provider in deterministic
    // orchestrator order, which is suitable for tests and generic scripts.
    const ConfigBase *config = print == nullptr ? nullptr : &print->full_print_config();
    Plugin *plugin = Steps::selected_or_active_plugin_for_step(orchestrator, GCODE_READER, config);
    if (plugin == nullptr)
        throw std::runtime_error("No active G-code reader provider is available.");

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
    return read_gcode(orchestrator, print, initial_state, std::move(source));
}

} // namespace Slic3r
