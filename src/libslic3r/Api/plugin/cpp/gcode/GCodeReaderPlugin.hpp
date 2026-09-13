///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_plugin_cpp_gcode_GCodeReaderPlugin_hpp_
#define slic3r_Api_plugin_cpp_gcode_GCodeReaderPlugin_hpp_

#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_gcode_reader.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeStateTracker.hpp"

/*
G-code reader C++ provider skeleton
==================================

GCodeReaderPluginBase turns the synchronous C payload into a small C++
contract. A derived provider pulls borrowed chunks with next_chunk(), converts
them into normalized events and publishes those events with publish(). The base
class owns the tracker for one run and sends its complete final state after the
derived parser reaches end-of-stream.

Typical provider control flow:

    GCodeReaderPluginBase::run_impl()
        |
        +-- copy and validate initial_state -> GCodeStateTracker
        |
        +-- derived read_impl()
        |      +-- next_chunk() until false
        |      +-- parse complete commands
        |      `-- publish(event) for each semantic action
        |
        `-- emit_final_state(tracker.state())

The derived class never publishes the final state itself. This guarantees one
final callback even when different providers organize their parsing loops
differently. Exceptions and PluginCancelled are handled by PluginBase at the
outer plugin boundary.

No parser session or owning pointer crosses the ABI. The registered plugin may
be called concurrently, so all per-input parser state belongs in read_impl()
locals rather than fields of the plugin singleton.
*/

namespace slic3r_api {

class GCodeReaderPluginBase : public PluginBase
{
protected:
    explicit GCodeReaderPluginBase(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}

    // Implement one complete parse. run_context supplies common cancellation
    // and diagnostics; context supplies Reader callbacks; state is the mutable
    // owned copy of initial_state. The implementation must consume through the
    // end marker and publish every semantic command action through publish().
    // State-only bookkeeping such as the physical line cursor remains in the
    // tracker and appears in the final state. The implementation must not
    // retain any of these references after returning.
    virtual void read_impl(const plugin_run_context &run_context,
                           const run_ctx_gcode_reader &context,
                           GCodeStateTracker &state) const = 0;

    // Inspect the bounded prefix supplied by the host and return how strongly
    // it identifies this provider's dialect. Zero is the neutral answer used
    // by generic Readers; 100 is reserved for an exact identifying marker.
    virtual uint32_t probe_impl(std::string_view) const { return 0; }

    // Return the orchestrator-local identity which scopes provider-private
    // state. The portable schema has the permanent static id zero; a provider
    // with private keys registers a stable schema name and returns its dynamic
    // id. A mismatch is rejected before the provider sees any input chunk.
    virtual uint32_t reader_state_id_impl() const
    { return GCODE_READER_STATE_ID_PORTABLE; }

    // Check common plugin cancellation, request one host chunk and convert its
    // borrowed byte pair into a string_view. Return false only for the valid
    // null/zero end marker. The view becomes invalid at the next call because
    // a streaming host may refill the same buffer.
    static bool next_chunk(const plugin_run_context &run_context,
                           const run_ctx_gcode_reader &context,
                           std::string_view &bytes)
    {
        throw_if_cancelled(&run_context);
        raw_gcode_reader_chunk chunk = {};
        raw_gcode_reader_result result = make_result();
        context.get_next_chunk(context.callback_context, &chunk, &result);
        require_success(result, "The G-code reader input source failed.");
        if (chunk.bytes == nullptr && chunk.byte_count == 0) {
            bytes = {};
            return false;
        }
        if (chunk.bytes == nullptr || chunk.byte_count == 0 ||
            chunk.byte_count > uint64_t(std::numeric_limits<size_t>::max()))
            throw std::invalid_argument("The G-code reader input source returned an invalid chunk.");
        bytes = std::string_view(chunk.bytes, static_cast<size_t>(chunk.byte_count));
        return true;
    }

    // Check cancellation, validate and apply the event to the provider-owned
    // tracker, then lend the same event to the host. Applying first means a
    // host rejection aborts the run and no contradictory final state is sent.
    // The caller keeps ownership of event and its item array.
    static void publish(const plugin_run_context &run_context,
                        const run_ctx_gcode_reader &context,
                        GCodeStateTracker &state,
                        const raw_gcode_reader_event &event)
    {
        throw_if_cancelled(&run_context);
        state.apply(event);
        raw_gcode_reader_result result = make_result();
        context.emit_event(context.callback_context, &event, &result);
        require_success(result, "The host rejected a G-code reader event.");
    }

private:
    // Bind every derived provider to the non-pipeline GCODE_READER service.
    slicing_step_t step_impl() const noexcept final { return GCODE_READER; }

    // Readers are addressable services that may all remain active at once.
    // Returning an empty id opts out of PluginBase's default singleton group;
    // selection is performed explicitly by GCodeReaderService, never through
    // a generated PrintConfig selector.
    const char *exclusive_group_impl() const noexcept final { return ""; }

    // Validate the typed payload, create the per-run state, invoke the derived
    // parser and finally lend the complete state to the host. The final state
    // callback occurs only after read_impl() returned normally and cancellation
    // was checked once more, so failed or cancelled runs cannot look complete.
    void run_impl(const plugin_run_context *run_context) const final
    {
        run_ctx_gcode_reader *context = plugin_ctx_as_gcode_reader(run_context);
        if (context == nullptr || context->get_next_chunk == nullptr)
            throw std::invalid_argument("G-code reader received an incomplete synchronous context.");

        if (context->operation == RAW_GCODE_READER_OPERATION_PROBE) {
            if (context->emit_probe == nullptr)
                throw std::invalid_argument("G-code reader probe has no result callback.");
            std::string prefix;
            std::string_view chunk;
            while (next_chunk(*run_context, *context, chunk))
                prefix.append(chunk.data(), chunk.size());
            const uint32_t confidence = probe_impl(prefix);
            if (confidence > 100)
                throw std::invalid_argument("G-code reader probe confidence exceeds 100.");
            raw_gcode_reader_result result = make_result();
            context->emit_probe(context->callback_context, confidence, &result);
            require_success(result, "The host rejected the G-code reader probe result.");
            return;
        }

        if (context->operation != RAW_GCODE_READER_OPERATION_READ ||
            context->initial_state == nullptr || context->emit_event == nullptr ||
            context->emit_final_state == nullptr)
            throw std::invalid_argument("G-code reader received an incomplete read context.");

        GCodeStateTracker state(*context->initial_state);
        if (state.reader_id() != reader_state_id_impl())
            throw std::invalid_argument("The initial G-code Reader state belongs to another Reader identity.");
        read_impl(*run_context, *context, state);
        throw_if_cancelled(run_context);
        if (state.reader_id() != reader_state_id_impl())
            throw std::invalid_argument("The G-code Reader changed its state identity while parsing.");

        raw_gcode_reader_result result = make_result();
        context->emit_final_state(context->callback_context, &state.state(), &result);
        require_success(result, "The host rejected the G-code reader final state.");
    }

    // Create the required sentinel result. A callback which forgets to set a
    // status is rejected by require_success() rather than accepted by default.
    static raw_gcode_reader_result make_result()
    {
        raw_gcode_reader_result result = {};
        result.status = RAW_GCODE_READER_STATUS_UNSET;
        return result;
    }

    // Translate the C result channel back to the C++ exception vocabulary used
    // by PluginBase. The callback's diagnostic is copied into std::string before
    // the callback context may replace or destroy its buffer.
    static void require_success(const raw_gcode_reader_result &result, const char *fallback)
    {
        if (result.status == RAW_GCODE_READER_STATUS_SUCCESS)
            return;
        const std::string message = result.error_message != nullptr ? result.error_message : fallback;
        if (result.status == RAW_GCODE_READER_STATUS_INVALID_ARGUMENT)
            throw std::invalid_argument(message);
        throw std::runtime_error(message);
    }
};

} // namespace slic3r_api

#endif // slic3r_Api_plugin_cpp_gcode_GCodeReaderPlugin_hpp_
