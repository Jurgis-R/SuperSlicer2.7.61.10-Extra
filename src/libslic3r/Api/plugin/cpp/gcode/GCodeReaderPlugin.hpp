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

/*
Firmware-side client for the host G-code Reader service
=======================================================

GCodeReaderServiceView is not a Reader provider and does not own or construct
one. It is a non-owning adapter around raw_gcode_reader_service, which the host
places in run_ctx_gcode_firmware. A firmware uses this view when text that it
has just generated, such as custom G-code, must be interpreted by the same
Reader dialect as the completed output.

The call chain is deliberately synchronous:

    firmware session
        `-- GCodeReaderServiceView::read(reader id, initial state, text)
              |-- lends the text to the host through next()
              |-- host resolves and runs exactly the requested Reader
              `-- final_state() copies the Reader result into GCodeStateTracker

An empty Reader id asks for gcode.reader.default. A non-empty id is exact: this
view performs no probing and never falls back to another provider.

The raw service pointer is borrowed and is valid only until the firmware
session is destroyed. The input text and initial state need to remain valid
only for read(), because the service consumes them before returning. The
returned GCodeStateTracker owns its copied state and is independent of all
temporary C callbacks. Do not retain this view or its service pointer beyond
the firmware session lifetime.
*/
class GCodeReaderServiceView
{
public:
    // Validate and borrow the host-owned service. Construction transfers no
    // ownership and destruction performs no host call.
    explicit GCodeReaderServiceView(const raw_gcode_reader_service *service) : m_service(service)
    {
        if (service == nullptr || service->read == nullptr)
            throw std::invalid_argument("G-code Reader service is incomplete.");
    }

    // Parse one in-memory fragment with the requested provider. This helper
    // exposes the host's chunked Reader contract as a convenient single-string
    // operation for firmware-generated fragments; the host still feeds that
    // string to the provider through the normal chunk callback. The final
    // state is copied before the synchronous service call returns.
    GCodeStateTracker read(std::string_view reader_plugin_id,
                           const raw_gcode_reader_state &initial_state,
                           std::string_view text) const
    {
        Invocation invocation(text);
        raw_gcode_reader_result result = {};
        result.status = RAW_GCODE_READER_STATUS_UNSET;
        const std::string id(reader_plugin_id);
        m_service->read(m_service->context, id.c_str(), &initial_state,
                        &invocation, &Invocation::next,
                        &invocation, &Invocation::final_state, &result);
        require_success(result, "G-code Reader service failed.");
        if (!invocation.state)
            throw std::runtime_error("G-code Reader service returned without a final state.");
        return std::move(*invocation.state);
    }

private:
    // Invocation owns all adapters needed by one synchronous read(). Its text
    // is borrowed from read()'s caller, while state is the owned copy produced
    // by the final-state callback. Neither C callback nor its opaque pointer
    // may escape raw_gcode_reader_service::read().
    struct Invocation {
        explicit Invocation(std::string_view input) : text(input) {}

        static void next(void *opaque, raw_gcode_reader_chunk *chunk,
                         raw_gcode_reader_result *result) noexcept
        {
            Invocation *invocation = static_cast<Invocation *>(opaque);
            if (invocation == nullptr || chunk == nullptr || result == nullptr)
                return;
            try {
                *chunk = {};
                if (!invocation->supplied && !invocation->text.empty()) {
                    invocation->supplied = true;
                    chunk->bytes = invocation->text.data();
                    chunk->byte_count = static_cast<uint64_t>(invocation->text.size());
                } else {
                    invocation->supplied = true;
                }
                result->status = RAW_GCODE_READER_STATUS_SUCCESS;
                result->error_message = nullptr;
            } catch (const std::exception &exception) {
                invocation->error = exception.what();
                result->status = RAW_GCODE_READER_STATUS_ERROR;
                result->error_message = invocation->error.c_str();
            } catch (...) {
                invocation->error = "Unknown error while supplying Reader input.";
                result->status = RAW_GCODE_READER_STATUS_ERROR;
                result->error_message = invocation->error.c_str();
            }
        }

        static void final_state(void *opaque, const raw_gcode_reader_state *state,
                                raw_gcode_reader_result *result) noexcept
        {
            Invocation *invocation = static_cast<Invocation *>(opaque);
            if (invocation == nullptr || result == nullptr)
                return;
            if (state == nullptr) {
                result->status = RAW_GCODE_READER_STATUS_INVALID_ARGUMENT;
                result->error_message = "G-code Reader service supplied a null final state.";
                return;
            }
            try {
                invocation->state.emplace(*state);
                result->status = RAW_GCODE_READER_STATUS_SUCCESS;
                result->error_message = nullptr;
            } catch (const std::exception &exception) {
                invocation->error = exception.what();
                result->status = RAW_GCODE_READER_STATUS_INVALID_ARGUMENT;
                result->error_message = invocation->error.c_str();
            } catch (...) {
                invocation->error = "Unknown error while copying the Reader final state.";
                result->status = RAW_GCODE_READER_STATUS_ERROR;
                result->error_message = invocation->error.c_str();
            }
        }

        std::string_view text;
        std::optional<GCodeStateTracker> state;
        std::string error;
        bool supplied = false;
    };

    static void require_success(const raw_gcode_reader_result &result, const char *fallback)
    {
        if (result.status == RAW_GCODE_READER_STATUS_SUCCESS)
            return;
        throw std::runtime_error(result.error_message != nullptr ? result.error_message : fallback);
    }

    const raw_gcode_reader_service *m_service;
};

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
        read_impl(*run_context, *context, state);
        throw_if_cancelled(run_context);

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
