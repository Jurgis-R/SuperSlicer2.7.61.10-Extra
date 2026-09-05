///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Native-thread lifetime probe for Python G-code firmware
=======================================================

The probe invokes end_print from a native worker, waits until ctypes has
detached that worker's temporary Python thread state, and keeps the worker
alive while the caller copies the borrowed output or diagnostic. This directly
checks the lifetime promised by raw_gcode_firmware_result.
*/

#include "src/libslic3r/Api/plugin/c/slic3r_gcode_firmware.h"

#include <algorithm>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <thread>

#if defined(_WIN32)
#define SLIC3R_TEST_EXPORT __declspec(dllexport)
#else
#define SLIC3R_TEST_EXPORT __attribute__((visibility("default")))
#endif

namespace {

// Keep the callback result and native worker alive until explicit destruction.
struct NativeFirmwareProbe {
    std::mutex mutex;
    std::condition_variable condition;
    raw_gcode_firmware_result result{};
    bool callback_finished{false};
    bool release_worker{false};
    std::thread worker;
};

// Copy one borrowed byte range without reading beyond its declared size.
uint32_t copy_bytes(const char *source,
                    size_t source_size,
                    char *output,
                    uint32_t capacity);

} // namespace

extern "C" {

// Invoke end_print and return after its temporary Python thread state is gone.
SLIC3R_TEST_EXPORT void *python_gcode_firmware_native_probe_create(
    const raw_gcode_firmware_instance *instance);

// Return the callback status captured by the probe.
SLIC3R_TEST_EXPORT int32_t python_gcode_firmware_native_probe_status(void *probe);

// Copy the exact output bytes, including embedded NUL characters.
SLIC3R_TEST_EXPORT uint32_t python_gcode_firmware_native_probe_copy_text(
    void *probe, char *output, uint32_t capacity);

// Copy the NUL-terminated diagnostic retained by the session.
SLIC3R_TEST_EXPORT uint32_t python_gcode_firmware_native_probe_copy_error(
    void *probe, char *output, uint32_t capacity);

// Release the worker and all probe-owned synchronization state.
SLIC3R_TEST_EXPORT void python_gcode_firmware_native_probe_destroy(void *probe);

SLIC3R_TEST_EXPORT void *python_gcode_firmware_native_probe_create(
    const raw_gcode_firmware_instance *instance)
{
    if (instance == nullptr || instance->session == nullptr || instance->vtable == nullptr ||
        instance->vtable->end_print == nullptr)
        return nullptr;

    std::unique_ptr<NativeFirmwareProbe> probe = std::make_unique<NativeFirmwareProbe>();
    NativeFirmwareProbe *raw_probe = probe.get();
    void *session = instance->session;
    const gcode_firmware_end_scope_fn end_print = instance->vtable->end_print;
    raw_probe->worker = std::thread([raw_probe, session, end_print]() {
        raw_probe->result.struct_size = sizeof(raw_gcode_firmware_result);
        end_print(session, &raw_probe->result);

        std::unique_lock<std::mutex> lock(raw_probe->mutex);
        raw_probe->callback_finished = true;
        raw_probe->condition.notify_all();
        raw_probe->condition.wait(lock, [raw_probe]() { return raw_probe->release_worker; });
    });

    std::unique_lock<std::mutex> lock(raw_probe->mutex);
    raw_probe->condition.wait(lock, [raw_probe]() { return raw_probe->callback_finished; });
    return probe.release();
}

SLIC3R_TEST_EXPORT int32_t python_gcode_firmware_native_probe_status(void *probe)
{
    if (probe == nullptr)
        return RAW_GCODE_FIRMWARE_STATUS_INVALID_ARGUMENT;
    return static_cast<int32_t>(static_cast<NativeFirmwareProbe *>(probe)->result.status);
}

SLIC3R_TEST_EXPORT uint32_t python_gcode_firmware_native_probe_copy_text(
    void *probe, char *output, uint32_t capacity)
{
    if (probe == nullptr)
        return 0;
    const raw_gcode_firmware_result &result = static_cast<NativeFirmwareProbe *>(probe)->result;
    return copy_bytes(result.text, static_cast<size_t>(result.text_size), output, capacity);
}

SLIC3R_TEST_EXPORT uint32_t python_gcode_firmware_native_probe_copy_error(
    void *probe, char *output, uint32_t capacity)
{
    if (probe == nullptr)
        return 0;
    const char *message = static_cast<NativeFirmwareProbe *>(probe)->result.error_message;
    return copy_bytes(message, message != nullptr ? std::strlen(message) : 0, output, capacity);
}

SLIC3R_TEST_EXPORT void python_gcode_firmware_native_probe_destroy(void *probe)
{
    std::unique_ptr<NativeFirmwareProbe> owned(static_cast<NativeFirmwareProbe *>(probe));
    if (owned == nullptr)
        return;
    {
        std::lock_guard<std::mutex> lock(owned->mutex);
        owned->release_worker = true;
    }
    owned->condition.notify_all();
    owned->worker.join();
}

} // extern "C"

namespace {

uint32_t copy_bytes(const char *source,
                    size_t source_size,
                    char *output,
                    uint32_t capacity)
{
    if (source == nullptr)
        return 0;
    if (output != nullptr && capacity > 0) {
        const size_t copied = std::min(source_size, static_cast<size_t>(capacity - 1));
        std::memcpy(output, source, copied);
        output[copied] = '\0';
    }
    return static_cast<uint32_t>(source_size);
}

} // namespace
