///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Native-thread lifetime probe for the Python seam placer
=======================================================

ctypes temporarily attaches a Python thread state when a native thread enters
a Python callback, then detaches it as soon as the callback returns. This test
fixture keeps that native thread alive and lets Python inspect the returned C
diagnostic before releasing it. It therefore verifies the lifetime promised by
the seam placer ABI rather than the longer-lived state of a Python thread pool.
*/

#include "src/libslic3r/Api/plugin/c/steps/slic3r_step_seam_placer.h"

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

// Own the worker until the caller has inspected the callback's borrowed result.
struct NativeSeamProbe {
    std::mutex mutex;
    std::condition_variable condition;
    raw_seam_placer_result result{};
    bool callback_finished{false};
    bool release_worker{false};
    std::thread worker;
};

} // namespace

extern "C" {

// Start one native callback and return only after its temporary Python state is gone.
SLIC3R_TEST_EXPORT void *python_seam_placer_native_probe_create(
    const raw_seam_placer_instance *instance,
    const extrusion_entity_handle *loop,
    const layer_region_handle *layer_region,
    c_point start_position);

// Return the status written by the completed callback.
SLIC3R_TEST_EXPORT int32_t python_seam_placer_native_probe_status(void *probe);

// Copy the borrowed diagnostic while the originating native thread remains alive.
SLIC3R_TEST_EXPORT uint32_t python_seam_placer_native_probe_copy_message(
    void *probe, char *output, uint32_t capacity);

// Release and join the worker, then destroy the probe.
SLIC3R_TEST_EXPORT void python_seam_placer_native_probe_destroy(void *probe);

SLIC3R_TEST_EXPORT void *python_seam_placer_native_probe_create(
    const raw_seam_placer_instance *instance,
    const extrusion_entity_handle *loop,
    const layer_region_handle *layer_region,
    c_point start_position)
{
    if (instance == nullptr || instance->session == nullptr || instance->vtable == nullptr ||
        instance->vtable->place_seam == nullptr)
        return nullptr;

    std::unique_ptr<NativeSeamProbe> probe = std::make_unique<NativeSeamProbe>();
    NativeSeamProbe *raw_probe = probe.get();
    const void *session = instance->session;
    const seam_placer_place_fn place_seam = instance->vtable->place_seam;
    raw_probe->worker = std::thread([raw_probe, session, place_seam, loop, layer_region, start_position]() {
        raw_probe->result.struct_size = sizeof(raw_seam_placer_result);
        place_seam(session, loop, layer_region, start_position, &raw_probe->result);

        std::unique_lock<std::mutex> lock(raw_probe->mutex);
        raw_probe->callback_finished = true;
        raw_probe->condition.notify_all();
        raw_probe->condition.wait(lock, [raw_probe]() { return raw_probe->release_worker; });
    });

    std::unique_lock<std::mutex> lock(raw_probe->mutex);
    raw_probe->condition.wait(lock, [raw_probe]() { return raw_probe->callback_finished; });
    return probe.release();
}

SLIC3R_TEST_EXPORT int32_t python_seam_placer_native_probe_status(void *probe)
{
    if (probe == nullptr)
        return RAW_SEAM_PLACER_STATUS_INVALID_ARGUMENT;
    return static_cast<int32_t>(static_cast<NativeSeamProbe *>(probe)->result.status);
}

SLIC3R_TEST_EXPORT uint32_t python_seam_placer_native_probe_copy_message(
    void *probe, char *output, uint32_t capacity)
{
    if (probe == nullptr)
        return 0;

    const char *message = static_cast<NativeSeamProbe *>(probe)->result.error_message;
    if (message == nullptr)
        return 0;

    const size_t length = std::strlen(message);
    if (output != nullptr && capacity > 0) {
        const size_t copied = std::min(length, static_cast<size_t>(capacity - 1));
        std::memcpy(output, message, copied);
        output[copied] = '\0';
    }
    return static_cast<uint32_t>(length);
}

SLIC3R_TEST_EXPORT void python_seam_placer_native_probe_destroy(void *probe)
{
    std::unique_ptr<NativeSeamProbe> owned(static_cast<NativeSeamProbe *>(probe));
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
