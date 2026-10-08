/* PluginStorage tracks allocations created through the C API. The registry is
   shared, but each worker edits only its own objects. The stress test starts
   eight threads together, keeps handles alive while other threads allocate and
   free, and checks the final registry after joining. The execution test exercises
   the same sharing through four parallel STEP_SUPPORT plugin runs. These tests check
   exact counts and membership; exposing an unlocked race depends on scheduling. */
#include <catch2/catch.hpp>

#include <array>
#include <atomic>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#include "plugin_test_helpers.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/cpp/Views.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_support.h"
#include "libslic3r/ExPolygon.hpp"

namespace {
using namespace Slic3r;
std::atomic_uint completed_objects {0};

class StorageProbe final : public slic3r_api::PluginBase
{
public:
    // Connect the recording provider to the test orchestrator.
    explicit StorageProbe(orchestrator_handle *orch) : PluginBase(orch) {}
private:
    // Identify the provider for activation in this test only.
    const char *id_impl() const noexcept override { return "test.plugin_storage.support"; }
    // Exercise the object runner without adding support geometry.
    slicing_step_t step_impl() const noexcept override { return STEP_SUPPORT; }
    // Run this allocation probe at the default priority.
    int32_t priority_impl() const noexcept override { return 0; }
    // No other plugin is needed for the allocation probe.
    const char *const *dependencies_impl() const noexcept override { return nullptr; }
    // Check one object's payload and repeatedly clip temporary polygons in the shared storage; failures use the plugin error channel.
    void run_impl(const plugin_run_context *run_ctx) const override
    {
        const run_ctx_generate_support *payload = plugin_ctx_as_generate_support(run_ctx);
        if (payload == nullptr || payload->object == nullptr)
            throw std::runtime_error("Missing support object for storage probe");
        const ExPolygons areas {ExPolygon(Polygon({Point(0, 0), Point(scale_i(4.), 0),
                                                   Point(scale_i(4.), scale_i(4.)), Point(0, scale_i(4.))}))};
        const slic3r_api::ClipperContext clipper(run_ctx->plugin_storage);
        for (size_t idx = 0; idx < 256; ++idx) {
            const slic3r_api::StoredExPolygonCollection clipped = slic3r_api::clipper_offset(
                clipper(slic3r_api::ExPolygonCollection(reinterpret_cast<const expolygon_collection_handle *>(&areas))),
                scale_d(0.01)).to_expolygon_collection();
            if (clipped.size() != 1)
                throw std::runtime_error("Unexpected temporary clipping result");
        }
        completed_objects.fetch_add(1, std::memory_order_relaxed);
    }
};
} // namespace

namespace Slic3r::Test::Plugins {
// Register the allocation probe before option finalization; registration failure stops test initialization.
void register_plugin_storage_test_provider(orchestrator_handle *orch)
{
    static StorageProbe provider(orch);
    if (!reinterpret_cast<Orchestrator *>(orch)->register_plugin(provider.c_instance()))
        throw std::runtime_error("Cannot register plugin storage probe");
}
} // namespace Slic3r::Test::Plugins

TEST_CASE("Shared plugin storage registers and frees mixed allocations concurrently", "[plugins][plugin-storage]")
{
    constexpr size_t workers = 8;
    PluginStorage storage;
    storage_handle *handle = reinterpret_cast<storage_handle *>(&storage);
    void *anchor = storage_new_polygon(handle);
    void *config = storage_new_config(handle);
    REQUIRE(storage.contains(anchor));
    REQUIRE(storage.contains(config));
    std::array<void *, workers> retained {};
    std::array<std::exception_ptr, workers> failures {};
    std::vector<std::thread> threads;
    std::mutex start_mutex;
    std::condition_variable start_signal;
    size_t ready = 0;
    bool start = false;
    for (size_t worker = 0; worker < workers; ++worker)
        threads.emplace_back([&, worker] {
            {
                std::unique_lock<std::mutex> lock(start_mutex);
                ++ready;
                start_signal.notify_all();
                start_signal.wait(lock, [&] { return start; });
            }
            try {
                retained[worker] = storage_new_polygon(handle);
                for (size_t iteration = 0; iteration < 256; ++iteration) {
                    std::vector<void *> allocations {
                        storage_new_polygon(handle), storage_new_polyline(handle), storage_new_expolygon(handle),
                        storage_new_polygons(handle), storage_new_polylines(handle), storage_new_expolygons(handle),
                        storage_new_surface_collection(handle), storage_new_config(handle), extrusion_create_empty(handle)
                    };
                    const slic3r_api::ClipperContext clipper(handle);
                    slic3r_api::ClipperOperand shapes = clipper.empty();
                    for (void *allocation : allocations)
                        if (!storage.contains(allocation))
                            throw std::runtime_error("Allocation absent from shared registry");
                    if (!storage.contains(const_cast<clipper_shapes_handle *>(shapes.handle())) ||
                        storage.size() < allocations.size() + 1)
                        throw std::runtime_error("Clipper allocation absent from shared registry");
                    for (void *allocation : allocations)
                        if (!storage.free(allocation))
                            throw std::runtime_error("Cannot release shared registry allocation");
                }
            } catch (...) {
                failures[worker] = std::current_exception();
            }
        });
    {
        std::unique_lock<std::mutex> lock(start_mutex);
        start_signal.wait(lock, [&] { return ready == workers; });
        start = true;
    }
    start_signal.notify_all();
    for (std::thread &thread : threads)
        thread.join();
    for (const std::exception_ptr &failure : failures)
        if (failure)
            std::rethrow_exception(failure);
    CHECK(storage.size() == workers + 2);
    CHECK(storage.contains(anchor));
    CHECK(storage.contains(config));
    for (void *allocation : retained)
        CHECK(storage.contains(allocation));
    storage.clear();
    CHECK(storage.size() == 0);
    CHECK_FALSE(storage.contains(anchor));
    CHECK_FALSE(storage.contains(config));
    CHECK_FALSE(storage.contains(nullptr));
}

TEST_CASE("Four parallel plugin runs share storage without retaining clipping temporaries", "[plugins][plugin-storage]")
{
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();
    struct Deactivate {
        // Leave the probe inactive even when an assertion fails.
        ~Deactivate() { Orchestrator::instance().set_plugin_active("test.plugin_storage.support", false); }
    } deactivate;
    REQUIRE(orchestrator.set_plugin_active("test.plugin_storage.support", true));
    completed_objects = 0;
    Plugin *plugin = orchestrator.get_plugin("test.plugin_storage.support");
    REQUIRE(plugin != nullptr);
    PluginStorage &storage = orchestrator.plugin_storage()[plugin];
    const size_t initial_count = storage.size();
    constexpr size_t run_count = 4;
    std::array<run_ctx_generate_support, run_count> support_payloads {};
    std::array<raw_plugin_run_payload, run_count> run_payloads {};
    for (size_t idx = 0; idx < run_count; ++idx) {
        // The storage probe only needs a distinct, non-null object identity;
        // the host runner carries the payload address to each parallel call.
        support_payloads[idx].object = reinterpret_cast<const object_handle *>(&support_payloads[idx]);
        run_payloads[idx].data = &support_payloads[idx];
    }
    REQUIRE(orchestrator.execute_plugin(*plugin, nullptr, run_payloads.data(), run_count) ==
            RAW_PLUGIN_EXECUTION_SUCCESS);
    CHECK(completed_objects.load() == 4);
    CHECK(storage.size() == initial_count);
}
