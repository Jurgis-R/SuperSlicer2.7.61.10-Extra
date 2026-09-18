#include <catch2/catch.hpp>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_bridge_detector.h"
#include "libslic3r/Plugins/BridgeDetector.hpp"

namespace {

int cancellation_requested(plugin_host_context *)
{
    return 1;
}

} // namespace

TEST_CASE("Bridge detector contains cancellation at its C ABI callbacks",
          "[plugins][bridge-detector][cancellation]")
{
    slic3r_api::BridgeDetectorPlugin::BridgeDetector detector(nullptr);
    const plugin_instance plugin = detector.c_instance();

    run_ctx_bridge_detector payload = {};
    plugin_run_context run_context = {};
    run_context.step = BRIDGE_DETECTOR;
    run_context.data = &payload;
    run_context.is_cancelled = &cancellation_requested;

    CHECK_NOTHROW(plugin.vt->setup(plugin.ctx, &run_context, 1));
    CHECK_NOTHROW(plugin.vt->setup_run(plugin.ctx, &run_context));
    CHECK_NOTHROW(plugin.vt->run(plugin.ctx, &run_context));
    CHECK(payload.detector.ctx == nullptr);
    CHECK(payload.detector.vt == nullptr);
}
