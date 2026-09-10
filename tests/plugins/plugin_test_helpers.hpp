#ifndef test_plugins_plugin_test_helpers_hpp_
#define test_plugins_plugin_test_helpers_hpp_

struct orchestrator_handle;

namespace Slic3r::Test::Plugins {

/* Implemented by the regional SeamPlacer test fixture and called before config finalization. */
void register_regional_seam_test_provider(orchestrator_handle *orchestrator);
void ensure_plugin_test_runtime_initialized();
bool python_plugin_test_runtime_available();

} // namespace Slic3r::Test::Plugins

#endif // test_plugins_plugin_test_helpers_hpp_
