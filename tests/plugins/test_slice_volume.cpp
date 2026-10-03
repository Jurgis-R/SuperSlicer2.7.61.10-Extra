#include <catch2/catch.hpp>

#include <algorithm>
#include <string>
#include <vector>

#include "plugin_test_helpers.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/Api/plugin/c/steps/slic3r_step_slicing.h"
#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/Config/PrintConfig.hpp"

namespace {

using namespace Slic3r;

} // namespace

TEST_CASE("Slice volume plugin owns the XY compensation settings",
          "[plugins][slice-volume][config]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();
    Orchestrator &orchestrator = Orchestrator::instance();

    Plugin *plugin = orchestrator.get_plugin("slice_volume");
    REQUIRE(plugin != nullptr);

    std::vector<std::string> defined_keys;
    for (const Plugin::DefinedConfigKey &key : plugin->get_defined_config_keys())
        defined_keys.push_back(key.key);
    CHECK(defined_keys.size() == 3);
    for (const char *expected : { "hole_size_compensation", "xy_size_compensation", "xy_inner_size_compensation" })
        CHECK(std::find(defined_keys.begin(), defined_keys.end(), expected) != defined_keys.end());

    for (const char *key : { "hole_size_compensation", "xy_size_compensation", "xy_inner_size_compensation" }) {
        const ConfigOptionDef *def = PrintConfigDef::instance().get(key);
        REQUIRE(def != nullptr);
        CHECK(def->type == coFloat);
        CHECK(def->container_type == ConfigOptionContainerType::Object);
        CHECK(def->option_preset_type == RAW_PRESET_TYPE_FFF_PRINT);
        CHECK(def->invalidates_step == STEP_SLICING);
        CHECK(def->sidetext == "mm");
        CHECK(def->default_value->serialize() == "0");
        CHECK(PrintObjectConfig::defaults().option(key) != nullptr);
        CHECK(PrintConfig::defaults().option(key) == nullptr);
    }

    // The compensation settings stay always editable: the plugin owns no rule.
    for (const Orchestrator::PluginGuiRule &rule : orchestrator.gui_rules().rules())
        CHECK(rule.owner != "slice_volume");
}
