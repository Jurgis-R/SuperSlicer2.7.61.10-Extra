// Exercise installed Python clear hooks through the native loader and host cleanup path.
#include <catch2/catch.hpp>

#include <string>
#include <vector>

#include "plugin_test_helpers.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Utils.hpp"

#ifdef SLIC3R_TEST_PYTHON_PLUGINS

TEST_CASE("Python clear results cross the native bridge and retain failed participants",
          "[plugins][python][clear]")
{
    REQUIRE(Slic3r::Test::Plugins::python_plugin_test_runtime_available());
    Slic3r::Orchestrator &orchestrator = Slic3r::Orchestrator::instance();
    Slic3r::Plugin *without_clear = orchestrator.get_plugin("python.external.package_root");
    REQUIRE(without_clear != nullptr);
    CHECK_FALSE(without_clear->has_clear());

    struct ClearCase {
        const char *name;
        bool success;
        std::string message;
    };
    std::string long_utf8;
    for (int index = 0; index < 300; ++index)
        long_utf8 += "\xc3\xa9";
    const std::string invalid_return = "must return an int32 status or None";
    const std::vector<ClearCase> cases = {
        {"success", true, ""},
        {"none", true, ""},
        {"error", false, "Python clear error: \xc3\xa9" "chec"},
        {"general", false, "Plugin reported an error without a diagnostic."},
        {"unknown_id", false, "Plugin reported an unknown diagnostic id."},
        {"long_utf8", false, long_utf8},
        {"exception", false, "Python clear exception: \xc3\xa9" "chec"},
        {"unprintable_exception", false, "Python plugin failed in clear()"},
        {"bool", false, invalid_return},
        {"float", false, invalid_return},
        {"str", false, invalid_return},
        {"bytes", false, invalid_return},
        {"list", false, invalid_return},
        {"tuple", false, invalid_return},
        {"negative", false, "status is outside the int32 range"},
        {"overflow", false, "status is outside the int32 range"},
        {"noncallable", false, "clear attribute is not callable"},
    };

    for (const ClearCase &test : cases) {
        DYNAMIC_SECTION(test.name) {
            Slic3r::Plugin *plugin = orchestrator.get_plugin(std::string("python.test.clear.") + test.name);
            REQUIRE(plugin != nullptr);
            REQUIRE(plugin->has_clear());
            REQUIRE_FALSE(orchestrator.is_plugin_active(plugin));
            const Slic3r::ScopeGuard deactivate([&]() { orchestrator.set_plugin_active(plugin, false); });
            REQUIRE(orchestrator.set_plugin_active(plugin, true));
            orchestrator.reset_plugin_cancel();
            orchestrator.consume_plugin_messages();

            Slic3r::Print print;
            REQUIRE(orchestrator.execute_plugin(*plugin, &print, nullptr, 0, true) ==
                    RAW_PLUGIN_EXECUTION_SUCCESS);
            REQUIRE(print.plugin_cleanup_participants().size() == 1);
            REQUIRE(orchestrator.consume_plugin_messages().empty());
            CHECK(orchestrator.clear_plugin_participants(print, STEP_POST_SLICING) == test.success);
            CHECK_FALSE(orchestrator.is_plugin_cancelled());
            CHECK_FALSE(print.canceled());

            const std::vector<Slic3r::Orchestrator::PluginMessage> messages = orchestrator.consume_plugin_messages();
            if (test.success) {
                CHECK(messages.empty());
                CHECK(print.plugin_cleanup_participants().empty());
            } else {
                REQUIRE(messages.size() == 1);
                CHECK(messages.front().plugin_id == plugin->get_id());
                CHECK(messages.front().step == STEP_POST_SLICING);
                CHECK(messages.front().level == Slic3r::Orchestrator::PluginMessageLevel::Error);
                CHECK(messages.front().message.find(test.message) != std::string::npos);
                if (test.message == long_utf8)
                    CHECK(messages.front().message == long_utf8);
                REQUIRE(print.plugin_cleanup_participants().size() == 1);
                CHECK(orchestrator.clear_plugin_participants(print, STEP_POST_SLICING));
                CHECK(print.plugin_cleanup_participants().empty());
                CHECK(orchestrator.consume_plugin_messages().empty());
            }
        }
    }
}

#endif
