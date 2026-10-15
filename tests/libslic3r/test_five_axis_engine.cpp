#include "libslic3r/FiveAxisEngine.hpp"

#include <catch2/catch.hpp>
#include <limits>

using namespace Slic3r;

static std::vector<FiveAxisCommand> make_valid_five_axis_path()
{
    return {
        { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 600.0 },
        { 1.0, 0.0, 1.0, 0.1, 1.0, 0.5, 600.0 },
        { 2.0, 0.0, 1.0, 0.2, 2.0, 1.0, 600.0 }
    };
}

TEST_CASE("Five-axis output validates and formats configured rotary words",
          "[FiveAxisEngine]")
{
    FiveAxisOutputConfig config;
    config.rotary_1_word = 'B';
    config.rotary_2_word = 'C';
    std::string output;
    std::string error;

    REQUIRE(emit_five_axis_linear_gcode(config, make_valid_five_axis_path(), output, &error));
    REQUIRE(error.empty());
    REQUIRE(output.find("B1.000 C0.500") != std::string::npos);
    REQUIRE(output.find("G1 X0.000 Y0.000 Z1.000 B0.000 C0.000 E0.000 F600.000") != std::string::npos);
    REQUIRE(output.back() == '\n');
}

TEST_CASE("Five-axis output rejects invalid axis words and rotary limits",
          "[FiveAxisEngine]")
{
    FiveAxisOutputConfig config;
    config.rotary_1_word = 'X';
    REQUIRE_FALSE(validate_five_axis_output_config(config).valid);

    config = FiveAxisOutputConfig{};
    config.rotary_2_word = 'A';
    REQUIRE_FALSE(validate_five_axis_output_config(config).valid);

    config = FiveAxisOutputConfig{};
    auto path = make_valid_five_axis_path();
    path.back().rotary_1_deg = 181.0;
    REQUIRE_FALSE(validate_five_axis_path(config, path).valid);
}

TEST_CASE("Five-axis output rejects excessive rotary delta and angular rate",
          "[FiveAxisEngine]")
{
    FiveAxisOutputConfig config;
    auto path = make_valid_five_axis_path();

    path[1].rotary_1_deg = 30.0;
    REQUIRE_FALSE(validate_five_axis_path(config, path).valid);

    path = make_valid_five_axis_path();
    path[1].rotary_1_deg = 5.0;
    path[1].feed_mm_min = 6000.0;
    path[0].feed_mm_min = 6000.0;
    config.max_rotary_delta_deg_per_mm = 100.0;
    REQUIRE_FALSE(validate_five_axis_path(config, path).valid);
}

TEST_CASE("Five-axis output fails closed without partial G-code",
          "[FiveAxisEngine]")
{
    FiveAxisOutputConfig config;
    auto path = make_valid_five_axis_path();
    path[1].z = std::numeric_limits<double>::quiet_NaN();

    std::string output = "stale output";
    std::string error;
    REQUIRE_FALSE(emit_five_axis_linear_gcode(config, path, output, &error));
    REQUIRE(output.empty());
    REQUIRE_FALSE(error.empty());
}

TEST_CASE("Five-axis output can forbid simultaneous rotary motion",
          "[FiveAxisEngine]")
{
    FiveAxisOutputConfig config;
    config.supports_simultaneous_rotary = false;
    auto path = make_valid_five_axis_path();

    REQUIRE_FALSE(validate_five_axis_path(config, path).valid);

    path[1].rotary_2_deg = 0.0;
    path[2].rotary_2_deg = 0.0;
    REQUIRE(validate_five_axis_path(config, path).valid);
}
