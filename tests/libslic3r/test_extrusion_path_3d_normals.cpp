#include "libslic3r/ExtrusionEntity.hpp"

#include <catch2/catch.hpp>

using namespace Slic3r;

TEST_CASE("3D extrusion path preserves optional tool-axis directions", "[ExtrusionPath3D][FiveAxis]")
{
    ExtrusionPath3D path(ExtrusionAttributes{ExtrusionRole::None}, true);
    path.push_back(Point(0, 0), 0);
    path.push_back(Point(1000, 0), 250);

    REQUIRE(path.tool_axis_normals.empty());
    CHECK_FALSE(path.has_tool_axis_normals());

    path.tool_axis_normals = { { { 0.0, 0.0, 1.0 } }, { { 0.0, 1.0, 0.0 } } };
    REQUIRE(path.has_tool_axis_normals());

    ExtrusionPath3D copied(path);
    REQUIRE(copied.has_tool_axis_normals());
    CHECK(copied.tool_axis_normals[0][2] == Approx(1.0));
    CHECK(copied.tool_axis_normals[1][1] == Approx(1.0));

    copied.reverse();
    REQUIRE(copied.has_tool_axis_normals());
    CHECK(copied.tool_axis_normals[0][1] == Approx(1.0));
    CHECK(copied.tool_axis_normals[1][2] == Approx(1.0));

    copied.tool_axis_normals.pop_back();
    CHECK_FALSE(copied.has_tool_axis_normals());
}
