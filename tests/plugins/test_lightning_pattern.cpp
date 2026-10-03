#include <catch2/catch.hpp>

#include <tuple>

#include "perimeter_test_helpers.hpp"
#include "libslic3r/EdgeGrid.hpp"
#include "libslic3r/Fill/Lightning/DistanceField.hpp"
#include "libslic3r/Fill/Lightning/TreeNode.hpp"

using namespace Slic3r;
using namespace Slic3r::Test::PerimeterPluginTests;

namespace {
// Inspect the native field without changing its production interface.
class InspectableField : public FillLightning::DistanceField
{
public:
    using DistanceField::DistanceField;
    std::vector<std::pair<Point, coord_t>> cells() const
    {
        std::vector<std::pair<Point, coord_t>> out;
        for (const UnsupportedCell &cell : m_unsupported_points)
            out.emplace_back(cell.loc, cell.dist_to_boundary);
        return out;
    }
};
}

TEST_CASE("Lightning intersections choose the hit nearest the destination", "[plugins][lightning][core]")
{
    const Polygon outline = rectangle_expolygon(-5., -5., 5., 5.).contour;
    EdgeGrid::Grid grid;
    grid.set_bbox(BoundingBox(Point(-scale_i(10.), -scale_i(10.)), Point(scale_i(10.), scale_i(10.))));
    grid.create(outline, scale_i(4.));
    Point hit;
    REQUIRE(FillLightning::lineSegmentPolygonsIntersection(Point(-scale_i(7.), 0), Point(scale_i(7.), 0), grid, hit, scale_i(3.)));
    CHECK(hit == Point(scale_i(5.), 0));
    CHECK_FALSE(FillLightning::lineSegmentPolygonsIntersection(Point(-scale_i(7.), 0), Point(scale_i(7.), 0), grid, hit, scale_i(1.)));
    CHECK_FALSE(FillLightning::lineSegmentPolygonsIntersection(Point(-scale_i(2.), 0), Point(scale_i(2.), 0), grid, hit, scale_i(3.)));
}

TEST_CASE("Lightning unsupported points have a strict distance-band order", "[plugins][lightning][core]")
{
    const coord_t radius = scale_i(2.);
    const Polygons outline{rectangle_expolygon(-10., -10., 10., 10.).contour};
    const BoundingBox bounds(outline.front().points);
    const InspectableField field(radius, outline, bounds, outline);
    const std::vector<std::pair<Point, coord_t>> cells = field.cells();
    REQUIRE(cells.size() > 100);
    const auto key = [radius](const std::pair<Point, coord_t> &cell) {
        return std::make_tuple(cell.second / radius, PointHash{}(cell.first) % 191);
    };
    for (size_t i = 1; i < cells.size(); ++i)
        CHECK_FALSE(key(cells[i]) < key(cells[i - 1]));
    const InspectableField repeated(radius, outline, bounds, outline);
    CHECK(cells == repeated.cells());
}
