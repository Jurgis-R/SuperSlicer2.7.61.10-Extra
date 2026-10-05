#include <catch2/catch.hpp>

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <utility>
#include <vector>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/cpp/GeometryViews.hpp"

/*
Extrusion view helpers
======================

These tests cover the mutable polyline API on extrusion entities, in particular
set_polyline() which transfers points from another API polyline view without
building a temporary std::vector<c_point>.
*/

using namespace Slic3r;

namespace {

c_point point_mm(const double x, const double y)
{
    return c_point{scale_i(x), scale_i(y)};
}

slic3r_api::StoredPolyline polyline_mm(storage_handle *storage,
                                       std::initializer_list<std::pair<double, double>> points)
{
    slic3r_api::StoredPolyline polyline(storage);
    for (const std::pair<double, double> &point : points)
        polyline.push_back(point_mm(point.first, point.second));
    return polyline;
}

std::vector<c_point> points_mm(std::initializer_list<std::pair<double, double>> points)
{
    std::vector<c_point> out;
    out.reserve(points.size());
    for (const std::pair<double, double> &point : points)
        out.push_back(point_mm(point.first, point.second));
    return out;
}

bool same_points(const std::vector<c_point> &lhs, const std::vector<c_point> &rhs)
{
    if (lhs.size() != rhs.size())
        return false;
    for (size_t idx = 0; idx < lhs.size(); ++idx)
        if (lhs[idx].x != rhs[idx].x || lhs[idx].y != rhs[idx].y)
            return false;
    return true;
}

} // namespace

TEST_CASE("set_polyline transfers points from an API polyline view", "[plugins][extrusion][views]")
{
    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);

    SECTION("a stored polyline is copied without changing the source") {
        const slic3r_api::StoredPolyline source =
            polyline_mm(storage, { { 0.0, 0.0 }, { 10.0, 5.0 }, { 20.0, 0.0 } });
        slic3r_api::StoredExtrusionEntity entity(storage);

        REQUIRE(entity.set_polyline(source));

        CHECK(same_points(entity.points(), points_mm({ { 0.0, 0.0 }, { 10.0, 5.0 }, { 20.0, 0.0 } })));
        CHECK(same_points(source.points(), points_mm({ { 0.0, 0.0 }, { 10.0, 5.0 }, { 20.0, 0.0 } })));
    }

    SECTION("a collection element can be transferred directly") {
        slic3r_api::StoredPolylineCollection collection(storage);
        collection.push_back_move(polyline_mm(storage, { { 1.0, 2.0 }, { 3.0, 4.0 } }));

        slic3r_api::StoredExtrusionEntity entity(storage);
        REQUIRE(entity.set_polyline(collection.at(0)));

        CHECK(same_points(entity.points(), points_mm({ { 1.0, 2.0 }, { 3.0, 4.0 } })));
        CHECK(same_points(collection.at(0).points(), points_mm({ { 1.0, 2.0 }, { 3.0, 4.0 } })));
    }

    SECTION("an empty polyline clears the local polyline") {
        slic3r_api::StoredExtrusionEntity entity(storage, points_mm({ { 1.0, 1.0 }, { 2.0, 2.0 } }));
        REQUIRE(entity.point_count() == 2);

        const slic3r_api::StoredPolyline empty(storage);
        REQUIRE(entity.set_polyline(empty));

        CHECK(entity.point_count() == 0);
    }

    SECTION("an entity with children rejects the replacement") {
        slic3r_api::StoredExtrusionEntity entity(storage);
        slic3r_api::StoredExtrusionEntity child(storage);
        CHECK(entity.append_child_move(child.mutable_view()) != EXTRUSION_INDEX_INVALID);
        REQUIRE(entity.child_count() == 1);

        const slic3r_api::StoredPolyline replacement = polyline_mm(storage, { { 5.0, 5.0 }, { 6.0, 6.0 } });
        CHECK_FALSE(entity.set_polyline(replacement));

        // The rejected call must not change the entity content.
        CHECK(entity.point_count() == 0);
        CHECK(entity.child_count() == 1);
    }
}
