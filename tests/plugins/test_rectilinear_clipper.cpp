#include <catch2/catch.hpp>

#include <algorithm>
#include <utility>
#include <vector>

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Plugins/Infill/Rectilinear/RectilinearClipper.hpp"

#include "libslic3r/ExPolygon.hpp"

namespace rect_work = slic3r_api::Infill::RectilinearPatternsPlugin::work;
namespace rect_clipper = slic3r_api::Infill::RectilinearPatternsPlugin::clipper;

namespace {

slic3r_api::ExPolygon view(const Slic3r::ExPolygon &expolygon)
{
    return slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&expolygon));
}

Slic3r::ExPolygon make_boundary()
{
    Slic3r::ExPolygon out(Slic3r::Polygon({ Slic3r::Point(-10000000, -8000000), Slic3r::Point(12000000, -8000000),
                                             Slic3r::Point(12000000, 9000000), Slic3r::Point(-10000000, 9000000) }));
    out.holes.push_back(Slic3r::Polygon({ Slic3r::Point(-3000000, -2000000), Slic3r::Point(-3000000, 3000000),
                                          Slic3r::Point(4000000, 3000000), Slic3r::Point(4000000, -2000000) }));
    return out;
}

rect_work::Polygons as_polygons(const rect_work::ExPolygon &expolygon)
{
    rect_work::Polygons out;
    out.push_back(expolygon.contour);
    for (const rect_work::Polygon &hole : expolygon.holes)
        out.push_back(hole);
    return out;
}

using PolygonSignature = std::vector<std::pair<coord_t, coord_t>>;

std::vector<PolygonSignature> signatures(const rect_work::Polygons &polygons)
{
    std::vector<PolygonSignature> out;
    out.reserve(polygons.size());
    for (const rect_work::Polygon &polygon : polygons) {
        PolygonSignature signature;
        signature.reserve(polygon.points.size());
        for (const rect_work::Point &point : polygon.points)
            signature.emplace_back(point.x(), point.y());
        std::sort(signature.begin(), signature.end());
        out.push_back(std::move(signature));
    }
    std::sort(out.begin(), out.end());
    return out;
}

} // namespace

TEST_CASE("Rectilinear clipper offsets an API ExPolygon without source materialization",
          "[plugins][infill][rectilinear]")
{
    Slic3r::ExPolygon core_boundary = make_boundary();
    const slic3r_api::ExPolygon api_boundary = view(core_boundary);

    Slic3r::PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    const rect_work::ExPolygon value_boundary = rect_clipper::from_expolygon_view(api_boundary);
    const rect_work::Polygons value_source = as_polygons(value_boundary);

    const rect_work::Polygons direct_offset = rect_clipper::offset(storage, api_boundary, 250000.);
    const rect_work::Polygons value_offset = rect_clipper::offset(storage, value_source, 250000.);
    REQUIRE(signatures(direct_offset) == signatures(value_offset));

    const rect_work::Polygons direct_shrink = rect_clipper::shrink(storage, api_boundary, 250000.);
    const rect_work::Polygons value_shrink = rect_clipper::shrink(storage, value_source, 250000.);
    CHECK(signatures(direct_shrink) == signatures(value_shrink));
}
