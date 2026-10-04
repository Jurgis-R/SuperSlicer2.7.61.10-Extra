#include <catch2/catch.hpp>

#include <chrono>
#include <future>
#include <iostream>
#include <random>
#include "libslic3r/Api/plugin/cpp/LineDistancer.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Polygon.hpp"

namespace {
using namespace Slic3r;

Point point_mm(const double x, const double y)
{
    return Point(scale_i(x), scale_i(y));
}

Polygon rectangle_mm(const double min_x, const double min_y, const double max_x, const double max_y)
{
    Polygon polygon({
        point_mm(min_x, min_y),
        point_mm(max_x, min_y),
        point_mm(max_x, max_y),
        point_mm(min_x, max_y)
    });
    polygon.make_counter_clockwise();
    return polygon;
}

ExPolygon rectangle_with_two_holes()
{
    ExPolygon expolygon(rectangle_mm(0., 0., 20., 20.));
    expolygon.holes.push_back(rectangle_mm(2., 2., 5., 5.));
    expolygon.holes.push_back(rectangle_mm(8., 8., 11., 11.));
    for (Polygon &hole : expolygon.holes)
        hole.make_clockwise();
    return expolygon;
}

slic3r_api::Polygon view(const Polygon &polygon)
{
    return slic3r_api::Polygon(reinterpret_cast<const polygon_handle *>(&polygon));
}

slic3r_api::ExPolygon view(const ExPolygon &expolygon)
{
    return slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&expolygon));
}

void check_bbox(const c_bounding_box &bbox,
                const double min_x,
                const double min_y,
                const double max_x,
                const double max_y)
{
    CHECK(bbox.min.x == scale_i(min_x));
    CHECK(bbox.min.y == scale_i(min_y));
    CHECK(bbox.max.x == scale_i(max_x));
    CHECK(bbox.max.y == scale_i(max_y));
}

} // namespace

TEST_CASE("ClipperOperand path_count counts raw non-empty paths", "[plugins][clipper]")
{
    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    slic3r_api::ClipperContext clipper(storage);

    slic3r_api::ClipperOperand empty = clipper.empty();
    CHECK(empty.path_count() == 0);
    CHECK(empty.empty());

    Polygon polygon = rectangle_mm(0., 0., 10., 10.);
    slic3r_api::ClipperOperand polygon_operand = clipper(view(polygon));
    CHECK(polygon_operand.path_count() == 1);
    CHECK(!polygon_operand.empty());

    ExPolygon expolygon = rectangle_with_two_holes();
    slic3r_api::ClipperOperand expolygon_operand = clipper(view(expolygon));
    CHECK(expolygon_operand.path_count() == 3);

    Polygon other_polygon = rectangle_mm(30., 0., 40., 10.);
    slic3r_api::ClipperOperand concatenated = clipper.empty();
    concatenated += polygon_operand;
    CHECK(concatenated.path_count() == 1);
    concatenated += clipper(view(other_polygon));
    CHECK(concatenated.path_count() == 2);

    slic3r_api::ClipperOperand unioned = slic3r_api::clipper_union(concatenated);
    CHECK(unioned.path_count() == 2);
    CHECK(!unioned.empty());
}

TEST_CASE("ClipperOperand concatenation treats null operands as empty", "[plugins][clipper]")
{
    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    slic3r_api::ClipperContext clipper(storage);
    Polygon polygon = rectangle_mm(0., 0., 10., 10.);

    // clipper() is a null view; accumulating into it and appending it are no-ops.
    slic3r_api::ClipperOperand accumulated = clipper();
    accumulated += clipper();
    CHECK(accumulated.empty());
    accumulated += clipper(view(polygon));
    CHECK(accumulated.path_count() == 1);
    accumulated += clipper();
    CHECK(accumulated.path_count() == 1);
    accumulated += clipper.empty();
    CHECK(accumulated.path_count() == 1);

    slic3r_api::ClipperOperand null_operand = clipper();
    CHECK((null_operand + accumulated).path_count() == 1);
    CHECK((accumulated + null_operand).path_count() == 1);
    CHECK((null_operand + clipper()).empty());

    // Null views can be moved like owned ones.
    slic3r_api::ClipperOperand moved(std::move(null_operand));
    CHECK(moved.empty());
    moved = clipper();
    CHECK(moved.empty());

    // The C call keeps the first handle when there is nothing to append.
    clipper_shapes_handle *raw = ::clipper_shapes_from_polygon(storage, view(polygon).handle());
    raw = ::clipper_concat_replace(storage, raw, nullptr);
    REQUIRE(raw != nullptr);
    CHECK(::clipper_shapes_path_count(raw) == 1);
    CHECK(storage_free(storage, raw));
}

TEST_CASE("ClipperOperand bounding_box reads raw Clipper points", "[plugins][clipper]")
{
    PluginStorage plugin_storage;
    storage_handle *storage = reinterpret_cast<storage_handle *>(&plugin_storage);
    slic3r_api::ClipperContext clipper(storage);

    CHECK(clipper.empty().bounding_box().min.x == 0);
    CHECK(clipper.empty().bounding_box().min.y == 0);
    CHECK(clipper.empty().bounding_box().max.x == 0);
    CHECK(clipper.empty().bounding_box().max.y == 0);

    Polygon polygon = rectangle_mm(-2., 3., 10., 12.);
    check_bbox(clipper(view(polygon)).bounding_box(), -2., 3., 10., 12.);

    ExPolygon expolygon = rectangle_with_two_holes();
    check_bbox(clipper(view(expolygon)).bounding_box(), 0., 0., 20., 20.);

    Polygon other_polygon = rectangle_mm(30., -5., 40., 6.);
    slic3r_api::ClipperOperand concatenated = clipper(view(polygon));
    concatenated += clipper(view(other_polygon));
    check_bbox(concatenated.bounding_box(), -2., -5., 40., 12.);

    slic3r_api::ClipperOperand unioned = slic3r_api::clipper_union(concatenated);
    check_bbox(unioned.bounding_box(), -2., -5., 40., 12.);
}


TEST_CASE("Public LineDistancer index matches exhaustive queries and owns its data", "[plugins][line-distancer]")
{
    slic3r_api::LineDistancer reference;
    std::mt19937 random(42);
    std::uniform_int_distribution<int> coordinate(-10000,10000);
    for (size_t i=0;i<1200;++i) {
        const c_point a{coordinate(random),coordinate(random)};
        const c_point b=i%7==0?a:c_point{a.x+coordinate(random)/30,a.y+coordinate(random)/30};
        reference.append_segment(a,b);
    }
    slic3r_api::LineDistancer indexed=reference;
    indexed.build_index();
    for (int i=0;i<120;++i) {
        const c_point p{coordinate(random),coordinate(random)};
        CHECK(indexed.distance_from_lines(p)==Approx(reference.distance_from_lines(p)));
        for (double radius:{-1.,0.,1.,500.,50000.})
            CHECK(indexed.all_lines_in_radius(p,radius)==reference.all_lines_in_radius(p,radius));
        const c_point min{p.x-200,p.y-300},max{p.x+200,p.y+300};
        CHECK(indexed.all_lines_in_box(min,max)==reference.all_lines_in_box(min,max));
        CHECK(indexed.all_lines_in_box(max,min).empty());
    }
    // Invalidation must not expose the stale tree or change the original copy.
    const double original=indexed.distance_from_lines({50000,50000});
    slic3r_api::LineDistancer copy=indexed;
    copy.append_segment({50000,50000},{50000,50000});
    CHECK(copy.distance_from_lines({50000,50000})==0.);
    CHECK(indexed.distance_from_lines({50000,50000})==original);
    copy.build_index();
    slic3r_api::LineDistancer moved=std::move(copy);
    copy=slic3r_api::LineDistancer();
    CHECK(moved.distance_from_lines({50000,50000})==0.);
    CHECK(moved.all_lines_in_radius({50000,50000},0.)==std::vector<size_t>{1200});
    std::vector<std::future<double>> workers;
    for (int i=0;i<8;++i)
        workers.push_back(std::async(std::launch::async,[&indexed] {
            double sum=0;
            for (int j=0;j<100;++j) sum+=indexed.distance_from_lines({j*47,j*31});
            return sum;
        }));
    const double expected=workers.front().get();
    for (size_t i=1;i<workers.size();++i) CHECK(workers[i].get()==expected);

    slic3r_api::LineDistancer empty;
    empty.build_index();
    CHECK(std::isinf(empty.distance_from_lines({0,0},true)));
    CHECK(empty.all_lines_in_box({-1,-1},{1,1}).empty());
    empty.append_segment({0,0},{10,0});
    empty.build_index();
    CHECK(empty.all_lines_in_box({10,0},{10,0})==std::vector<size_t>{0});
    CHECK(empty.distance_from_lines({20,0})==10.); // finite segment endpoint
    CHECK(empty.distance_from_lines({5,0},true)==0.);
}

TEST_CASE("Public LineDistancer signed queries preserve area union and holes", "[plugins][line-distancer]")
{
    PluginStorage storage;
    slic3r_api::StoredExPolygonCollection areas(reinterpret_cast<storage_handle *>(&storage));
    const ExPolygon first=rectangle_with_two_holes();
    const ExPolygon second(rectangle_mm(9.,9.,15.,15.));
    areas.push_back(view(first)); areas.push_back(view(second));
    slic3r_api::LineDistancer indexed(areas.readonly());
    // An added segment does not add an interior; it also forces exhaustive queries.
    slic3r_api::LineDistancer exhaustive=indexed;
    exhaustive.append_segment({scale_i(100),scale_i(100)},{scale_i(100),scale_i(100)});
    for (double x:{-1.,0.,1.,2.,3.,5.,8.,9.,10.,11.,15.,20.,21.}) {
        for (double y:{-1.,0.,1.,3.,5.,9.,10.,11.,15.,20.,21.}) {
            const c_point p{scale_i(x),scale_i(y)};
            CHECK(indexed.distance_from_lines(p,true)==Approx(exhaustive.distance_from_lines(p,true)));
            const double distance=indexed.distance_from_lines(p,true);
            if (distance!=0.)
                CHECK((distance<0.)==(first.contains(Point(p.x,p.y)) || second.contains(Point(p.x,p.y))));
        }
    }
    CHECK(indexed.distance_from_lines({scale_i(3),scale_i(3)},true)>0.);
    CHECK(indexed.distance_from_lines({scale_i(10),scale_i(10)},true)<0.); // second area fills a hole
    CHECK(indexed.distance_from_lines({scale_i(1),scale_i(1)},true)<0.);
    // Closing the stored owners cannot invalidate the copied boundaries.
    areas.clear();
    CHECK(indexed.distance_from_lines({scale_i(1),scale_i(1)},true)<0.);
}

// Opt-in measurement, never a timing assertion. Include index preparation and
// identical query workloads; the checksum prevents dead-code elimination.
TEST_CASE("Public LineDistancer dense boundary benchmark", "[.][plugins][line-distancer-benchmark]")
{
    for (int count:{1000,10000,40000}) {
        slic3r_api::LineDistancer reference;
        for (int i=0;i<count;++i) {
            const coord_t x=(i%200)*1000,y=(i/200)*1000;
            reference.append_segment({x,y},{x+100,y+100});
        }
        slic3r_api::LineDistancer indexed=reference;
        const std::chrono::steady_clock::time_point start=std::chrono::steady_clock::now();
        indexed.build_index();
        const std::chrono::steady_clock::time_point built=std::chrono::steady_clock::now();
        double fast=0,slow=0;
        for (int i=0;i<1000;++i) fast+=indexed.distance_from_lines({(i%200)*1000+50,(i/200)*1000+200});
        const std::chrono::steady_clock::time_point queried=std::chrono::steady_clock::now();
        for (int i=0;i<1000;++i) slow+=reference.distance_from_lines({(i%200)*1000+50,(i/200)*1000+200});
        const std::chrono::steady_clock::time_point done=std::chrono::steady_clock::now();
        CHECK(fast==Approx(slow));
        std::cout << "LineDistancer edges=" << count
                  << " build_ms=" << std::chrono::duration<double,std::milli>(built-start).count()
                  << " indexed_ms=" << std::chrono::duration<double,std::milli>(queried-built).count()
                  << " exhaustive_ms=" << std::chrono::duration<double,std::milli>(done-queried).count() << '\n';
    }
}
