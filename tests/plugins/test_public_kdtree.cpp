///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
// Intentionally include the SDK utility before any framework/native header.
// This translation unit must remain independent of Eigen and slicer geometry.
#include "libslic3r/Api/plugin/cpp/KDTreeIndirect.hpp"
#ifdef EIGEN_WORLD_VERSION
#error The public KD-tree must not include Eigen.
#endif
#include <catch2/catch.hpp>
#include <future>
struct PublicCoordinates
{
    const std::vector<std::array<float, 3>> *points;
    float operator()(size_t i, size_t d) const { return (*points)[i][d]; }
};
TEST_CASE("Public KD-tree matches exhaustive searches", "[plugins][mesh-queries][kdtree]") {
    std::vector<std::array<float, 3>> points;
    for (int i = 0; i < 200; ++i)
        points.push_back({float(i % 17), float(i % 11), float(i % 7)});
    slic3r_api::KDTreeIndirect<3, float, PublicCoordinates> tree(PublicCoordinates{&points}, points.size());
    for (const std::array<float, 3> q : {std::array<float, 3>{0, 0, 0}, {3, 4, 5}, {30, 30, 30}}) {
        for (float radius : {0.f, 1.f, 4.f, 100.f}) {
            std::vector<size_t> expected;
            for (size_t i = 0; i < points.size(); ++i) {
                double d = 0;
                for (size_t j = 0; j < 3; ++j)
                    d += double(q[j] - points[i][j]) * (q[j] - points[i][j]);
                if (d < double(radius) * radius)
                    expected.push_back(i);
            }
            std::vector<size_t> actual = slic3r_api::find_nearby_points(tree, q, radius);
            std::sort(actual.begin(), actual.end());
            CHECK(actual == expected);
        }
        const size_t closest = slic3r_api::find_closest_point(tree, q);
        double best = std::numeric_limits<double>::infinity(), found = 0;
        for (size_t j = 0; j < 3; ++j)
            found += double(q[j] - points[closest][j]) * (q[j] - points[closest][j]);
        for (const std::array<float, 3> &p : points) {
            double d = 0;
            for (size_t j = 0; j < 3; ++j)
                d += double(q[j] - p[j]) * (q[j] - p[j]);
            best = std::min(best, d);
        }
        CHECK(found == best);
    }
    // The traversal must use the farthest retained neighbor, not the closest,
    // as its pruning threshold when more than one result is requested.
    const std::array<float, 3> multi_query{3.1f, 4.2f, 5.3f};
    const std::array<size_t, 5> nearest = slic3r_api::find_closest_points<5>(tree, multi_query);
    std::vector<float> distances;
    for (const std::array<float, 3> &point : points) {
        float distance = 0;
        for (size_t d = 0; d < 3; ++d) distance += (point[d] - multi_query[d]) * (point[d] - multi_query[d]);
        distances.push_back(distance);
    }
    std::vector<float> sorted = distances;
    std::sort(sorted.begin(), sorted.end());
    for (size_t i = 0; i < nearest.size(); ++i) CHECK(distances[nearest[i]] == sorted[i]);
    const std::array<float, 3> lo{1, 2, 3}, hi{8, 9, 5};
    std::vector<size_t> expected;
    for (size_t i = 0; i < points.size(); ++i)
        if (points[i][0] >= 1 && points[i][0] <= 8 && points[i][1] >= 2 && points[i][1] <= 9 && points[i][2] >= 3 &&
            points[i][2] <= 5)
            expected.push_back(i);
    std::vector<size_t> actual = slic3r_api::find_nearby_points(tree, lo, hi, [](size_t) { return true; });
    std::sort(actual.begin(), actual.end());
    CHECK(actual == expected);
    std::future<size_t> parallel = std::async(std::launch::async,
                                              [&] { return slic3r_api::find_closest_point(tree, lo); });
    CHECK(parallel.get() == slic3r_api::find_closest_point(tree, lo));
    tree.clear();
    CHECK(slic3r_api::find_closest_point(tree, lo) == size_t(-1));
    CHECK(slic3r_api::find_nearby_points(tree, lo, 2.f).empty());
}
