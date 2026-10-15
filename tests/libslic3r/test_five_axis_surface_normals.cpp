#include "libslic3r/FiveAxisSurfaceNormals.hpp"

#include <catch2/catch.hpp>

using namespace Slic3r;

static indexed_triangle_set make_test_surface()
{
    indexed_triangle_set mesh;
    mesh.vertices.emplace_back(-10.f, -10.f, 0.f);
    mesh.vertices.emplace_back( 10.f, -10.f, 0.f);
    mesh.vertices.emplace_back(  0.f,  10.f, 0.f);
    mesh.indices.emplace_back(0, 1, 2); // +Z normal
    return mesh;
}

TEST_CASE("Five-axis surface sampler derives a normal from the nearest mesh triangle", "[FiveAxisSurfaceNormals]")
{
    const indexed_triangle_set mesh = make_test_surface();
    const FiveAxisSurfaceNormalSampler sampler(mesh);
    FiveAxisVec3 normal;
    double distance = -1.0;

    const auto result = sampler.sample({0.0, 0.0, 2.0}, normal, &distance, 3.0);
    REQUIRE(result.valid);
    CHECK(normal.x == Approx(0.0).margin(1e-6));
    CHECK(normal.y == Approx(0.0).margin(1e-6));
    CHECK(normal.z == Approx(1.0).margin(1e-6));
    CHECK(distance == Approx(2.0).margin(1e-5));
}

TEST_CASE("Five-axis surface sampler rejects distant samples and invalid limits", "[FiveAxisSurfaceNormals]")
{
    const indexed_triangle_set mesh = make_test_surface();
    const FiveAxisSurfaceNormalSampler sampler(mesh);
    FiveAxisVec3 normal;

    CHECK_FALSE(sampler.sample({0.0, 0.0, 2.0}, normal, nullptr, 1.0).valid);
    CHECK_FALSE(sampler.sample({0.0, 0.0, 2.0}, normal, nullptr, -1.0).valid);
}

TEST_CASE("Five-axis surface sampler assigns per-vertex normals atomically", "[FiveAxisSurfaceNormals]")
{
    const indexed_triangle_set mesh = make_test_surface();
    const FiveAxisSurfaceNormalSampler sampler(mesh);
    ExtrusionPath3D path(ExtrusionAttributes{}, true);
    path.push_back(Point(0, 0), 0);
    path.push_back(Point(1000, 0), 0);

    auto result = sampler.assign_path_normals(path, 0.0, 0.1);
    REQUIRE(result.valid);
    REQUIRE(path.has_tool_axis_normals());
    REQUIRE(path.tool_axis_normals.size() == 2);
    CHECK(path.tool_axis_normals[0][2] == Approx(1.0).margin(1e-6));
    CHECK(path.tool_axis_normals[1][2] == Approx(1.0).margin(1e-6));

    const auto previous = path.tool_axis_normals;
    result = sampler.assign_path_normals(path, 20.0, 0.1);
    CHECK_FALSE(result.valid);
    CHECK(path.tool_axis_normals == previous);
}


TEST_CASE("Five-axis surface mesh transform maps geometry into print coordinates", "[FiveAxisSurfaceNormals]")
{
    const indexed_triangle_set source = make_test_surface();
    indexed_triangle_set print_mesh;
    Transform3d transform = Transform3d::Identity();
    transform.translate(Vec3d(5.0, 0.0, 3.0));

    const auto transformed = transform_five_axis_surface_mesh(source, transform, print_mesh);
    REQUIRE(transformed.valid);
    REQUIRE(print_mesh.vertices.size() == source.vertices.size());
    REQUIRE(print_mesh.indices.size() == source.indices.size());

    const FiveAxisSurfaceNormalSampler sampler(print_mesh);
    FiveAxisVec3 normal;
    double distance = -1.0;
    const auto sampled = sampler.sample({5.0, 0.0, 5.0}, normal, &distance, 3.0);
    REQUIRE(sampled.valid);
    CHECK(normal.z == Approx(1.0).margin(1e-6));
    CHECK(distance == Approx(2.0).margin(1e-5));
}

TEST_CASE("Five-axis surface mesh transform preserves normal orientation for mirrored transforms", "[FiveAxisSurfaceNormals]")
{
    const indexed_triangle_set source = make_test_surface();
    indexed_triangle_set print_mesh;
    Transform3d transform = Transform3d::Identity();
    transform.linear()(0, 0) = -1.0;

    REQUIRE(transform_five_axis_surface_mesh(source, transform, print_mesh).valid);
    const FiveAxisSurfaceNormalSampler sampler(print_mesh);
    FiveAxisVec3 normal;
    REQUIRE(sampler.sample({0.0, 0.0, 2.0}, normal, nullptr, 3.0).valid);
    CHECK(normal.z == Approx(1.0).margin(1e-6));
}

TEST_CASE("Five-axis surface mesh transform rejects singular transforms without changing output", "[FiveAxisSurfaceNormals]")
{
    const indexed_triangle_set source = make_test_surface();
    indexed_triangle_set output = source;
    Transform3d transform = Transform3d::Identity();
    transform.linear()(2, 2) = 0.0;

    CHECK_FALSE(transform_five_axis_surface_mesh(source, transform, output).valid);
    CHECK(output.vertices.size() == source.vertices.size());
    CHECK(output.indices.size() == source.indices.size());
}


TEST_CASE("Five-axis volume mesh transform composes centered object and volume transforms", "[FiveAxisSurfaceNormals]")
{
    const indexed_triangle_set source = make_test_surface();
    indexed_triangle_set print_mesh;
    Transform3d object_transform = Transform3d::Identity();
    object_transform.translate(Vec3d(5.0, 0.0, 3.0));
    Transform3d volume_transform = Transform3d::Identity();
    volume_transform.translate(Vec3d(2.0, 0.0, 0.0));

    REQUIRE(transform_five_axis_volume_mesh(
        source, object_transform, volume_transform, print_mesh).valid);
    const FiveAxisSurfaceNormalSampler sampler(print_mesh);
    FiveAxisVec3 normal;
    double distance = -1.0;
    REQUIRE(sampler.sample({7.0, 0.0, 5.0}, normal, &distance, 3.0).valid);
    CHECK(normal.z == Approx(1.0).margin(1e-6));
    CHECK(distance == Approx(2.0).margin(1e-5));
}

TEST_CASE("Five-axis correspondence selects one source volume for a whole path", "[FiveAxisSurfaceNormals]")
{
    const indexed_triangle_set lower = make_test_surface();
    indexed_triangle_set upper = make_test_surface();
    for (Vec3f &v : upper.vertices)
        v.z() += 5.f;

    FiveAxisSurfaceCorrespondence correspondence;
    REQUIRE(correspondence.add_source(17, lower).valid);
    REQUIRE(correspondence.add_source(23, upper).valid);

    ExtrusionPath3D path(ExtrusionAttributes{}, true);
    path.push_back(Point(0, 0), 0);
    path.push_back(Point(1000, 0), 0);
    int matched_id = -1;
    const auto result = correspondence.assign_path_normals(path, 0.0, 1.0, 0.05, &matched_id);
    REQUIRE(result.valid);
    CHECK(matched_id == 17);
    REQUIRE(path.has_tool_axis_normals());
    CHECK(path.tool_axis_normals[0][2] == Approx(1.0).margin(1e-6));
}

TEST_CASE("Five-axis correspondence rejects equally plausible overlapping source volumes", "[FiveAxisSurfaceNormals]")
{
    const indexed_triangle_set first = make_test_surface();
    const indexed_triangle_set second = make_test_surface();
    FiveAxisSurfaceCorrespondence correspondence;
    REQUIRE(correspondence.add_source(1, first).valid);
    REQUIRE(correspondence.add_source(2, second).valid);

    ExtrusionPath3D path(ExtrusionAttributes{}, true);
    path.push_back(Point(0, 0), 0);
    path.push_back(Point(1000, 0), 0);
    path.tool_axis_normals = { { { 0.0, 1.0, 0.0 } }, { { 0.0, 1.0, 0.0 } } };
    const auto previous = path.tool_axis_normals;
    CHECK_FALSE(correspondence.assign_path_normals(path, 0.0, 1.0).valid);
    CHECK(path.tool_axis_normals == previous);
}

TEST_CASE("Five-axis surface sampler rejects nearby opposing faces on a thin wall", "[FiveAxisSurfaceNormals]")
{
    indexed_triangle_set thin_wall = make_test_surface();
    const int offset = int(thin_wall.vertices.size());
    for (const Vec3f &v : make_test_surface().vertices)
        thin_wall.vertices.emplace_back(v.x(), v.y(), v.z() + 0.1f);
    // Reverse winding on the opposite side of the 0.1 mm wall.
    thin_wall.indices.emplace_back(offset + 0, offset + 2, offset + 1);

    const FiveAxisSurfaceNormalSampler sampler(thin_wall);
    FiveAxisVec3 normal;
    const auto result = sampler.sample({0.0, 0.0, 0.01}, normal, nullptr, 1.0);
    CHECK_FALSE(result.valid);
    CHECK(result.error.find("opposing face") != std::string::npos);
}

TEST_CASE("Five-axis surface sampler can disable thin-wall ambiguity probing explicitly", "[FiveAxisSurfaceNormals]")
{
    indexed_triangle_set thin_wall = make_test_surface();
    const int offset = int(thin_wall.vertices.size());
    const indexed_triangle_set opposite = make_test_surface();
    for (const Vec3f &v : opposite.vertices)
        thin_wall.vertices.emplace_back(v.x(), v.y(), v.z() + 0.1f);
    thin_wall.indices.emplace_back(offset + 0, offset + 2, offset + 1);

    const FiveAxisSurfaceNormalSampler sampler(thin_wall);
    FiveAxisVec3 normal;
    const auto result = sampler.sample({0.0, 0.0, 0.01}, normal, nullptr, 1.0, 0.0);
    CHECK(result.valid);
    CHECK(normal.z == Approx(1.0).margin(1e-6));
}
