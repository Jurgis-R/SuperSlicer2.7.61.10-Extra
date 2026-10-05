///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
Adaptive Cubic octree implementation
=====================================

The builder stores model and bridge triangles in the legacy octree frame. It
rotates only the generated cube centers back to slicer coordinates, keeping
line extraction independent from the mesh lifetime.
*/

#include "AdaptiveOctree.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "libslic3r/Api/plugin/c/slic3r_def.h"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"

/*
The SAT test below is the only geometric predicate needed while building the
tree. Keeping the tree and its insertion predicate in this private module
avoids exposing the legacy core octree types through the plugin boundary.
*/

namespace slic3r_api { namespace Infill { namespace AdaptiveCubic {
namespace {

constexpr double k_pi = 3.14159265358979323846;
constexpr double k_octree_x_rotation = 5.0 * k_pi / 4.0;
constexpr double k_octree_y_rotation = 215.264 * k_pi / 180.0;
constexpr double k_octree_z_rotation = k_pi / 6.0;
constexpr double k_child_epsilon = EPSILON;
constexpr coord_t k_line_join_epsilon = 1000;

const std::array<Vec3, 8> k_child_centers = {
    Vec3{-1., -1., -1.}, Vec3{1., -1., -1.}, Vec3{-1., 1., -1.}, Vec3{1., 1., -1.},
    Vec3{-1., -1., 1.},  Vec3{1., -1., 1.},  Vec3{-1., 1., 1.},  Vec3{1., 1., 1.}
};

const std::array<std::array<int, 8>, 3> k_child_traversal_order = {{
    {{2, 3, 0, 1, 6, 7, 4, 5}},
    {{4, 0, 6, 2, 5, 1, 7, 3}},
    {{1, 5, 0, 4, 3, 7, 2, 6}}
}};

struct BoundingBox
{
    Vec3 min;
    Vec3 max;
};

struct Vec2
{
    double x = 0.;
    double y = 0.;
};

struct LineContext
{
    LineContext(const std::vector<CubeProperties> &properties, double z_position, int direction)
        : cubes_properties(properties)
        , z_position(z_position)
        , traversal_order(k_child_traversal_order[direction])
        , cos_angle(std::cos(direction == 0 ? 0. : direction == 1 ? 2. * k_pi / 3. : -2. * k_pi / 3.))
        , sin_angle(std::sin(direction == 0 ? 0. : direction == 1 ? 2. * k_pi / 3. : -2. * k_pi / 3.))
    {
        if (properties.size() >= sizeof(size_t) * 8)
            throw std::runtime_error("Adaptive Cubic octree is too deep for line addressing.");
        const c_point empty_point{ std::numeric_limits<coord_t>::max(), std::numeric_limits<coord_t>::max() };
        const std::pair<c_point, c_point> empty_line{ empty_point, empty_point };
        temp_lines.resize((size_t(1) << properties.size()) - 1, empty_line);
    }

    Vec2 rotate(double x, double y) const
    {
        return { cos_angle * x - sin_angle * y, sin_angle * x + cos_angle * y };
    }

    const std::vector<CubeProperties> &cubes_properties;
    double z_position;
    std::array<int, 8> traversal_order;
    double cos_angle;
    double sin_angle;
    std::vector<std::pair<c_point, c_point>> temp_lines;
    std::vector<std::pair<c_point, c_point>> output_lines;
};

// Select the legacy upward-facing model triangles used for Support Cubic refinement.
bool is_support_surface(const Triangle &triangle, const Vec3 &up);

Vec3 operator+(Vec3 lhs, Vec3 rhs)
{
    return { lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z };
}

Vec3 operator-(Vec3 lhs, Vec3 rhs)
{
    return { lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z };
}

Vec3 operator*(Vec3 point, double factor)
{
    return { point.x * factor, point.y * factor, point.z * factor };
}

double dot(Vec3 lhs, Vec3 rhs)
{
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

Vec3 cross(Vec3 lhs, Vec3 rhs)
{
    return { lhs.y * rhs.z - lhs.z * rhs.y,
             lhs.z * rhs.x - lhs.x * rhs.z,
             lhs.x * rhs.y - lhs.y * rhs.x };
}

Vec3 rotate_x(Vec3 point, double angle)
{
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    return { point.x, cosine * point.y - sine * point.z, sine * point.y + cosine * point.z };
}

Vec3 rotate_y(Vec3 point, double angle)
{
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    return { cosine * point.x + sine * point.z, point.y, -sine * point.x + cosine * point.z };
}

Vec3 rotate_z(Vec3 point, double angle)
{
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    return { cosine * point.x - sine * point.y, sine * point.x + cosine * point.y, point.z };
}

Vec3 octree_to_world(Vec3 point)
{
    return rotate_z(rotate_y(rotate_x(point, k_octree_x_rotation), k_octree_y_rotation), k_octree_z_rotation);
}

Vec3 world_to_octree(Vec3 point)
{
    return rotate_x(rotate_y(rotate_z(point, -k_octree_z_rotation), -k_octree_y_rotation), -k_octree_x_rotation);
}

Triangle transform_triangle_to_octree(const Triangle &triangle)
{
    return { world_to_octree(triangle.a), world_to_octree(triangle.b), world_to_octree(triangle.c) };
}

bool is_support_surface(const Triangle &triangle, const Vec3 &up)
{
    const Vec3 normal = cross(triangle.b - triangle.a, triangle.c - triangle.b);
    return dot(normal, up) > 0.707 * std::sqrt(dot(normal, normal));
}

BoundingBox triangle_bounds(const Triangle &triangle)
{
    return {
        { std::min({triangle.a.x, triangle.b.x, triangle.c.x}),
          std::min({triangle.a.y, triangle.b.y, triangle.c.y}),
          std::min({triangle.a.z, triangle.b.z, triangle.c.z}) },
        { std::max({triangle.a.x, triangle.b.x, triangle.c.x}),
          std::max({triangle.a.y, triangle.b.y, triangle.c.y}),
          std::max({triangle.a.z, triangle.b.z, triangle.c.z}) }
    };
}

bool triangle_intersects_box(const Triangle &triangle, const BoundingBox &box)
{
    const BoundingBox bounds = triangle_bounds(triangle);
    if (bounds.max.x <= box.min.x || bounds.min.x >= box.max.x ||
        bounds.max.y <= box.min.y || bounds.min.y >= box.max.y ||
        bounds.max.z <= box.min.z || bounds.min.z >= box.max.z)
        return false;

    const Vec3 center = (box.min + box.max) * 0.5;
    const Vec3 half = box.max - center;
    const Vec3 edges[] = {triangle.b - triangle.a, triangle.c - triangle.a, triangle.c - triangle.b};
    const Vec3 a = triangle.a - center;
    const Vec3 b = triangle.b - center;
    const Vec3 c = triangle.c - center;
    const Vec3 normal = cross(edges[0], edges[1]);
    if (std::abs(dot(normal, a)) >=
        std::abs(half.x * std::abs(normal.x) + half.y * std::abs(normal.y) + half.z * std::abs(normal.z)))
        return false;
    for (int axis = 0; axis < 3; ++axis)
        for (int edge = 0; edge < 3; ++edge) {
            const Vec3 &v = edges[edge];
            const Vec3 &other = edge == 0 ? c : b;
            double first, second, radius;
            if (axis == 0) {
                first = v.y * a.z - v.z * a.y;
                second = v.y * other.z - v.z * other.y;
                radius = std::abs(half.y * std::abs(v.z) + half.z * std::abs(v.y));
            } else if (axis == 1) {
                first = v.z * a.x - v.x * a.z;
                second = v.z * other.x - v.x * other.z;
                radius = std::abs(half.x * std::abs(v.z) + half.z * std::abs(v.x));
            } else {
                first = v.x * a.y - v.y * a.x;
                second = v.x * other.y - v.y * other.x;
                radius = std::abs(half.y * std::abs(v.x) + half.x * std::abs(v.y));
            }
            const double midpoint = (first + second) * 0.5;
            if (radius + std::abs(midpoint - first) < std::abs(midpoint))
                return false;
        }
    return true;
}

std::vector<CubeProperties> make_cube_properties(double max_cube_edge_length, double line_spacing)
{
    std::vector<CubeProperties> properties;
    for (double edge_length = line_spacing * 2.;; edge_length *= 2.) {
        CubeProperties property;
        property.edge_length = edge_length;
        property.height = edge_length * std::sqrt(3.);
        property.diagonal_length = edge_length * std::sqrt(2.);
        property.line_z_distance = edge_length / std::sqrt(3.);
        property.line_xy_distance = edge_length / std::sqrt(6.);
        properties.push_back(property);
        if (edge_length > max_cube_edge_length + EPSILON)
            break;
    }
    return properties;
}

BoundingBox child_box(const BoundingBox &parent, Vec3 parent_center, Vec3 child_direction)
{
    BoundingBox result = parent;
    if (child_direction.x < 0.)
        result.max.x = parent_center.x + k_child_epsilon;
    else
        result.min.x = parent_center.x - k_child_epsilon;
    if (child_direction.y < 0.)
        result.max.y = parent_center.y + k_child_epsilon;
    else
        result.min.y = parent_center.y - k_child_epsilon;
    if (child_direction.z < 0.)
        result.max.z = parent_center.z + k_child_epsilon;
    else
        result.min.z = parent_center.z - k_child_epsilon;
    return result;
}

void insert_triangle(Cube *current,
                     const Vec3 &a,
                     const Vec3 &b,
                     const Vec3 &c,
                     const Vec3 &current_min,
                     const Vec3 &current_max,
                     int depth,
                      const std::vector<CubeProperties> &properties,
                      const plugin_run_context *run_ctx)
{
    assert(current != nullptr);
    throw_if_cancelled(run_ctx);
    --depth;
    const BoundingBox parent_box{current_min, current_max};
    for (size_t index = 0; index < k_child_centers.size(); ++index) {
        const Vec3 child_direction = k_child_centers[index];
        const Vec3 child_center = current->center + child_direction * (properties[size_t(depth)].edge_length / 2.);
        const BoundingBox box = child_box(parent_box, current->center, child_direction);
        const Triangle triangle{a, b, c};
        if (!triangle_intersects_box(triangle, box))
            continue;
        if (!current->children[index])
            current->children[index] = std::make_unique<Cube>(child_center);
        if (depth > 0)
            insert_triangle(current->children[index].get(), a, b, c,
                             box.min, box.max, depth, properties, run_ctx);
    }
}

void transform_centers(Cube *cube, const plugin_run_context *run_ctx)
{
    if (cube == nullptr)
        return;
    throw_if_cancelled(run_ctx);
    cube->center = octree_to_world(cube->center);
    for (std::unique_ptr<Cube> &child : cube->children)
        transform_centers(child.get(), run_ctx);
}

void append_line(LineContext &context, const Cube &cube, int address, int depth,
                 const plugin_run_context *run_ctx)
{
    throw_if_cancelled(run_ctx);
    const CubeProperties &property = context.cubes_properties[size_t(depth)];
    const double z_difference = context.z_position - cube.center.z;
    const double absolute_z_difference = std::abs(z_difference);
    if (absolute_z_difference > property.height / 2.)
        return;

    if (absolute_z_difference < property.line_z_distance) {
        const double z_distance = property.line_z_distance;
        const double from_x = 0.5 * property.diagonal_length *
                              (z_distance - absolute_z_difference) / z_distance;
        const double from_y = property.line_xy_distance -
                              (z_distance + z_difference) / std::sqrt(2.);
        const Vec2 rotated_from = context.rotate(from_x, from_y);
        const Vec2 rotated_to = context.rotate(-from_x, from_y);
        const c_point from = { scale_i(rotated_from.x + cube.center.x),
                               scale_i(rotated_from.y + cube.center.y) };
        const c_point to = { scale_i(rotated_to.x + cube.center.x),
                             scale_i(rotated_to.y + cube.center.y) };
        std::pair<c_point, c_point> &last_line = context.temp_lines[size_t(address)];
        const std::pair<c_point, c_point> new_line = { from, to };
        if (last_line.first.x == std::numeric_limits<coord_t>::max())
            last_line.first = new_line.first;
        else if (std::max(std::abs(new_line.first.x - last_line.second.x),
                          std::abs(new_line.first.y - last_line.second.y)) > k_line_join_epsilon) {
            context.output_lines.push_back(last_line);
            last_line.first = new_line.first;
        }
        last_line.second = new_line.second;
    }

    int child_address = address * 2 + 1;
    size_t child_number = 0;
    for (const int child_index : context.traversal_order) {
        const std::unique_ptr<Cube> &child = cube.children[size_t(child_index)];
        if (child != nullptr)
            append_line(context, *child, child_address, depth - 1, run_ctx);
        if (++child_number == 4)
            ++child_address;
    }
}

} // namespace

Octree::Octree(Vec3 origin, std::vector<CubeProperties> properties)
    : m_origin(origin)
    , m_cubes_properties(std::move(properties))
    , m_root_cube(std::make_unique<Cube>(origin))
{}

std::unique_ptr<Octree> Octree::build(const std::vector<Triangle> &triangle_mesh,
                                      const std::vector<Triangle> &overhang_triangles,
                                      double line_spacing,
                                      const CubicMode mode,
                                      const plugin_run_context *run_ctx)
{
    if (triangle_mesh.empty() || line_spacing <= 0. || !std::isfinite(line_spacing))
        return nullptr;

    std::vector<Triangle> octree_triangles;
    octree_triangles.reserve(triangle_mesh.size());
    for (const Triangle &triangle : triangle_mesh) {
        throw_if_cancelled(run_ctx);
        Triangle converted = transform_triangle_to_octree(triangle);
        for (Vec3 *point : {&converted.a, &converted.b, &converted.c})
            *point = {double(float(point->x)), double(float(point->y)), double(float(point->z))};
        octree_triangles.push_back(converted);
    }

    std::vector<Triangle> octree_overhang_triangles;
    octree_overhang_triangles.reserve(overhang_triangles.size());
    for (const Triangle &triangle : overhang_triangles)
        octree_overhang_triangles.push_back(transform_triangle_to_octree(triangle));

    const Vec3 up = mode == CubicMode::Support ? world_to_octree({0., 0., 1.}) : Vec3{};

    BoundingBox bounds{octree_triangles.front().a, octree_triangles.front().a};
    auto extend_bounds = [&bounds](const Vec3 &point) {
        bounds.min.x = std::min(bounds.min.x, point.x);
        bounds.min.y = std::min(bounds.min.y, point.y);
        bounds.min.z = std::min(bounds.min.z, point.z);
        bounds.max.x = std::max(bounds.max.x, point.x);
        bounds.max.y = std::max(bounds.max.y, point.y);
        bounds.max.z = std::max(bounds.max.z, point.z);
    };
    for (const Triangle &triangle : octree_triangles) {
        extend_bounds(triangle.a);
        extend_bounds(triangle.b);
        extend_bounds(triangle.c);
    }

    const double max_edge = std::max({float(bounds.max.x) - float(bounds.min.x),
                                      float(bounds.max.y) - float(bounds.min.y),
                                      float(bounds.max.z) - float(bounds.min.z)});
    std::vector<CubeProperties> properties = make_cube_properties(max_edge, line_spacing);
    const Vec3 center{double((float(bounds.min.x) + float(bounds.max.x)) * 0.5f),
                      double((float(bounds.min.y) + float(bounds.max.y)) * 0.5f),
                      double((float(bounds.min.z) + float(bounds.max.z)) * 0.5f)};
    std::unique_ptr<Octree> octree(new Octree(center, std::move(properties)));

    if (octree->m_cubes_properties.size() > 1) {
        const CubeProperties &root_property = octree->m_cubes_properties.back();
        const Vec3 half_size{root_property.edge_length / 2., root_property.edge_length / 2., root_property.edge_length / 2.};
        const Vec3 root_min = octree->m_origin - half_size;
        const Vec3 root_max = octree->m_origin + half_size;
        const int max_depth = int(octree->m_cubes_properties.size()) - 1;
        for (const Triangle &triangle : octree_triangles) {
            if (mode == CubicMode::Support && !is_support_surface(triangle, up))
                continue;
            insert_triangle(octree->m_root_cube.get(), triangle.a, triangle.b, triangle.c,
                            root_min, root_max, max_depth, octree->m_cubes_properties, run_ctx);
        }
        for (const Triangle &triangle : octree_overhang_triangles)
            insert_triangle(octree->m_root_cube.get(), triangle.a, triangle.b, triangle.c,
                            root_min, root_max, max_depth, octree->m_cubes_properties, run_ctx);
        transform_centers(octree->m_root_cube.get(), run_ctx);
    }

    return octree;
}

std::vector<std::pair<c_point, c_point>> Octree::generate_lines(double z_position,
                                                             const plugin_run_context *run_ctx) const
{
    std::vector<std::pair<c_point, c_point>> lines;
    if (m_root_cube == nullptr || m_cubes_properties.empty())
        return lines;

    for (int direction = 0; direction < 3; ++direction) {
        LineContext context(m_cubes_properties, z_position, direction);
        append_line(context, *m_root_cube, 0, int(m_cubes_properties.size()) - 1, run_ctx);
        lines.insert(lines.end(), context.output_lines.begin(), context.output_lines.end());
        for (const std::pair<c_point, c_point> &line : context.temp_lines)
            if (line.first.x != std::numeric_limits<coord_t>::max())
                lines.push_back(line);
    }
    return lines;
}

}}} // namespace slic3r_api::Infill::AdaptiveCubic
