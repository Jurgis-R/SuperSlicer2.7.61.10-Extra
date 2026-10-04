///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher

// Snapshot configured spacing, transformed meshes and marked bridge projections into one setup-owned octree.
#include "AdaptivePreparation.hpp"

#include <algorithm>
#include <array>
#include <deque>
#include <exception>
#include <set>
#include <stdexcept>
#include <string>
#include <glu-libtess.h>

#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginContext.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/Api/plugin/cpp/VolumeViews.hpp"

namespace slic3r_api { namespace Infill { namespace AdaptiveCubic {
namespace {

// Average only populated regions belonging to the requested Cubic mode.
double line_spacing(const Print &print, const Object &object, CubicMode mode);
// The linear determinant identifies mirrored transforms requiring reversed triangle winding.
double determinant3(const c_matrix4d &matrix);
// API matrices use row-major affine coordinates, including the centered object translation.
Vec3 apply_matrix(const c_matrix4d &matrix, Vec3 point);

// Keep triangle-strip/fan assembly synchronized with GluTessWrapper in libslic3r/Tesselate.cpp.
class Tessellator
{
public:
    explicit Tessellator(double z) : m_tess(gluNewTess(), gluDeleteTess), m_z(z)
    {
        if (!m_tess)
            throw std::bad_alloc();
        gluTessCallback(m_tess.get(), GLU_TESS_BEGIN_DATA, reinterpret_cast<_GLUfuncptr>(begin));
        gluTessCallback(m_tess.get(), GLU_TESS_VERTEX_DATA, reinterpret_cast<_GLUfuncptr>(vertex));
        gluTessCallback(m_tess.get(), GLU_TESS_COMBINE_DATA, reinterpret_cast<_GLUfuncptr>(combine));
        gluTessCallback(m_tess.get(), GLU_TESS_ERROR_DATA, reinterpret_cast<_GLUfuncptr>(error));
    }

    std::vector<Triangle> run(const ExPolygon &boundary)
    {
        gluTessBeginPolygon(m_tess.get(), this);
        append(boundary.contour());
        for (const Polygon hole : boundary.holes())
            append(hole);
        gluTessEndPolygon(m_tess.get());
        if (m_exception)
            std::rethrow_exception(m_exception);
        if (m_error != 0)
            throw std::runtime_error("Adaptive Cubic boundary triangulation failed.");
        return std::move(m_triangles);
    }

private:
    void append(const Polygon &polygon)
    {
        gluTessBeginContour(m_tess.get());
        const multipoint_const_view points = polygon.view();
        for (uint32_t index = 0; index < points.size; ++index) {
            m_vertices.push_back({unscaled(points.array[index].x), unscaled(points.array[index].y), 0.});
            gluTessVertex(m_tess.get(), m_vertices.back().data(), m_vertices.back().data());
        }
        gluTessEndContour(m_tess.get());
    }

    static void begin(GLenum primitive, void *data)
    {
        Tessellator &self = *static_cast<Tessellator *>(data);
        self.m_primitive = primitive;
        self.m_count = 0;
    }

    static void vertex(const void *vertex_data, void *data) noexcept
    {
        Tessellator &self = *static_cast<Tessellator *>(data);
        if (vertex_data == nullptr || self.m_exception)
            return;
        try {
            const GLdouble *coordinates = static_cast<const GLdouble *>(vertex_data);
            const Vec3 point{coordinates[0], coordinates[1], self.m_z};
            if (++self.m_count == 1)
                self.m_first = point;
            else if (self.m_count == 2)
                self.m_second = point;
            else {
                const bool flip = self.m_primitive == GL_TRIANGLE_STRIP && self.m_count == 4;
                self.m_triangles.push_back(flip ? Triangle{self.m_first, point, self.m_second} :
                                                 Triangle{self.m_first, self.m_second, point});
                if (self.m_primitive == GL_TRIANGLE_STRIP) {
                    if (flip)
                        self.m_count = 2;
                    self.m_first = self.m_second;
                    self.m_second = point;
                } else if (self.m_primitive == GL_TRIANGLE_FAN) {
                    self.m_second = point;
                } else {
                    self.m_count = 0;
                }
            }
        } catch (...) {
            self.m_exception = std::current_exception();
        }
    }

    static void combine(const GLdouble point[3], const GLdouble *[4], const GLfloat[4],
                        GLdouble **output, void *data) noexcept
    {
        Tessellator &self = *static_cast<Tessellator *>(data);
        *output = nullptr;
        try {
            self.m_vertices.push_back({point[0], point[1], point[2]});
            *output = self.m_vertices.back().data();
        } catch (...) {
            self.m_exception = std::current_exception();
        }
    }

    static void error(GLenum code, void *data)
    {
        static_cast<Tessellator *>(data)->m_error = code;
    }

    std::unique_ptr<GLUtesselator, decltype(&gluDeleteTess)> m_tess;
    // GLU keeps vertex addresses until the polygon is complete, including combined vertices.
    std::deque<std::array<GLdouble, 3>> m_vertices;
    std::vector<Triangle> m_triangles;
    std::exception_ptr m_exception;
    Vec3 m_first;
    Vec3 m_second;
    double m_z;
    GLenum m_primitive = GL_TRIANGLES;
    GLenum m_error = 0;
    int m_count = 0;
};

Vec3 apply_matrix(const c_matrix4d &matrix, Vec3 point)
{
    return {matrix.value[0] * point.x + matrix.value[1] * point.y + matrix.value[2] * point.z + matrix.value[3],
            matrix.value[4] * point.x + matrix.value[5] * point.y + matrix.value[6] * point.z + matrix.value[7],
            matrix.value[8] * point.x + matrix.value[9] * point.y + matrix.value[10] * point.z + matrix.value[11]};
}

double line_spacing(const Print &print, const Object &object, const CubicMode mode)
{
    std::set<const print_region_handle *> populated_regions;
    for (uint32_t layer_index = 0; layer_index < object.layer_count(); ++layer_index) {
        const Layer layer = object.layer(layer_index);
        for (uint32_t island_index = 0; island_index < layer.island_count(); ++island_index) {
            const LayerIsland island = layer.island(island_index);
            for (uint32_t region_index = 0; region_index < island.region_island_count(); ++region_index) {
                const LayerRegionIsland region_island = island.region_island(region_index);
                if (!region_island.fill_surfaces().empty())
                    for (const LayerRegion region : region_island.regions())
                        populated_regions.insert(region.print_region().handle());
            }
        }
    }

    const Config print_config = print.config();
    double nozzle = 0.;
    for (uint32_t index = 0; index < print_config.size("nozzle_diameter", 0); ++index)
        nozzle = std::max(nozzle, print_config.vector_float_or_default("nozzle_diameter", index, 0.));
    // Match Flow::auto_extrusion_width(frInfill) in libslic3r/Flow.cpp, including float rounding.
    const double auto_width = 1.125f * float(nozzle);
    double density_percent = 0.;
    double width_sum = 0.;
    size_t count = 0;
    for (uint32_t index = 0; index < object.print_region_count(); ++index) {
        const PrintRegion region = object.print_region(index);
        if (populated_regions.count(region.handle()) == 0)
            continue;
        const Config config = region.config();
        if (!config.has("fill_pattern"))
            continue;
        std::string id = config.get("fill_pattern").serialize();
        if (!id.empty() && id.front() == '!')
            id.erase(id.begin());
        // Public percent getters return fractions, while the object average uses percentages.
        const double density = 100. * config.percent_or_default("fill_density", 0.);
        const char *requested_id = mode == CubicMode::Adaptive ? "adaptivecubic" : "supportcubic";
        if (id != requested_id || density <= 0.)
            continue;
        const double width = config.effective_float_or_percent_or_default("infill_extrusion_width", nozzle, 0.);
        density_percent += density;
        width_sum += width == 0. ? auto_width : width;
        ++count;
    }
    if (count == 0)
        throw std::runtime_error(std::string(mode == CubicMode::Adaptive ? "Adaptive Cubic" : "Support Cubic") +
            " requires a configured region with positive density and nonempty fill surfaces.");
    return (width_sum / double(count)) / ((density_percent / double(count) / 100.0f) * 0.333333333f);
}

double determinant3(const c_matrix4d &matrix)
{
    return matrix.value[0] * (matrix.value[5] * matrix.value[10] - matrix.value[6] * matrix.value[9]) -
           matrix.value[1] * (matrix.value[4] * matrix.value[10] - matrix.value[6] * matrix.value[8]) +
           matrix.value[2] * (matrix.value[4] * matrix.value[9] - matrix.value[5] * matrix.value[8]);
}

} // namespace

std::vector<Triangle> triangulate_boundary(const ExPolygon &boundary, double bottom_z)
{
    return Tessellator(bottom_z).run(boundary);
}

std::unique_ptr<AdaptiveCache> prepare_cache(const Print &print, const Object &object,
                                            const CubicMode mode,
                                            const plugin_run_context *run_ctx)
{
    std::unique_ptr<AdaptiveCache> cache = std::make_unique<AdaptiveCache>();
    cache->mode = mode;
    cache->line_spacing = line_spacing(print, object, mode);
    if (cache->line_spacing <= 0.)
        return cache;

    const c_matrix4d object_matrix = object.transform_centered();
    const double object_determinant = determinant3(object_matrix);
    std::vector<Triangle> triangles;
    for (uint32_t volume_index = 0; volume_index < object.volume_count(); ++volume_index) {
        throw_if_cancelled(run_ctx);
        const Volume volume = object.volume(volume_index);
        if (!volume.is_model_part())
            continue;
        const TriangleMesh mesh = volume.mesh();
        const c_matrix4d matrix = volume.matrix();
        const bool reverse_winding = determinant3(matrix) * object_determinant < 0.;
        std::vector<Vec3> vertices;
        vertices.reserve(mesh.vertex_count());
        for (uint32_t index = 0; index < mesh.vertex_count(); ++index) {
            if ((index & 1023u) == 0)
                throw_if_cancelled(run_ctx);
            const c_vec3f vertex = mesh.vertex(index);
            Vec3 point = apply_matrix(matrix, {vertex.x, vertex.y, vertex.z});
            // The native mesh stores the volume-transformed coordinates as floats.
            point = {double(float(point.x)), double(float(point.y)), double(float(point.z))};
            vertices.push_back(apply_matrix(object_matrix, point));
        }
        for (uint32_t index = 0; index < mesh.triangle_count(); ++index) {
            if ((index & 1023u) == 0)
                throw_if_cancelled(run_ctx);
            const c_triangle_indices indices = mesh.triangle(index);
            Triangle triangle{vertices[indices.a], vertices[indices.b], vertices[indices.c]};
            if (reverse_winding)
                std::swap(triangle.a, triangle.b);
            triangles.push_back(triangle);
        }
    }

    std::vector<Triangle> bridges;
    for (uint32_t layer_index = 0; layer_index < object.layer_count(); ++layer_index) {
        const Layer layer = object.layer(layer_index);
        throw_if_cancelled(run_ctx);
        for (uint32_t island_index = 0; island_index < layer.island_count(); ++island_index) {
            const LayerIsland island = layer.island(island_index);
            for (uint32_t region_index = 0; region_index < island.region_island_count(); ++region_index) {
                const LayerRegionIsland region = island.region_island(region_index);
                for (const Surface surface : region.fill_surfaces()) {
                    if (!surface.has_any_flag(RAW_SURFACE_TYPE_MOD_BRIDGE))
                        continue;
                    throw_if_cancelled(run_ctx);
                    std::vector<Triangle> projected = triangulate_boundary(surface.expolygon(), unscaled(layer.bottom_z()));
                    bridges.insert(bridges.end(), projected.begin(), projected.end());
                }
            }
        }
    }
    cache->octree = Octree::build(triangles, bridges, cache->line_spacing, mode, run_ctx);
    return cache;
}

}}}
