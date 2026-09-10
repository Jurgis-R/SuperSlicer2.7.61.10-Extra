///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

/*
SeamPlacementConstraints: preparation and analytic classification
===============================================================
The implementation separates immutable source constraints from per-query work.
It uses public C contracts and C++ views to read value snapshots, without accessing
internal slicer classes or modifying any extrusion.

During initialize(), walk the printing plan to associate each extrusion and its
descendants with a source layer, instance shift and inherited extrusion properties.
For each referenced object, project enforcer and blocker paint separately. Slice
only SEAM_POSITION_INSIDE volumes at layer print heights, composing the volume and
object transforms with the object's XY recentering. Copy all contours into private
layer caches before temporary storage owners are destroyed. Volume winding preserves
holes and disconnected contours; painted fragments are treated independently so
oppositely oriented faces cannot cancel one another.

During portions(), remove the instance shift from each segment to work in the cached
object frame. Resolve paint margin from the inherited extrusion width, falling back
to the corresponding regional flow. Paint uses the full width as a Euclidean margin;
volumes use their undilated interior. Classify blockers first, then the union of paint
and volume enforcers, and finally free space. The helper reports these categories;
the calling seam policy decides how to select candidates and apply its fallback.

Straight segments and signed minor/major arcs are partitioned analytically against
polygon edges and the lines/circles bounding the paint margin. Extra intersections
may add redundant cuts. Midpoint tests classify the resulting open intervals, while
separate endpoint tests keep closed blocker boundaries excluded from admissible
portions. This avoids uniform sampling that could skip a narrow constraint between
samples. Curves returned to callers are restored to platter coordinates and retain
their actual arc geometry rather than a chord approximation.

Calculations use floating-point parameters and scaled XY coordinates; Curve::point()
rounds the result to integer coordinates. Recheck rounded candidates when boundary
membership matters. Arcs with coincident endpoints cannot be resolved and are
rejected. Paint projection approximates the legacy proximity behavior and does not
promise equivalent 3D triangle-distance results. Query-local vectors and immutable
caches allow concurrent reads while the host keeps borrowed inputs stable.
*/

#include "SeamPlacementConstraints.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <stdexcept>
#include "libslic3r/Api/plugin/cpp/VolumeViews.hpp"
#include "libslic3r/Api/plugin/c/slic3r_extrusion_property.h"


namespace slic3r_api { namespace SeamPlacement {
namespace {
constexpr double pi = 3.14159265358979323846;
using Curve = SeamPlacementConstraints::Curve;
using Status = SeamPlacementConstraints::Status;
using Result = SeamPlacementConstraints::Result;
using Ring = std::vector<c_point>;
using Shape = std::vector<Ring>;
struct XY { double x, y; };
struct Zones { Shape enforced, blocked; std::vector<Shape> volumes; };
struct Binding {
    const layer_handle *layer = nullptr;
    c_point shift{};
    double width = 0;
    raw_extrusion_role role = RAW_EXTRUSION_ROLE_PERIMETER;
};

/* Copy borrowed polygon values before temporary storage is released. */
Shape copy_shape(const PolygonCollection &polygons);
/* Construct the signed minor/major arc encoded by the public segment. */
Curve curve_from(c_extrusion_segment segment);
XY position(const Curve &curve, double t);
/* Closed nonzero-winding containment plus Euclidean dilation. */
bool contains(const Shape &shape, XY point, double margin, bool independent_rings = false);
Status classify(const Zones &zones, XY point, double margin);
/* Add analytic intersections; extra events are harmless and deduplicated. */
void add_cut(std::vector<double> &cuts, double t);
void cut_line(const Curve &curve, XY normal, double offset, std::vector<double> &cuts);
void cut_circle(const Curve &curve, XY center, double radius, std::vector<double> &cuts);
void cut_shape(const Curve &curve, const Shape &shape, double margin, std::vector<double> &cuts);
void visit_segments(const ExtrusionEntity &entity, const Zones &zones, c_point shift,
                    double width, raw_extrusion_role role, const LayerRegion &region, Result &result);

Shape copy_shape(const PolygonCollection &polygons)
{
    Shape result;
    for (uint32_t i = 0; i < polygons.size(); ++i) {
        const Polygon polygon = polygons.at(i);
        Ring ring;
        for (uint32_t j = 0; j < polygon.size(); ++j)
            ring.push_back(polygon.at(j));
        if (ring.size() >= 3) result.push_back(std::move(ring));
    }
    return result;
}

Curve curve_from(c_extrusion_segment segment)
{
    Curve curve;
    curve.segment = segment;
    if (segment.radius == 0) return curve;
    const double dx = double(segment.point_b.x) - segment.point_a.x;
    const double dy = double(segment.point_b.y) - segment.point_a.y;
    const double length = std::hypot(dx, dy);
    if (length == 0) throw std::invalid_argument("Seam constraints cannot resolve an arc with coincident endpoints.");
    curve.radius = std::max(std::abs(double(segment.radius)), length / 2);
    const double h = std::sqrt(std::max(0., curve.radius * curve.radius - length * length / 4));
    // Orientation chooses the center side; a negative radius flips that
    // choice to describe the major arc, matching the public ABI convention.
    const double sign = (segment.orientation == RAW_EXTRUSION_ARC_ORIENTATION_CCW ? 1. : -1.) *
                        (segment.radius > 0 ? 1. : -1.);
    curve.cx = (double(segment.point_a.x) + segment.point_b.x) / 2 - sign * dy * h / length;
    curve.cy = (double(segment.point_a.y) + segment.point_b.y) / 2 + sign * dx * h / length;
    curve.start = std::atan2(segment.point_a.y - curve.cy, segment.point_a.x - curve.cx);
    const double finish = std::atan2(segment.point_b.y - curve.cy, segment.point_b.x - curve.cx);
    curve.sweep = finish - curve.start;
    if (segment.orientation == RAW_EXTRUSION_ARC_ORIENTATION_CCW) {
        if (curve.sweep <= 0) curve.sweep += 2 * pi;
    } else if (curve.sweep >= 0) curve.sweep -= 2 * pi;
    return curve;
}

XY position(const Curve &c, double t)
{
    if (c.radius == 0)
        return {double(c.segment.point_a.x) + t * (double(c.segment.point_b.x) - c.segment.point_a.x),
                double(c.segment.point_a.y) + t * (double(c.segment.point_b.y) - c.segment.point_a.y)};
    return {c.cx + c.radius * std::cos(c.start + t * c.sweep),
            c.cy + c.radius * std::sin(c.start + t * c.sweep)};
}

bool contains(const Shape &shape, XY p, double margin, bool independent_rings)
{
    // Nonzero winding preserves holes and accepts disconnected contours.
    // Edge distance dilates both outside contours and the inside of holes.
    int winding = 0;
    for (const Ring &ring : shape) {
        const int previous_winding = winding;
        for (size_t i = 0; i < ring.size(); ++i) {
            const c_point a = ring[i], b = ring[(i + 1) % ring.size()];
            const double dx = double(b.x) - a.x, dy = double(b.y) - a.y;
            const double length2 = dx * dx + dy * dy;
            const double t = length2 == 0 ? 0 : std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/length2, 0., 1.);
            if (std::hypot(p.x-a.x-t*dx, p.y-a.y-t*dy) <= margin + 1e-5) return true;
            const double cross = dx*(p.y-a.y)-dy*(p.x-a.x);
            if (a.y <= p.y && b.y > p.y && cross > 0) ++winding;
            if (a.y > p.y && b.y <= p.y && cross < 0) --winding;
        }
        // Paint projections are independent triangle fragments, not holes.
        // Oppositely oriented painted faces must not cancel each other.
        if (independent_rings && winding != previous_winding) return true;
    }
    return winding != 0;
}

Status classify(const Zones &zones, XY point, double margin)
{
    if (contains(zones.blocked, point, margin, true)) return Status::Blocked;
    if (contains(zones.enforced, point, margin, true)) return Status::Enforced;
    for (const Shape &volume : zones.volumes)
        if (contains(volume, point, 0)) return Status::Enforced;
    return Status::Free;
}

void add_cut(std::vector<double> &cuts, double t)
{
    if (std::isfinite(t) && t >= -1e-10 && t <= 1+1e-10)
        cuts.push_back(std::clamp(t, 0., 1.));
}

void cut_line(const Curve &c, XY n, double offset, std::vector<double> &cuts)
{
    if (c.radius == 0) {
        const XY a = position(c,0), b = position(c,1);
        const double denominator = n.x*(b.x-a.x)+n.y*(b.y-a.y);
        if (denominator != 0) add_cut(cuts,(offset-n.x*a.x-n.y*a.y)/denominator);
    } else {
        const double norm = std::hypot(n.x,n.y);
        const double d = (offset-n.x*c.cx-n.y*c.cy)/(norm*c.radius);
        if (d >= -1 && d <= 1) {
            const double angle = std::atan2(n.y,n.x), delta = std::acos(d);
            add_cut(cuts,c.parameter(angle+delta));
            add_cut(cuts,c.parameter(angle-delta));
        }
    }
}

void cut_circle(const Curve &c, XY center, double radius, std::vector<double> &cuts)
{
    if (c.radius == 0) {
        const XY a = position(c,0), b = position(c,1);
        const double dx=b.x-a.x, dy=b.y-a.y, x=a.x-center.x, y=a.y-center.y;
        const double aa=dx*dx+dy*dy, bb=2*(x*dx+y*dy), cc=x*x+y*y-radius*radius;
        const double discriminant=bb*bb-4*aa*cc;
        if (aa > 0 && discriminant >= 0) {
            add_cut(cuts,(-bb-std::sqrt(discriminant))/(2*aa));
            add_cut(cuts,(-bb+std::sqrt(discriminant))/(2*aa));
        }
    } else {
        const double dx=center.x-c.cx, dy=center.y-c.cy, d=std::hypot(dx,dy);
        if (d == 0) return;
        const double cosine=(c.radius*c.radius+d*d-radius*radius)/(2*c.radius*d);
        if (cosine >= -1 && cosine <= 1) {
            const double angle=std::atan2(dy,dx), delta=std::acos(cosine);
            add_cut(cuts,c.parameter(angle+delta));
            add_cut(cuts,c.parameter(angle-delta));
        }
    }
}

void cut_shape(const Curve &c, const Shape &shape, double margin, std::vector<double> &cuts)
{
    // A dilated edge is a capsule: two parallel lines and endpoint circles.
    // Intersections outside the finite edge only add redundant split points.
    for (const Ring &ring : shape) {
        for (size_t i=0;i<ring.size();++i) {
            const c_point a=ring[i],b=ring[(i+1)%ring.size()];
            const XY n={double(a.y)-b.y,double(b.x)-a.x};
            const double norm=std::hypot(n.x,n.y), offset=n.x*a.x+n.y*a.y;
            if (norm > 0) {
                cut_line(c,n,offset,cuts);
                if (margin > 0) {
                    cut_line(c,n,offset+margin*norm,cuts);
                    cut_line(c,n,offset-margin*norm,cuts);
                }
            }
            if (margin > 0) cut_circle(c,{double(a.x),double(a.y)},margin,cuts);
        }
    }
}

void visit_segments(const ExtrusionEntity &entity, const Zones &zones, c_point shift,
                    double width, raw_extrusion_role role, const LayerRegion &region, Result &result)
{
    const c_extrusion_property_attributes *attributes = static_cast<const c_extrusion_property_attributes *>(
        entity.property_data(EXTRUSION_PROPERTY_TYPE_ATTRIBUTES));
    if (attributes != nullptr) {
        if (attributes->width > 0) width=scale_d(attributes->width);
        role=attributes->role;
    }
    // Like the legacy proximity radius, use the full extrusion width.
    // Width is inherited down the tree; the regional flow is the last fallback.
    const double margin = width > 0 ? width : double(region.flow(role).width);
    for (uint32_t i=0;i<entity.segment_count();++i) {
        c_extrusion_segment segment=entity.segment(i);
        segment.point_a.x-=shift.x; segment.point_a.y-=shift.y;
        segment.point_b.x-=shift.x; segment.point_b.y-=shift.y;
        const Curve curve=curve_from(segment);
        std::vector<double> cuts={0,1};
        cut_shape(curve,zones.enforced,margin,cuts);
        cut_shape(curve,zones.blocked,margin,cuts);
        for (const Shape &volume:zones.volumes) cut_shape(curve,volume,0,cuts);
        std::sort(cuts.begin(),cuts.end());
        cuts.erase(std::unique(cuts.begin(),cuts.end(),[](double a,double b){return std::abs(a-b)<1e-12;}),cuts.end());
        Curve platter=curve;
        platter.segment=entity.segment(i);
        platter.cx+=shift.x; platter.cy+=shift.y;
        for (size_t j=0;j<cuts.size();++j) {
            const Status endpoint=classify(zones,position(curve,cuts[j]),margin);
            result.relevant |= endpoint!=Status::Free;
            result.portions.push_back({platter,cuts[j],cuts[j],endpoint,true,true,margin});
            if (j+1==cuts.size()) continue;
            const Status middle=classify(zones,position(curve,(cuts[j]+cuts[j+1])/2),margin);
            result.relevant |= middle!=Status::Free;
            result.portions.push_back({platter,cuts[j],cuts[j+1],middle,
                endpoint==middle,classify(zones,position(curve,cuts[j+1]),margin)==middle,margin});
        }
    }
    for (uint32_t i=0;i<entity.child_count();++i)
        visit_segments(entity.child(i),zones,shift,width,role,region,result);
}
} // namespace

struct SeamPlacementConstraints::Impl {
    std::map<const layer_handle *,Zones> layers;
    std::map<const extrusion_entity_handle *,Binding> bindings;
};

SeamPlacementConstraints::SeamPlacementConstraints() : m_impl(new Impl()) {}
SeamPlacementConstraints::~SeamPlacementConstraints() = default;

c_point SeamPlacementConstraints::Curve::point(double t) const
{
    if (t<=0) return segment.point_a;
    if (t>=1) return segment.point_b;
    const XY p=position(*this,t);
    return {coord_t(std::llround(p.x)),coord_t(std::llround(p.y))};
}

double SeamPlacementConstraints::Curve::parameter(double angle) const
{
    double delta=std::fmod(angle-start,2*pi);
    if (sweep>0 && delta<0) delta+=2*pi;
    if (sweep<0 && delta>0) delta-=2*pi;
    return delta/sweep;
}

void SeamPlacementConstraints::initialize(const PrintingPlan &plan, storage_handle *storage)
{
    if (!plan.valid() || storage==nullptr) throw std::invalid_argument("Seam constraints need plan and temporary storage.");
    std::unique_ptr<Impl> prepared(new Impl());
    std::map<const object_handle *,bool> objects;
    /* Register all descendant handles with their explicit source instance. No
       tool-position inference or spatial search across translated copies. */
    std::function<void(const ExtrusionEntity &,Binding)> register_tree;
    register_tree=[&](const ExtrusionEntity &entity,Binding binding) {
        prepared->bindings.emplace(entity.handle(),binding);
        const c_extrusion_property_attributes *attributes=static_cast<const c_extrusion_property_attributes *>(
            entity.property_data(EXTRUSION_PROPERTY_TYPE_ATTRIBUTES));
        if (attributes!=nullptr) {
            if (attributes->width>0) binding.width=scale_d(attributes->width);
            binding.role=attributes->role;
        }
        for (uint32_t i=0;i<entity.child_count();++i) register_tree(entity.child(i),binding);
    };
    for (uint32_t g=0;g<plan.group_count();++g) {
        const PrintingGroup group=plan.group(g);
        for (uint32_t l=0;l<group.layer_group_count();++l) {
            const PrintingLayerGroup layer_group=group.layer_group(l);
            for (uint32_t t=0;t<layer_group.tool_group_count();++t) {
                const PrintingToolGroup tool=layer_group.tool_group(t);
                for (uint32_t e=0;e<tool.extrusion_count();++e) {
                    const PrintingExtrusion extrusion=tool.extrusion(e);
                    const LayerRegionIsland island=extrusion.region_island();
                    if (!island.valid() || island.region_count()==0) continue;
                    const Layer layer=island.region(0).layer();
                    const Object object=layer.object();
                    if (extrusion.object_instance_idx()>=object.instance_count())
                        throw std::invalid_argument("Seam extrusion has invalid source instance.");
                    objects.emplace(object.handle(),true);
                    register_tree(extrusion.root(),{layer.handle(),object.instance_shift(extrusion.object_instance_idx()),0,extrusion.role()});
                }
            }
        }
    }
    // One projection per source object, shared by all its translated instances.
    // Stored collections release their allocations even if preparation throws.
    for (const std::pair<const object_handle *const,bool> &entry:objects) {
        const Object object(entry.first);
        std::vector<StoredPolygonCollection> enforced=project_painting_to_polygons(storage,object,RAW_FACET_PAINTING_SEAM,RAW_FACET_PAINTING_ENFORCER);
        std::vector<StoredPolygonCollection> blocked=project_painting_to_polygons(storage,object,RAW_FACET_PAINTING_SEAM,RAW_FACET_PAINTING_BLOCKER);
        std::vector<float> zs;
        for (uint32_t l=0;l<object.layer_count();++l) {
            const Layer layer=object.layer(l);
            Zones &zones=prepared->layers[layer.handle()];
            zones.enforced=copy_shape(enforced[l].readonly());
            zones.blocked=copy_shape(blocked[l].readonly());
            zs.push_back(float(unscaled(layer.print_z())));
        }
        const c_point center=object.center_offset();
        const c_matrix4d transform=matrix4d_mul(matrix4d_translation(-unscaled(center.x),-unscaled(center.y),0),object.transform());
        for (uint32_t v=0;v<object.volume_count();++v) {
            const Volume volume=object.volume(v);
            if (volume.type()!=RAW_VOLUME_TYPE_SEAM_POSITION_INSIDE) continue;
            std::vector<StoredPolygonCollection> slices=volume.mesh().slice_to_polygons(
                storage,matrix4d_mul(transform,volume_get_matrix(volume.handle())),zs);
            for (uint32_t l=0;l<object.layer_count();++l)
                prepared->layers[object.layer(l).handle()].volumes.push_back(copy_shape(slices[l].readonly()));
        }
    }
    m_impl=std::move(prepared);
}

SeamPlacementConstraints::Result SeamPlacementConstraints::portions(const ExtrusionEntity &loop,const LayerRegion &region) const
{
    Result result;
    const std::map<const extrusion_entity_handle *,Binding>::const_iterator binding=m_impl->bindings.find(loop.handle());
    if (binding==m_impl->bindings.end()) {
        if (!m_impl->bindings.empty()) throw std::invalid_argument("Seam loop was not present in the initialized plan.");
        return result;
    }
    const std::map<const layer_handle *,Zones>::const_iterator zones=m_impl->layers.find(binding->second.layer);
    if (zones==m_impl->layers.end()) return result;
    if (zones->second.enforced.empty() && zones->second.blocked.empty() && zones->second.volumes.empty()) return result;
    if (region.layer().handle()!=binding->second.layer)
        throw std::invalid_argument("Seam loop and source region belong to different layers.");
    visit_segments(loop,zones->second,binding->second.shift,binding->second.width,binding->second.role,region,result);
    return result;
}

SeamPlacementConstraints::Status SeamPlacementConstraints::classify_point(
    const ExtrusionEntity &loop, c_point point, double paint_margin) const
{
    const std::map<const extrusion_entity_handle *,Binding>::const_iterator binding=m_impl->bindings.find(loop.handle());
    if (binding==m_impl->bindings.end()) return Status::Free;
    const std::map<const layer_handle *,Zones>::const_iterator zones=m_impl->layers.find(binding->second.layer);
    if (zones==m_impl->layers.end()) return Status::Free;
    return classify(zones->second,{double(point.x)-binding->second.shift.x,double(point.y)-binding->second.shift.y},paint_margin);
}

}} // namespace slic3r_api::SeamPlacement
