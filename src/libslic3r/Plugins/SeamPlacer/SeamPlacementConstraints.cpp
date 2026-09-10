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
object transforms with the object's XY recentering. Copy all contours and build their owned AABB indexes in private
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
indexed finite polygon edges and the sides/caps of the paint margin. AABB
queries bound the actual arcs and retain exact floating-point classification. Midpoint tests classify the resulting open intervals, while
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
#include "libslic3r/Api/plugin/cpp/LineDistancer.hpp"
#include "libslic3r/Api/plugin/c/slic3r_extrusion_property.h"


namespace slic3r_api { namespace SeamPlacement {
namespace {
constexpr double pi = 3.14159265358979323846;
using Curve = SeamPlacementConstraints::Curve;
using Status = SeamPlacementConstraints::Status;
using Result = SeamPlacementConstraints::Result;
// Indices match the distancer insertion order. Everything is owned by the
// session; no index stores pointers into vectors or borrowed host polygons.
struct Edge { c_point a, b; size_t ring; };
struct Shape { std::vector<Edge> edges; LineDistancer index; };
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
/* Round broad-phase boxes outwards; exact tests retain floating query points. */
coord_t bounded_coordinate(double value, bool upper);
c_point lower_bound(XY point);
c_point upper_bound(XY point);
/* Bound the actual arc including cardinal extrema, never just its chord. */
std::pair<c_point,c_point> curve_bounds(const Curve &curve,double margin);
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
    for (uint32_t i=0;i<polygons.size();++i) {
        const Polygon polygon=polygons.at(i);
        if (polygon.size()<3) continue;
        for (uint32_t j=0;j<polygon.size();++j) {
            const c_point a=polygon.at(j), b=polygon.at((j+1)%polygon.size());
            result.edges.push_back({a,b,i});
            result.index.append_segment(a,b);
        }
    }
    result.index.build_index();
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

coord_t bounded_coordinate(double value, bool upper)
{
    // double(INT64_MAX) rounds upwards: saturate before any integer conversion.
    if (value>=double(std::numeric_limits<coord_t>::max())) return std::numeric_limits<coord_t>::max();
    if (value<=double(std::numeric_limits<coord_t>::lowest())) return std::numeric_limits<coord_t>::lowest();
    return coord_t(upper?std::ceil(value):std::floor(value));
}

c_point lower_bound(XY p)
{
    return {bounded_coordinate(p.x,false),bounded_coordinate(p.y,false)};
}

c_point upper_bound(XY p)
{
    return {bounded_coordinate(p.x,true),bounded_coordinate(p.y,true)};
}

std::pair<c_point,c_point> curve_bounds(const Curve &curve,double margin)
{
    const XY a=position(curve,0), b=position(curve,1);
    XY min{std::min(a.x,b.x),std::min(a.y,b.y)}, max{std::max(a.x,b.x),std::max(a.y,b.y)};
    if (curve.radius>0) {
        for (int i=0;i<4;++i) {
            const double t=curve.parameter(i*pi/2);
            if (t<0 || t>1) continue;
            const XY p=position(curve,t);
            min.x=std::min(min.x,p.x); min.y=std::min(min.y,p.y);
            max.x=std::max(max.x,p.x); max.y=std::max(max.y,p.y);
        }
    }
    return {lower_bound({min.x-margin,min.y-margin}),upper_bound({max.x+margin,max.y+margin})};
}

bool contains(const Shape &shape, XY p, double margin, bool independent_rings)
{
    // AABB selection is conservative; retain the previous exact boundary test
    // at the floating point, including its scaled-coordinate tolerance.
    const double radius=margin+1e-5;
    const std::vector<size_t> nearby=shape.index.all_lines_in_box(
        lower_bound({p.x-radius,p.y-radius}),upper_bound({p.x+radius,p.y+radius}));
    for (size_t idx:nearby) {
        const Edge &edge=shape.edges[idx];
        const c_point a=edge.a,b=edge.b;
        const double dx=double(b.x)-a.x,dy=double(b.y)-a.y,length2=dx*dx+dy*dy;
        const double t=length2==0?0:std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/length2,0.,1.);
        if (std::hypot(p.x-a.x-t*dx,p.y-a.y-t*dy)<=radius) return true;
    }
    // A horizontal ray also recognizes interiors far from every boundary.
    // Independent paint fragments use per-ring winding, volumes total winding.
    const std::vector<size_t> crossing=shape.index.all_lines_in_box(
        lower_bound(p),upper_bound({double(std::numeric_limits<coord_t>::max()),p.y}));
    int winding=0;
    std::map<size_t,int> rings;
    for (size_t idx:crossing) {
        const Edge &edge=shape.edges[idx];
        const c_point a=edge.a,b=edge.b;
        const double cross=(double(b.x)-a.x)*(p.y-a.y)-(double(b.y)-a.y)*(p.x-a.x);
        int delta=0;
        if (a.y<=p.y && b.y>p.y && cross>0) delta=1;
        if (a.y>p.y && b.y<=p.y && cross<0) delta=-1;
        winding+=delta;
        if (independent_rings && delta!=0) rings[edge.ring]+=delta;
    }
    if (independent_rings) {
        for (const std::pair<const size_t,int> &ring:rings)
            if (ring.second!=0) return true;
        return false;
    }
    return winding!=0;
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
    const std::pair<c_point,c_point> box=curve_bounds(c,margin+1e-5);
    const std::vector<size_t> candidates=shape.index.all_lines_in_box(box.first,box.second);
    for (size_t idx:candidates) {
        const Edge &edge=shape.edges[idx];
        const c_point a=edge.a,b=edge.b;
        const XY n={double(a.y)-b.y,double(b.x)-a.x};
        const double dx=double(b.x)-a.x,dy=double(b.y)-a.y;
        const double norm=std::hypot(n.x,n.y),offset=n.x*a.x+n.y*a.y;
        std::vector<double> events;
        if (norm>0) {
            if (margin>0) {
                cut_line(c,n,offset+margin*norm,events);
                cut_line(c,n,offset-margin*norm,events);
            } else cut_line(c,n,offset,events);
            // Parallel capsule sides terminate at the endpoint normal planes.
            // With zero margin the finite edge itself is the boundary.
            for (double t:events) {
                const XY p=position(c,t);
                const double along=((p.x-a.x)*dx+(p.y-a.y)*dy)/norm;
                if (along>=-1e-5 && along<=norm+1e-5) add_cut(cuts,t);
            }
        }
        if (margin>0) {
            for (int endpoint=0;endpoint<2;++endpoint) {
                if (norm==0 && endpoint==1) continue;
                events.clear();
                const c_point center=endpoint==0?a:b;
                cut_circle(c,{double(center.x),double(center.y)},margin,events);
                for (double t:events) {
                    const XY p=position(c,t);
                    const double along=(p.x-center.x)*dx+(p.y-center.y)*dy;
                    // Only the outward semicircle belongs to the capsule boundary.
                    if (norm==0 || (endpoint==0?along<=1e-5*norm:along>=-1e-5*norm)) add_cut(cuts,t);
                }
            }
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
    if (zones->second.enforced.edges.empty() && zones->second.blocked.edges.empty() && zones->second.volumes.empty()) return result;
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
