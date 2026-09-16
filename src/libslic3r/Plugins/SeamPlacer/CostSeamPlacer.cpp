///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "CostSeamPlacer.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

#include "SeamPlacementConstraints.hpp"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/LineDistancer.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/Api/plugin/cpp/SeamPlacerViews.hpp"
#include "libslic3r/Api/plugin/cpp/properties/ExtrusionProperties.hpp"
#include "libslic3r/Plugins/SeamPlacer/SeamCostModel.hpp"

/*
Cost-based seam placement
=========================
The router runs this provider once per assigned PrintRegion batch. run_impl()
uses make_seam_placer_instance() to initialize a session from the PrintingPlan
before publishing it to the extrusion orderer. Preparation follows plan order
and stores candidates in scaled object-local XY by subtracting the instance
shift from scaled platter XY. Effective external-perimeter loops supply
island-wide candidates, and every bound perimeter loop
is a projection target; a loop may have both roles. Painted constraints and
neighboring-region boundaries add endpoints and analytically located transition
points, rounded to scaled integer coordinates and reclassified there.

Initialization filters and snapshots angle, visibility, and overhang costs.
Placement projects those candidates onto the requested loop and combines the
snapshot with the current travel-cost setting. The orderer queries approach
probes, then the actual approach position before rotating a loop to its seam.
Published maps are read-only; the host data read by concurrent placement calls
must not be mutated concurrently. Source changes do not refresh the snapshots.
*/

namespace slic3r_api { namespace SeamPlacement { namespace CostPlugin {

/*
The session has a build phase followed by a read-only placement phase.
m_storage is borrowed scratch storage used only by initialize(); its handle is
cleared after successful preparation, before ABI publication, without freeing
host storage. Region/island handles and loop keys remain borrowed: the plan and
referenced host objects must outlive the session. Candidate vectors and instance
shifts are owned values. Concurrent placement uses local scratch state and never
mutates these maps; this does not freeze host geometry between placement calls.
*/
class CostSession final : public SeamPlacerSession
{
public:
    explicit CostSession(storage_handle *storage) : m_storage(storage) {}
    void initialize(const PrintingPlan &plan, const std::vector<PrintRegion> &regions) override;
    // Cost has no regional preference; the router keeps the first region on equal priorities.
    float get_priority(const ExtrusionEntity &, const LayerRegion &) const override { return 50.f; }
    c_point place_seam(const ExtrusionEntity &loop, const LayerRegion &region,
                       c_point start_position) const override;

private:
    storage_handle *m_storage;
    bool m_initialization_started = false;
    std::set<const print_region_handle *> m_regions;
    std::map<RegionIslandKey, std::vector<SeamCandidate>> m_candidates;
    std::map<const extrusion_entity_handle *, LoopBinding> m_bindings;
};

namespace {

const char *const k_plugin_id = "seam_placer.cost";
const char *const k_group_id = "seam_placer_plugin";
const char *const k_dependencies[] = { nullptr };
const raw_used_config_key k_used_config_keys[] = {
    { "seam_angle_cost", RAW_CO_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "seam_travel_cost", RAW_CO_PERCENT, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE },
    { "seam_visibility", RAW_CO_BOOL, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE }
};

/* Mutable preparation data: point is scaled object-local XY; width, overhang
   and embedded are millimetres. Filtering reduces these records to SeamCandidate. */
struct CandidateWork {
    c_point point{};
    double angle = 0;
    double visibility = 0;
    double overhang = 0;
    double embedded = 0;
    double width = 0;
    bool blocked = false;
    bool enforced = false;
    bool boundary = false;
};

/* Temporary per-island work owns candidate values and the first instance shift,
   but only borrows the island handle. It is discarded after preparation. */
struct IslandWork {
    LayerRegionIsland island;
    c_point shift{};
    bool has_perimeter_loop = false;
    std::vector<CandidateWork> candidates;
};

class CostSeamPlacer final : public PluginBase
{
public:
    static CostSeamPlacer &instance(orchestrator_handle *orchestrator);
    explicit CostSeamPlacer(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}
private:
    const char *id_impl() const noexcept override { return k_plugin_id; }
    const char *name_impl() const noexcept override { return "Cost-based"; }
    const char *description_impl() const noexcept override { return "Balances seam visibility, corner shape, overhang quality and travel."; }
    const char *exclusive_group_impl() const noexcept override { return k_group_id; }
    const char *exclusive_group_label_impl() const noexcept override { return "Seam placement"; }
    const char *exclusive_group_tooltip_impl() const noexcept override { return "Selects the service used to choose the starting point of closed extrusion loops."; }
    slicing_step_t step_impl() const noexcept override { return SEAM_PLACER; }
    const char *const *dependencies_impl() const noexcept override { return k_dependencies; }
    int32_t priority_impl() const noexcept override { return 900; }
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override;
    int32_t defined_config_keys(const char **) const noexcept override { return 0; }
    void inilialize_impl(storage_handle *) const override {}
    void run_impl(const plugin_run_context *run_context) const override;
};

/* Return scaled distance; only travel scoring converts this result to millimetres. */
double point_distance(c_point a, c_point b)
{
    return std::hypot(double(a.x) - b.x, double(a.y) - b.y);
}

/*
Apply this node's extrusion attributes to the values inherited from its parent.
When attributes exist, their role overrides the ancestor; a non-positive width
leaves the inherited width unchanged so the caller can still apply its flow fallback.
*/
void update_attributes(const ExtrusionEntity &entity, raw_extrusion_role &role, double &width)
{
    const c_extrusion_property_attributes *attributes = static_cast<const c_extrusion_property_attributes *>(
        entity.property_data(EXTRUSION_PROPERTY_TYPE_ATTRIBUTES));
    if (attributes == nullptr) return;
    role = attributes->role;
    if (attributes->width > 0) width = attributes->width;
}

/* Keep discovery order and distinct attributes even for equal points. Deduplicate
   only after filtering and static scoring, when the lowest surviving cost is known. */
void add_candidate(std::vector<CandidateWork> &out, CandidateWork candidate)
{
    out.push_back(candidate);
}

struct ExternalGeometry {
    bool present = false;
    double width = 0;
};

/*
Walk an extrusion subtree while accumulating perimeter, role, and width
properties. Every non-empty perimeter loop is bound to its source island and
instance shift. It also supplies candidates when its subtree contains segments
with an effective external-perimeter role and perimeter marking. The node's own
external geometry supplies the width first; otherwise the first qualifying child
does. All children are visited, and source loops are appended after their children.
*/
ExternalGeometry collect_tree(
    const ExtrusionEntity &entity, RegionIslandKey island, c_point shift,
    raw_extrusion_role role, double width, bool perimeter,
    std::map<const extrusion_entity_handle *, LoopBinding> &bindings,
    std::vector<std::pair<ExtrusionEntity, double>> &sources,
    bool &has_perimeter_loop)
{
    update_attributes(entity, role, width);
    if (entity.property_data(EXTRUSION_PROPERTY_TYPE_PERIMETER) != nullptr) perimeter = true;
    ExternalGeometry external;
    if (perimeter && entity.segment_count() != 0 &&
        RAW_EXTRUSION_ROLE_IS_EXTERNAL_PERIMETER(role)) {
        external.present = true;
        external.width = width;
    }
    for (uint32_t index = 0; index < entity.child_count(); ++index) {
        const ExternalGeometry child = collect_tree(
            entity.child(index), island, shift, role, width, perimeter, bindings, sources,
            has_perimeter_loop);
        if (!external.present && child.present) external = child;
    }
    if (entity.is_loop() && !entity.empty() && perimeter) {
        bindings.emplace(entity.handle(), LoopBinding{island, shift});
        has_perimeter_loop = true;
        if (external.present) sources.push_back({entity, external.width});
    }
    return external;
}

/* Diagnose missing source-island metadata without binding loops or creating candidates. */
bool contains_perimeter_loop(const ExtrusionEntity &entity, bool perimeter = false)
{
    if (entity.property_data(EXTRUSION_PROPERTY_TYPE_PERIMETER) != nullptr) perimeter = true;
    if (perimeter && entity.is_loop() && !entity.empty()) return true;
    for (uint32_t index = 0; index < entity.child_count(); ++index)
        if (contains_perimeter_loop(entity.child(index), perimeter)) return true;
    return false;
}

/*
Project a scaled platter-coordinate target onto all line and arc segments in
tree order. A node without segments and with one point supplies a degenerate
fallback; its children are still visited. Strict comparison keeps the first
projection on ties. If no geometry is visited, the distance remains infinite.
*/
std::pair<c_point, double> project_onto_loop(const ExtrusionEntity &entity, c_point target)
{
    c_point best{};
    double best_distance = std::numeric_limits<double>::infinity();
    std::function<void(const ExtrusionEntity &)> visit = [&](const ExtrusionEntity &node) {
        for (uint32_t index = 0; index < node.segment_count(); ++index) {
            const SeamPlacementConstraints::Curve curve = SeamPlacementConstraints::Curve::from_segment(node.segment(index));
            const c_point point = curve.point(curve.closest_parameter(target));
            const double distance = point_distance(point, target);
            if (distance < best_distance) { best = point; best_distance = distance; }
        }
        if (node.segment_count() == 0 && node.point_count() == 1) {
            const double distance = point_distance(node.point(0), target);
            if (distance < best_distance) { best = node.point(0); best_distance = distance; }
        }
        for (uint32_t index = 0; index < node.child_count(); ++index) visit(node.child(index));
    };
    visit(entity);
    return {best, best_distance};
}

/* Find the containing island by scanning plugin view handles; no match means
   no boundary zone can be established. */
LayerIsland parent_island(const Layer &layer, const LayerRegionIsland &region_island)
{
    for (uint32_t island_index = 0; island_index < layer.island_count(); ++island_index) {
        const LayerIsland candidate = layer.island(island_index);
        for (uint32_t region_island_index = 0;
             region_island_index < candidate.region_island_count(); ++region_island_index) {
            const LayerRegionIsland child = candidate.region_island(region_island_index);
            if (child.handle() == region_island.handle()) return candidate;
        }
    }
    return LayerIsland();
}

/*
Test object-local scaled XY against surfaces of regions belonging to other
LayerRegionIslands in the same LayerSliceIsland. Containment or edge distance
within tolerance counts as a boundary. Callers convert half the effective width
from millimetres to scaled units; it need not equal the regional fallback width.
No source region or parent island means no boundary can be established.
*/
bool is_boundary_point(const LayerRegionIsland &source, c_point point, double tolerance)
{
    if (source.region_count() == 0) return false;
    const Layer layer = source.region(0).layer();
    const LayerIsland parent = parent_island(layer, source);
    if (!parent.valid()) return false;
    for (uint32_t island_index = 0; island_index < parent.region_island_count(); ++island_index) {
        const LayerRegionIsland other = parent.region_island(island_index);
        if (other.handle() == source.handle()) continue;
        for (uint32_t region_index = 0; region_index < other.region_count(); ++region_index) {
            const ExPolygonCollection slices = other.region(region_index).slices();
            const LineDistancer boundaries(slices);
            if (boundaries.distance_from_lines(point) <= tolerance ||
                boundaries.distance_from_lines(point, true) < 0)
                return true;
        }
    }
    return false;
}

/* Own offset copies of the neighboring regions' complete surfaces, without
   modifying or clipping the host slices to individual islands. Membership in
   any copied polygon counts as membership in the union in inside_boundary_zone(). */
StoredExPolygonCollection boundary_zone(storage_handle *storage,
                                        const LayerRegionIsland &source,
                                        double tolerance)
{
    StoredExPolygonCollection geometry(storage);
    if (source.region_count() == 0) return geometry;
    const Layer layer = source.region(0).layer();
    const LayerIsland parent = parent_island(layer, source);
    if (!parent.valid()) return geometry;
    const ClipperContext clip(storage);
    for (uint32_t island_index = 0; island_index < parent.region_island_count(); ++island_index) {
        const LayerRegionIsland other = parent.region_island(island_index);
        if (other.handle() == source.handle()) continue;
        for (uint32_t region_index = 0; region_index < other.region_count(); ++region_index) {
            const ExPolygonCollection slices = other.region(region_index).slices();
            for (const ExPolygon polygon : slices) {
                ClipperOperand subject = clip(polygon);
                ClipperOperand expanded = clipper_offset(subject, tolerance);
                StoredExPolygonCollection expanded_geometry = expanded.to_expolygon_collection();
                geometry.append_move_from(std::move(expanded_geometry));
            }
        }
    }
    return geometry;
}

bool inside_boundary_zone(const StoredExPolygonCollection &zone, c_point point)
{
    for (const ExPolygon polygon : zone)
        if (polygon.contains(point)) return true;
    return false;
}

/*
Use a boundary zone offset by half the effective width, converted from mm to
scaled polygon units. Each source line or arc is converted from platter to
object coordinates, cut against contours and holes, and sampled on
both sides of each cut with rounded points and a bounded parameter step. A change
in sampled membership makes a candidate; this is not an exhaustive exact test.
The rounded point is then converted back to platter coordinates for painted-
constraint classification with a full-width scaled margin. Blockers take priority
over enforcers in that classification; the stored point remains object-local.
*/
void add_boundary_candidates(storage_handle *storage,
                             const SeamPlacementConstraints &constraints,
                             const LayerRegionIsland &island,
                             const ExtrusionEntity &loop, c_point shift,
                             double width, std::vector<CandidateWork> &out)
{
    const StoredExPolygonCollection zone = boundary_zone(storage, island, scale_d(width / 2));
    if (zone.empty()) return;
    std::function<void(const ExtrusionEntity &)> visit;
    visit = [&](const ExtrusionEntity &node) {
        for (uint32_t segment_index = 0; segment_index < node.segment_count(); ++segment_index) {
            c_extrusion_segment segment = node.segment(segment_index);
            segment.point_a.x -= shift.x; segment.point_a.y -= shift.y;
            segment.point_b.x -= shift.x; segment.point_b.y -= shift.y;
            const SeamPlacementConstraints::Curve curve = SeamPlacementConstraints::Curve::from_segment(segment);
            std::vector<double> cuts;
            for (const ExPolygon polygon : zone) {
                const std::function<void(const Polygon &)> intersect = [&](const Polygon &edge_loop) {
                    for (size_t edge_index = 0; edge_index < edge_loop.size(); ++edge_index) {
                        c_extrusion_segment edge{};
                        edge.point_a = edge_loop[edge_index];
                        edge.point_b = edge_loop[(edge_index + 1) % edge_loop.size()];
                        curve.intersection_cuts(SeamPlacementConstraints::Curve::from_segment(edge), cuts);
                    }
                };
                intersect(polygon.contour());
                for (uint32_t hole_index = 0; hole_index < polygon.hole_size(); ++hole_index)
                    intersect(polygon.hole(hole_index));
            }
            std::sort(cuts.begin(), cuts.end());
            cuts.erase(std::unique(cuts.begin(), cuts.end(), [](double a, double b) { return std::abs(a - b) < 1e-12; }), cuts.end());
            const double epsilon = curve.length() > 0 ? std::min(1e-4, 1.0 / curve.length()) : 0;
            for (const double cut : cuts) {
                if (cut < 0 || cut > 1) continue;
                const bool before = inside_boundary_zone(zone, curve.point(std::max(0.0, cut - epsilon)));
                const bool after = inside_boundary_zone(zone, curve.point(std::min(1.0, cut + epsilon)));
                if (before == after) continue;
                CandidateWork candidate;
                candidate.point = curve.point(cut);
                candidate.width = width;
                candidate.boundary = true;
                const c_point platter = {candidate.point.x + shift.x, candidate.point.y + shift.y};
                const SeamPlacementConstraints::Status status = constraints.classify_point(loop, platter, scale_d(width));
                candidate.blocked = status == SeamPlacementConstraints::Status::Blocked;
                candidate.enforced = status == SeamPlacementConstraints::Status::Enforced;
                add_candidate(out, candidate);
            }
        }
        for (uint32_t child_index = 0; child_index < node.child_count(); ++child_index)
            visit(node.child(child_index));
    };
    visit(loop);
}

/*
Publish both ends of every painted-constraint portion, including ordinary
endpoints emitted for unconstrained portions. Curve evaluation rounds analytic
positions to scaled integer platter coordinates; classification is repeated at
that published point before the candidate is shifted to object coordinates.
*/
void add_constraint_candidates(const SeamPlacementConstraints &constraints,
                               const ExtrusionEntity &loop, const LayerRegion &region,
                               c_point shift, double width, std::vector<CandidateWork> &out)
{
    const SeamPlacementConstraints::Result portions = constraints.portions(loop, region, true);
    for (const SeamPlacementConstraints::Portion &portion : portions.portions) {
        for (const double parameter : {portion.begin, portion.end}) {
            const c_point platter = portion.curve.point(parameter);
            const SeamPlacementConstraints::Status status = constraints.classify_point(loop, platter, portion.paint_margin);
            CandidateWork candidate;
            candidate.point = {platter.x - shift.x, platter.y - shift.y};
            candidate.width = width;
            candidate.blocked = status == SeamPlacementConstraints::Status::Blocked;
            candidate.enforced = status == SeamPlacementConstraints::Status::Enforced;
            add_candidate(out, candidate);
        }
    }
}

/*
Attach legacy layer heuristics before filtering. Points remain scaled; width and
signed distances are in millimetres. Overhang is the non-negative part of the
previous layer's signed distance, or zero without a previous-layer distancer.
When multiple regional islands have non-empty perimeter extrusions, embedding
uses the current layer's signed distance plus 0.6 * width. Boundary membership
adds to any existing boundary flag, using half-width converted to scaled units.
*/
void calculate_layer_attributes(const LayerRegionIsland &island,
                                std::vector<CandidateWork> &candidates)
{
    if (island.region_count() == 0) return;
    const Layer layer = island.region(0).layer();
    const LineDistancer current(layer.slices());
    size_t islands_with_perimeters = 0;
    for (uint32_t island_index = 0; island_index < layer.island_count(); ++island_index) {
        const LayerIsland layer_island = layer.island(island_index);
        for (uint32_t region_island_index = 0;
             region_island_index < layer_island.region_island_count(); ++region_island_index) {
            const LayerRegionIsland region_island = layer_island.region_island(region_island_index);
            if (region_island.has_extrusion(RAW_EXTRUSION_ROLE_PERIMETER) &&
                !ExtrusionEntity(region_island.extrusion(RAW_EXTRUSION_ROLE_PERIMETER)).empty())
                ++islands_with_perimeters;
        }
    }
    LineDistancer previous;
    bool has_previous = false;
    const Object object = layer.object();
    for (uint32_t index = 0; index < object.layer_count(); ++index) {
        if (object.layer(index).handle() != layer.handle()) continue;
        if (index != 0) {
            previous = LineDistancer(object.layer(index - 1).slices());
            has_previous = true;
        }
        break;
    }
    for (CandidateWork &candidate : candidates) {
        if (has_previous)
            candidate.overhang = std::max(0.0, unscaled(previous.distance_from_lines(candidate.point, true)));
        if (islands_with_perimeters > 1)
            candidate.embedded = unscaled(current.distance_from_lines(candidate.point, true)) + 0.6 * candidate.width;
        candidate.boundary = candidate.boundary ||
                             is_boundary_point(island, candidate.point, scale_d(candidate.width / 2));
    }
}

/*
Apply preferences to the survivors of each preceding stage, preserving their order:
1. Remove blocked points; restore the original set if none survive.
2. If any enforced point remains, keep only enforced points.
3. If any boundary point remains, keep only boundary points.
4. If any overhang is zero, remove overhangs greater than width / 4 (mm).
5. If any embedded value is below -0.5 mm, remove values above -0.5 mm.
Small overhangs and the exact embedded value -0.5 can therefore survive. The
all-blocked fallback permits a seam even when every source candidate is blocked;
reordering these passes changes the policy and can change the final winner.
*/
void filter_candidates(std::vector<CandidateWork> &candidates)
{
    const std::vector<CandidateWork> original = candidates;
    candidates.erase(std::remove_if(candidates.begin(), candidates.end(), [](const CandidateWork &c) { return c.blocked; }), candidates.end());
    if (candidates.empty()) candidates = original;
    const bool enforced = std::any_of(candidates.begin(), candidates.end(), [](const CandidateWork &c) { return c.enforced; });
    if (enforced) candidates.erase(std::remove_if(candidates.begin(), candidates.end(), [](const CandidateWork &c) { return !c.enforced; }), candidates.end());
    const bool boundary = std::any_of(candidates.begin(), candidates.end(), [](const CandidateWork &c) { return c.boundary; });
    if (boundary) candidates.erase(std::remove_if(candidates.begin(), candidates.end(), [](const CandidateWork &c) { return !c.boundary; }), candidates.end());
    const bool supported = std::any_of(candidates.begin(), candidates.end(), [](const CandidateWork &c) { return c.overhang == 0; });
    if (supported) candidates.erase(std::remove_if(candidates.begin(), candidates.end(), [](const CandidateWork &c) { return c.overhang > c.width / 4; }), candidates.end());
    const bool embedded = std::any_of(candidates.begin(), candidates.end(), [](const CandidateWork &c) { return c.embedded < -0.5; });
    if (embedded) candidates.erase(std::remove_if(candidates.begin(), candidates.end(), [](const CandidateWork &c) { return c.embedded > -0.5; }), candidates.end());
}

} // namespace

/*
Prepare once, including failed attempts: the guard is set before input validation
and a failure does not roll back this session's partially built state. The normal
make_seam_placer_instance() path destroys it on failure and publishes only after
success. Constraint caches copy their polygons; they and the mutable island work
are local temporaries, not part of the published session.
*/
void CostSession::initialize(const PrintingPlan &plan, const std::vector<PrintRegion> &regions)
{
    if (m_initialization_started)
        throw std::logic_error(
            "Cost-based seam placement sessions may only be initialized once.");
    m_initialization_started = true;
    if (!plan.valid() || m_storage == nullptr)
        throw std::invalid_argument("Cost-based seam placement needs a valid plan and temporary storage.");
    SeamPlacementConstraints constraints;
    constraints.initialize_for_regions(plan, m_storage, regions);
    std::map<RegionIslandKey, IslandWork> work;
    for (const PrintRegion &region : regions)
        if (region.valid()) m_regions.insert(region.handle());

    /*
    Traverse the plan's group/layer/tool/extrusion order. Invalid island metadata
    is tolerated only when the extrusion has no perimeter loop, and an island is
    prepared when any of its PrintRegions belongs to this provider session.
    */
    for (uint32_t group_index = 0; group_index < plan.group_count(); ++group_index) {
        const PrintingGroup group = plan.group(group_index);
        for (uint32_t layer_index = 0; layer_index < group.layer_group_count(); ++layer_index) {
            const PrintingLayerGroup layer_group = group.layer_group(layer_index);
            for (uint32_t tool_index = 0; tool_index < layer_group.tool_group_count(); ++tool_index) {
                const PrintingToolGroup tool = layer_group.tool_group(tool_index);
                for (uint32_t extrusion_index = 0; extrusion_index < tool.extrusion_count(); ++extrusion_index) {
                    const PrintingExtrusion extrusion = tool.extrusion(extrusion_index);
                    const LayerRegionIsland island = extrusion.region_island();
                    if (!island.valid() || island.region_count() == 0) {
                        if (contains_perimeter_loop(extrusion.root()))
                            throw std::runtime_error(
                                "Cost-based seam placement found a perimeter loop with an invalid source LayerRegionIsland.");
                        continue;
                    }
                    bool assigned = false;
                    for (uint32_t region_index = 0; region_index < island.region_count(); ++region_index) {
                        const PrintRegion print_region = island.region(region_index).print_region();
                        if (print_region.valid() && m_regions.count(print_region.handle()) != 0) {
                            assigned = true;
                            break;
                        }
                    }
                    if (!assigned) continue;
                    /*
                    The first occurrence fixes this island's preparation shift.
                    All occurrences still bind their loops and copied shifts, but
                    only occurrences with that same shift contribute candidates.
                    Different shifts reuse the shared object-local set, assuming
                    equivalent source geometry after removing the translation.
                    */
                    const c_point shift = plan.object_instance_shift(extrusion);
                    const RegionIslandKey key = island.handle();
                    const bool first = work.find(key) == work.end();
                    IslandWork &island_work = work[key];
                    if (first) { island_work.island = island; island_work.shift = shift; }
                    std::vector<std::pair<ExtrusionEntity, double>> sources;
                    bool has_perimeter_loop = false;
                    collect_tree(extrusion.root(), key, shift, extrusion.role(), 0, false, m_bindings, sources,
                                 has_perimeter_loop);
                    if (shift.x != island_work.shift.x || shift.y != island_work.shift.y) continue;
                    island_work.has_perimeter_loop = island_work.has_perimeter_loop || has_perimeter_loop;
                    // Assignment checks all regions, but region(0) supplies the
                    // fallback flow and external-perimeter nozzle parameters.
                    const LayerRegion region = island.region(0);
                    const c_flow external_flow = region.flow(RAW_EXTRUSION_ROLE_EXTERNAL_PERIMETER);
                    const double fallback_width = unscaled(external_flow.width);
                    for (const std::pair<ExtrusionEntity, double> &source : sources) {
                        const ExtrusionEntity loop = source.first;
                        const double width = source.second > 0 ? source.second : fallback_width;
                        // Keep non-repeated consecutive vertices for the legacy
                        // angle model; constraint and boundary cuts add further
                        // candidates, whose angle retains its default value.
                        const std::vector<c_point> path = loop.collect_points();
                        std::vector<c_point> points;
                        points.reserve(path.size());
                        for (const c_point point : path) {
                            if (points.empty() || points.back().x != point.x || points.back().y != point.y)
                                points.push_back(point);
                        }
                        const std::vector<float> angles = Slic3r::SeamCostModel::calculate_cost_angles(
                            points, float(unscaled(external_flow.nozzle_diameter)));
                        for (size_t point_index = 0; point_index < points.size(); ++point_index) {
                            CandidateWork candidate;
                            const c_point point = points[point_index];
                            candidate.point = {point.x - shift.x, point.y - shift.y};
                            candidate.angle = angles[point_index];
                            candidate.width = width;
                            const SeamPlacementConstraints::Status status = constraints.classify_point(loop, point, scale_d(width));
                            candidate.blocked = status == SeamPlacementConstraints::Status::Blocked;
                            candidate.enforced = status == SeamPlacementConstraints::Status::Enforced;
                            add_candidate(island_work.candidates, candidate);
                        }
                        add_constraint_candidates(constraints, loop, region, shift, width, island_work.candidates);
                        add_boundary_candidates(m_storage, constraints, island_work.island, loop, shift, width,
                                                island_work.candidates);
                    }
                }
            }
        }
    }

    /*
    Finalize each island once: enrich and filter the temporary CandidateWork
    records, snapshot static costs, then deduplicate equal integer points. Equal
    points retain discovery position and the lowest static cost. After this pass
    only compact owned SeamCandidate values are published.
    */
    for (auto &[key, island_work] : work) {
        if (!island_work.has_perimeter_loop) continue;
        if (island_work.candidates.empty())
            throw std::runtime_error(
                "Cost-based seam placement found perimeter loops but no external geometric candidate for LayerRegionIsland.");
        calculate_layer_attributes(island_work.island, island_work.candidates);
        filter_candidates(island_work.candidates);
        const Layer layer = island_work.island.region(0).layer();
        const Object object = layer.object();
        const double angle_weight = object.config().effective_float_or_percent_or_default(
            "seam_angle_cost", 1.0, 0.0);
        const double visibility_weight = object.config().bool_or_default("seam_visibility", false) ? 1.0 : 0.0;
        // Visibility samples use object-local XY and the legacy slice-Z convention
        // in millimetres; the resulting values are captured once, after filtering.
        if (visibility_weight != 0) {
            std::vector<c_vec3f> points;
            points.reserve(island_work.candidates.size());
            for (const CandidateWork &candidate : island_work.candidates)
                points.push_back({float(unscaled(candidate.point.x)), float(unscaled(candidate.point.y)),
                                  float(unscaled(layer.slice_z()))});
            const std::vector<float> visibility = Slic3r::SeamCostModel::calculate_cost_visibility(object, points);
            for (size_t index = 0; index < visibility.size(); ++index)
                island_work.candidates[index].visibility = visibility[index];
        }
        // The static score combines overhang, optional visibility and weighted
        // legacy angle penalty. Travel is deferred until the approach is known.
        std::vector<SeamCandidate> prepared;
        prepared.reserve(island_work.candidates.size());
        for (const CandidateWork &candidate : island_work.candidates) {
            const double overhang_penalty = candidate.overhang > 0 && candidate.width > 0 ?
                std::clamp(2 * (candidate.overhang - candidate.width / 8) / candidate.width, 0.0, 3.0) : 0.0;
            const double static_cost = overhang_penalty + visibility_weight * candidate.visibility +
                                       angle_weight * Slic3r::SeamCostModel::compute_angle_penalty(float(candidate.angle));
            std::vector<SeamCandidate>::iterator duplicate = std::find_if(
                prepared.begin(), prepared.end(), [&](const SeamCandidate &entry) {
                    return entry.point.x == candidate.point.x && entry.point.y == candidate.point.y;
                });
            if (duplicate == prepared.end()) prepared.push_back({candidate.point, static_cost});
            else duplicate->static_cost = std::min(duplicate->static_cost, static_cost);
        }
        m_candidates.emplace(key, std::move(prepared));
    }
    m_storage = nullptr;
}

c_point CostSession::place_seam(const ExtrusionEntity &loop, const LayerRegion &region,
                                c_point start_position) const
{
    /*
    The router resolves a compatible loop/region pair before this call. These
    checks validate region assignment and loop binding separately; they do not
    fully validate that relationship for direct callers. Missing candidates are
    an error, not an implicit fallback to another provider.
    */
    if (!loop.is_loop() || loop.empty())
        throw std::invalid_argument("Cost-based seam placement needs a non-empty closed loop.");
    if (!region.valid()) throw std::invalid_argument("Cost-based seam placement needs a valid LayerRegion.");
    const PrintRegion print_region = region.print_region();
    if (!print_region.valid() || m_regions.count(print_region.handle()) == 0)
        throw std::invalid_argument("Cost seam LayerRegion was not assigned to this provider session.");
    const std::map<const extrusion_entity_handle *, LoopBinding>::const_iterator binding =
        m_bindings.find(loop.handle());
    if (binding == m_bindings.end()) throw std::invalid_argument("Cost seam loop was not present in the initialized plan.");
    const std::map<RegionIslandKey, std::vector<SeamCandidate>>::const_iterator candidates =
        m_candidates.find(binding->second.island);
    if (candidates == m_candidates.end() || candidates->second.empty())
        throw std::runtime_error("Cost seam source island has no candidates.");
    const Object object = region.layer().object();
    // Unlike static costs, this weight is read per request from borrowed host
    // configuration, which must not change concurrently with these reads.
    const double travel_weight = object.config().effective_float_or_percent_or_default(
        "seam_travel_cost", 1.0, 0.0);
    c_point best{};
    double best_cost = std::numeric_limits<double>::infinity();
    /*
    Reapply the loop instance shift to each object-coordinate candidate, project
    it onto the requested loop in scaled platter XY, and measure distance to the
    scaled start_position (an approach probe or the actual approach). Convert that
    distance to mm before the Gaussian travel penalty, then add the weighted score,
    not a length. Strict comparison preserves the first winner on equal total cost.
    The projected point is not reclassified against paint or regional boundaries;
    those preferences describe the source candidate, not necessarily its projection.
    */
    for (const SeamCandidate &candidate : candidates->second) {
        const c_point target = {candidate.point.x + binding->second.instance_shift.x,
                                candidate.point.y + binding->second.instance_shift.y};
        const std::pair<c_point, double> projection = project_onto_loop(loop, target);
        const double distance_mm = unscaled(point_distance(projection.first, start_position));
        const double cost = candidate.static_cost + travel_weight *
            (1.0 - Slic3r::SeamCostModel::gauss(float(distance_mm), 0.f, 1.f, 0.005f));
        if (cost < best_cost) { best = projection.first; best_cost = cost; }
    }
    return best;
}

namespace {

CostSeamPlacer &CostSeamPlacer::instance(orchestrator_handle *orchestrator)
{
    static CostSeamPlacer plugin(orchestrator);
    return plugin;
}

int32_t CostSeamPlacer::used_config_keys(raw_used_config_key *keys) const noexcept
{
    if (keys != nullptr)
        for (size_t index = 0; index < std::size(k_used_config_keys); ++index) keys[index] = k_used_config_keys[index];
    return int32_t(std::size(k_used_config_keys));
}

void CostSeamPlacer::run_impl(const plugin_run_context *run_context) const
{
    /*
    Validate the raw C context and copy non-owning region wrappers. The factory
    creates a session; make_seam_placer_instance() initializes it, then transfers
    its unique_ptr into the ABI holder. The router's SeamPlacer wrapper consumes
    that instance and eventually invokes its destroy callback. The context only
    carries the output; neither it nor the session owns the referenced host graph.
    Placement callbacks translate exceptions into C error results; the consuming
    C++ wrapper raises errors and checks the returned point against the loop path.
    */
    run_ctx_seam_placer *context = plugin_ctx_as_seam_placer(run_context);
    if (context == nullptr || context->plan == nullptr || (context->region_count != 0 && context->regions == nullptr))
        throw std::invalid_argument("Cost-based seam placement received an invalid run context.");
    std::vector<PrintRegion> regions;
    for (uint32_t index = 0; index < context->region_count; ++index) regions.emplace_back(context->regions[index]);
    context->instance = make_seam_placer_instance(make_cost_seam_session(run_context->plugin_storage),
                                                  PrintingPlan(context->plan), regions);
}

} // namespace

std::unique_ptr<SeamPlacerSession> make_cost_seam_session(storage_handle *storage)
{
    return std::unique_ptr<SeamPlacerSession>(new CostSession(storage));
}

void register_cost_seam_placer_plugin(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(orchestrator, CostSeamPlacer::instance(orchestrator).c_instance());
}

}}} // namespace slic3r_api::SeamPlacement::CostPlugin
