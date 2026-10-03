# Writing an infill pattern plugin

Infill patterns are `INFILL_PATTERN` service plugins. The selected `STEP_INFILL`
generator asks the host to run one pattern plugin per fill surface, and the
plugin writes its extrusion into the output entity it receives. A pattern
plugin only needs the public C ABI and the C++ view helpers; the gyroid pattern
in `src/libslic3r/Plugins/Infill/GyroidPattern.cpp` is the reference
implementation. Honeycomb, 3D honeycomb, Hilbert curve, Archimedean chords and
octagram spiral live in the same directory as separate plugins built on the
shared `PatternPluginHelpers.hpp` utilities.

## Initialization

Before the first run, the generator performs a discovery pass and reports the
work groups of every pattern through `report_pattern_assignments()`. The host
aggregates them and calls `setup()` once per used pattern with a
`run_ctx_infill_pattern_init` payload listing the objects and their groups. Use
it to precompute per-object state (grids, bounding boxes, synchronization); the
payload stays valid until the run for the same object completes.

## Step contract

A pattern plugin is a `PluginBase` subclass with `step_impl()` returning
`INFILL_PATTERN` and `run_impl()` reading `run_ctx_infill_pattern`:

```cpp
const run_ctx_infill_pattern *ctx = plugin_ctx_as_infill_pattern(run_ctx);
if (ctx == nullptr || ctx->surface == nullptr || ctx->params == nullptr || ctx->output == nullptr)
    return;

const raw_infill_pattern_params &params = *ctx->params;
const slic3r_api::Surface surface(ctx->surface);
const slic3r_api::ExPolygon area = surface.expolygon();
```

`params` is the complete per-surface recipe: role, flow, density, spacing,
angle, overlap and gap-fill flags. Distances stored as `coord_t` are scaled
integers; `double` fields are unscaled millimetres unless documented otherwise.
`params.object_bounding_box` is the scaled bounding box of the whole object,
used by patterns that align across layers.

The boolean recipe options are grouped in `params.infill_flags`, a bitmask of
`c_infill_flag` values (`steps/slic3r_step_infill.h`). This changed the layout of
`raw_infill_pattern_params`, so `steps/slic3r_step_infill.h` moved to MAJOR 2:
packages built against MAJOR 1 must be recompiled against the current headers.

Built-in plugins can use the helpers in
`src/libslic3r/Plugins/Infill/PatternPluginHelpers.hpp` instead of repeating the
legacy `Fill` frame handling: `infill_angle()`, `shrink_for_fill()`,
`append_clipped_pieces()`, `order_pieces()`, `append_polyline()`,
`publish_infill_paths()` and `add_self_to_pattern_list()`. The helpers use the
public plugin API and the ordering engine interface, so they are also a readable
reference for external plugins.

## Declaring membership

The pattern list settings (`fill_pattern`, `top_fill_pattern`, ...) are
plugin-defined enums of plugin ids. A pattern declares the lists it belongs to
during initialization; the host keeps the contribution even when the list is
created later. Built-in plugins call `add_self_to_pattern_list()`:

```cpp
void inilialize_impl(storage_handle *) const override
{
    add_self_to_pattern_list(m_orchestrator, k_id, "Honeycomb", { "fill_pattern" });
}
```

The underlying patch is the generic option-definition API:

```cpp
key_value_string_pair_t entry = { id_impl(), name_impl() };
key_value_string_pair_array_t entries = { &entry, 1 };

raw_option_def_patch patch = raw_option_def_patch_init();
patch.kind = RAW_OPTION_DEF_PATCH_ADD_ENUM_VALUES;
patch.opt_key = "fill_pattern";
patch.enum_values = entries;
orchestrator_modify_option_def(m_orchestrator, &patch);
```

Inactive plugins are pruned from the lists during finalization.

## Building geometry

Use the view helpers instead of internal slicer types:

- `slic3r_api::Surface`, `ExPolygon`, `Polygon`, `Polyline` from
  `SurfaceViews.hpp` / `GeometryViews.hpp`;
- `ClipperOperand`, `clipper_offset()`,
  `clipper_intersection_polyline_expolygons()` from `ClipperViews.hpp` for
  clipping and offsetting;
- `scale_i()`, `scale_d()` and `unscaled()` from `slic3r_def.h` for unit
  conversion;
- `slic3r_safe_rand()` from `slic3r_utils.h` for thread-safe inclusive random
  values when a pattern needs a host-seeded random sequence;
- the `c_bounding_box` helpers from `GeometryViews.hpp` (`bounding_box_empty()`,
  `bounding_box_rotate()`, `bounding_box_merge_rotated()`,
  `bounding_box_merge_aligned()`, ...) to build a pattern frame.

A pattern that generates its lines from `box.min` must anchor the lattice to the
world frame, otherwise the pattern shifts with the cross-section of every layer:
call `bounding_box_merge_aligned(box, {module_x, module_y})` before generating,
like the legacy `Fill` implementations did. The honeycomb, 3D honeycomb, gyroid
and plane-path patterns do this; sparse plane-path and gyroid frames stay tight
to the surface, and only their minimum corner is snapped.

`run_ctx->plugin_storage` owns every temporary `Stored*` object. The gyroid
plugin, for example, shrinks the surface by half the line spacing, clips each
generated wave against it and drops the pieces shorter than one extrusion
width.

Clipper ABI 1.1 also provides `clipper_intersection_polylines_expolygons()` and
`clipper_diff_polylines_expolygons()` for a whole `PolylineCollection` against an
`ExPolygonCollection`, with C, C++ and Python wrappers. The host executes the
core operation once for the batch, leaves both inputs unchanged and returns an
independent storage-owned polyline collection. An empty subject produces an
empty result; an empty mask produces an empty intersection or a copy of the
subjects for difference. The C entry points also accept a null mask with those
same semantics and return null for a null storage or subject. Collection order
and correspondence between output and source indices are unspecified; paths
remain open and retain their bends. Cubic and Lightning use batch intersection
once per inset component, then copy the results into their mutable connector
buffers; they do not embed a separate Clipper implementation.

## Publishing extrusion

Wrap `ctx->output` in a `MutableExtrusionEntity` and move a storage-owned child
into it. Set the printable attributes on each leaf:

```cpp
slic3r_api::StoredExtrusionEntity group(run_ctx->plugin_storage);
group.set_flags(group.flags() | RAW_EXTRUSION_FLAG_SORTABLE | RAW_EXTRUSION_FLAG_REVERSIBLE);

slic3r_api::StoredExtrusionEntity path(run_ctx->plugin_storage);
path.set_polyline(piece);   // piece is a Polyline view, transferred without a point copy
path.get_or_add(slic3r_api::EPropertyAttributes::key)
    .extrusion_role(int32_t(params.extrusion_role))
    .mm3_per_mm(params.flow.mm3_per_mm * double(params.flow_mult))
    .width(float(unscaled(params.flow.width) * double(params.flow_mult)))
    .height(float(unscaled(params.flow.height)));
group.append_child_move(path.mutable_view());

slic3r_api::MutableExtrusionEntity output(ctx->output);
output.append_child_move(group.mutable_view());
```

`set_polyline()` borrows the point array from another API polyline view instead
of building a `std::vector<c_point>`, which is what the helpers use to publish a
`StoredPolylineCollection` directly.

Mark the parent `SORTABLE`/`REVERSIBLE` when the order and direction of the
paths may be changed by `STEP_ORDERING`. A pattern that produces one fixed path
should clear those flags instead.

## Volume correction

The host stamps `c_extrusion_property_infill` on the subtree root of every
surface: `source_surface_id` identifies the surface and the other fields carry
its raw fill recipe. `area_to_fill_mm2` is the free-of-perimeter-overlap area
the surface expects when the recipe requests an enforced full fill
(`enforce_full_fill_volume`); it is 0 otherwise. A pattern neither reads nor
sets those fields.

The correction itself is a `STEP_POST_INFILL` service
(`infill.post_process.full_fill_volume`): it rebuilds the volume target and the
legacy spacing/width normalization from the stamped recipe, then scales the
`mm3_per_mm` and `width` of the leaves by the clamped ratio, exactly like the
legacy `Fill::fill_surface_extrusion()` did. A pattern therefore only generates
geometry and publishes `params.flow` scaled by `params.flow_mult`; it does not
implement `fill_exactly` or 100% density handling itself.

The normalizer is a no-op with the default `solid_infill_overlap = 100%`; a
reduced overlap stays an intentional void instead of being refilled by the
correction.

## Rectilinear family port

The family exposes twelve stable plugin ids:
`rectilinear`, `alignedrectilinear`, `monotonic`, `monotoniclines`, `grid`,
`triangles`, `stars`, `cubic`, `scatteredrectilinear`, `sawtooth`,
`rectiwithperimeter` and `supportbase`. All twelve ids share the raster /
multi-sweep code ported from `FillRectilinear.cpp` into
`Plugins/Infill/Rectilinear`; `sawtooth`, `rectiwithperimeter` and `supportbase`
add or replace a stage on top of it:

- `RectilinearFills.cpp` implements the raster / multi-sweep algorithms on
  private work buffers while receiving the boundary as an API `ExPolygon` view
  and the fill recipe as raw plugin parameters.
- `RectilinearConnect.hpp` links the generated lines along the surface boundary
  (`PrusaSimpleConnect` / `FakePerimeterConnect`), so those patterns still
  produce connected paths. It consumes borrowed `BoundaryContours` views and
  materializes only contours that need cleanup or inserted junction points.
- `RectilinearPatterns.cpp` passes the API geometry view, raw recipe, and
  `plugin_storage` directly to the pattern implementation. The monotonic
  patterns publish their group without `SORTABLE` / `REVERSIBLE` so the path
  order and direction survive `STEP_ORDERING`; the other patterns stay
  sortable.
- `rectiwithperimeter` and `supportbase` are registered as `INFILL_PATTERN`
  providers but intentionally do not join any user option list. `supportbase`
  is selected by its stable plugin id for sparse support-base recipes.
  `rectiwithperimeter` is reachable through direct plugin-id selection, while
  normal brim and preset paths continue to use the core `Fill` implementation.
- `sawtooth` reuses the rectilinear XY raster and publishes a Z-profiled
  extrusion tree. Its z-hop offsets use `slic3r_safe_rand()`, and the
  generator sets `C_INFILL_FLAG_DONT_ADJUST` for the sawtooth recipe so later
  flow normalization does not alter the generated profile.

The legacy pattern adapters (`LegacyInfillPatterns.cpp`), brim, support and the
wipe tower still instantiate the legacy core `Fill` classes. Dense infill uses
recipe modifiers and post-infill plugins instead of those core fillers.

## Adaptive and Support Cubic

`adaptivecubic` and `supportcubic` are `INFILL_PATTERN` providers with priorities
15 and 16, respectively. Both belong only to `fill_pattern` and share the
implementation in `Plugins/Infill/AdaptiveCubic`, while retaining separate
per-instance caches and progress state.

Each plugin's `setup()` prepares one owned octree per assigned object, then
publishes the complete cache only after preparation succeeds. Repeated surfaces
for that object share the mode-specific immutable octree. The next `setup()`
starts a new generation and discards the previous caches; object handles are
generation-local lookup keys, not persistent cache identities. There is no
preparatory pipeline step or global cache registry.

Preparation reads model-part meshes, transforms, configuration and surfaces
through public views. Object-wide cell spacing is computed independently for
each mode and remains separate from per-surface flow spacing. It uses unweighted
mean widths and positive densities of configured regions matching that mode and
having nonempty fill surfaces. Width zero uses `1.125f * maximum_nozzle`; percent
widths use the maximum nozzle diameter as their base. An assigned object without
an eligible region fails explicitly, even if its first surface recipe requests
that pattern with positive density; there is no recipe-based spacing fallback.
Existing surfaces carrying `MOD_BRIDGE` contribute triangles at their layer's bottom Z;
`MOD_OVERBRIDGE` alone does not. No new bridge candidates are inferred, and
source surfaces and their metadata are never modified. This differs intentionally
from the legacy pre-bridge candidate filter.

Adaptive mode refines the octree with every model triangle. Support mode uses the
same complete mesh bounds and level spacing, but refines it only with faces whose
normal points upward by more than the legacy 0.707 threshold in octree coordinates.
The volume and object transform handedness are applied to triangle winding before
that test. Marked bridge triangles bypass the normal filter in both modes.

Private glu-libtess triangulates contours with holes without interpreting native
handles. API batch intersection delegates open-path coordinate preparation and
restoration to the host; Boost.Geometry indexes hook collisions. The host uses
its existing filtered determinant predicate with an exact 128-bit fallback.

`InfillWorkBuffers.hpp` owns the shared mutable points, double vectors, bounding
boxes and polylines, including core-compatible rounding and Douglas-Peucker
simplification. `InfillPaths` consumes these neutral buffers directly; polygon
containers, borrowed contour views and rectilinear geometry helpers remain in
the rectilinear module. `InfillPaths` converts between API-owned collections and
the private buffers without reinterpreting their layouts; the copies retain
all points and check cancellation while moving temporary stored payloads.

Surface generation applies the half-spacing inset once per surface, processes
each resulting component independently, builds Adaptive hooks, then chains or
connects paths using the selected connection and anchor settings. Publication
keeps paths sortable and reversible. Parity tests compare complete directed
polylines without requiring their order within the output group to match.
Adaptive parity covers both inputs without additional bridges and projected
bridge surfaces with holes across the connection and anchor settings.

Lightning is the only remaining legacy pattern blocked by missing object-level
preparation. Adaptive and Support Cubic prepare their own caches in `setup()`.

## Limits

The plugin API does not expose a polygon-aware `connect_infill()` substitute:
paths cannot be linked through the surface interior. A pattern therefore
publishes independent sortable paths and lets `STEP_ORDERING` fix their order,
direction and travels. The self-contained connect port used by the rectilinear
family and Adaptive Cubic supplies boundary linking without becoming part of
the public API.

`OrderingEngine.hpp` provides
`connect_infill(StoredPolylineCollection&, const OrderingEngine&)` for the
pattern-side pre-pass. It orders and orients the collection in place with the
supplied engine (for example
`connect_infill(pieces, NearestNeighborOrderingEngine{})`) but it never joins
two polylines. That is a hard requirement: clipping splits a path wherever it
leaves the surface, so a joined path would extrude a straight segment across the
hole or void between the pieces.

Plane-path patterns (`hilbertcurve`, `archimedeanchords`, `octagramspiral`)
reproduce the legacy frame handling in `fill_plane_path_surface()`: they
generate in a rotated grid, use `params.object_bounding_box` to align sparse
infill across layers, then clip and order in the surface frame.
