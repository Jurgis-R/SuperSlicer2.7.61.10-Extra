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
remain open and retain their bends. These calls do not use or replace the
private Cubic/Lightning clipping implementation.

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

## Limits

The plugin API does not expose a polygon-aware `connect_infill()` substitute:
paths cannot be linked through the surface interior. A pattern therefore
publishes independent sortable paths and lets `STEP_ORDERING` fix their order,
direction and travels.

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
