# Writing an infill pattern plugin

Infill patterns are `INFILL_PATTERN` service plugins. The selected `STEP_INFILL`
generator asks the host to run one pattern plugin per fill surface, and the
plugin writes its extrusion into the output entity it receives. A pattern
plugin only needs the public C ABI and the C++ view helpers; the gyroid pattern
in `src/libslic3r/Plugins/Infill/GyroidPattern.cpp` is the reference
implementation.

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
The boolean recipe options are grouped in `params.infill_flags`, a bitmask of
`c_infill_flag` values (`steps/slic3r_step_infill.h`). This changed the layout of
`raw_infill_pattern_params`, so `steps/slic3r_step_infill.h` moved to MAJOR 2:
packages built against MAJOR 1 must be recompiled against the current headers.


## Declaring membership

The pattern list settings (`fill_pattern`, `top_fill_pattern`, ...) are
plugin-defined enums of plugin ids. A pattern declares the lists it belongs to
during initialization; the host keeps the contribution even when the list is
created later:

```cpp
void inilialize_impl(storage_handle *) const override
{
    key_value_string_pair_t entry = { id_impl(), name_impl() };
    key_value_string_pair_array_t entries = { &entry, 1 };

    raw_option_def_patch patch = raw_option_def_patch_init();
    patch.kind = RAW_OPTION_DEF_PATCH_ADD_ENUM_VALUES;
    patch.opt_key = "fill_pattern";
    patch.enum_values = entries;
    orchestrator_modify_option_def(m_orchestrator, &patch);
}
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
  conversion.

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

slic3r_api::StoredExtrusionEntity path(run_ctx->plugin_storage, points);
path.get_or_add(slic3r_api::EPropertyAttributes::key)
    .extrusion_role(int32_t(params.extrusion_role))
    .mm3_per_mm(params.flow.mm3_per_mm * double(params.flow_mult))
    .width(float(unscaled(params.flow.width) * double(params.flow_mult)))
    .height(float(unscaled(params.flow.height)));
group.append_child_move(path.mutable_view());

slic3r_api::MutableExtrusionEntity output(ctx->output);
output.append_child_move(group.mutable_view());
```

Mark the parent `SORTABLE`/`REVERSIBLE` when the order and direction of the
paths may be changed by `STEP_ORDERING`. A pattern that produces one fixed path
should clear those flags instead.

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
hole or void between the pieces. The gyroid plugin uses it to order its clipped
waves before publication.
