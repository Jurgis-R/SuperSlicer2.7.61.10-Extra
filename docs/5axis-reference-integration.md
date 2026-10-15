# SuperSlicer 5X: integration plan

## Scope

The `dev_5x` branch targets genuine five-axis slicing and coordinated five-axis printing. Conventional three-axis hybrid/non-planar experiments are out of scope and are not prerequisites for this branch.

The five-axis implementation must remain opt-in until the complete toolpath, machine transform, motion validation, and postprocessor are integrated and tested. Ordinary three-axis output must remain unchanged when five-axis mode is disabled.

## Current code

- `src/libslic3r/FiveAxisKinematics.hpp`: configurable two-joint orientation solving, rotary-bed point transforms, and rotary-head tool-tip compensation.
- `src/libslic3r/FiveAxisEngine.hpp`: candidate-path checks for finite coordinates, rotary limits, simultaneous-axis policy, rotary change per linear distance, and estimated rotary rate; emits XYZ, two configured rotary words, E, and F only after whole-path validation. `FiveAxisMachineProfile.hpp` now validates transformed XYZ commands against its configured machine envelope, and the extrusion adapter rejects out-of-envelope paths before returning commands.
- `src/libslic3r/FiveAxisSurfaceNormals.hpp`: transforms a source volume mesh into print-object space, samples surface normals, and provides `FiveAxisSurfaceCorrespondence` to choose one candidate source volume for an entire path. It rejects equally plausible volume matches instead of silently mixing normals across overlapping volumes. The sampler now also probes along the nearest face normal and rejects a nearby, opposing face within a configurable thin-wall ambiguity distance (0.25 mm by default). This conservative ray-based check is not a general closest-face enumeration and must still be validated against real meshes.
- `src/libslic3r/FiveAxisBuildDirectionPlanner.hpp`: generates a deterministic full-sphere set of build-up candidates (including six cardinal axes), scores directions using a mesh face-area-weighted overhang proxy and projected build height, and chooses the lowest-scoring valid candidate. Candidate generation and scoring are now automated as a geometry-only first heuristic, not a complete slicer planner.
- `tests/libslic3r/test_five_axis_kinematics.cpp` and `tests/libslic3r/test_five_axis_engine.cpp`: unit coverage for the isolated math and output helpers.

These are helper-level primitives. They are not yet a complete slicer-to-machine pipeline, and output validation is not a geometric collision test. The current slicer merges multiple model-volume slices into region geometry, so a generated perimeter or infill path does not yet retain reliable source-volume provenance. The correspondence helper is deliberately explicit: a caller must supply candidate meshes in the path's coordinate frame, and it only accepts a single source that explains every vertex. It does not yet resolve competing faces inside one mesh (for example, opposite sides of a thin wall), nor does it infer path provenance from merged regions.

## Next integration milestones

1. **Path/source correspondence (new helper, not yet connected to generation)**
   - Keep the source volume ID alongside each candidate transformed mesh.
   - Resolve one candidate for a complete path, requiring all vertices to match within a distance limit.
   - Reject near-tied source-volume matches; do not silently blend normals from overlapping parts.
   - A conservative within-mesh check now probes along the nearest normal and rejects a nearby opposing face within a configurable distance. This catches a common thin-wall ambiguity, but is not a complete competing-face query for arbitrary geometry.
   - Next: preserve provenance during slicing/region construction. `GCodeGenerator::extrude_path_3D()` currently emits XYZ-only moves and ignores `tool_axis_normals`.

2. **Automatic build-direction candidate scoring (initial helper)**
   - Generate a deterministic, approximately uniform full-sphere candidate set, including the six cardinal directions.
   - Score candidate directions by area-weighted overhang proxy and projected part height.
   - Reject invalid directions, malformed triangle indices, degenerate-only meshes, and invalid scoring settings.
   - This score does not yet simulate supports, choose multidirectional regions, check machine reachability, evaluate tool access, or perform collision checks. Machine-aware candidate filtering remains future work.

3. **Machine configuration and coordinate frames**
   - Define rotary axis order, local axis vectors, pivots, zero offsets, sign conventions, tool-center/tool-tip offsets, and joint limits.
   - Explicitly distinguish rotary-bed and rotary-head chains.
   - Validate configurations and reject singular or unreachable poses.

4. **Tool orientation and inverse kinematics**
   - Derive tool orientation from a generated five-axis path, not from layer normals alone.
   - Solve each path pose using the configured machine chain.
   - Track the previous solution to avoid unnecessary rotary flips and discontinuities.
   - Reject solutions outside joint limits or tolerance.

5. **Machine-aware motion planning**
   - Transform path points into machine coordinates and compensate the tool-center point.
   - Check XYZ travel against the configured machine envelope, rotary travel, rotary rate, linear speed, acceleration and extrusion flow.
   - Add conservative swept-tool and machine-envelope collision validation. A path must be rejected if required collision geometry is unavailable.

6. **G-code integration**
   - Connect a dedicated five-axis path representation to the real `GCodeGenerator` pipeline.
   - Keep firmware-specific axis words, coordinate modes, homing/setup commands, and postprocessing in a machine-specific output layer.
   - Preserve normal three-axis G-code when five-axis mode is disabled.
   - Validate the entire candidate path before writing any of its moves.

7. **Verification**
   - Add tests for non-orthogonal joint axes, both rotary-bed and rotary-head chains, pivot compensation, limits, singularities, rotary discontinuities, extrusion continuity, and collision rejection.
   - Add golden G-code regression tests and a Windows 64-bit build.
   - Inspect output in a toolpath preview and use staged, low-risk machine tests before attempting a real part.

## Safety boundary

The current helpers do not prove that the nozzle, tool body, part, fixture, or machine structure cannot collide. They also do not configure or home real hardware. No generated five-axis output should be considered print-ready until the machine-specific transform, complete collision checks, postprocessor, and regression/build tests are implemented and validated.

## External references

External slicers and research projects may inform architecture, but no external implementation should be copied without checking its exact license, file notices, and dependencies. Machine conventions from one project must not be assumed to match another machine.


## Source-volume provenance status

The slice-to-region path now carries five-axis provenance alongside each layer-region's sliced polygons. A source volume is recorded when a non-empty slice remains directly attributable to one printable model volume. When multiple volume contributions are merged into the same output region, the metadata records a mixed source set rather than choosing one ID. If cross-volume gap closing is active, affected slices are marked ambiguous because offset/intersection operations can create or redistribute boundary area.

The metadata is exposed through `LayerRegion::five_axis_source_provenance()` for future orientation/path generation. Five-axis code must accept only the `Unique` state; `Mixed`, `Ambiguous`, and `Unknown` are not permission to guess a source mesh. The source IDs describe volume-level provenance, not a triangle-level proof for every point. Later operations such as multi-material reassignment, curve smoothing, overhang expansion, and path generation still need explicit provenance propagation or conservative invalidation before this metadata can be used to select surface normals. These changes have not been compiled or tested in a full SuperSlicer build.


## Machine-aware build-direction filtering

The build-direction planner now exposes `filter_five_axis_build_direction_candidates()`. It filters already-scored candidates against a validated, explicitly enabled machine profile by planning a caller-supplied, candidate-specific representative pose sequence. The path planner checks rotary inverse-kinematics convergence and limits, output motion constraints, configured AABB obstacles, and the machine XYZ envelope. A failed check invalidates that candidate and records a reason; a disabled or invalid profile fails closed.

This is an integration seam, not automatic geometry-wide accessibility analysis. The caller must still create representative pose sequences for each candidate in the correct coordinate frame. Axis-aligned obstacle boxes and sampled path checks do not prove swept-volume clearance, and the current build-direction score remains a geometric heuristic. The filter does not yet model toolhead mesh geometry, clamps/fixtures unless supplied as obstacle boxes, full-part collision under reorientation, or path generation from the original mesh. The helper is not yet called from the production slicer pipeline.
