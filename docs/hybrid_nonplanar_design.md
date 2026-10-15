# Hybrid non-planar printing: implementation plan

## Status

This document tracks the implementation in the SuperSlicer EXTRA development branch. The feature is still experimental, has not been compiled in this environment, and has not been validated on a printer.

### Implemented in the development branch

- Added a **Hybrid non-planar printing** checkbox, mutually exclusive with ZAA during G-code generation.
- Added configurable interval `H` (default 2.0 mm), maximum slope angle (default 30 degrees), and maximum Z offset (default 0.4 mm).
- Added a conservative non-planar speed cap (default 20 mm/s) for active deformed sections.
- Added an initial **Mode B** path-deformation implementation: on the existing layer whose vertical span crosses each H boundary, eligible perimeter and top-solid paths are sampled against the model mesh and receive bounded Z offsets.
- Added an adjacent-sample slope limiter and ramps the path offsets back to nominal layer Z at both ends.
- Corrected the existing `ExtrusionPath3D` G-code writer to use each destination vertex's Z offset and true 3D segment length.

### Scheduler foundation added (not yet connected to G-code emission)

- Added the header-only `HybridNonplanarScheduler` in `src/libslic3r/HybridNonplanarScheduler.hpp`. It tracks a configurable positive interval, emits each due Z boundary at most once, tolerates floating-point boundary error, handles variable-height layers, and rejects invalid/non-monotonic layer input.
- Connected the scheduler to `GCodeGenerator`: it resets for each export, advances once for each processed planar layer, and Mode B now checks the scheduler's boundary result instead of recomputing an interval band with `floor()` per extrusion path.
- Added Catch2 coverage in `tests/libslic3r/test_hybrid_nonplanar_scheduler.cpp` and registered it in `tests/libslic3r/CMakeLists.txt`.
- The scheduler is now called by `GCodeGenerator` and controls Mode B eligibility, but it does not yet insert a separate finishing pass. Mode B still emits its contoured segments inline while the planar layer is generated; typed deferred-path queuing and safe separate-pass emission remain outstanding.
- The C++ test target has not been compiled or run in this environment. Treat these tests as authored, not passed.

### Section-boundary planner foundation (not yet wired into G-code)

- Added `HybridNonplanarSectionPlanner.hpp`, a layer-group boundary tracker that describes a completed planar H section without retaining/replaying old extrusion paths.
- The tracker is designed to be called once after each complete print-Z layer group, after all objects and tools at that height have finished. It records section start Z, nominal H boundary, actual boundary-layer top Z, and the number of planar layers in the section.
- It fails closed for non-monotonic layer groups, invalid dimensions, or a single indivisible layer spanning multiple H boundaries. That multi-boundary case needs explicit subdivision/planning rather than guessed grouping.
- Added Catch2 tests for normal boundaries, variable-height boundary layers, reset, and invalid/ambiguous input.
- This is a testable scheduling primitive, not yet the section-level surface-path generator. It is not yet connected to `GCodeGenerator::process_layer`; the existing per-object deferred queue and its limitations remain. Do not treat this commit as completing the finishing pass.

### Deferred Mode B finishing-pass implementation (not yet built)

- Mode B now converts an eligible contour into a typed `ExtrusionPath3D` and queues it with its layer pointer, active `FullPrintConfig`, object origin, description, requested speed, and extruder ID.
- The original planar extrusion for that selected path is skipped, so the same selected path is not deposited once planar and again as a duplicate contour. The queue is now flushed at the outer print-Z layer-group boundary, after all ordinary object/tool work at that Z, using the normal travel planner and typed 3D extrusion writer. A layer/extruder context mismatch aborts export instead of silently dropping the queued path. This improves ordering within one layer group but does not make the queue a whole-H-section surface planner.
- The finishing pass is marked in G-code with `HYBRID_PLANAR_SECTION_END`, `HYBRID_FINISHING_PASS_BEGIN`, and `HYBRID_FINISHING_PASS_END`; these markers support output-order regression checks.
- Added a G-code integration test in `tests/fff_print/test_printgcode.cpp` using the sloped pyramid test mesh. It checks marker order and that the finishing-pass region contains extrusion moves.
- Scope limitation: this initial queue currently defers eligible paths only on the layer that reaches an H boundary, then flushes after that object/extruder's layer work. It does not yet accumulate an entire multi-layer H section into a single pass, nor does it implement a full swept-volume collision check.
- Safety gate added: while the current flush remains inside per-object/per-extruder layer processing, export now aborts when hybrid mode is enabled for multiple print objects, multiple instances, multiple configured extruders, or sequential/complete-object ordering. This is a temporary fail-closed restriction, not support for those layouts. A regression test asserts that multi-object export is rejected.
- The planar-path preservation requirement is still not met by the prototype: eligible source paths are replaced by deferred deformed paths. The section-level planner must instead leave ordinary planar toolpaths intact and generate a distinct finishing-path set from eligible exposed surface patches. Do not remove the safety gate or describe the current queue as a complete H-section pass until that planner and outer layer-group emission are integrated.

### Not yet implemented / not yet verified

- This first Mode B increment deforms paths on an existing layer; it does **not** insert a separate additional non-planar pass between planar layer groups.
- Mode A (generating new surface-following paths) is not implemented yet.
- Full nozzle, heater block, fan duct, probe, and carriage-envelope collision checking is not implemented.
- C++ compilation, G-code regression tests, and physical printer tests remain outstanding. Do not treat the branch as printer-ready.

Target workflow:

1. Print ordinary planar layers for a configurable vertical interval `H` (for example, 2.0 mm).
2. Insert a separately planned non-planar pass.
3. Resume planar printing for another interval `H`.
4. Repeat until the object is complete.

Both modes must be available:

- **Surface-following mode:** generate a new path over a selected model surface.
- **Existing-path deformation mode:** retain the XY paths generated by the slicer and assign safe per-vertex Z offsets to those paths.

The feature is intended for conventional 3-axis Cartesian FFF. A Z-axis move is not a substitute for a collision check.

## Source-code audit

### This SuperSlicer branch

- `ExtrusionPath3D` in `src/libslic3r/ExtrusionEntity.hpp` already stores one `z_offsets` entry per polyline vertex and reverses the offsets together with the polyline.
- `GCodeGenerator::extrude_path_3D()` in `src/libslic3r/GCode.cpp` is the active emission point for these paths.
- The existing implementation used the XY segment length for extrusion and selected `z_offsets[i]` as the destination offset. The first hybrid development commit changes this to use the destination vertex's `z_offsets[i + 1]` and the true 3D segment length. This is a focused correctness fix, not the scheduler.
- The current branch has no end-to-end scheduler that inserts non-planar passes at interval H. The legacy Z-offset path type alone does not provide surface generation, safe scheduling, or collision protection.

### Slic3r non-planar fork

Source inspected: `DrEricEbert/Slic3r_NonPlanar_Slicing`, branch `nonplanar`.

Its `NonplanarSurface` code groups adjacent mesh facets; `PrintObject::find_nonplanar_surfaces()` filters facets by a normal-angle threshold, rejects surfaces above a maximum height or below a minimum area, then calls `check_nonplanar_collisions()`. The collision routine projects a candidate surface to XY and compares it with existing layer regions, with an angle-derived offset.

Useful ideas:
- Use model triangles and their normals to identify eligible surfaces.
- Group adjacent eligible triangles into coherent surface patches.
- Reject candidates that violate height or area constraints.
- Check against existing printed geometry, not only against the target surface.

Limitations for this project:
- It is a legacy codebase and must not be copied wholesale into the current SuperSlicer branch.
- Its collision routine is based on 2D projected polygons and a simplified angle clearance. It does not model this printer's actual fan duct/heater block/probe volume.
- Its top-surface generation is not the requested recurring H-interval scheduler, and it does not implement the requested general existing-path deformation mode.

### AtomSlicer

Source inspected: `iota97/AtomSlicer`, branch `main`, including `src/bvhtriangle.h`, `src/hierarchy.h`, `src/isolines.cpp`, `src/curve.cpp`, and `src/slicer.cpp`. The current default-branch tree does not contain `src/toolpath.cpp` or `src/machine.cpp`; those names should not be treated as verified source locations.

It produces field-aligned toolpaths and uses mesh/BVH structures, normals, a maximum top tilt angle, and a nozzle collision cone setting. It is a useful reference for local surface direction and collision-aware path planning, but it is a separate research slicer with its own pipeline and machine support. Its multi-axis examples are not directly reusable as a drop-in 3-axis SuperSlicer implementation.

### CurviSlicer

Source inspected: `mfx-inria/curvislicer`, branch `master`.

It generates slightly curved layers across a part rather than only conforming the top surface. Its README explicitly warns about carriage/print collisions and describes clearance around the nozzle as a machine requirement. It is useful for understanding curved-layer thickness constraints, but its separate meshing/solver and G-code pipeline make direct integration expensive. Its current README also notes that some configurations require flow adaptation on Cartesian printers.

### RotBot

Source inspected: `RotBotSlicer/Nonplanar_Slicing`, branch `main`, including `surface.py` and `prusa_slicer.py`. The current default-branch tree does not contain `transform_method_2.py`; transform-first approaches should be attributed to the separate RotBot Transform project only after its exact files are verified.

It demonstrates two transform-first pipelines: transform the model, slice it, then map paths back; or invert/transform the part and project paths toward the bed. Its README exposes a maximum printing-angle precaution for fan/probe collisions. This is useful as a geometry experiment, but transforming complete G-code adds modal-state, extrusion, retraction, and coordinate-restoration risks. The production implementation should stay inside the slicer's typed geometry and G-code writer rather than blindly rewriting arbitrary G-code text.

## Proposed architecture

### 1. Configuration

Add object-level settings, initially disabled by default:

- `hybrid_nonplanar_enabled` (bool)
- `hybrid_nonplanar_interval` (H, mm; must be > 0)
- `hybrid_nonplanar_mode` (surface-following / existing-path deformation / both)
- `hybrid_nonplanar_max_angle` (degrees; maximum local surface slope)
- `hybrid_nonplanar_max_height` (mm; maximum permitted offset from the nominal layer)
- `hybrid_nonplanar_min_clearance` (mm; safety margin)
- `hybrid_nonplanar_max_slope_rate` (degrees per mm or a documented curvature bound)
- `hybrid_nonplanar_max_speed` and `hybrid_nonplanar_max_acceleration` (conservative motion caps for Z-varying moves)

Settings must be registered in the config schema, CLI mapping, default values, GUI option definitions, serialization, and tests. Do not expose a “collision-safe” claim unless the relevant tool geometry and print volume are actually checked.

### 2. Scheduling

Schedule candidate passes from object-space Z, not from G-code line counts or raw `;LAYER:` comments. Maintain a clear planar baseline and place each candidate pass only after the ordinary layers needed to support it have been printed.

- Use an explicit next-pass target Z and a tolerance to avoid floating-point duplicate/missed passes.
- Never move the printer's global layer index backwards.
- Keep first-layer, raft, support, modifier, and complete-object printing behavior unchanged until explicitly supported.
- If a candidate is unsafe or has no eligible paths, skip that pass and continue planar printing; never emit an unvalidated pass.

### 3. Surface-following mode

- Build/reuse a triangle mesh and acceleration structure in the same transformed coordinate space as sliced paths.
- Find eligible upward-facing surface patches, grouped by adjacency.
- Sample candidate paths at a bounded XY spacing and interpolate surface height only where the ray/triangle query is valid.
- Enforce maximum slope, offset, curvature/slope-rate, printable area, and material-thickness constraints.
- Generate typed `ExtrusionPath3D` entities with consistent per-vertex Z offsets and extrusion attributes.
- Do not extrapolate across holes or unsupported regions. Invalid samples break the path or reject the candidate.

### 4. Existing-path deformation mode

- Select ordinary paths at the scheduled pass and preserve their XY geometry and extrusion role.
- Compute a candidate Z offset per vertex from the model surface or a documented curve field.
- Resample long segments before applying a curve so the maximum Z change and curvature between points are bounded.
- Preserve path reversal, seam handling, extrusion flow, wipe behavior, and retraction semantics.
- Reject any path whose offsets are inconsistent with the underlying vertex count.

### 5. Collision model

A surface-slope limit is not a complete collision model. Safety checks must cover:

1. Nozzle tip to already printed material.
2. Heater block and fan duct to the printed part, using a configurable conservative footprint/height envelope.
3. Probe or other protruding hardware if present.
4. Travel moves between separate non-planar segments.
5. Bed limits and maximum Z travel.

The current source does not yet have a printer-specific CAD envelope for these parts. Until that is represented, collision checks must be described as conservative/incomplete, and the feature must remain opt-in.

### 6. Motion and extrusion

- Emit Z-varying moves through the existing writer and typed extrusion path pipeline.
- Calculate deposited path length in 3D when determining extrusion; do not base E solely on XY distance for a sloped segment.
- Limit segment slope and slope-rate; subdivide where needed.
- Treat speed/acceleration caps as motion constraints, not just a G-code F value. A per-move feedrate cap alone does not guarantee a bounded Z acceleration at corners.
- Preserve relative/absolute extrusion modes and ensure the writer remains the authority for coordinates, E, and modal state.

## Validation gates

1. Compile the affected C++ target with the project's supported toolchain.
2. Add unit tests for per-vertex Z indexing, 3D extrusion length, reversal, missing/short offset vectors, and segment subdivision.
3. Add geometry tests for slope, offset, curvature/slope-rate, mesh holes, and candidate surface rejection.
4. Export G-code for a flat cube with the feature disabled and verify byte-level or semantic equivalence to baseline.
5. Export test meshes with sloped roofs, domes, holes, narrow ridges, and nearby vertical walls; inspect every candidate pass in the G-code viewer.
6. Run collision-envelope checks and a no-extrusion/dry-run before any real print.
7. Test the first physical print at reduced speed with a deliberately conservative clearance envelope.

## Current status

- [x] Read the live target branch and relevant `ExtrusionPath3D` / G-code writer code.
- [x] Compare the main source-code approaches in AtomSlicer, CurviSlicer, RotBot, and the Slic3r non-planar fork.
- [x] Create an isolated development branch and commit the narrow 3D extrusion correctness fix.
- [ ] Compile and test the fix in the SuperSlicer build.
- [x] Add an experimental `Hybrid non-planar printing` checkbox beside ZAA; selecting it suppresses ZAA so both modes cannot run together.
- [ ] Implement the H-interval scheduler. The checkbox is only a reserved switch at this stage; it does not yet generate non-planar passes.
- [ ] Implement both path-generation modes.
- [ ] Implement and validate the collision envelope and motion constraints.
- [ ] Add end-to-end G-code regression tests.

## References

- SuperSlicer target branch: https://github.com/Jurgis-R/SuperSlicer2.7.61.10-Extra/tree/SS-2.7.61.10%2Bextra
- AtomSlicer: https://github.com/iota97/AtomSlicer
- CurviSlicer: https://github.com/mfx-inria/curvislicer
- RotBot: https://github.com/RotBotSlicer/Nonplanar_Slicing
- Slic3r non-planar fork: https://github.com/DrEricEbert/Slic3r_NonPlanar_Slicing/tree/nonplanar


## Implementation status

- [x] Add the Hybrid non-planar checkbox and keep it mutually exclusive with ZAA in generated G-code.
- [x] Add H interval, maximum slope angle, and maximum Z-offset controls.
- [x] Implement the first Mode B increment: interval-triggered deformation of eligible existing paths.
- [ ] Insert a separate non-planar pass between planar layer groups (the current interval trigger selects an existing layer instead).
- [ ] Implement generated surface-following paths (Mode A).
- [ ] Add full toolhead-envelope collision checks and safe fallback to planar paths.
- [ ] Compile the C++ application and add automated G-code regression tests.
- [ ] Validate with a small test print on the target Cartesian printer.


## Expanded external-source audit (2026-10-15)

This section records verified source locations and integration decisions before importing code. It is deliberately not a claim that external algorithms have already been ported or validated.

### AtomSlicer (MIT)

Repository: https://github.com/iota97/AtomSlicer

Verified source files:
- `src/bvhtriangle.h`: triangle BVH and closest-point queries. This is a candidate reference for accelerating mesh queries, but must be compared with SuperSlicer's existing mesh acceleration utilities before introducing a duplicate structure.
- `src/hierarchy.h`: mesh adjacency, vertex normals and field-aligned mesh hierarchy. This is relevant to Mode A surface direction / field estimation, not a drop-in replacement for SuperSlicer's mesh or path types.
- `src/isolines.cpp`: extracts isolines from per-vertex scalar values over triangles. This is a candidate for generating surface curves only if the required scalar field and topology assumptions match.
- `src/bvhpoint.h`, `src/bvhsegment.h`, `src/mathutils.h`, `src/curve.cpp`, and related files should be audited together before considering reuse because the geometry types and invariants are shared.
- `LICENSE`: MIT, copyright Giovanni Cocco and Inria (2026). Preserve the copyright and license notice for copied or substantial adapted portions.

Do not copy an isolated file until its dependencies, empty-mesh behavior, numerical tolerances, coordinate conventions, and tests have been reviewed. In particular, the observed `BVHTriangle::closestPoint` implementation initializes from triangle zero, so empty input handling must be checked before any reuse.

### Slic3r NonPlanar Slicing fork

Repository: https://github.com/DrEricEbert/Slic3r_NonPlanar_Slicing/tree/nonplanar

Verified source locations:
- `xs/src/libslic3r/NonplanarSurface.cpp`: surface-patch statistics and geometry.
- `xs/src/libslic3r/PrintObject.cpp`: the fork's integration point for finding eligible non-planar surfaces and checking candidate surfaces against existing printed geometry.
- Inspect the matching headers, facet representation, collision routine, and call sites as one unit before porting. The fork is structurally closer to SuperSlicer than a standalone pipeline, but branch ancestry and exact compatibility still need verification.

### CurviSlicer

Repository: https://github.com/mfx-inria/curvislicer

The upstream README explicitly describes the project as a research prototype, lists substantial external dependencies, and warns that generated curved trajectories may collide with the carriage. Its license is AGPL-3.0. Treat it as an algorithm/research reference unless a deliberate license-compatible integration decision is made; do not copy its implementation into this repository without resolving the licensing implications.

### RotBot Nonplanar Slicing and RotBot Transform

Repositories:
- https://github.com/RotBotSlicer/Nonplanar_Slicing
- https://github.com/RotBotSlicer/Transform

The Nonplanar_Slicing project is Python-based and transforms meshes/toolpaths around a conventional slicer. The separate Transform repository identifies GPL-3.0. These are useful for studying coordinate transforms, interpolation, resampling and extrusion correction, but are not direct C++ components for SuperSlicer's native path pipeline. Verify the exact license of each repository and file before copying.

### Integration rules

1. Prefer existing SuperSlicer geometry, mesh, polyline, extrusion, configuration, and G-code facilities when they already satisfy the need. Avoid parallel data structures without a measured reason.
2. Reuse algorithmic ideas freely as a design reference, but copy/adapt source code only after checking the exact repository/file license, notices, dependencies, and compatibility with this project's license.
3. Keep the H-interval scheduler separate from surface sampling and path generation. A scheduled pass must be represented explicitly in the print pipeline; an H boundary crossing on an ordinary layer is not a separate pass.
4. Both Mode A and Mode B must produce native path objects with explicit per-vertex 3D positions/offsets and correct extrusion accounting. Do not rewrite arbitrary final G-code as the primary implementation.
5. Collision validation must fail closed: if a candidate pass cannot be validated against the model/printed geometry and configured toolhead envelope, omit that candidate and preserve the safe planar path. A nozzle cone alone is not a full carriage/heater-block/fan/probe model.
6. Use the existing planar output as a regression baseline. With hybrid mode disabled, output should remain unchanged. With hybrid mode enabled, tests must verify pass ordering, E continuity, retract/unretract state, travel safety, units, coordinate mode, and bounds.
7. Do not mark the pull request ready or printer-safe until the actual SuperSlicer C++ target builds, automated tests pass, generated G-code is inspected, and controlled printer validation is performed.

### Next code milestones (source-led, no guessed APIs)

1. Map the current branch's mesh query and collision utilities against AtomSlicer's BVH and the Slic3r fork's surface/collision implementation; record which existing SuperSlicer utility wins and why.
2. Port one small, independently testable geometry component only if there is a demonstrated gap in existing utilities.
3. Implement the explicit H-interval pass scheduler in the native print pipeline, with tests before connecting either path strategy.
4. Implement Mode B as a separate pass first, then Mode A using a reviewed field/isolines strategy.
5. Implement toolhead-envelope configuration and conservative collision checks, then GUI mutual exclusion and full build/regression verification.


### SuperSlicer-native mesh infrastructure: reuse before porting

Additional direct inspection of the target branch confirms that `src/libslic3r/TriangleMesh.hpp` and `TriangleMesh.cpp` are already substantial native mesh infrastructure, with the project's existing AGPL-family licensing and geometry types. Therefore, AtomSlicer's standalone `BVHTriangle` should not be transplanted merely because it has a BVH: first identify the existing triangle acceleration/query paths in SuperSlicer and benchmark whether they can answer the exact surface-sampling and clearance queries needed. Introducing a second BVH without that comparison would duplicate infrastructure and risk mismatched coordinate units, mesh transforms, and invalid/empty-mesh behavior.

The Slic3r fork's `NonplanarSurface` uses its own `mesh_vertex`, `mesh_stats`, facet map, surface grouping, angle filtering, and projected-area representation. Porting that class unchanged would also duplicate geometry types and assumptions. The reusable target is the algorithmic sequence—candidate facet grouping, angle/height/area eligibility, then collision rejection—implemented against SuperSlicer's native mesh and polygon types after reviewing the fork's actual collision function and callers.

### Review limits and evidence discipline

The source review so far confirms the files and data structures listed above, but it is not yet a complete function-by-function audit of every repository. In particular, the exact Slic3r-fork collision implementation and AtomSlicer's full field-generation pipeline still need a traced review of callers, inputs, outputs, and edge cases. No external algorithm is considered port-ready until that review is complete. This is intentionally explicit so future implementation does not turn assumptions into undocumented facts.


## Broader non-planar project inventory (web + repository review, 2026-10-15)

This is a candidate inventory, not a claim that the entire internet has been exhausted. Projects are classified by their actual approach; similarly named projects are not automatically interchangeable.

### Directly relevant candidates

- **SelectiveNonPlanarSlicer** — https://github.com/Monta3D/SelectiveNonPlanarSlicer. Its README describes selecting STL surface patches, generating a separate non-planar toolpath, and exporting G-code to be inserted into a conventional planar workflow. The repository LICENSE is GPL-3.0. This is a particularly relevant reference for the requested interval-triggered finishing-pass architecture. Review its surface segmentation, path generation, G-code registration/insertion, and known limitations before implementation.
- **AtomSlicer** — https://github.com/iota97/AtomSlicer. C++ reference implementation for constant-thickness, field-aligned non-planar slicing and continuous toolpaths; MIT-licensed. Relevant to mesh acceleration, surface direction fields, isolines and continuous path generation.
- **CurviSlicer** — https://github.com/mfx-inria/curvislicer. Curved slicing for 3-axis printers; AGPL-3.0. Relevant to curved-layer construction, but its architecture and dependencies are not assumed to be drop-in compatible.
- **Slic3r NonPlanar Slicing** — https://github.com/DrEricEbert/Slic3r_NonPlanar_Slicing/tree/nonplanar. Older integrated slicer fork; relevant to surface candidate grouping and collision rejection. Must verify its branch history and implementation details against the target SuperSlicer revision.
- **Real3DFFF** — https://github.com/seeul8er/Real3DFFF. Curved-layer FFF workflow using a preform, then adjusting/splitting G-code segments against the target geometry and recalculating extrusion; also supports normal vectors for 5-axis use. Useful for studying segment subdivision, Z reconstruction, and extrusion compensation; its mixed 3-/5-axis scope must be separated carefully.
- **RotBot Nonplanar Slicing** — https://github.com/RotBotSlicer/Nonplanar_Slicing. Python workflow that transforms a model/preform and post-processes planar G-code. Relevant as a comparative method, not a native C++ implementation.
- **non-planar-post-processor** — https://github.com/eliasankerhold/non-planar-post-processor. Projects planar toolpaths onto a curved bed/surface. Useful for understanding G-code preservation and coordinate transformation, but its README warns that projection changes distances/angles and describes a limited experimental workflow; not sufficient as the main surface-aware slicer.
- **Unlayered Slicer** — https://github.com/Unlayered3D/Unlayered-Slicer. OrcaSlicer-derived project with multiplanar segment orientation and hotend collision warnings, plus a documented external non-planar slicing-service protocol. Its multiplanar feature is not equivalent to continuous curved-layer finishing, but its collision UI/workflow and service boundary are worth studying. Check exact source version and AGPL-3.0 obligations before reuse.
- **COMPAS Slicer** — https://compas.dev/compas_slicer/latest/. Research-oriented FDM slicing package with curved slicing using scalar fields/interpolation. Useful for field-driven path planning and fabrication parameters; it is a framework/toolkit, not a direct SuperSlicer patch.
- **Spliner** — https://spliner.io/. A current non-planar toolpath product described by its website. Include it in the landscape and evaluate publicly available technical documentation, but do not count it as reusable source code unless a public source repository and compatible license are found.
- **Nonplanar FDM / makertum** — https://github.com/makertum/non-planar-layer-fdm. Historical non-planar layer-FDM reference linked from the Slic3r fork. Inspect history and license before reuse; treat it as a historical candidate until current buildability and source behavior are verified.
- **Slicer-independent generation of customizable non-planar toolpaths for FFF (2026)** — https://doi.org/10.1007/s00170-026-19258-0; associated implementation: https://github.com/Monta3D/SelectiveNonPlanarSlicer. This recent paper describes surface-patch selection, planar reference path generation, vertical mesh projection, and insertion of separate finishing blocks into conventional G-code. It is unusually close to the requested hybrid-pass workflow, but the paper also describes limitations including unaccelerated pointwise projection and hardware clearance constraints.

### What this changes in the implementation plan

The first four-repository comparison was too narrow. The next source review must prioritize (1) SelectiveNonPlanarSlicer for independent-pass generation/insertion, (2) the Slic3r fork for native slicer integration and collision filtering, (3) AtomSlicer for mesh queries and field/isolines methods, and (4) Real3DFFF for segment subdivision and extrusion correction. CurviSlicer and COMPAS Slicer are references for curved-layer geometry; RotBot and the post-processor are comparison baselines; Unlayered is a reference for collision feedback and external-engine integration.

Before importing any code, inspect the exact source files, dependency graph, license, coordinate assumptions, mesh validity handling, and tests. Reuse SuperSlicer's native mesh and G-code infrastructure where possible. This inventory does not yet constitute a complete code audit of every listed project.


## Comparative source audit: implementation details (2026-10-15)

This pass traced actual functions rather than relying only on project descriptions. Findings below distinguish reusable engineering patterns from code that should not be transplanted.

### 1. SuperSlicer-native mesh queries — first choice for geometry acceleration

Verified target-branch files:
- `src/libslic3r/AABBMesh.hpp`
- `src/libslic3r/AABBMesh.cpp`
- `src/libslic3r/AABBTreeIndirect.hpp`
- `tests/libslic3r/test_aabbindirect.cpp`
- `src/libslic3r/TriangleMesh.hpp/.cpp`

The target already has an indexed-triangle AABB mesh wrapper with ray queries (first hit and all hits), squared-distance/closest-point queries, face normals, vertex-face indexing, and face-neighbor indexing. The indirect AABB implementation already has triangle closest-point logic; its existing tests cover ray casting, closest-point queries and line-distance queries.

**Decision:** do not add AtomSlicer's BVH as a second acceleration structure yet. First instantiate/reuse the existing SuperSlicer mesh/query classes in the model's transformed coordinate frame. Verify lifetime/ownership, scaling, transforms, mesh validity, face IDs, and query semantics. Use all-hit ray queries when the surface is multi-valued along Z; a first-hit-only query can select the wrong shell or underside. Use squared distances where only a threshold comparison is needed. Add targeted tests for rays parallel to triangles, edge/vertex hits, duplicate intersections, open meshes, and empty meshes.

### 2. SelectiveNonPlanarSlicer — best architectural match for a separate finishing pass, but not production-ready geometry

Verified files:
- `Monta3D/SelectiveNonPlanarSlicer/projectiondot.py`
- `Monta3D/SelectiveNonPlanarSlicer/stlprocessor.py`
- `Monta3D/SelectiveNonPlanarSlicer/trajectoryplanner.py`
- `Monta3D/SelectiveNonPlanarSlicer/gcodeexport.py`
- `Monta3D/SelectiveNonPlanarSlicer/gcode_merger.py`
- `Monta3D/SelectiveNonPlanarSlicer/gcode_jump.py`

Observed algorithm:
1. Split candidate triangles/surfaces from an STL using angle/height/area-related filters.
2. Generate a planar reference trajectory (the inspected example uses a 45-degree raster).
3. Project each reference point along a configurable vector onto candidate triangles.
4. Reorder projected points into scan runs, build path steps, and export separate G-code.
5. Merge generated surface G-code into a planar G-code file at a marker.

Important limitations in the inspected implementation:
- `GetProjectionDot` tiles the query point against every triangle, so the projection scan is effectively point-count × triangle-count instead of using a spatial index.
- The projection math divides by triangle-plane and projected-XY denominators without consistently rejecting near-zero denominators before division; vertical/degenerate projected triangles need explicit handling.
- The implementation is prototype-oriented (plotting imports, hard-coded example files/coordinates and fixed layer offsets appear in the inspected export path).
- `gcode_merger.py` inserts files at a hard-coded marker occurrence and assumes four surface files (using empty files for unused slots). It is not a robust pass scheduler or a modal-state-aware G-code integration layer.
- `gcode_jump.py` parses coordinates from text heuristically and inserts Z lifts based on XY distance. That does not constitute collision checking against the actual printed volume or printer toolhead.

**Decision:** reuse the workflow concept—generate a separate pass and insert it at an explicit scheduler boundary—but reimplement the path generation with SuperSlicer's native mesh queries and typed extrusion entities. Do not port the projection loop or merger script as-is. The repository license is GPL-3.0; any code reuse requires a deliberate compatibility and attribution review.

### 3. Slic3r NonPlanar fork — useful integrated surface filtering and collision rejection

Verified functions:
- `NonplanarSurface::group_surfaces()`, `mark_neighbor_surfaces()`, `check_max_printing_height()`, `check_surface_area()`, and `horizontal_projection()` in `xs/src/libslic3r/NonplanarSurface.cpp`.
- `PrintObject::check_nonplanar_collisions()` in `xs/src/libslic3r/PrintObject.cpp`, plus its call sites.

Observed pattern:
- Group connected eligible facets by neighbor links.
- Reject patches exceeding a height span or falling below a minimum area.
- Project the candidate patch into XY and test it against existing layer slices and a progressively offset 2D collider polygon.
- Reject when the computed overlap area exceeds a threshold (the inspected code uses 1 mm²).

Limitations:
- The collision test is a 2D projected-region heuristic with a slope-derived offset; it is not a 3D swept-volume test of the nozzle, heater block, fan duct, probe, or carriage.
- Several thresholds are fixed constants, and the angle eligibility method is not a complete collision model.
- This is a legacy fork, so types, config plumbing and scheduling must be mapped to the exact SuperSlicer revision before any port.

**Decision:** retain the sequence (connected patch extraction → angle/height/area filtering → collision rejection), but adapt it to the current indexed mesh and existing polygon/region types. Generalize thresholds into settings only where the setting has a clear physical meaning. Collision acceptance must include a conservative toolhead envelope, not only the projected nozzle path.

### 4. Real3DFFF — strongest reference for segment subdivision, extrusion correction and travel planning

Verified files:
- `gcode/real_3d/generate_curved_layer.py`
- `ZGetter.py`
- `data_io/LDGCode.py` and the G-code movement representation.

Observed pattern:
- Start from a conventional slicer's G-code for a constant-XY preform.
- Intersect planar extrusion segments with projected geometry edges and split them where the target surface changes.
- Recompute Z using vertical surface intersections at each segment endpoint.
- Scale extrusion for segment length changes; subdivide linear cross-section changes to bound extrusion error.
- Treat travel moves separately, using direct, low-lift and high-lift classes; high lifts use the highest known geometry plus a clearance.
- Optionally calculate surface normals, primarily for multi-axis use.

Strengths:
- Explicit split points, ordered segment subdivision, minimum segment length, extrusion-error control, and separate travel handling are more mature concepts than simply moving existing vertices vertically.
- The implementation's own documentation clearly states its preform/XY-congruence assumptions and that undercuts/complex cross-sections require special treatment.

Limitations for SuperSlicer:
- It is a Python/OpenCascade pipeline with a different G-code representation, not a C++ library that can be dropped into `libslic3r`.
- Its preform method assumes matching XY projections and a known layer mapping; that is not generally valid for every arbitrary model/path.
- Its travel clearance heuristics are not a guarantee of toolhead collision freedom.
- Extrusion corrections must be reconciled with SuperSlicer's own flow/volumetric calculations, extrusion modes, and typed path writer; do not stack two independent correction schemes.

**Decision:** port the *engineering principles*, not the Python code: split at geometry events, resample to a bounded geometric error, conserve/derive extrusion from actual deposited 3D path and local bead cross-section, and classify travel routes against the actual clearance model. Add tests for zero-length segments, duplicated split points, tiny segments, and discontinuous surfaces.

### 5. AtomSlicer — advanced field and isoline reference, not the hybrid scheduler

Verified files:
- `src/bvhtriangle.h`: BVH closest-point and ray/triangle intersection helpers.
- `src/hierarchy.h`: adjacency graph, normals and multilevel mesh hierarchy for field-aligned optimization.
- `src/isolines.cpp`: extraction of curves from a scalar field defined at mesh vertices.
- `src/curve.cpp` and `src/slicer.cpp`: additional toolpath/slicing pipeline context.

Strengths:
- Separates mesh acceleration, field processing, and curve extraction rather than mixing all geometry into G-code emission.
- The isoline approach can generate connected curves from a well-defined scalar field; this is a possible Mode A direction if the field represents the intended surface-following path family.
- The hierarchy is useful for understanding mesh adjacency, orientation and field smoothing.

Limitations:
- Its custom mesh/BVH and raw-pointer hierarchy types are not SuperSlicer types; a direct transplant would create duplicate infrastructure.
- The closest-point routine's initial triangle assumption makes empty input an explicit edge case to handle before any reuse.
- Isoline extraction depends on consistent triangle topology and scalar values; open/non-manifold meshes and exact-zero vertices require explicit tests.
- Field-aligned constant-thickness slicing is a different objective from inserting a finishing pass every H millimeters.

**Decision:** first use SuperSlicer's existing AABB mesh. Study AtomSlicer for the field/isolines algorithm only if Mode A requires new connected surface curves beyond what native path/polygon tools can provide. Any copied MIT code must retain the relevant license/copyright notices.

### 6. RotBot, CurviSlicer, Unlayered and COMPAS — role in the design

- **RotBot:** `surface.py` builds/interpolates a surface grid from selected mesh facets; `prusa_slicer.py` invokes a conventional slicer via subprocess. Useful as a transform/interpolation experiment, but the inspected repository does not supply a native SuperSlicer C++ integration or a full toolhead collision model.
- **CurviSlicer:** the source tree contains its own mesh/tetrahedral and G-code pipeline. It is aimed at curved layers across the part, so it is relevant to curved-layer constraints but not the most direct starting point for a discrete H-interval finishing pass. Its AGPL-3.0 license and external dependencies need to be respected.
- **Unlayered Slicer:** `NONPLANAR_SLICE_SERVICE.md` specifies an external-service request/result protocol and preview integration. This is useful if an external research engine is eventually needed, but it does not itself supply the core surface projection algorithm for the native implementation.
- **COMPAS Slicer:** useful algorithmic reference for scalar-field-based curved slicing and fabrication planning; its Python framework does not remove the need for native C++ mesh queries, scheduling and toolhead clearance.

### 6.1 Why the current deferred-path queue cannot simply span a whole H section

The current Mode B contour path is sampled against a mesh ray and stores Z offsets relative to the *source layer's* nominal `print_z`. That makes it a layer-local replacement path, not a section-level finishing path. Naively keeping these paths until the H boundary and then emitting them together would revisit lower nominal Z values after higher layers have already been printed; this can collide with the part and does not implement a true section finishing pass.

A correct H-section implementation therefore needs a different data product:
- Keep the normal planar section's extrusion untouched.
- At the section boundary, build a finishing-pass path set from the section's exposed surface/eligible mesh patches, not by replaying per-layer replacement contours.
- Project/plan those paths against the final section surface, split at mesh/query and geometric-error events, and assign one explicit pass Z reference.
- Order all objects and regions in the layer group before emitting that pass; do not flush from the per-object/per-extruder helper.
- Reject the candidate pass if any path crosses an invalid surface, violates slope/offset limits, or fails the configured toolhead clearance model.

Until that planner exists, the current queue is only a limited boundary-layer experiment. It must not be described as accumulating or finishing a full H section.

### 7. Resulting implementation choices

1. **Geometry query:** reuse `AABBMesh` / `AABBTreeIndirect` and current `TriangleMesh` data; do not introduce another BVH unless profiling shows a missing capability or a measured bottleneck.
2. **Separate pass architecture:** implement an explicit H-interval scheduler in the native print pipeline. Do not splice standalone G-code text at marker comments.
3. **Mode B first:** split existing eligible extrusion paths at surface/query and geometric-error events, then build a separate pass with valid per-vertex Z offsets. Keep the ordinary planar paths intact as fallback.
4. **Mode A second:** evaluate connected patch + field/isolines generation after query semantics and surface validity rules are tested.
5. **Extrusion:** use SuperSlicer's typed writer and one clearly defined extrusion calculation. Account for actual 3D segment length and bead cross-section without double-applying Real3DFFF-style compensation.
6. **Collision:** start with candidate patch rejection against existing printed geometry, then add a conservative configurable toolhead envelope and safe travel planning. The fork's 2D collision heuristic is a reference, not the final safety check.
7. **Fail-closed behavior:** invalid mesh queries, ambiguous surface hits, degenerate triangles, unsupported travel or a collision-envelope violation must skip the candidate pass, not emit guessed Z values.
8. **Testing:** build a native geometry unit-test suite first (projection hits, edge/vertex cases, holes, empty meshes, multi-hit vertical rays, surface-angle limits, segment splitting, E continuity and travel clearance), then test generated G-code and the actual build.

This is a source-level comparative audit of the listed implementations, not an exhaustive review of every file in every repository. The current findings are enough to choose the integration direction, but not enough to claim that any external algorithm is already safely ported or that the feature is printer-ready.
