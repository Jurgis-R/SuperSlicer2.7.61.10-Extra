# Five-axis reference study for SuperSlicer 5X

## Goal

Build genuine five-axis slicing and printing into SuperSlicer by reusing established slicer architecture and validated practices where possible. This branch is not for conventional three-axis non-planar printing. Five-axis paths must ultimately be generated from sliced model geometry, solved for a configured machine, validated, and emitted by the real G-code export pipeline.

This is a source-based architecture study, not a claim that any reference implementation is drop-in compatible.

## Reference repositories inspected

| Project | Repository license reported by GitHub | Useful source/design patterns | Limits and cautions |
| --- | --- | --- | --- |
| Fractal Cortex | GPL-3.0 | Multidirectional slicing, chunk/part orientation changes, example five-axis G-code | Chunk reorientation is a different strategy from continuous simultaneous five-axis contouring. Review its machine model and coordinate conventions before reuse. |
| 5AxisSlicer | MIT | Desktop slicing workbenches, machine profiles, rotary axis word mapping, path preview and exported G-code | It is a separate slicer with its own geometry, machine model, and export assumptions. Inspect specific algorithms before considering adaptation. |
| Open5x Slicer (web) | MIT | Separate modules for slicing planes, toolpaths, machine profiles, inverse kinematics, rotary G-code, and collision checks; automated tests for G-code and kinematic validation | The current backend has its own data model and dialect. A module named collision is not, by itself, proof of complete swept-volume collision safety. |
| Open5x (Rhino/Grasshopper) | MIT | Machine/hardware configurations and examples of coordinated rotary-axis printing | The project is coupled to a Rhino/Grasshopper workflow and its supported hardware. |
| 5Axis-Gcode-Generator | No license identified in repository metadata | Historical examples of rotary-axis G-code and toolpath generation | Do not copy code until the repository license and provenance are established. Its angle-specific approach may not solve continuous contour paths. |
| Real3DFFF | LGPL-3.0 | Surface and curved-layer geometry, normal handling, and slicing research | Treat it as a geometry/toolpath reference, not as a complete machine-specific five-axis postprocessor. |
| Commercial 5 Axis Slicer | Commercial product; some postprocessors reportedly open source | Postprocessor and robot-controller workflow concepts | Only use source that is explicitly published under a compatible license. Do not infer rights to copy the commercial application. |

Repository links:
- https://github.com/fractalrobotics/Fractal-Cortex
- https://github.com/txf0101/5AxisSlicer
- https://github.com/ee24b022-gif/5_axis_slicer
- https://github.com/FreddieHong19/Open5x
- https://github.com/Caryhao/5Axis-Gcode-Generator
- https://github.com/seeul8er/Real3DFFF
- https://5-axis-slicer.com/

## Practices to carry into SuperSlicer

### 1. Keep machine semantics separate from output dialect

Represent the actual machine joints and kinematic chain independently from G-code letters. A mapping from internal joints to A/B/C/U/V/W output words changes formatting, not physical kinematics. Profiles must include rotary axis vectors/order, pivots, sign and zero offsets, travel limits, units, and the applicable tool-center/tool-tip reference.

### 2. Keep geometry, kinematics, motion validation, and postprocessing as distinct stages

Recommended pipeline:
1. Slice/derive candidate geometry and a toolpath in model/work coordinates.
2. Assign a tool pose (position plus orientation) along the path.
3. Solve inverse kinematics for the configured machine, retaining continuous joint solutions and rejecting unreachable poses.
4. Transform points with correct rotary pivots and tool-tip compensation.
5. Plan coordinated linear and rotary motion under joint, speed, acceleration, and extrusion constraints.
6. Check the nozzle/tool body, part, fixture, bed, and relevant machine envelope for collisions.
7. Emit the fully validated path through a machine-specific G-code/postprocessor.
8. Parse or simulate the exported file and compare it with the intended machine-space path.

The output formatter must not infer rotary angles from surface normals on its own.

### 3. Validate complete paths before emitting output

The Open5x Slicer repository has explicit tests for rotary G-code, machine profiles, kinematic validation, collision boundaries, and G-code snapshots/security. SuperSlicer should adopt comparable test categories within its existing C++/Catch2 build. Validate an entire candidate before writing its motion lines, so a rejected path cannot leave partial unsafe output.

### 4. Use SuperSlicer's existing architecture

The current codebase's actual integration point is GCodeGenerator in src/libslic3r/GCode.cpp, with declarations in GCode.hpp, configuration in PrintConfig.hpp/.cpp, and build/test registration in CMake. The existing FiveAxisKinematics.hpp and FiveAxisEngine.hpp are helper-level work; they do not yet constitute an integrated model-to-five-axis pipeline.

Keep five-axis mode opt-in. When it is disabled, ordinary three-axis output must remain byte-for-byte unchanged where practical and pass the existing G-code regression suite.

### 5. License and provenance

Use reference projects to learn interfaces, algorithms, test cases, and failure modes. Before copying or adapting source, record the exact source file, copyright notice, license, dependencies, and compatibility with SuperSlicer's licensing/distribution. Repository-level license metadata does not necessarily settle the license of every bundled dependency or individual file. Do not copy from projects with no clearly established license.

## Implementation order

1. Add a typed five-axis machine profile and validate its coordinate frames and rotary limits.
2. Define a tool-pose/path representation connected to actual sliced geometry.
3. Implement and test machine-specific inverse-kinematics adapters, including rotary-bed and rotary-head arrangements.
4. Add continuous solution selection, singularity/unreachable-pose rejection, and rotary pivot/tool-tip compensation.
5. Add machine-space motion constraints and conservative geometric collision validation.
6. Integrate a dedicated five-axis export path in GCodeGenerator, guarded by an explicit feature setting and postprocessor selection.
7. Add end-to-end model-to-G-code tests, golden output tests, and Windows 64-bit build verification.
8. Test offline simulation and then staged machine tests only after the target machine profile is calibrated.

## Current implementation status on dev_5x

The first motion-planning increment now exists in `src/libslic3r/FiveAxisMotionPlanner.hpp`:

- Accepts ordered work-coordinate tool poses (position, requested tool axis, extrusion, and feedrate).
- Solves each pose using the existing rotary inverse-kinematics helper, seeding each solve from the previous joint solution to encourage continuity.
- Applies rotary-bed point transforms or rotary-head tool-tip compensation.
- Validates the resulting complete command list against configured rotary limits, angular change, rate, and simultaneous-axis rules before returning it.
- Supports conservative sampled clearance checks against caller-supplied axis-aligned obstacle boxes.
- Clears the caller's output on failure and only returns a candidate after all checks pass.

Catch2 tests were added in `tests/libslic3r/test_five_axis_motion_planner.cpp` and registered in `tests/libslic3r/CMakeLists.txt`.

**This is not yet the complete slicer pipeline.** The planner currently consumes tool poses; it does not derive them from the actual model mesh or SuperSlicer's extrusion entities, does not model a full nozzle/head swept volume or mesh collision, and is not yet called by `GCodeGenerator`. Machine profile settings/UI, postprocessor selection, setup/mode commands, extrusion semantics, and end-to-end G-code parsing remain to be implemented. The added tests have not been compiled or executed because no build was run.

### Toolpath data-contract increment

`ExtrusionPath3D` now has an optional `tool_axis_normals` array with one desired tool-axis direction per polyline vertex. The array is copied and moved with the path and reversed together with its vertices. Legacy paths keep the array empty, so existing XYZ-only paths retain their prior behavior. `has_tool_axis_normals()` reports whether the metadata is populated and aligned with the path vertices.

This is only the data contract for a future model-derived surface-path producer. Existing path producers do not populate the directions, and `GCodeGenerator::extrude_path_3D()` does not yet consume them. No five-axis output is enabled by this change. A Catch2 test covers alignment detection, copy, reverse, and stale metadata detection; it has been registered but has not been compiled or run.

### Oriented extrusion adapter and typed machine profile

The engine now has two additional helper-level pieces:

- `FiveAxisMachineProfile.hpp` defines an opt-in machine profile containing the rotary mounting/axis/pivot configuration, rotary output words and limits, collision sampling settings, an explicit postprocessor identifier, and an XYZ envelope. Validation rejects mismatched kinematics/output limits and invalid envelopes. This is currently a C++ schema, not yet persisted in `PrintConfig` or exposed in the settings UI. The XYZ envelope is validated but not yet enforced by the motion planner.
- `FiveAxisExtrusionAdapter.hpp` converts an existing `ExtrusionPath3D` with complete per-vertex tool-axis directions into normalized planner poses. It converts SuperSlicer scaled coordinates to millimetres, applies the supplied nominal layer Z and path Z offsets, and calculates cumulative E from an explicitly supplied E-per-mm value. `plan_five_axis_extrusion_path()` then validates the profile and runs the poses through the motion planner. It rejects disabled profiles, missing/misaligned directions, invalid settings, and planning failures without returning partial commands.

Catch2 tests cover the adapter, cumulative extrusion, fail-closed behavior, and profile validation. They are registered in the test target but have not been compiled or run.

These changes establish a path-data-to-planner API; they do not mean the regular slicer currently produces the normals or calls the adapter. The current `GCodeGenerator` still uses its ordinary `extrude_path_3D()` implementation. Do not enable physical five-axis printing based on these helpers alone.


### Accelerated model-surface normal sampling

- `FiveAxisSurfaceNormals.hpp` adds a sampler backed by SuperSlicer's existing `AABBMesh` nearest-triangle query, avoiding a brute-force scan of every triangle for every path vertex.
- It returns the nearest triangle's normalized, mesh-winding normal and can reject samples farther than a caller-supplied distance threshold. It can also populate an `ExtrusionPath3D`'s per-vertex direction metadata; assignment is atomic, so a failed sample leaves existing normals untouched.
- Coordinate-frame responsibility is explicit: the mesh and queried path vertices must already be in the same frame and in millimetres. The helper does not apply model-instance transforms, choose inward versus outward orientation, infer a tool tilt strategy, or guarantee that the nearest face is the intended surface on thin/overlapping geometry. The caller must choose an appropriate surface mesh and distance threshold.
- Catch2 coverage was added for nearest-face normal lookup, distance rejection, path metadata assignment, and failure atomicity. These tests are registered but have **not** been compiled or run.

This is the first concrete model-geometry-to-normal producer, but it is not yet connected to actual slicer-generated extrusion paths. Coordinate transforms, intended-surface selection, path generation, persistent machine settings, and the G-code export integration remain outstanding.


The surface-normal helper now also provides `transform_five_axis_surface_mesh()`. It applies an explicitly supplied affine transform to mesh vertices before the accelerated sampler is built, rejects non-finite or singular transforms, and reverses face winding for mirrored transforms so sampled face normals retain their intended orientation. Tests cover translated geometry, mirrored winding, and rejection of singular transforms. The helper is intentionally explicit: it does not choose or compose SuperSlicer's volume, object, instance, and centering transforms on behalf of the caller. That transform chain still needs to be connected at the real slicing integration point.

## Safety and verification status

The present helper-level kinematics and output code does not yet establish complete collision safety, slicer integration, or a validated machine postprocessor. Do not use its output for physical printing until those pieces and the end-to-end tests are complete. No successful Windows build is claimed by this document.