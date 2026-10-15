# SuperSlicer 5X: integration plan

## Scope

The `dev_5x` branch targets genuine five-axis slicing and coordinated five-axis printing. Conventional three-axis hybrid/non-planar experiments are out of scope and are not prerequisites for this branch.

The five-axis implementation must remain opt-in until the complete toolpath, machine transform, motion validation, and postprocessor are integrated and tested. Ordinary three-axis output must remain unchanged when five-axis mode is disabled.

## Current code

- `src/libslic3r/FiveAxisKinematics.hpp`: configurable two-joint orientation solving, rotary-bed point transforms, and rotary-head tool-tip compensation.
- `src/libslic3r/FiveAxisEngine.hpp`: candidate-path checks for finite coordinates, rotary limits, simultaneous-axis policy, rotary change per linear distance, and estimated rotary rate; emits XYZ, two configured rotary words, E, and F only after whole-path validation.
- `tests/libslic3r/test_five_axis_kinematics.cpp` and `tests/libslic3r/test_five_axis_engine.cpp`: unit coverage for the isolated math and output helpers.

These are helper-level primitives. They are not yet a complete slicer-to-machine pipeline, and output validation is not a geometric collision test.

## Next integration milestones

1. **Machine configuration and coordinate frames**
   - Define rotary axis order, local axis vectors, pivots, zero offsets, sign conventions, tool-center/tool-tip offsets, and joint limits.
   - Explicitly distinguish rotary-bed and rotary-head chains.
   - Validate configurations and reject singular or unreachable poses.

2. **Tool orientation and inverse kinematics**
   - Derive tool orientation from a generated five-axis path, not from layer normals alone.
   - Solve each path pose using the configured machine chain.
   - Track the previous solution to avoid unnecessary rotary flips and discontinuities.
   - Reject solutions outside joint limits or tolerance.

3. **Machine-aware motion planning**
   - Transform path points into machine coordinates and compensate the tool-center point.
   - Check XYZ travel, rotary travel, rotary rate, linear speed, acceleration and extrusion flow.
   - Add conservative swept-tool and machine-envelope collision validation. A path must be rejected if required collision geometry is unavailable.

4. **G-code integration**
   - Connect a dedicated five-axis path representation to the real `GCodeGenerator` pipeline.
   - Keep firmware-specific axis words, coordinate modes, homing/setup commands, and postprocessing in a machine-specific output layer.
   - Preserve normal three-axis G-code when five-axis mode is disabled.
   - Validate the entire candidate path before writing any of its moves.

5. **Verification**
   - Add tests for non-orthogonal joint axes, both rotary-bed and rotary-head chains, pivot compensation, limits, singularities, rotary discontinuities, extrusion continuity, and collision rejection.
   - Add golden G-code regression tests and a Windows 64-bit build.
   - Inspect output in a toolpath preview and use staged, low-risk machine tests before attempting a real part.

## Safety boundary

The current helpers do not prove that the nozzle, tool body, part, fixture, or machine structure cannot collide. They also do not configure or home real hardware. No generated five-axis output should be considered print-ready until the machine-specific transform, complete collision checks, postprocessor, and regression/build tests are implemented and validated.

## External references

External slicers and research projects may inform architecture, but no external implementation should be copied without checking its exact license, file notices, and dependencies. Machine conventions from one project must not be assumed to match another machine.
