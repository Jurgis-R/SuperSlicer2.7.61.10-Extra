# Five-axis / non-planar implementation references

This document records how external projects inform the SuperSlicer `dev_5x` design. They are references for distinct algorithms and machine interfaces, not one interchangeable implementation. The existing planar slicing, extrusion planning, and G-code pipeline remain the baseline.

## Reference matrix

| Reference | Technique worth studying | Intended use in this fork | Boundary / caution |
|---|---|---|---|
| [Fractal Cortex](https://github.com/fractalrobotics/Fractal-Cortex) | Multidirectional printing by dividing a model into chunks and reorienting the part between chunks | Chunk planner, chunk ordering, transition sequencing, part-frame transforms | Chunked reorientation is different from continuously tilted nozzle motion; validate the exact machine/firmware convention |
| [5AxisSlicer](https://github.com/txf0101/5AxisSlicer) | Dedicated five-axis slicing, rotary-bed workflows, multiple geometry-oriented workbenches and NC/G-code preview | Compare rotary-bed path generation, machine setup, export and preview validation | It is a separate Python application, not a drop-in C++ slicing library |
| [Open5x web slicer](https://github.com/ee24b022-gif/5_axis_slicer) | Web-based path generation with inverse kinematics in the backend (as described by the project) | Study the separation between geometric path generation and IK/post-processing | Verify the current source and supported machine models before relying on details |
| [Open5x Rhino/Grasshopper](https://github.com/FreddieHong19/Open5x) | Tool/bed articulation and generation for a converted Cartesian printer using two rotary axes | Machine-frame calibration, rotary-joint conventions, setup/zero-position assumptions | Its hardware and Grasshopper/Rhino workflow are specific; don't assume its U/V axes map to A/B or B/C on another machine |
| [5Axis-Gcode-Generator](https://github.com/Caryhao/5Axis-Gcode-Generator) | B/C rotary output for supported fixed-angle cases | Reference for rotary words and postprocessor conventions | Fixed-angle output is not a solution for continuously varying surface normals |
| [Commercial 5 Axis Slicer](https://5-axis-slicer.com/) | Multiple machine-code postprocessors, including G-code and robot languages according to its public product information | Compare the postprocessor boundary and machine-specific output responsibilities | Commercial slicer algorithms are not treated as open-source; use only publicly available postprocessor code under its actual license |
| [Real3DFFF](https://github.com/seeul8er/Real3DFFF) | Curved-layer paths, normal-vector metadata, adaptive Z adjustment, segment splitting, extrusion recalculation and lifted travel strategies | Surface-following algorithm reference and G-code regression cases | It is a post-processing workflow with a preform model; its N/O/R normal-vector convention is not standard XYZAB rotary output |
| [SciSlice](https://github.com/VanHulleOne/SciSlice) | Research-oriented toolpaths and ABB RAPID robot output | Keep robot-language output behind a separate postprocessor interface | RAPID output is not ordinary printer G-code and needs robot-specific kinematics and safety checks |
| Printer3D-Application (CuraEngine extension) | Potential extension reference mentioned for investigation | Research candidate only until the exact repository and axis-output format are verified | The specific repository URL and real rotary-axis output have not yet been verified; do not base implementation on the name alone |
| [S4 Slicer](https://github.com/jyjblrd/S4_Slicer) | Generic non-planar toolpath generation from a conventional slicer workflow | Study surface/path generation and collision-relevant geometry handling for non-planar modes | Its generic non-planar path approach is not itself a general five-axis inverse-kinematics solution |

## Target architecture

1. **Preserve planar baseline.** The existing SuperSlicer path generation remains the default and fallback. Experimental output is opt-in.
2. **Strategy layer.** Keep two distinct strategies: (a) continuous surface-following / curved-layer finishing passes, and (b) multidirectional chunk slicing with discrete part reorientation. They may share mesh, path and validation utilities but must not share assumptions about transitions.
3. **Machine kinematics layer.** Convert desired tool orientation or part-frame orientation into machine joint positions. Model rotary-axis order, axis vectors, pivots, zero offsets, sign conventions, limits, and whether each joint moves the bed/part or the head/tool. No machine type is inferred from the axis letters.
4. **Safety preflight.** Check the complete candidate path before emission: valid coordinates, rotary travel and rate limits, extrusion continuity, and a conservative swept-tool collision test against the printed mesh/part and machine envelope. Missing collision validation must reject the path.
5. **Output/postprocessor layer.** Emit XYZ plus the configured rotary words only after the kinematics and safety layers approve the full path. Firmware-specific start/end code, coordinate modes and axis words belong here.
6. **Regression and preview.** Compare normal 3-axis G-code byte-for-byte where practical; add synthetic flat, ramp, curved, and multi-chunk models; verify toolpaths in preview before any physical print.

## Current implementation boundary

`src/libslic3r/FiveAxisKinematics.hpp` adds a configurable two-rotary-joint orientation solver, serial-axis rotation math, rotary-bed point transforms, and rotary-head tool-tip pivot compensation. The solver enforces configured angle ranges and fails closed when its numerical inverse-kinematics iteration does not converge.

`src/libslic3r/FiveAxisEngine.hpp` adds the output stage: configurable rotary axis words and limits, whole-path validation, rotary angle-change/rate checks, and all-or-nothing XYZ/rotary/E/F formatting. The kinematics and output primitives are registered in the existing libslic3r and unit-test CMake lists.

This is still **not a complete or print-ready five-axis engine**. The kinematics core is not yet called by `GCodeGenerator`; UI/profile configuration, full XYZ frame/calibration handling, machine-envelope checks, and firmware-specific output setup remain unintegrated. The output validator is a motion-limit guard, not a geometric collision test. The hybrid non-planar surface planner's collision callback must be backed by a real mesh/tool sweep implementation before its paths can be approved for G-code.

## Licensing and reuse

Use the references to compare behavior and derive interfaces. Before copying any implementation, record its exact repository license, file-level notices, and dependency licenses; do not copy code from the commercial product or from a project whose terms have not been checked. Keep attribution and notices with any code that is legitimately reused.
