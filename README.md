# BoneFlex

GPU-accelerated, bone-driven softbody simulation for high-resolution character meshes.

BoneFlex is being developed as a standalone C++ library. The long-term target is a hybrid reduced simulation pipeline that combines skeletal animation, clustered shape matching, local XPBD constraints, and GPU mesh deformation.

## Goals

- Support high-resolution render meshes (initial target: approximately 450,000 vertices).
- Reuse an existing skeletal rig and skinning weights.
- Simulate a reduced set of control particles rather than treating every render vertex as a physical particle.
- Combine global shape preservation with local, controllable deformation.
- Keep simulation algorithms independent from rendering and GPU backends.
- Provide deterministic tests and reproducible benchmarks.

## Architecture

```text
BoneFlex/
  include/boneflex/       Public library headers
  src/                    Library implementation
  tests/                  Automated tests
  examples/               Standalone experiments
  benchmarks/             Performance workloads
```

Planned runtime pipeline:

1. Evaluate the input skeleton pose using the animation module and build the skinning palette.
2. Update kinematic anchors from the skeleton.
3. Solve reduced softbody constraints (XPBD and shape matching).
4. Transfer reduced deformation to the render mesh.
5. Upload the deformed mesh through a selectable compute backend.

The skinning reference uses a compact CSR representation (`offsets`, `bone_indices`, `weights`) so vertices can have a variable number of influences. Per-vertex weights are validated and normalized when loaded.

The reduced-mesh foundation stores simulation control points and surface triangle connectivity separately from render topology. A second CSR structure (`vertex_embedding`) maps each render vertex to a weighted set of control points. The CPU reconstruction function is the reference implementation for future GPU deformation kernels.

## Build

Requirements: CMake 3.20+ and a C++20 compiler.

```powershell
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

## Current status

The current milestone includes a dependency-free C++20 library, an XPBD distance constraint, skeletal hierarchy evaluation, compact CSR skinning weights, a CPU reference skinning path, reduced surface mesh storage, CPU render-vertex reconstruction, a CPU clustered rigid shape-matching reference solver, and an integrated reduced soft-body CPU solver combining XPBD distance constraints with clustered shape matching. An initial deterministic voxel-based reduced-mesh generator and local inverse-distance render embedding generator are now included. Automatic overlapping shape-matching cluster generation from reduced surface topology is included, with a deformation test that checks local displacement propagation and preservation of distant points. GPU execution and the interactive demo are subsequent milestones.


## Mesh reduction benchmark

Build the optional benchmark with:

```powershell
cmake -S . -B build -DBONEFLEX_BUILD_BENCHMARKS=ON
cmake --build build --config Release --target boneflex_mesh_reduction_benchmark
.\build\Release\boneflex_mesh_reduction_benchmark.exe
```

The default workload creates a 450,000-vertex UV sphere and targets 5,000 reduced control points. The benchmark reports reduction time, CPU reconstruction time, control-point ratio, RMS and maximum rest-pose reconstruction error, and the storage size of the reduced mesh and embedding. Optional arguments are target control-point count, longitude segments, and latitude segments.

## Design principles

- Public API lives under `include/boneflex`.
- Rendering APIs do not leak into the core simulation library.
- CPU implementations act as reference implementations for future GPU kernels.
- Use explicit types and predictable data layouts.
- Keep experiments in separate executables; do not couple them to the library API.

## Shape matching benchmark

Build and run the optional CPU shape-matching benchmark with:

```powershell
cmake -S . -B build -DBONEFLEX_BUILD_BENCHMARKS=ON
cmake --build build --config Release --target boneflex_shape_matching_benchmark
.\\build\\Release\\boneflex_shape_matching_benchmark.exe
```

The default workload uses a 10,000-control-point UV sphere, generates overlapping surface clusters, applies a localized pull near the north pole, and measures the average CPU shape-matching solve time over 10 iterations after warmup. It reports cluster generation time, cluster count, membership count, and RMS/maximum correction. Optional arguments are longitude segments, latitude segments, and cluster size.

## Reduced soft-body solver benchmark

Build and run the integrated XPBD + shape-matching benchmark:

```powershell
cmake -S . -B build -DBONEFLEX_BUILD_BENCHMARKS=ON
cmake --build build --config Release --target boneflex_reduced_soft_body_benchmark
.\\build\\Release\\boneflex_reduced_soft_body_benchmark.exe
```

The default workload simulates 10,000 reduced control points for 60 frames using four solver iterations per frame. It reports average CPU frame time and displacement from the rest pose. Optional arguments are longitude and latitude segments.
