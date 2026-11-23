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

1. Evaluate the input skeleton pose.
2. Update kinematic anchors from the skeleton.
3. Solve reduced softbody constraints (XPBD and shape matching).
4. Transfer reduced deformation to the render mesh.
5. Upload the deformed mesh through a selectable compute backend.

## Build

Requirements: CMake 3.20+ and a C++20 compiler.

```powershell
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

## Current status

The initial milestone establishes a dependency-free C++20 library, a small XPBD distance-constraint core, and automated tests. Skeleton integration, reduced meshes, shape matching, GPU execution, and the interactive demo are subsequent milestones.

## Design principles

- Public API lives under `include/boneflex`.
- Rendering APIs do not leak into the core simulation library.
- CPU implementations act as reference implementations for future GPU kernels.
- Use explicit types and predictable data layouts.
- Keep experiments in separate executables; do not couple them to the library API.
