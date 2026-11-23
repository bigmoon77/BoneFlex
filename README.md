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

The current milestone includes a dependency-free C++20 library, an XPBD distance constraint, skeletal hierarchy evaluation, compact CSR skinning weights, a CPU reference skinning path, reduced surface mesh storage, CPU render-vertex reconstruction, a CPU clustered rigid shape-matching reference solver, an integrated reduced soft-body CPU solver combining XPBD distance constraints with clustered shape matching, and automatic unique-edge XPBD constraint generation from reduced surface topology. An initial deterministic voxel-based reduced-mesh generator and compact-kernel render embedding generator are now included. The embedding uses a Wendland C2 support kernel so control-point influences fade smoothly to zero at their support boundary. Automatic overlapping shape-matching cluster generation from reduced surface topology is included, with a deformation test that checks local displacement propagation and preservation of distant points. The CPU DirectX 11 interactive demo and the first Blender asset import path are implemented. A GPU compute backend, animated Blender pose playback, and skeleton-driven kinematic anchors remain subsequent milestones.

## Mesh reduction benchmark

Build the optional benchmark with:

```powershell
cmake -S . -B build -DBONEFLEX_BUILD_BENCHMARKS=ON
cmake --build build --config Release --target boneflex_mesh_reduction_benchmark
.\build\Release\boneflex_mesh_reduction_benchmark.exe
```

The default workload creates a 450,000-vertex UV sphere and targets 5,000 reduced control points. The benchmark reports reduction time, CPU reconstruction time, control-point ratio, RMS and maximum rest-pose reconstruction error, and the storage size of the reduced mesh and embedding. Optional arguments are target control-point count, longitude segments, and latitude segments.

## Blender asset export and import

The Blender add-on in `blender/boneflex_exporter.py` exports the active skinned mesh and its Armature modifier to two files:

- `.bflex`: BoneFlex's versioned binary interchange asset containing armature-local bind vertices, triangle indices, the bone hierarchy and bind-local transforms, and normalized CSR skinning weights.
- `.glb`: a standard glTF 2.0 binary export for inspection and use by other tools. Animation data is retained in the GLB; the first C++ importer milestone loads the static rig and skinning data from the `.bflex` sidecar.

Install the script in Blender via **Edit > Preferences > Add-ons > Install from Disk**, enable **BoneFlex Asset Exporter**, select the skinned mesh as the active object, then use **3D View > Sidebar > BoneFlex > Export BoneFlex Asset**. The mesh must have an Armature modifier and every vertex must have a positive weight assigned to an exported bone. Apply non-unit bone rest scale before exporting.

For a repeatable smoke test, generate a small two-bone weighted sphere directly from Blender in background mode:

```powershell
blender --background --python .\\blender\\create_test_character.py -- --output .\\test_character.bflex
```

This creates `test_character.bflex` and `test_character.glb` in the requested output location. It uses a simple Root/Upper armature with smoothly blended vertex groups, so it is useful for checking axis conversion, parent ordering, bind-pose skinning, and DX11 import without preparing a character manually. The Blender glTF exporter must be available for the companion GLB. Add `--skip-glb` to generate only the BoneFlex payload; the DevTrace test command uses this mode when Blender is installed and automatically runs the C++ importer demo on the generated asset.

Build and run the C++ import validation demo:

```powershell
cmake -S . -B build -DBONEFLEX_BUILD_EXAMPLES=ON
cmake --build build --config Release --target boneflex_blender_asset_demo
ctest --test-dir build -C Release --output-on-failure
.\\build\\Release\\boneflex_blender_asset_demo.exe .\\path\\to\\character.bflex
```

The demo loads the exported rig and weights, evaluates the bind-pose CPU skinning reference, generates a reduced surface mesh, and reports the resulting control-point and embedding sizes. The importer does not depend on Blender or a graphics API.

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
.\build\Release\boneflex_shape_matching_benchmark.exe
```

The default workload uses a 10,000-control-point UV sphere, generates overlapping surface clusters, applies a localized pull near the north pole, and measures the average CPU shape-matching solve time over 10 iterations after warmup. It reports cluster generation time, cluster count, membership count, and RMS/maximum correction. Optional arguments are longitude segments, latitude segments, and cluster size.

## Reduced soft-body solver benchmark

Build and run the integrated XPBD + shape-matching benchmark:

```powershell
cmake -S . -B build -DBONEFLEX_BUILD_BENCHMARKS=ON
cmake --build build --config Release --target boneflex_reduced_soft_body_benchmark
.\build\Release\boneflex_reduced_soft_body_benchmark.exe
```

The default workload uses a 10,000-control-point UV sphere. It generates XPBD distance constraints from unique surface edges (with a configurable compliance in the benchmark), applies a smooth, localized acceleration to one side for 30 frames, then releases the force for 60 frames. The benchmark reports pull/release frame times and mean, RMS, and maximum displacement for the pulled region, its neighbors, and the opposite side of the sphere. This makes local deformation propagation and post-release recovery measurable. Optional arguments are longitude and latitude segments.


## Reduced soft-body parameter sweep

Build and run the optional parameter sweep:

```powershell
cmake -S . -B build -DBONEFLEX_BUILD_BENCHMARKS=ON
cmake --build build --config Release --target boneflex_reduced_soft_body_parameter_sweep
.\build\Release\boneflex_reduced_soft_body_parameter_sweep.exe
```

The default workload fixes cluster size at 128 and shape-matching stiffness at 0.80, then compares velocity damping values 0.93, 0.94, 0.95, 0.96, and 0.97. This fine sweep narrows the range around 0.95 after the wider sweep showed substantially better post-release recovery at 0.95 than at 0.98–0.999. Each case starts from the same 10,000-control-point sphere, applies a localized 30-frame pull, then releases it for 120 frames. It samples translation-removed RMS deformation in the pulled and far regions every 15 release frames, reporting recovery percentage relative to the pull RMS, pulled-region velocity RMS, and average CPU frame time for each interval. A release-frame-zero sample records deformation and velocity immediately after the pull. The sweep keeps overlap ratio, solver iterations, XPBD compliance, cluster size, and shape-matching stiffness fixed. Optional arguments are longitude and latitude segments.

## DirectX 11 interactive demo

The optional Windows demo visualizes the current CPU reference simulation on a dynamically updated DirectX 11 mesh. It creates a UV sphere, generates a reduced control mesh and render-vertex embedding, then runs XPBD distance constraints and clustered shape matching. The reconstructed render mesh is uploaded to a dynamic vertex buffer each frame.

Build with Visual Studio / MSVC:

```powershell
cmake -S . -B build -DBONEFLEX_BUILD_DX11_DEMO=ON
cmake --build build --config Release --target boneflex_dx11_soft_body_demo
.\build\Release\boneflex_dx11_soft_body_demo.exe
```

Controls:

- Hold the left mouse button on the sphere and drag to pull the surface.
- Release the mouse button to observe shape recovery.
- Press Esc to exit.

This first demo intentionally uses CPU simulation, CPU render-vertex reconstruction, and CPU vertex-buffer updates. It establishes the end-to-end rendering and interaction path before introducing a compute-shader backend.

Pass a Blender-exported `.bflex` asset as the first argument to render and simulate the imported character instead of the built-in sphere:

```powershell
.\\build\\Release\\boneflex_dx11_soft_body_demo.exe .\\path\\to\\character.bflex
```

The imported mesh is normalized to fit the demo camera, reduced to control points, and simulated with the same XPBD + shape-matching pipeline. Control-point bone weights are accumulated through the render embedding. Control points with a dominant bone influence of at least 98% become kinematic anchors and follow the evaluated skeleton pose; blended transition points remain dynamic. A scale-independent downward acceleration is applied to dynamic control points, so unsupported/blended regions sag under gravity while rigidly weighted regions follow the skeleton. The demo uses a lower shape-matching stiffness (0.15), lower XPBD distance compliance (1e-5), and 98% anchor threshold to expose deformation in blended regions; press **G** to toggle gravity. The demo currently drives the first child bone with a procedural oscillation. Press **A** to pause/resume this motion, **G** to toggle gravity, and drag the unanchored surface with the left mouse button. The same path accepts any Blender-exported skinned mesh with an Armature modifier and valid vertex-group weights; the visible result depends on the rig's weight distribution and available dynamic regions. Animation clip import, configurable per-model anchor masks, and more advanced blended anchor constraints remain future work.

## Blender integration roadmap

The first import milestone is implemented: the Blender add-on writes the static deformation data to the versioned `.bflex` payload and optionally writes a standard `.glb` companion. The C++ importer validates and reconstructs the skeleton and CSR skinning data, then the import demo generates the reduced mesh and render embedding.

The GLB currently serves as the portable scene/animation interchange copy. Importing and evaluating animated GLB clips, transferring skeleton motion into kinematic reduced-mesh anchors, and rendering the imported character in the DX11 demo are the next integration steps.
