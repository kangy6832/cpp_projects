# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Scope

This directory (`kdl/`) is a standalone CMake project of robot kinematics demos and dynamics utilities within the larger `cpp_projects` Git repository. The sibling `线程池/` projects have separate builds; commands below apply only to `kdl/`. These are command-line programs, not ROS nodes or a colcon package.

## Build and run

Requires CMake 3.16+, a C++17 compiler, `pkg-config`, Orocos KDL/Eigen headers, urdfdom, and ROS 2 `kdl_parser`. CMake finds `orocos_kdl` and `urdfdom` through pkg-config; it locates `kdl_parser` using prefix hints and `/opt/ros/*`. ROS Jazzy is an example environment:

```bash
# Run from kdl/; source the installed ROS distribution if needed.
source /opt/ros/jazzy/setup.bash

cmake -S . -B bin
cmake --build bin --parallel

# Build just one executable.
cmake --build bin --target forward_kinematics

# Configure a debug build in the same build directory.
cmake -S . -B bin -DCMAKE_BUILD_TYPE=Debug
cmake --build bin --parallel
```

CMake defaults to Release on initial configuration, rejects in-source builds, and exports [bin/compile_commands.json](bin/compile_commands.json). Intermediate files go in [bin/](bin/), but executables go in the **kdl project directory**, not in `bin`. Existing root-level binaries and [CMakeCache.txt](CMakeCache.txt) are tracked artifacts; use the source CMake configuration rather than assuming those artifacts are current.

All three executables receive an absolute `DEFAULT_URDF_FILE` pointing to [model/robotic_arm.urdf](model/robotic_arm.urdf) at configure time. Their default chain is `base_link` to `link6`. The chain demo accepts positional `[urdf_file] [root_link] [tip_link]`; FK and IK accept `--urdf`, `--root`, and `--tip`. Use `./forward_kinematics --help` or `./inverse_kinematics --help` for the full CLI.

## Verification

There is no configured linter, automated test suite, or single-test command in this project. `ctest --test-dir bin -N` reports zero registered tests. Build the relevant target and run its CLI as a smoke check:

```bash
./chain_from_urdf
./forward_kinematics --all
./forward_kinematics --deg 0 -30 60 0 0 0

# IK at the bundled model's zero-joint FK pose, with the default zero seed.
./inverse_kinematics --xyz 0.008008 0.038504 1.277540 \
  --rpy -3.124453 0.057182 0 --tries 1
```

FK prints a numerical comparison between the KDL solver and manual segment accumulation. IK prints the solver status and FK-back-substitution position/orientation errors. These are printed diagnostics, not an assertion-based test suite. The example IK target in `--help` can exhaust the default solver's iterations with the bundled model; use the known-pose check above for a basic success case.

## Architecture and conventions

- **Shared model pipeline:** [src/Kinematics/kdl_utils.hpp](src/Kinematics/kdl_utils.hpp) is header-only: urdfdom parses the URDF, `kdl_parser` converts the retained URDF model into a `KDL::Tree`, and `getChain(root, tip)` produces the solver input. Keeping the URDF model alongside the chain provides joint-limit lookup. The header also handles frame accumulation, pose/error calculations, and output formatting.
- **Independent executables:** the chain demo inspects the tree/chain and runs zero-position FK. The FK CLI consumes joint positions and optionally prints intermediate frames. The IK CLI consumes a Cartesian pose, uses `nr_jl` by default (joint-limited Newton–Raphson), supports `nr` and `lma`, and retries from seeded random initial guesses before checking successful solutions through FK. They share helpers, not a central application/runtime.
- **Dynamics extension:** [src/Dynamics/dynamics_utils.hpp](src/Dynamics/dynamics_utils.hpp) builds on the kinematics helpers and Eigen/KDL. It validates link inertia and provides mass matrix, Coriolis/gravity, inverse dynamics, Jacobian, energy, and numerical consistency helpers. Its convention is `tau = H(q) * qdd + C(q, qd) + G(q)`, with default base-frame gravity `(0, 0, -9.81)`. No executable currently includes this header, so a successful normal build does **not** validate it.
- **Model and units:** the URDF is the KDL runtime input, including geometry, axes, joint limits, and inertias. Its fixed `world -> base_link` joint is excluded from the default six-revolute-joint chain. Cartesian positions are metres; angles are radians unless `--deg` is used. For IK, `--deg` applies to RPY and `q0`, not position. FK zero-fills omitted joint values. Limit checks in FK and after IK only warn; only `nr_jl` constrains the solve to the joint limits.
- **Auxiliary assets:** the MuJoCo robot/scene XML, STL meshes, and CSV export under [model/](model/) are separate simulation/visualization/reference assets, not runtime inputs opened by the C++ demos. Do not substitute their parameters for the URDF's without checking consistency.

[CMakeLists.txt](CMakeLists.txt) discovers `.cpp` files directly under `src/Kinematics` and `src/Dynamics`. It detects an entry point by a line beginning with `int main(` (allowing whitespace), then creates an executable named after the source basename. Files without a matching entry point go into an optional `kdl_common` static library linked to every executable. Merely adding a header does not compile it. All targets receive the shared dependencies, the `src` include path, and the default URDF definition.
