# Upstream sources

Imported on 2026-11-02 from Unitree Robotics:

| Tree | Repository | Commit |
| --- | --- | --- |
| SDK (`include`, `lib`, `examples`, `examples_py`) | https://github.com/unitreerobotics/z1_sdk | `f1af2b42f2a39a5010946e049ad3fe324e2e6f06` |
| `controller` | https://github.com/unitreerobotics/z1_controller | `639bb773bd11b0e8495faf40eadc9e8eb8bc8400` |

The source checkout contains all upstream tracked files. Upstream LICENSE files
remain in their respective trees. SDK CMake is replaced by the native integration;
the original is at `cmake/upstream-sdk-CMakeLists.txt`. The SDK README is this
project's usage guide. All other upstream source and binary files are unmodified.

The build generates two adapted source files without editing upstream originals:

1. `controller/main.cpp`: replace only the hard-coded SDK ports 8072/8071 with
   `Z1_SDK_PORT` / `Z1_CONTROLLER_PORT` environment parameters. The upstream
   control period, FSM registrations, algorithms and communication library remain.
2. `examples_py/arm_python_interface.cpp`: construct the same upstream arm with
   per-instance ports and load. The original boolean constructor remains usable
   through defaults. All upstream bound methods and fields remain.

`dual_arm/include/z1_official/sdk.hpp` initializes the upstream components using
the upstream multi-robot example and the official gripper constructor parameters.
It does not calculate references, inverse kinematics, dynamics or control torques.

The Python launcher creates independent working/config directories because the
upstream controller reads `../config/`. Existing saved poses and taught
trajectories in these runtime directories are not overwritten on restart.
