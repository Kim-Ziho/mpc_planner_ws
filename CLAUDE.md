# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 언어 설정

항상 한국어로 답변한다.

## Overview

This is the **Topology-Driven MPC (T-MPC++)** workspace — a ROS1 (Noetic) C++ research platform for robot motion planning in dynamic environments. It computes multiple distinct parallel trajectories, each passing dynamic obstacles differently.

Associated publications:
- **T-RO 2024:** *Topology-Driven Parallel Trajectory Optimization in Dynamic Environments*
- **ICRA 2023:** *Globally Guided Trajectory Optimization in Dynamic Environments*

---

## Build & Development Commands

### Initial Setup (one-time)
```bash
./setup_poetry.sh        # Install Poetry, Python deps, and Acados
source /opt/ros/noetic/setup.sh
```

### Build
```bash
# Build a system (e.g., jackalsimulator, rosnavigation, jackal, dingo)
./build.sh <system_name>

# Build AND regenerate solver (Acados only)
./build.sh <system_name> true
```

Internally this runs:
```bash
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DPYTHON_VERSION=3
catkin build mpc_planner_<system_name>
```

### Generate Solver Only
```bash
cd src/mpc_planner
poetry run python mpc_planner_<system>/scripts/generate_<system>_solver.py
```

### Run (VSCode Tasks — Ctrl+Shift+B)
- `JackalSimulator: Build`
- `JackalSimulator: Build and Generate Solver`
- `JackalSimulator: Run Simulator`
- `ROSNavigation: Build`, `ROSNavigation: Run Simulator`

Or via terminal:
```bash
source devel/setup.bash
roslaunch mpc_planner_jackalsimulator ros1_jackalsimulator.launch
```

### Tests
```bash
cd src/mpc_planner
poetry run python -m pytest solver_generator/test/ --cov-report term --cov-config=solver_generator/test/.coveragerc --cov --cov-fail-under=70
```

### ROS/ROS2 Switching
```bash
cd src/mpc_planner
python3 switch_to_ros.py 2   # Switch to ROS2
python3 switch_to_ros.py 1   # Switch back to ROS1
```

---

## Architecture

### System Flow
```
pedestrian_simulator  ──→  obstacle predictions
roadmap               ──→  reference path
                              ↓
                    guidance_planner     (global: topology-distinct trajectories)
                              ↓
                    mpc_planner_<system> (local: T-MPC++ trajectory optimization)
                              ↓
                         /cmd_vel        (velocity commands)
```

### Key Packages

| Package | Role |
|---------|------|
| `mpc_planner` | Core MPC library — solver interface, module system, controller loop |
| `mpc_planner_modules` | Pluggable Python (solver-gen) + C++ (runtime) cost/constraint modules |
| `mpc_planner_solver` | Abstraction over Acados / Forces Pro solvers |
| `mpc_planner_types` | Core data structures shared across packages |
| `mpc_planner_util` | YAML parameter loading (`CONFIG["key"]` macro), math utilities |
| `mpc_planner_<system>` | System-specific ROS node (jackalsimulator, rosnavigation, jackal, dingo) |
| `guidance_planner` | Sampling-based global planner — produces topology-distinct spline paths via PRM |
| `mpc_planner_stepmap` | 3D spacetime occupancy map from costmap + dynamic obstacles, fed into guidance_planner |
| `ros_tools` | ROS1/ROS2 abstraction, visualization markers, profiling, data saving |
| `pedestrian_simulator` | Social-forces pedestrian simulation |
| `roadmap` | Polyline reference path definition and management |
| `decomp_util` | Header-only convex decomposition for static obstacle avoidance |
| `scenario_module` | Safe Horizon MPC with Gaussian mixture uncertainty handling |

### Solver Generation (Python → C++)
Solver generation is a **two-stage** process:
1. **Python (Poetry):** `generate_<system>_solver.py` defines the MPC problem — states, inputs, costs, constraints — by composing `ModuleManager` with modules (e.g., `ContouringModule`, `GuidanceConstraintModule`). This generates C++ code via CasADi + Acados.
2. **C++ (catkin):** The generated solver code is compiled as part of `mpc_planner_<system>`.

Available solver configurations (defined in each `generate_*_solver.py`):
- `configuration_tmpc` — T-MPC++ (default): `GuidanceConstraintModule` + `EllipsoidConstraintModule`
- `configuration_basic` — Basic MPC with ellipsoidal obstacle avoidance
- `configuration_safe_horizon` — SH-MPC with scenario-based uncertainty
- `configuration_lmpcc` — Goal-based MPC without path contouring

**Robot models** (in `solver_generator/solver_model.py`):
- `ContouringSecondOrderUnicycleModel` — standard model (states: `v`, `w`, `x`, `y`, `psi`, `θ_ref`; inputs: `a`, `α`)
- `ContouringSecondOrderUnicycleModelWithSlack` — adds slack variable for SH-MPC
- `ContouringSecondOrderUnicycleModelCurvatureAware` — for CA-MPCC

### C++ Controller Architecture

Each `mpc_planner_<system>` ROS node follows the same pattern:
1. Subscribe to pose, goal, reference path, obstacles
2. Convert to `State` + `RealTimeData` structs (`mpc_planner_types`)
3. Call `Planner::solveMPC(state, data)` → returns `PlannerOutput`
4. Publish `/output/command`

`Planner` owns a `Solver` and a list of `ControllerModule` objects (C++ counterparts of the Python modules). On each call, modules update parameters passed to the solver.

`RealTimeData` carries: `costmap`, `robot_area` (list of `Disc`), `dynamic_obstacles`, `static_obstacles`, reference path, guidance paths.

Configuration is accessed globally via the `CONFIG["key"]` macro (singleton `Configuration` loading settings.yaml via `SYSTEM_CONFIG_PATH(__FILE__, "settings")`).

### StepMap (mpc_planner_stepmap)
`StepMapBuilder::update()` takes a `Costmap2D` + dynamic obstacle predictions and produces a `StepMap` — a 3D (x, y, t) occupancy grid used by `guidance_planner` as a collision model for PRM sampling.

For detailed architecture, class relationships, coordinate systems, dynamic obstacle modeling (Gaussian sampling methods), and parameter reference, see:
**[`docs/stepmap.md`](docs/stepmap.md)**

Key parameters in `settings.yaml` under `step_map:`:
- `resolution_ratio` — coarsening factor relative to costmap
- `dynamic_method` — `"gaussian_independent"` (per-step covariance) or `"gaussian_trajectory"` (velocity-noise trajectory)
- `propagate_uncertainty` — whether to accumulate prediction uncertainty over time

### Guidance Planner Internals
`GlobalGuidance` runs a **Visibility-PRM** in 3D (x, y, t) space-time:
1. Sample nodes avoiding the `StepMap`
2. Connect nodes with straight-line or Dubins paths
3. Graph-search (`graph_search`) to find `n_paths` topology-distinct paths (distinguished by `Homology`, `Winding`, or `UVD`)
4. Fit cubic splines → output as `GeometricPath` / `CubicSpline3D`

Homotopy comparison is selectable in `params.yaml` (`homotopy/comparison_function`).

For detailed algorithm flow, class relationships, topology comparison methods, spline fitting, and parameter reference, see:
**[`docs/visibility-prm.md`](docs/visibility-prm.md)**

### Topic Remapping Conventions
- `/input/state_pose` — Robot pose
- `/input/goal` — Goal position
- `/input/reference_path` — From `roadmap`
- `/input/obstacles` — From `pedestrian_simulator` or perception
- `/output/command` — Velocity commands (`/cmd_vel`)

### Configuration Files
- `mpc_planner_<system>/config/settings.yaml` — MPC weights, horizon `N`, `integrator_step`, solver settings, robot dimensions, `t-mpc/` topology settings, `step_map/` parameters
- `guidance_planner/config/params.yaml` — Sampling count, number of distinct paths (`homotopy/n_paths`), comparison function (`H-signature`, `winding_angle`, `UVD`), `T`/`N` must match MPC
- `pedestrian_simulator/config/configuration.yaml` — Pedestrian scenario selection
- MPC weights are also tunable at runtime via RQT reconfigure (enabled in launch files)

### Adding a New Module
1. **Python side** (`mpc_planner_modules/scripts/`): subclass `Module`, define costs/constraints using CasADi, register sources
2. **C++ side** (`mpc_planner_modules/src/` + `include/`): subclass `ControllerModule`, implement `update()` to set solver parameters
3. Add the module to the desired `configuration_*()` function in `generate_<system>_solver.py`
4. Regenerate solver before building

### Profiling
Enable with `debug_visuals: true` in settings.yaml. Output: `<package>/profiler.json`. View via `chrome://tracing/`.

---

## Commit Message Style

커밋 메시지 형식은 [`docs/commit.md`](docs/commit.md)를 따른다.
