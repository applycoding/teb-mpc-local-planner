# References

Permissively licensed GitHub projects used as **references only**, except OSQP and QDLDL which are also **dependencies** (fetched at the pins in PLAN.md, not copied by hand). Do not copy planner source. Credit any conceptually derived idea in the README. Licenses were checked on 2026-11-10 with `gh api repos/OWNER/REPO --jq .license.spdx_id` and with the raw LICENSE, LICENSE.md, or package.xml when GitHub reported `NOASSERTION` or `null`.

**Outbound license for this demo repo is Apache-2.0**, not MIT. A LICENSE file with the official Apache-2.0 text, a NOTICE file crediting Arpan Bom, and SPDX-License-Identifier: Apache-2.0 headers are required at build time.

This demo must not duplicate existing applycoding repos:

| Repo | What it already is | How this demo differs |
| --- | --- | --- |
| [applycoding/nav2-coverage-planner-compare](https://github.com/applycoding/nav2-coverage-planner-compare) | Global planners (A*, Dijkstra, RRT*, Lazy Theta*) and boustrophedon coverage on **static** Nav2-style costmaps | **Local** receding-horizon TEB-style vs OSQP MPC on a unicycle, static **and moving** obstacles, closed-loop `cmd_vel`, executed-trace metrics. Costmap *conventions* are reused conceptually and reimplemented; this tree does not depend on that repo. No coverage stripes, no global-planner bake-off. |
| [applycoding/ros2-nav2-slam-sim](https://github.com/applycoding/ros2-nav2-slam-sim) | Gazebo + slam_toolbox + Nav2 bringup and a waypoint runner | No Gazebo, SLAM, or Nav2 bringup. A planner comparison library plus a browser sim. |
| [applycoding/rosbag-eval-bench](https://github.com/applycoding/rosbag-eval-bench) | Offline MCAP metric regression | Live local planning, not bag replay. |
| [applycoding/multi-robot-warehouse-planner](https://github.com/applycoding/multi-robot-warehouse-planner) | CBS / MAPF fleet | Single-robot local control. |
| [applycoding/teleop-console-demo](https://github.com/applycoding/teleop-console-demo) | Human teleop of a diff-drive robot | Autonomous local planners. |
| [applycoding/sim-gimbal-stabilizer](https://github.com/applycoding/sim-gimbal-stabilizer) / [applycoding/ekf-gnss-localization](https://github.com/applycoding/ekf-gnss-localization) | Gimbal GNC and GNSS/EKF localization | Different domain. |

## Selected references (7)

### 1. rst-tu-dortmund/teb_local_planner

- URL: https://github.com/rst-tu-dortmund/teb_local_planner
- License: **BSD-3-Clause** (`gh api` SPDX: BSD-3-Clause; root [LICENSE](https://github.com/rst-tu-dortmund/teb_local_planner/blob/noetic-devel/LICENSE) is the BSD 3-Clause text, copyright TU Dortmund Lehrstuhl RST; `package.xml` says BSD). The `humble-devel` tree is a two-package layout (`teb_local_planner`, `teb_msgs`) and `teb_local_planner/package.xml` also says BSD (checked 2026-11-10).
- Borrow: the *idea* of a timed pose band with time, kinematics, and obstacle costs, and the Rösmann et al. Timed-Elastic-Band papers. **Do not copy sources, g2o edges, launch files, or tutorials.** This demo is a teaching single-band optimizer, not a drop-in plugin and not the distinctive-topologies planner.

### 2. osqp/osqp

- URL: https://github.com/osqp/osqp
- License: **Apache-2.0** (`gh api` SPDX: Apache-2.0; [v1.0.0 LICENSE](https://github.com/osqp/osqp/blob/v1.0.0/LICENSE))
- Docs: https://osqp.org (checked 2026-11-10, HTTP 200)
- Release: [v1.0.0](https://github.com/osqp/osqp/releases/tag/v1.0.0) (2025-03-21), commit `236713ce9a56c182ac3230d52108f952afce1523`. Release notes mention QDLDL 0.1.8 and wasm32.
- This is a **dependency** as well as a reference. Use the v1.0 C API at the pin in PLAN.md. Do not copy examples as the MPC formulation.

### 3. osqp/qdldl

- URL: https://github.com/osqp/qdldl
- License: **Apache-2.0** (`gh api` SPDX: Apache-2.0; [v0.1.8 LICENSE](https://github.com/osqp/qdldl/blob/v0.1.8/LICENSE))
- Pin: v0.1.8, commit `138fdac58b9cd1c4137ff1b99152c8108a6cff5b`, as pulled (and hash-pinned) by the OSQP pin. Do not bump independently to 0.1.9.
- NOTICE must also name **SuiteSparse AMD** (BSD-3-Clause, authors Timothy A. Davis, Patrick R. Amestoy, and Iain S. Duff), which is bundled under `algebra/_common/lin_sys/qdldl/amd/` in the **OSQP** tree, not in the standalone QDLDL repository: https://github.com/osqp/osqp/blob/v1.0.0/algebra/_common/lin_sys/qdldl/amd/LICENSE

### 4. ros-navigation/navigation2 `nav2_costmap_2d`

- URL: https://github.com/ros-navigation/navigation2/tree/main/nav2_costmap_2d
- License: **BSD-3-Clause and Apache-2.0** (package.xml lists both; the navigation2 root LICENSE explains mixed SPDX and says unmarked files are Apache-2.0)
- Borrow: costmap layering / inflation *ideas* only. Reimplement the simplified raw-geometry model in PLAN.md. Do not vendor the package. The repo as a whole is `NOASSERTION` because licenses vary by package; do not treat the monorepo as a single Apache grant.

### 5. ros-navigation/navigation2 `nav2_core`

- URL: https://github.com/ros-navigation/navigation2/tree/main/nav2_core
- License: **Apache-2.0** (package.xml)
- Borrow: the idea of a controller API shape (pose, costmap, cmd_vel). v1 is **not** a `nav2_core::Controller` plugin and does not depend on Nav2.

### 6. ros-navigation/navigation2 `nav2_mppi_controller`

- URL: https://github.com/ros-navigation/navigation2/tree/main/nav2_mppi_controller
- License: **MIT** ([package.xml](https://github.com/ros-navigation/navigation2/blob/main/nav2_mppi_controller/package.xml) and [LICENSE.md](https://github.com/ros-navigation/navigation2/blob/main/nav2_mppi_controller/LICENSE.md)). Some source headers say Apache-2.0. **Do not call the package Apache.**
- Borrow: idea-only related local controller (sampling MPC / critics). This demo does **not** implement MPPI. Do not copy critics or code.

### 7. AtsushiSakai/PythonRobotics

- URL: https://github.com/AtsushiSakai/PythonRobotics
- License: **MIT** (LICENSE file starts “The MIT License (MIT)”, copyright Atsushi Sakai and contributors). GitHub SPDX was empty / `NOASSERTION` — use the file: https://github.com/AtsushiSakai/PythonRobotics/blob/master/LICENSE
- Borrow: teaching examples of path planning and kinematic MPC as *ideas*. Do not port the Python.

## Idea-only or exclude

- RainerKuemmerle/g2o — core is described as BSD in the README, but some extensions are LGPL or GPL and CHOLMOD can be GPL. **Idea-only** (graph optimization for TEB). Do **not** vendor g2o or SuiteSparse. The only SuiteSparse code that may ship is the AMD sources already bundled in the pinned OSQP tree, with their BSD-3 notice kept.
- rst-tu-dortmund/mpc_local_planner — **GPL-3.0** (package.xml declares GPLv3). Idea-only if mentioned at all; do not depend on it, do not read sources to implement our MPC.
- CasADi, qpOASES — LGPL. Not dependencies.
- ECOS — GPL. Not a dependency.
- Nav2 packages licensed LGPL. Do not read them to implement this demo. `nav2_dwb_controller` / `dwb_core` is BSD-3-Clause but a different algorithm; idea-only, do not vendor.
