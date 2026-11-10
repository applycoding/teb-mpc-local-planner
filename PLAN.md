# teb-mpc-local-planner — v1 plan

Revised 2026-11-10 after an independent review and version, URL, license, unicycle, QP, and scenario checks. Outbound license is **Apache-2.0**, not MIT.

## Goal

Public portfolio demo for Arpan Bom, aimed at Lionsbot Senior SWE (Path Planning) and Senior Navigation & Control Engineer language that asks for local planning, TEB, and MPC. Two obstacle-avoiding **local** planners run on the same simulated differential-drive robot and are compared on success, clearance, time, smoothness, jerk, and compute time:

- A Timed-Elastic-Band-style single-band trajectory optimizer (teaching implementation).
- A linearized kinematic MPC solved with OSQP v1.0.0.

Both consume a simplified Nav2-style costmap plus constant-velocity circular movers. A fixed-step C++17 core is tested with GoogleTest. An optional ROS 2 Humble wrapper is off by default. A GitHub Pages demo shows both planners side by side. The same C++ core, sim, and OSQP (builtin algebra) compile to WASM. TypeScript is UI only.

This is a **portfolio demo, simulation only**. Do not claim that Arpan has shipped a production Nav2 TEB or MPC plugin, that this is a drop-in `teb_local_planner` or `nav2_core::Controller`, or that there are real-robot, warehouse, or certified navigation results. README must call the plant a kinematic teaching model, not a dynamic vehicle and not a real Nav2 controller, and must say “TEB-style single-band optimizer,” not “teb_local_planner.”

Commit author and committer are Arpan Bom <applycoding@gmail.com>; no co-author trailers.

`gh repo list applycoding --limit 200` on 2026-11-10 shows **no** `applycoding/teb-mpc-local-planner`.

## Scope

One unicycle (two-wheel differential drive) in the plane. No slip, no chassis inertia, no Ackermann or omni base in v1.

- Core (`tml_core`): SE(2) unicycle, raw-geometry collision and clearance, costmap raster for display and soft cost, dynamic-obstacle queries, TEB-style solver on a caller-owned workspace, reference rollout, metric formulas. No OSQP, no ROS, no JSON, no `new`/`malloc` in plan/step functions.
- OSQP adapter (`tml_osqp`): `MpcSolver` owns one `osqp_setup`. The control loop only updates data and calls `osqp_solve`. This is the documented heap exception. Do not claim a no-heap MCU core for the MPC path. Setup-once does **not** mean factorization-once; numeric updates may refactor.
- Sim (`tml_sim`, host and WASM): plant at 100 Hz, control at 20 Hz, seeded fixtures, trace buffers. Not part of the core no-heap lint.
- Browser: one WASM module. Canvas 2D only. No chart library. No TypeScript TEB or MPC. Two robots have **separate** planner and OSQP state.
- ROS 2 Humble adapter is optional and does not gate core. It is **not** a `nav2_core::Controller` plugin.

Do not duplicate applycoding repos, and do not depend on, submodule, vendor, or copy them. Reimplement the costmap. `nav2-coverage-planner-compare` is **global** geometric search and boustrophedon coverage on **static** grids (A*, Dijkstra, RRT*, Lazy Theta*). TEB, MPC, DWA, NavFn, and Smac were out of its scope. This repo is **local** receding-horizon control: unicycle kinematics, time-parameterized trajectories, static and moving obstacles, `cmd_vel`, closed-loop metrics, TEB-style vs OSQP MPC. No global-planner bake-off UI and no coverage stripes. `ros2-nav2-slam-sim` is Gazebo + slam_toolbox + Nav2 bringup. `rosbag-eval-bench` is offline bag regression. `multi-robot-warehouse-planner` is CBS/MAPF. `teleop-console-demo` is human teleop. `sim-gimbal-stabilizer` and `ekf-gnss-localization` are other domains. Reimplement TEB-style and MPC math from first principles. Do not copy or vendor `teb_local_planner`, Nav2 MPPI/DWB, g2o, Ceres, or `mpc_local_planner`.

## Architecture

**Decision:** three static libraries. `tml_core` is the kinematics, geometry, TEB, reference, and metric artifact. `tml_osqp` is the only TU that includes OSQP and the only place `osqp_setup` runs. `tml_sim` owns the plant loop and links both. WASM exports wrap `tml_sim`. The Humble node is an adapter that **plans from incoming odometry and publishes `cmd_vel`**; it does not integrate a competing internal plant when real odometry is present. A sim-only launch, if provided, is documented as such.

**Why:** the interview artifact is C++, not a second planner in TypeScript. Keeping OSQP out of `tml_core` makes the no-heap lint on TEB and kinematics honest while the QP setup stays in one owner.

### MPC is linearized kinematic OSQP, not MPPI

**Decision:** v1 MPC is a sparse convex QP from a linearization of the exact discrete unicycle, solved with OSQP. v1 does **not** implement MPPI.

**Why:** (a) “TEB and MPC” in this hiring language means an optimization-based receding-horizon controller, which contrasts with the nonlinear TEB-style band. MPPI would be a second sampling method and would leave no QP solver to pin. (b) OSQP v1.0.0 is Apache-2.0 and its release notes say it can be compiled for wasm32, which is what makes the same-core Pages demo possible. (c) The builtin QDLDL algebra needs no MKL, CUDA, or Fortran. Nav2 MPPI may be cited as an idea-only related local controller. **License (verified 2026-11-10):** `nav2_mppi_controller/package.xml` and `LICENSE.md` say **MIT**; many headers say Apache-2.0. Do not call that package Apache. Do not copy it.

### Solver pin and lifecycle

OSQP **v1.0.0**, commit `236713ce9a56c182ac3230d52108f952afce1523`. Prefer the official release source archive `https://github.com/osqp/osqp/releases/download/v1.0.0/osqp-v1.0.0-src.tar.gz` with `URL_HASH SHA256=ec0bb8fd34625d0ea44274ab3e991aa56e3e360ba30935ae62476557b101c646` (downloaded and hashed 2026-11-10). Do not use a floating tag. Pass `-DOSQP_VERSION=1.0.0` at configure (the cache default is `0.0.0` and does not rewrite the header; the checked-in `version.h` already has `"1.0.0"`, and a clean tree plus this override keeps `osqp_version()` equal to `"1.0.0"`). A compiled check of this pin on 2026-11-10 printed `osqp_version=1.0.0` and solved the one-step QP below.

QDLDL is **not** pinned by the parent commit alone: OSQP’s `qdldl.cmake` FetchContent uses the mutable tag `v0.1.8`. Override that download to commit `138fdac58b9cd1c4137ff1b99152c8108a6cff5b` or the archive `https://github.com/osqp/qdldl/archive/refs/tags/v0.1.8.tar.gz` with `URL_HASH SHA256=ecf113fd6ad8714f16289eb4d5f4d8b27842b6775b978c39def5913f983f6daa` (hashed 2026-11-10). Do not bump QDLDL to 0.1.9. Record both hashes in `cmake/pins.sha256` before the first CI run; configure must fail on mismatch. Hashes are not “recorded at first green configure.”

CMake flags: `OSQP_ALGEBRA_BACKEND=builtin`, `OSQP_BUILD_SHARED_LIB=OFF`, `OSQP_BUILD_DEMO_EXE=OFF`, `OSQP_BUILD_UNITTESTS=OFF`, `OSQP_USE_FLOAT=OFF`, **`OSQP_USE_LONG=OFF` on both native and WASM** (the builtin default is ON on 64-bit hosts and is forced OFF when pointers are not 8 bytes, so x86_64 and wasm32 would otherwise disagree on `OSQPInt`). Match QDLDL’s `QDLDL_LONG`. Compile-time asserts `sizeof(OSQPInt)==sizeof(int)`. Leave `OSQP_EMBEDDED_MODE` unset so the setup/update API remains available. Disable OSQP printing, interrupt, and profiler annotations in the product build (`OSQP_ENABLE_PRINTING=OFF`, `OSQP_ENABLE_INTERRUPT=OFF` for WASM at least).

**v1.0 C API only:** `OSQPSolver`, `OSQPCscMatrix`, `OSQPFloat`, `OSQPInt`, `osqp_set_default_settings`, `osqp_setup`, `osqp_solve`, `osqp_update_data_vec`, `osqp_update_data_mat`, `osqp_warm_start`, `osqp_cleanup`, `osqp_version`. Not the 0.6 `OSQPWorkspace` API. Pass only the upper triangle of P. Check every return code. A test requires `strcmp(osqp_version(), "1.0.0") == 0` with no escape clause.

Horizon `N = 25` and the constraint **sparsity pattern** are fixed. Reserve CSC slots for every potentially nonzero dynamics entry and for four half-plane rows (x and y coefficients), **including entries that are currently zero**. Never prune those zeros. After `osqp_setup`, each tick updates matrix **values** with `osqp_update_data_mat` and `q,l,u` (including equality bounds that carry the affine `c`) with `osqp_update_data_vec`, warm-starts, and solves. P stays fixed. Setup-once is not factorization-once. A test walks a straight-to-turning motion and an inactive-to-active obstacle row and checks that the CSC index map is unchanged. Setup or update failure returns `SolveFailed`, command `(0, 0)`, and does not crash.

Locked OSQP settings: start from `osqp_set_default_settings`, then `verbose=0`, `max_iter=400`, `eps_abs=1e-4`, `eps_rel=1e-4`, `adaptive_rho=OSQP_ADAPTIVE_RHO_UPDATE_DISABLED` (0), **`polishing=0`** (this is the v1.0 field; there is no `polish` setting), `warm_starting=1`, `check_termination=25`, `scaling=10`. **Accept `OSQP_SOLVED` only.** `OSQP_SOLVED_INACCURATE` is treated as failure in v1 (OSQP documents ten-times-looser tolerances). The first-step `(v, ω)` must also be finite; otherwise emergency brake. Brake takes **precedence over rate limits** (it may exceed `α_max` / `a_max` to command zero). This is not a certified safety fallback.

### Frames and plant

Planar `map` frame, +x East, +y North, +z up (REP-103-like ENU). State is SE(2): `(x, y, θ)`. Exact integration of constant `(v, ω)` over `h` uses the stable sinc form, which is algebraically identical to the classical arc map and avoids cancellation at small `ω`:

```
a = ω h,   q = a / 2,
sinc(q) = 1 if |q| < 1e-12 else sin(q)/q,     # plus a short Taylor series for |q| < 1e-4 if desired
x' = x + v h sinc(q) cos(θ + q),
y' = y + v h sinc(q) sin(θ + q),
θ' = wrap(θ + a).
```

Wrap is to `(-π, π]`: after a modulo that yields `[-π, π)`, map `-π` to `+π`. Reject non-finite angles before wrapping. Tests lock `wrap(π) = π`, `wrap(-π) = π`, `wrap(π + 0.1) = π + 0.1 - 2π`.

Do **not** drop the `ω` contribution below `1e-9`. The sinc form is used for the plant, the MPC linearization, and TEB forward residuals, so there is a single map. Independent check: start `(0,0,0)`, `v=1`, `ω=0`, `h=2` ⇒ `(2, 0, 0)`; `v=1`, `ω=π/2`, `h=1` ⇒ `(2/π, 2/π, π/2)`. Five plant substeps of `0.01 s` match one `0.05 s` step of the same constant command within `1e-12`. Reverse and pure rotation are tested.

Track width `L = 0.30 m`. `v = (v_r + v_l)/2`, `ω = (v_r − v_l)/L`. `v_wheel_max = 1.025 m/s`. The command rectangle `|v| ≤ 0.8`, `|ω| ≤ 1.5` lies in the wheel diamond `|v| + (L/2)|ω| ≤ v_wheel_max` (corners lie **on** the diamond). At `(v, ω) = (0.8, 1.5)`, `(v_r, v_l) = (1.025, 0.575)`: **one** wheel saturates, not both. Both wheels saturate only at diamond vertices that are outside this rectangle. Commands are clamped to the rectangle, then (except emergency brake) to `|Δv| ≤ a_max dt` and `|Δω| ≤ α_max dt` with `a_max = 1.0 m/s²`, `α_max = 3.0 rad/s²` (0.05 m/s and 0.15 rad/s per control tick). If `u_prev` is invalid, treat it as `0` before the rate clamp. Circular footprint `robot_radius_m = 0.22`. `dt_plant = 0.01 s`. `dt_ctl = 0.05 s`: one `plan` per tick, hold `(v, ω)` for five plant steps. The core never reads a wall clock; `dt` is an argument.

`Status step_plant(double dt, Twist2D u, Pose2D pose, Pose2D* out)` writes `*out` on success. `dt <= 0` or any non-finite returns `InvalidDt`, does not write `*out`, and does not crash. A convenience overload that returns `Pose2D` is test-only and assumes a valid `dt`.

### Linearization (shared with the plant map)

In an **unwrapped** yaw chart along the horizon (lift the reference relative to the measured yaw; wrap only when publishing poses), the Jacobian of the sinc map is unique. With `d = p' − p`, `e = [cos(θ+q), sin(θ+q)]`, `e_perp = [−sin(θ+q), cos(θ+q)]`:

- `A` is `I` except `A_{0,2} = −d_y`, `A_{1,2} = d_x`
- `B_{xy,v} = h sinc(q) e`
- `B_{xy,ω} = v h²/2 (sinc'(q) e + sinc(q) e_perp)`
- `B_{θ,v} = 0`, `B_{θ,ω} = h`
- At `ω = 0`: `B_{xy,ω} = (v h²/2) [−sin θ, cos θ]`

`c = f(z_ref, u_ref) − A z_ref − B u_ref` in the same chart. No division by `ω_ref`. Finite-difference checks (step `1e-4`, away from the wrap cut) must match. Independent check on 2026-11-10: max abs error vs central differences was `~6e-11`.

### TEB-style optimizer

**Decision:** one band, not distinctive topologies. `M_max = 30`, default `M = 25`. Pose 0 is fixed to the current robot pose and is **not** a free variable. Free vector: poses `1..M-1` and `M-1` durations, `4(M-1)` unknowns (96 at `M=25`, 116 at `M_max`). `ΔT_i` is kept in `[0.05, 0.30]` s by projection. Goal pose is **not** hard-pinned.

`(v, ω)` is **not** assumed to exist for an arbitrary increment. Unwrapped `a = θ_{i+1} − θ_i` (local lift; `|a|` is encouraged to stay well below `π`; infeasible iterates are still defined). `ω = a / ΔT` for `ΔT > 0`. Let `b = ΔT sinc(a/2) [cos(θ_i + a/2), sin(θ_i + a/2)]`. Signed least-squares `v = (b · Δp) / (b · b)` if `b · b > 0`, else `v = 0`. Nonholonomic residual is `Δp − b v` (lateral infeasibility is visible, reverse travel keeps its sign). Reject non-finite or `ΔT <= 0` before division. A 180° spin with `Δp = 0` is `v = 0` and the chosen sign of `a`.

Objective / residuals (Levenberg-Marquardt, caller-owned dense workspace sized for `M_max`; do not put the workspace on a tiny WASM stack):

- Time: residual `sqrt(w_time * ΔT_i)` so the squared term is `w_time ΔT_i`
- Velocity / omega / accel / alpha boxed as soft squared violations, with first accel/alpha measured against `u_prev`
- Kinematics: the residual `Δp − b v` and a heading-consistency term already implied by `a`
- Obstacle: squared hinge on `max(0, d_min − signed_distance_raw)` at each pose, **and** at `dt_plant` samples along each band segment, against raw geometry (below). The hinge is a soft penalty, not a hard guarantee. A final collision check on the first interval can reject the command.
- Terminal pose: quadratic on xy and yaw (wrapped)

Default weights (retune only in the same commit that locks scenario bounds): `w_time=1`, `w_lim=10`, `w_kin=50`, `w_obs=20`, `w_goal_xy=40`, `w_goal_yaw=8`. Max 25 iterations. `λ = 1e-3`, ×10 on reject, ÷3 on accept, cap `1e8`. Stop on a small **nonnegative** improvement of an **accepted** iterate (`rel drop < 1e-4`). Factorization failure or non-finite cost ⇒ `EmergencyBrake`. After a valid solve, the first interval’s projected `(v, ω)` (limits; rate clamp unless already braking) is held for one tick.

Both planners receive the same optional polyline, pose, goal, world, limits, and evaluator. TEB initializes / shifts the band along the same goal-directed or lookahead reference that MPC uses, then optimizes. This is not a corridor-homotopy claim.

### Linearized MPC

State `z = [x, y, θ]` (unwrapped along the horizon), control `u = [v, ω]`. `N = 25`, `h = 0.05 s` (1.25 s). Stages `z_0 … z_N`, `u_0 … u_{N-1}`, with `z_0` fixed to the current pose. First rate constraint is against the actually applied `u_prev`. Cost: `‖z − z_ref‖_Q² + ‖u − u_ref‖_R² + ‖Δu‖_{Rd}²` plus terminal `Q_N`. Defaults: `Q = diag(2, 2, 0.4)`, `R = diag(0.05, 0.05)`, `Rd = diag(0.4, 0.15)`, `Q_N = diag(8, 8, 1)`. Box constraints on `v, ω` and on `|Δv|, |Δω|`.

**Half-planes (four rows, translation included).** For a unit normal `n` and a support point `o` on the obstacle boundary, `n · (p − o) >= r_eff`, i.e. **`n · p >= n · o + r_eff`**. Omitting `n · o` is incorrect (the constraint would be origin-relative). `r_eff = robot_radius + extra_margin` for a static square (o = closest point on the occupied square to the reference position; `n` points from o toward that reference). `r_eff = robot_radius + obs_radius + extra_margin` for a mover circle (o = predicted center). Overlap or zero distance: if the reference is inside, `n` is from the obstacle outward along a deterministic axis (larger `|x|` then `|y|`, then +x). Ties: lowest obstacle index, then static before mover.

Four slots, filled in this order: two nearest static support points and two nearest movers, sorted by predicted signed clearance at that stage, stable IDs. Unused rows are `0 · p >= −1` (upper bound +∞) but **keep** their x/y CSC slots. Planners query **raw** occupied squares and raw mover circles, not the inflated raster, so radius and margin are applied once. Dummy or opposing planes can still cut a gap; if a solve is `OSQP_SOLVED` but the first-interval forward rollout (plant, `dt_plant` samples) collides, command emergency brake.

**Reference is independent of the TEB band** and is shared as a function. Cruise `v_cruise = 0.60 m/s`. If the polyline is empty, heading `P`: `ω = clamp(2.0 * wrap(bearing − θ), ±ω_max)`, `v = v_cruise` tapered linearly to 0 inside 0.6 m of the goal **xy**. If a polyline is present, the same law tracks a monotonically advancing arclength with lookahead 0.8 m (project onto the current segment, break ties by earliest arclength, skip zero-length segments; at the endpoint, hold the last heading then apply the terminal yaw policy). **Terminal yaw:** when xy error `< 0.40 m`, fade `v` and rotate to goal yaw with bounded `ω`; if already within both tolerances, command `(0, 0)`. Bearing at zero xy distance is not used. Reverse is allowed when the heading error exceeds `2π/3` and xy error exceeds 0.5 m (S4). Warm-start shifts the previous solution by one step and fills the tail with the reference.

### Costmap, movers, collision, clearance

Reimplement a simplified costmap. Occupancy `Free` / `Occupied` / `Unknown` is stored separately from a `uint8` display/soft-cost field: 0–252 soft, 253 inscribed, 254 lethal, 255 unknown. Fixture resolution `0.05 m`. Occupied cells are solid squares (cell `(i,j)` covers `[origin_x + i res, origin_x + (i+1) res] × [origin_y + j res, origin_y + (j+1) res]`; index `j * width + i`). Unknown defaults to lethal (`UnknownPolicy::Lethal`). Out-of-map is lethal for collision (a center outside the map collides). The raster is for visualization and optional TEB soft cost (linear decay from the hard radius out to `robot_radius + extra_margin + 0.30 m`). Soft cost is not a second collision radius.

**Collision and planner distances query raw geometry**, not inflated cell indices. `extra_margin_m = 0.05`. A pose at time `t` collides iff any of: distance from center to the union of occupied (and unknown-as-lethal) **squares** `< robot_radius + extra_margin`; distance to a mover center `< robot_radius + obs_radius + extra_margin`; center is out of map. Tangency (`==`) is **not** a collision. Segment and predicted-band checks sample at `dt_plant` (a discrete predicate, not a continuous certificate). Movers in fixtures are bounded: `radius <= 0.50 m`, `|v| <= 0.50 m/s`, so they cannot fully tunnel a robot diameter in one plant step.

Dynamic obstacles are circles `(x, y, radius, vx, vy)` with positions **anchored at scenario t0**: `p(t) = p0 + v * t` where `t` is the global scenario time (`World.t`). Prediction at horizon step `k` uses `t + k h` (MPC) or the cumulative band time (TEB). Do not add elapsed time twice.

**clearance_m:** min over executed plant steps of (distance to occupied squares − `robot_radius`) and (distance to each mover center − `robot_radius` − `obs_radius`), then `max(0, ·)`. Margin is **not** subtracted again. Collision forces the reported clearance to 0 even if the physical (no-margin) number is still slightly positive. Quiet NaN / JSON `null` only when there are no occupied or unknown-lethal cells, no movers, and the whole executed trace stays in-map (S1). A nearby lethal map edge prevents null.

### Metrics (executed closed-loop trace)

Computed on plant samples, not on the planned band alone. Goal success does **not** require zero speed. Integration stops at the first plant sample that meets the goal, or at `T_max`, or at collision.

- **success:** 1 iff no collision at any plant step (including t0) and some pose is within `goal_tol_xy = 0.25 m` and `goal_tol_yaw = 0.30 rad` at `t <= T_max` (per scenario, default 30 s). Else 0 and a reason: `collision`, `timeout`, `invalid_start`, `invalid_input`. An already-at-goal, collision-free t0 is success with `time_to_goal = 0` and smoothness 0; do not spin. Collision / invalid input take precedence over success.
- **clearance_m:** as above.
- **time_to_goal_s:** first timestamp inside the goal tolerance, minus t0. Null if unsuccessful.
- **smoothness_rad_per_m:** `(1/L) * Σ |wrap(θ_{i+1} − θ_i)|`, `L` = polyline length of the trace. Straight line is 0. Two 2 m legs with one 90° corner equal `(π/2)/4`. Stationary spins with tiny `L` are capped by the `L < 1e-6` rule. If `L < 1e-6` and success is 0, null. A successful stay at an already-at-goal start (`L = 0`) is smoothness 0.
- **jerk_m_s3:** RMS of `(v_{k+1} − 2 v_k + v_{k-1}) / dt_plant²` after the first two samples, using the **held command** at each plant step (command-signal jerk, not a chassis model; jumps at control ticks are expected). Also report `yaw_jerk_rad_s3` on `ω`; do not gate yaw jerk in v1. Trace shorter than 3 samples ⇒ null.
- **compute_ms:** wall clock of each `plan`/`step`, shown in the demo and JSON. **Never golden-compare it.** TEB iteration counts and OSQP iteration counts may be asserted under generous caps (TEB `≤ 25`, OSQP `≤ 400`).

JSON never contains NaN or Infinity: undefined metrics are `null`. Include `schema_version`, units, planner id, status, seed, and fixture id.

### Scenarios (locked 2026-11-10)

Fixtures live in `fixtures/scenarios/` (JSON + a short README). Time origin is 0. Initial command is `(0, 0)`. Map for S1/S3/S4 is a large free rectangle (at least `[-2, 12] × [-4, 4] m`) so the robot stays in-map. Default TEB and default MPC must both succeed. The **mutant** is each planner with **all obstacle-dependent terms disabled** (TEB `w_obs = 0` and no hinge; MPC half-planes inactive; no obstacle-aware reference dodge) while the collision evaluator stays on. The mutant must not get a “smart” brake-from-obstacles path; it may still rate-limit and use the goal-directed reference.

Independent prototype (shared plant, not the product solvers) on 2026-11-10:

| Id | Locked scene | Locked gates (both product planners) | Prototype evidence |
| --- | --- | --- | --- |
| S1 | Open 8 m: start `(0,0,0)`, goal `(8,0,0)`, no obstacles | success; `time_to_goal < 25 s`; clearance null; smoothness `< 0.2` rad/m | Accel-ramp lower bound **10.38 s**. A 0.55 m/s goal-directed run finished in **14.65 s**, smoothness 0. 25 s leaves margin for 0.60 m/s cruise, taper, and yaw. |
| S2 | Staggered boxes: `[-1, 3.5]×[0.40, 3.0]` and `[2.5, 7.0]×[-3.0, -0.40]`. Wall-to-wall opening **0.80 m**. Start `(0,0,0)`, goal `(8,0,0)` | success; `time_to_goal < 30 s`; physical clearance `> 0` | Open-loop centerline (`ω=0`, accel to `v_max`) finished in **10.10 s** with clearance **0.18 m**. Inflation/MPC `d_min` leaves a 0.26 m lethal-center gap; do not silently widen the fixture if a planner fails. |
| S3 | Open floor, start `(0,0,0)`, goal `(8,0,0)`, one mover: circle **r = 0.45 m** at **(4.0, 1.45) m**, **v = (0, −0.15) m/s**. Collision window on the line `y=0` near `x=4` covers the accel-to-0.8 arrival (~5.4 s) | success; clearance `> 0`; `time_to_goal < 30 s`; max lateral offset over the trace `> 0.3 m` | Straight mutant cruise 0.8 **collides**. A scripted south dodge (`y ≈ −1.3` then to the goal) succeeded in **20.08 s**, clearance **0.67 m**, lateral **1.57 m**. |
| S4 | No obstacles, start `(0,0,0)`, goal `(−2.0, 0.5, π)` | success; `time_to_goal < 25 s` | A goal-directed run with reverse-allowed heading finished in **5.50 s**. Product planners must use the terminal yaw policy. |

Brake-only must not reach the S1 goal inside `T_max`. A test asserts the mutant **collides** on S3 (not merely times out) and both defaults succeed on that same fixture. Do not repair a failed gate by widening S2 or slowing the S3 mover; retune weights or document a requirement change in the same commit.

Optional polyline fixtures, if used, are authored and identical for both planners. A seed field is recorded but inert for these four (no planner RNG). Same-build, same-platform traces match within `1e-9` absolute on states. Native and WASM are different platforms; do not golden-compare them against each other.

### WASM, Pages, ROS, determinism

emsdk **7.0.0**, manager commit `1693ce7359fc9694f198694f902a2cc061239b86`, toolchain release `6afbd83493d647051772d491124187d4e9a0cda9`. Vite `base` is `/teb-mpc-local-planner/`. `locateFile` and every `.wasm` / fixture URL must honour that prefix. `extern "C"`: `tml_reset` (resets both robots, OSQP warm starts, and time), `tml_set_scenario(int id)` (invalid id returns a documented error), `tml_step_ticks(int n)` with **hard cap 20 control ticks** (1.0 s) per call (negative or zero is a no-op; oversized is clamped; return the ticks actually run), a fixed-capacity trace buffer, and a metrics snapshot. Layout, stride, and pointer lifetime after memory growth are documented; reacquire views if memory grows, or fix memory. UI work is bounded per animation frame (one call). Labels “TEB-style” and “MPC (OSQP)”. Playwright: production `vite preview` at the Pages base (not only `/`), both labels, a short **explicit** tick run that changes a **non-compute_ms** numeric metric, wasm served as `application/wasm`, module instantiation, and `docs/demo.png`. CI may upload a fresh screenshot artifact; it must **not** commit or push `docs/demo.png`. A reviewed local copy is checked in once. Viewport, S3 or S1, and tick count are documented in the Playwright test. `pages.yml` deploys the **tested** `web/dist` from `main` only after core and web of **that SHA** succeed (needs the artifact; do not rebuild untested). No `pull_request_target`. PR jobs are read-only.

Humble node (off by default): subscribe `nav_msgs/OccupancyGrid`, `nav_msgs/Odometry`, `geometry_msgs/PoseStamped` goal, and movers as `geometry_msgs/PoseArray` **plus** `std_msgs/Float64MultiArray` with length `3 * n` and the same index order (`radius, vx, vy` in meters and m/s, map frame). Both messages are applied only when they arrive as a consistent pair (same count; if one is stale `> 0.2 s` or counts disagree, keep the last consistent set or, at t0, treat as no movers). No custom message in v1; the pairing limitation is documented. Publish `geometry_msgs/Twist` on `/cmd_vel` and `nav_msgs/Path`. OccupancyGrid conversion: `-1` unknown, `>= 50` occupied, else free (standard ROS convention). Plan from odometry; do not also step an internal plant in that mode. Paused until a start flag. Command timeout publishes zero. Jazzy is not v1. `docs.ros.org` was intermittently blocked from some reviewers; use the Humble source docs and package names, and pin the image digest at the first green run.

## Pinned stack

Pins are exact. CI does not float `latest`. FetchContent uses commit SHAs or URL + `URL_HASH`. Checked 2026-11-10 against GitHub releases, PyPI, and npm (Vite 8.3.7 is current on both GitHub and npm; Playwright, TypeScript, Vitest, pytest, pybind11 likewise).

| Component | Pin |
| --- | --- |
| Language | C++17, no GNU extensions, no Eigen in the core. OSQP (C) is the only numeric solver. |
| Core CI | `ubuntu-24.04`, explicitly `g++-13` / `gcc-13`, Python 3.12 (image updates; record the exact compiler once). |
| Humble CI | `ros:humble-ros-base` digest pinned at first green run, linux/amd64. g++ 11, system Python 3.10 only because the image ships it (3.10 is EOL upstream as of 2026-10-01). Does not gate core. Subsequent apt installs can still float; document them. |
| CMake | minimum 3.22; current GitHub stable **4.4.5** (https://github.com/Kitware/CMake/releases/tag/v4.4.5). PyPI’s `cmake` package may lag (4.4.4 was seen). The minimum is a compatibility promise, not the CI-installed version. |
| GoogleTest | v1.18.0, `063de7e9578f82b369302001269680b4b1553359` |
| pybind11 | v3.1.0, `97bf890db679505a14dfe547a5e77bb2bd05dc90` (optional, off by default; omit from the repo until bindings have a purpose) |
| nlohmann/json | v3.12.0, `55f93686c01528224f448c19128836e7df245f72` (MIT; NOTICE and the MIT permission text if distributed). Scenarios and tools only. |
| OSQP | v1.0.0, commit and official archive hash above, flags above. QDLDL `138fdac58b9cd1c4137ff1b99152c8108a6cff5b` / archive hash above. |
| pytest | 9.2.0 only if Python benches exist; otherwise omit. |
| Node | **26.13.0** Active LTS (Lithium, bundled npm 11.20.0). Node 20 is EOL (ended 2026-04-30). Install this Node; do not trust the runner’s default. |
| TS / Vite / Vitest / Playwright | 7.0.2 / 8.3.7 / 5.1.0 / `@playwright/test` 1.65.0, Chromium only (that package’s revision). Lockfile via `npm ci`. |
| emsdk | 7.0.0, manager `1693ce7359fc9694f198694f902a2cc061239b86`, toolchain `6afbd83493d647051772d491124187d4e9a0cda9` |
| ROS apt | rclcpp, nav-msgs, geometry-msgs, std-msgs, ament-cmake. visualization-msgs / rosidl generators only if actually used. Humble only. |
| Plots | Canvas 2D in this repo. No chart library. No GPL charts. |
| Actions | Full commit SHAs in `.github/workflow-pins.md` when first written (checkout, setup-node, artifact, Pages, any emsdk helper). Do not invent SHAs in this plan. No `pull_request_target`. |

## File layout

```text
LICENSE  NOTICE  README.md  CMakeLists.txt
cmake/pins.sha256
cpp/include/tml/   cpp/src/core/   cpp/src/osqp/   cpp/src/sim/
cpp/test/   cpp/tools/   tools/check_no_heap.py
fixtures/scenarios/{s1,s2,s3,s4}.json  fixtures/README.md
web/   (Vite, Canvas 2D, playwright.config.ts, e2e)
ros2/tml_planner/   (off by default)
docs/demo.png
.github/workflows/{core,web,ros2-humble,pages}.yml
.github/workflow-pins.md
```

Namespace `tml`. `TML_BUILD_TESTS` defaults ON. `TML_BUILD_PYTHON` and `TML_BUILD_ROS` default OFF and must not appear in the ordinary core configure graph. Core ctest does not need Node, ROS, or a Python module. `tools/check_no_heap.py` may run under the image Python 3.12 in the core job; that is a CI script, not a core link dependency.

## Interfaces

```text
namespace tml {

struct Pose2D { double x, y, yaw; };
struct Twist2D { double v, omega; };

struct Limits {
  double v_max;         // 0.8
  double omega_max;     // 1.5
  double a_max;         // 1.0
  double alpha_max;     // 3.0
  double v_wheel_max;   // 1.025
  double track_width_m; // 0.30
};
struct RobotParams { double radius_m; Limits limits; }; // radius 0.22

enum class Occupancy : uint8_t { Free, Occupied, Unknown };
enum class UnknownPolicy { Lethal, Free };

struct Inflation {
  double extra_margin_m;     // 0.05; used in collision and planner d_min, not subtracted again in clearance
  bool collide_inscribed;    // reserved; v1 collision uses raw distance, not the 253 raster
  UnknownPolicy unknown_policy;
};
struct Costmap; // occupancy + uint8 display costs, resolution 0.05, origin, width, height

struct DynamicObstacle { double x, y, radius, vx, vy; }; // pose at t0
struct World { const Costmap* map; const DynamicObstacle* movers; int n_movers; double t; };

struct Band { Pose2D poses[30]; double dt[29]; int M; };
struct TebConfig { int M; int max_iters; double dt_min, dt_max; /* weights */ };
struct TebWorkspace; // caller-owned, fixed arrays; not a WASM stack temporary

struct MpcConfig { int N; double h; int n_halfplanes; /* Q, R, Rd, Q_N */ };
struct MpcSolver; // owns OSQPSolver*; setup() once; step() does not allocate in authored code

enum class PlannerId { TebStyle, MpcOsqp };
enum class Status { Ok, InvalidDt, InvalidInput, SolveFailed, NearGoal, EmergencyBrake };

struct PlanRequest {
  Pose2D pose, goal;
  const Pose2D* polyline; int n_polyline; // optional, may be 0
  World world;
  double dt;
  Twist2D u_prev;
};
struct PlanResult {
  Twist2D twist;
  Pose2D predicted[30];
  int n_predicted;
  Status status;
  int iterations;
  const char* reason; // static string, never a dangling pointer
};

struct Metrics {
  int success;
  const char* reason;
  double clearance_m;      // NaN only in-memory; JSON null if undefined
  double time_to_goal_s;
  double smoothness;
  double jerk;
  double yaw_jerk;
  double compute_ms;       // reported, never golden-compared
};

Status step_plant(double dt, Twist2D u, Pose2D pose, Pose2D* out);
Status teb_step(const PlanRequest&, const TebConfig&, TebWorkspace&, PlanResult*);
Status mpc_step(MpcSolver&, const PlanRequest&, const MpcConfig&, PlanResult*);

struct Scenario;
struct Trace;
int run_scenario(uint32_t seed, const Scenario&, PlannerId, Trace*);
}

extern "C" int tml_reset(void);
extern "C" int tml_set_scenario(int id);
extern "C" int tml_step_ticks(int n); // n clamped to [0, 20]
```

Validate pose/goal/`u_prev`, `dt`, map dimensions, mover pointers/count/radius/velocity, polyline, limits, and capacities **before** touching workspaces. `N`, `M`, and half-plane counts that are not the locked v1 values are `InvalidInput` (v1 does not accept “any smaller N”). Every `PlanResult` field is set on failure. `teb_step` and `mpc_step` do not call `new`/`malloc`. `MpcSolver` setup may, once. `NearGoal` still returns a finite clamped twist and is success-eligible. Start already in collision ⇒ `invalid_start`, command `(0, 0)`.

The frozen-QP unit test uses a **deliberately non-optimal** feasible baseline. It compares `0.5 xᵀ P x + qᵀ x` on that baseline versus the returned decision under identical `P,q,A,l,u`. A strict decrease is required on that fixture; an already-optimal reference may be equality and is a separate case. This is **not** a claim about nonlinear plant cost.

## Edge cases

- `dt <= 0`, NaN/Inf pose, yaw, or twist: `InvalidDt` or `InvalidInput`, brake, no crash, TEB workspace and plant pose unchanged.
- Start in collision or overlapping a mover at `t0`: `invalid_start`, no integration.
- OSQP not set up, setup/update error, or not `OSQP_SOLVED`: `SolveFailed` or `EmergencyBrake`, `(0, 0)`, brake precedes rate limits.
- Sinc map at both signs of tiny `ω`, reverse, pure rotation, wrap at `±π`.
- `ΔT` clamp, wheel box, and accel box on the integrated command, except emergency zero.
- Empty polyline vs polyline; already at goal; terminal yaw after xy arrival; bearing never evaluated at zero distance.
- Mover miss at `t=0` and hit later, and the reverse.
- Out-of-map center is collision. Unknown-as-lethal default.
- Half-plane `n · o` term; dummy rows stay inactive; reserved CSC zeros.
- Band local minimum: S2’s opening is 0.80 m and the centerline is a feasible homotopy; do not claim global optimality.
- `M`, `N`, and half-plane counts other than the locked values: `InvalidInput`.

## Test plan

- Unicycle identities: straight `(2,0,0)`, quarter-circle `(2/π, 2/π, π/2)`, sinc vs five substeps, wrap, reverse, tiny-`ω` both signs.
- Inverse / projection: exact arcs recover `(v, ω)`; a sideways increment has a large residual and a defined `v`; backwards motion keeps sign.
- Jacobian FD (step `1e-4`) matches the locked A,B; affine reproduces the reference in the unwrapped chart.
- Frozen QP: non-optimal baseline, successful `osqp_solve` strictly reduces `0.5 xᵀP x + qᵀx`; `osqp_version` is `"1.0.0"`; `OSQP_SOLVED_INACCURATE` is not accepted by the product wrapper (the unit QP itself should be `OSQP_SOLVED`).
- TEB-style, empty map: a crooked initial band toward the goal decreases the accepted objective within 25 iterations and the kinematics residual shrinks.
- Geometry: tangency is not a collision; a pose just inside `r + margin` is; clearance does not subtract margin; mover collision is time-aware; `n · o` is tested on an offset circle (a missing term fails).
- Metrics: L-path smoothness `(π/2)/4` within `1e-12`; clearance 0 on reported collision; already-at-goal success and smoothness 0; short trace ⇒ null jerk; S1 ⇒ null clearance.
- Closed loop: default TEB and default MPC succeed on locked S1–S4; mutant **collides** on S3; brake-only fails S1; same-build traces match within `1e-9`.
- CSC index map stable across a straight-to-turn update. `tools/check_no_heap.py` on `tml_core`. `tml_osqp` setup is the allocation exception.
- Playwright as above. Optional Humble smoke does not gate core.
- Do not golden-compare `compute_ms`.

## Numbered success criteria

1. `cmake -S . -B build && cmake --build build && ctest --test-dir build` passes on ubuntu-24.04 with g++-13 and no ROS.
2. Exact unicycle tests match the straight and `ω = π/2` hand cases, wrap, reverse, tiny-`ω`, and five-substep consistency, using the sinc map.
3. A frozen no-obstacle QP with a deliberately non-optimal feasible baseline strictly reduces `0.5 xᵀ P x + qᵀ x` after `OSQP_SOLVED`.
4. On an empty map, the TEB-style accepted objective on a crooked band decreases within 25 iterations and the kinematic residual drops.
5. Default TEB-style and default OSQP MPC both succeed on locked S1–S4 within the locked bounds (S1 time `< 25 s`, S3 lateral `> 0.3 m` and time `< 30 s`, S2 clearance `> 0`).
6. The obstacle-disabled mutant **collides** on the locked S3 fixture; both defaults succeed on that same fixture.
7. Brake-only `(0, 0)` does not reach the S1 goal before `T_max`.
8. Smoothness of a 2 m + 2 m right-angle path equals `(π/2)/4` rad/m; reported clearance is 0 on collision; already-at-goal success has smoothness 0.
9. `osqp_version()` equals `"1.0.0"`, FetchContent uses the locked OSQP and QDLDL hashes, builtin algebra, doubles, and `OSQP_USE_LONG=OFF`.
10. Invalid `dt`, NaN pose, in-collision start, and missing OSQP setup return the documented status and a zero command without crashing. `step_plant` reports `InvalidDt` through `Status`.
11. Core plan/step sources do not call `new`/`malloc`; the lint fails if they do. OSQP setup stays inside `MpcSolver`. Half-planes include the `n · o` term.
12. Pages demo is served at `/teb-mpc-local-planner/`, WASM is the same C++ core plus OSQP builtin, and Playwright writes `docs/demo.png` with both planner labels and a changed non-timing metric.
13. Optional Humble smoke is a separate job and is not required for the core or Pages gates. Jazzy is not built.
14. `LICENSE` is the verbatim Apache-2.0 text. `NOTICE` starts with `teb-mpc-local-planner, Copyright 2026 Arpan Bom` and lists OSQP (Apache-2.0), QDLDL (Apache-2.0), SuiteSparse AMD (BSD-3-Clause) bundled under the OSQP tree (not the standalone QDLDL repo), plus GoogleTest, nlohmann/json (MIT text if shipped), and Emscripten/runtime notices if they ship in the Pages bundle. Authored files carry `SPDX-License-Identifier: Apache-2.0`.
15. README states simulation only, kinematic teaching model, “TEB-style single-band optimizer,” and that this is not `teb_local_planner` and not a Nav2 controller plugin.
16. The UI and README do not host a global-planner comparison or coverage stripes, and the tree does not depend on `nav2-coverage-planner-compare`.
17. Commit author and committer are Arpan Bom `<applycoding@gmail.com>`, with no co-author trailer.
18. The tree contains no g2o, Ceres, MPPI, qpOASES, CasADi, acados, or GPL `mpc_local_planner` sources.
19. Tests do not golden-compare `compute_ms` or cross-compiler OSQP iterates, and do not accept `OSQP_SOLVED_INACCURATE` as a product success.

## Out of scope

MPPI. Distinctive-topology or multi-band TEB. g2o or Ceres. CasADi, acados, qpOASES. `nav2_core` plugin. Gazebo. SLAM or AMCL. Ackermann or omni bases. 3D. Real robot hardware. Windows CI. Jazzy CI. Copying `teb_local_planner`, Nav2, PythonRobotics, or g2o sources. GPL `mpc_local_planner`. Production or certified local-planning claims. A second planner engine in TypeScript. Depending on `applycoding/nav2-coverage-planner-compare`. Continuous (unsamped) collision certification. Claiming OSQP WASM parity before the first green web job.

## Documentation and honesty

README credits ideas, not copied code. See `REFERENCES.md`. Primary idea references (permissive): `rst-tu-dortmund/teb_local_planner` (BSD-3-Clause), `osqp/osqp` and `osqp/qdldl` (Apache-2.0, also dependencies), `nav2_costmap_2d` (BSD-3-Clause and Apache-2.0), `nav2_core` (Apache-2.0), `nav2_mppi_controller` (**MIT**, idea-only, we do not implement MPPI), `AtsushiSakai/PythonRobotics` (MIT). Exclude as dependencies: `mpc_local_planner` (GPL-3.0), CasADi and qpOASES (LGPL), ECOS (GPL), g2o (mixed BSD/LGPL/GPL; idea-only, do not vendor), Nav2 LGPL packages.

## License

Apache-2.0. `LICENSE` is the verbatim file from https://www.apache.org/licenses/LICENSE-2.0.txt. `NOTICE` begins with `teb-mpc-local-planner, Copyright 2026 Arpan Bom` and lists the dependency notices above. SPDX on authored files. Do not use MIT. Do not relabel third-party files. The Pages bundle is not “exclusively Apache-2.0” if OSQP, AMD, JSON, or Emscripten glue ships in it.

## Build and CI

- `core.yml`: ubuntu-24.04, g++-13, CMake, ctest, no-heap lint, OSQP version assert. Merge gate. `TML_BUILD_ROS=OFF`, `TML_BUILD_PYTHON=OFF`.
- `web.yml`: Node 26.13.0, emsdk 7.0.0, Vite `base=/teb-mpc-local-planner/`, Vitest, Playwright Chromium. Upload `web/dist` and a screenshot artifact. Do not commit `demo.png` from CI. A first-run OSQP WASM smoke (setup, matrix update, repeated solve, cleanup) is part of this job.
- `pages.yml`: `main` only, after core and web of the same SHA, deploy that `web/dist`.
- `ros2-humble.yml`: non-required. Digest pinned at first green run. Does not block core or Pages.

Pin GitHub Actions to full SHAs in `.github/workflow-pins.md` when first written.

## Risks

- **OSQP on wasm32.** v1.0.0 notes say it builds; emsdk 7 + this application still has to link. The first web job must prove setup, update, solve, and cleanup. Do not swap solvers or drop MPC from the browser.
- **OSQP determinism.** Locked settings should repeat on one compiler. Cross-compiler bit stability is not promised.
- **Half-planes are local.** Opposing or origin-wrong constraints can close a gap. The locked `n · o` term is mandatory. Do not silently widen S2.
- **Single-band local minima.** No homotopy search. S2’s centerline is a documented feasible homotopy, not a proof of TEB optimality.
- **Token no-heap lint** cannot prove OSQP update/solve allocates nothing. Be honest: authored `mpc_step` does not allocate; the solver may refactor.
- **S3 time is tight.** The scripted dodge took 20.08 s against a 30 s cap. Defaults must not dawdle.
- **Humble mover pairing** is a limitation of avoiding a custom message.
- FetchContent tags move. Commits and `URL_HASH` are required. `ros:humble-ros-base` is mutable until the digest is recorded.

## Verification already done (do not re-litigate)

- Existing applycoding repos were listed on 2026-11-10; this design does not duplicate `nav2-coverage-planner-compare` or the other robotics demos named above.
- Licenses checked: teb_local_planner BSD-3-Clause; OSQP and QDLDL Apache-2.0; bundled AMD BSD-3-Clause; nav2_mppi_controller MIT (not Apache); mpc_local_planner GPL-3.0; PythonRobotics MIT; nav2_costmap_2d dual BSD/Apache; nav2_core Apache-2.0.
- Pins checked 2026-11-10 as in the table. Official OSQP source archive and QDLDL v0.1.8 archive hashes recorded above. Apache license URL 200.
- Compiled OSQP v1.0.0 (builtin, `OSQP_USE_LONG=OFF`, `-DOSQP_VERSION=1.0.0`) on this host: `osqp_version=1.0.0`, one-step QP `OSQP_SOLVED`, `v ≈ 0.8`, objective `0 → −0.0224`.
- Unicycle hand cases, wrap, wheel diamond (one-wheel corner saturation), inverse-on-exact-arcs, Jacobians vs FD, L-path smoothness, S1–S4 fixture feasibility and S3 mutant collision as in the scenario table. Cost reduction is of the QP objective.
