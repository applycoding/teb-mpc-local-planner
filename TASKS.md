<!-- SPDX-License-Identifier: Apache-2.0 -->
# Build tasks — teb-mpc-local-planner

Execute in order. Follow PLAN.md. Deviate only if a pin or test proves a step broken, and record why. License is **Apache-2.0**, not MIT. Commit author and committer are Arpan Bom <applycoding@gmail.com>; no co-author trailers.

Each task has acceptance checks. A task is done only when those checks pass.

## 1. Repository skeleton and license

- Create the tree in PLAN.md (empty dirs may hold `.gitkeep`).
- Fetch `https://www.apache.org/licenses/LICENSE-2.0.txt` into `LICENSE` and confirm byte equality (including the appendix).
- Write `NOTICE` starting with `teb-mpc-local-planner, Copyright 2026 Arpan Bom`, then dependency notices (filled as pins land): OSQP Apache-2.0, QDLDL Apache-2.0, SuiteSparse AMD BSD-3-Clause (bundled in the OSQP tree, not the standalone QDLDL repo), GoogleTest, nlohmann/json, Emscripten/runtime if shipped.
- README stub: portfolio demo, simulation only, Apache-2.0, author Arpan Bom, “TEB-style single-band optimizer,” kinematic teaching model, not `teb_local_planner`, not a Nav2 plugin.
- CMake skeleton with `TML_BUILD_TESTS=ON`, `TML_BUILD_ROS=OFF`, `TML_BUILD_PYTHON=OFF`. SPDX on authored files.

**Accept:** `cmp LICENSE` against a fresh curl of the official text exits 0. NOTICE and README name Apache-2.0 and Arpan Bom.

## 2. Pin OSQP and QDLDL

- Commit `cmake/pins.sha256` with the official OSQP v1.0.0 source archive hash and the QDLDL v0.1.8 archive hash from PLAN.md. Configure must use `URL_HASH` (or an equivalent fetch of those exact objects) and fail on mismatch.
- CMake flags as in PLAN.md, including `OSQP_USE_LONG=OFF`, `-DOSQP_VERSION=1.0.0`, builtin algebra, no shared lib.
- A gtest (or a tiny exe in ctest) calls `osqp_version()` and requires `"1.0.0"`.

**Accept:** Criterion 9’s version and hash parts. `sizeof(OSQPInt)==sizeof(int)`.

## 3. Unicycle plant and wrap

- Implement the sinc-form `step_plant` and `wrap` from PLAN.md. No `ω` suppression below `1e-9`.

**Accept:** Straight `(2,0,0)`, quarter-circle `(2/π, 2/π, π/2)`, five `0.01 s` substeps vs one `0.05 s`, wrap tests including `±π`, reverse, tiny-`ω` both signs. `InvalidDt` on `dt <= 0` or non-finite, no write to `out`.

## 4. Inverse / projection residual and Jacobians

- Implement the signed least-squares TEB increment (`ω = a/ΔT`, `v = (b · Δp)/(b · b)`, residual `Δp − b v`).
- Implement the locked unwrapped A, B, and `c` of the sinc map.

**Accept:** Exact arcs recover `(v, ω)` within `1e-9`. A sideways increment has a large residual. FD step `1e-4` matches A,B (max abs error typically `~1e-10`). Affine reproduction of the reference.

## 5. Raw geometry, collision, clearance

- Occupied squares, unknown-as-lethal, out-of-map, movers with `p(t)=p0+v t` at global t0.
- Collision: distance `< robot_radius + extra_margin` (and mover equivalent). Tangency is not a collision.
- Clearance as specified (no second margin subtraction). Planner / MPC / TEB distances query raw geometry.

**Accept:** Tangency vs just-inside; time-aware mover hit/miss; S1-style empty in-map world ⇒ null clearance; `n · o` is not tested here yet but the signed-distance primitive is ready.

## 6. Frozen QP cost reduction and MpcSolver lifecycle

- One-step (and a small stacked) no-obstacle QP. Deliberately non-optimal feasible baseline. Product wrapper accepts `OSQP_SOLVED` only.
- Reserved CSC zeros, `osqp_update_data_mat` / `_vec`, stable index map on a straight-to-turn case.
- Settings: `polishing=0`, `adaptive_rho=OSQP_ADAPTIVE_RHO_UPDATE_DISABLED`.

**Accept:** Criterion 3. Update failure and missing setup return `SolveFailed` and `(0, 0)`.

## 7. Half-planes and TEB obstacle hinge

- Four slots: two static supports, two movers, `n · p >= n · o + r_eff`. Dummy rows reserved.
- TEB soft hinge plus sampled segment checks. Final first-interval collision can reject.

**Accept:** A unit test with an offset circle fails if `n · o` is omitted. Inactive dummy rows do not pull the robot.

## 8. TEB solver and shared reference

- Dense LM on the packed vector, caller-owned workspace, weights and limits from PLAN.md.
- Shared goal-directed / polyline reference, cruise `0.60 m/s`, terminal yaw policy, reverse allowed as specified.

**Accept:** Criterion 4. Already-at-goal returns success-eligible `(0, 0)` without spinning. Empty-map crooked band improves.

## 9. Metrics, fixtures, and closed-loop gates

- Implement metrics and JSON (no NaN). Commit locked S1–S4 JSON.
- `run_scenario` for both planners, mutant (all obstacle terms off, collision evaluator on), brake-only.

**Accept:** Criteria 5–8. Mutant **collides** on S3. Do not widen S2 or slow S3 to pass.

## 10. No-heap lint, CI core job

- `tools/check_no_heap.py` on `tml_core`. Core workflow on ubuntu-24.04 / g++-13, no ROS.

**Accept:** Criteria 1, 10, 11. Core is a merge gate.

## 11. WASM Pages demo and Playwright

- Same core+sim+OSQP to WASM with pinned emsdk 7.0.0 (manager and toolchain SHAs in PLAN.md).
- Vite `base=/teb-mpc-local-planner/`. Cap `tml_step_ticks` at 20. Separate state per robot.
- First web job includes OSQP setup / matrix update / repeated solve / cleanup.
- Playwright against `vite preview` at the Pages base; non-timing metric changes; wasm MIME; screenshot artifact. Check in `docs/demo.png` locally once.
- `pages.yml` deploys the tested `web/dist` from `main` after core and web of that SHA. Action SHAs in `.github/workflow-pins.md`. No `pull_request_target`.

**Accept:** Criterion 12. CI does not commit the screenshot.

## 12. README honesty, NOTICE, optional Humble

- Finish README and NOTICE as in criteria 14–16, 18–19.
- Optional Humble adapter: odometry in, `cmd_vel` out, PoseArray + Float64MultiArray pairing, OccupancyGrid conversion. Non-gating job. Digest recorded at first green run.

**Accept:** Criteria 13–19. Commit author is Arpan Bom `<applycoding@gmail.com>`.
