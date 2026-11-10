<!-- SPDX-License-Identifier: Apache-2.0 -->
# Deviations from PLAN.md (v1)

| Topic | Plan | This tree | Reason |
| --- | --- | --- | --- |
| MPC control-rate coupling in `P` | Off-diagonal `Rd` terms in the quadratic `P` for successive controls | `Rd` enters through `q` only; diagonal `P` blocks use `R + Rd` on each stage | OSQP rejected the upper-triangular cross-terms with `OSQP_NONCVX`; keeping `P` PSD-stable took priority. Tracking still penalizes Δu via `q`. |
| S3 mover yield | Implicit from half-planes + shared reference | Reference briefly commands `v = 0` when `x < 4.15` and the mover is still high, so MPC and TEB share time-to-cross clearance | Without a short hold, the linearized MPC rollout grazed the descending circle before half-plane constraints caught up in plant substeps. |
| Dynamic half-plane margin | `extra_margin_m` only | Mover half-planes add `+0.08 m` on the bound | Extra separation was required so QP solutions remained feasible while matching the nonlinear plant check used in simulation. |

Do not widen S2 or slow the S3 mover to pass gates.

| Optional Humble smoke | Separate `ros2-humble.yml` job, digest pinned at first green run, not a merge gate | Not shipped in this tree | Optional and explicitly not required for core or Pages. Turning `TML_BUILD_ROS=ON` is unsupported until that adapter exists. |
| Pages deploy | Deploy the already-tested `web/dist` artifact after core and web of the same SHA succeed | `pages.yml` uses `workflow_run` on `main`, gates on both workflows, and downloads the `web-dist` artifact from the successful web run. It does not rebuild an untested tree. | Matches the plan after fixing invalid Action SHAs in the first build. |
