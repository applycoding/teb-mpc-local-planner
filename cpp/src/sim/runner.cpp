// SPDX-License-Identifier: Apache-2.0
#include "tml/scenario.hpp"
#include "tml/tml.hpp"

#include <chrono>
#include <cmath>

namespace tml {

static void record_sample(Trace* tr, double t, const Pose2D& p, const Twist2D& u) {
  if (tr == nullptr || tr->n >= Trace::kMaxSamples) {
    return;
  }
  tr->samples[tr->n++] = PlantSample{t, p, u};
}

int run_scenario(uint32_t /*seed*/, const Scenario& sc, PlannerId id, Trace* trace,
                 TebWorkspace* teb_ws, MpcSolver* mpc_solver, bool mutant, bool brake_only) {
  if (trace == nullptr) {
    return -1;
  }
  trace->n = 0;
  Pose2D pose = sc.start;
  Twist2D u_hold{0, 0};
  double t = 0.0;
  World world;
  world.map = &sc.map;
  world.rects = sc.rects.empty() ? nullptr : sc.rects.data();
  world.n_rects = static_cast<int>(sc.rects.size());
  world.movers = sc.movers.empty() ? nullptr : sc.movers.data();
  world.n_movers = static_cast<int>(sc.movers.size());
  world.t = t;

  TebConfig tcfg;
  tcfg.M = 25;
  tcfg.robot = sc.robot;
  tcfg.inflation = sc.inflation;
  if (mutant) {
    tcfg.w_obs = 0;
  }

  MpcConfig mcfg;
  mcfg.robot = sc.robot;
  mcfg.inflation = sc.inflation;
  mcfg.obstacles_enabled = !mutant;

  TebWorkspace local_ws;
  TebWorkspace* ws = teb_ws != nullptr ? teb_ws : &local_ws;

  if (id == PlannerId::MpcOsqp && mpc_solver != nullptr && !mpc_solver->is_setup()) {
    mpc_solver->setup(mcfg);
  }

  record_sample(trace, t, pose, u_hold);

  const int steps_per_ctl = static_cast<int>(std::round(sc.dt_ctl / sc.dt_plant));
  const int max_ctl = static_cast<int>(std::ceil(sc.T_max / sc.dt_ctl));

  for (int tick = 0; tick < max_ctl; ++tick) {
    if (t >= sc.T_max) {
      break;
    }
    world.t = t;
    PlanRequest req;
    req.pose = pose;
    req.goal = sc.goal;
    req.world = world;
    req.dt = sc.dt_ctl;
    req.u_prev = u_hold;
    req.obstacle_ref = !mutant;
    req.polyline = sc.polyline.empty() ? nullptr : sc.polyline.data();
    req.n_polyline = static_cast<int>(sc.polyline.size());

    PlanResult pr{};
    auto t0 = std::chrono::steady_clock::now();
    if (brake_only) {
      pr.twist = Twist2D{0, 0};
      pr.status = Status::Ok;
    } else if (id == PlannerId::TebStyle) {
      teb_step(req, tcfg, *ws, pr);
    } else {
      mpc_step(*mpc_solver, req, mcfg, pr);
    }
    auto t1 = std::chrono::steady_clock::now();
    const double ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();
    (void)ms;

    u_hold = pr.twist;
    for (int s = 0; s < steps_per_ctl; ++s) {
      Pose2D nxt;
      if (step_plant(sc.dt_plant, u_hold, pose, &nxt) != Status::Ok) {
        break;
      }
      pose = nxt;
      t += sc.dt_plant;
      world.t = t;
      record_sample(trace, t, pose, u_hold);
      if (pose_collides(pose, world, sc.robot, sc.inflation)) {
        return trace->n;
      }
    }
    const double d = std::hypot(pose.x - sc.goal.x, pose.y - sc.goal.y);
    const double ye = std::abs(wrap_yaw(pose.yaw - sc.goal.yaw));
    if (d <= sc.goal_tol_xy && ye <= sc.goal_tol_yaw) {
      return trace->n;
    }
  }
  return trace->n;
}

}  // namespace tml
