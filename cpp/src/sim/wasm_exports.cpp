// SPDX-License-Identifier: Apache-2.0
#include "tml/mpc_solver.hpp"
#include "tml/scenario.hpp"
#include "tml/tml.hpp"

#include <cmath>

namespace {

tml::Scenario g_sc;
tml::Trace g_teb_trace;
tml::Trace g_mpc_trace;
tml::TebWorkspace g_teb_ws;
tml::MpcSolver g_mpc_solver;
tml::Pose2D g_teb_pose;
tml::Pose2D g_mpc_pose;
tml::Twist2D g_teb_u;
tml::Twist2D g_mpc_u;
tml::Metrics g_teb_metrics;
tml::Metrics g_mpc_metrics;
double g_t = 0.0;
int g_scenario_id = 1;

void record_trace(tml::Trace* tr, double t, const tml::Pose2D& p, const tml::Twist2D& u) {
  if (tr == nullptr || tr->n >= tml::Trace::kMaxSamples) {
    return;
  }
  tr->samples[tr->n++] = tml::PlantSample{t, p, u};
}

void reset_metrics() {
  g_teb_metrics = tml::Metrics{};
  g_mpc_metrics = tml::Metrics{};
}

void recompute_metrics() {
  tml::World w;
  w.map = &g_sc.map;
  w.rects = g_sc.rects.empty() ? nullptr : g_sc.rects.data();
  w.n_rects = static_cast<int>(g_sc.rects.size());
  w.movers = g_sc.movers.empty() ? nullptr : g_sc.movers.data();
  w.n_movers = static_cast<int>(g_sc.movers.size());
  w.t = g_t;
  g_teb_metrics = tml::compute_metrics(g_teb_trace, g_sc.goal, g_sc.goal_tol_xy, g_sc.goal_tol_yaw,
                                       g_sc.dt_plant, w, g_sc.robot, g_sc.inflation);
  g_mpc_metrics = tml::compute_metrics(g_mpc_trace, g_sc.goal, g_sc.goal_tol_xy, g_sc.goal_tol_yaw,
                                       g_sc.dt_plant, w, g_sc.robot, g_sc.inflation);
}

void reset_robot(bool teb) {
  if (teb) {
    g_teb_pose = g_sc.start;
    g_teb_u = tml::Twist2D{0, 0};
    g_teb_trace.n = 0;
    record_trace(&g_teb_trace, 0.0, g_teb_pose, g_teb_u);
  } else {
    g_mpc_pose = g_sc.start;
    g_mpc_u = tml::Twist2D{0, 0};
    g_mpc_trace.n = 0;
    record_trace(&g_mpc_trace, 0.0, g_mpc_pose, g_mpc_u);
    g_mpc_solver.cleanup();
    tml::MpcConfig mc;
    mc.robot = g_sc.robot;
    mc.inflation = g_sc.inflation;
    g_mpc_solver.setup(mc);
  }
}

bool load_fixture(int id) {
  const char* path = nullptr;
  switch (id) {
    case 1:
      path = "/fixtures/scenarios/s1.json";
      break;
    case 2:
      path = "/fixtures/scenarios/s2.json";
      break;
    case 3:
      path = "/fixtures/scenarios/s3.json";
      break;
    case 4:
      path = "/fixtures/scenarios/s4.json";
      break;
    default:
      return false;
  }
  return tml::load_scenario_file(path, &g_sc);
}

void step_one(tml::PlannerId pid) {
  if (g_t >= g_sc.T_max) {
    return;
  }
  tml::World world;
  world.map = &g_sc.map;
  world.rects = g_sc.rects.empty() ? nullptr : g_sc.rects.data();
  world.n_rects = static_cast<int>(g_sc.rects.size());
  world.movers = g_sc.movers.empty() ? nullptr : g_sc.movers.data();
  world.n_movers = static_cast<int>(g_sc.movers.size());
  world.t = g_t;

  tml::PlanRequest req;
  req.dt = g_sc.dt_ctl;
  req.goal = g_sc.goal;
  req.world = world;
  req.obstacle_ref = true;

  const int n = static_cast<int>(std::round(g_sc.dt_ctl / g_sc.dt_plant));

  if (pid == tml::PlannerId::TebStyle) {
    req.pose = g_teb_pose;
    req.u_prev = g_teb_u;
    tml::TebConfig cfg;
    cfg.robot = g_sc.robot;
    cfg.inflation = g_sc.inflation;
    cfg.max_iters = 12;
    tml::PlanResult pr;
    tml::teb_step(req, cfg, g_teb_ws, pr);
    g_teb_u = pr.twist;
    for (int i = 0; i < n; ++i) {
      tml::Pose2D nxt;
      tml::step_plant(g_sc.dt_plant, g_teb_u, g_teb_pose, &nxt);
      g_teb_pose = nxt;
      g_t += g_sc.dt_plant;
      world.t = g_t;
      if (tml::pose_collides(g_teb_pose, world, g_sc.robot, g_sc.inflation)) {
        record_trace(&g_teb_trace, g_t, g_teb_pose, g_teb_u);
        return;
      }
      record_trace(&g_teb_trace, g_t, g_teb_pose, g_teb_u);
    }
  } else {
    req.pose = g_mpc_pose;
    req.u_prev = g_mpc_u;
    tml::MpcConfig cfg;
    cfg.robot = g_sc.robot;
    cfg.inflation = g_sc.inflation;
    tml::PlanResult pr;
    tml::mpc_step(g_mpc_solver, req, cfg, pr);
    g_mpc_u = pr.twist;
    for (int i = 0; i < n; ++i) {
      tml::Pose2D nxt;
      tml::step_plant(g_sc.dt_plant, g_mpc_u, g_mpc_pose, &nxt);
      g_mpc_pose = nxt;
      g_t += g_sc.dt_plant;
      world.t = g_t;
      if (tml::pose_collides(g_mpc_pose, world, g_sc.robot, g_sc.inflation)) {
        record_trace(&g_mpc_trace, g_t, g_mpc_pose, g_mpc_u);
        return;
      }
      record_trace(&g_mpc_trace, g_t, g_mpc_pose, g_mpc_u);
    }
  }
}

}  // namespace

extern "C" int tml_reset(void) {
  g_t = 0.0;
  reset_robot(true);
  reset_robot(false);
  reset_metrics();
  recompute_metrics();
  return 0;
}

extern "C" int tml_set_scenario(int id) {
  if (!load_fixture(id)) {
    return -1;
  }
  g_scenario_id = id;
  return tml_reset();
}

extern "C" int tml_load_scenario_json(const char* json) {
  if (json == nullptr || !tml::load_scenario_json(json, &g_sc)) {
    return -1;
  }
  g_scenario_id = 0;
  return tml_reset();
}

extern "C" int tml_step_ticks(int n) {
  if (n <= 0) {
    return 0;
  }
  if (n > 20) {
    n = 20;
  }
  for (int i = 0; i < n; ++i) {
    step_one(tml::PlannerId::TebStyle);
    step_one(tml::PlannerId::MpcOsqp);
    if (g_t >= g_sc.T_max) {
      break;
    }
  }
  recompute_metrics();
  return n;
}

extern "C" double tml_teb_x(void) { return g_teb_pose.x; }
extern "C" double tml_teb_y(void) { return g_teb_pose.y; }
extern "C" double tml_mpc_x(void) { return g_mpc_pose.x; }
extern "C" double tml_mpc_y(void) { return g_mpc_pose.y; }
extern "C" double tml_sim_time(void) { return g_t; }
extern "C" int tml_scenario_id(void) { return g_scenario_id; }
extern "C" int tml_teb_success(void) { return g_teb_metrics.success; }
extern "C" int tml_mpc_success(void) { return g_mpc_metrics.success; }
extern "C" double tml_teb_path_len(void) {
  double len = 0.0;
  for (int i = 1; i < g_teb_trace.n; ++i) {
    len += std::hypot(g_teb_trace.samples[i].pose.x - g_teb_trace.samples[i - 1].pose.x,
                      g_teb_trace.samples[i].pose.y - g_teb_trace.samples[i - 1].pose.y);
  }
  return len;
}
extern "C" double tml_mpc_path_len(void) {
  double len = 0.0;
  for (int i = 1; i < g_mpc_trace.n; ++i) {
    len += std::hypot(g_mpc_trace.samples[i].pose.x - g_mpc_trace.samples[i - 1].pose.x,
                      g_mpc_trace.samples[i].pose.y - g_mpc_trace.samples[i - 1].pose.y);
  }
  return len;
}
extern "C" double tml_teb_time_to_goal(void) { return g_teb_metrics.time_to_goal_s; }
extern "C" double tml_mpc_time_to_goal(void) { return g_mpc_metrics.time_to_goal_s; }
