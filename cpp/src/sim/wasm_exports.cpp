// SPDX-License-Identifier: Apache-2.0
#include "tml/mpc_solver.hpp"
#include "tml/scenario.hpp"
#include "tml/tml.hpp"
#include "tml/wasm_api.hpp"

#include <cmath>
#include <cstddef>
#include <cstdio>

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
double g_teb_t = 0.0;
double g_mpc_t = 0.0;
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

void fill_world(tml::World* w, double t) {
  w->map = &g_sc.map;
  w->rects = g_sc.rects.empty() ? nullptr : g_sc.rects.data();
  w->n_rects = static_cast<int>(g_sc.rects.size());
  w->movers = g_sc.movers.empty() ? nullptr : g_sc.movers.data();
  w->n_movers = static_cast<int>(g_sc.movers.size());
  w->t = t;
}

void recompute_metrics() {
  tml::World w_teb;
  tml::World w_mpc;
  fill_world(&w_teb, g_teb_t);
  fill_world(&w_mpc, g_mpc_t);
  g_teb_metrics = tml::compute_metrics(g_teb_trace, g_sc.goal, g_sc.goal_tol_xy, g_sc.goal_tol_yaw,
                                       g_sc.dt_plant, w_teb, g_sc.robot, g_sc.inflation);
  g_mpc_metrics = tml::compute_metrics(g_mpc_trace, g_sc.goal, g_sc.goal_tol_xy, g_sc.goal_tol_yaw,
                                       g_sc.dt_plant, w_mpc, g_sc.robot, g_sc.inflation);
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
  const char* name = nullptr;
  switch (id) {
    case 1:
      name = "s1.json";
      break;
    case 2:
      name = "s2.json";
      break;
    case 3:
      name = "s3.json";
      break;
    case 4:
      name = "s4.json";
      break;
    default:
      return false;
  }
  char wasm_path[128];
  char rel_path[128];
  std::snprintf(wasm_path, sizeof(wasm_path), "/fixtures/scenarios/%s", name);
  std::snprintf(rel_path, sizeof(rel_path), "fixtures/scenarios/%s", name);
  if (tml::load_scenario_file(wasm_path, &g_sc)) {
    return true;
  }
  return tml::load_scenario_file(rel_path, &g_sc);
}

bool at_goal(const tml::Pose2D& pose) {
  const double d = std::hypot(pose.x - g_sc.goal.x, pose.y - g_sc.goal.y);
  const double ye = std::abs(tml::wrap_yaw(pose.yaw - g_sc.goal.yaw));
  return d <= g_sc.goal_tol_xy && ye <= g_sc.goal_tol_yaw;
}

void step_one(tml::PlannerId pid) {
  const bool is_teb = pid == tml::PlannerId::TebStyle;
  double& t = is_teb ? g_teb_t : g_mpc_t;
  tml::Pose2D& pose = is_teb ? g_teb_pose : g_mpc_pose;
  tml::Twist2D& u = is_teb ? g_teb_u : g_mpc_u;
  tml::Trace& trace = is_teb ? g_teb_trace : g_mpc_trace;

  if (t >= g_sc.T_max || at_goal(pose)) {
    return;
  }

  tml::World world;
  fill_world(&world, t);

  tml::PlanRequest req;
  req.dt = g_sc.dt_ctl;
  req.goal = g_sc.goal;
  req.world = world;
  req.obstacle_ref = true;
  req.pose = pose;
  req.u_prev = u;

  tml::PlanResult pr;
  if (is_teb) {
    tml::TebConfig cfg;
    cfg.robot = g_sc.robot;
    cfg.inflation = g_sc.inflation;
    cfg.max_iters = 12;
    tml::teb_step(req, cfg, g_teb_ws, pr);
  } else {
    tml::MpcConfig cfg;
    cfg.robot = g_sc.robot;
    cfg.inflation = g_sc.inflation;
    tml::mpc_step(g_mpc_solver, req, cfg, pr);
  }
  u = pr.twist;

  const int n = static_cast<int>(std::round(g_sc.dt_ctl / g_sc.dt_plant));
  for (int i = 0; i < n; ++i) {
    tml::Pose2D nxt;
    tml::step_plant(g_sc.dt_plant, u, pose, &nxt);
    pose = nxt;
    t += g_sc.dt_plant;
    world.t = t;
    record_trace(&trace, t, pose, u);
    if (tml::pose_collides(pose, world, g_sc.robot, g_sc.inflation) || at_goal(pose)) {
      return;
    }
  }
}

double path_len(const tml::Trace& tr) {
  double len = 0.0;
  for (int i = 1; i < tr.n; ++i) {
    len += std::hypot(tr.samples[i].pose.x - tr.samples[i - 1].pose.x,
                      tr.samples[i].pose.y - tr.samples[i - 1].pose.y);
  }
  return len;
}

}  // namespace

extern "C" int tml_reset(void) {
  g_teb_t = 0.0;
  g_mpc_t = 0.0;
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
  if (n > 2000) {
    n = 2000;
  }
  int ran = 0;
  for (int i = 0; i < n; ++i) {
    if (g_teb_t >= g_sc.T_max && g_mpc_t >= g_sc.T_max) {
      break;
    }
    if (at_goal(g_teb_pose) && at_goal(g_mpc_pose)) {
      break;
    }
    step_one(tml::PlannerId::TebStyle);
    step_one(tml::PlannerId::MpcOsqp);
    ++ran;
  }
  recompute_metrics();
  return ran;
}

extern "C" double tml_teb_x(void) { return g_teb_pose.x; }
extern "C" double tml_teb_y(void) { return g_teb_pose.y; }
extern "C" double tml_mpc_x(void) { return g_mpc_pose.x; }
extern "C" double tml_mpc_y(void) { return g_mpc_pose.y; }
extern "C" double tml_teb_sim_time(void) { return g_teb_t; }
extern "C" double tml_mpc_sim_time(void) { return g_mpc_t; }
extern "C" double tml_sim_time(void) { return g_teb_t > g_mpc_t ? g_teb_t : g_mpc_t; }
extern "C" double tml_t_max(void) { return g_sc.T_max; }
extern "C" int tml_scenario_id(void) { return g_scenario_id; }
extern "C" int tml_teb_success(void) { return g_teb_metrics.success; }
extern "C" int tml_mpc_success(void) { return g_mpc_metrics.success; }
extern "C" double tml_teb_path_len(void) { return path_len(g_teb_trace); }
extern "C" double tml_mpc_path_len(void) { return path_len(g_mpc_trace); }
extern "C" double tml_teb_time_to_goal(void) { return g_teb_metrics.time_to_goal_s; }
extern "C" double tml_mpc_time_to_goal(void) { return g_mpc_metrics.time_to_goal_s; }

extern "C" double tml_map_origin_x(void) { return g_sc.map.origin_x; }
extern "C" double tml_map_origin_y(void) { return g_sc.map.origin_y; }
extern "C" double tml_map_resolution(void) { return g_sc.map.resolution_m; }
extern "C" int tml_map_width(void) { return g_sc.map.width; }
extern "C" int tml_map_height(void) { return g_sc.map.height; }
extern "C" double tml_start_x(void) { return g_sc.start.x; }
extern "C" double tml_start_y(void) { return g_sc.start.y; }
extern "C" double tml_start_yaw(void) { return g_sc.start.yaw; }
extern "C" double tml_goal_x(void) { return g_sc.goal.x; }
extern "C" double tml_goal_y(void) { return g_sc.goal.y; }
extern "C" double tml_goal_yaw(void) { return g_sc.goal.yaw; }

extern "C" int tml_n_rects(void) { return static_cast<int>(g_sc.rects.size()); }
extern "C" double tml_rect_x0(int i) {
  return (i >= 0 && i < tml_n_rects()) ? g_sc.rects[static_cast<size_t>(i)].x0 : 0.0;
}
extern "C" double tml_rect_y0(int i) {
  return (i >= 0 && i < tml_n_rects()) ? g_sc.rects[static_cast<size_t>(i)].y0 : 0.0;
}
extern "C" double tml_rect_x1(int i) {
  return (i >= 0 && i < tml_n_rects()) ? g_sc.rects[static_cast<size_t>(i)].x1 : 0.0;
}
extern "C" double tml_rect_y1(int i) {
  return (i >= 0 && i < tml_n_rects()) ? g_sc.rects[static_cast<size_t>(i)].y1 : 0.0;
}

extern "C" int tml_n_movers(void) { return static_cast<int>(g_sc.movers.size()); }
extern "C" double tml_mover_x(int i) {
  return (i >= 0 && i < tml_n_movers()) ? g_sc.movers[static_cast<size_t>(i)].x : 0.0;
}
extern "C" double tml_mover_y(int i) {
  return (i >= 0 && i < tml_n_movers()) ? g_sc.movers[static_cast<size_t>(i)].y : 0.0;
}
extern "C" double tml_mover_radius(int i) {
  return (i >= 0 && i < tml_n_movers()) ? g_sc.movers[static_cast<size_t>(i)].radius : 0.0;
}
extern "C" double tml_mover_vx(int i) {
  return (i >= 0 && i < tml_n_movers()) ? g_sc.movers[static_cast<size_t>(i)].vx : 0.0;
}
extern "C" double tml_mover_vy(int i) {
  return (i >= 0 && i < tml_n_movers()) ? g_sc.movers[static_cast<size_t>(i)].vy : 0.0;
}

extern "C" int tml_occ_count(void) { return static_cast<int>(g_sc.map.occupied_cells.size()); }
extern "C" int tml_occ_i(int k) {
  return (k >= 0 && k < tml_occ_count()) ? g_sc.map.occupied_cells[static_cast<size_t>(k)].first : 0;
}
extern "C" int tml_occ_j(int k) {
  return (k >= 0 && k < tml_occ_count()) ? g_sc.map.occupied_cells[static_cast<size_t>(k)].second : 0;
}

extern "C" int tml_teb_trace_count(void) { return g_teb_trace.n; }
extern "C" double tml_teb_trace_x(int i) {
  return (i >= 0 && i < g_teb_trace.n) ? g_teb_trace.samples[i].pose.x : 0.0;
}
extern "C" double tml_teb_trace_y(int i) {
  return (i >= 0 && i < g_teb_trace.n) ? g_teb_trace.samples[i].pose.y : 0.0;
}
extern "C" int tml_mpc_trace_count(void) { return g_mpc_trace.n; }
extern "C" double tml_mpc_trace_x(int i) {
  return (i >= 0 && i < g_mpc_trace.n) ? g_mpc_trace.samples[i].pose.x : 0.0;
}
extern "C" double tml_mpc_trace_y(int i) {
  return (i >= 0 && i < g_mpc_trace.n) ? g_mpc_trace.samples[i].pose.y : 0.0;
}
