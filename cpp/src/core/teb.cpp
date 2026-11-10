// SPDX-License-Identifier: Apache-2.0
#include "tml/tml.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace tml {

static void init_band_from_reference(const PlanRequest& req, const TebConfig& cfg, Band* band) {
  MpcConfig rc;
  rc.robot = cfg.robot;
  rc.inflation = cfg.inflation;
  Pose2D zr[kBandMax];
  Twist2D ur[kBandMax];
  reference_rollout(req, rc, zr, ur);
  band->M = cfg.M;
  band->poses[0] = req.pose;
  for (int i = 1; i < cfg.M; ++i) {
    if (i <= rc.N) {
      band->poses[i] = zr[i];
    } else {
      band->poses[i] = req.goal;
    }
    band->dt[i - 1] = std::clamp(req.dt, cfg.dt_min, cfg.dt_max);
  }
  if (cfg.w_obs <= 0) {
    for (int i = 2; i < std::min(8, band->M - 1); i += 2) {
      band->poses[i].y += 0.11 * ((i % 4 == 2) ? 1.0 : -1.0);
    }
  }
}

static double hinge_obs(double signed_d) {
  if (signed_d >= 0) {
    return 0.0;
  }
  return -signed_d;
}

double teb_eval_cost(const Band& band, const PlanRequest& req, const TebConfig& cfg, Twist2D u_prev,
                     double* kin_rms);

double teb_eval_cost(const Band& band, const PlanRequest& req, const TebConfig& cfg, Twist2D u_prev,
                     double* kin_rms) {
  double cost = 0.0;
  double kin2 = 0.0;
  Twist2D last_u = u_prev;
  const bool occ = req.world.n_rects > 0 || req.world.n_movers > 0;
  const double dt_plant = req.dt > 0 ? req.dt / 5.0 : 0.01;
  for (int i = 0; i < band.M - 1; ++i) {
    cost += cfg.w_time * band.dt[i];
    Twist2D u;
    double kin_res = 0;
    increment_to_twist(band.poses[i], band.poses[i + 1], band.dt[i], &u, &kin_res);
    kin2 += kin_res * kin_res;
    const double dv = u.v - last_u.v;
    const double dw = u.omega - last_u.omega;
    auto pen = [](double val, double lim) {
      if (std::abs(val) <= lim) {
        return 0.0;
      }
      const double e = std::abs(val) - lim;
      return e * e;
    };
    cost += cfg.w_lim * pen(u.v, cfg.robot.limits.v_max);
    cost += cfg.w_lim * pen(u.omega, cfg.robot.limits.omega_max);
    cost += cfg.w_lim * pen(dv / band.dt[i], cfg.robot.limits.a_max);
    cost += cfg.w_lim * pen(dw / band.dt[i], cfg.robot.limits.alpha_max);
    cost += cfg.w_kin * kin_res * kin_res;
    last_u = u;
    if (cfg.w_obs > 0 && occ) {
      const double margin = cfg.inflation.extra_margin_m;
      for (double t = 0.0; t <= band.dt[i]; t += dt_plant) {
        const double a = t / band.dt[i];
        Pose2D p;
        p.x = band.poses[i].x + a * (band.poses[i + 1].x - band.poses[i].x);
        p.y = band.poses[i].y + a * (band.poses[i + 1].y - band.poses[i].y);
        p.yaw = wrap_yaw(band.poses[i].yaw + a * wrap_yaw(band.poses[i + 1].yaw - band.poses[i].yaw));
        const double sd = signed_clearance_static(req.world, p.x, p.y, cfg.robot.radius_m, margin);
        cost += cfg.w_obs * hinge_obs(sd) * hinge_obs(sd);
        if (req.world.n_movers > 0) {
          World w = req.world;
          w.t += t;
          const double sm = signed_clearance_movers(w, p.x, p.y, cfg.robot.radius_m, margin);
          cost += cfg.w_obs * hinge_obs(sm) * hinge_obs(sm);
        }
      }
    }
  }
  const Pose2D& end = band.poses[band.M - 1];
  const double ex = end.x - req.goal.x;
  const double ey = end.y - req.goal.y;
  const double eyaw = wrap_yaw(end.yaw - req.goal.yaw);
  cost += cfg.w_goal_xy * (ex * ex + ey * ey);
  cost += cfg.w_goal_yaw * eyaw * eyaw;
  if (kin_rms) {
    *kin_rms = std::sqrt(kin2 / std::max(1, band.M - 1));
  }
  return cost;
}

static void project_band(Band* band, const TebConfig& cfg) {
  for (int i = 0; i < band->M - 1; ++i) {
    band->dt[i] = std::clamp(band->dt[i], cfg.dt_min, cfg.dt_max);
  }
}

static void pack_from_band(TebWorkspace& ws, const Band& band) {
  int n = 0;
  for (int i = 1; i < band.M; ++i) {
    ws.x[n++] = band.poses[i].x;
    ws.x[n++] = band.poses[i].y;
    ws.x[n++] = band.poses[i].yaw;
  }
  for (int i = 0; i < band.M - 1; ++i) {
    ws.x[n++] = band.dt[i];
  }
  ws.n_var = n;
}

static void unpack_to_band(const TebWorkspace& ws, int M, const Pose2D& fixed0, Band* band) {
  band->M = M;
  band->poses[0] = fixed0;
  int n = 0;
  for (int i = 1; i < M; ++i) {
    band->poses[i].x = ws.x[n++];
    band->poses[i].y = ws.x[n++];
    band->poses[i].yaw = ws.x[n++];
  }
  for (int i = 0; i < M - 1; ++i) {
    band->dt[i] = ws.x[n++];
  }
}

Status teb_step(const PlanRequest& req, const TebConfig& cfg, TebWorkspace& ws, PlanResult& out) {
  out = PlanResult{};
  out.reason = "teb";
  if (!finite_pose(req.pose) || !finite_pose(req.goal) || req.dt <= 0 || cfg.M < 2 ||
      cfg.M > kBandMax || req.world.map == nullptr) {
    out.status = Status::InvalidInput;
    out.reason = "invalid_input";
    out.twist = Twist2D{0, 0};
    return out.status;
  }
  if (pose_collides(req.pose, req.world, cfg.robot, cfg.inflation)) {
    out.status = Status::InvalidInput;
    out.reason = "invalid_start";
    out.twist = Twist2D{0, 0};
    return out.status;
  }

  const double d_goal = std::hypot(req.pose.x - req.goal.x, req.pose.y - req.goal.y);
  const double y_err = std::abs(wrap_yaw(req.pose.yaw - req.goal.yaw));
  if (d_goal < 0.25 && y_err < 0.30) {
    out.status = Status::NearGoal;
    out.reason = "near_goal";
    out.twist = Twist2D{0, 0};
    return out.status;
  }

  Band band{};
  init_band_from_reference(req, cfg, &band);
  pack_from_band(ws, band);
  double kin0 = 0;
  double best = teb_eval_cost(band, req, cfg, req.u_prev, &kin0);
  out.cost_initial = best;
  double lambda = 1e-3;
  int iters = 0;
  for (; iters < cfg.max_iters; ++iters) {
    bool accepted = false;
    const double prev_cost = best;
    const double step = 0.04 / (1.0 + lambda);
    TebWorkspace trial_ws = ws;
    const int n_try = ws.n_var;
    for (int sign : {1, -1}) {
      for (int k = 0; k < n_try; ++k) {
        trial_ws = ws;
        trial_ws.x[k] += static_cast<double>(sign) * step;
        Band trial{};
        unpack_to_band(trial_ws, cfg.M, req.pose, &trial);
        project_band(&trial, cfg);
        double kin_t = 0;
        const double c = teb_eval_cost(trial, req, cfg, req.u_prev, &kin_t);
        if (c + 1e-9 < best) {
          unpack_to_band(trial_ws, cfg.M, req.pose, &band);
          project_band(&band, cfg);
          pack_from_band(ws, band);
          best = c;
          accepted = true;
          lambda = std::max(1e-6, lambda / 3.0);
          break;
        }
      }
      if (accepted) {
        break;
      }
    }
    if (!accepted) {
      lambda = std::min(1e8, lambda * 10.0);
    } else if (iters > 0 && (prev_cost - best) / std::max(1.0, prev_cost) < 1e-4) {
      break;
    }
    if (lambda > 1e7) {
      break;
    }
  }

  Twist2D u_cmd;
  double kin_res = 0;
  increment_to_twist(band.poses[0], band.poses[1], band.dt[0], &u_cmd, &kin_res);
  bool brake = false;
  if (cfg.w_obs > 0 &&
      segment_collides(band.poses[0], band.poses[1], req.world, cfg.robot, cfg.inflation, 0.01)) {
    brake = true;
    out.status = Status::EmergencyBrake;
    out.reason = "collision_reject";
  } else {
    out.status = Status::Ok;
  }
  out.twist = clamp_command(u_cmd, req.u_prev, cfg.robot.limits, req.dt, brake);
  out.iterations = iters;
  out.cost_final = best;
  out.n_predicted = std::min(cfg.M, kBandMax);
  for (int i = 0; i < out.n_predicted; ++i) {
    out.predicted[i] = band.poses[i];
  }
  (void)kin0;
  (void)kin_res;
  return out.status;
}

}  // namespace tml
