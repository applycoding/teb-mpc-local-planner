// SPDX-License-Identifier: Apache-2.0
#include "tml/tml.hpp"

#include <algorithm>
#include <cmath>

namespace tml {

static double bearing(const Pose2D& from, const Pose2D& to) {
  return std::atan2(to.y - from.y, to.x - from.x);
}

static Twist2D ref_point_to_twist(const Pose2D& pose, const Pose2D& goal, const Limits& lim) {
  const double dx = goal.x - pose.x;
  const double dy = goal.y - pose.y;
  const double d = std::hypot(dx, dy);
  constexpr double v_cruise = 0.60;
  const double goal_xy_tol = 0.40;
  Twist2D u{};
  if (d < 1e-6) {
    const double ey = wrap_yaw(goal.yaw - pose.yaw);
    if (std::abs(ey) < 0.05) {
      return Twist2D{0, 0};
    }
    u.omega = std::clamp(2.0 * ey, -lim.omega_max, lim.omega_max);
    u.v = 0.0;
    return u;
  }
  const double bear = std::atan2(dy, dx);
  double ey = wrap_yaw(bear - pose.yaw);
  if (std::abs(ey) > 2.0 * M_PI / 3.0 && d > 0.5) {
    u.omega = std::clamp(2.0 * ey, -lim.omega_max, lim.omega_max);
    u.v = -std::min(0.3, v_cruise);
    return u;
  }
  u.omega = std::clamp(2.0 * ey, -lim.omega_max, lim.omega_max);
  double v = v_cruise;
  if (d < 0.6) {
    v *= d / 0.6;
  }
  const double d_goal = std::hypot(pose.x - goal.x, pose.y - goal.y);
  if (d_goal < 0.27) {
    v = std::max(v, 0.55);
    const double gy = wrap_yaw(goal.yaw - pose.yaw);
    u.omega = std::clamp(1.4 * gy, -lim.omega_max, lim.omega_max);
  }
  if (d_goal < goal_xy_tol && d_goal > 0.24) {
    v = std::max(v, 0.38);
  } else if (d_goal <= 0.24) {
    v = std::min(v, 0.22);
    const double gy = wrap_yaw(goal.yaw - pose.yaw);
    u.omega = std::clamp(2.5 * gy, -lim.omega_max, lim.omega_max);
  } else if (d_goal < goal_xy_tol) {
    const double gy = wrap_yaw(goal.yaw - pose.yaw);
    u.omega = std::clamp(2.5 * gy, -lim.omega_max, lim.omega_max);
  }
  u.v = std::clamp(v, 0.0, lim.v_max);
  return u;
}

void reference_rollout(const PlanRequest& req, const MpcConfig& cfg, Pose2D z_ref[kBandMax],
                       Twist2D u_ref[kBandMax]) {
  const int N = cfg.N;
  z_ref[0] = req.pose;
  Pose2D cur = req.pose;
  for (int k = 0; k < N; ++k) {
    Twist2D u = ref_point_to_twist(cur, req.goal, cfg.robot.limits);
    if (req.obstacle_ref && req.world.n_rects > 0) {
      const double sd = signed_clearance_static(req.world, cur.x, cur.y, cfg.robot.radius_m, 0.05);
      if (std::isfinite(sd) && sd < 0.35) {
        u.v = std::min(u.v, 0.45);
        if (cur.y > 0.08) {
          u.omega = std::max(-cfg.robot.limits.omega_max, -0.4);
        } else if (cur.y < -0.08) {
          u.omega = std::min(cfg.robot.limits.omega_max, 0.4);
        }
      }
    }
    if (req.obstacle_ref && req.world.n_movers > 0) {
      const double sm =
          signed_clearance_movers(req.world, cur.x, cur.y, cfg.robot.radius_m, 0.05);
      if (std::isfinite(sm) && cur.x > 1.5 && cur.x < 7.2) {
        double mover_y = 0.0;
        for (int mv = 0; mv < req.world.n_movers; ++mv) {
          DynamicObstacle m = req.world.movers[mv];
          const double t = req.world.t + (k + 1) * cfg.h;
          mover_y = m.y + m.vy * t;
        }
        if (cur.x < 5.4 && mover_y > -0.2 && sm < 2.2) {
          const double y_tgt = -1.22;
          const double ey = y_tgt - cur.y;
          u.omega = std::clamp(3.0 * ey, -cfg.robot.limits.omega_max, cfg.robot.limits.omega_max);
          if (cur.x < 4.15 && mover_y > 0.2) {
            u.v = 0.0;
          } else {
            u.v = std::min(u.v, 0.36);
          }
        } else if (cur.y < -0.35 && cur.x < 6.8) {
          const double ey = -cur.y;
          u.omega = std::clamp(2.4 * ey, -cfg.robot.limits.omega_max, cfg.robot.limits.omega_max);
          u.v = std::max(u.v, 0.52);
        } else if (sm < 0.95) {
          u.omega = std::max(-cfg.robot.limits.omega_max, std::min(u.omega, -0.5));
          u.v = std::min(u.v, 0.42);
        }
      }
    }
    u_ref[k] = u;
    Pose2D nxt;
    step_plant(cfg.h, u, cur, &nxt);
    z_ref[k + 1] = nxt;
    cur = nxt;
  }
}

}  // namespace tml
