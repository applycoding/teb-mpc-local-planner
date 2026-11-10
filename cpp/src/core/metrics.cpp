// SPDX-License-Identifier: Apache-2.0
#include "tml/tml.hpp"

#include <cmath>
#include <cstring>
#include <limits>

namespace tml {

static bool at_goal(const Pose2D& p, const Pose2D& goal, double tol_xy, double tol_yaw) {
  const double d = std::hypot(p.x - goal.x, p.y - goal.y);
  const double y = std::abs(wrap_yaw(p.yaw - goal.yaw));
  return d <= tol_xy && y <= tol_yaw;
}

Metrics compute_metrics(const Trace& trace, const Pose2D& goal, double goal_tol_xy,
                        double goal_tol_yaw, double dt_plant, const World& world,
                        const RobotParams& robot, const Inflation& infl) {
  Metrics m{};
  m.clearance_m = std::numeric_limits<double>::quiet_NaN();
  m.time_to_goal_s = std::numeric_limits<double>::quiet_NaN();
  m.smoothness = std::numeric_limits<double>::quiet_NaN();
  m.jerk = std::numeric_limits<double>::quiet_NaN();
  m.yaw_jerk = std::numeric_limits<double>::quiet_NaN();
  if (trace.n <= 0) {
    m.reason = "invalid_input";
    return m;
  }

  bool collided = false;
  double min_clear = std::numeric_limits<double>::infinity();
  bool any_geom = false;
  if (world.map != nullptr) {
    if (world.n_rects > 0 || world.n_movers > 0) {
      any_geom = true;
    }
    if (!world.map->in_map(trace.samples[0].pose.x, trace.samples[0].pose.y)) {
      any_geom = true;
    }
  }
  if (world.n_movers > 0) {
    any_geom = true;
  }

  double time_goal = -1.0;
  double path_len = 0.0;
  double yaw_sum = 0.0;

  for (int i = 0; i < trace.n; ++i) {
    World w = world;
    w.t = trace.samples[i].t;
    if (pose_collides(trace.samples[i].pose, w, robot, infl)) {
      collided = true;
      min_clear = 0.0;
    } else {
      const double c = clearance_at_pose(trace.samples[i].pose, w, robot);
      if (std::isfinite(c)) {
        min_clear = std::min(min_clear, c);
      }
    }
    if (!collided && time_goal < 0 && at_goal(trace.samples[i].pose, goal, goal_tol_xy,
                                             goal_tol_yaw)) {
      time_goal = trace.samples[i].t;
    }
    if (i > 0) {
      path_len += std::hypot(trace.samples[i].pose.x - trace.samples[i - 1].pose.x,
                             trace.samples[i].pose.y - trace.samples[i - 1].pose.y);
      yaw_sum += std::abs(wrap_yaw(trace.samples[i].pose.yaw - trace.samples[i - 1].pose.yaw));
    }
  }

  if (collided) {
    m.success = 0;
    m.reason = "collision";
    m.clearance_m = 0.0;
  } else if (time_goal >= 0.0) {
    m.success = 1;
    m.reason = "ok";
    m.time_to_goal_s = time_goal - trace.samples[0].t;
    if (path_len < 1e-6) {
      m.smoothness = 0.0;
    } else {
      m.smoothness = yaw_sum / path_len;
    }
    if (!any_geom) {
      m.clearance_m = std::numeric_limits<double>::quiet_NaN();
    } else {
      m.clearance_m = std::isfinite(min_clear) ? min_clear : 0.0;
    }
  } else {
    m.success = 0;
    m.reason = "timeout";
    if (path_len >= 1e-6) {
      m.smoothness = yaw_sum / path_len;
    }
    m.clearance_m = collided ? 0.0 : (std::isfinite(min_clear) ? min_clear : 0.0);
  }

  if (trace.n >= 3) {
    double sum_j = 0.0, sum_yj = 0.0;
    int cnt = 0;
    for (int i = 2; i < trace.n; ++i) {
      const double vk = trace.samples[i].cmd.v;
      const double vkm1 = trace.samples[i - 1].cmd.v;
      const double vkm2 = trace.samples[i - 2].cmd.v;
      const double wk = trace.samples[i].cmd.omega;
      const double wkm1 = trace.samples[i - 1].cmd.omega;
      const double wkm2 = trace.samples[i - 2].cmd.omega;
      const double jv = (vk - 2.0 * vkm1 + vkm2) / (dt_plant * dt_plant);
      const double jw = (wk - 2.0 * wkm1 + wkm2) / (dt_plant * dt_plant);
      sum_j += jv * jv;
      sum_yj += jw * jw;
      ++cnt;
    }
    m.jerk = std::sqrt(sum_j / cnt);
    m.yaw_jerk = std::sqrt(sum_yj / cnt);
  }

  return m;
}

}  // namespace tml
