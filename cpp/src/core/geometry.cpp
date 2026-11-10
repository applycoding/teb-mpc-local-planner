// SPDX-License-Identifier: Apache-2.0
#include "tml/costmap.hpp"
#include "tml/tml.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tml {

static double dist_point_rect(double px, double py, double x0, double y0, double x1, double y1) {
  const double cx = std::clamp(px, x0, x1);
  const double cy = std::clamp(py, y0, y1);
  const double dx = px - cx;
  const double dy = py - cy;
  return std::sqrt(dx * dx + dy * dy);
}

static DynamicObstacle mover_at(const DynamicObstacle& m, double t) {
  DynamicObstacle o = m;
  o.x += m.vx * t;
  o.y += m.vy * t;
  return o;
}

double signed_clearance_movers(const World& world, double x, double y, double robot_r,
                               double margin) {
  double best = std::numeric_limits<double>::infinity();
  for (int k = 0; k < world.n_movers; ++k) {
    DynamicObstacle m = mover_at(world.movers[k], world.t);
    const double d = std::hypot(x - m.x, y - m.y) - m.radius;
    best = std::min(best, d);
  }
  if (!std::isfinite(best)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return best - robot_r - margin;
}

static double clearance_rects(const World& world, double x, double y, double robot_r,
                              double margin) {
  double best = std::numeric_limits<double>::infinity();
  for (int k = 0; k < world.n_rects; ++k) {
    const auto& r = world.rects[k];
    best = std::min(best, dist_point_rect(x, y, r.x0, r.y0, r.x1, r.y1));
  }
  for (int k = 0; k < world.n_movers; ++k) {
    DynamicObstacle m = mover_at(world.movers[k], world.t);
    best = std::min(best, std::hypot(x - m.x, y - m.y) - m.radius);
  }
  if (!std::isfinite(best)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return best - robot_r - margin;
}

bool pose_collides(const Pose2D& pose, const World& world, const RobotParams& robot,
                   const Inflation& infl) {
  if (world.map == nullptr || !world.map->in_map(pose.x, pose.y)) {
    return true;
  }
  const double margin = infl.extra_margin_m;
  const double sd = clearance_rects(world, pose.x, pose.y, robot.radius_m, margin);
  return std::isfinite(sd) && sd < 0.0;
}

double clearance_at_pose(const Pose2D& pose, const World& world, const RobotParams& robot) {
  if (world.map == nullptr || !world.map->in_map(pose.x, pose.y)) {
    return 0.0;
  }
  const double best = clearance_rects(world, pose.x, pose.y, robot.radius_m, 0.0);
  if (!std::isfinite(best)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::max(0.0, best);
}

bool segment_collides(const Pose2D& a, const Pose2D& b, const World& world,
                      const RobotParams& robot, const Inflation& infl, double dt_sample) {
  const int total = std::max(3, static_cast<int>(std::ceil(std::hypot(b.x - a.x, b.y - a.y) / 0.05)));
  for (int i = 0; i <= total; ++i) {
    const double t = static_cast<double>(i) / total;
    Pose2D p{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y),
             wrap_yaw(a.yaw + t * wrap_yaw(b.yaw - a.yaw))};
    if (pose_collides(p, world, robot, infl)) {
      return true;
    }
  }
  (void)dt_sample;
  return false;
}

double signed_clearance_static(const World& world, double x, double y, double robot_r,
                               double margin) {
  return clearance_rects(world, x, y, robot_r, margin);
}

}  // namespace tml
