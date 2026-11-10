// SPDX-License-Identifier: Apache-2.0
#include "tml/tml.hpp"

#include <cmath>
#include <cstring>

namespace tml {

double wrap_yaw(double yaw) {
  if (!std::isfinite(yaw)) {
    return yaw;
  }
  double y = std::fmod(yaw + M_PI, 2.0 * M_PI);
  if (y < 0) {
    y += 2.0 * M_PI;
  }
  y -= M_PI;
  if (y <= -M_PI + 1e-15) {
    y = M_PI;
  }
  return y;
}

static double sinc_half(double q) {
  if (std::abs(q) < 1e-12) {
    return 1.0;
  }
  if (std::abs(q) < 1e-4) {
  double q2 = q * q;
    return 1.0 - q2 / 6.0 + q2 * q2 / 120.0;
  }
  return std::sin(q) / q;
}

bool finite_pose(const Pose2D& p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw);
}

bool finite_twist(const Twist2D& u) {
  return std::isfinite(u.v) && std::isfinite(u.omega);
}

Status step_plant(double dt, Twist2D u, Pose2D pose, Pose2D* out) {
  if (out == nullptr || dt <= 0.0 || !finite_twist(u) || !finite_pose(pose)) {
    return Status::InvalidDt;
  }
  const double a = u.omega * dt;
  const double q = 0.5 * a;
  const double s = sinc_half(q);
  const double th = pose.yaw + q;
  const double c = std::cos(th);
  const double sn = std::sin(th);
  Pose2D n = pose;
  n.x += u.v * dt * s * c;
  n.y += u.v * dt * s * sn;
  n.yaw = wrap_yaw(pose.yaw + a);
  if (!finite_pose(n)) {
    return Status::InvalidDt;
  }
  *out = n;
  return Status::Ok;
}

}  // namespace tml
