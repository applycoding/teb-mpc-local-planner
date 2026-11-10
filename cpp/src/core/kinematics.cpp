// SPDX-License-Identifier: Apache-2.0
#include "tml/tml.hpp"

#include <cmath>

namespace tml {

static double sinc_half(double q) {
  if (std::abs(q) < 1e-12) {
    return 1.0;
  }
  return std::sin(q) / q;
}

static double sinc_half_deriv(double q) {
  if (std::abs(q) < 1e-12) {
    return 0.0;
  }
  return (q * std::cos(q) - std::sin(q)) / (q * q);
}

void increment_to_twist(const Pose2D& p0, const Pose2D& p1, double dt, Twist2D* u,
                        double* kin_residual_xy) {
  if (u == nullptr || dt <= 0.0) {
    if (u) {
      *u = Twist2D{0, 0};
    }
    if (kin_residual_xy) {
      *kin_residual_xy = 0;
    }
    return;
  }
  const double a = wrap_yaw(p1.yaw - p0.yaw);
  const double omega = a / dt;
  const double qh = 0.5 * a;
  const double s = sinc_half(qh);
  const double th = p0.yaw + qh;
  const double bx = dt * s * std::cos(th);
  const double by = dt * s * std::sin(th);
  const double b2 = bx * bx + by * by;
  const double dx = p1.x - p0.x;
  const double dy = p1.y - p0.y;
  double v = 0.0;
  if (b2 > 1e-18) {
    v = (bx * dx + by * dy) / b2;
  }
  u->v = v;
  u->omega = omega;
  if (kin_residual_xy) {
    const double rx = dx - bx * v;
    const double ry = dy - by * v;
    *kin_residual_xy = std::sqrt(rx * rx + ry * ry);
  }
}

void unicycle_linearize(const Pose2D& ref_pose, const Twist2D& ref_u, double h, double A[3][3],
                        double B[3][2], double c[3]) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      A[i][j] = (i == j) ? 1.0 : 0.0;
    }
    for (int j = 0; j < 2; ++j) {
      B[i][j] = 0.0;
    }
    c[i] = 0.0;
  }
  const double a = ref_u.omega * h;
  const double q = 0.5 * a;
  const double s = sinc_half(q);
  const double th = ref_pose.yaw + q;
  const double e0 = std::cos(th);
  const double e1 = std::sin(th);
  const double ep0 = -std::sin(th);
  const double ep1 = std::cos(th);

  Pose2D next = ref_pose;
  step_plant(h, ref_u, ref_pose, &next);

  const double dx = next.x - ref_pose.x;
  const double dy = next.y - ref_pose.y;
  A[0][2] = -dy;
  A[1][2] = dx;

  B[0][0] = h * s * e0;
  B[1][0] = h * s * e1;
  B[2][1] = h;

  const double sd = sinc_half_deriv(q);
  const double v = ref_u.v;
  const double term0 = v * h * h * 0.5 * (sd * e0 + s * ep0);
  const double term1 = v * h * h * 0.5 * (sd * e1 + s * ep1);
  B[0][1] = term0;
  B[1][1] = term1;

  const double z0[3] = {ref_pose.x, ref_pose.y, ref_pose.yaw};
  const double u0[2] = {ref_u.v, ref_u.omega};
  c[0] = next.x - (z0[0] + A[0][2] * z0[2] + B[0][0] * u0[0] + B[0][1] * u0[1]);
  c[1] = next.y - (z0[1] + A[1][2] * z0[2] + B[1][0] * u0[0] + B[1][1] * u0[1]);
  c[2] = next.yaw - (z0[2] + B[2][1] * u0[1]);
  c[2] = wrap_yaw(c[2]);
}

}  // namespace tml
