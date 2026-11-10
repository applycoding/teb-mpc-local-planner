// SPDX-License-Identifier: Apache-2.0
#include "tml/tml.hpp"

#include <algorithm>
#include <cmath>

namespace tml {

Twist2D clamp_command(Twist2D desired, Twist2D u_prev, const Limits& lim, double dt,
                      bool brake) {
  if (brake) {
    return Twist2D{0, 0};
  }
  if (!finite_twist(u_prev)) {
    u_prev = Twist2D{0, 0};
  }
  Twist2D u = desired;
  if (dt > 0) {
    const double dv = std::clamp(u.v - u_prev.v, -lim.a_max * dt, lim.a_max * dt);
    const double dw = std::clamp(u.omega - u_prev.omega, -lim.alpha_max * dt, lim.alpha_max * dt);
    u.v = u_prev.v + dv;
    u.omega = u_prev.omega + dw;
  }
  u.v = std::clamp(u.v, -lim.v_max, lim.v_max);
  u.omega = std::clamp(u.omega, -lim.omega_max, lim.omega_max);
  const double half = 0.5 * lim.track_width_m;
  const auto wheel_ok = [&](double v, double w) {
    return std::abs(v) + half * std::abs(w) <= lim.v_wheel_max + 1e-12;
  };
  if (!wheel_ok(u.v, u.omega)) {
    double lo = 0.0;
    double hi = 1.0;
    for (int i = 0; i < 30; ++i) {
      double mid = 0.5 * (lo + hi);
      double vv = mid * u.v;
      double ww = mid * u.omega;
      if (wheel_ok(vv, ww)) {
        lo = mid;
      } else {
        hi = mid;
      }
    }
    u.v *= lo;
    u.omega *= lo;
  }
  return u;
}

}  // namespace tml
