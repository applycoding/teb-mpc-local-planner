// SPDX-License-Identifier: Apache-2.0
#include "tml/geometry_query.hpp"
#include "tml/tml.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tml {

static double dist_point_rect(double px, double py, double x0, double y0, double x1, double y1) {
  const double cx = std::clamp(px, x0, x1);
  const double cy = std::clamp(py, y0, y1);
  return std::hypot(px - cx, py - cy);
}

struct Cand {
  HalfPlane hp;
  double clearance;
  int id;
  bool mover;
};

void select_halfplanes(const World& world, const Pose2D& ref, double stage_t, HalfPlane out[4],
                       const RobotParams& robot, const Inflation& infl) {
  for (int i = 0; i < 4; ++i) {
    out[i] = {};
    out[i].active = false;
    out[i].bound = -1.0;
  }
  Cand cands[64];
  int nc = 0;
  const double margin = infl.extra_margin_m;
  for (int ri = 0; ri < world.n_rects && nc < 64; ++ri) {
    const ObstacleRect& r = world.rects[ri];
    const double cx = std::clamp(ref.x, r.x0, r.x1);
    const double cy = std::clamp(ref.y, r.y0, r.y1);
    const double dist = std::hypot(ref.x - cx, ref.y - cy);
    double nx = ref.x - cx;
    double ny = ref.y - cy;
    if (std::hypot(nx, ny) < 1e-9) {
      const double mx = std::abs(ref.x - 0.5 * (r.x0 + r.x1));
      const double my = std::abs(ref.y - 0.5 * (r.y0 + r.y1));
      nx = mx >= my ? 1.0 : 0.0;
      ny = mx >= my ? 0.0 : 1.0;
    }
    const double nlen = std::hypot(nx, ny);
    nx /= nlen;
    ny /= nlen;
    HalfPlane hp;
    hp.nx = nx;
    hp.ny = ny;
    hp.bound = nx * cx + ny * cy + robot.radius_m + margin;
    hp.active = true;
    cands[nc++] = Cand{hp, dist - robot.radius_m - margin, ri, false};
  }
  for (int k = 0; k < world.n_movers && nc < 64; ++k) {
    DynamicObstacle m = world.movers[k];
    m.x += m.vx * stage_t;
    m.y += m.vy * stage_t;
    double nx = ref.x - m.x;
    double ny = ref.y - m.y;
    const double dist = std::hypot(nx, ny);
    if (dist < 1e-9) {
      nx = 1.0;
      ny = 0.0;
    } else {
      nx /= dist;
      ny /= dist;
    }
    HalfPlane hp;
    hp.nx = nx;
    hp.ny = ny;
    hp.bound = nx * m.x + ny * m.y + robot.radius_m + m.radius + margin + 0.08;
    hp.active = true;
    cands[nc++] = Cand{hp, dist - robot.radius_m - m.radius - margin, k, true};
  }
  std::sort(cands, cands + nc, [](const Cand& a, const Cand& b) {
    if (a.clearance != b.clearance) {
      return a.clearance < b.clearance;
    }
    if (a.mover != b.mover) {
      return !a.mover;
    }
    return a.id < b.id;
  });
  for (int i = 0; i < 4; ++i) {
    if (i < nc) {
      out[i] = cands[i].hp;
    }
  }
}

}  // namespace tml
