// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <tml/costmap.hpp>
#include <tml/tml.hpp>

using namespace tml;

TEST(Geometry, TangencyNotCollision) {
  Costmap map;
  map.origin_x = -2;
  map.origin_y = -2;
  map.resize(80, 80);
  map.fill_free_rect(-2, -2, 2, 2, UnknownPolicy::Lethal);
  map.add_rect_occ(0.5, -0.5, 1.0, 0.5);
  const double r = 0.22;
  const double margin = 0.05;
  Inflation infl;
  ObstacleRect rect{0.5, -0.5, 1.0, 0.5};
  World w{&map, &rect, 1, nullptr, 0, 0};
  EXPECT_FALSE(pose_collides(Pose2D{0.5 - r - margin - 1e-6, 0, 0}, w, RobotParams{}, infl));
  EXPECT_TRUE(pose_collides(Pose2D{0.7, 0, 0}, w, RobotParams{}, infl));
}

TEST(HalfPlane, OffsetRequiresDotO) {
  Costmap map;
  map.origin_x = 0;
  map.origin_y = 0;
  map.resize(40, 40);
  map.fill_free_rect(0, 0, 2, 2, UnknownPolicy::Lethal);
  map.add_rect_occ(1.0, 1.0, 1.5, 1.5);
  Pose2D ref{2.0, 1.25, 0};
  const double margin = 0.05;
  const double r = 0.22;
  double x0, y0, x1, y1;
  map.cell_corners(20, 25, &x0, &y0, &x1, &y1);
  const double ox = x0;
  const double oy = 1.25;
  const double nx = 1.0;
  const double ny = 0.0;
  const double bound = nx * ox + ny * oy + r + margin;
  const double bad_bound = r + margin;
  EXPECT_GT(bound, bad_bound + 0.5);
  EXPECT_GE(nx * ref.x + ny * ref.y, bound);
}
