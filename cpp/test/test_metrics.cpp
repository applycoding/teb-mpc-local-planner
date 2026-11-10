// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <tml/tml.hpp>

#include <cmath>

using namespace tml;

TEST(Metrics, SmoothnessRightAngle) {
  Trace tr{};
  const double dt = 0.01;
  Pose2D goal{2, 2, M_PI / 2};
  for (int i = 0; i <= 100; ++i) {
    tr.samples[tr.n++] = PlantSample{static_cast<double>(i) * dt,
                                     Pose2D{std::min(2.0, i * 0.02), 0, 0}, Twist2D{0.2, 0}};
  }
  for (int i = 1; i <= 100; ++i) {
    const double yaw = (i == 100) ? M_PI / 2 : 0.0;
    tr.samples[tr.n++] = PlantSample{1.0 + static_cast<double>(i) * dt,
                                     Pose2D{2.0, std::min(2.0, i * 0.02), yaw}, Twist2D{0.2, 0}};
  }
  tr.samples[tr.n - 1].pose = goal;
  Costmap map;
  map.origin_x = -5;
  map.origin_y = -5;
  map.resize(200, 200);
  map.fill_free_rect(-5, -5, 5, 5, UnknownPolicy::Free);
  World w{&map, nullptr, 0, 0};
  RobotParams rp{};
  Inflation inf{};
  auto m = compute_metrics(tr, goal, 0.25, 0.3, dt, w, rp, inf);
  EXPECT_NEAR(m.smoothness, (M_PI / 2) / 4.0, 1e-9);
}

TEST(Metrics, ClearanceZeroOnCollision) {
  Trace tr{};
  tr.samples[tr.n++] = PlantSample{0.0, Pose2D{0.75, 0, 0}, Twist2D{0, 0}};
  Costmap map;
  map.origin_x = -2;
  map.origin_y = -2;
  map.resize(80, 80);
  map.fill_free_rect(-2, -2, 2, 2, UnknownPolicy::Lethal);
  map.add_rect_occ(0.5, -0.5, 1.0, 0.5);
  ObstacleRect rect{0.5, -0.5, 1.0, 0.5};
  World w{&map, &rect, 1, nullptr, 0, 0};
  RobotParams rp{};
  Inflation infl{};
  auto m = compute_metrics(tr, Pose2D{8, 0, 0}, 0.25, 0.3, 0.01, w, rp, infl);
  EXPECT_EQ(m.success, 0);
  EXPECT_STREQ(m.reason, "collision");
  EXPECT_EQ(m.clearance_m, 0.0);
}

TEST(Metrics, AlreadyAtGoalSmoothnessZero) {
  Trace tr{};
  Pose2D goal{1, 1, 0};
  tr.samples[tr.n++] = PlantSample{0.0, goal, Twist2D{0, 0}};
  Costmap map;
  map.origin_x = -5;
  map.origin_y = -5;
  map.resize(200, 200);
  map.fill_free_rect(-5, -5, 5, 5, UnknownPolicy::Free);
  World w{&map, nullptr, 0, nullptr, 0, 0};
  RobotParams rp{};
  Inflation infl{};
  infl.unknown_policy = UnknownPolicy::Free;
  auto m = compute_metrics(tr, goal, 0.25, 0.3, 0.01, w, rp, infl);
  EXPECT_EQ(m.success, 1);
  EXPECT_NEAR(m.smoothness, 0.0, 1e-12);
}
