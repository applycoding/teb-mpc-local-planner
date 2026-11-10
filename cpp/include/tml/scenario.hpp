// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "tml/costmap.hpp"
#include "tml/types.hpp"

#include <string>
#include <vector>

namespace tml {

struct Scenario {
  std::string id;
  uint32_t seed = 0;
  Pose2D start{};
  Pose2D goal{};
  double T_max = 30.0;
  double goal_tol_xy = 0.25;
  double goal_tol_yaw = 0.30;
  double dt_ctl = 0.05;
  double dt_plant = 0.01;
  Costmap map;
  std::vector<ObstacleRect> rects;
  std::vector<DynamicObstacle> movers;
  std::vector<Pose2D> polyline;
  RobotParams robot{};
  Inflation inflation{};
};

}  // namespace tml
