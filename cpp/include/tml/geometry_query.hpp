// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "tml/types.hpp"

namespace tml {

struct HalfPlane {
  double nx = 0;
  double ny = 0;
  double bound = 0;  // n·o + r_eff
  bool active = false;
};

void select_halfplanes(const World& world, const Pose2D& ref, double stage_t,
                     HalfPlane out[4], const RobotParams& robot, const Inflation& infl);

}  // namespace tml
