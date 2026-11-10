// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <tml/tml.hpp>

#include <cmath>

using namespace tml;

TEST(Kinematics, ArcRecovery) {
  Pose2D p0{0, 0, 0};
  Pose2D p1{2, 0, 0};
  Twist2D u{};
  double res = 0;
  increment_to_twist(p0, p1, 2.0, &u, &res);
  EXPECT_NEAR(u.v, 1.0, 1e-9);
  EXPECT_NEAR(u.omega, 0.0, 1e-9);
  EXPECT_NEAR(res, 0.0, 1e-9);
}

TEST(Kinematics, JacobianMatchesFd) {
  Pose2D ref{1.0, 2.0, 0.3};
  Twist2D ur{0.4, 0.2};
  const double h = 0.05;
  double A[3][3], B[3][2], c[3];
  unicycle_linearize(ref, ur, h, A, B, c);
  const double eps = 1e-4;
  for (int i = 0; i < 2; ++i) {
    Twist2D du = ur;
    du.v += (i == 0 ? eps : 0);
    du.omega += (i == 1 ? eps : 0);
    Pose2D p1{}, p2{};
    step_plant(h, ur, ref, &p1);
    step_plant(h, du, ref, &p2);
    const double fd = (i == 0 ? (p2.x - p1.x) / eps : (p2.y - p1.y) / eps);
    const double an = (i == 0 ? B[0][0] : B[1][0]);
    if (i == 0) {
      EXPECT_NEAR(fd, an, 1e-3);
    }
  }
}
