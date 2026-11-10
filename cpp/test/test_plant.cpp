// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <tml/tml.hpp>

#include <cmath>

using namespace tml;

TEST(Unicycle, StraightTwoSeconds) {
  Pose2D p0{0, 0, 0};
  Pose2D out{};
  ASSERT_EQ(step_plant(2.0, Twist2D{1, 0}, p0, &out), Status::Ok);
  EXPECT_NEAR(out.x, 2.0, 1e-12);
  EXPECT_NEAR(out.y, 0.0, 1e-12);
}

TEST(Unicycle, QuarterTurn) {
  Pose2D p0{0, 0, 0};
  Pose2D out{};
  ASSERT_EQ(step_plant(1.0, Twist2D{1, M_PI / 2}, p0, &out), Status::Ok);
  EXPECT_NEAR(out.x, 2.0 / M_PI, 1e-10);
  EXPECT_NEAR(out.y, 2.0 / M_PI, 1e-10);
  EXPECT_NEAR(out.yaw, M_PI / 2, 1e-10);
}

TEST(Wrap, PiAndNegPi) {
  EXPECT_NEAR(wrap_yaw(M_PI), M_PI, 1e-12);
  EXPECT_NEAR(wrap_yaw(-M_PI), M_PI, 1e-12);
  EXPECT_NEAR(wrap_yaw(M_PI + 0.1), M_PI + 0.1 - 2 * M_PI, 1e-12);
}

TEST(Unicycle, Substeps) {
  Pose2D p0{0, 0, 0};
  Pose2D a{}, b{};
  step_plant(0.05, Twist2D{0.7, 0.2}, p0, &a);
  Pose2D p = p0;
  for (int i = 0; i < 5; ++i) {
    Pose2D n{};
    step_plant(0.01, Twist2D{0.7, 0.2}, p, &n);
    p = n;
  }
  b = p;
  EXPECT_NEAR(a.x, b.x, 1e-12);
  EXPECT_NEAR(a.y, b.y, 1e-12);
  EXPECT_NEAR(a.yaw, b.yaw, 1e-12);
}

TEST(Unicycle, InvalidDt) {
  Pose2D out{1, 2, 3};
  EXPECT_NE(step_plant(0.0, Twist2D{1, 0}, Pose2D{}, &out), Status::Ok);
  EXPECT_EQ(out.x, 1);
}

TEST(Unicycle, Reverse) {
  Pose2D p0{0, 0, 0};
  Pose2D out{};
  ASSERT_EQ(step_plant(2.0, Twist2D{-1, 0}, p0, &out), Status::Ok);
  EXPECT_NEAR(out.x, -2.0, 1e-12);
  EXPECT_NEAR(out.y, 0.0, 1e-12);
  EXPECT_NEAR(out.yaw, 0.0, 1e-12);
}

TEST(Unicycle, TinyOmegaBothSigns) {
  Pose2D p0{0, 0, 0};
  for (double w : {1e-9, -1e-9, 1e-6, -1e-6}) {
    Pose2D out{};
    ASSERT_EQ(step_plant(1.0, Twist2D{1.0, w}, p0, &out), Status::Ok) << w;
    // sinc map: x = v dt sinc(w dt / 2) cos(yaw + w dt / 2)
    const double q = 0.5 * w * 1.0;
    const double s = (std::abs(q) < 1e-12) ? 1.0 : std::sin(q) / q;
    const double th = q;
    EXPECT_NEAR(out.x, 1.0 * s * std::cos(th), 1e-12) << w;
    EXPECT_NEAR(out.y, 1.0 * s * std::sin(th), 1e-12) << w;
    EXPECT_NEAR(out.yaw, wrap_yaw(w), 1e-12) << w;
  }
}

TEST(Unicycle, InvalidDtStatus) {
  Pose2D out{1, 2, 3};
  EXPECT_EQ(step_plant(0.0, Twist2D{1, 0}, Pose2D{}, &out), Status::InvalidDt);
  EXPECT_EQ(out.x, 1);
  EXPECT_EQ(step_plant(-0.05, Twist2D{1, 0}, Pose2D{}, &out), Status::InvalidDt);
  EXPECT_EQ(out.x, 1);
  EXPECT_EQ(step_plant(0.05, Twist2D{1, 0}, Pose2D{NAN, 0, 0}, &out), Status::InvalidDt);
  EXPECT_EQ(out.x, 1);
}
