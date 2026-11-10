// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <tml/mpc_solver.hpp>
#include <tml/scenario.hpp>
#include <tml/tml.hpp>

#include <cmath>
#include <vector>

using namespace tml;

TEST(MpcStep, SetupAndSolveS1) {
  Scenario sc;
  ASSERT_TRUE(load_scenario_file(FIXTURE_DIR "/s1.json", &sc));
  MpcSolver solver;
  MpcConfig cfg;
  cfg.robot = sc.robot;
  cfg.inflation = sc.inflation;
  ASSERT_TRUE(solver.setup(cfg));
  World w{&sc.map, sc.rects.data(), static_cast<int>(sc.rects.size()), nullptr, 0, 0};
  PlanRequest req{sc.start, sc.goal, nullptr, 0, w, sc.dt_ctl, Twist2D{0.55, 0}};
  PlanResult pr;
  EXPECT_EQ(mpc_step(solver, req, cfg, pr), Status::Ok);
  EXPECT_GT(pr.twist.v, 0.5);
}

TEST(MpcStep, CscIndexMapStable) {
  Scenario sc;
  ASSERT_TRUE(load_scenario_file(FIXTURE_DIR "/s1.json", &sc));
  MpcSolver solver;
  MpcConfig cfg;
  cfg.robot = sc.robot;
  cfg.inflation = sc.inflation;
  ASSERT_TRUE(solver.setup(cfg));
  std::vector<int> i0;
  std::vector<int> p0;
  solver.test_snapshot_a_pattern(&i0, &p0);
  World w{&sc.map, nullptr, 0, nullptr, 0, 0};
  PlanRequest req{sc.start, sc.goal, nullptr, 0, w, sc.dt_ctl, Twist2D{0.5, 0}};
  for (int k = 0; k < 14; ++k) {
    req.pose.x = 0.18 * k;
    req.pose.y = 0.03 * std::sin(0.4 * k);
    req.pose.yaw = 0.08 * k;
    PlanResult pr;
    ASSERT_EQ(mpc_step(solver, req, cfg, pr), Status::Ok) << k;
  }
  std::vector<int> i1;
  std::vector<int> p1;
  solver.test_snapshot_a_pattern(&i1, &p1);
  EXPECT_EQ(i0, i1);
  EXPECT_EQ(p0, p1);
}

TEST(MpcStep, MissingSetupReturnsZeroCommand) {
  Scenario sc;
  ASSERT_TRUE(load_scenario_file(FIXTURE_DIR "/s1.json", &sc));
  MpcSolver solver;
  MpcConfig cfg;
  cfg.robot = sc.robot;
  cfg.inflation = sc.inflation;
  World w{&sc.map, nullptr, 0, nullptr, 0, 0};
  PlanRequest req{sc.start, sc.goal, nullptr, 0, w, sc.dt_ctl, Twist2D{0, 0}};
  PlanResult pr;
  pr.twist = Twist2D{1, 1};
  EXPECT_EQ(mpc_step(solver, req, cfg, pr), Status::SolveFailed);
  EXPECT_EQ(pr.twist.v, 0.0);
  EXPECT_EQ(pr.twist.omega, 0.0);
}

TEST(MpcStep, NanPoseReturnsZeroCommand) {
  Scenario sc;
  ASSERT_TRUE(load_scenario_file(FIXTURE_DIR "/s1.json", &sc));
  MpcSolver solver;
  MpcConfig cfg;
  cfg.robot = sc.robot;
  cfg.inflation = sc.inflation;
  ASSERT_TRUE(solver.setup(cfg));
  World w{&sc.map, nullptr, 0, nullptr, 0, 0};
  PlanRequest req{Pose2D{NAN, 0, 0}, sc.goal, nullptr, 0, w, sc.dt_ctl, Twist2D{0, 0}};
  PlanResult pr;
  pr.twist = Twist2D{1, 1};
  EXPECT_EQ(mpc_step(solver, req, cfg, pr), Status::SolveFailed);
  EXPECT_EQ(pr.twist.v, 0.0);
  EXPECT_EQ(pr.twist.omega, 0.0);
}

TEST(Planners, InCollisionStartZeroCommand) {
  Scenario sc;
  ASSERT_TRUE(load_scenario_file(FIXTURE_DIR "/s2.json", &sc));
  World w{&sc.map, sc.rects.data(), static_cast<int>(sc.rects.size()), nullptr, 0, 0};
  // Inside the first staggered box.
  PlanRequest req{Pose2D{0.5, 1.0, 0}, sc.goal, nullptr, 0, w, sc.dt_ctl, Twist2D{0, 0}};
  ASSERT_TRUE(pose_collides(req.pose, w, sc.robot, sc.inflation));
  TebConfig tcfg;
  tcfg.robot = sc.robot;
  tcfg.inflation = sc.inflation;
  TebWorkspace ws;
  PlanResult pr;
  pr.twist = Twist2D{1, 1};
  EXPECT_EQ(teb_step(req, tcfg, ws, pr), Status::InvalidInput);
  EXPECT_STREQ(pr.reason, "invalid_start");
  EXPECT_EQ(pr.twist.v, 0.0);
  EXPECT_EQ(pr.twist.omega, 0.0);

  MpcSolver solver;
  MpcConfig mcfg;
  mcfg.robot = sc.robot;
  mcfg.inflation = sc.inflation;
  ASSERT_TRUE(solver.setup(mcfg));
  PlanResult prm;
  prm.twist = Twist2D{1, 1};
  EXPECT_EQ(mpc_step(solver, req, mcfg, prm), Status::InvalidInput);
  EXPECT_STREQ(prm.reason, "invalid_start");
  EXPECT_EQ(prm.twist.v, 0.0);
  EXPECT_EQ(prm.twist.omega, 0.0);
}

TEST(TebStep, NanPoseReturnsZeroCommand) {
  Scenario sc;
  ASSERT_TRUE(load_scenario_file(FIXTURE_DIR "/s1.json", &sc));
  World w{&sc.map, nullptr, 0, nullptr, 0, 0};
  PlanRequest req{Pose2D{NAN, 0, 0}, sc.goal, nullptr, 0, w, sc.dt_ctl, Twist2D{0, 0}};
  TebConfig cfg;
  cfg.robot = sc.robot;
  TebWorkspace ws;
  PlanResult pr;
  pr.twist = Twist2D{1, 1};
  EXPECT_EQ(teb_step(req, cfg, ws, pr), Status::InvalidInput);
  EXPECT_EQ(pr.twist.v, 0.0);
  EXPECT_EQ(pr.twist.omega, 0.0);
}
