// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <tml/mpc_solver.hpp>
#include <tml/scenario.hpp>
#include <tml/tml.hpp>

#include <cmath>
#include <string>

using namespace tml;

static std::string fixture(const char* name) {
  return std::string(FIXTURE_DIR) + "/" + name;
}

static Metrics run_fixture(const char* name, PlannerId id, bool mutant, bool brake_only) {
  Scenario sc;
  if (!load_scenario_file(fixture(name).c_str(), &sc)) {
    ADD_FAILURE() << "load " << name;
    return Metrics{};
  }
  Trace tr{};
  TebWorkspace ws{};
  MpcSolver mpc;
  run_scenario(sc.seed, sc, id, &tr, &ws, &mpc, mutant, brake_only);
  return compute_metrics(tr, sc.goal, sc.goal_tol_xy, sc.goal_tol_yaw, sc.dt_plant,
                         World{&sc.map, sc.rects.data(), static_cast<int>(sc.rects.size()),
                               sc.movers.data(), static_cast<int>(sc.movers.size()), 0},
                         sc.robot, sc.inflation);
}

TEST(Scenarios, S1TebMpc) {
  auto mt = run_fixture("s1.json", PlannerId::TebStyle, false, false);
  auto mm = run_fixture("s1.json", PlannerId::MpcOsqp, false, false);
  EXPECT_EQ(mt.success, 1);
  EXPECT_EQ(mm.success, 1);
  EXPECT_LT(mt.time_to_goal_s, 25.0);
  EXPECT_LT(mm.time_to_goal_s, 25.0);
  EXPECT_TRUE(std::isnan(mt.clearance_m));
}

TEST(Scenarios, S2Clearance) {
  auto mt = run_fixture("s2.json", PlannerId::TebStyle, false, false);
  auto mm = run_fixture("s2.json", PlannerId::MpcOsqp, false, false);
  EXPECT_EQ(mt.success, 1) << (mt.reason ? mt.reason : "");
  EXPECT_EQ(mm.success, 1) << (mm.reason ? mm.reason : "");
  EXPECT_GT(mt.clearance_m, 0.0);
  EXPECT_GT(mm.clearance_m, 0.0);
}

TEST(Scenarios, S3MutantCollides) {
  auto mt = run_fixture("s3.json", PlannerId::TebStyle, true, false);
  auto mm = run_fixture("s3.json", PlannerId::MpcOsqp, true, false);
  EXPECT_EQ(mt.success, 0);
  EXPECT_EQ(mm.success, 0);
  EXPECT_STREQ(mt.reason, "collision");
  EXPECT_STREQ(mm.reason, "collision");
  auto dt = run_fixture("s3.json", PlannerId::TebStyle, false, false);
  auto dm = run_fixture("s3.json", PlannerId::MpcOsqp, false, false);
  EXPECT_EQ(dt.success, 1) << (dt.reason ? dt.reason : "");
  EXPECT_EQ(dm.success, 1) << (dm.reason ? dm.reason : "");
}

TEST(Scenarios, BrakeOnlyFailsS1) {
  auto m = run_fixture("s1.json", PlannerId::TebStyle, false, true);
  EXPECT_EQ(m.success, 0);
  EXPECT_STREQ(m.reason, "timeout");
  EXPECT_TRUE(std::isnan(m.time_to_goal_s));
}

TEST(Scenarios, S3AndS4Gates) {
  Scenario sc3;
  ASSERT_TRUE(load_scenario_file(fixture("s3.json").c_str(), &sc3));
  Trace tr3{};
  TebWorkspace ws{};
  MpcSolver mpc{};
  run_scenario(sc3.seed, sc3, PlannerId::TebStyle, &tr3, &ws, &mpc, false, false);
  double max_lat = 0;
  for (int i = 0; i < tr3.n; ++i) {
    max_lat = std::max(max_lat, std::abs(tr3.samples[i].pose.y));
  }
  auto m3 = compute_metrics(
      tr3, sc3.goal, sc3.goal_tol_xy, sc3.goal_tol_yaw, sc3.dt_plant,
      World{&sc3.map, sc3.rects.data(), static_cast<int>(sc3.rects.size()), sc3.movers.data(),
            static_cast<int>(sc3.movers.size()), 0},
      sc3.robot, sc3.inflation);
  EXPECT_EQ(m3.success, 1) << m3.reason;
  EXPECT_GT(max_lat, 0.3);
  EXPECT_LT(m3.time_to_goal_s, 30.0);
  auto mm3 = run_fixture("s3.json", PlannerId::MpcOsqp, false, false);
  EXPECT_EQ(mm3.success, 1) << (mm3.reason ? mm3.reason : "");
  EXPECT_LT(mm3.time_to_goal_s, 30.0);
  auto t4 = run_fixture("s4.json", PlannerId::TebStyle, false, false);
  auto m4 = run_fixture("s4.json", PlannerId::MpcOsqp, false, false);
  EXPECT_EQ(t4.success, 1) << (t4.reason ? t4.reason : "");
  EXPECT_EQ(m4.success, 1) << (m4.reason ? m4.reason : "");
  EXPECT_LT(t4.time_to_goal_s, 25.0);
  EXPECT_LT(m4.time_to_goal_s, 25.0);
}
