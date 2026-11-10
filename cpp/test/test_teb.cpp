// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <tml/scenario.hpp>
#include <tml/tml.hpp>

#include <algorithm>
#include <cmath>

using namespace tml;

TEST(Teb, ObjectiveDecreasesOnFreeMap) {
  Scenario sc;
  ASSERT_TRUE(load_scenario_file(FIXTURE_DIR "/s1.json", &sc));
  World w{&sc.map, nullptr, 0, nullptr, 0, 0};
  PlanRequest req{sc.start, sc.goal, nullptr, 0, w, sc.dt_ctl, Twist2D{0, 0}};
  TebConfig cfg;
  cfg.w_obs = 0;
  cfg.robot = sc.robot;
  TebWorkspace ws;
  PlanResult pr;
  EXPECT_EQ(teb_step(req, cfg, ws, pr), Status::Ok);
  EXPECT_GT(pr.cost_initial, 0);
  EXPECT_LT(pr.cost_final, pr.cost_initial - 1e-6);
}

TEST(Teb, CrookedBandImprovesOnFreeMap) {
  Scenario sc;
  ASSERT_TRUE(load_scenario_file(FIXTURE_DIR "/s1.json", &sc));
  World w{&sc.map, nullptr, 0, nullptr, 0, 0};
  PlanRequest req{sc.start, sc.goal, nullptr, 0, w, sc.dt_ctl, Twist2D{0, 0}};
  TebConfig cfg;
  cfg.w_obs = 0;
  cfg.max_iters = 25;
  cfg.robot = sc.robot;
  // Reconstruct the same crooked initial band teb_step uses when w_obs <= 0.
  MpcConfig rc;
  rc.robot = cfg.robot;
  rc.inflation = cfg.inflation;
  Pose2D zr[kBandMax];
  Twist2D ur[kBandMax];
  reference_rollout(req, rc, zr, ur);
  Band init{};
  init.M = cfg.M;
  init.poses[0] = req.pose;
  for (int i = 1; i < cfg.M; ++i) {
    init.poses[i] = (i <= rc.N) ? zr[i] : req.goal;
    init.dt[i - 1] = std::clamp(req.dt, cfg.dt_min, cfg.dt_max);
  }
  for (int i = 2; i < std::min(8, init.M - 1); i += 2) {
    init.poses[i].y += 0.11 * ((i % 4 == 2) ? 1.0 : -1.0);
  }
  double kin0 = 0;
  const double cost0 = teb_eval_cost(init, req, cfg, req.u_prev, &kin0);
  TebWorkspace ws;
  PlanResult pr;
  EXPECT_EQ(teb_step(req, cfg, ws, pr), Status::Ok);
  EXPECT_LE(pr.iterations, 25);
  EXPECT_LT(pr.cost_final, pr.cost_initial - 1e-6);
  EXPECT_NEAR(pr.cost_initial, cost0, 1e-9);
  Band fin{};
  fin.M = pr.n_predicted;
  for (int i = 0; i < fin.M; ++i) {
    fin.poses[i] = pr.predicted[i];
  }
  const int n_pose = 3 * (fin.M - 1);
  for (int i = 0; i < fin.M - 1; ++i) {
    fin.dt[i] = ws.x[n_pose + i];
  }
  double kin1 = 0;
  const double cost1 = teb_eval_cost(fin, req, cfg, req.u_prev, &kin1);
  EXPECT_LT(cost1, cost0 - 1e-6);
  EXPECT_LT(kin1, kin0);
}
