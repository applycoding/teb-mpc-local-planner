// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <tml/wasm_api.hpp>

#include <cmath>

TEST(WasmExports, SeparateSimTimePerPlanner) {
  ASSERT_EQ(tml_set_scenario(1), 0);
  EXPECT_NEAR(tml_map_origin_x(), -2.0, 1e-12);
  EXPECT_NEAR(tml_map_origin_y(), -4.0, 1e-12);
  EXPECT_NEAR(tml_map_resolution(), 0.05, 1e-12);
  EXPECT_NEAR(tml_teb_sim_time(), 0.0, 1e-12);
  EXPECT_NEAR(tml_mpc_sim_time(), 0.0, 1e-12);
  ASSERT_EQ(tml_step_ticks(20), 20);
  EXPECT_NEAR(tml_teb_sim_time(), 1.0, 1e-9);
  EXPECT_NEAR(tml_mpc_sim_time(), 1.0, 1e-9);
  EXPECT_NEAR(tml_sim_time(), 1.0, 1e-9);
  EXPECT_GT(tml_teb_path_len(), 0.1);
  EXPECT_GT(tml_mpc_path_len(), 0.1);
}

TEST(WasmExports, S4MapOriginAndBothSucceed) {
  ASSERT_EQ(tml_set_scenario(4), 0);
  EXPECT_NEAR(tml_map_origin_x(), -6.0, 1e-12);
  EXPECT_NEAR(tml_map_origin_y(), -4.0, 1e-12);
  EXPECT_EQ(tml_map_width(), 360);
  EXPECT_EQ(tml_map_height(), 160);
  EXPECT_NEAR(tml_goal_x(), -2.0, 1e-12);
  EXPECT_NEAR(tml_goal_y(), 0.5, 1e-12);
  EXPECT_NEAR(tml_t_max(), 25.0, 1e-12);

  int ticks = 0;
  for (int i = 0; i < 30; ++i) {
    if (tml_teb_success() && tml_mpc_success()) {
      break;
    }
    ticks += tml_step_ticks(20);
  }
  EXPECT_EQ(tml_teb_success(), 1);
  EXPECT_EQ(tml_mpc_success(), 1);
  EXPECT_LE(tml_teb_sim_time(), 25.0 + 1e-9);
  EXPECT_LE(tml_mpc_sim_time(), 25.0 + 1e-9);
  EXPECT_LT(tml_teb_sim_time(), 25.0);
  EXPECT_LT(tml_mpc_sim_time(), 25.0);
  EXPECT_GT(ticks, 0);
}

TEST(WasmExports, S2ExportsOccupiedGeometry) {
  ASSERT_EQ(tml_set_scenario(2), 0);
  EXPECT_GE(tml_n_rects(), 2);
  EXPECT_GT(tml_occ_count(), 0);
  EXPECT_LT(tml_rect_x0(0), tml_rect_x1(0));
}
