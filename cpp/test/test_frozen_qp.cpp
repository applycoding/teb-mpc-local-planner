// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <osqp.h>

#include <cmath>

static double eval_obj(int n, const double* x, const double* q, const OSQPCscMatrix* P) {
  double cost = 0.0;
  for (int j = 0; j < n; ++j) {
    cost += q[j] * x[j];
    for (int p = P->p[j]; p < P->p[j + 1]; ++p) {
      const int i = P->i[p];
      cost += 0.5 * x[i] * P->x[p] * x[j];
    }
  }
  return cost;
}

TEST(FrozenQp, CostDecreases) {
  const int n = 2;
  const int m = 1;
  double P_x[2] = {2.0, 2.0};
  OSQPInt P_i[2] = {0, 1};
  OSQPInt P_p[3] = {0, 1, 2};
  OSQPCscMatrix P{n, n, P_p, P_i, P_x, 0};
  double q[2] = {-2.0, -1.0};
  double A_x[2] = {1.0, 1.0};
  OSQPInt A_i[2] = {0, 0};
  OSQPInt A_p[3] = {0, 1, 2};
  OSQPCscMatrix A{m, n, A_p, A_i, A_x, 0};
  double l[1] = {0.0};
  double u[1] = {1.0};

  double x0[2] = {0.9, 0.9};
  const double cost0 = eval_obj(n, x0, q, &P);

  OSQPSettings settings;
  osqp_set_default_settings(&settings);
  settings.verbose = 0;
  settings.polishing = 0;
  settings.adaptive_rho = OSQP_ADAPTIVE_RHO_UPDATE_DISABLED;

  OSQPSolver* solver = nullptr;
  ASSERT_EQ(osqp_setup(&solver, &P, q, &A, l, u, m, n, &settings), 0);
  ASSERT_EQ(osqp_solve(solver), 0);
  ASSERT_EQ(solver->info->status_val, OSQP_SOLVED);
  const double cost1 = eval_obj(n, solver->solution->x, q, &P);
  EXPECT_LT(cost1, cost0 - 1e-6);
  osqp_cleanup(solver);
}
