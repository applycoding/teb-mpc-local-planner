// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "tml/types.hpp"

#include <vector>

namespace tml {

class MpcSolver {
 public:
  MpcSolver();
  ~MpcSolver();

  MpcSolver(const MpcSolver&) = delete;
  MpcSolver& operator=(const MpcSolver&) = delete;

  bool setup(const MpcConfig& cfg);
  void cleanup();

  bool is_setup() const { return solver_ != nullptr; }

#ifdef TML_TESTING
  void test_snapshot_a_pattern(std::vector<int>* i, std::vector<int>* p) const {
    *i = A_i_;
    *p = A_p_;
  }
#endif

 private:
  friend Status mpc_step(MpcSolver& solver, const PlanRequest& req, const MpcConfig& cfg,
                         PlanResult& out);
  void* solver_ = nullptr;
  MpcConfig cfg_{};
  int n_var_ = 0;
  int n_con_ = 0;
  std::vector<double> l_;
  std::vector<double> u_;
  std::vector<double> q_;
  std::vector<double> P_x_;
  std::vector<int> P_i_;
  std::vector<int> P_p_;
  std::vector<double> A_x_;
  std::vector<int> A_i_;
  std::vector<int> A_p_;
  struct DynSlot {
    int row;
    int col;
    int kind;  // 0=Az,1=Bu,2=znext
  };
  std::vector<DynSlot> dyn_slots_;
  struct HpSlot {
    int row;
    int col_x;
    int col_y;
    int idx_nx;
    int idx_ny;
  };
  std::vector<HpSlot> hp_slots_;
};

}  // namespace tml
