// SPDX-License-Identifier: Apache-2.0
#include <osqp.h>

#include "tml/geometry_query.hpp"
#include "tml/mpc_solver.hpp"
#include "tml/tml.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

namespace tml {

namespace {

constexpr int idx_z(int k, int c) { return (k - 1) * 3 + c; }
constexpr int idx_u(int k, int c) { return 3 * kMpcHorizon + k * 2 + c; }
constexpr int DYN_ROWS = 3 * kMpcHorizon;
constexpr int BOX_BASE = DYN_ROWS;
constexpr int HP_BASE = DYN_ROWS + 4 * kMpcHorizon;

static OSQPSolver* as_solver(void* p) { return static_cast<OSQPSolver*>(p); }

static bool control_collides_moving(const Pose2D& p0, const Twist2D& u, double dt,
                                    const World& world0, const RobotParams& robot,
                                    const Inflation& infl) {
  const int sub = std::max(1, static_cast<int>(std::lround(dt / 0.01)));
  const double ds = dt / sub;
  Pose2D cur = p0;
  double t = world0.t;
  World w = world0;
  if (pose_collides(cur, w, robot, infl)) {
    return true;
  }
  for (int s = 0; s < sub; ++s) {
    Pose2D nxt;
    step_plant(ds, u, cur, &nxt);
    t += ds;
    w.t = t;
    if (pose_collides(nxt, w, robot, infl)) {
      return true;
    }
    cur = nxt;
  }
  return false;
}

void pack_csc(const std::vector<std::vector<std::pair<int, double>>>& cols, std::vector<double>& x,
              std::vector<int>& i, std::vector<int>& p) {
  p.assign(cols.size() + 1, 0);
  x.clear();
  i.clear();
  for (size_t col = 0; col < cols.size(); ++col) {
    auto entries = cols[col];
    std::sort(entries.begin(), entries.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    size_t t = 0;
    while (t < entries.size()) {
      size_t u = t + 1;
      double sum = entries[t].second;
      while (u < entries.size() && entries[u].first == entries[t].first) {
        sum += entries[u].second;
        ++u;
      }
      i.push_back(entries[t].first);
      x.push_back(sum);
      t = u;
    }
    p[col + 1] = static_cast<int>(x.size());
  }
}

double affine_yaw_rhs(double yaw_component) { return wrap_yaw(yaw_component); }

}  // namespace

MpcSolver::MpcSolver() = default;
MpcSolver::~MpcSolver() { cleanup(); }

void MpcSolver::cleanup() {
  if (solver_ != nullptr) {
    osqp_cleanup(as_solver(solver_));
    solver_ = nullptr;
  }
}

bool MpcSolver::setup(const MpcConfig& cfg) {
  cleanup();
  cfg_ = cfg;
  if (cfg.N != kMpcHorizon) {
    return false;
  }
  n_var_ = 5 * cfg.N;
  n_con_ = 11 * cfg.N;
  q_.assign(n_var_, 0.0);
  l_.assign(n_con_, 0.0);
  u_.assign(n_con_, 0.0);

  std::vector<std::vector<std::pair<int, double>>> Pcols(n_var_);
  for (int k = 1; k <= cfg.N; ++k) {
    for (int c = 0; c < 3; ++c) {
      const double w = (k < cfg.N) ? cfg.Q[c] : cfg.Qn[c];
      Pcols[idx_z(k, c)].push_back({idx_z(k, c), 2.0 * w});
    }
  }
  for (int k = 0; k < cfg.N; ++k) {
    Pcols[idx_u(k, 0)].push_back({idx_u(k, 0), 2.0 * (cfg.R[0] + cfg.Rd[0])});
    Pcols[idx_u(k, 1)].push_back({idx_u(k, 1), 2.0 * (cfg.R[1] + cfg.Rd[1])});
  }
  pack_csc(Pcols, P_x_, P_i_, P_p_);

  std::vector<std::vector<std::pair<int, double>>> Acols(n_var_);
  dyn_slots_.clear();
  hp_slots_.clear();

  for (int k = 0; k < cfg.N; ++k) {
    for (int comp = 0; comp < 3; ++comp) {
      const int row = 3 * k + comp;
      const int col_znext = idx_z(k + 1, comp);
      Acols[col_znext].push_back({row, 1.0});
      dyn_slots_.push_back({row, col_znext, 2});
      if (k > 0) {
        for (int j = 0; j < 3; ++j) {
          const int col = idx_z(k, j);
          Acols[col].push_back({row, 0.0});
          dyn_slots_.push_back({row, col, 0});
        }
      }
      for (int j = 0; j < 2; ++j) {
        const int colu = idx_u(k, j);
        Acols[colu].push_back({row, 0.0});
        dyn_slots_.push_back({row, colu, 1});
      }
    }
    const int brow = BOX_BASE + 4 * k;
    Acols[idx_u(k, 0)].push_back({brow + 0, 1.0});
    Acols[idx_u(k, 1)].push_back({brow + 1, 1.0});
    Acols[idx_u(k, 0)].push_back({brow + 2, 1.0});
    Acols[idx_u(k, 1)].push_back({brow + 3, 1.0});
    if (k > 0) {
      Acols[idx_u(k - 1, 0)].push_back({brow + 2, -1.0});
      Acols[idx_u(k - 1, 1)].push_back({brow + 3, -1.0});
    }
    for (int h = 0; h < 4; ++h) {
      const int hrow = HP_BASE + 4 * k + h;
      const int colx = idx_z(k + 1, 0);
      const int coly = idx_z(k + 1, 1);
      Acols[colx].push_back({hrow, 0.0});
      Acols[coly].push_back({hrow, 0.0});
      HpSlot slot;
      slot.row = hrow;
      slot.col_x = colx;
      slot.col_y = coly;
      slot.idx_nx = static_cast<int>(Acols[colx].size()) - 1;
      slot.idx_ny = static_cast<int>(Acols[coly].size()) - 1;
      hp_slots_.push_back(slot);
    }
  }
  pack_csc(Acols, A_x_, A_i_, A_p_);

  for (int k = 0; k < cfg.N; ++k) {
    const int brow = BOX_BASE + 4 * k;
    l_[brow + 0] = -cfg.robot.limits.v_max;
    u_[brow + 0] = cfg.robot.limits.v_max;
    l_[brow + 1] = -cfg.robot.limits.omega_max;
    u_[brow + 1] = cfg.robot.limits.omega_max;
    l_[brow + 2] = -cfg.robot.limits.a_max * cfg.h;
    u_[brow + 2] = cfg.robot.limits.a_max * cfg.h;
    l_[brow + 3] = -cfg.robot.limits.alpha_max * cfg.h;
    u_[brow + 3] = cfg.robot.limits.alpha_max * cfg.h;
    for (int h = 0; h < 4; ++h) {
      const int hrow = HP_BASE + 4 * k + h;
      l_[hrow] = -1.0;
      u_[hrow] = 1e30;
    }
  }
  for (int r = 0; r < DYN_ROWS; ++r) {
    l_[r] = 0;
    u_[r] = 0;
  }

  OSQPSettings settings;
  osqp_set_default_settings(&settings);
  settings.verbose = 0;
  settings.max_iter = 400;
  settings.eps_abs = 1e-4;
  settings.eps_rel = 1e-4;
  settings.adaptive_rho = OSQP_ADAPTIVE_RHO_UPDATE_DISABLED;
  settings.polishing = 0;
  settings.warm_starting = 1;
  settings.check_termination = 25;
  settings.scaling = 10;

  OSQPCscMatrix P, A;
  P.m = n_var_;
  P.n = n_var_;
  P.p = P_p_.data();
  P.i = P_i_.data();
  P.x = P_x_.data();
  P.owned = 0;
  A.m = n_con_;
  A.n = n_var_;
  A.p = A_p_.data();
  A.i = A_i_.data();
  A.x = A_x_.data();
  A.owned = 0;

  OSQPSolver* sp = nullptr;
  for (int col = 0; col < n_var_; ++col) {
    for (int t = P_p_[col]; t < P_p_[col + 1]; ++t) {
      if (P_i_[t] > col) {
        return false;
      }
    }
  }
  std::vector<char> row_has(n_con_, 0);
  for (size_t t = 0; t < A_x_.size(); ++t) {
    row_has[A_i_[t]] = 1;
  }
  for (int r = 0; r < n_con_; ++r) {
    if (!row_has[r]) {
      return false;
    }
    if (l_[r] > u_[r]) {
      return false;
    }
  }

  const OSQPInt ok = osqp_setup(&sp, &P, q_.data(), &A, l_.data(), u_.data(), n_con_, n_var_, &settings);
  solver_ = sp;
  if (ok != 0) {
    fprintf(stderr, "osqp_setup exit %d\n", static_cast<int>(ok));
    return false;
  }
  settings.verbose = 0;
  return sp != nullptr;
}

Status mpc_step(MpcSolver& solver, const PlanRequest& req, const MpcConfig& cfg, PlanResult& out) {
  out = PlanResult{};
  out.reason = "mpc";
  if (!solver.is_setup() || cfg.N != kMpcHorizon || !finite_pose(req.pose) || req.dt <= 0) {
    out.status = Status::SolveFailed;
    out.twist = Twist2D{0, 0};
    return out.status;
  }
  if (pose_collides(req.pose, req.world, cfg.robot, cfg.inflation)) {
    out.status = Status::InvalidInput;
    out.reason = "invalid_start";
    out.twist = Twist2D{0, 0};
    return out.status;
  }

  Pose2D z_ref[kBandMax];
  Twist2D u_ref[kBandMax];
  reference_rollout(req, cfg, z_ref, u_ref);

  for (int i = 0; i < solver.n_var_; ++i) {
    solver.q_[i] = 0.0;
  }
  for (int k = 1; k <= cfg.N; ++k) {
    for (int c = 0; c < 3; ++c) {
      const double w = (k < cfg.N) ? cfg.Q[c] : cfg.Qn[c];
      solver.q_[idx_z(k, c)] = -2.0 * w * (&z_ref[k].x)[c];
    }
  }
  for (int k = 0; k < cfg.N; ++k) {
    solver.q_[idx_u(k, 0)] = -2.0 * cfg.R[0] * u_ref[k].v;
    solver.q_[idx_u(k, 1)] = -2.0 * cfg.R[1] * u_ref[k].omega;
    if (k == 0) {
      solver.q_[idx_u(0, 0)] += -2.0 * cfg.Rd[0] * req.u_prev.v;
      solver.q_[idx_u(0, 1)] += -2.0 * cfg.Rd[1] * req.u_prev.omega;
    } else {
      solver.q_[idx_u(k, 0)] += -2.0 * cfg.Rd[0] * (-u_ref[k - 1].v);
      solver.q_[idx_u(k, 1)] += -2.0 * cfg.Rd[1] * (-u_ref[k - 1].omega);
      solver.q_[idx_u(k - 1, 0)] += -2.0 * cfg.Rd[0] * (-u_ref[k].v);
      solver.q_[idx_u(k - 1, 1)] += -2.0 * cfg.Rd[1] * (-u_ref[k].omega);
    }
  }

  auto set_a = [&](int row, int col, double val) {
    for (int t = solver.A_p_[col]; t < solver.A_p_[col + 1]; ++t) {
      if (solver.A_i_[t] == row) {
        solver.A_x_[t] = val;
        return;
      }
    }
  };
  for (int k = 0; k < cfg.N; ++k) {
    const Pose2D& z_k = k == 0 ? req.pose : z_ref[k];
    double A[3][3], B[3][2], cvec[3];
    const Twist2D ur = u_ref[k];
    unicycle_linearize(z_k, ur, cfg.h, A, B, cvec);
    for (int comp = 0; comp < 3; ++comp) {
      double rhs = cvec[comp];
      if (k == 0) {
        for (int j = 0; j < 3; ++j) {
          rhs += A[comp][j] * (&z_k.x)[j];
        }
      }
      if (k > 0) {
        for (int j = 0; j < 3; ++j) {
          set_a(3 * k + comp, idx_z(k, j), -A[comp][j]);
        }
      }
      for (int j = 0; j < 2; ++j) {
        set_a(3 * k + comp, idx_u(k, j), -B[comp][j]);
      }
      const double val = comp == 2 ? affine_yaw_rhs(rhs) : rhs;
      solver.l_[3 * k + comp] = val;
      solver.u_[3 * k + comp] = val;
    }
  }

  const double ah = cfg.robot.limits.a_max * cfg.h;
  const double alphah = cfg.robot.limits.alpha_max * cfg.h;
  for (int k = 0; k < cfg.N; ++k) {
    const int brow = BOX_BASE + 4 * k;
    if (k == 0) {
      solver.l_[brow + 2] = req.u_prev.v - ah;
      solver.u_[brow + 2] = req.u_prev.v + ah;
      solver.l_[brow + 3] = req.u_prev.omega - alphah;
      solver.u_[brow + 3] = req.u_prev.omega + alphah;
    }
  }

  for (int k = 0; k < cfg.N; ++k) {
    HalfPlane hp[4];
    World w = req.world;
    w.t = req.world.t + k * cfg.h;
    Pose2D zstage = (k == 0) ? req.pose : z_ref[k];
    if (cfg.obstacles_enabled) {
      select_halfplanes(w, zstage, w.t, hp, cfg.robot, cfg.inflation);
    }
    for (int h = 0; h < 4; ++h) {
      const int hrow = HP_BASE + 4 * k + h;
      double nx = 0.0;
      double ny = 0.0;
      double lb = -1.0;
      if (cfg.obstacles_enabled && hp[h].active) {
        nx = hp[h].nx;
        ny = hp[h].ny;
        lb = hp[h].bound;
      }
      solver.l_[hrow] = lb;
      solver.u_[hrow] = 1e30;
      const int colx = idx_z(k + 1, 0);
      const int coly = idx_z(k + 1, 1);
      for (size_t t = solver.A_p_[colx]; t < static_cast<size_t>(solver.A_p_[colx + 1]); ++t) {
        if (solver.A_i_[t] == hrow) {
          solver.A_x_[t] = nx;
        }
      }
      for (size_t t = solver.A_p_[coly]; t < static_cast<size_t>(solver.A_p_[coly + 1]); ++t) {
        if (solver.A_i_[t] == hrow) {
          solver.A_x_[t] = ny;
        }
      }
    }
  }

  OSQPSolver* sp = as_solver(solver.solver_);
  if (osqp_update_data_mat(sp, nullptr, nullptr, 0, solver.A_x_.data(), nullptr,
                           static_cast<OSQPInt>(solver.A_x_.size())) != 0 ||
      osqp_update_data_vec(sp, solver.q_.data(), solver.l_.data(), solver.u_.data()) != 0) {
    out.status = Status::SolveFailed;
    out.twist = Twist2D{0, 0};
    return out.status;
  }
  if (osqp_solve(sp) != 0 || sp->info->status_val != OSQP_SOLVED) {
    out.status = Status::SolveFailed;
    out.twist = Twist2D{0, 0};
    out.reason = "osqp_fail";
    return out.status;
  }
  Twist2D u_cmd{sp->solution->x[idx_u(0, 0)], sp->solution->x[idx_u(0, 1)]};
  if (!finite_twist(u_cmd)) {
    out.status = Status::EmergencyBrake;
    out.twist = Twist2D{0, 0};
    return out.status;
  }
  bool brake = false;
  if (cfg.obstacles_enabled &&
      control_collides_moving(req.pose, u_cmd, req.dt, req.world, cfg.robot, cfg.inflation)) {
    brake = true;
    out.status = Status::EmergencyBrake;
    out.reason = "collision";
  } else {
    out.status = Status::Ok;
  }
  out.twist = clamp_command(u_cmd, req.u_prev, cfg.robot.limits, req.dt, brake);
  out.iterations = sp->info->iter;
  out.n_predicted = cfg.N + 1;
  out.predicted[0] = req.pose;
  Pose2D cur = req.pose;
  for (int k = 0; k < cfg.N; ++k) {
    Twist2D uk{sp->solution->x[idx_u(k, 0)], sp->solution->x[idx_u(k, 1)]};
    step_plant(cfg.h, uk, cur, &cur);
    out.predicted[k + 1] = cur;
  }
  return out.status;
}

}  // namespace tml
