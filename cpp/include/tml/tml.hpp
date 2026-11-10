// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "tml/costmap.hpp"
#include "tml/mpc_solver.hpp"
#include "tml/types.hpp"

namespace tml {

double wrap_yaw(double yaw);
bool finite_pose(const Pose2D& p);
bool finite_twist(const Twist2D& u);

Status step_plant(double dt, Twist2D u, Pose2D pose, Pose2D* out);

void unicycle_linearize(const Pose2D& ref_pose, const Twist2D& ref_u, double h,
                        double A[3][3], double B[3][2], double c[3]);

void increment_to_twist(const Pose2D& p0, const Pose2D& p1, double dt, Twist2D* u,
                        double* kin_residual_xy);

Status teb_step(const PlanRequest& req, const TebConfig& cfg, TebWorkspace& ws,
                PlanResult& out);

double teb_eval_cost(const Band& band, const PlanRequest& req, const TebConfig& cfg,
                     Twist2D u_prev, double* kin_rms);

Status mpc_step(MpcSolver& solver, const PlanRequest& req, const MpcConfig& cfg,
                PlanResult& out);

Twist2D clamp_command(Twist2D desired, Twist2D u_prev, const Limits& lim, double dt,
                      bool brake);

void reference_rollout(const PlanRequest& req, const MpcConfig& cfg, Pose2D z_ref[kBandMax],
                       Twist2D u_ref[kBandMax]);

bool pose_collides(const Pose2D& pose, const World& world, const RobotParams& robot,
                   const Inflation& infl);
bool segment_collides(const Pose2D& a, const Pose2D& b, const World& world,
                      const RobotParams& robot, const Inflation& infl, double dt_sample);
double clearance_at_pose(const Pose2D& pose, const World& world, const RobotParams& robot);
double signed_clearance_static(const World& world, double x, double y, double robot_r,
                               double margin);
double signed_clearance_movers(const World& world, double x, double y, double robot_r,
                               double margin);

Metrics compute_metrics(const Trace& trace, const Pose2D& goal, double goal_tol_xy,
                        double goal_tol_yaw, double dt_plant, const World& world,
                        const RobotParams& robot, const Inflation& infl);

int run_scenario(uint32_t seed, const Scenario& sc, PlannerId id, Trace* trace,
                 TebWorkspace* teb_ws, MpcSolver* mpc_solver, bool mutant,
                 bool brake_only);

bool load_scenario_file(const char* path, Scenario* out);
bool load_scenario_json(const char* json_text, Scenario* out);

}  // namespace tml
