// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>

namespace tml {

struct Pose2D {
  double x = 0;
  double y = 0;
  double yaw = 0;
};

struct Twist2D {
  double v = 0;
  double omega = 0;
};

struct Limits {
  double v_max = 0.8;
  double omega_max = 1.5;
  double a_max = 1.0;
  double alpha_max = 3.0;
  double v_wheel_max = 1.025;
  double track_width_m = 0.30;
};

struct RobotParams {
  double radius_m = 0.22;
  Limits limits;
};

enum class Occupancy : uint8_t { Free = 0, Occupied = 1, Unknown = 2 };
enum class UnknownPolicy { Lethal, Free };

struct Inflation {
  double extra_margin_m = 0.05;
  bool collide_inscribed = false;
  UnknownPolicy unknown_policy = UnknownPolicy::Lethal;
};

struct DynamicObstacle {
  double x = 0;
  double y = 0;
  double radius = 0;
  double vx = 0;
  double vy = 0;
};

struct ObstacleRect {
  double x0, y0, x1, y1;
};

class Costmap;

struct World {
  const Costmap* map = nullptr;
  const ObstacleRect* rects = nullptr;
  int n_rects = 0;
  const DynamicObstacle* movers = nullptr;
  int n_movers = 0;
  double t = 0;
};

constexpr int kBandMax = 30;
constexpr int kMpcHorizon = 25;
constexpr int kHalfPlaneSlots = 4;

struct Band {
  Pose2D poses[kBandMax];
  double dt[kBandMax - 1];
  int M = 0;
};

struct TebConfig {
  int M = 25;
  int max_iters = 25;
  double dt_min = 0.05;
  double dt_max = 0.30;
  double w_time = 1.0;
  double w_lim = 10.0;
  double w_kin = 50.0;
  double w_obs = 20.0;
  double w_goal_xy = 40.0;
  double w_goal_yaw = 8.0;
  RobotParams robot;
  Inflation inflation;
};

struct TebWorkspace {
  double x[4 * kBandMax];
  double residual[512];
  double jacobian[512 * (4 * kBandMax)];
  int n_residual = 0;
  int n_var = 0;
};

struct MpcConfig {
  int N = kMpcHorizon;
  double h = 0.05;
  int n_halfplanes = kHalfPlaneSlots;
  double Q[3] = {2, 2, 0.4};
  double R[2] = {0.05, 0.05};
  double Rd[2] = {0.4, 0.15};
  double Qn[3] = {8, 8, 1};
  RobotParams robot;
  Inflation inflation;
  bool obstacles_enabled = true;
};

enum class PlannerId { TebStyle, MpcOsqp };
enum class Status {
  Ok,
  InvalidDt,
  InvalidInput,
  SolveFailed,
  NearGoal,
  EmergencyBrake
};

struct PlanRequest {
  Pose2D pose{};
  Pose2D goal{};
  const Pose2D* polyline = nullptr;
  int n_polyline = 0;
  World world{};
  double dt = 0.05;
  Twist2D u_prev{};
  bool obstacle_ref = true;
};

struct PlanResult {
  Twist2D twist{};
  Pose2D predicted[kBandMax];
  int n_predicted = 0;
  Status status = Status::InvalidInput;
  int iterations = 0;
  const char* reason = "unset";
  double cost_initial = 0;
  double cost_final = 0;
};

struct Metrics {
  int success = 0;
  const char* reason = nullptr;
  double clearance_m = 0;
  double time_to_goal_s = 0;
  double smoothness = 0;
  double jerk = 0;
  double yaw_jerk = 0;
  double compute_ms = 0;
};

struct PlantSample {
  double t = 0;
  Pose2D pose{};
  Twist2D cmd{};
};

struct Trace {
  static constexpr int kMaxSamples = 4000;
  PlantSample samples[kMaxSamples];
  int n = 0;
};

struct Scenario;

}  // namespace tml
