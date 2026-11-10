// SPDX-License-Identifier: Apache-2.0
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

int tml_reset(void);
int tml_set_scenario(int id);
int tml_load_scenario_json(const char* json);
int tml_step_ticks(int n);

int tml_scenario_id(void);
double tml_sim_time(void);
double tml_teb_sim_time(void);
double tml_mpc_sim_time(void);
double tml_t_max(void);

double tml_teb_x(void);
double tml_teb_y(void);
double tml_mpc_x(void);
double tml_mpc_y(void);
int tml_teb_success(void);
int tml_mpc_success(void);
double tml_teb_path_len(void);
double tml_mpc_path_len(void);
double tml_teb_time_to_goal(void);
double tml_mpc_time_to_goal(void);

double tml_map_origin_x(void);
double tml_map_origin_y(void);
double tml_map_resolution(void);
int tml_map_width(void);
int tml_map_height(void);
double tml_start_x(void);
double tml_start_y(void);
double tml_start_yaw(void);
double tml_goal_x(void);
double tml_goal_y(void);
double tml_goal_yaw(void);

int tml_n_rects(void);
double tml_rect_x0(int i);
double tml_rect_y0(int i);
double tml_rect_x1(int i);
double tml_rect_y1(int i);

int tml_n_movers(void);
double tml_mover_x(int i);
double tml_mover_y(int i);
double tml_mover_radius(int i);
double tml_mover_vx(int i);
double tml_mover_vy(int i);

int tml_occ_count(void);
int tml_occ_i(int k);
int tml_occ_j(int k);

int tml_teb_trace_count(void);
double tml_teb_trace_x(int i);
double tml_teb_trace_y(int i);
int tml_mpc_trace_count(void);
double tml_mpc_trace_x(int i);
double tml_mpc_trace_y(int i);

#ifdef __cplusplus
}
#endif
