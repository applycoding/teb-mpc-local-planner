// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "tml/types.hpp"

#include <cstdint>
#include <vector>

namespace tml {

class Costmap {
 public:
  double resolution_m = 0.05;
  double origin_x = 0;
  double origin_y = 0;
  int width = 0;
  int height = 0;
  std::vector<Occupancy> occ;
  std::vector<uint8_t> display_cost;
  std::vector<std::pair<int, int>> occupied_cells;
  bool unknown_lethal_scan = true;

  void resize(int w, int h);

  Occupancy at_index(int i, int j) const;
  void set_occ(int i, int j, Occupancy o);

  bool in_map(double x, double y) const;
  void world_to_cell(double x, double y, int* i, int* j) const;
  void cell_corners(int i, int j, double* x0, double* y0, double* x1, double* y1) const;

  void add_rect_occ(double x0, double y0, double x1, double y1);
  void fill_free_rect(double x0, double y0, double x1, double y1, UnknownPolicy unk);
  void rebuild_occupied_index();
};

}  // namespace tml
