// SPDX-License-Identifier: Apache-2.0
#include "tml/costmap.hpp"

#include <algorithm>
#include <cmath>

namespace tml {

void Costmap::resize(int w, int h) {
  width = w;
  height = h;
  occ.assign(static_cast<size_t>(w * h), Occupancy::Unknown);
  display_cost.assign(static_cast<size_t>(w * h), 255);
  occupied_cells.clear();
  unknown_lethal_scan = true;
}

Occupancy Costmap::at_index(int i, int j) const {
  if (i < 0 || j < 0 || i >= width || j >= height) {
    return Occupancy::Unknown;
  }
  return occ[static_cast<size_t>(j * width + i)];
}

void Costmap::set_occ(int i, int j, Occupancy o) {
  if (i < 0 || j < 0 || i >= width || j >= height) {
    return;
  }
  occ[static_cast<size_t>(j * width + i)] = o;
  display_cost[static_cast<size_t>(j * width + i)] =
      (o == Occupancy::Occupied) ? 254 : (o == Occupancy::Unknown ? 255 : 0);
  (void)o;
}

bool Costmap::in_map(double x, double y) const {
  return x >= origin_x && y >= origin_y && x < origin_x + width * resolution_m &&
         y < origin_y + height * resolution_m;
}

void Costmap::world_to_cell(double x, double y, int* i, int* j) const {
  *i = static_cast<int>(std::floor((x - origin_x) / resolution_m));
  *j = static_cast<int>(std::floor((y - origin_y) / resolution_m));
}

void Costmap::cell_corners(int i, int j, double* x0, double* y0, double* x1, double* y1) const {
  *x0 = origin_x + i * resolution_m;
  *y0 = origin_y + j * resolution_m;
  *x1 = *x0 + resolution_m;
  *y1 = *y0 + resolution_m;
}

void Costmap::add_rect_occ(double x0, double y0, double x1, double y1) {
  int i0, j0, i1, j1;
  world_to_cell(x0, y0, &i0, &j0);
  world_to_cell(x1 - 1e-9, y1 - 1e-9, &i1, &j1);
  i0 = std::max(0, i0);
  j0 = std::max(0, j0);
  i1 = std::min(width - 1, i1);
  j1 = std::min(height - 1, j1);
  for (int j = j0; j <= j1; ++j) {
    for (int i = i0; i <= i1; ++i) {
      set_occ(i, j, Occupancy::Occupied);
    }
  }
}

void Costmap::fill_free_rect(double x0, double y0, double x1, double y1, UnknownPolicy unk) {
  const Occupancy def = unk == UnknownPolicy::Lethal ? Occupancy::Unknown : Occupancy::Free;
  if (unk == UnknownPolicy::Free) {
    unknown_lethal_scan = false;
  }
  int i0, j0, i1, j1;
  world_to_cell(x0, y0, &i0, &j0);
  world_to_cell(x1 - 1e-9, y1 - 1e-9, &i1, &j1);
  i0 = std::max(0, i0);
  j0 = std::max(0, j0);
  i1 = std::min(width - 1, i1);
  j1 = std::min(height - 1, j1);
  for (int j = j0; j <= j1; ++j) {
    for (int i = i0; i <= i1; ++i) {
      if (at_index(i, j) != Occupancy::Occupied) {
        set_occ(i, j, def);
      }
    }
  }
}

void Costmap::rebuild_occupied_index() {
  occupied_cells.clear();
  for (int j = 0; j < height; ++j) {
    for (int i = 0; i < width; ++i) {
      if (at_index(i, j) == Occupancy::Occupied) {
        occupied_cells.push_back({i, j});
      }
    }
  }
}

}  // namespace tml
