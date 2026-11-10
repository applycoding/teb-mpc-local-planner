// SPDX-License-Identifier: Apache-2.0
#include "tml/scenario.hpp"
#include "tml/tml.hpp"

#include <nlohmann/json.hpp>

#include <fstream>

namespace tml {

namespace {

bool parse_scenario_json(const nlohmann::json& j, Scenario* out) {
  if (out == nullptr) {
    return false;
  }
  Scenario sc;
  sc.id = j.value("id", "");
  sc.seed = j.value("seed", 0u);
  sc.T_max = j.value("T_max", 30.0);
  sc.goal_tol_xy = j.value("goal_tol_xy", 0.25);
  sc.goal_tol_yaw = j.value("goal_tol_yaw", 0.30);
  sc.dt_ctl = j.value("dt_ctl", 0.05);
  sc.dt_plant = j.value("dt_plant", 0.01);
  const auto& st = j.at("start");
  sc.start.x = st.at("x");
  sc.start.y = st.at("y");
  sc.start.yaw = st.at("yaw");
  const auto& gl = j.at("goal");
  sc.goal.x = gl.at("x");
  sc.goal.y = gl.at("y");
  sc.goal.yaw = gl.value("yaw", 0.0);

  const auto& mapj = j.at("map");
  sc.map.resolution_m = mapj.value("resolution", 0.05);
  sc.map.origin_x = mapj.value("origin_x", -2.0);
  sc.map.origin_y = mapj.value("origin_y", -4.0);
  sc.map.width = mapj.at("width");
  sc.map.height = mapj.at("height");
  sc.map.resize(sc.map.width, sc.map.height);
  sc.map.fill_free_rect(sc.map.origin_x, sc.map.origin_y,
                        sc.map.origin_x + sc.map.width * sc.map.resolution_m,
                        sc.map.origin_y + sc.map.height * sc.map.resolution_m,
                        UnknownPolicy::Free);
  if (mapj.contains("obstacles")) {
    for (const auto& ob : mapj.at("obstacles")) {
      ObstacleRect r{ob.at("x0"), ob.at("y0"), ob.at("x1"), ob.at("y1")};
      sc.rects.push_back(r);
      sc.map.add_rect_occ(r.x0, r.y0, r.x1, r.y1);
    }
  }
  sc.map.rebuild_occupied_index();

  if (j.contains("movers")) {
    for (const auto& mv : j.at("movers")) {
      DynamicObstacle d;
      d.x = mv.at("x");
      d.y = mv.at("y");
      d.radius = mv.at("radius");
      d.vx = mv.value("vx", 0.0);
      d.vy = mv.value("vy", 0.0);
      sc.movers.push_back(d);
    }
  }
  sc.robot.radius_m = 0.22;
  sc.inflation.extra_margin_m = 0.05;
  *out = sc;
  return true;
}

}  // namespace

bool load_scenario_file(const char* path, Scenario* out) {
  if (out == nullptr || path == nullptr) {
    return false;
  }
  std::ifstream f(path);
  if (!f) {
    return false;
  }
  nlohmann::json j;
  f >> j;
  return parse_scenario_json(j, out);
}

bool load_scenario_json(const char* json_text, Scenario* out) {
  if (out == nullptr || json_text == nullptr) {
    return false;
  }
  try {
    const auto j = nlohmann::json::parse(json_text);
    return parse_scenario_json(j, out);
  } catch (...) {
    return false;
  }
}

}  // namespace tml
