#pragma once
#define XN_VIRTUAL
#include "../app/app_state.hpp"
#include "../util/xn_math.hpp"
#include "../util/xn_vec.hpp"
// #include "xn_gpio.hpp"
#include "xn_ik.hpp"
#include "xn_log.hpp"
#include "xn_pointcloud.hpp"
#include "xn_search.hpp"
#include <arm/xn_gpio.hpp>


#ifndef XN_LIT
#define XN_LIT 1
#endif

namespace xn {

struct CarInfo {
  glm::vec2 &pos;
  float &angle;
  float gridbox_size;
  glm::vec3 &grid_offset;
  std::vector<GridNode> &graph;
  glm::vec3 target_world;
};

int car_move_forward_sim(glm::vec2 &car_pos, glm::vec2 &heading, float d);

int car_turn_sim(float &car_angle, int deg = 90);

void car_follow_path_sim(CarInfo & info, std::vector<GridNode> & path
                           /* ,std::vector<std::pair<float, glm::vec2>> &
                               error_list*/);

} // namespace xn