#pragma once
#include "xn_net.hpp"
#include "xn_pointcloud.hpp"
#include "xn_search.hpp"

namespace xn {

struct Rover {
  SOCKET esp32;

  float wheelSeparation;
  float wheelDiameter;
  unsigned motorCpr;

  float rotation = 0; // CCW rotation around +y axis
  glm::vec2 position{0};
  glm::ivec2 scan_range{700, 800};
  glm::vec3 target;

  void lidar_scan(std::vector<std::vector<glm::vec3>> &nav_verts,
                  xn::GridGraph &navgraph);

  void lidar_scan_error_correct(PointCloud &pc_new, const PointCloud &pc_prev,
                                int iterations = 2);

  // void lidar_scan_ex(PointCloud &pc_new, int servo_step, int step_incr,
  //                   int step_max);

  int move_forward(float gridsquares, float gridbox_size);

  int turn(int degrees = 90);

  void follow_path(std::vector<std::vector<glm::vec3>> &nav_verts,
                   GridGraph &navgraph);

  void lidar_scan_postprocess(const PointCloud &pc,
                              std::vector<std::vector<glm::vec3>> &nav_verts,
                              GridGraph &navgraph);
  void lidar_scan_ex(PointCloud &pc_new, int servo_step, int step_incr,
                     int step_max, const glm::vec2 &offset = {0, 0});
};

void wifi_car_test(SOCKET &sockfd, float t = 1);

}; // namespace xn