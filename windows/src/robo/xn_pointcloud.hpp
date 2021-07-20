#pragma once
#include "../util/xn_math.hpp"
#include <array>
#include <fstream>
#include <utility>

#include <glm/gtx/norm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <numeric>
namespace xn {

class PointCloud {
public:
  static std::vector<PointCloud> all;
  glm::vec3 origin;
  std::vector<glm::vec3> points;

  PointCloud::PointCloud(
      glm::vec3 origin = glm::vec3(0),
      const std::vector<glm::vec3> &points = std::vector<glm::vec3>())
      : origin(origin), points(points) {}

  size_t appendToFile(std::ofstream &save_file, bool verbose = false) const;

  void toNavmesh(std::vector<glm::vec3> &out_tris,
                 glm::vec3 normal = glm::vec3(0, 1, 0)) const;

  static void save_pointclouds(std::string filename,
                               const std::vector<PointCloud> &pcs = all);

  static void read_pointclouds(const std::string filename,
                               std::vector<PointCloud> &pc_out);

  static void combine_points(const std::vector<PointCloud> &inp,
                             std::vector<glm::vec3> &out);

  static void join(const PointCloud &a, const PointCloud &b, PointCloud &out);

  static void hillclimb_transform(const PointCloud &a, PointCloud &b,
                                  glm::vec3 &t, float &angle,
                                  float ang_step = 0.01);

  static void gridgraph_to_verts(const std_vec2d<bool> &gridgraph,
                                 std::vector<glm::vec3> &out_verts,
                                 float gridbox_size,
                                 const glm::vec3 &grid_offset);

  static std::vector<glm::vec3>
  get_all_points(const std::vector<PointCloud> &pcs, glm::vec3 &offset_out,
                 const float distance_cutoff = 10);

  static std::vector<glm::vec3>
  march_squares2(const std::vector<PointCloud> &pcs, const int w, const int h,
                 const float s, const glm::vec2 &car_position,
                 std_vec2d<bool> &grid_out, glm::vec3 &offset_out,
                 const float distance_cutoff = 10,
                 unsigned points_per_square_threshold = 1);

  static void march_squares(const std::vector<PointCloud> &pcs, const int w,
                            const int h, const float s,
                            const glm::vec2 &car_position,
                            std_vec2d<bool> &grid_out, glm::vec3 &offset_out,
                            const float distance_cutoff = 10,
                            unsigned points_per_square_threshold = 1);
};

} // namespace xn