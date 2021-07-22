#pragma once
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <algorithm>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// #include <unistd.h>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

// #include "opengl/xn_gl.hpp"
#include "xn_vec.hpp"

namespace xn {
template <class T> using std_vec2d = std::vector<std::vector<T>>;

class BezierCurve {
public:
  vec2 points[4];

  BezierCurve() {
    points[0] = {0, 0};
    points[1] = {0.5, 0};
    points[2] = {0.5, 1};
    points[3] = {1, 1};
  }

  BezierCurve(vec2 points[]) {
    for (int i = 0; i < 4; i++)
      this->points[i] = points[i];
  }

  inline vec2 solve(float t) {
    return points[0] * (1 - t) * (1 - t) * (1 - t) +
           points[1] * (1 - t) * (1 - t) * t * 3 +
           points[2] * (1 - t) * t * t * 3 + points[3] * t * t * t;
  }
};

struct Eigen {
  float val;
  glm::vec3 vec;
};

struct OBB {
  glm::vec3 pos;
  glm::vec3 ext[3];
};

struct BoundedPoints {
  OBB box;
  std::vector<glm::vec3> points;
};

struct ExtremeInfo {
  int idx_min;
  int idx_max;
  float proj_min;
  float proj_max;
};

struct VectorSortInfo {
  glm::vec3 vec;
  float dot;
};

struct Plane {
  float d; // distance along norm from origin
  glm::vec3 norm;
};

template <typename numtype>
numtype clamp(numtype n, numtype minval, numtype maxval) {
  return MAX(minval, MIN(n, maxval));
}

// print glm types w/ iostream
std::ostream &operator<<(std::ostream &os, const glm::vec2 &r);

std::ostream &operator<<(std::ostream &os, const glm::vec3 &r);

std::ostream &operator<<(std::ostream &os, const glm::mat3 &r);

float expected_val(float v[], int num_points, float p[] = NULL);

float stdev_3d(const std::vector<glm::vec3> &samples);

float covariance(float v1[], float v2[], int num_vars);

float **cov_mat(float **df, int rows, int cols,
                bool fill_redundant_values = true);

float trace(float **a, int n);

float trace(glm::mat3 a);

glm::mat3 raw_mat_to_glm3x3(float **a);

// get eigenvalues for a symmetric, real, 3x3 matrix. 1 of the values may be
// repeated
glm::vec3 eigenval_3x3(glm::mat3 a);

glm::vec3 find_eigvec(glm::mat3 &eigmat, bool print_info = false);

std::vector<Eigen> eigenvec_3x3(float **a);

template <class T>
void extreme_pts_in_dir(const T &dir, const T pt[], int n, ExtremeInfo &out,
                        float threshold = 10);

std::vector<glm::vec3> spatial_hash(glm::vec3 pt[], int n, int x, int y, int z,
                                    float cube_size);

bool eigenComp(Eigen a, Eigen b);

OBB pca_obb(std::vector<glm::vec3> pt, bool print_info = false);

// gets closest points c1 & c2 of S1(s) = P1 + s*(Q1-P1) and S2(t) = P2 +
// t*(Q2-P2), returning s & t. function result is squared distance between S1(s)
// and S2(t)
float closest_pt_segment_segment(glm::vec3 p1, glm::vec3 q1, glm::vec3 p2,
                                 glm::vec3 q2, float &s, float &t,
                                 glm::vec3 &c1, glm::vec3 &c2);

void project_points(const std::vector<glm::vec3> &points_in,
                    std::vector<glm::vec3> &points_out, glm::vec3 plane_p,
                    glm::vec3 plane_norm);

bool dotComp(VectorSortInfo a, VectorSortInfo b);

void sort_rays_by_angle(const std::vector<glm::vec3> &in_vec,
                        std::vector<glm::vec3> &out_vec,
                        glm::vec3 normal = glm::vec3(0, 1, 0));

void scan_split_lines(const std_vec2d<glm::vec3> &samples,
                      std_vec2d<glm::vec3> &samples_out);

int test_separating_axis(const std::vector<glm::vec2> &a,
                         const std::vector<glm::vec2> &b,
                         const glm::vec2 &axis);

int collision_convex_polygons(const std::vector<glm::vec2> &a,
                              const std::vector<glm::vec2> &b);

int collision_segment_plane(const glm::vec3 &a, const glm::vec3 &b,
                            const Plane &p, float &t, glm::vec3 &q);
} // namespace xn