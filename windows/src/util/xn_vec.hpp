#pragma once
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>
#include <iostream>
// #include <pthread.h>
#include <stdio.h>

#include <windows.h> /* WinAPI */

#ifdef PIO_VIRTUAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#endif

// #include "xn_gpio.hpp"

// #include <algorithm>
#include <chrono>
#include <thread>

#define SMEPSILON 1E-16f

#define DUMP(a)                                                                \
  { std::cout << #a " = " << (a) << std::endl; }

namespace xn {
#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

//  {
//   return MAX(minval, MIN(n, maxval));
// }

namespace pio {
typedef std::chrono::high_resolution_clock::time_point TimePoint;

TimePoint get_time();
//  { return std::chrono::high_resolution_clock::now(); }

double time_diff_seconds(const TimePoint &t1, const TimePoint &t2);
//  {
//   return std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
//       .count();
// }
} // namespace pio

// #ifdef PIO_VIRTUAL

// may only be up to ~0.01 seconds of precision on windows!
void nanosleep(uint64_t ns);
//  {
//   std::this_thread::sleep_for(std::chrono::nanoseconds(ns));
// }

// windows sleep is flaky, so return the time slept
double time_sleep(double seconds);
//  {
//   if (seconds > 0.0) {
//     auto s = pio::get_time();
//     nanosleep((uint64_t)(seconds * 1e9));
//     return pio::time_diff_seconds(s, pio::get_time());
//   }
//   return 0;
// }
// #endif
// backup if glm::vec3 is not available
struct vec3 {
  float x;
  float y;
  float z;

  static float dist_sqr(const vec3 &l, const vec3 &r);
  // {
  //   float x = l.x - r.x;
  //   float y = l.y - r.y;
  //   float z = l.z - r.z;
  //   // printf("d=(%f, %f, %f)\n", x,y,z);
  //   return x * x + y * y + z * z;
  // }

  static float dist(const vec3 &l, const vec3 &r);
  // { return sqrt(dist_sqr(l, r)); }

  static float dot(const vec3 &l, const vec3 &r);
  // { return l.x * r.x + l.y * r.y + l.z * r.z; }

  static vec3 cross(const vec3 &l, const vec3 &r);
  // {
  //   return {l.y * r.z - l.z * r.y, l.z * r.x - l.x * r.z,
  //           l.x * r.y - l.y * r.x};
  // }

  static vec3 rotate_axis(vec3 point, vec3 axis, float angle);
  // {
  //   float cos_ang = cos(angle);
  //   float sin_ang = sin(angle);

  //   return point * cos_ang + cross(axis, point) * sin_ang +
  //          axis * dot(axis, point) * (1 - cos_ang);
  // }

  float mag();
  // { return dist(*this, {0, 0, 0}); }

  float mag_sqr();
  // { return dist_sqr(*this, {0, 0, 0}); }

  void normalize();
  // { *this = *this / mag(); }

  vec3 abs();
  // { return {fabsf(x), fabsf(y), fabsf(z)}; }

  vec3 operator+(const vec3 &r);
  // { return {this->x + r.x, this->y + r.y, this->z + r.z}; }

  vec3 operator-(const vec3 &r);
  // { return {this->x - r.x, this->y - r.y, this->z - r.z}; }

  vec3 operator*(const vec3 &r);
  // { return {this->x * r.x, this->y * r.y, this->z * r.z}; }

  vec3 operator*(const float &r);
  // { return {this->x * r, this->y * r, this->z * r}; }

  vec3 operator/(vec3 &r);
  // {
  //   if (fabs(r.x) < SMEPSILON || fabs(r.y) < SMEPSILON ||
  //       fabs(r.z) < SMEPSILON) {
  //     printf("attempted divide by zero!\n");
  //     std::cout << toString() << " / " << r.toString() << '\n';
  //     return *this;
  //   }

  //   return {this->x / r.x, this->y / r.y, this->z / r.z};
  // }

  vec3 operator/(const float &r);
  // {
  //   // printf("%f\n", fabs(r));
  //   if (fabs(r) < SMEPSILON) {
  //     printf("attempted divide by zero!\n");
  //     std::cout << toString() << " / " << r << '\n';
  //     // return *this;
  //   }

  //   float inv = 1 / r;
  //   return {this->x * inv, this->y * inv, this->z * inv};
  // }

  void print();
  // { printf("(%f, %f, %f)\n", x, y, z); }

  std::string toString();
  // {
  //   char buf[256];
  //   sprintf_s(buf, "(%f, %f, %f)", x, y, z);
  //   return std::string(buf);
  // }

#ifdef PIO_VIRTUAL
  glm::vec3 toGLM() { return glm::vec3(x, y, z); }
#endif
};
std::ostream &operator<<(std::ostream &os, const vec3 &r);
//  {
//   return os << "(" << r.x << ", " << r.y << ", " << r.z << ")";
// }

struct vec2 {
  float x;
  float y;

  static inline float dist_sqr(const vec2 &l, const vec2 &r) {
    float x = l.x - r.x;
    float y = l.y - r.y;
    // printf("d=(%f, %f, %f)\n", x,y,z);
    return x * x + y * y;
  }

  static inline float dist(const vec2 &l, const vec2 &r) {
    return sqrt(dist_sqr(l, r));
  }

  static inline float dot(const vec2 &l, const vec2 &r) {
    return l.x * r.x + l.y * r.y;
  }

  inline float mag() { return dist(*this, {0, 0}); }

  inline float mag_sqr() { return dist_sqr(*this, {0, 0}); }

  inline void normalize() { *this = *this / mag(); }

  inline vec2 abs() { return {fabsf(x), fabsf(y)}; }

  inline vec2 operator+(const vec2 &r) {
    return {this->x + r.x, this->y + r.y};
  }

  inline vec2 operator-(const vec2 &r) {
    return {this->x - r.x, this->y - r.y};
  }

  inline vec2 operator*(const vec2 &r) {
    return {this->x * r.x, this->y * r.y};
  }

  inline vec2 operator*(const float &r) { return {this->x * r, this->y * r}; }

  inline vec2 operator/(vec2 &r) {
    if (fabs(r.x) < SMEPSILON || fabs(r.y) < SMEPSILON) {
      printf("attempted divide by zero!\n");
      std::cout << toString() << " / " << r.toString() << '\n';
      return *this;
    }

    return {this->x / r.x, this->y / r.y};
  }

  inline vec2 operator/(const float &r) {
    // printf("%f\n", fabs(r));
    if (fabs(r) < SMEPSILON) {
      printf("attempted divide by zero!\n");
      std::cout << toString() << " / " << r << '\n';
      // return *this;
    }

    float inv = 1 / r;
    return {this->x * inv, this->y * inv};
  }

  void print() { printf("(%f, %f)\n", x, y); }

  std::string toString() {
    char buf[256];
    sprintf_s(buf, "(%f, %f)", x, y);
    return std::string(buf);
  }
};

std::ostream &operator<<(std::ostream &os, const vec2 &r);
//  {
//   return os << "(" << r.x << ", " << r.y << ")";
// }
} // namespace xn