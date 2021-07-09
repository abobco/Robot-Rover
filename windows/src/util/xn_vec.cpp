#include "xn_vec.hpp"

namespace xn {

namespace pio {
TimePoint get_time() { return std::chrono::high_resolution_clock::now(); }

double time_diff_seconds(const TimePoint &t1, const TimePoint &t2) {
  return std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
      .count();
}
} // namespace pio

float vec3::dist_sqr(const vec3 &l, const vec3 &r) {
  float x = l.x - r.x;
  float y = l.y - r.y;
  float z = l.z - r.z;
  // printf("d=(%f, %f, %f)\n", x,y,z);
  return x * x + y * y + z * z;
}

float vec3::dist(const vec3 &l, const vec3 &r) { return sqrt(dist_sqr(l, r)); }

float vec3::dot(const vec3 &l, const vec3 &r) {
  return l.x * r.x + l.y * r.y + l.z * r.z;
}

vec3 vec3::cross(const vec3 &l, const vec3 &r) {
  return {l.y * r.z - l.z * r.y, l.z * r.x - l.x * r.z, l.x * r.y - l.y * r.x};
}

vec3 vec3::rotate_axis(vec3 point, vec3 axis, float angle) {
  float cos_ang = cos(angle);
  float sin_ang = sin(angle);

  return point * cos_ang + cross(axis, point) * sin_ang +
         axis * dot(axis, point) * (1 - cos_ang);
}

float vec3::mag() { return dist(*this, {0, 0, 0}); }

float vec3::mag_sqr() { return dist_sqr(*this, {0, 0, 0}); }

void vec3::normalize() { *this = *this / mag(); }

vec3 vec3::abs() { return {fabsf(x), fabsf(y), fabsf(z)}; }

vec3 vec3::operator+(const vec3 &r) {
  return {this->x + r.x, this->y + r.y, this->z + r.z};
}

vec3 vec3::operator-(const vec3 &r) {
  return {this->x - r.x, this->y - r.y, this->z - r.z};
}

vec3 vec3::operator*(const vec3 &r) {
  return {this->x * r.x, this->y * r.y, this->z * r.z};
}

vec3 vec3::operator*(const float &r) {
  return {this->x * r, this->y * r, this->z * r};
}

vec3 vec3::operator/(vec3 &r) {
  if (fabs(r.x) < SMEPSILON || fabs(r.y) < SMEPSILON || fabs(r.z) < SMEPSILON) {
    printf("attempted divide by zero!\n");
    std::cout << toString() << " / " << r.toString() << '\n';
    return *this;
  }

  return {this->x / r.x, this->y / r.y, this->z / r.z};
}

vec3 vec3::operator/(const float &r) {
  // printf("%f\n", fabs(r));
  if (fabs(r) < SMEPSILON) {
    printf("attempted divide by zero!\n");
    std::cout << toString() << " / " << r << '\n';
    // return *this;
  }

  float inv = 1 / r;
  return {this->x * inv, this->y * inv, this->z * inv};
}

void vec3::print() { printf("(%f, %f, %f)\n", x, y, z); }

std::string vec3::toString() {
  char buf[256];
  sprintf_s(buf, "(%f, %f, %f)", x, y, z);
  return std::string(buf);
}

std::ostream &operator<<(std::ostream &os, const vec3 &r) {
  return os << "(" << r.x << ", " << r.y << ", " << r.z << ")";
}

std::ostream &operator<<(std::ostream &os, const vec2 &r) {
  return os << "(" << r.x << ", " << r.y << ")";
}

// #ifdef PIO_VIRTUAL
// may only be up to ~0.01 seconds of precision on windows!
void nanosleep(uint64_t ns) {
  std::this_thread::sleep_for(std::chrono::nanoseconds(ns));
}

// windows sleep is flaky, so return the time slept
double time_sleep(double seconds) {
  if (seconds > 0.0) {
    auto s = pio::get_time();
    nanosleep((uint64_t)(seconds * 1e9));
    return pio::time_diff_seconds(s, pio::get_time());
  }
  return 0;
}
// #endif
} // namespace xn