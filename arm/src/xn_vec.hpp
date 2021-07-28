#pragma once
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>
#include <iostream>
// #include <pthread.h>
#include <stdio.h>

// #ifdef _WIN32
// #include <windows.h> /* WinAPI */
// #endif

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

#define DUMP(a)                                                                                    \
    { std::cout << #a " = " << (a) << std::endl; }

namespace xn{

namespace pio{
typedef std::chrono::high_resolution_clock::time_point TimePoint;

TimePoint get_time();

double time_diff_seconds(const TimePoint& t1, const TimePoint& t2);
} // namespace pio


#ifdef _WIN32
void nanosleep(uint64_t ns); // may only be up to ~0.01 seconds of precision on windows!

double time_sleep(double seconds); // windows sleep is flaky, so return the time slept

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif
template <typename T> T clamp(T n, T minval, T maxval){
	return MAX(minval, MIN(n, maxval));
}

template <typename T> T clamp_vec2(T vec, const T& minvec, const T& maxvec){
	vec.x = clamp(vec.x, minvec.x, maxvec.x);
	vec.y = clamp(vec.y, minvec.y, maxvec.y);
	return vec;
}
template <typename T> T clamp_vec3(T vec, const T& minvec, const T& maxvec){
	vec.x = clamp(vec.x, minvec.x, maxvec.x);
	vec.y = clamp(vec.y, minvec.y, maxvec.y);
	vec.z = clamp(vec.z, minvec.z, maxvec.z);
	return vec;
}
#endif

// backup if glm::vec3 is not available
struct vec3{
	float x;
	float y;
	float z;

	static float dist_sqr(const vec3& l, const vec3& r);

	static float dist(const vec3& l, const vec3& r);

	static float dot(const vec3& l, const vec3& r);

	static vec3 cross(const vec3& l, const vec3& r);

	static vec3 rotate_axis(vec3 point, vec3 axis, float angle);

	float mag();

	float mag_sqr();

	void normalize();

	vec3 abs();

	vec3 operator+(const vec3& r);

	vec3 operator-(const vec3& r);

	vec3 operator*(const vec3& r);

	vec3 operator*(const float& r);

	vec3 operator/(vec3& r);

	vec3 operator/(const float& r);

	void print();

	std::string toString();

#ifdef PIO_VIRTUAL
	glm::vec3 toGLM(){ return glm::vec3(x, y, z); }
#endif
};
std::ostream& operator<<(std::ostream& os, const vec3& r);

struct vec2{
	float x;
	float y;

	static inline float dist_sqr(const vec2& l, const vec2& r){
		float x = l.x - r.x;
		float y = l.y - r.y;
		// printf("d=(%f, %f, %f)\n", x,y,z);
		return x * x + y * y;
	}

	static inline float dist(const vec2& l, const vec2& r){ return sqrt(dist_sqr(l, r)); }

	static inline float dot(const vec2& l, const vec2& r){ return l.x * r.x + l.y * r.y; }

	inline float mag(){ return dist(*this, {0, 0}); }

	inline float mag_sqr(){ return dist_sqr(*this, {0, 0}); }

	inline void normalize(){ *this = *this / mag(); }

	inline vec2 abs(){ return {fabsf(x), fabsf(y)}; }

	inline vec2 operator+(const vec2& r){ return {this->x + r.x, this->y + r.y}; }

	inline vec2 operator-(const vec2& r){ return {this->x - r.x, this->y - r.y}; }

	inline vec2 operator*(const vec2& r){ return {this->x * r.x, this->y * r.y}; }

	inline vec2 operator*(const float& r){ return {this->x * r, this->y * r}; }

	inline vec2 operator/(vec2& r){
		if (fabs(r.x) < SMEPSILON || fabs(r.y) < SMEPSILON){
			printf("attempted divide by zero!\n");
			std::cout << toString() << " / " << r.toString() << '\n';
			return *this;
		}

		return {this->x / r.x, this->y / r.y};
	}

	inline vec2 operator/(const float& r){
		// printf("%f\n", fabs(r));
		if (fabs(r) < SMEPSILON){
			printf("attempted divide by zero!\n");
			std::cout << toString() << " / " << r << '\n';
			// return *this;
		}

		float inv = 1 / r;
		return {this->x * inv, this->y * inv};
	}

	void print(){ printf("(%f, %f)\n", x, y); }

	std::string toString(){
		char buf[256];
#ifdef _WIN32
		sprintf_s(buf, "(%f, %f)", x, y);
#else
		snprintf(buf, 256, "(%f, %f)", x, y);
#endif
		return std::string(buf);
	}
};

std::ostream& operator<<(std::ostream& os, const vec2& r);
//  {
//   return os << "(" << r.x << ", " << r.y << ")";
// }
} // namespace xn