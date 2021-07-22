#pragma once

// #include "../windows/src/robo/xn_gpio.hpp"
#include "../windows/src/util/xn_vec.hpp"
#include <arm/xn_gpio.hpp>
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <thread>
#include <vector>

#ifdef PIO_VIRTUAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#endif

namespace xn {
namespace ik {

// smooth servo movement
void move_servo_thread(pio::SmoothServo *s);

// cyclic coordinate descent inverse kinematics solver w/ poles
class IkChain {
public:
  int bone_count;
  int iterations;       // max iterations
  float delta = 0.001f; // max distance to target for early success

  float *bone_lengths;
  float chain_length = 0;
  vec3 *bones;
  vec3 *positions;
  vec3 pole_main;
  vec3 *poles;

  IkChain() {}

  IkChain(int _bone_count, vec3 ordered_bones[], vec3 &_pole,
          int _iterations = 10);

  void resolve(vec3 &target);

  void print();

  void reset();
};

// top level arm servo controller
class ServoChain {
public:
  IkChain ideal_chain;
  std::vector<pio::SmoothServo> *servos;
  std::vector<std::vector<pio::SmoothServo>> servos_orig;

  vec3 *positions;
  vec3 *positions_orig;
  uint16_t numJoints = 4;
  float arm_len;

  ServoChain() {}

  ServoChain(IkChain bonechain, std::vector<pio::SmoothServo> _servos[],
             bool *loopvar, float _arm_len = 1);

  void create_threads(std::vector<std::thread> &threadpool);

  void resolve(vec3 &target);

  void curl();

  void straighten();

  // solve for a point on the line from arm to target, wristlen away from the
  // target
  void grab_safe(double wristlen, vec3 &target, const vec3 &arm_pos = {0, 0, 0}
                 //  const vec3 &arm_pos = {60, 0, -22}
  );

  void move_wrist(pio::SmoothServo &wrist);

  // TODO: save start values from constructor instead of this bs
  void reset();
};
} // namespace ik
} // namespace xn