#pragma once
#ifdef _WIN32
#include "../../windows/src/robo/xn_net.hpp"
#else
#include "../raspberrypi/src/XNlib/xn_net.hpp"

#endif
#include "xn_ik.hpp"
#include <cstdint>
#include <mutex>
#include <thread>

namespace xn {

enum ClawTarget { CLAW_OPEN, CLAW_CLOSE };

struct ClawController {
  unsigned servo = 5; // servo pin
  // current sensor for claw servo
  unsigned max_buf_len = 10;
  // float rest_volts = 1.5;
  float rest_volts = 2.0;
  float max_diff = 0.12;
  std::vector<float> volt_buf;
  std::mutex volt_buf_lock;
};

#ifndef _WIN32
typedef int SOCKET;
#endif

struct RobotArm {
  SOCKET sock;
  const double base_len = 170;
  double wristlen = 150;
  vec3 target{0, 0, 0};
  bool run = true;
  pio::SmoothServo *wrist, *claw;
  ik::ServoChain joints;
  ClawController claw_ctl;
  std::mutex mut;
  std::thread updateThread;
  std::vector<xn::pio::SmoothServo> servos[4];
  const float w_to_ang = M_PI / (2500 - 500);
  const float claw_open_a = (1500) * w_to_ang;
  const float claw_close_a = (2000) * w_to_ang;

  RobotArm();

  ~RobotArm();

  void createThreads();

  void updateJoints(double distance);

  void updateWrist();

  void waitActionComplete(double timeout = 10);

  void prepGrab();

  void alignToGrabTarget(double distance);

  void setClawState(ClawTarget state);

  void liftObject();
};

} // namespace xn