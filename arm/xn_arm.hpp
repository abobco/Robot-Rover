#pragma once
#ifdef _WIN32
#include "../windows/src/robo/xn_net.hpp"
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
};

struct RobotArm {
  SOCKET raspi;
  const double base_len = 170;
  double wristlen = 150;
  vec3 target{0, 0, 0};
  pio::SmoothServo *wrist;
  ik::ServoChain joints;
  ClawController claw_ctl;
  std::mutex mut;
  std::thread updateThread;
  std::vector<xn::pio::SmoothServo> servos[4];

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