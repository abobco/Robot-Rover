#include "xn_arm.hpp"
#include "xn_gpio.hpp"
#include <windows/src/app/app_state.hpp>

namespace xn {

RobotArm::RobotArm() {
  AppState::get().arm_mut.lock();
  float arm_len = 104;
  int mode = 0;
  vec3 pole = {0, 0, 1};
  vec3 bonechain[] = {vec3{0, 0, 0}, vec3{0, 1, 0}, vec3{0, 1 + 7.5f / 18, 0},
                      vec3{0, 1 + 15.0f / 18, 0}};
  float start_bone_lengths[] = {vec3::dist(bonechain[0], bonechain[1]),
                                vec3::dist(bonechain[1], bonechain[2])};

  ik::IkChain arm(4, bonechain, pole);

  servos[0].push_back(xn::pio::SmoothServo(xn::pio::ServoAngular(12, 500, 2500),
                                           {0, -1, 0}, bonechain[0], 1500));
  servos[0].push_back(xn::pio::SmoothServo(xn::pio::ServoAngular(13, 500, 2500),
                                           {1, 0, 0}, bonechain[0], 1570));
  servos[1].push_back(xn::pio::SmoothServo(xn::pio::ServoAngular(26, 500, 2500),
                                           {1, 0, 0}, bonechain[1], 1420));
  servos[2].push_back(xn::pio::SmoothServo(xn::pio::ServoAngular(16, 500, 2500),
                                           {1, 0, 0}, bonechain[2], 1500));

  joints = ik::ServoChain(arm, servos, &AppState::get().run);
  AppState::get().arm_mut.unlock();
  joints.reset();
  wrist = new pio::SmoothServo(pio::ServoAngular(6, 500, 2500));
}

RobotArm::~RobotArm() {
  delete wrist;
  wrist = NULL;
}

void RobotArm::createThreads() {
  updateThread = std::thread(
      [](RobotArm *arm) {
        while (AppState::get().run) {
          arm->joints.move_wrist(*arm->wrist);
        }
      },
      this);
}

void RobotArm::updateJoints(double distance) {
  joints.grab_safe(distance, target);
}

void RobotArm::updateWrist() { joints.move_wrist(*wrist); }

void RobotArm::waitActionComplete(double timeout) {
  const float MAX_DISTANCE = 0.01f;
  const double wait_interval = 0.1;
  auto t1 = pio::get_time();
  time_sleep(0.5);
  while (pio::time_diff_seconds(t1, pio::get_time()) < timeout) {
    bool finished = true;
    for (auto i = 0; i < joints.ideal_chain.bone_count; i++) {
      for (pio::SmoothServo &s : joints.servos[i]) {
        if (s.servo.getWidth() != s.targetw) {
          finished = false;
          break;
        }
      }
    }
    if (finished)
      break;
    time_sleep(wait_interval);
  }
}

void RobotArm::prepGrab() {
  setClawState(CLAW_OPEN);
  joints.straighten();
  joints.ideal_chain.reset();
  waitActionComplete();
  joints.reset();
}

void RobotArm::alignToGrabTarget(double distance) {
  mut.lock();
  updateJoints(distance);
  updateJoints(distance);
  mut.unlock();
  waitActionComplete();
}

void RobotArm::setClawState(ClawTarget state) {
  // set_mode(PigDaemon::pi, c.servo, PI_OUTPUT);
  int rest_width = 2500;
  int cur_width = 2000;
  int max_w_diff = 500;
  int cur_step = 5;

  claw_ctl.rest_volts = claw_ctl.volt_buf.back();
  switch (state) {
  case CLAW_CLOSE: {
    if (claw_ctl.volt_buf.size() == 0)
      break;
    while (fabsf(claw_ctl.volt_buf.back() - claw_ctl.rest_volts) <
               claw_ctl.max_diff &&
           cur_width < rest_width) {
      cur_width += cur_step;
      cur_width = clamp(cur_width, rest_width - max_w_diff, rest_width);
      PigDaemon::servo(claw_ctl.servo, cur_width);
      time_sleep(1.0 / 20);
      DUMP(claw_ctl.volt_buf.back());
    }
    std::cout << "closed at w=" << cur_width << '\n';
  } break;
  case CLAW_OPEN: {
    cur_width = rest_width - max_w_diff;
    PigDaemon::servo(claw_ctl.servo, cur_width);
  } break;
  }
}

void RobotArm::liftObject() {
  joints.curl();
  waitActionComplete();
  joints.straighten();
  waitActionComplete();
  joints.reset();
  waitActionComplete();
}

} // namespace xn