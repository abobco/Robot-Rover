#pragma once
#include "../robo/xn_car_virtual.hpp"
#include "../robo/xn_homo.hpp"
#include "../robo/xn_rover.hpp"
#include "../robo/xn_yolo.hpp"
#include "app_state.hpp"
#include <opencv2/opencv.hpp>
#include <util/xn_json.hpp>
#include <xn_arm.hpp>

namespace xn {

typedef uint32_t CamPixel;

static const size_t MAX_CAM_JPEG_SIZE = 640 * 480 * sizeof(CamPixel);

struct Jpeg {
  uint8_t buffer[MAX_CAM_JPEG_SIZE];
  size_t size = 0;
};

struct ArmInfo {
  const double base_len = 170;
  double wristlen = 150;
  vec3 target{0, 0, 0};
  pio::SmoothServo *wrist;
  ik::ServoChain joints;
};

struct RobotController {
  glm::ivec2 cam_servo_width{1573, 2056};
  Jpeg cam_pic;
  Jpeg cam_pic_processed;
  std::vector<uint8_t> cam_outframe;

  // ArmInfo armInfo;
  RobotArm arm;

  Rover rover;
};

// frame queue for object detection neural net
template <typename T> class QueueFPS : public std::queue<T> {
public:
  QueueFPS() : counter(0) {}

  void push(const T &entry) {
    std::lock_guard<std::mutex> lock(mutex);

    std::queue<T>::push(entry);
    counter += 1;
    if (counter == 1) {
      // Start counting from a second frame (warmup).
      tm.reset();
      tm.start();
    }
  }

  T get() {
    std::lock_guard<std::mutex> lock(mutex);
    T entry = this->front();
    this->pop();
    return entry;
  }

  float getFPS() {
    tm.stop();
    double fps = counter / tm.getTimeSec();
    tm.start();
    return static_cast<float>(fps);
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex);
    while (!this->empty())
      this->pop();
  }

  unsigned int counter;

private:
  cv::TickMeter tm;
  std::mutex mutex;
};

// ik::ServoChain *getArm();

void car_sim_thread(GridGraph &navgraph, RobotController &robot);

void cam_io_thread(SOCKET &pi_cam, Jpeg &cam_pic,
                   std::vector<uint8_t> &cam_outframe);

void cam_servo_ctl_thread(const json &settings, SOCKET &pi_cam,
                          RobotController &robot);

void esp_log_thread(SOCKET &esp_log);

void arm_wait_action_complete(ArmInfo &arm, double timeout = 10);

// void arm_update(ArmInfo &arm);

// void arm_update_thread(ArmInfo &arm);

void ik_sim_thread(RobotArm &arm);

void rover_ctl_thread(Rover &rover, GridGraph &navgraph);

void yolo_thread(const json &jsettings, SOCKET &pi_arm, vec3 &arm_target,
                 Jpeg &cam_pic, std::vector<uint8_t> &cam_outframe);

} // namespace xn