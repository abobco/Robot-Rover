#pragma once
#include "../robo/xn_car_virtual.hpp"
#include "../robo/xn_homo.hpp"
#include "../robo/xn_ik.hpp"
#include "../robo/xn_search.hpp"
#include "../robo/xn_yolo.hpp"
#include "app_state.hpp"
#include <opencv2/opencv.hpp>
#include <util/xn_json.hpp>

namespace xn {

typedef uint32_t CamPixel;
static const size_t MAX_CAM_JPEG_SIZE = 640 * 480 * sizeof(CamPixel);
struct Jpeg {
  uint8_t buffer[MAX_CAM_JPEG_SIZE];
  size_t size = 0;
};

struct RobotWorldInfo {
  // static const glm::uvec2 cam_pic_dimensions(640, 480);

  const double arm_base_len = 170;
  double wristlen = 150;
  float rotation = 0; // CCW rotation around +y axis
  vec3 arm_target{0, 0, 0};
  glm::vec2 position{0};
  glm::vec3 target_world;
  glm::ivec2 scan_range{100, 800};
  glm::ivec2 cam_servo_width{1573, 2056};
  pio::SmoothServo *wrist;
  // std::vector<unsigned char> cam_pic_buffer;
  // std::vector<unsigned char> cam_pic_processed_buffer;
  Jpeg cam_pic;
  Jpeg cam_pic_processed;
  // cv::Mat cam_outframe;
  std::vector<uint8_t> cam_outframe;
};

ik::ServoChain *getArm();

void car_sim_thread(GridGraph &navgraph, RobotWorldInfo &robot);

void cam_io_thread(SOCKET &pi_cam, Jpeg &cam_pic,
                   std::vector<uint8_t> &cam_outframe);

void cam_servo_ctl_thread(const json &settings, SOCKET &pi_cam,
                          RobotWorldInfo &robot);

void esp_log_thread(SOCKET &esp_log);

void arm_wait_action_complete(double timeout = 10);

void arm_update_thread(pio::SmoothServo &wrist);

void arm_update(double wristlen, vec3 &arm_target);

void ik_sim_thread(RobotWorldInfo &robot);

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

void yolo_thread(const json &jsettings, SOCKET &pi_arm, vec3 &arm_target,
                 Jpeg &cam_pic, std::vector<uint8_t> &cam_outframe);

void arm_ctl_thread(vec3 &arm_target, RobotWorldInfo &robot);

} // namespace xn