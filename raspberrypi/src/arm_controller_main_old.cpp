#include "XNlib/xn_net.hpp"
#include <arm/xn_arm.hpp>

#include "raspicam_cv.h"
#include <opencv2/opencv.hpp>

#include <mutex>
#include <thread>

using namespace xn;
using namespace pio;
bool run = true;
ClawController claw_ctl;
std::mutex volt_buf_lock;

// int PigDaemon::pi = 0;

static const int JPEG_QUALITY = 100; // default(95) 0-100

bool start_read_data = false;
bool receiving_commands = true;
bool move_to_target = false;
int cam_sock = 0, arm_ctl_sock = 0, esp_ser_sock = 0;
std::vector<std::thread> threads;
std::mutex frameread_lock, arm_mut;

pio::ServoAngular servo_cam_x(23, 500, 2500);
pio::ServoAngular servo_cam_y(24, 500, 2500);
const vec2 servo_cam_origin{1675, 853};

ik::ServoChain *armptr;
pio::SmoothServo *clawptr, *wristptr;
const float w_to_ang = M_PI / (2500 - 500);
const float claw_open_a = (1500) * w_to_ang;
const float claw_close_a = (2000) * w_to_ang;
vec3 target;
double wristlen = 150;

void INThandler(int sig) {
  signal(sig, SIG_IGN);
  receiving_commands = false;
}

void ABRThandler(int sig) {
  signal(sig, SIG_IGN);
  receiving_commands = false;
  time_sleep(0.1);
  wristptr->target_angle = M_PI_2;
  armptr->curl();
  time_sleep(5);
  for (auto &t : threads) {
    t.join();
  }

  PigDaemon::stop();
}

void cam_io_thread() {
  raspicam::RaspiCam_Cv Camera;
  cv::Mat image;

  // set camera params
  Camera.set(cv::CAP_PROP_FORMAT, CV_8UC3);
  Camera.set(cv::CAP_PROP_FPS, 16);
  Camera.set(cv::CAP_PROP_FRAME_WIDTH, 320 * 2);
  Camera.set(cv::CAP_PROP_FRAME_HEIGHT, 240 * 2);

  // Open cameraT
  if (!Camera.open()) {
    std::cerr << "Error opening the camera" << std::endl;
    return;
  }

  std::vector<int> param(2);
  param[0] = cv::IMWRITE_JPEG_QUALITY;
  param[1] = JPEG_QUALITY;

  while (receiving_commands) {
    Camera.grab();
    Camera.retrieve(image);

    std::vector<uchar> buff; // buffer for coding

    cv::imencode(".jpg", image, buff, param);

    int64_t size = buff.size();
    int64_t tmp = htonl(size);
    send_buf(cam_sock, (char *)&tmp, sizeof(tmp));
    DUMP(tmp);

    send_buf(cam_sock, (char *)&buff.front(), size);
  }

  std::cout << "Stop camera..." << std::endl;
  Camera.release();
}

void i2c_read_current_sensor_thread(ClawController *c) {
  int h = PigDaemon::i2cOpen(1, 4); // bus 1 => SDA = GPIO2, SCL = GPIO3

  while (receiving_commands) {
    float v = 0;
    if (i2c_read_frame(h, (char *)&v, sizeof(v)) < 0) {
      // time_sleep(5);
      continue;
    }
    // DUMP(v);
    if (c->volt_buf.size() < c->max_buf_len) {
      volt_buf_lock.lock();
      c->volt_buf.push_back(v);
      volt_buf_lock.unlock();
    } else {
      volt_buf_lock.lock();
      std::rotate(c->volt_buf.begin(), c->volt_buf.begin() + 1,
                  c->volt_buf.end());
      c->volt_buf.back() = v;
      volt_buf_lock.unlock();
    }
    // time_sleep(5);
  }

  PigDaemon::i2cClose(h);
}

// enum ClawTarget { CLAW_OPEN, CLAW_CLOSE };
void set_claw_target(ClawController &c, ClawTarget claw_target) {
  // set_mode(PigDaemon::pi, c.servo, PI_OUTPUT);
  int rest_width = 2500;
  int cur_width = 2000;
  int max_w_diff = 500;
  int cur_step = 5;

  c.rest_volts = c.volt_buf.back();
  switch (claw_target) {
  case CLAW_CLOSE: {
    if (c.volt_buf.size() == 0)
      break;
    while (fabsf(c.volt_buf.back() - c.rest_volts) < c.max_diff &&
           cur_width < rest_width && run) {
      cur_width += cur_step;
      cur_width = clamp(cur_width, rest_width - max_w_diff, rest_width);
      PigDaemon::servo(c.servo, cur_width);
      time_sleep(1.0 / 20);
      DUMP(c.volt_buf.back());
    }
    std::cout << "closed at w=" << cur_width << '\n';
  } break;
  case CLAW_OPEN: {
    cur_width = rest_width - max_w_diff;
    PigDaemon::servo(c.servo, cur_width);
  } break;
  }
}

void esp_serial_thread() {
  int h = PigDaemon::serialOpen("/dev/serial0", 115200);

  while (receiving_commands) {
    int n = PigDaemon::serialDataAvailable(h);

    if (n > 0) {
      char *buf = new char[n];
      PigDaemon::serialRead(h, buf, n);
      send(esp_ser_sock, buf, n, 0);
      delete[] buf;
    } else {
      time_sleep(0.1);
    }
  }
}

static void read_vec3(int sock, vec3 &v) {
  int32_t x, y, z;
  read_int<int32_t>(sock, x);
  read_int<int32_t>(sock, y);
  read_int<int32_t>(sock, z);
  v = {(float)x, (float)y, (float)z};
}

void arm_update() { armptr->grab_safe(wristlen, target); }
void wrist_update() {
  arm_mut.lock();
  armptr->move_wrist(*wristptr);
  arm_mut.unlock();
}

void arm_wait_action_complete(double timeout = 10) {
  const float MAX_DISTANCE = 0.01;
  const double wait_interval = 0.1;
  auto t1 = pio::get_time();
  time_sleep(0.25);
  while (pio::time_diff_seconds(t1, pio::get_time()) < timeout) {
    bool finished = true;
    for (auto i = 0; i < armptr->ik_chain.bone_count; i++) {
      for (pio::SmoothServo &s : armptr->servos[i]) {
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

void arm_ctl_thread() {
  bool first = true;
  while (receiving_commands) {
    vec3 v_target;
    read_vec3(arm_ctl_sock, v_target);

    // if (first) {
    //     // set_claw_target(claw_ctl, CLAW_OPEN);
    //     // armptr->straighten();
    //     // arm_wait_action_complete();
    //     // first = false;
    // }

    // prep grab
    set_claw_target(claw_ctl, CLAW_OPEN);
    armptr->straighten();
    armptr->ik_chain.print();
    armptr->ik_chain.reset();
    arm_wait_action_complete();
    armptr->reset();
    // for (unsigned i = 0; i < armptr->ik_chain.bone_count; i++) {
    //     armptr->positions[i] = armptr->ik_chain.positions[i];
    // }

    // align arm to target
    // offset = {60, 0, -22}
    target = {-v_target.x, -8, -v_target.y};
    // DUMP(target);
    wristlen = 30;
    // move_to_target = true;
    arm_mut.lock();
    arm_update();
    arm_update();
    arm_mut.unlock();
    arm_wait_action_complete();

    // move to target
    wristptr->t_max = armptr->servos[0][1].t_max * 1.5;
    wristlen = -50;
    arm_mut.lock();
    arm_update();
    arm_update();
    arm_mut.unlock();
    arm_wait_action_complete();

    // grab
    set_claw_target(claw_ctl, CLAW_CLOSE);

    // move to neutral position
    move_to_target = false;
    wristptr->t_max = armptr->servos[0][1].t_max / 2;
    armptr->curl();
    arm_wait_action_complete();

    armptr->straighten();
    arm_wait_action_complete();
    // amrptr->update_positions();

    // release
    set_claw_target(claw_ctl, CLAW_OPEN);

    armptr->reset();
    arm_wait_action_complete();
  }
}

void arm_update_thread() {
  while (receiving_commands) {
    // if (move_to_target) {
    //     armptr->grab_safe(wristlen, target);
    // }
    // armptr->move_wrist(*wristptr);
    wrist_update();
  }
}

int main(int argc, char *argv[]) {
  PigDaemon::init();
  signal(SIGINT, INThandler);
  signal(SIGABRT, ABRThandler);

  servo_cam_x.setWidth(1500);
  servo_cam_y.setWidth(1500);

  printf("%d\n", PORT_RPI_CAM);
  printf("%s\n", LAPTOP_IP_STRING);

  cam_sock = connect_socket_blocking(PORT_RPI_CAM);
  arm_ctl_sock = connect_socket_blocking(PORT_RPI_ARM);
  esp_ser_sock = connect_socket_blocking(PORT_ESP_LOG);

  vec3 pole = {0, 0, 1};
  vec3 bonechain[] = {vec3{0, 0, 0}, vec3{0, 1, 0}, vec3{0, 1 + 6.9 / 17.2, 0},
                      vec3{0, 1 + 13.8 / 17.2, 0}};

  ik::IkChain arm(4, bonechain, pole);
  std::vector<pio::SmoothServo> servos[] = {
      std::vector<pio::SmoothServo>(), std::vector<pio::SmoothServo>(),
      std::vector<pio::SmoothServo>(), std::vector<pio::SmoothServo>()};
  servos[0].push_back(pio::SmoothServo(pio::ServoAngular(12, 500, 2500),
                                       {0, 1, 0}, bonechain[0], 1450));
  servos[0].push_back(pio::SmoothServo(pio::ServoAngular(13, 500, 2500),
                                       {1, 0, 0}, bonechain[0], 1600));
  servos[1].push_back(pio::SmoothServo(pio::ServoAngular(26, 500, 2500),
                                       {1, 0, 0}, bonechain[1], 1600));
  servos[2].push_back(pio::SmoothServo(pio::ServoAngular(16, 500, 2500),
                                       {-1, 0, 0}, bonechain[2], 1500));

  pio::SmoothServo wrist(pio::ServoAngular(6, 500, 2500));
  wrist.servo.setAngle(M_PI_2);
  wrist.t_max *= 0.5;
  wrist.target_angle = 0;
  // pthread_create(&wrist.tid, NULL, ik::move_servo_thread, &wrist);
  wristptr = &wrist;

  pio::SmoothServo claw(pio::ServoAngular(5, 500, 2500));
  claw.servo.setAngle(claw_open_a);
  claw.target_angle = claw_open_a;
  // pthread_create(&claw.tid, NULL, ik::move_servo_thread, &claw);
  clawptr = &claw;

  ik::ServoChain phys_arm(arm, servos, &run, 172);
  phys_arm.servos[0][0].t_max *= 0.5;
  armptr = &phys_arm;

  phys_arm.servos[0][1].servo.setAngle(M_PI / 2);
  phys_arm.servos[1][0].servo.setAngle(0.1);
  phys_arm.servos[1][0].target_angle = 0;
  phys_arm.servos[2][0].servo.setAngle(M_PI);
  phys_arm.servos[2][0].target_angle = M_PI;

  // arm_wait_action_complete();
  // phys_arm.saveResetPosition();

  phys_arm.create_threads(threads);
  threads.push_back(std::thread(ik::move_servo_thread, &wrist));
  set_mode(PigDaemon::pi, claw_ctl.servo, PI_OUTPUT);
  start_read_data = 1;

  threads.push_back(std::thread(cam_io_thread));
  threads.push_back(std::thread(arm_ctl_thread));
  threads.push_back(std::thread(arm_update_thread));
  threads.push_back(std::thread(esp_serial_thread));
  // threads.push_back(std::thread(esp_i2c_thread));
  threads.push_back(std::thread(i2c_read_current_sensor_thread, &claw_ctl));

  while (receiving_commands) {
    int32_t pin, value;
    read_int(cam_sock, pin, true, 0);
    read_int(cam_sock, value, true, 0);

    PigDaemon::servo(pin, value);
  }

  // pthread_join(cam_io_tid, NULL);
}