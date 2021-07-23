#include "XNlib/xn_net.hpp"
#include <arm/xn_arm.hpp>

#include "raspicam_cv.h"
#include <opencv2/opencv.hpp>

#include <mutex>
#include <thread>

using namespace xn;
using namespace pio;
bool run = true;

RobotArm arm;

static const int JPEG_QUALITY = 100; // default(95) 0-100

bool receiving_commands = true;
int cam_sock = 0, arm_ctl_sock = 0, esp_ser_sock = 0;
std::vector<std::thread> threads;

pio::ServoAngular servo_cam_x(23, 500, 2500);
pio::ServoAngular servo_cam_y(24, 500, 2500);
const vec2 servo_cam_origin{1675, 853};

void INThandler(int sig) {
    signal(sig, SIG_IGN);
    receiving_commands = false;
}

void ABRThandler(int sig) {
    signal(sig, SIG_IGN);
    receiving_commands = false;
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

    // Open camera
    if (!Camera.open()) {
        std::cerr << "Error opening the camera" << std::endl;
        return;
    }

    std::vector<int> param(2);
    param[0] = cv::IMWRITE_JPEG_QUALITY;
    param[1] = JPEG_QUALITY;

    std::vector<uchar> buff; // buffer for coding
    while (receiving_commands) {
        Camera.grab();
        Camera.retrieve(image);

        cv::imencode(".jpg", image, buff, param);

        int64_t size = buff.size();
        int64_t tmp = htonl(size);
        send_buf(cam_sock, (char *)&tmp, sizeof(tmp));
        // DUMP(tmp);

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
            continue;
        }
        // DUMP(v);
        if (c->volt_buf.size() < c->max_buf_len) {
            c->volt_buf_lock.lock();
            c->volt_buf.push_back(v);
            c->volt_buf_lock.unlock();
        } else {
            c->volt_buf_lock.lock();
            std::rotate(c->volt_buf.begin(), c->volt_buf.begin() + 1, c->volt_buf.end());
            c->volt_buf.back() = v;
            c->volt_buf_lock.unlock();
        }
    }

    PigDaemon::i2cClose(h);
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

void read_vec3(int sock, vec3 &v) {
    int32_t x, y, z;
    read_int<int32_t>(sock, x);
    read_int<int32_t>(sock, y);
    read_int<int32_t>(sock, z);
    v = {(float)x, (float)y, (float)z};
}

void arm_ctl_thread() {
    bool first = true;
    while (receiving_commands) {
        vec3 v_target;
        read_vec3(arm_ctl_sock, v_target);

        arm.setClawState(CLAW_OPEN);
        arm.prepGrab();

        // align arm to target
        arm.target = {-v_target.x, -8, -v_target.y};
        arm.alignToGrabTarget(30);

        // finish moving to target
        arm.wrist->t_max = arm.joints.servos[0][1].t_max * 1.5;
        arm.alignToGrabTarget(-50);

        // grab
        arm.setClawState(CLAW_CLOSE);

        // move to neutral position
        arm.wrist->t_max = arm.joints.servos[0][1].t_max * 0.5;
        arm.liftObject();
        arm.setClawState(CLAW_OPEN);
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

    arm.wrist->servo.setAngle(M_PI_2);
    arm.wrist->t_max *= 0.5;
    arm.wrist->target_angle = 0;

    arm.claw->servo.setAngle(arm.claw_open_a);
    arm.claw->target_angle = arm.claw_open_a;

    arm.joints.servos[0][0].t_max *= 0.5;
    arm.joints.servos[0][1].servo.setAngle(M_PI / 2);
    arm.joints.servos[1][0].servo.setAngle(0.1);
    arm.joints.servos[1][0].target_angle = 0;
    arm.joints.servos[2][0].servo.setAngle(M_PI);
    arm.joints.servos[2][0].target_angle = M_PI;

    arm.joints.create_threads(threads);
    threads.push_back(std::thread(ik::move_servo_thread, arm.wrist));
    set_mode(PigDaemon::pi, arm.claw_ctl.servo, PI_OUTPUT);

    threads.push_back(std::thread(cam_io_thread));
    threads.push_back(std::thread(arm_ctl_thread));
    arm.createThreads();
    threads.push_back(std::thread(esp_serial_thread));
    threads.push_back(std::thread(i2c_read_current_sensor_thread, &arm.claw_ctl));

    while (receiving_commands) {
        int32_t pin, value;
        read_int(cam_sock, pin, true, 0);
        read_int(cam_sock, value, true, 0);

        PigDaemon::servo(pin, value);
    }
}