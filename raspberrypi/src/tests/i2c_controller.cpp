/**
 * raspi i2c controller master read from esp peripheral slave
 *
 *   Bytes are sometimes lost/garbled in the transfer, so 1 padding byte is added at the
 *   beginning and end of each message as means for a validity check.
 *
 *   If a packet is invalid, read 1 byte at a time until the padding bytes are in the right
 *   position
 * */

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include <fstream>
#include <iostream>
#include <string>
#include <thread>

#include "XNlib/xn_gpio.hpp"
#include "XNlib/xn_vec.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <string.h>
#include <vector>

using namespace xn;
using namespace pio;

int PigDaemon::pi;

bool run = 1;
float rest_volts = 1.56;
float max_diff = 0.044;
int pin_servo = 5;
std::vector<float> volt_buf;
std::mutex volt_buf_lock;
int max_len = 10;

std::vector<std::thread> threads;

struct ClawController {
    // current sensor for claw servo
    std::vector<float> volt_buf;
    unsigned max_buf_len = 10;
    float rest_volts = 1.56;
    float max_diff = 0.044;
    std::mutex volt_buf_lock;
    unsigned servo;

    ClawController(){};
    ClawController(unsigned servo) { this->servo = servo; }

    static void i2c_read_current_sensor_thread(ClawController *c, unsigned i2c_bus = 0,
                                               unsigned i2c_address = 4) {
        int h = PigDaemon::i2cOpen(i2c_bus, i2c_address); // bus 1 => SDA = GPIO2, SCL = GPIO3

        while (run) {
            float v = 0;
            if (i2c_read_frame(h, (char *)&v, sizeof(v)) < 0)
                continue;

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
};

void stop(int signum) { run = 0; }

void servo_ctl_thread() {
    set_mode(PigDaemon::pi, pin_servo, PI_OUTPUT);
    int rest_width = 2500;
    int cur_width = 2000;
    int max_w_diff = 500;
    int cur_step = 5;

    while (run) {
        bool closeclaw;
        std::cout << "0 - open\n1 - close\n choice:";
        std::cin >> closeclaw;
        std::cout << '\n';
        if (closeclaw) {
            while (fabsf(volt_buf.back() - rest_volts) < max_diff && cur_width < rest_width &&
                   run) {
                cur_width += cur_step;
                cur_width = clamp(cur_width, rest_width - max_w_diff, rest_width);
                PigDaemon::servo(pin_servo, cur_width);
                time_sleep(1.0 / 20);
            }
            std::cout << "closed at w=" << cur_width << '\n';
        } else {
            cur_width = rest_width - max_w_diff;
            PigDaemon::servo(pin_servo, cur_width);
        }
    }
}

void i2c_read_current_sensor_thread() {
    int h = PigDaemon::i2cOpen(1, 4); // bus 1 => SDA = GPIO2, SCL = GPIO3

    while (run) {
        float v = 0;
        if (i2c_read_frame(h, (char *)&v, sizeof(v)) < 0)
            continue;
        DUMP(v);
        if (volt_buf.size() < max_len) {
            volt_buf_lock.lock();
            volt_buf.push_back(v);
            volt_buf_lock.unlock();
        } else {
            volt_buf_lock.lock();
            std::rotate(volt_buf.begin(), volt_buf.begin() + 1, volt_buf.end());
            volt_buf.back() = v;
            volt_buf_lock.unlock();
        }
    }

    PigDaemon::i2cClose(h);
}

int main(int argc, char *argv[]) {
    signal(SIGINT, stop);
    PigDaemon::init();

    // serial_tid = std::thread(serial_read_thead);
    threads.push_back(std::thread(servo_ctl_thread));
    threads.push_back(std::thread(i2c_read_current_sensor_thread));

    for (auto &t : threads) {
        t.join();
    }
    return 0;
}