#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <pthread.h>

// #include "XNlib/xn_gpio.hpp"
#include <arm/xn_gpio.hpp>

/*
sudo ./servo_demo          # Send servo pulses to GPIO 12
sudo ./servo_demo 23 24 25 # Send servo pulses to GPIO 23, 24, 25.

HITEC HS311 servo notes:
   - takes ~0.67 seconds to move from min to max angle
   - 575-2460 ms pulse width on the data sheet

servo pins:
   12, 13
   26, 16, 6

arm servo mid widths:
   - 13 : 1460
   - 26 : 1570
   - 16 : 1420
   - 6  : 1500
*/

using namespace xn;
using namespace pio;
int PigDaemon::pi = 0;

int run = 1;

float pos = 0.5;

void stop(int signum) {
    run = 0;
    PigDaemon::stop();
    exit(0);
}

int main(int argc, char *argv[]) {
    // init(argc, argv);
    // if (gpioInitialise() < 0)
    //     return -1;
    PigDaemon::init();
    signal(SIGINT, stop);

    int led_pin = 18;
    // xn::pio::ServoAngular serv(servo_pin, 500, 2500);
    set_mode(PigDaemon::pi, led_pin, PI_OUTPUT);
    // set_PWM_frequency(PigDaemon::pi, led_pin, 50);
    gpio_write(PigDaemon::pi, led_pin, 1);

    while (run) {
        int duty = 0;
        std::cin >> duty;
        set_PWM_dutycycle(PigDaemon::pi, led_pin, duty);

        // int t = 1500;
        // float t = 0.5;
        // printf("Enter w for %d: ", servo_pin);
        // std::cin >> t;
        // serv.setWidth(t);

        // hitec_hs311.setPosition(hitec_hs311.getAngle()*0.95 + t*0.05);

        // x_step = servo_step(mg995, x_step, x_min, x_max);
        // y_step = servo_step(hitec_hs311, y_step, y_min, y_max);
    }

    printf("\nlater nerd\n");

    // hitec_hs311.setPosition(0.5);

    gpioTerminate();

    return 0;
}
