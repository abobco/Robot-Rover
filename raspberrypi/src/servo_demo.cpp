#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <pthread.h>

#include "XNlib/xn_gpio.hpp"

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
int pio::PigDaemon::pi = 0;

int run = 1;

float pos = 0.5;

void stop(int signum) {
    run = 0;
    pio::PigDaemon::stop();
    exit(0);
}

float servo_step(xn::pio::ServoAngular &servo, float step, float pos_min, float pos_max) {
    float new_pos = servo.getPosition() + step;
    if (new_pos > pos_max || new_pos < pos_min) {
        step *= -1;
        new_pos += step;
    }
    servo.setPosition(new_pos);

    return step;
}

int main(int argc, char *argv[]) {
    // init(argc, argv);
    // if (gpioInitialise() < 0)
    //     return -1;
    pio::PigDaemon::init();
    signal(SIGINT, stop);

    int servo_pin = 12;
    int mid_width = 1500;
    if (argc > 1) {
        servo_pin = atoi(argv[1]);
    }

    if (argc > 2) {
        mid_width = atoi(argv[2]);
    }

    xn::pio::ServoAngular serv(servo_pin, 500, 2500);

    while (run) {
        int t = 1500;
        // float t = 0.5;
        printf("Enter w for %d: ", servo_pin);
        std::cin >> t;
        serv.setWidth(t);

        // hitec_hs311.setPosition(hitec_hs311.getAngle()*0.95 + t*0.05);

        // x_step = servo_step(mg995, x_step, x_min, x_max);
        // y_step = servo_step(hitec_hs311, y_step, y_min, y_max);
    }

    printf("\nlater nerd\n");

    // hitec_hs311.setPosition(0.5);

    gpioTerminate();

    return 0;
}
