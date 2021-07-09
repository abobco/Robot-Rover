/*
    Stepper motor/rotary encoder drivr program
    Motor gradually speeds up to 1250 rpm until the goal angle is nearby, then gradually slows down
   and stops Uses a 4 point bezier curve to decide motor speed Tested with a 17hs19-2004s1 stepper
   motor, a4988 driver, and 360 PPR incremental rotary encoder
*/

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include <fstream>
#include <iostream>
#include <pthread.h>
#include <string>

#include "XNlib/xn_gpio.hpp"
#include "XNlib/xn_vec.hpp"

using namespace xn;
int pio::PigDaemon::pi = 0;

pio::Stepper *n17_ptr;

int pin_dir = 1, pin_step = 12;
int pin_a = 7, pin_b = 8;

int motor_steps_rev = 200;

float phase_to_radians = M_PI_2 / 355;
float phase_to_degrees = 90.0 / 355;

float target_angle = 0, cur_angle = 0;

int run = 1;
int cpos, tpos = 0;

#define PIG_TX_FOR_Y(wid, n, y) 255, 0, wid, 255, 1, n, y
#define PIG_TX_FOR(wid, n) PIG_TX_FOR_Y(wid, n, 0)
// #define PIG_TX_FOR_V(n, y, ...) 255, 0, __VA_ARGS__, 255, 1, n, y

// create pulse w/ usDelay
int pulse_create(int pin, int usHigh, int numPulses = 2, int usLow = -1) {
    // gpioWaveClear();
    if (usLow == -1) {
        usLow = usHigh;
    }
    gpioPulse_t pulse[numPulses * 2];
    for (int i = 0; i < numPulses * 2; i += 2) {
        pulse[i].gpioOn = (1 << pin);
        pulse[i].gpioOff = 0;
        pulse[i].usDelay = usHigh;

        pulse[i + 1].gpioOn = 0;
        pulse[i + 1].gpioOff = (1 << pin);
        pulse[i + 1].usDelay = usLow;
    }

    gpioWaveAddNew();
    gpioWaveAddGeneric(numPulses * 2, pulse);
    return gpioWaveCreate();
}

// send 1 pulse, wait for it to finish
double wavechain_tx(int pin, int usDelay, int n = 200) {
    pio::TimePoint t_start = pio::get_time();
    int wid = pulse_create(pin, usDelay);

    char wavechain[] = {PIG_TX_FOR(wid, n)};
    gpioWaveChain(wavechain, sizeof(wavechain));
    DUMP(gpioDelay(n * usDelay * 2));

    gpioWaveDelete(wid);

    return pio::time_diff_seconds(t_start, pio::get_time());
}

int *wavechain_curve(int pin, float curveVals[], int curveLen, int usStart, int usEnd,
                     int ppf = 2) {
    gpioWaveClear();
    int *wid = new int[curveLen];
    char wavechain[curveLen + 5];

    for (int i = 0; i < curveLen; i++) {
        int us = (1 - curveVals[i]) * usStart + curveVals[i] * usEnd;
        wid[i] = pulse_create(pin, us, ppf);
        wavechain[i] = wid[i];
    }

    wavechain[curveLen] = 255;
    wavechain[curveLen + 1] = 0;
    wavechain[curveLen + 2] = wid[curveLen - 1];
    wavechain[curveLen + 3] = 255;
    wavechain[curveLen + 4] = 3;

    gpioWaveChain(wavechain, sizeof(wavechain));
    return wid;
}

int *wavechain_linear(int pin, int num_freqs, int usStart, int usEnd) {
    int *wid = new int[num_freqs];
    char wavechain[num_freqs + 5];
    uint32_t micros_tot = 0;

    for (int i = 0; i < num_freqs; i++) {
        // int us =  usMax - (float)i/num_freqs *(usMax - usMin);

        float t = (float)i / num_freqs;
        int us = (1 - t) * usStart + t * usEnd;
        wid[i] = pulse_create(pin, us);
        micros_tot += 2 * us;

        wavechain[i] = wid[i];
    }

    wavechain[num_freqs] = 255;
    wavechain[num_freqs + 1] = 0;
    wavechain[num_freqs + 2] = wid[num_freqs - 1];
    wavechain[num_freqs + 3] = 255;
    wavechain[num_freqs + 4] = 3;

    gpioWaveChain(wavechain, sizeof(wavechain));
    return wid;
}

void *wavechain_thread(void *argv) {
    pio::Stepper *n17_ptr = (pio::Stepper *)argv;
    wavechain_linear(n17_ptr->pin_step, 600, 120, 2500);
    return NULL;
}

void stop(int signum) {
    run = 0;
    std::ofstream of("rot_pos.txt");
    if (of.is_open()) {
        of << cpos;
    }
}

void cbf(int pos) {
    // DUMP(pos);
    // DUMP(tpos);
    // DUMP(pos*phase_to_radians);
    // DUMP(pos*phase_to_degrees);

    cur_angle = pos * phase_to_radians;
    cpos = pos;
    // if ( fabsf(cur_angle - target_angle) < SMEPSILON ) {
    //     n17_ptr->setDutyCycle(0);
    // }
}

void *step_thread(void *argv) {
    bool prev_moving = false;
    bool prev_slow = false;
    pio::Stepper *n17_ptr = (pio::Stepper *)argv;
    int *wid_current;
    int ramp_steps = 200;

    BezierCurve b;
    float bez_buf[ramp_steps];
    for (int i = 0; i < ramp_steps; i++) {
        bez_buf[i] = b.solve((float)i / ramp_steps).y;
    }

    while (run) {
        if (cpos == tpos) {
            if (!prev_moving)
                continue;
            gpioWaveTxStop();
            // delete[] wid_current;
            gpioWrite(n17_ptr->pin_step, 0);

            prev_slow = false;
            prev_moving = false;
            std::cout << "durr\n";
        } else if (abs(cpos - tpos) < 100 && !prev_slow && prev_moving) {
            // if ( !prev_moving )
            //     continue;
            // n17_ptr->setDutyCycle(0);
            DUMP(cur_angle);
            DUMP(target_angle);
            // gpioWaveTxStop();

            // ramp_steps = 1;
            // int *twid = wavechain_linear(n17_ptr->pin_step, ramp_steps, gpioWaveGetMicros(),
            // 1000);
            DUMP(gpioWaveGetMicros);
            int *twid = wavechain_curve(n17_ptr->pin_step, bez_buf, ramp_steps, 600, 3000, 4);

            std::cout << "error inc\n";
            for (int i = 0; i < ramp_steps; i++) {
                gpioWaveDelete(wid_current[i]);
            }
            delete[] wid_current;
            wid_current = twid;
            prev_slow = true;

        } else {
            if (!prev_moving && !prev_slow) {
                if (target_angle > cur_angle) {
                    n17_ptr->setDir(pio::STEP_FORWARD);
                } else {
                    n17_ptr->setDir(pio::STEP_REVERSE);
                }
                // ramp_steps = 100;
                // wid_current = wavechain_linear(n17_ptr->pin_step, ramp_steps, 1500, 200);
                wid_current = wavechain_curve(n17_ptr->pin_step, bez_buf, ramp_steps, 2500, 600, 4);
            }
            prev_moving = true;
        }
    }
}

int main(int argc, char *argv[]) {
    gpioCfgClock(2, 0, 0);
    if (gpioInitialise() < 0)
        return -1;
    gpioSetSignalFunc(SIGINT, stop);

    printf("%d, %d\n", pin_dir, pin_step);
    int freq = 10;
    if (argc > 1) {
        freq = atoi(argv[1]);
    }

    std::ifstream rot_ser("rot_pos.txt");
    std::string line;
    if (rot_ser.is_open()) {
        while (std::getline(rot_ser, line)) {
            tpos = cpos = atoi(line.c_str());
            cur_angle = target_angle = cpos * phase_to_radians;
            DUMP(cpos);
            DUMP(cur_angle);
        }
        rot_ser.close();
    }

    pio::Stepper n17(pin_dir, pin_step, 0);
    n17_ptr = &n17;
    pio::RotaryEncoder red(pin_a, pin_b, cbf, pio::RED_MODE_STEP, cpos);

    pthread_t step_tid;
    pthread_create(&step_tid, NULL, step_thread, (void *)&n17);
    // for ( int i = 0; i < 10; i++)
    // n17.singleStep_thread(NULL);
    // n17.setFreq(1000);
    // DUMP(gpioGetPWMfrequency(n17.pin_step));

    n17.setDir(pio::STEP_FORWARD);
    // DUMP(wavechain_linear(pin_step, 200, 120, 1500));
    // gpioWaveTxStop();
    // printf("%d\n", cpos);

    while (run) {
        float t;
        printf("\nangle: ");
        std::cin >> t;
        target_angle = t;
        tpos = target_angle / phase_to_radians;
    }

    pthread_join(step_tid, NULL);
    gpioTerminate();
    return 0;

    // gpioHardwarePWM(n17.pin_step, 1e6*0.25, 1e6*0.5);
    // time_sleep(2);
    // gpioHardwarePWM(n17.pin_step, 1e6, 0);
    // gpioTerminate();
    // return 0;
}