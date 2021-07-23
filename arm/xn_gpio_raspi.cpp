#include "xn_gpio.hpp"

#ifndef XN_I2C_PRINT_ERRORS
#define XN_I2C_PRINT_ERRORS 0
#endif

namespace xn {

void PigDaemon::init() { pi = pigpio_start(NULL, NULL); }

void PigDaemon::stop() { pigpio_stop(pi); }

int PigDaemon::error_check(int r) {
    if (r < 0 && XN_I2C_PRINT_ERRORS) {
        std::cout << "[PIGD ERROR] " << pigpio_error(r) << '\n';
    }
    return r;
}

int PigDaemon::i2cOpen(unsigned bus, unsigned addr) {
    return error_check(i2c_open(pi, bus, addr, 0));
}
int PigDaemon::i2cClose(unsigned handle) { return error_check(i2c_close(pi, handle)); }

char PigDaemon::i2cReadByte(int h) { return error_check(i2c_read_byte(pi, h)); }

int PigDaemon::i2cRead(int h, char *buf, int n) {
    return error_check(i2c_read_device(pi, h, buf, n));
}

int PigDaemon::i2cWrite(int h, char *buf, int n) {
    return error_check(i2c_write_device(pi, h, buf, n));
}

int PigDaemon::servo(unsigned gpio, unsigned pulsewidth) {
    return error_check(set_servo_pulsewidth(pi, gpio, pulsewidth));
}

int PigDaemon::serialOpen(char *ser_tty, unsigned baud) {
    return error_check(serial_open(pi, ser_tty, baud, NULL));
}

int PigDaemon::serialClose(int h) { return error_check(serial_close(pi, h)); }

int PigDaemon::serialRead(int h, char *buf, unsigned count) {
    return error_check(serial_read(pi, h, buf, count));
}

int PigDaemon::serialWrite(int h, char *buf, unsigned count) {
    return error_check(serial_write(pi, h, buf, count));
}

int PigDaemon::serialDataAvailable(int h) { return error_check(serial_data_available(pi, h)); }

namespace pio {

// longitudinal redundancy check for i2c data
uint8_t checksum_LRC(uint8_t *buf, unsigned n) {
    int LRC = 0;
    for (int i = 0; i < n; i++) {
        LRC -= buf[i];
    }
    return (uint8_t)LRC;
}

int i2c_read_frame(int h, char *buf, int n, const char begin_frame_byte) {
    char *frame = new char[n + 2];

    int rb = PigDaemon::i2cRead(h, frame, n + 2);

    while (frame[0] != begin_frame_byte) {
        for (int i = 0; i < (n + 1); i++) {
            frame[i] = frame[i + 1];
        }
        frame[n + 1] = PigDaemon::i2cReadByte(h);
        // time_sleep(0.005);
        // std::cout << "invalid packet: " << frame << '\n';
    }

    uint8_t rx_checksum = frame[n + 1];
    uint8_t calc_checksum = checksum_LRC((uint8_t *)frame, n + 1);
    if (rx_checksum == calc_checksum) {
        std::memcpy(buf, &frame[1], n);
    } else {
        // std::cout << "failed checksum: " << rx_checksum << " : " << calc_checksum << '\n';
        delete[] frame;
        return -1;
    }

    delete[] frame;
    return 1;
}

// // create gpio pulse w/ usDelay
// int pulse_create(int pin, int usHigh, int numPulses, int usLow) {
//     // gpioWaveClear();
//     if (usLow == -1) {
//         usLow = usHigh;
//     }
//     gpioPulse_t pulse[numPulses * 2];
//     for (int i = 0; i < numPulses * 2; i += 2) {
//         pulse[i].gpioOn = (1 << pin);
//         pulse[i].gpioOff = 0;
//         pulse[i].usDelay = usHigh;

//         pulse[i + 1].gpioOn = 0;
//         pulse[i + 1].gpioOff = (1 << pin);
//         pulse[i + 1].usDelay = usLow;
//     }

//     gpioWaveAddNew();
//     gpioWaveAddGeneric(numPulses * 2, pulse);
//     return gpioWaveCreate();
// }

// /*

//              +---------+         +---------+      0
//              |         |         |         |
//    A         |         |         |         |
//              |         |         |         |
//    +---------+         +---------+         +----- 1

//        +---------+         +---------+            0
//        |         |         |         |
//    B   |         |         |         |
//        |         |         |         |
//    ----+         +---------+         +---------+  1

// */
// const int RotaryEncoder::transits[] = {
//     /* 0000 0001 0010 0011 0100 0101 0110 0111 */
//     0, -1, 1, 0, 1, 0, 0, -1,
//     /* 1000 1001 1010 1011 1100 1101 1110 1111 */
//     -1, 0, 0, 1, 0, 1, -1, 0};

// RotaryEncoder::RotaryEncoder(unsigned pin_a, unsigned pin_b, void (*callback)(int), int mode,
//                              int step)
//     : pin_a(pin_a), pin_b(pin_b), callback(callback), step(step), mode(mode) {

//     lev_a = 0;
//     lev_b = 0;
//     glitch = 1000;

//     gpioSetMode(pin_a, PI_INPUT);
//     gpioSetMode(pin_b, PI_INPUT);

//     // need pull up b/c enncoder common is grounded
//     gpioSetPullUpDown(pin_a, PI_PUD_UP);
//     gpioSetPullUpDown(pin_b, PI_PUD_UP);

//     gpioGlitchFilter(pin_a, glitch);
//     gpioGlitchFilter(pin_b, glitch);

//     state_old = (gpioRead(pin_a) << 1) | gpioRead(pin_b);

//     cb_id_a = gpioSetAlertFuncEx(pin_a, RotaryEncoder::callback_internal, this);
//     cb_id_a = gpioSetAlertFuncEx(pin_b, RotaryEncoder::callback_internal, this);
// }

// void RotaryEncoder::callback_internal(int gpio, int level, uint32_t tick, void *userdata) {
//     RotaryEncoder *obj = (RotaryEncoder *)userdata;
//     int state_new, inc, detent;

//     if (level != PI_TIMEOUT) {
//         if (gpio == obj->pin_a)
//             obj->lev_a = level;
//         else
//             obj->lev_b = level;

//         state_new = obj->lev_a << 1 | obj->lev_b;
//         inc = transits[obj->state_old << 2 | state_new];

//         if (inc) {
//             obj->state_old = state_new;
//             detent = obj->step / 4;

//             obj->step += inc;

//             if (obj->callback) {
//                 if (obj->mode == RED_MODE_DETENT) {
//                     if (detent != obj->step / 4)
//                         obj->callback(obj->step / 4);
//                 } else
//                     obj->callback(obj->step);
//             }
//         }
//     }
// }

// Stepper::Stepper(int pin_dir, int pin_step, float dutycycle, unsigned freq, int pin_ms1,
//                  int pin_ms2, int pin_ms3) {
//     this->pin_dir = pin_dir;
//     this->pin_step = pin_step;
//     this->pin_ms1 = pin_ms1;
//     this->pin_ms2 = pin_ms2;
//     this->pin_ms3 = pin_ms3;

//     gpioSetMode(pin_dir, PI_OUTPUT);
//     gpioSetMode(pin_step, PI_OUTPUT);
//     setDutyCycle(dutycycle);
//     setFreq(freq);
// }

// void Stepper::setDutyCycle(float ds) {
//     dutycycle = ds;
//     gpioPWM(pin_step, dutycycle_range * ds);
// }

// void Stepper::setDir(enum StepDir dir) {
//     this->dir = dir;
//     gpioWrite(pin_dir, dir);
// }

// void Stepper::setFreq(unsigned f) {
//     freq = f;
//     gpioSetPWMfrequency(pin_step, f);
// }

/*
    This waits for step_duration, should probably happen in a separate thread
TODO:
    - async callback alternative (callback is gpioWrite(pin_step, 0))
    - disable pwm for class users during the wait
    - queue of step instructions?
*/
// void *Stepper::singleStep_thread(void *argv) {
//     setDutyCycle(0);
//     gpioWrite(pin_step, 1);
//     time_sleep(step_duration);
//     gpioWrite(pin_step, 0);
//     time_sleep(step_duration);
//     return NULL;
// }

} // namespace pio
} // namespace xn