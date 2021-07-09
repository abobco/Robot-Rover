// const static unsigned BUF_LEN = 256;
// const float rest_volts = 2.09579;
// const float ratio = 0.45 / (rest_volts - 2.05941);

// unsigned current_sensor_read_count = 0;
// unsigned current_sensor_read_buf[BUF_LEN];
#pragma once
#include <iostream>
#include <thread>

#define DUMP(a)                                                                                    \
    { std::cout << #a " = " << (a) << std::endl; }

struct CurrentSensor {
    const static unsigned READ_BUF_LEN = 256;
    static constexpr float rest_volts = 1.389;
    static constexpr float ratio = 20 / rest_volts;

    unsigned pin;
    unsigned read_count = 0;
    unsigned read_buf[READ_BUF_LEN];

    CurrentSensor() {}

    CurrentSensor(unsigned pin) {
        this->pin = pin;
        pinMode(pin, INPUT);
    }

    static void currentSensorThread(CurrentSensor *cs) {
        DUMP(cs->pin);
        while (true) {
            if (cs->read_count < READ_BUF_LEN) {
                cs->read_buf[cs->read_count++] = analogRead(cs->pin);
            } else {
                for (int i = (READ_BUF_LEN - 1); i > 0; i--) {
                    cs->read_buf[i] = cs->read_buf[i - 1];
                }
                cs->read_buf[0] = analogRead(cs->pin);
                // DUMP(cs->read_buf[0]);
            }
            delay(1);
        }
    }

    std::thread init() { return std::thread(currentSensorThread, this); }

    float getVolts() {
        unsigned sum = 0;
        float volts;
        for (int i = 0; i < read_count; i++) {
            sum += read_buf[i];
        }

        return (sum / (float)read_count) * 3.3f / 4095;
    }

    float getAmps() { return abs(rest_volts - getVolts()) * ratio; }
};