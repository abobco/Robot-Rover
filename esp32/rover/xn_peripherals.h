#pragma once

// #include
// "C:/Users/abobc/OneDrive/Documents/Arduino/libraries/ESP32_AnalogWrite/analogWrite.h"

#include "MPU9250.h"
#include <ESP32Servo.h>
#include <Stepper.h>
// #include <pthread.h>
#include <thread>

// #include <analogWrite.h>
// #include <PS4Controller.h>

namespace xn {
template <class T> T clamp(T n, T minval, T maxval) {
    return std::max(minval, std::min(n, maxval));
}

template <typename T> T sign(T value) { return T((value > 0) - (value < 0)); }

volatile long interrupt_counter[4] = {0};
void InterruptCallback_0() { interrupt_counter[0]++; }
void InterruptCallback_1() { interrupt_counter[1]++; }
void InterruptCallback_2() { interrupt_counter[2]++; }
void InterruptCallback_3() { interrupt_counter[3]++; }

// trigger a pthread signal when counter breaks threshold
// volatile long interrupt_counter[4] = {0};
// pthread_cond_t interrupt_cond[4] = {
//     PTHREAD_COND_INITIALIZER, PTHREAD_COND_INITIALIZER,
//     PTHREAD_COND_INITIALIZER, PTHREAD_COND_INITIALIZER};
// volatile long threshold[4] = {INT_MAX, INT_MAX, INT_MAX, INT_MAX};
// void InterruptCallback_0() {
//   if (interrupt_counter[0]++ > threshold[0]) {
//     pthread_cond_signal(&interrupt_cond[0]);
//   }
// }
// void InterruptCallback_1() {
//   if (interrupt_counter[1]++ > threshold[1]) {
//     pthread_cond_signal(&interrupt_cond[1]);
//   }
// }
// void InterruptCallback_2() {
//   if (interrupt_counter[2]++ > threshold[2]) {
//     pthread_cond_signal(&interrupt_cond[2]);
//   }
// }
// void InterruptCallback_3() {
//   if (interrupt_counter[3]++ > threshold[3]) {
//     pthread_cond_signal(&interrupt_cond[3]);
//   }
// }

void (*InterruptCallback[4])() = {InterruptCallback_0, InterruptCallback_1, InterruptCallback_2,
                                  InterruptCallback_3};

class RotaryEncoder {
    uint8_t value_a_idx = 0, value_b_idx = 0;

  public:
    static int cb_num;
    uint8_t a = 0;
    uint8_t b = 0;

    RotaryEncoder() {}

    RotaryEncoder(uint8_t a, uint8_t b) {
        this->a = a;
        this->b = b;
        this->value_a_idx = cb_num++;
        // this->value_b_idx = cb_num++;
        printf("a_idx %d\n", this->value_a_idx);
        // printf("b_idx %d\n", this->value_b_idx);
    }

    void init() {
        pinMode(a, INPUT_PULLUP);
        pinMode(b, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(a), InterruptCallback[value_a_idx], RISING);
        // attachInterrupt(digitalPinToInterrupt(b), InterruptCallback[value_b_idx],
        //                 RISING);
    }

    long getval_a() { return interrupt_counter[value_a_idx]; }
    // long getval_b() { return interrupt_counter[value_b_idx]; }
    void setval_a(long v) { interrupt_counter[value_a_idx] = v; }
    // void setval_b(long v) { interrupt_counter[value_b_idx] = v; }
    void reset_counters() {
        setval_a(0);
        // setval_b(0);
    }
};

struct MotorDC {
    uint8_t in1 = 0;
    uint8_t in2 = 0;
    int16_t speed = 0;

    MotorDC(uint8_t in1 = 0, uint8_t in2 = 0) {
        this->in1 = in1;
        this->in2 = in2;
        pinMode(in1, OUTPUT);
        pinMode(in2, OUTPUT);

        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }

    void set_speed(int v, int range = 1024) {
        speed = clamp(v, -range, range);
        if (speed >= 0) {
            analogWrite(in1, speed, range);
            digitalWrite(in2, LOW);
        } else {
            analogWrite(in1, range - abs(speed), range);
            digitalWrite(in2, HIGH);
        }
    }

    static void set_2WD(MotorDC motors[], int16_t speed, int16_t dir) {
        if (dir == 0) {
            for (int i = 0; i < 2; i++) {
                int16_t speed_corrected = speed * 2 - i * 12;
                if (sign(speed_corrected) != sign(speed))
                    speed_corrected = 0;

                motors[i].set_speed(-speed_corrected);
            }
        } else if (dir > 0) {
            motors[0].set_speed(abs(speed * 2));
            motors[1].set_speed(-abs(speed * 2));
        } else {
            motors[0].set_speed(-abs(speed * 2));
            motors[1].set_speed(abs(speed * 2));
        }
    }
};

enum MotorDirection { MOTOR_FORWARD, MOTOR_REVERSE };

struct WheelMotor {
    const static int ENC_CPR = 4320;
    const static int interval_ms = 1;
    static pthread_cond_t rpm_cond;
    static float target_rpm;

    MotorDC motor;
    RotaryEncoder encoder;
    float wheel_radius;
    pthread_t rotate_tid;
    float enc_target = 0;
    bool reached_target = true;
    bool stop = true;
    bool prev_stop = false;
    int best_speed = 900;
    int prev_pulse;
    unsigned long prev_time;
    MotorDirection dir = MOTOR_FORWARD;

    WheelMotor() {}

    WheelMotor(int mot_in1, int mot_in2, int rot_a, int rot_b, float radius = 5) {
        this->motor = MotorDC(mot_in1, mot_in2);
        this->encoder = RotaryEncoder(rot_a, rot_b);
        this->wheel_radius = radius;
        prev_time = micros();
    }

    long get_encoder_pulses() { return encoder.getval_a(); }

    static void move_motors(const MotorInstruction &instr, WheelMotor wheels[]) {
        for (int i = 0; i < 2; i++) {
            // reset encoder pulse counters
            wheels[i].encoder.reset_counters();
            wheels[i].stop = false;
            wheels[i].motor.set_speed(wheels[i].best_speed);
        }
        // set wheel directions individually
        wheels[0].dir = (xn::MotorDirection)instr.dir_left;
        wheels[1].dir = (xn::MotorDirection)instr.dir_right;

        while (1) {
            // bool all_done = true;
            // for (int i = 0; i < 2; i++) {
            //   if (wheels[i].motor.speed != 0) {
            //     all_done = false;
            //     break;
            //   }
            // }

            // if (all_done)
            //   break;

            //   long min_pulse = INT_MAX;
            //   for (int i = 0; i < 2; i++) {
            //     long p = wheels[i].get_encoder_pulses();
            //     if (p < min_pulse)
            //       min_pulse = p;
            //     if (wheels[i].get_encoder_pulses() >= instr.length_pulses)
            //       wheels[i].stop = true;
            //   }
            //   if (instr.length_pulses - min_pulse > 1000) {
            //     delay(1);
            //   } else
            //     delayMicroseconds(200);
            //   // delay(1);

            delay(interval_ms);
            for (int i = 0; i < 2; i++) {
                WheelMotor &w = wheels[i];

                // regulate rpm
                unsigned time = micros();
                long pulses = w.get_encoder_pulses();

                if (pulses >= instr.length_pulses) {
                    wheels[0].motor.set_speed(0);
                    wheels[1].motor.set_speed(0);
                    return;
                }

                float val = (float)pulses / ENC_CPR;
                float rpm =
                    1e6f / (time - w.prev_time) * 60.0f * (pulses - w.prev_pulse) / (float)ENC_CPR;

                int sign = 1;
                float drift_correction = 0;
                if (w.dir == MOTOR_FORWARD)
                    sign = -1;
                if (rpm < (i == 1 ? target_rpm : target_rpm + drift_correction)) {
                    w.motor.set_speed(sign * (abs(w.motor.speed) + 1));
                } else if (rpm > (i == 1 ? target_rpm : target_rpm + drift_correction)) {
                    w.motor.set_speed(sign * (abs(w.motor.speed) - 1));
                }

                // printf("%d: rpm: %f\n", i, rpm);
                // printf("%d: time: %ld\n", i, time - w.prev_time);
                w.prev_pulse = pulses;
                w.prev_time = time;
                w.best_speed = w.motor.speed;
            }
        }
    }
}; // namespace xn

// TODO: replace w/ hardware timer
void step_motor(int pin_step, int delay_ms) {
    digitalWrite(pin_step, HIGH);
    delay(delay_ms);
    digitalWrite(pin_step, LOW);
}

bool tfMiniDetectFrame(unsigned char *readBuffer) {
    return readBuffer[0] == 0x59 && readBuffer[1] == 0x59 &&
           (unsigned char)(0x59 + 0x59 + readBuffer[2] + readBuffer[3] + readBuffer[4] +
                           readBuffer[5] + readBuffer[6] + readBuffer[7]) == readBuffer[8];
}

// void handlePS4Controller() {
//   int thres = 30;
//   if (PS4.isConnected()) {
//       if ( PS4.event.analog_move.stick.lx )
//         lx = PS4.data.analog.stick.lx;
//       if ( PS4.event.analog_move.stick.ly )
//         ly = PS4.data.analog.stick.ly;
//       if ( lx < -thres && ly > thres ) {
//         setMotorSpeed(fr, ly*2, MOTOR_FORWARD);
//         setMotorSpeed(br, ly*2, MOTOR_FORWARD);
//         setMotorSpeed(fl, 0, MOTOR_FORWARD);
//         setMotorSpeed(bl, 0, MOTOR_FORWARD);
//       } else if ( lx > thres && ly > thres ) {
//         setMotorSpeed(fl, ly*2, MOTOR_FORWARD);
//         setMotorSpeed(bl, ly*2, MOTOR_FORWARD);
//         setMotorSpeed(fr, 0, MOTOR_FORWARD);
//         setMotorSpeed(br, 0, MOTOR_FORWARD);
//       } else if ( abs(lx) <= thres && ly > thres ) {
//         setMotorSpeed(fl, ly*2, MOTOR_FORWARD);
//         setMotorSpeed(bl, ly*2, MOTOR_FORWARD);
//         setMotorSpeed(fr, ly*2, MOTOR_FORWARD);
//         setMotorSpeed(br, ly*2, MOTOR_FORWARD);
//       } else if ( ly <= -thres ) {
//         setMotorSpeed(fl, abs(ly)*2-1, MOTOR_REVERSE);
//         setMotorSpeed(bl, abs(ly)*2-1, MOTOR_REVERSE);
//         setMotorSpeed(fr, abs(ly)*2-1, MOTOR_REVERSE);
//         setMotorSpeed(br, abs(ly)*2-1, MOTOR_REVERSE);
//       } else {
//         setMotorSpeed(fr, 0, MOTOR_FORWARD);
//         setMotorSpeed(br, 0, MOTOR_FORWARD);
//         setMotorSpeed(fl, 0, MOTOR_FORWARD);
//         setMotorSpeed(bl, 0, MOTOR_FORWARD);
//       }
//   } else {
//         setMotorSpeed(fr, 0, MOTOR_FORWARD);
//         setMotorSpeed(br, 0, MOTOR_FORWARD);
//         setMotorSpeed(fl, 0, MOTOR_FORWARD);
//         setMotorSpeed(bl, 0, MOTOR_FORWARD);
//     }

//    delay(14);
// }

} // namespace xn