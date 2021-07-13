#include <analogWrite.h>

// #include "xn_ik.h"
#include "xn_net.h"
#include "xn_peripherals.h"

using namespace xn;

int RotaryEncoder::cb_num = 0;
float WheelMotor::target_rpm = 23;
WheelMotor wheels[2];

void setup() {
  Serial.begin(115200);

  wheels[0] = WheelMotor(18, 5, 33, 25);
  wheels[1] = WheelMotor(4, 0, 26, 27);

  ESP32PWM::allocateTimer(2);

  wheels[0].encoder.init();
  wheels[1].encoder.init();
}

void loop() {
  printf("start time: %d\n", millis());

  MotorInstruction instr;
  instr.length_pulses = 500;
  instr.dir_left = 0;
  instr.dir_right = 0;

  WheelMotor::move_motors(instr, wheels);
  // wheels[0].motor.set_speed(1024);
  // // wheels[1].motor.set_speed(1024);
  // for (int i = 0; i < 2; i++) {
  //     wheels[i].motor.set_speed(1000);
  // }

  // delay(1000);

  // for (int i = 0; i < 2; i++) {
  //     wheels[i].motor.set_speed(0);
  // }

  delay(1000);

  printf("end time: %d\n", millis());
}