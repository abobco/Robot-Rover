#pragma once
#include <cstdint>

namespace xn {

enum InstructionType {
  INSTR_MOTOR,
  INSTR_SENSOR_EX,
  INSTR_SENSOR_360,
  INSTR_LIDAR_STEP,
  INSTR_LIDAR_SERVO,
  INSTR_LIDAR_READ,
  INSTR_RESTART_ESP
};

struct MotorInstruction {
  const int8_t type = INSTR_MOTOR;
  uint32_t length_pulses;
  int8_t dir_right;
  int8_t dir_left;
};

struct LidarInstruction360 {
  const int8_t type = INSTR_SENSOR_360;
  int8_t y_degrees_per_step;
};

struct LidarInstructionEx {
  const int8_t type = INSTR_SENSOR_EX;
  int16_t x_degrees_min;
  int16_t x_degrees_max;
  int16_t y_degrees_min;
  int16_t y_degrees_max;
  int8_t y_degrees_per_step;
};

struct LidarStepInstruction {
  const int8_t type = INSTR_LIDAR_STEP;
  int8_t dir;
};

struct LidarServoInstruction {
  const int8_t type = INSTR_LIDAR_SERVO;
  uint16_t width;
};

struct LidarReadInstruction {
  const int8_t type = INSTR_LIDAR_READ;
};

struct RestartEspInstruction {
  const int8_t type = INSTR_RESTART_ESP;
};

template <class T> struct InstructionPacket {
  size_t size;
  T instr;
};

struct MessageRaw {
  size_t size;
  char *buf;
};
} // namespace xn