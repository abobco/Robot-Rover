#pragma once

#include <WiFi.h>

namespace xn {
bool run = true;
// control pc info
IPAddress laptop_ip(192, 168, 0, 100);
const int laptop_port = 5005;

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

struct MessageRaw {
  uint32_t size;
  char *buf;
};

void print_msg_type(int8_t type) {
#define printcase(a)                                                           \
  case a:                                                                      \
    printf(#a);                                                                \
    break;

  switch (type) {
    printcase(INSTR_MOTOR);
    printcase(INSTR_SENSOR_EX);
    printcase(INSTR_SENSOR_360);
    printcase(INSTR_LIDAR_STEP);
    printcase(INSTR_LIDAR_SERVO);
    printcase(INSTR_LIDAR_READ);
    printcase(INSTR_RESTART_ESP);
  default:
    printf("invalid type");
    break;
  }
}

void send_buf(WiFiClient &client, char *buf, int len) {
  for (int i = 0; i < len; i++)
    client.write(buf[i]);
}

void read_buf(WiFiClient &client, char *buf, int len) {
  int t = millis(), timeout = 1000;
  int left = len;
  char *tbuf = buf;
  int rc;
  // printf("reading sock\n");
  do {
    // printf("%d\n", millis());
    if (millis() - t > timeout) {
      printf("disconnected\n");
      client.connect(laptop_ip, laptop_port);
      t = millis();
    }
    rc = client.readBytes(buf, left);
    tbuf += rc;
    left -= rc;
    printf("rc=%d\n", rc);
  } while (left > 0);
  // return out;

  // printf("reading %d bytes took %d ms\n", len, millis() - t);
}

MessageRaw read_message(WiFiClient &client) {
  MessageRaw msg = {0};
  read_buf(client, (char *)&(msg.size), sizeof(msg.size));
  msg.buf = new char[msg.size];
  read_buf(client, msg.buf, msg.size);
  return msg;
}
} // namespace xn
