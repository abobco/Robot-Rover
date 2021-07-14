#pragma once

#include "robot_instructions.h"
#include "robot_ipconfig.h"
#include <WiFi.h>

namespace xn {
bool run = true;
// control pc info
IPAddress laptop_ip;
const int laptop_port = PORT_ESP_DATA;

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
