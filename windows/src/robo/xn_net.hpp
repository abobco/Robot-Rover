/*
    TODO:
    - fix disconnect during scan:
        - the arduino immediately detects the disconnect, but cannot reliably
   detect successfuly connection:
        - after attempting to connect to the host, the arduino will always
   return connected() = true, even if there is no host program running
        - the host pc does not detect the disconnect until failing to write to
   the socket
        - IDEA: host pc should write to the socket after every read to verify
   the connection exists
            - maintain servo and motor angles on host pc instead of sending them
   from the car.
            - break scan instruction into 3 instructions:
                1 get lidar value
                2 step lidar motor
                3 step lidar servo
              why?
                - simplifies arduino code: no need to remember the scan state
   after a disconnect
                - less bandwidth consumption(kinda... more reads for the
   arduino, less writes and overall traffic)
                - eliminates the long while loop that currently holds up the
   loop() function on the arduino
        - UPDATE: 2/10:
            - I broke the lidar scan instruction into the 3 lower level
   instructions described above, This works, but is much slower than the
   previous method. Almost all of the delay comes from reading from wifi socket
   on the arduino. More specifically, the delay comes from the following
   scenario:
                1. The arduino is busily waiting to read the next instruction
   from the host
                2. The host sends the data.
                3. The arduino waits "too long" for the data, taking 50-200 ms
   before reading I improved the delay by sending the data size and instruction
   together with 1 send, but it is still at ~1/2 the speed of the old method.
            - IDEA: send multiple instructions at once, then wait for all
   acknowledgements of these instructions to return. After some timeout, if the
   expected acks are not received, wait for a reconnect and restart the protocol
   at the last unacknowledged instruction
*/

#pragma once

// #pragma comment(lib, "ws2_32.lib")
#include <winsock2.h>
#include <ws2tcpip.h>
// #include <arpa/inet.h>
#include <assert.h>
#include <cstring>
// #include <netdb.h>
#include "../util/xn_vec.hpp"
#include "xn_gpio.hpp"
#include "xn_log.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <type_traits>
// #include <unistd.h>
#include <esp32/rover/robot_instructions.h>

#ifndef bzero
#define bzero(b, len) (memset((b), '\0', (len)), (void)0)
#endif
#ifndef bcopy
#define bcopy(b1, b2, len) (memmove((b2), (b1), (len)), (void)0)
#endif

namespace xn {

void error(const char *msg);

void wserror(const char *msg);

SOCKET accept_connection_blocking(unsigned port, bool verbose = true);

int read_buf(SOCKET sock, char *buf, uint32_t len, float timeout = 5);

int wait_ack(SOCKET &sock, float timeout = 1);

MessageRaw read_message(int sock);

template <class IntGeneric>
int read_int(SOCKET sock, IntGeneric &val,
             bool convert_from_net_byte_order = true, float timeout = 1) {

  int res = read_buf(sock, (char *)&val, sizeof(val), timeout);
  if (res == -1)
    return res;

  if (convert_from_net_byte_order)
    val = (IntGeneric)ntohl((u_long)val);
  return 1;
}

template <class IntGeneric>
void send_int(SOCKET sock, IntGeneric val,
              bool convert_to_net_byte_order = true) {
  IntGeneric nval = val;
  if (convert_to_net_byte_order)
    nval = htonl(val);
  int bs = send(sock, (const char *)&nval, sizeof(nval), 0);
  if (bs == -1)
    perror("socket write fail");
  assert(bs == sizeof(nval));
}

template <class InstructionGeneric>
uint32_t pack_instruction(const InstructionGeneric &instruction, char *buf) {
  uint32_t size_out = sizeof(instruction);
  std::copy((char *)&size_out, (char *)&size_out + sizeof(size_out), buf);
  std::copy((char *)&instruction, (char *)&instruction + sizeof(instruction),
            &buf[4]);
  return size_out + sizeof(size_out);
}

template <class T> int send_pack(SOCKET &sock, T &instruction) {
  char *buf = new char[sizeof(uint32_t) + sizeof(instruction)];
  uint32_t size = sizeof(instruction);
  std::copy((char *)&size, (char *)&size + sizeof(size), buf);
  std::copy((char *)&instruction, (char *)&instruction + sizeof(instruction),
            &buf[4]);
  int bs = send(sock, buf, sizeof(uint32_t) + sizeof(instruction), 0);
  assert(bs == sizeof(uint32_t) + sizeof(instruction));

  return wait_ack(sock);
}

} // namespace xn