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

#include "xn_vec.hpp"
#include <arpa/inet.h>
#include <assert.h>
#include <cstring>
#include <esp32/rover/robot_instructions.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <type_traits>
#include <unistd.h>

#define laptop_port 5002
// #define laptop_ip "192.168.0.102"
#define laptop_ip "192.168.0.100"

// #define bzero(b, len) (memset((b), '\0', (len)), (void)0)

namespace xn {

void error(const char *msg) {
  perror(msg);
  exit(1);
}

int accept_connection_blocking(unsigned port) {
  int sockfd;
  socklen_t clilen;
  struct sockaddr_in serv_addr, cli_addr;
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
    error("ERROR opening socket");
  bzero((char *)&serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  serv_addr.sin_port = htons(port);
  int enable = 1;
  if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
    error("setsockopt(SO_REUSEADDR) failed");
  if (bind(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    error("ERROR on binding");
  listen(sockfd, 5);
  clilen = sizeof(cli_addr);

  printf("waiting for connection...\n");
  int newsock = accept(sockfd, (struct sockaddr *)&cli_addr, &clilen);
  close(sockfd);

  struct timeval tv;
  tv.tv_sec = 1;
  tv.tv_usec = 0;
  setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(tv));
  return newsock;
}

int connect_socket_blocking(int port) {
  int sock;
  struct sockaddr_in serv_addr;
  if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    printf("socket() failed:\n");
    return -1;
  }
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(port);
  if (inet_pton(AF_INET, laptop_ip, &serv_addr.sin_addr) <= 0) {
    printf("Invalid address\n");
    return -1;
  }

  // blocks until accepted
  if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
    printf("connect() failed\n");
    return -1;
  }
  return sock;
}

int read_buf(int sock, char *buf, uint32_t len, float timeout = 1) {
  uint32_t left = len;
  int rc;
  if (timeout > 0.0f) {
    auto start_time = pio::get_time();
    do {
      rc = recv(sock, buf, left, 0);
      buf += rc;
      left -= rc;
    } while (left > 0 &&
             pio::time_diff_seconds(start_time, pio::get_time()) < timeout);
    if (left > 0) {
      return -1;
    }
  } else {
    do {
      rc = recv(sock, buf, left, 0);
      buf += rc;
      left -= rc;
    } while (left > 0);
  }
  return 0;
}

int send_buf(int sock, char *buf, uint32_t len) {
  uint32_t left = len;
  int bs;
  do {
    bs = send(sock, buf, left, 0);
    if (bs < 0) {
      return bs;
    }
    buf += bs;
    left -= bs;
  } while (left > 0);
  return 0;
}

template <class IntGeneric>
int read_int(int sock, IntGeneric &val, bool convert_from_net_byte_order = true,
             float timeout = 1) {
  // IntGeneric tmp;
  // uint32_t left = sizeof(IntGeneric);
  // char *buf = (char *)&tmp;
  // int rc;
  // auto start_time = pio::get_time();
  // do {
  //     rc = recv(sock, buf, left, 0);
  //     buf += rc;
  //     left -= rc;
  // } while (left > 0 && pio::time_diff_seconds(start_time, pio::get_time()) <
  // timeout); if (left > 0) {
  //     return -1;
  // }
  int res = read_buf(sock, (char *)&val, sizeof(val), timeout);
  if (res == -1)
    return res;

  if (convert_from_net_byte_order)
    val = ntohl(val);
  return 1;
}

template <class IntGeneric>
void send_int(int sock, IntGeneric val, bool convert_to_net_byte_order = true) {
  IntGeneric nval = val;
  if (convert_to_net_byte_order)
    nval = htonl(val);
  int bs = send(sock, (const char *)&nval, sizeof(nval), 0);
  // DUMP(bs);
  if (bs == -1)
    perror("socket write fail");
  assert(bs == sizeof(nval));
}

template <class InstructionGeneric>
uint32_t pack_instruction(const InstructionGeneric &instruction, char *buf) {
  // *buf_out = new char[sizeof(uint32_t) + sizeof(instruction)];
  uint32_t size_out = sizeof(instruction);
  std::copy((char *)&size_out, (char *)&size_out + sizeof(size_out), buf);
  std::copy((char *)&instruction, (char *)&instruction + sizeof(instruction),
            &buf[4]);
  return size_out + sizeof(size_out);
}

int wait_ack(int &sock, int timeout = 1) {
  int32_t ack = 0;
  if (read_int<int32_t>(sock, ack, false, timeout) < 0) {
    close(sock);
    // shutdown(sock, SHUT_WR);
    printf("reconnecting\n");
    sock = accept_connection_blocking(4000);
    return 0;
  }
  return 1;
}

template <class T> int send_pack(int &sock, T &instruction) {
  char *buf = new char[sizeof(uint32_t) + sizeof(instruction)];
  // DUMP(instruction.type);
  uint32_t size = sizeof(instruction);
  std::copy((char *)&size, (char *)&size + sizeof(size), buf);
  std::copy((char *)&instruction, (char *)&instruction + sizeof(instruction),
            &buf[4]);
  // send_int<uint32_t>(sock, sizeof(instruction), false);
  // int bs = send(sock, (const char *)&instruction, sizeof(instruction), 0);
  int bs = send(sock, buf, sizeof(uint32_t) + sizeof(instruction), 0);
  // DUMP(bs);
  assert(bs == sizeof(uint32_t) + sizeof(instruction));

  return wait_ack(sock);
}

MessageRaw read_message(int sock) {
  MessageRaw out = {0};
  read_int<size_t>(sock, out.size, false);
  uint32_t left = out.size;
  char *tbuf = out.buf;
  int rc;
  do {
    rc = recv(sock, tbuf, left, 0);
    tbuf += rc;
    left -= rc;
  } while (left > 0);
  return out;
}
} // namespace xn