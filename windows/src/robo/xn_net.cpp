#include "xn_net.hpp"

namespace xn {
void error(const char *msg) {
  perror(msg);
  exit(1);
}

void wserror(const char *msg) {
  xn_printf("[WinSock2 Error] %s: %d\n", msg, WSAGetLastError());
}

SOCKET accept_connection_blocking(unsigned port, bool verbose) {
  WSADATA wsa;
  struct sockaddr_in address;
  int opt = 1;
  int addrlen = sizeof address;
  SOCKET client_sock, server_sock;

  if (verbose) {
    printf("\ninitialising winsock... ");
  }
  if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
    wserror("WSAStartup() failed");
    throw -1;
  }
  if (verbose)
    printf("success\n");

  server_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (server_sock == INVALID_SOCKET) {
    xn_printf("socket() failed: %d", WSAGetLastError());
    throw -1;
  }

  if (setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, (const char *)&opt,
                 sizeof(opt))) {
    wserror("setsockopt");
    throw -1;
  }
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(port);

  if (::bind(server_sock, (struct sockaddr *)&address, sizeof(address)) < 0) {
    wserror("bind failed");
    throw -1;
  }
  if (listen(server_sock, 3) < 0) {
    perror("listen failed");
    throw -1;
  }

  if (verbose)
    printf("waiting for client connection...\n");
  if ((client_sock = accept(server_sock, (struct sockaddr *)&address,
                            (socklen_t *)&addrlen)) < 0) {
    wserror("accept failed");
    throw -1;
  }

  if (verbose)
    printf("connection accepted\n");

  closesocket(server_sock);

  // DWORD timeout = 1000;
  // if (setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout,
  //                sizeof(DWORD))) {
  //   wserror("setsockopt");
  //   throw -1;
  // }

  // unsigned long mode = 1;
  // if (ioctlsocket(client_sock, FIONBIO, &mode) != 0) {
  //   wserror("setnonblock");
  //   throw -1;
  // }

  return client_sock;
}

int read_buf(SOCKET sock, char *buf, uint32_t len, float timeout) {
  uint32_t left = len;
  int rc;
  fd_set set;
  struct timeval tv;
  FD_ZERO(&set);      /* clear the set */
  FD_SET(sock, &set); /* add our file descriptor to the set */
  tv.tv_sec = (long)timeout;
  tv.tv_usec = (long)((timeout - tv.tv_sec) * 1e6);
  int rv = select((int)sock + 1, &set, NULL, NULL, &tv);
  if (rv == SOCKET_ERROR) {
    std::cerr << "select error\n";
    return -1;
  } else if (rv == 0) {
    std::cerr << "select timeout\n";
    return -1;
  } else {
    do {
      rc = recv(sock, buf, left, 0);
      buf += rc;
      left -= rc;
    } while (left > 0);
  }
  return 0;
}

int wait_ack(SOCKET &sock, float timeout, unsigned reconnectPort) {
  int32_t ack = 0;
  if (read_int<int32_t>(sock, ack, false, timeout) < 0) {
    closesocket(sock);
    // shutdown(sock, SHUT_WR);
    printf("reconnecting\n");
    sock = accept_connection_blocking(reconnectPort);
    return 0;
  }
  return 1;
}

MessageRaw read_message(int sock) {
  MessageRaw out = {0};
  read_int<size_t>(sock, out.size, false);
  int left = (int)out.size;
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