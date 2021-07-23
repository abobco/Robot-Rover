#include "xn_net.hpp"

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
    if (inet_pton(AF_INET, LAPTOP_IP_STRING, &serv_addr.sin_addr) <= 0) {
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

int read_buf(int sock, char *buf, uint32_t len, float timeout) {
    uint32_t left = len;
    int rc;
    if (timeout > 0.0f) {
        auto start_time = pio::get_time();
        do {
            rc = recv(sock, buf, left, 0);
            buf += rc;
            left -= rc;
        } while (left > 0 && pio::time_diff_seconds(start_time, pio::get_time()) < timeout);
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

int wait_ack(int &sock, int timeout) {
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