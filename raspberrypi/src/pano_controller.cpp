#include "XNlib/xn_gpio.hpp"
#include "XNlib/xn_net.hpp"
#include <pthread.h>

#define laptop_port 8080
#define laptop_ip "192.168.0.183"

using namespace xn;

int pio::PigDaemon::pi = 0;

int start_read_data = 0;
int sock = 0;

void *cam_io_thread(void *arg) {
    pio::Arducam cam = *(pio::Arducam *)arg;
    while (1) {
        if (start_read_data == 1) {
            // auto t1 = pio::get_time();
            cam.capture();
            // printf("time to capture: %f\n", pio::time_diff_seconds(t1, pio::get_time()));

            // send frame over TCP socket
            if (sock) {
                // send buffer size 1st cuz jpeg compression makes the size unpredictable
                int32_t send_len = cam.frame_size;
                int32_t tmp = htonl(send_len);
                send(sock, &tmp, sizeof(tmp), 0);

                // send frame buffer
                size_t bs = send(sock, cam.frame_buffer, cam.frame_size, 0);

                // printf("frame_len = %d\n", cam.frame_size);
                // printf("bs = %d\n", bs);

                assert(bs == cam.frame_size);
            }
        }
    }
    return NULL;
}

void INThandler(int sig) {
    signal(sig, SIG_IGN);
    exit(0);
}

int main(int argc, char **argv) {
    pthread_t cam_io_tid;
    // int sock;

    if (gpioInitialise() < 0)
        return -1;
    gpioSetSignalFunc(SIGINT, INThandler);

    pio::Arducam cam;
    cam.setQuality(4);

    // open socket for video stream
    struct sockaddr_in serv_addr;
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        printf("socket() failed:\n");
        return -1;
    }
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(laptop_port);
    if (inet_pton(AF_INET, laptop_ip, &serv_addr.sin_addr) <= 0) {
        printf("Invalid address\n");
        return -1;
    }

    // blocks until accepted
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        printf("connect() failed\n");
        return -1;
    }

    start_read_data = 1;
    // pthread_create(&cam_io_tid, NULL, cam_io_thread,
    //                (void *)&cam); // send camera feed to laptop for processing

    pio::ServoAngular servo_x(13), servo_y(6);
    servo_x.setWidth(500);
    servo_y.setWidth(1000);
    time_sleep(1);
    while (servo_y.getWidth() < 1500) {
        static int incr = 100;
        if (servo_x.getWidth() + incr > 1700 || servo_x.getWidth() + incr < 500) {
            incr *= -1;
            servo_y.setWidth(servo_y.getWidth() + 100);
        }
        servo_x.setWidth(servo_x.getWidth() + incr);

        time_sleep(1);
        cam.capture();
        // printf("time to capture: %f\n", pio::time_diff_seconds(t1, pio::get_time()));

        // send frame over TCP socket
        if (sock) {
            // send buffer size 1st cuz jpeg compression makes the size unpredictable
            int32_t send_len = cam.frame_size;
            int32_t tmp = htonl(send_len);
            send(sock, &tmp, sizeof(tmp), 0);

            // send frame buffer
            size_t bs = send(sock, cam.frame_buffer, cam.frame_size, 0);

            // printf("frame_len = %d\n", cam.frame_size);
            // printf("bs = %d\n", bs);

            assert(bs == cam.frame_size);
        }
    }

    pthread_join(cam_io_tid, NULL);
    if (sock)
        close(sock);
}