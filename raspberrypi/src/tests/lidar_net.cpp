#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include <assert.h>
#include <netdb.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <iostream>

#include "XNlib/xn_gpio.hpp"
#include "XNlib/xn_net.hpp"

#define laptop_port 4000
#define laptop_ip "192.168.0.183"

using namespace xn;

int pio::PigDaemon::pi = 0;

int pin_sonar = 23, pin_step = 12, pin_step_dir = 16, sock;
bool ready = false, run = true, dir = 0;
const float step_time = 0.01, usDelay = 20000;
float servo_ang;

uint32_t step = 0;
int step_inc = 1;

#define PIG_TX_FOR_Y(wid, n, y) 255, 0, wid, 255, 1, n, y
#define PIG_TX_FOR(wid, n) PIG_TX_FOR_Y(wid, n, 0)

void stop(int signum) { run = 0; }

// void error(const char *msg) {
//     perror(msg);
//     exit(0);
// }

int lidar_val;

void *lidar_io_thread(void *argv) {

    int handle = serOpen("/dev/ttyS0", 115200, 0);
    if (handle < 0) {
        printf("error opening serial port\n");
        return NULL;
    }
    while (run) {
        char packet[9];
        if (serDataAvailable(handle) >= 9)
            serRead(handle, packet, 9);

        if (packet[0] == 0x59 && packet[1] == 0x59) {
            int low = packet[2];
            int high = packet[3];
            lidar_val = low + high * 256;
            DUMP(lidar_val);
        }
    }
    return NULL;
}

// send 1 pulse, wait for it to finish
double wavechain_tx(int pin, int usDelay, int n = 200) {
    pio::TimePoint t_start = pio::get_time();
    int wid = pio::pulse_create(pin, usDelay);

    char wavechain[] = {PIG_TX_FOR(wid, n)};
    gpioWaveChain(wavechain, sizeof(wavechain));
    DUMP(gpioDelay(n * usDelay * 2 + 100));

    gpioWaveDelete(wid);

    return pio::time_diff_seconds(t_start, pio::get_time());
}

int main(int argc, char *argv[]) {
    if (gpioInitialise() < 0)
        return -1;
    gpioSetSignalFunc(SIGINT, stop);

    gpioSetMode(pin_step, PI_OUTPUT);
    gpioSetMode(pin_step_dir, PI_OUTPUT);
    gpioWrite(pin_step_dir, dir);

    gpioSetPullUpDown(pin_step_dir, PI_PUD_OFF);
    gpioSetPullUpDown(pin_step, PI_PUD_OFF);

    int servo_pin = 13;
    if (argc > 1) {
        servo_pin = atoi(argv[1]);
    }

    auto servo = pio::ServoAngular(servo_pin, 500, 2500);
    servo.setAngle(0.3);
    const float servo_start_ang = servo.getAngle();

    // int sonar_pulse = pio::pulse_create(pin_sonar, 5, 1);
    int step_pulse = pio::pulse_create(pin_step, 300, 2);

    struct sockaddr_in serv_addr = {0};
    struct hostent *server;

    // connect to gui host
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0)
        error("ERROR opening socket");
    server = gethostbyname(laptop_ip);
    if (server == NULL) {
        fprintf(stderr, "ERROR, no such host\n");
        exit(0);
    }
    // bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
    serv_addr.sin_port = htons(laptop_port);
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
        error("ERROR connecting");

    pthread_t tid;
    pthread_create(&tid, NULL, lidar_io_thread, NULL);

    while (run) {
        // gpioWaveTxSend(step_pulse, PI_WAVE_MODE_ONE_SHOT);
        wavechain_tx(pin_step, usDelay, 1);
        // gpioWrite(pin_step, 1);
        // time_sleep(step_time);
        // gpioWrite(pin_step, 0);
        if (step + step_inc > 100 || step + step_inc < 0) {
            step_inc *= -1;
            dir = !dir;
            gpioWrite(pin_step_dir, dir);
            servo.setAngle(servo.getAngle() + 0.1);
            DUMP(servo_ang = servo.getAngle() - servo_start_ang);
        }
        step += step_inc;

        time_sleep(step_time);
        DUMP(lidar_val);
        // DUMP(step);
        // DUMP(servo_ang);
        send_int<int32_t>(sock, lidar_val);
        send_int<int32_t>(sock, step);
        send_int<int32_t>(sock, servo_ang * 1000);
    }

    printf("\nlater nerd\n");

    servo.setPosition(0);

    gpioTerminate();

    return 0;
}
