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

#define laptop_port 4000
#define laptop_ip "192.168.0.183"

using namespace xn;
int pio::PigDaemon::pi = 0;

int pin_sonar = 23, pin_step = 12, pin_step_dir = 16, sock;
bool ready = false, run = true, dir = 0;
float servo_ang;

uint32_t step = 0;
int step_inc = 1;

#define PIG_TX_FOR_Y(wid, n, y) 255, 0, wid, 255, 1, n, y
#define PIG_TX_FOR(wid, n) PIG_TX_FOR_Y(wid, n, 0)

void stop(int signum) { run = 0; }

void error(const char *msg) {
    perror(msg);
    exit(0);
}

void send_uint32_t(int sock, uint32_t val) {
    uint32_t nval = htonl(val);
    int bs = send(sock, (const char *)&nval, sizeof(nval), 0);
    assert(bs == sizeof nval);
}

void sonarAlert(int gpio, int level, uint32_t tick) {
    static uint32_t t1, dt, srv_ang_conv;

    if (level == 1) {
        t1 = tick;
    } else if (level == 0) {
        dt = tick - t1;
        if (dt < 100 && dt > 0) {
            std::cout << tick << ": Sonar ack, ";
        } else {
            DUMP(dt);
            send_uint32_t(sock, dt);
            send_uint32_t(sock, step);
            srv_ang_conv = servo_ang * 1000;
            send_uint32_t(sock, srv_ang_conv);
            ready = true;
        }
    }
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
    servo.setWidth(650);
    const float servo_start_ang = servo.getAngle();

    int sonar_pulse = pio::pulse_create(pin_sonar, 5, 1);
    int step_pulse = pio::pulse_create(pin_step, 300, 2);

    struct sockaddr_in serv_addr;
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
    bzero((char *)&serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
    serv_addr.sin_port = htons(laptop_port);
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
        error("ERROR connecting");

    while (run) {
        gpioSetMode(pin_sonar, PI_OUTPUT);

        gpioWrite(pin_sonar, 0);
        gpioWaveTxSend(sonar_pulse, PI_WAVE_MODE_ONE_SHOT);
        time_sleep(5e-6);
        gpioWrite(pin_sonar, 0);

        ready = false;
        gpioSetMode(pin_sonar, PI_INPUT);
        gpioSetAlertFunc(pin_sonar, sonarAlert);
        while (ready == false) {
            time_sleep(0.01);
        }

        gpioSetAlertFunc(pin_sonar, NULL);

        // gpioWrite(pin_step_dir, 1);
        // gpioWaveTxSend(step_pulse, PI_WAVE_MODE_ONE_SHOT);
        // time_sleep(0.1);
        // wavechain_tx(pin_step, 40000, 1);
        gpioWrite(pin_step, 1);
        time_sleep(0.1);
        gpioWrite(pin_step, 0);
        // gpioServo(pin_step, 1500);
        // gpioServo(pin_step, )
        if (step + step_inc > 100 || step + step_inc < 0) {
            step_inc *= -1;
            dir = !dir;
            gpioWrite(pin_step_dir, dir);
            servo.setAngle(servo.getAngle() + 0.1);
            servo_ang = servo.getAngle() - servo_start_ang;
        }

        step += step_inc;
    }

    printf("\nlater nerd\n");

    servo.setPosition(0);

    gpioTerminate();

    return 0;
}
