#include "XNlib/xn_gpio.hpp"

using namespace xn;
int pio::PigDaemon::pi = 0;

bool run = 1;
int pin_a = 2, pin_b = 3;

void cbf(int pos) { printf("%d\n", pos); }

void stop(int signum) { run = 0; }

int main(int argc, const char **argv) {
    if (gpioInitialise() < 0)
        return -1;
    gpioSetSignalFunc(SIGINT, stop);

    pio::RotaryEncoder red(pin_a, pin_b, cbf);
    while (run) {
        time_sleep(0.1);
    }

    gpioTerminate();
    return 0;
}