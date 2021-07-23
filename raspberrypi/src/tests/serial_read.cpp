#include "XNlib/xn_gpio.hpp"
#include "XNlib/xn_vec.hpp"

using namespace xn;
int pio::PigDaemon::pi = 0;
int main() {
    // if (gpioInitialise() < 0)
    //     return -1;

    // gpioServo(5, 2300);
    pio::PigDaemon::init();

    int h = pio::PigDaemon::serialOpen("/dev/serial0", 115200);

    while (true) {
        int n = pio::PigDaemon::serialDataAvailable(h);

        if (n > 0) {
            char *buf = new char[n + 1];
            pio::PigDaemon::serialRead(h, buf, n);
            buf[n] = 0;
            std::cout << std::string(buf);
        } else {
            time_sleep(0.01);
        }
    }

    pio::PigDaemon::serialClose(h);
}