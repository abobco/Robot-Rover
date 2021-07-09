#include "XNlib/xn_gpio.hpp"

bool run=1;

void stop(int signum) {
   run = 0;
}

int main (int argc, const char **argv ) {
    if (gpioInitialise() < 0) return -1;
        gpioSetSignalFunc(SIGINT, stop);
    
    int handle = serOpen("/dev/ttyS0", 115200, 0);
    if ( handle < 0 ) {
        printf("error opening serial port\n");
        return -1;
    }

    while(run) {
        char packet[9];
        if ( serDataAvailable(handle) >= 9)
            serRead(handle, packet, 9);

        
        if ( packet[0] == 0x59 && packet[1] == 0x59) {
            int low = packet[2];
            int high = packet[3];
            int d = low + high*256;
            DUMP(d);
        }
    }

}