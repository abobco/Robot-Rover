#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#include <iostream>

extern "C" {
#include <pigpio.h>
}


#include "XNlib/xn_gpio.hpp"

/*
sudo ./servo360        # Send servo pulses to GPIO 12
*/

int run=1;

void stop(int signum)
{
   run = 0;
}

int main(int argc, char *argv[])
{
   if (gpioInitialise() < 0) return -1;
   gpioSetSignalFunc(SIGINT, stop);

   
   int servo_pin = 12;

   if ( argc > 1 ) {
      servo_pin = atoi(argv[1]);
   }

   auto hitec_hs311 = xn::pio::ServoContinuous(servo_pin, 575, 2460, 1050);

   while(run) {
      float t;
      printf("Enter position for %d: ", servo_pin);
      std::cin >> t;
      hitec_hs311.setPosition(t);
   }

   printf("\nlater nerd\n");

   hitec_hs311.setPosition(0);

   gpioTerminate();

   return 0;
}

