// #define PIO_VIRTUAL

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#include <iostream>
#include <pthread.h>
#include <thread>

#include "XNlib/xn_gpio.hpp"

/*
sudo ./servo_demo          # Send servo pulses to GPIO 12
sudo ./servo_demo 23 24 25 # Send servo pulses to GPIO 23, 24, 25.

HITEC HS311 servo notes:
   - takes ~0.67 seconds to move from min to max angle 
   - 575-2460 ms pulse width on the data sheet 

servo pins:
   12, 13
   26, 16, 6

arm servo mid widths:
- 13 : 1460
- 26 : 1570
- 16 : 1460
*/

using namespace xn;
int run=1;

float pos=0.5;
pthread_mutex_t sem;

pio::SmoothServo *servo_ptr;

void *move_servo_thread(void *argv) {
   pio::SmoothServo *s = (pio::SmoothServo*) argv;

   while ( run ) {
      pthread_mutex_lock(&sem);
      // s.setPosition(s.getPosition()*0.98 + pos*0.02);
      s->update(1.0/60);
      pthread_mutex_unlock(&sem);

      time_sleep(1/60.0);
   }
   return NULL;
}

void stop(int signum)
{
   printf("\nmoving servos back to neutral\n");
   pthread_mutex_lock(&sem);
   servo_ptr->target_angle = M_PI/2;
   pthread_mutex_unlock(&sem);
   
   time_sleep(servo_ptr->t_max/2);
   run = 0;
}

int main(int argc, char *argv[])
{
   if (gpioInitialise() < 0) return -1;
   gpioSetSignalFunc(SIGINT, stop);

   int servo_pin = 12;
   int mid_width = 1500;
   if ( argc > 1 ) {
      servo_pin = atoi(argv[1]);
   }
   
   if (argc > 2 ) {
      mid_width = atoi(argv[2]);
   }

   pio::SmoothServo servo(pio::ServoAngular(servo_pin, 500, 2500), {0}, {0}, mid_width);
   servo_ptr = &servo;
   pthread_t tid;

    if (pthread_mutex_init(&sem, NULL) != 0) { 
        printf("\n mutex init has failed\n"); 
        return -1; 
    }

   pthread_create(&tid, NULL, move_servo_thread, &servo);

   while(run)
   {
      float t = 0.5;
      printf("Enter pos for %d: ", servo_pin);
      std::cin >> t;
      pthread_mutex_lock(&sem);
      servo.target_angle = M_PI*t;
      pthread_mutex_unlock(&sem);
   }

   printf("\nlater nerd\n");
   pthread_join(tid, NULL);

   gpioTerminate();

   return 0;
}

