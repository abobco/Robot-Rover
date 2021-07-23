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
   - 13 : 1420
   - 26 : 1570
   - 16 : 1440
*/

using namespace xn;
int run=1;
static const int num_servos = 4;

pio::SmoothServo servos[] = { 
   pio::SmoothServo(pio::ServoAngular(13, 500, 2500), {0}, {0}, 1600),
   pio::SmoothServo(pio::ServoAngular(26, 500, 2500), {0}, {0}, 1570),
   pio::SmoothServo(pio::ServoAngular(16, 500, 2500), {0}, {0}, 1500),
   pio::SmoothServo(pio::ServoAngular( 6, 500, 2500), {0}, {0}, 1500),
};

float pos=0.5;
pthread_mutex_t sem;

void *move_servo_thread(void *argv) {
   pio::SmoothServo *s = (pio::SmoothServo*) argv;
   double step = 1.0/120;
   while ( run ) {
      // pthread_mutex_lock(&sem);
      // s.setPosition(s.getPosition()*0.98 + pos*0.02);
      s->update(step);
      // pthread_mutex_unlock(&sem);

      time_sleep(step);
   }
   return NULL;
}

void stop(int signum)
{
   printf("\nmoving servos back to neutral\n");
   pthread_mutex_lock(&sem);
   for ( int i = 0; i < num_servos; i++ )
      servos[i].target_angle = M_PI/2;
   
   // servos[2].target_angle = 0;

   // servos[0].servo.setPosition(0.5);
   // servos[1].servo.setPosition(0);
   servos[1].target_angle = 0.1;
   servos[2].target_angle = M_PI*0.99;
   servos[3].target_angle = 0.1;
   pthread_mutex_unlock(&sem);
   
   time_sleep(servos[0].t_max/2);
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

   for ( int i = 0; i < num_servos; i++ ) {
      // servos[i].servo.setPosition(0.5);
      servos[i].target_angle = M_PI/2;
   }

   servos[0].servo.setPosition(0.5);
   servos[1].servo.setAngle(0.1);
   servos[1].target_angle = 0.1;
   servos[2].servo.setAngle(M_PI*0.99);
   servos[2].target_angle = M_PI*0.99;
   servos[3].servo.setAngle(0.1);
   servos[3].target_angle = 0.1;

   pthread_t tid[num_servos];

    if (pthread_mutex_init(&sem, NULL) != 0) { 
        printf("\n mutex init has failed\n"); 
        return -1; 
    }

   for ( int i = 0; i < num_servos; i++ )
      pthread_create(&tid[i], NULL, move_servo_thread, &servos[i]);


   float x_min = 0, x_max = 1.0, y_min = 0.5, y_max = 1.0;
   float x_step = 0.005;
   float y_step = 0.005;

   while(run)
   {
      int pin = 0;
      float t = 0.5;
      printf("Enter a pin and angle \n");
      std::cin >> pin >> t;
      pthread_mutex_lock(&sem);

      bool found = false;
      for ( int i = 0; i < num_servos; i++ )
         if (servos[i].servo.ctl_pin == pin) {
            servos[i].target_angle = t;
         }
      

      float pt = 0;
      for ( int i = 0; i < 3; i ++) {
         float fac = servos[i].target_angle - M_PI/2;
         if ( i != 2 ) fac *= -1;
         pt += fac;
      }
      DUMP(pt);
      servos[num_servos - 1].target_angle = M_PI - pt;
      pthread_mutex_unlock(&sem);
   }

   printf("\nlater nerd\n");

   // hitec_hs311.setPosition(0.5);
   for ( int i = 0; i < num_servos; i++ )
      pthread_join(tid[i], NULL);

   gpioTerminate();

   return 0;
}

