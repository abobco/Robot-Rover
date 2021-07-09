#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <signal.h>
#include <assert.h> 
#include <string.h>
#include <pthread.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h> 
#include <math.h>

extern "C" {
#include "ws/ws_ring_server.c"
}

#include "XNlib/xn_gpio.hpp"
#include "XNlib/xn_ik.hpp"

#define laptop_port 8080 
#define laptop_ip "192.168.0.183"

/*
  arm dims: 
    total len: 54 cm
*/

bool receiving_commands = true;
xn::ik::ServoChain *armptr;

void  INThandler(int sig);

void *move_servo_thread(void *argv) {
   xn::pio::SmoothServo *s = (xn::pio::SmoothServo*) argv;
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

int main(int argc, char *argv[])
{

  if (gpioInitialise() < 0) return -1;
  gpioSetSignalFunc(SIGINT, INThandler);

  xn::pio::SmoothServo servos_flat[] = {
    xn::pio::SmoothServo(xn::pio::ServoAngular(12, 500, 2500), {0}, {0}, 1500),
    xn::pio::SmoothServo(xn::pio::ServoAngular(13, 500, 2500), {0}, {0}, 1420),
    xn::pio::SmoothServo(xn::pio::ServoAngular(26, 500, 2500), {0}, {0}, 1570),
    xn::pio::SmoothServo(xn::pio::ServoAngular(16, 500, 2500), {0}, {0}, 1420),
    xn::pio::SmoothServo(xn::pio::ServoAngular(6, 500, 2500), {0}, {0}, 1500),
    xn::pio::SmoothServo(xn::pio::ServoAngular(5, 500, 2500), {0}, {0}, 1500),
    // xn::pio::ServoAngular(13, 500, 2500),
    // xn::pio::ServoAngular(26, 500, 2500),
    // xn::pio::ServoAngular(16, 500, 2500),
    // xn::pio::ServoAngular(6, 500, 2500),
    // xn::pio::ServoAngular(5, 500, 2500),
  };

  for ( int i = 0; i < 6; i++ ) {
    // servos[i].servo.setPosition(0.5);
    servos_flat[i].target_angle = M_PI/2;
  }

  servos_flat[2].servo.setAngle(0.1);
  servos_flat[2].target_angle = 0.1;
  servos_flat[3].servo.setAngle(M_PI*0.99);
  servos_flat[3].target_angle = M_PI*0.99;
  servos_flat[4].servo.setAngle(M_PI);
  servos_flat[4].target_angle = M_PI;

  pthread_t tid[6];
  for ( int i = 0; i < 6; i++ )
    pthread_create(&tid[i], NULL, move_servo_thread, &servos_flat[i]);

  // host webapp for manual servo control
  pthread_t websocket_thread; 
  init_msg_q();
  websocket_thread = ws_create_thread("www.studiostudios.net", 3000);

  // start_read_data = 1;
  while(receiving_commands)
  {
    if ( get_connections() ) { 
      MessageList msglist = pop_new_messages();
      if ( msglist.length > 0  ) {
        for ( int i = 0; i < msglist.length; i++ ) {
          // printf("%d: %d\n",msglist.buffer[i].user_id, msglist.buffer[i].value);
           
          xn::pio::SmoothServo& s = servos_flat[msglist.buffer[i].user_id];
          s.target_angle = M_PI * (msglist.buffer[i].value - s.servo.min_width) /(float) (s.servo.max_width - s.servo.min_width);
        }
      }
    }
  }

  pthread_join(websocket_thread, NULL); 
  for (int i = 0; i < 6; i++ ){
    pthread_join(tid[i], NULL);
  }
  printf("\nlater nerd\n");
  return 0;
}

void  INThandler(int sig) {
  signal(sig, SIG_IGN); 
  
  // receiving_commands = false;
  // interrupted = false;
  // time_sleep(0.1);
  // printf("\nmoving servos back to neutral\n");
  // for ( int i = 0; i < 3; i++ )
  //   for ( xn::pio::SmoothServo& s : armptr->servos[i] )
  //     s.target_angle = M_PI/2;
  // // pthread_mutex_unlock(&sem);
  
  // time_sleep(armptr->servos[0][0].t_max/2);	
  run = 0;
  exit(0);
}