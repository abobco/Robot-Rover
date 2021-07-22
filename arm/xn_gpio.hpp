#pragma once
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include "../windows/src/util/xn_math.hpp"
#include "../windows/src/util/xn_vec.hpp"

#include <chrono>
#include <cmath>
#include <ctime>
#include <signal.h>
#include <thread>

#ifdef _WIN32
#ifndef PIO_VIRTUAL
#define PIO_VIRTUAL
#endif
#endif

#ifndef PIO_VIRTUAL
extern "C" {
#include <ArduCAM.h>
#include <pigpio.h>
#include <sccb_bus.h>
}
#endif

namespace xn {

// PigPiod Daemon wrapper
class PigDaemon {
  static int pi;

public:
  static void init();

  static void stop();

  static int i2cOpen(unsigned bus, unsigned addr);

  static int i2cClose(unsigned handle);

  static char i2cReadByte(int h);

  static int i2cRead(int h, char *buf, int n);

  static int i2cWrite(int h, char *buf, int n);

  static int servo(unsigned gpio, unsigned pulsewidth);

  static int serialOpen(char *ser_tty, unsigned baud);

  static int serialClose(int h);

  static int serialRead(int h, char *buf, unsigned count);

  static int serialWrite(int h, char *buf, unsigned count);

  static int serialDataAvailable(int h);
};

namespace pio {

class ServoAngular {

protected:
  int cur_width;
  float cur_angle;
  float position_normalized;

public:
  int ctl_pin;
  int min_width;
  int max_width;

  float min_angle;
  float max_angle;
  float mid_offset;

  ServoAngular() {}

  ServoAngular(int ctl_pin, int min_width = 500, int max_width = 2500,
               float min_angle = 0, float max_angle = M_PI);

  virtual int setWidth(int w);

  virtual int setPosition(float t);

  virtual int setAngle(float a);

  virtual int moveAngle(float a) { return setAngle(cur_angle + a); }
  virtual int movePosition(float a) {
    return setPosition(position_normalized + a);
  }
  virtual int moveWidth(int w) { return setWidth(cur_width + w); }

  int getWidth() { return cur_width; }
  float getPosition() { return position_normalized; }
  float getAngle() { return cur_angle; }
};

class SmoothServo {
public:
  ServoAngular servo;
  BezierCurve curve;
  vec3 axis;
  vec3 position;
  float target_angle = (float)M_PI_2;
  float target_prev = (float)M_PI_2;
  float prev_reached_target = (float)M_PI_2;
  int targetw = 1500;
  int targetw_prev = 1500;
  int prev_reached_targetw = 1500;
  float t;
  float t_max = 5;
  int mid_offset;
  // pthread_t tid;
  // std::thread tid;

  SmoothServo(ServoAngular servo, vec3 axis = {1, 0, 0},
              vec3 position = {0, 0, 0}, int mid_width = 0, float t_max = 10);

  void update(float dt);

  int ang_to_width_corrected(float ang);
};

#ifndef PIO_VIRTUAL

// create gpio pulse w/ usDelay
int pulse_create(int pin, int usHigh, int numPulses = 2, int usLow = -1);

enum RotaryMode { RED_MODE_DETENT, RED_MODE_STEP };

class RotaryEncoder {
private:
  int cb_id_a, cb_id_b;
  static const int transits[16];

public:
  unsigned pin_a, pin_b, lev_a, lev_b;
  int state_old, step, mode, glitch;

  void (*callback)(int);

  RotaryEncoder(unsigned pin_a, unsigned pin_b, void (*callback)(int),
                int mode = RED_MODE_DETENT, int step = 0);

private:
  static void callback_internal(int gpio, int level, uint32_t tick,
                                void *userdata);
};

// const int RotaryEncoder::transits[] =;

enum StepDir { STEP_REVERSE, STEP_FORWARD };

class Stepper {
public:
  int pin_dir, pin_step, pin_ms1, pin_ms2, pin_ms3;
  unsigned freq, dutycycle_range = 255;
  float dutycycle;
  enum StepDir dir = STEP_FORWARD;

  double step_duration = 0.01;

  // pthread_t singlestep_tid;
  // static pthread_mutex_t step_lock;

  Stepper(int pin_dir, int pin_step, float dutycycle = 0.5, unsigned freq = 1,
          int pin_ms1 = -1, int pin_ms2 = -1, int pin_ms3 = -1);

  void setDutyCycle(float ds);

  void setDutyCycleRange(unsigned dsr);

  void setDir(enum StepDir dir);

  void setFreq(unsigned f);

  void *singleStep_thread(void *argv);
};

class Arducam {
private:
  vec2 resolution;

public:
  char *frame_buffer;
  size_t frame_size;

  Arducam() {
    pioInit();
    ArduCAM_CS_init(CAM_CS1, -1, -1, -1); // init the cs

    sccb_bus_init();
    spiInit(4000000, 0); // 8MHZ
    Arducam_bus_detect(CAM_CS1, -1, -1, -1);

    resetFirmware(CAM_CS1, -1, -1, -1); // reset the firmware
    ArduCAM_Init(sensor_model);
  }

  // blocks until picture is read into memory
  void capture() {
    singleCapture(CAM_CS1);

    frame_buffer = readbuf;
    frame_size = length;
  }

  vec2 get_resolution() { return resolution; }

  int setQuality(int quality) {
    if (quality < 0 || quality > 9)
      return -1;

    // i sure as hell didnt write this shit
    if (quality == 0) {
      if (sensor_model == OV2640) {
        resolution = {160, 120};
        OV2640_set_JPEG_size(OV2640_160x120);
        printf("Set the resolution to OV2640_160x120 successfully\r\n");
      } else if (sensor_model == OV5640) {
        OV5640_set_JPEG_size(OV5640_320x240);
        printf("Set the resolution to OV5640_320x240 successfully\r\n");
      } else if (sensor_model == OV5642) {
        OV5642_set_JPEG_size(OV5642_320x240);
        printf("Set the resolution to OV5642_320x240 successfully\r\n");
      }
    } else if (quality == 1) {
      if (sensor_model == OV2640) {
        resolution = {176, 144};
        OV2640_set_JPEG_size(OV2640_176x144);
        printf("Set the resolution to OV2640_176x144 successfully\r\n");

      } else if (sensor_model == OV5640) {
        OV5640_set_JPEG_size(OV5640_352x288);
        printf("Set the resolution to OV5640_352x288 successfully\r\n");
      } else if (sensor_model == OV5642) {
        OV5642_set_JPEG_size(OV5642_640x480);
        printf("Set the resolution to OV5642_640x480 successfully\r\n");
      }
    } else if (quality == 2) {
      if (sensor_model == OV2640) {
        resolution = {320, 240};
        OV2640_set_JPEG_size(OV2640_320x240);
        printf("Set the resolution to OV2640_320x240 successfully\r\n");
      } else if (sensor_model == OV5640) {
        OV5640_set_JPEG_size(OV5640_640x480);
        printf("Set the resolution to OV5640_640x480 successfully\r\n");
      } else if (sensor_model == OV5642) {
        OV5642_set_JPEG_size(OV5642_1024x768);
        printf("Set the resolution to OV5642_1024x768 successfully\r\n");
      }
    } else if (quality == 3) {
      if (sensor_model == OV2640) {
        resolution = {352, 288};
        OV2640_set_JPEG_size(OV2640_352x288);
        printf("Set the resolution to OV2640_352x288 successfully\r\n");
      } else if (sensor_model == OV5640) {
        OV5640_set_JPEG_size(OV5640_800x480);
        printf("Set the resolution to OV5640_800x480 successfully\r\n");
      } else if (sensor_model == OV5642) {
        OV5642_set_JPEG_size(OV5642_1280x960);
        printf("Set the resolution to OV5642_1280x960 successfully\r\n");
      }
    } else if (quality == 4) {
      if (sensor_model == OV2640) {
        resolution = {640, 480};
        OV2640_set_JPEG_size(OV2640_640x480);
        printf("Set the resolution to OV2640_640x480 successfully\r\n");
      } else if (sensor_model == OV5640) {
        OV5640_set_JPEG_size(OV5640_1024x768);
        printf("Set the resolution to OV5640_1024x768 successfully\r\n");
      } else if (sensor_model == OV5642) {
        OV5642_set_JPEG_size(OV5642_1600x1200);
        printf("Set the resolution to OV5642_1600x1200 successfully\r\n");
      }
    } else if (quality == 5) {
      if (sensor_model == OV2640) {
        resolution = {800, 600};
        OV2640_set_JPEG_size(OV2640_800x600);
        printf("Set the resolution to OV2640_800x600 successfully\r\n");
      } else if (sensor_model == OV5640) {
        OV5640_set_JPEG_size(OV5640_1280x960);
        printf("Set the resolution to OV5640_1280x960 successfully\r\n");
      } else if (sensor_model == OV5642) {
        OV5642_set_JPEG_size(OV5642_2048x1536);
        printf("Set the resolution to OV5642_2048x1536 successfully\r\n");
      }
    } else if (quality == 6) {
      if (sensor_model == OV2640) {
        resolution = {1024, 768};
        OV2640_set_JPEG_size(OV2640_1024x768);
        printf("Set the resolution to OV2640_1024x768 successfully\r\n");
      } else if (sensor_model == OV5640) {
        OV5640_set_JPEG_size(OV5640_1600x1200);
        printf("Set the resolution to OV5640_1600x1200 successfully\r\n");
      } else if (sensor_model == OV5642) {
        OV5642_set_JPEG_size(OV5642_2592x1944);
        printf("Set the resolution to OV5642_2592x1944 successfully\r\n");
      }
    } else if (quality == 7) {
      if (sensor_model == OV2640) {
        resolution = {1280, 1024};
        OV2640_set_JPEG_size(OV2640_1280x1024);
        printf("Set the resolution to OV2640_1280x1024 successfully\r\n");
      } else if (sensor_model == OV5640) {
        OV5640_set_JPEG_size(OV5640_2048x1536);
        printf("Set the resolution to OV5640_2048x1536 successfully\r\n");
      }
    } else if (quality == 8) {
      if (sensor_model == OV2640) {
        resolution = {1600, 1200};
        OV2640_set_JPEG_size(OV2640_1600x1200);
        printf("Set the resolution to OV2640_1600x1200 successfully\r\n");
      } else if (sensor_model == OV5640) {
        OV5640_set_JPEG_size(OV5640_2592x1944);
        printf("Set the resolution to OV5640_2592x1944 successfully\r\n");
      }
    }
    return 0;
  }
};

#endif // #ifndef PIO_VIRTUAL

// class ServoContinuous : public ServoAngular {
// public:
//   int idle_width;

//   ServoContinuous(int ctl_pin, int min_width, int max_width, int idle_width)
//       : ServoAngular(ctl_pin, min_width, max_width) {
//     this->idle_width = idle_width;

//     cur_width = idle_width;
// #ifndef PIO_VIRTUAL
//     gpioServo(ctl_pin, cur_width);
// #endif
//   }

//   int setWidth(int w) {
//     cur_width = w;

//     int neg_range = idle_width - min_width;
//     int pos_range = max_width - idle_width;

//     if (w < idle_width) {
//       position_normalized = (float)(cur_width - idle_width) / neg_range;
//     } else if (w > idle_width) {
//       position_normalized = (float)(cur_width - idle_width) / pos_range;
//     } else {
//       position_normalized = 0;
//     }
// #ifndef PIO_VIRTUAL
//     return gpioServo(ctl_pin, w);
// #endif
//     return 0;
//   }

//   int setWidthFromCenter(int offset_w) {
//     return setWidth(idle_width + offset_w);
//   }

//   int setPosition(float t) {
//     position_normalized = t;

//     int neg_range = idle_width - min_width;
//     int pos_range = max_width - idle_width;

//     if (t < 0) {
//       t = 1 + t;
//       cur_width = (int)(min_width + neg_range * t);
//     } else if (t > 0) {
//       cur_width = (int)(idle_width + pos_range * t);
//     } else {
//       cur_width = idle_width;
//     }
// #ifndef PIO_VIRTUAL
//     return gpioServo(ctl_pin, cur_width);
// #endif
//     return 0;
//   }
// };

} // namespace pio
} // namespace xn