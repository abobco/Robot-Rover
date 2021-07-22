/*
  servo info:

  DS04-NFC
    40.8*20*39.5mm
    1.6*0.8*1.6inch

  HITEC HS311
    - 575-2460μsec width
    - takes ~0.67 seconds to move from min to max angle
    - 900-2100 ms pulse width on the data sheet
    - 1050 idle width for continuous modification

  MG995 RC servo
    - 500-2500μsec width
    - 0.2 s/60 degree turn
    - 4.0-7.2 volts

  arducam pins:
    Chip select:
    - CAM_CS1         17
    I2C:
    - sda_port        2
    - scl_port        3
    SPI:
    - MISO 9
    - MOSI 10
    - SCLK 11
    - SPI0 0 alt fxn

   servo pins:
    12, 13
    26, 16, 6

    servo mid widths:
      - 13 : 1460
      - 26 : 1570
      - 16 : 1460

    arm maff:
        Claw + hitec : 183 g
        Hi wonder + metal brackets: 114 g
        Mg995 + print bracket: 88 g
        Hi wonder + metal brackets + O: 138g

        total: 523 gT

        joints lengths:
            18, 6.9, 6.9, 7.5, 10

            total: 49.

        35 kg*cm / 39 cm =  0.89 kg
        35 kg*cm / 49 cm =  0.71 kg

        20 kg*cm / 39 cm =  0.51 kg
        20 kg*cm / 49 cm =  0.40 kg

    stepper motors:
        - open loop systems: no error correction built in
        - can miss steps
        - constant current draw
        - better at holding positions than servos

    rotary encoders:
        - PPR: Pulses Per Revolution. 360 / PPR = degrees per step
        - incremental encoder(quadrature, relative)
            - 2 sensors(input),
            - 2 identical output pulses
            - output pulses become out of phase proportional to the shaft
  rotation
            - rotation direction decided from which pulse comes first

    nema 17 motor(m1):
        - 1.8 deg / step
        - 2 phase
        - bipolar
        - 3.17 kg-cm hold
        - 200 g-cm detent

*/

#pragma once

#include "xn_vec.hpp"
#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <mutex>
#include <signal.h>
#include <thread>

#ifndef PIO_VIRTUAL
extern "C" {
#include <ArduCAM.h>
#include <pigpiod_if2.h>
#include <sccb_bus.h>
}
#endif

#ifndef XN_I2C_PRINT_ERRORS
#define XN_I2C_PRINT_ERRORS 0
#endif

namespace xn {
class BezierCurve {
public:
  vec2 points[4];

  BezierCurve() {
    points[0] = {0, 0};
    points[1] = {0.5, 0};
    points[2] = {0.5, 1};
    points[3] = {1, 1};
  }

  BezierCurve(vec2 points[]) {
    for (int i = 0; i < 4; i++)
      this->points[i] = points[i];
  }

  inline vec2 solve(float t) {
    return points[0] * (1 - t) * (1 - t) * (1 - t) +
           points[1] * (1 - t) * (1 - t) * t * 3 +
           points[2] * (1 - t) * t * t * 3 + points[3] * t * t * t;
  }
};

namespace pio {
typedef std::chrono::high_resolution_clock::time_point TimePoint;

TimePoint get_time() { return std::chrono::high_resolution_clock::now(); }

double time_diff_seconds(const TimePoint &t1, const TimePoint &t2) {
  return std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
      .count();
}

double time_diff_seconds(const TimePoint &t1) {
  return std::chrono::duration_cast<std::chrono::duration<double>>(
             pio::get_time() - t1)
      .count();
}

#ifndef PIO_VIRTUAL
struct PigDaemon {
  static int pi;
  static void init() { pi = pigpio_start(NULL, NULL); }
  static void stop() { pigpio_stop(pi); }

  static int error_check(int r) {
    if (r < 0 && XN_I2C_PRINT_ERRORS) {
      std::cout << "[PIGD ERROR] " << pigpio_error(r) << '\n';
    }
    return r;
  }

  static int i2cOpen(unsigned bus, unsigned addr) {
    return error_check(i2c_open(pi, bus, addr, 0));
  }
  static int i2cClose(unsigned handle) {
    return error_check(i2c_close(pi, handle));
  }

  static char i2cReadByte(int h) { return error_check(i2c_read_byte(pi, h)); }

  static int i2cRead(int h, char *buf, int n) {
    // uint32_t left = n;
    // int rc;
    // do {
    //     rc = error_check(i2c_read_device(pi, h, buf, left));
    //     if (rc < 0)
    //         continue;
    //     buf += rc;
    //     left -= rc;
    // } while (left > 0);
    return error_check(i2c_read_device(pi, h, buf, n));
  }

  static int i2cWrite(int h, char *buf, int n) {
    return error_check(i2c_write_device(pi, h, buf, n));
  }

  static int servo(unsigned gpio, unsigned pulsewidth) {
    return error_check(set_servo_pulsewidth(pi, gpio, pulsewidth));
  }

  static int serialOpen(char *ser_tty, unsigned baud) {
    return error_check(serial_open(pi, ser_tty, baud, NULL));
  }

  static int serialClose(int h) { return error_check(serial_close(pi, h)); }

  static int serialRead(int h, char *buf, unsigned count) {
    return error_check(serial_read(pi, h, buf, count));
  }

  static int serialWrite(int h, char *buf, unsigned count) {
    return error_check(serial_write(pi, h, buf, count));
  }

  static int serialDataAvailable(int h) {
    return error_check(serial_data_available(pi, h));
  }
};

int i2c_read_buf(int h, char *buf, int n, const bool &loopcond = true) {
  uint32_t left = n;
  int rc;
  do {
    int rc = i2cReadDevice(h, buf, left);
    if (rc < 0)
      return -1;
    buf += rc;
    left -= rc;

    time_sleep(0.01);
  } while (loopcond && left > 0);
  return rc;
}

int i2c_write_buf(int h, char *buf, int n) { return i2cWriteDevice(h, buf, n); }

// longitudinal redundancy check for i2c data
uint8_t checksum_LRC(uint8_t *buf, unsigned n) {
  int LRC = 0;
  for (int i = 0; i < n; i++) {
    LRC -= buf[i];
  }
  return (uint8_t)LRC;
}

/**
 * frame format:
 *  [begin_frame_byte:1] [buf:n] [checksum:1]
 *  if checksum in the frame != checksum_LRC(buf, n)
 *      returns -1
 *  else
 *      returns 1
 * */
int i2c_read_frame(int h, char *buf, int n, const char begin_frame_byte = '?') {
  char *frame = new char[n + 2];

  int rb = PigDaemon::i2cRead(h, frame, n + 2);

  while (frame[0] != begin_frame_byte) {
    for (int i = 0; i < (n + 1); i++) {
      frame[i] = frame[i + 1];
    }
    frame[n + 1] = PigDaemon::i2cReadByte(h);
    // time_sleep(0.005);
    // std::cout << "invalid packet: " << frame << '\n';
  }

  uint8_t rx_checksum = frame[n + 1];
  uint8_t calc_checksum = checksum_LRC((uint8_t *)frame, n + 1);
  if (rx_checksum == calc_checksum) {
    std::memcpy(buf, &frame[1], n);
  } else {
    // std::cout << "failed checksum: " << rx_checksum << " : " << calc_checksum
    // << '\n';
    delete[] frame;
    return -1;
  }

  delete[] frame;
  return 1;
}

// create pulse w/ usDelay
int pulse_create(int pin, int usHigh, int numPulses = 2, int usLow = -1) {
  // gpioWaveClear();
  if (usLow == -1) {
    usLow = usHigh;
  }
  gpioPulse_t pulse[numPulses * 2];
  for (int i = 0; i < numPulses * 2; i += 2) {
    pulse[i].gpioOn = (1 << pin);
    pulse[i].gpioOff = 0;
    pulse[i].usDelay = usHigh;

    pulse[i + 1].gpioOn = 0;
    pulse[i + 1].gpioOff = (1 << pin);
    pulse[i + 1].usDelay = usLow;
  }

  gpioWaveAddNew();
  gpioWaveAddGeneric(numPulses * 2, pulse);
  return gpioWaveCreate();
}

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
                int mode = RED_MODE_DETENT, int step = 0) {
    this->pin_a = pin_a;
    this->pin_b = pin_b;
    this->callback = callback;
    this->step = step;

    lev_a = 0;
    lev_b = 0;
    glitch = 1000;

    gpioSetMode(pin_a, PI_INPUT);
    gpioSetMode(pin_b, PI_INPUT);

    // need pull up b/c enncoder common is grounded
    gpioSetPullUpDown(pin_a, PI_PUD_UP);
    gpioSetPullUpDown(pin_b, PI_PUD_UP);

    gpioGlitchFilter(pin_a, glitch);
    gpioGlitchFilter(pin_b, glitch);

    state_old = (gpioRead(pin_a) << 1) | gpioRead(pin_b);

    cb_id_a = gpioSetAlertFuncEx(pin_a, RotaryEncoder::callback_internal, this);
    cb_id_a = gpioSetAlertFuncEx(pin_b, RotaryEncoder::callback_internal, this);
  }

private:
  static void callback_internal(int gpio, int level, uint32_t tick,
                                void *userdata) {
    RotaryEncoder *obj = (RotaryEncoder *)userdata;
    int state_new, inc, detent;

    if (level != PI_TIMEOUT) {
      if (gpio == obj->pin_a)
        obj->lev_a = level;
      else
        obj->lev_b = level;

      state_new = obj->lev_a << 1 | obj->lev_b;
      inc = transits[obj->state_old << 2 | state_new];

      if (inc) {
        obj->state_old = state_new;
        detent = obj->step / 4;

        obj->step += inc;

        if (obj->callback) {
          if (obj->mode == RED_MODE_DETENT) {
            if (detent != obj->step / 4)
              obj->callback(obj->step / 4);
          } else
            obj->callback(obj->step);
        }
      }
    }
  }
};
/*

             +---------+         +---------+      0
             |         |         |         |
   A         |         |         |         |
             |         |         |         |
   +---------+         +---------+         +----- 1

       +---------+         +---------+            0
       |         |         |         |
   B   |         |         |         |
       |         |         |         |
   ----+         +---------+         +---------+  1

*/
const int RotaryEncoder::transits[] = {
    /* 0000 0001 0010 0011 0100 0101 0110 0111 */
    0, -1, 1, 0, 1, 0, 0, -1,
    /* 1000 1001 1010 1011 1100 1101 1110 1111 */
    -1, 0, 0, 1, 0, 1, -1, 0};

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
          int pin_ms1 = -1, int pin_ms2 = -1, int pin_ms3 = -1) {
    this->pin_dir = pin_dir;
    this->pin_step = pin_step;
    this->pin_ms1 = pin_ms1;
    this->pin_ms2 = pin_ms2;
    this->pin_ms3 = pin_ms3;

    gpioSetMode(pin_dir, PI_OUTPUT);
    gpioSetMode(pin_step, PI_OUTPUT);
    setDutyCycle(dutycycle);
    setFreq(freq);
  }

  void setDutyCycle(float ds) {
    dutycycle = ds;
    gpioPWM(pin_step, dutycycle_range * ds);
  }

  void setDutyCycleRange(unsigned dsr) {
    dutycycle_range = dsr;
    gpioSetPWMrange(pin_step, dsr);
  }

  void setDir(enum StepDir dir) {
    this->dir = dir;
    gpioWrite(pin_dir, dir);
  }

  void setFreq(unsigned f) {
    freq = f;
    gpioSetPWMfrequency(pin_step, f);
  }

  /*
      This waits for step_duration, should probably happen in a separate thread
  TODO:
      - async callback alternative (callback is gpioWrite(pin_step, 0))
      - disable pwm for class users during the wait
      - queue of step instructions?
  */
  void *singleStep_thread(void *argv) {
    setDutyCycle(0);
    gpioWrite(pin_step, 1);
    time_sleep(step_duration);
    gpioWrite(pin_step, 0);
    time_sleep(step_duration);
    return NULL;
  }
};
#endif // #ifndef PIO_VIRTUAL
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
               float min_angle = 0, float max_angle = M_PI) {
    this->ctl_pin = ctl_pin;
    this->min_width = min_width;
    this->max_width = max_width;
    this->min_angle = min_angle;
    this->max_angle = max_angle;

    cur_width = (min_width + max_width) / 2;
    position_normalized = 0.5f;
    cur_angle = (min_angle + max_angle) / 2;
  }

  virtual int setWidth(int w) {
    cur_width = w;
    position_normalized = (w - min_width) / (max_width - min_width);
    cur_angle = min_angle + position_normalized * (max_angle - min_angle);

#ifndef PIO_VIRTUAL
    return PigDaemon::servo(ctl_pin, cur_width);
#endif
    return 0;
  }

  virtual int setPosition(float t) {
    position_normalized = t;
    cur_width = min_width + (max_width - min_width) * t;
    cur_angle = min_angle + (max_angle - min_angle) * t;
#ifndef PIO_VIRTUAL
    return PigDaemon::servo(ctl_pin, cur_width);
#endif
    return 0;
  }

  virtual int setAngle(float a) {
#ifdef PIO_VIRTUAL
    if (a > max_angle || a < min_angle) {

      printf("\nangle %f outside range! (%f, %f)\n", a, min_angle, max_angle);
      return -1;
    }
#endif
    cur_angle = a;
    position_normalized = (a - min_angle) / (max_angle - min_angle);
    cur_width = min_width + (max_width - min_width) * position_normalized;
#ifndef PIO_VIRTUAL
    return PigDaemon::servo(ctl_pin, cur_width);
#endif
    return 0;
  }

  virtual int moveAngle(float a) { return setAngle(cur_angle + a); }
  virtual int movePosition(float a) {
    return setPosition(position_normalized + a);
  }
  virtual int moveWidth(int w) { return setWidth(cur_width + w); }

  int getWidth() const { return cur_width; }
  float getPosition() const { return position_normalized; }
  float getAngle() const { return cur_angle; }
        return 0;
    }

    virtual int moveAngle(float a) { return setAngle(cur_angle + a); }
    virtual int movePosition(float a) { return setPosition(position_normalized + a); }
    virtual int moveWidth(int w) { return setWidth(cur_width + w); }

    int getWidth() const { return cur_width; }
    float getPosition() const { return position_normalized; }
    float getAngle() const { return cur_angle; }
};

class SmoothServo {
};

class ServoContinuous : public ServoAngular {
public:
  int idle_width;

  ServoContinuous(int ctl_pin, int min_width, int max_width, int idle_width)
      : ServoAngular(ctl_pin, min_width, max_width) {
    this->idle_width = idle_width;

    cur_width = idle_width;
#ifndef PIO_VIRTUAL
    gpioServo(ctl_pin, cur_width);
#endif
  }

  int setWidth(int w) {
    cur_width = w;

    int neg_range = idle_width - min_width;
    int pos_range = max_width - idle_width;

    if (w < idle_width) {
      position_normalized = (cur_width - idle_width) / neg_range;
    } else if (w > idle_width) {
      position_normalized = (cur_width - idle_width) / pos_range;
    } else {
      position_normalized = 0;
    }
#ifndef PIO_VIRTUAL
    return gpioServo(ctl_pin, w);
#endif
    return 0;
  }

  int setWidthFromCenter(int offset_w) {
    return setWidth(idle_width + offset_w);
  }

  int setPosition(float t) {
    position_normalized = t;

    int neg_range = idle_width - min_width;
    int pos_range = max_width - idle_width;

    if (t < 0) {
      t = 1 + t;
      cur_width = min_width + neg_range * t;
    } else if (t > 0) {
      cur_width = idle_width + pos_range * t;
    } else {
      cur_width = idle_width;
    }
#ifndef PIO_VIRTUAL
    return gpioServo(ctl_pin, cur_width);
#endif
    return 0;
  }
};

#ifndef PIO_VIRTUAL
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
#endif
} // namespace pio
} // namespace xn
