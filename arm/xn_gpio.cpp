#include "xn_gpio.hpp"

namespace xn {

namespace pio {

ServoAngular::ServoAngular(int ctl_pin, int min_width, int max_width,
                           float min_angle, float max_angle) {
  this->ctl_pin = ctl_pin;
  this->min_width = min_width;
  this->max_width = max_width;
  this->min_angle = min_angle;
  this->max_angle = max_angle;

  cur_width = (min_width + max_width) / 2;
  position_normalized = 0.5f;
  cur_angle = (min_angle + max_angle) / 2;
}

int ServoAngular::setWidth(int w) {
  cur_width = w;
  position_normalized = (float)(w - min_width) / (max_width - min_width);
  cur_angle = min_angle + position_normalized * (max_angle - min_angle);

#ifndef PIO_VIRTUAL
  return gpioServo(ctl_pin, cur_width);
#endif
  return 0;
}

int ServoAngular::setPosition(float t) {
  position_normalized = t;
  cur_width = (int)(min_width + (max_width - min_width) * t);
  cur_angle = min_angle + (max_angle - min_angle) * t;
#ifndef PIO_VIRTUAL
  return gpioServo(ctl_pin, cur_width);
#endif
  return 0;
}

int ServoAngular::setAngle(float a) {
#ifdef PIO_VIRTUAL
  if (a > max_angle || a < min_angle) {

    printf("\nangle %f outside range! (%f, %f)\n", a, min_angle, max_angle);
    return -1;
  }
#endif
  cur_angle = a;
  position_normalized = (a - min_angle) / (max_angle - min_angle);
  cur_width = (int)(min_width + (max_width - min_width) * position_normalized);
#ifndef PIO_VIRTUAL
  return gpioServo(ctl_pin, cur_width);
#endif
  return 0;
}

SmoothServo::SmoothServo(ServoAngular servo, vec3 axis, vec3 position,
                         int mid_width, float t_max) {
  this->servo = ServoAngular(servo.ctl_pin, servo.min_width, servo.max_width);
  if (mid_width == 0)
    this->mid_offset = 0;
  else
    this->mid_offset = mid_width - (servo.min_width + servo.max_width) / 2;
  this->axis = axis;
  this->position = position;
  this->t_max = t_max;

  this->target_angle = (float)M_PI_2;
  this->curve = BezierCurve();
  this->target_prev = this->target_angle;
  this->prev_reached_target = this->target_angle;
  this->t = 0;
}

void SmoothServo::update(float dt) {
  targetw = ang_to_width_corrected(target_angle);
  // if ( fabs(target_angle - servo.getAngle()) < SMEPSILON ) {
  //     target_prev = target_angle;
  //     prev_reached_target = target_angle;
  //     t = 0;
  //     return;
  // } else if (fabs(target_angle - target_prev) > SMEPSILON ) {
  //     prev_reached_target = servo.getAngle();
  //     t = 0;
  // }
  if (servo.getWidth() == targetw) {
    targetw_prev = targetw;
    prev_reached_targetw = targetw;
    t = 0;
    return;
  } else if (targetw != targetw_prev) {
    prev_reached_targetw = servo.getWidth();
    t = 0;
  }

  t += dt;
  float nt_max = t_max * abs(targetw - prev_reached_targetw) /
                 (float)(servo.max_width - servo.min_width);
  float nt = t / nt_max;
  // nt = clamp(nt, 0, 1);
  if (nt > 1)
    nt = 1;
  float y = curve.solve(nt).y;

  // DUMP(servo.ctl_pin);
  // DUMP(y);
  // DUMP(target_prev*(1-y) + target_angle*y);
  // float ang = prev_reached_target*(1-y) + target_angle*y;
  // float pos = (ang - servo.min_angle) / (servo.max_angle -
  // servo.min_angle); int w = mid_offset + servo.min_width + (servo.max_width
  // - servo.min_width) * pos;
  int w = clamp<int>((int)(0.5 + prev_reached_targetw * (1 - y) + targetw * y),
                     servo.min_width, servo.max_width);
  // printf("\n%d : %d", servo.ctl_pin, w );
  // DUMP(w);
  servo.setWidth(w);
  // servo.setAngle(prev_reached_target*(1-y) + target_angle*y);
  target_prev = target_angle;
  targetw_prev = targetw;
}

int SmoothServo::ang_to_width_corrected(float ang) {
  float pos = (ang - servo.min_angle) / (servo.max_angle - servo.min_angle);
  return (int)(mid_offset + servo.min_width +
               (servo.max_width - servo.min_width) * pos);
}

} // namespace pio
} // namespace xn