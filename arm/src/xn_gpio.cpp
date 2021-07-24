#include "xn_gpio.hpp"

namespace xn {

int PigDaemon::pi = 0;

namespace pio {

ServoAngular::ServoAngular(int ctl_pin, int min_width, int max_width, float min_angle,
                           float max_angle) {
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
    position_normalized = (w - min_width) / (max_width - min_width);
    cur_angle = min_angle + position_normalized * (max_angle - min_angle);

#ifndef PIO_VIRTUAL
    return PigDaemon::servo(ctl_pin, cur_width);
#endif
    return 0;
}

int ServoAngular::setPosition(float t) {
    position_normalized = t;
    cur_width = min_width + (max_width - min_width) * t;
    cur_angle = min_angle + (max_angle - min_angle) * t;
#ifndef PIO_VIRTUAL
    return PigDaemon::servo(ctl_pin, cur_width);
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
    cur_width = min_width + (max_width - min_width) * position_normalized;
#ifndef PIO_VIRTUAL
    return PigDaemon::servo(ctl_pin, cur_width);
#endif
    return 0;
}

SmoothServo::SmoothServo(ServoAngular servo, vec3 axis, vec3 position, int mid_width, float t_max) {
    this->servo = ServoAngular(servo.ctl_pin, servo.min_width, servo.max_width);
    if (mid_width == 0)
        this->mid_offset = 0;
    else
        this->mid_offset = mid_width - (servo.min_width + servo.max_width) / 2;
    this->axis = axis;
    this->position = position;
    this->t_max = t_max;

    this->target_angle = M_PI / 2;
    this->curve = BezierCurve();
    this->target_prev = this->target_angle;
    this->prev_reached_target = this->target_angle;
    this->t = 0;
}

void SmoothServo::update(float dt) {
    targetw = ang_to_width_corrected(target_angle);
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
    float nt_max =
        t_max * abs(targetw - prev_reached_targetw) / (float)(servo.max_width - servo.min_width);
    float nt = t / nt_max;
    if (nt > 1)
        nt = 1;
    float y = curve.solve(nt).y;

    int w = clamp<int>(0.5 + prev_reached_targetw * (1 - y) + targetw * y, servo.min_width,
                       servo.max_width);
    servo.setWidth(w);
    target_prev = target_angle;
    targetw_prev = targetw;
}

int SmoothServo::ang_to_width_corrected(float ang) {
    float pos = (ang - servo.min_angle) / (servo.max_angle - servo.min_angle);
    return mid_offset + servo.min_width + (servo.max_width - servo.min_width) * pos;
}

} // namespace pio
} // namespace xn