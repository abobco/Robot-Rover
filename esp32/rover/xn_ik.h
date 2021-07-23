#pragma once

#include <ESP32Servo.h>

#include <cmath>
#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>


#define SMEPSILON 1E-16f
#define ORTH_TOLERANCE 1E-3

#define DUMP(a)                                                                \
  { std::cout << #a " = " << (a) << std::endl; }

namespace xn {
struct vec3 {
  float x;
  float y;
  float z;

  static inline float dist_sqr(const vec3 &l, const vec3 &r) {
    float x = l.x - r.x;
    float y = l.y - r.y;
    float z = l.z - r.z;
    // printf("d=(%f, %f, %f)\n", x,y,z);
    return x * x + y * y + z * z;
  }

  static inline float dist(const vec3 &l, const vec3 &r) {
    return sqrt(dist_sqr(l, r));
  }

  static inline float dot(const vec3 &l, const vec3 &r) {
    return l.x * r.x + l.y * r.y + l.z * r.z;
  }

  static inline vec3 cross(const vec3 &l, const vec3 &r) {
    return {l.y * r.z - l.z * r.y, l.z * r.x - l.x * r.z,
            l.x * r.y - l.y * r.x};
  }

  static inline vec3 rotate_axis(vec3 point, vec3 axis, float angle) {
    float cos_ang = cos(angle);
    float sin_ang = sin(angle);

    return point * cos_ang + cross(axis, point) * sin_ang +
           axis * dot(axis, point) * (1 - cos_ang);
  }

  inline float mag() { return dist(*this, {0, 0, 0}); }

  inline float mag_sqr() { return dist_sqr(*this, {0, 0, 0}); }

  inline void normalize() { *this = *this / mag(); }

  inline vec3 abs() { return {fabs(x), fabs(y), fabs(z)}; }

  inline vec3 operator+(const vec3 &r) {
    return {this->x + r.x, this->y + r.y, this->z + r.z};
  }

  inline vec3 operator-(const vec3 &r) {
    return {this->x - r.x, this->y - r.y, this->z - r.z};
  }

  inline vec3 operator*(const vec3 &r) {
    return {this->x * r.x, this->y * r.y, this->z * r.z};
  }

  inline vec3 operator*(const float &r) {
    return {this->x * r, this->y * r, this->z * r};
  }

  inline vec3 operator/(vec3 &r) {
    if (fabs(r.x) < SMEPSILON || fabs(r.y) < SMEPSILON ||
        fabs(r.z) < SMEPSILON) {
      printf("attempted divide by zero!\n");
      std::cout << toString() << " / " << r.toString() << '\n';
      return *this;
    }

    return {this->x / r.x, this->y / r.y, this->z / r.z};
  }

  inline vec3 operator/(const float &r) {
    // printf("%f\n", fabs(r));
    if (fabs(r) < SMEPSILON) {
      printf("attempted divide by zero!\n");
      std::cout << toString() << " / " << r << '\n';
      // return *this;
    }

    float inv = 1 / r;
    return {this->x * inv, this->y * inv, this->z * inv};
  }

  void print() { printf("(%f, %f, %f)\n", x, y, z); }

  std::string toString() {
    char buf[256];
    int n = sprintf(buf, "(%f, %f, %f)", x, y, z);
    return std::string(buf);
  }
};

std::ostream &operator<<(std::ostream &os, const vec3 &r) {
  return os << "(" << r.x << ", " << r.y << ", " << r.z << ")";
}

struct Transform {
  struct Transform *parent;
  vec3 position;
  vec3 eulers;
  vec3 scale;

  Transform() {
    parent = NULL;
    position = {0, 0, 0};
    eulers = {0, 0, 0};
    scale = {1, 1, 1};
  }

  Transform(vec3 _position, vec3 _eulers = {0, 0, 0}, vec3 _scale = {1, 1, 1}) {
    parent = NULL;
    position = _position;
    eulers = _eulers;
    scale = _scale;
  }
};

float clamp(float n, float minval, float maxval) {
  return std::max(minval, std::min(n, maxval));
}
namespace ik {
class IkChain {
public:
  int bone_count;
  int iterations;
  // Transform *target;
  float delta = 0.001;

  float *bone_lengths;
  float chain_length = 0;
  Transform *bones;
  vec3 *positions;
  vec3 pole;

  //            vec3 *child_start_directions;
  //            Quaternion *bone_start_rotations;
  // Quaternion target_start_rotation;
  //            Quaternion root_start_rotation;

  IkChain() {}

  IkChain(int _bone_count, Transform ordered_bones[], vec3 &_pole,
          int _iterations = 10) {
    bone_count = _bone_count;
    iterations = _iterations;
    pole = _pole;

    bones = new Transform[bone_count + 1];
    bone_lengths = new float[bone_count + 1];
    positions = new vec3[bone_count + 1];
    //                child_start_directions = new vec3[bone_count+1];
    //                bone_start_rotations = new Quaternion[bone_count +1];

    // target_start_rotation = target.rotation;

    for (int i = 0; i < bone_count; i++) {
      bones[i] = ordered_bones[i];
    }
    for (int i = 1; i < bone_count; i++) {
      bones[i].parent = &bones[i - 1];
    }

    Transform current = bones[bone_count - 2];
    for (int i = 0; i < bone_count; i++) {
      bone_lengths[i] = 0;
    }

    for (int i = bone_count - 2; i >= 0; i--) {
      //                    bone_start_rotations[i] = current.rotation;
      bone_lengths[i] = vec3::dist(bones[i + 1].position, bones[i].position);
      // printf("bone_lengths[%d] = %f\n", i, bone_lengths[i]);
      chain_length += bone_lengths[i];
    }
  }

  void resolve(vec3 &target) {
    // save a copy of each bone position
    for (int i = 0; i < bone_count; i++) {
      positions[i] = bones[i].position;
    }

    // printf("d = %f\n", vec3::dist_sqr(target, bones[0].position ));
    // printf("cl = %f\n", chain_length);
    if (vec3::dist_sqr(target, bones[0].position) >
        chain_length * chain_length) {
      vec3 dir = target - bones[0].position;
      dir.normalize();

      for (int i = 1; i < bone_count; i++) {
        positions[i] = positions[i - 1] + dir * bone_lengths[i - 1];
        bones[i].position = positions[i];
      }
    } else {
      for (int iter = 0; iter < iterations; iter++) {
        // top->bottom traversal, move joints towards target
        for (int i = bone_count - 1; i > 0; i--) {
          if (i == bone_count - 1) {
            positions[i] = target;
          } else {
            vec3 directionChildToCurrentBone = positions[i] - positions[i + 1];
            directionChildToCurrentBone.normalize();
            vec3 d = directionChildToCurrentBone * bone_lengths[i];
            positions[i] = positions[i + 1] + d;
          }
        }

        // top->bottom traversal, move joints towards parent bones
        for (int i = 1; i < bone_count; i++) {
          vec3 directionParentToCurrentBone = positions[i] - positions[i - 1];
          directionParentToCurrentBone.normalize();
          vec3 d = directionParentToCurrentBone * bone_lengths[i - 1];
          positions[i] = positions[i - 1] + d;
        }

        // early success
        if (vec3::dist_sqr(positions[bone_count - 1], target) < delta * delta)
          break;
      }

      for (int i = 0; i < bone_count - 2; i++) {
        vec3 &par = positions[i];
        vec3 &mid = positions[i + 1];
        vec3 &child = positions[i + 2];

        vec3 norm = par - child;
        norm.normalize();
        vec3 proj_mid = mid - norm * vec3::dot(mid, norm);
        vec3 proj_pole = pole - norm * vec3::dot(pole, norm);

        proj_mid = proj_mid - par;
        proj_pole = proj_pole - par;

        // proj_mid.normalize();
        // proj_pole.normalize();

        vec3 y_ax = vec3::cross(norm, proj_mid);
        float x = vec3::dot(proj_pole, proj_mid);
        float y = vec3::dot(proj_pole, y_ax);

        // if ( x < SMEPSILON || y < SMEPSILON )
        //     continue;
        float ang = atan2f(y, x);

        // if ( vec3::dist_sqr(proj_mid, proj_pole) < ORTH_TOLERANCE  ||
        // vec3::dist_sqr(proj_mid*-1, proj_pole) < ORTH_TOLERANCE)
        //     continue;

        // float ang = acosf(clamp(vec3::dot(proj_mid, proj_pole), -1, 1));
        // printf("ang=%f\n", ang);

        for (int j = 0; j < bone_count; j++) {
          vec3 relpos = positions[j] - par;
          positions[j] = par + vec3::rotate_axis(relpos, norm, ang);
        }
      }

      for (int i = 0; i < bone_count; i++) {
        bones[i].position = positions[i];
      }
    }
  }

  void print() {
    for (int i = 0; i < bone_count; i++) {
      printf("bone[%d] = ", i);
      bones[i].position.print();
      printf("\tbone length: %f\n", bone_lengths[i]);
      if (bones[i].parent != NULL) {
        Transform p = *bones[i].parent;
        printf("\tparent = ");
        p.position.print();
      } else {
        printf("\troot of chain\n");
      }
    }
  }

  void reset(vec3 &pole) {
    Transform bonechain[] = {Transform({0, 0, 0}), Transform({0, 1, 0.3}),
                             Transform({0, 2, 0})};
    float start_bone_lengths[] = {
        xn::vec3::dist(bonechain[0].position, bonechain[1].position),
        xn::vec3::dist(bonechain[1].position, bonechain[2].position)};
    *this = ik::IkChain(3, bonechain, pole);
  }
};

struct IkConstraint {
  int num_axes;
  float *angle_min;
  float *angle_max;
  float *angles;
  vec3 *axes;

  IkConstraint() {}

  IkConstraint(int _num_axes, float _angle_min[], float _angle_max[],
               vec3 _axes[]) {
    num_axes = _num_axes;
    angle_min = new float[num_axes];
    angle_max = new float[num_axes];
    axes = new vec3[num_axes];
    angles = new float[num_axes];

    for (int i = 0; i < num_axes; i++) {
      angle_min[i] = _angle_min[i];
      angle_max[i] = _angle_max[i];
      axes[i] = _axes[i];
      angles[i] = M_PI / 2;
    }
  }
};

class ServoChain {
public:
  IkChain ik_chain;
  IkConstraint *constraints;
  std::vector<Servo> *servos;

  vec3 *positions;
  float arm_len;

  ServoChain() {}

  ServoChain(IkChain bonechain, std::vector<Servo> _servos[],
             std::vector<vec3> axes[]) {
    ik_chain = bonechain;

    servos = _servos;
    //                servos = new std::vector<Servo>[bonechain.bone_count];

    // servos = new std::vector<pio::ServoAngular>[ik_chain.bone_count];
    constraints = new IkConstraint[ik_chain.bone_count];
    positions = new vec3[ik_chain.bone_count];

    for (int i = 0; i < ik_chain.bone_count; i++) {
      constraints[i].num_axes = _servos[i].size();
      constraints[i].axes = new vec3[_servos[i].size()];
      constraints[i].angle_min = new float[_servos[i].size()];
      constraints[i].angle_max = new float[_servos[i].size()];
      constraints[i].angles = new float[_servos[i].size()];

      for (int j = 0; j < _servos[i].size(); j++) {
        //                        pio::ServoAngular s = _servos[i][j];
        // servos[i] = std::vector<pio::ServoAngular>();
        // printf("loading servo : %f %f\n", s.min_angle, s.max_angle);
        // servos[i].push_back(pio::ServoAngular(s.ctl_pin, s.min_width,
        // s.max_width));
        //                        printf("loaded servo : %f %f\n",
        //                        servos[i].at(j).servo.min_angle,
        //                        servos[i].at(j).servo.max_angle);
        // servos[i][j].setPosition(0.5);
        constraints[i].axes[j] = axes[i][j];
        constraints[i].angle_min[j] = 0;
        constraints[i].angle_max[j] = M_PI;
        constraints[i].angles[j] = M_PI / 2;
      }
      positions[i] = bonechain.bones[i].position;
    }
  }

  //            ~ServoChain() {
  //                for ( int i = 0; i < ik_chain.bone_count; i++ ) {
  //                    int j=0;
  //                    for (SmoothServoInfo s : servos[i]) {
  //                        pthread_join(s.tid, NULL);
  //                    }
  //                }
  //            }

  void resolve(vec3 &target) {
    vec3 &pax = constraints[0].axes[0];

    // vec3 pole{0,1,0};
    vec3 target_normalized = target / arm_len;
    vec3 pole = target_normalized - (pax * vec3::dot(target_normalized, pax));
    if (pole.mag() < SMEPSILON)
      pole = {0, 0, 1};
    else {
      // vec3 pole{target_normalized.x*2, 2, 1};
      pole.normalize();
      pole = pole * target_normalized.mag() * 2;
      // pole.z = target_normalized.z;
      pole.y = 1;
    }

    //                ik_chain.reset(pole);
    ik_chain.resolve(target_normalized);
    // printf("ideal chain:\n");
    // ik_chain.print();
    for (int i = 0; i < ik_chain.bone_count - 1; i++) {
      for (int j = 0; j < constraints[i].num_axes; j++) {
        vec3 next_target = ik_chain.bones[i + 1].position - positions[i];
        vec3 pos = positions[i + 1] - positions[i];
        vec3 delta = next_target - pos;
        vec3 &ax = constraints[i].axes[j];
        vec3 proj_pos = pos - (ax * vec3::dot(pos, ax));
        vec3 proj_target = next_target - (ax * vec3::dot(next_target, ax));

        proj_pos.normalize();

        // if ( proj_target.mag() < SMEPSILON )
        //     continue;

        // proj_target.normalize();

        // if ( vec3::dist_sqr(proj_pos, proj_target) < ORTH_TOLERANCE ||
        // vec3::dist_sqr(proj_pos*-1, proj_target) < ORTH_TOLERANCE )
        //     continue;

        vec3 y_ax = vec3::cross(ax, proj_pos);
        float x = vec3::dot(proj_target, proj_pos);
        float y = vec3::dot(proj_target, y_ax);
        float ang = atan2f(y, x);
        // float ang = acosf(clamp(vec3::dot(proj_pos, proj_target), -1, 1));
        // float s_ang = fmod(servos[i][j].getAngle()*0.95 + ang*0.05, M_PI);
        // float s_ang = fmod(servos[i][j].getAngle() + ang, M_PI);
        float s_ang;
        if (constraints[i].num_axes > 1) {
          s_ang =
              fmod(constraints[i].angles[j] + ang, constraints[i].angle_max[j]);
          if (s_ang < 0) {
            s_ang = constraints[i].angle_max[j] + s_ang;
          }
        } else
          s_ang =
              clamp(constraints[i].angles[j] + ang, constraints[i].angle_min[j],
                    constraints[i].angle_max[j]);

        ang = s_ang - constraints[i].angles[j];

        // servos[i][j].servo.setAngle(s_ang);
        constraints[i].angles[j] = s_ang;
        servos[i][j].write(s_ang * 180 / M_PI);
        // time_sleep(0.25);

        for (int k = i + 1; k < ik_chain.bone_count; k++) {
          vec3 relpos = positions[k] - positions[i];
          positions[k] = positions[i] + vec3::rotate_axis(relpos, ax, ang);
          for (int l = 0; l < constraints[k].num_axes; l++) {
            constraints[k].axes[l] =
                vec3::rotate_axis(constraints[k].axes[l], ax, ang);
            constraints[k].axes[l].normalize();
          }
        }

        for (int k = j + 1; k < constraints[i].num_axes; k++) {
          constraints[i].axes[k] =
              vec3::rotate_axis(constraints[i].axes[k], ax, ang);
          constraints[i].axes[k].normalize();
        }
      }
    }
  }

  //            void reset() {
  //                for ( int i = 0; i < ik_chain.bone_count; i++ ) {
  //                    for ( SmoothServoInfo& s : servos[i] ) {
  //                        s.servo.setPosition(0.5);
  //                    }
  //                }
  //            }
};
} // namespace ik
} // namespace xn
