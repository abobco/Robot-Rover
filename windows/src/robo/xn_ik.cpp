#ifdef _WIN32
#ifndef PIO_VIRTUAL
#define PIO_VIRTUAL
#endif
#endif

#include "xn_ik.hpp"
namespace xn {
namespace ik {
IkChain::IkChain(int _bone_count, vec3 ordered_bones[], vec3 &_pole, int _iterations) {
    bone_count = _bone_count;
    iterations = _iterations;
    pole_main = _pole;

    bones = new vec3[bone_count + 1];
    bone_lengths = new float[bone_count + 1];
    positions = new vec3[bone_count + 1];
    poles = new vec3[bone_count - 1];

    for (int i = 0; i < bone_count; i++) {
        bones[i] = ordered_bones[i];
    }
    // for ( int i = 1; i < bone_count; i++ ) {
    //     bones[i].parent = &bones[i-1];
    // }

    // vec3 current = bones[bone_count-2];
    for (int i = 0; i < bone_count; i++) {
        bone_lengths[i] = 0;
    }

    for (int i = bone_count - 2; i >= 0; i--) {
        // bone_start_rotations[i] = current.rotation;
        bone_lengths[i] = vec3::dist(bones[i + 1], bones[i]);
        // printf("bone_lengths[%d] = %f\n", i, bone_lengths[i]);
        chain_length += bone_lengths[i];
    }
}

void IkChain::resolve(vec3 &target) {
    // save a copy of each bone position
    for (int i = 0; i < bone_count; i++) {
        positions[i] = bones[i];
    }

    if (vec3::dist_sqr(target, bones[0]) > chain_length * chain_length) {
        // target is out of reach
        vec3 dir = target - bones[0];
        dir.normalize();

        for (int i = 1; i < bone_count; i++) {
            positions[i] = positions[i - 1] + dir * bone_lengths[i - 1];
            bones[i] = positions[i];
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

            // bottom->top traversal, move joints towards parent bones
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

        // resolve poles
        for (int i = 0; i < bone_count - 2; i++) {
            vec3 &par = positions[i];
            vec3 &mid = positions[i + 1];
            vec3 &child = positions[i + 2];

            vec3 norm = par - child; // axis of rotation
            if (norm.mag_sqr() < SMEPSILON) {
                std::cout << "axis of rotation has length 0, skipping\n";
                break;
            }
            norm.normalize();

            // TODO: solve for remaining poles, follow a curve or sphere around the
            // arm
            if (i > 0)
                poles[i] = {0, 3, 0};
            else
                poles[i] = pole_main;

            vec3 proj_mid = mid - norm * vec3::dot(mid, norm);
            vec3 proj_pole = poles[i] - norm * vec3::dot(poles[i], norm);

            proj_mid = proj_mid - par;
            proj_pole = proj_pole - par;

            vec3 y_ax = vec3::cross(norm, proj_mid);
            float x = vec3::dot(proj_pole, proj_mid);
            float y = vec3::dot(proj_pole, y_ax);

            float ang = atan2f(y, x);

            // rotate middle joint
            vec3 relpos = mid - par;
            positions[i + 1] = par + vec3::rotate_axis(relpos, norm, ang);
        }

        for (int i = 0; i < bone_count; i++) {
            bones[i] = positions[i];
        }
    }
}

void IkChain::print() {
    for (int i = 0; i < bone_count; i++) {
        printf("bone[%d] = ", i);
        bones[i].print();
        printf("\tbone length: %f\n", bone_lengths[i]);
        // if ( bones[i].parent != NULL) {
        //     Transform p = *bones[i].parent;
        //     printf("\tparent = ");
        //     p.print();
        // } else {
        //     printf("\troot of chain\n");
        // }
    }
}

void IkChain::reset() {
    vec3 pole = {0, 0, 1};
    vec3 bonechain[] = {vec3{0, 0, 0}, vec3{0, 1, 0}, vec3{0, 1 + 6.9f / 17.2f, 0},
                        vec3{0, 1 + 13.8f / 17.2f, 0}};
    // float start_bone_lengths[] = {
    //     vec3::dist(bonechain[0], bonechain[1]),
    //     vec3::dist(bonechain[1], bonechain[2])
    // };

    *this = xn::ik::IkChain(4, bonechain, pole);
}

void move_servo_thread(pio::SmoothServo *s) {
    // pio::SmoothServo *s = (pio::SmoothServo *)argv;
    double step = 1.0 / 120;

    while (true) {
        // asymptotic lerp
        // s->servo.setAngle(s->servo.getAngle()*0.95 + s->target_ang*0.05);

        // bezier curve
        s->update(step);
        time_sleep(step);
    }
    // return NULL;
}

ServoChain::ServoChain(IkChain bonechain, std::vector<pio::SmoothServo> _servos[], bool *loopvar,
                       float _arm_len) {
    ideal_chain = bonechain;
    arm_len = _arm_len;
    numJoints = ideal_chain.bone_count;

    servos = _servos;
    for (int i = 0; i < bonechain.bone_count; i++) {
        servos_orig.push_back(std::vector<pio::SmoothServo>());
        for (pio::SmoothServo &s : servos[i]) {
            servos_orig[i].push_back(s);
        }
    }

    positions = new vec3[ideal_chain.bone_count];
    positions_orig = new vec3[ideal_chain.bone_count];
    for (int i = 0; i < ideal_chain.bone_count; i++) {
        positions[i] = bonechain.bones[i];
        positions_orig[i] = bonechain.bones[i];
    }
}

// ~ServoChain() {
//   for (int i = 0; i < ideal_chain.bone_count; i++) {
//     for (pio::SmoothServo &s : servos[i]) {
//       s.tid.join();
//     }
//   }
// }

void ServoChain::create_threads(std::vector<std::thread> &threadpool) {
    for (int i = 0; i < ideal_chain.bone_count; i++) {
        for (pio::SmoothServo &s : servos[i]) {
            threadpool.push_back(std::thread(move_servo_thread, &s));
        }
    }
}

void ServoChain::resolve(vec3 &target) {
    vec3 &pax = servos[0][0].axis;
    vec3 target_normalized = target / arm_len;
    vec3 pole = target_normalized - (pax * vec3::dot(target_normalized, pax));
    if (pole.mag() < SMEPSILON)
        pole = {3, 0, 0};
    else {
        pole.normalize();
        pole = pole * -5;
    }

    ideal_chain.pole_main = pole;
    ideal_chain.resolve(target_normalized);
    // printf("ideal chain:\n");
    // ideal_chain.print();
    for (int i = 0; i < ideal_chain.bone_count - 1; i++) {
        for (int j = 0; j < servos[i].size(); j++) {
            vec3 next_target = ideal_chain.bones[i + 1] - positions[i];
            vec3 pos = positions[i + 1] - positions[i];
            vec3 &ax = servos[i][j].axis;
            vec3 proj_pos = pos - (ax * vec3::dot(pos, ax));
            vec3 proj_target = next_target - (ax * vec3::dot(next_target, ax));

            if (proj_pos.mag() < SMEPSILON)
                continue;

            proj_pos.normalize();

            vec3 y_ax = vec3::cross(ax, proj_pos);
            float x = vec3::dot(proj_target, proj_pos);
            float y = vec3::dot(proj_target, y_ax);
            float ang = atan2f(y, x);
            float s_ang;
            if (servos[i].size() > 1 && j == 0)
                s_ang = fmod(servos[i][j].target_angle + ang, servos[i][j].servo.max_angle);
            else
                s_ang = clamp<float>(servos[i][j].target_angle + ang, servos[i][j].servo.min_angle,
                                     servos[i][j].servo.max_angle);

            if (s_ang < 0) {
                s_ang = servos[i][j].servo.max_angle + s_ang;
            }
            ang = s_ang - servos[i][j].target_angle;

            servos[i][j].target_angle = s_ang;
            // servos[i][j].servo.setAngle(s_ang);

            // rotate servos in child joints
            for (int k = i + 1; k < ideal_chain.bone_count; k++) {
                vec3 relpos = positions[k] - positions[i];
                positions[k] = positions[i] + vec3::rotate_axis(relpos, ax, ang);
                for (int l = 0; l < servos[k].size(); l++) {
                    servos[k][l].axis = vec3::rotate_axis(servos[k][l].axis, ax, ang);
                    servos[k][l].axis.normalize();
                }
            }

            // rotate servos axes in current joint
            for (int k = j + 1; k < servos[i].size(); k++) {
                servos[i][k].axis = vec3::rotate_axis(servos[i][k].axis, ax, ang);
                servos[i][k].axis.normalize();
            }
        }
    }
}

void ServoChain::curl() {
    servos[0][1].target_angle = (float)M_PI_2;
    servos[1][0].target_angle = (float)0;
    servos[2][0].target_angle = (float)M_PI;
    servos[2][0].target_angle = (float)M_PI;
}

void ServoChain::straighten() {
    curl();
    // time_sleep(servos[0][0].t_max / 2);
    for (int i = 0; i < ideal_chain.bone_count; i++) {
        for (pio::SmoothServo &s : servos[i]) {
            s.target_angle = (float)M_PI_2;
        }
    }
    // time_sleep(servos[0][0].t_max / 2);
    reset();
    ideal_chain.reset();
}

// solve for a point on the line from arm to target, wristlen away from the
// target
void ServoChain::grab_safe(double wristlen, vec3 &target, const vec3 &arm_pos
                           //  const vec3 &arm_pos = {60, 0, -22}
) {
    xn::vec3 reltarg = target - arm_pos;

    // similar triangles solve for x & y offset of wrist
    double d_armpos = sqrt(reltarg.x * reltarg.x + reltarg.z * reltarg.z);
    double dx = wristlen / (d_armpos / reltarg.x);
    double dz = wristlen / (d_armpos / reltarg.z);
    reltarg.x -= (float)dx;
    reltarg.z -= (float)dz;

    resolve(reltarg);
}

void ServoChain::move_wrist(pio::SmoothServo &wrist) {
    float ac = 0;
    for (int i = 0; i < ideal_chain.bone_count; i++) {
        for (xn::pio::SmoothServo &s : servos[i]) {
            if (s.axis.y > 0.5)
                continue;
            float fac = s.target_angle - (float)M_PI_2;
            if (i != 2)
                fac *= -1;
            ac += fac;
        }
    }

    wrist.target_angle = (float)M_PI - ac;
}

// TODO: save start values from constructor instead of this bs
void ServoChain::reset() {
    for (auto i = 0; i < servos_orig.size(); i++) {
        std::vector<pio::SmoothServo> &joint = servos_orig[i];
        for (auto j = 0; j < joint.size(); j++) {
            servos[i][j].position = joint[j].position;
            servos[i][j].axis = joint[j].axis;
        }
    }

    for (auto i = 0; i < ideal_chain.bone_count; i++) {
        positions[i] = positions_orig[i];
    }
}
} // namespace ik
} // namespace xn