#include "XNlib/xn_gpio.hpp"
#include "XNlib/xn_ik.hpp"
#include "XNlib/xn_net.hpp"
#include "XNlib/xn_vec.hpp"

using namespace xn;

int pio::PigDaemon::pi = 0;

bool start_read_data = false;
bool receiving_commands = true;
bool move_to_target = false;
int cam_sock = 0, arm_ctl_sock = 0, esp_ser_sock = 0;
pthread_mutex_t frameread_lock;
pthread_t cam_io_tid, arm_ctl_tid, arm_update_tid, i2c_tid, ser_tid;

std::vector<std::thread> threads;

pio::ServoAngular servo_cam_x(23, 500, 2500);
pio::ServoAngular servo_cam_y(24, 500, 2500);
const vec2 servo_cam_origin{1675, 853};

ik::ServoChain *armptr;
pio::SmoothServo *clawptr, *wristptr;
const float w_to_ang = M_PI / (2500 - 500);
const float claw_open_a = (1500) * w_to_ang;
const float claw_close_a = (2000) * w_to_ang;
vec3 target;
double wristlen = 150;

void INThandler(int sig) {
    signal(sig, SIG_IGN);
    receiving_commands = false;
    // pthread_join(cam_io_tid, NULL);
}

void ABRThandler(int sig) {
    signal(sig, SIG_IGN);
    receiving_commands = false;
    time_sleep(0.1);
    wristptr->target_angle = M_PI_2;
    armptr->curl();
    time_sleep(5);
    gpioTerminate();
}

void *cam_io_thread(void *arg) {
    pio::Arducam cam = *(pio::Arducam *)arg;
    while (receiving_commands) {
        if (start_read_data == 1) {
            cam.capture();
            // send frame over TCP socket
            if (cam_sock) {
                // send buffer size 1st cuz jpeg compression makes the size unpredictable
                int32_t send_len = cam.frame_size;
                int32_t tmp = htonl(send_len);
                send(cam_sock, &tmp, sizeof(tmp), 0);

                // send frame buffer
                size_t bs = send(cam_sock, cam.frame_buffer, cam.frame_size, 0);

                // printf("frame_len = %d\n", cam.frame_size);
                // printf("bs = %d\n", bs);
                assert(bs == cam.frame_size);
            }
        }
    }
    return NULL;
}

void *esp_i2c_thread(void *arg) {
    std::cout << "opening i2c\n";
    int h = i2cOpen(0, 4, 0);
    if (h < 0) {
        printf("error %d\n", h);
    }

    char handshake_buf[] = "hello\0";
    std::cout << "sending handshake: " << handshake_buf << '\n';
    DUMP(pio::i2c_write_buf(h, handshake_buf, strlen(handshake_buf)));

    while (receiving_commands) {
        int32_t n = 0;
        pio::i2c_read_buf(h, (char *)&n, sizeof(n));
        // DUMP(n);

        if (n > 0) {
            char *buf = new char[n];
            DUMP(pio::i2c_read_buf(h, buf, n));

            std::cout << std::string(buf);
            delete[] buf;
        }

        time_sleep(1);
    }

    return NULL;
}

void *esp_serial_thread(void *argv) {
    int h = serOpen("/dev/serial0", 115200, 0);

    while (receiving_commands) {
        int n = serDataAvailable(h);

        if (n > 0) {
            char *buf = new char[n];
            serRead(h, buf, n);
            send(esp_ser_sock, buf, n, 0);
            // std::cout << std::string(buf);
            delete[] buf;
        } else {
            time_sleep(0.1);
        }
    }
}

static void read_vec3(int sock, vec3 &v) {
    int32_t x, y, z;
    read_int<int32_t>(sock, x);
    read_int<int32_t>(sock, y);
    read_int<int32_t>(sock, z);
    v = {(float)x, (float)y, (float)z};
}

void *arm_ctl_thread(void *argv) {
    bool first = true;
    while (receiving_commands) {
        vec3 v_target;
        read_vec3(arm_ctl_sock, v_target);

        if (first) {
            clawptr->target_angle = claw_open_a;
            armptr->stiffy();
            time_sleep(2.0);
            first = false;
        }

        // align arm to target
        target = {-v_target.x + 40, 5, -v_target.y};
        DUMP(target);
        wristlen = 150;
        wristptr->t_max = armptr->servos[0][1].t_max / 4;
        move_to_target = true;
        time_sleep(armptr->servos[0][1].t_max / 2);

        // move to target
        wristptr->t_max = armptr->servos[0][1].t_max * 0.75;
        wristlen = 5;
        time_sleep(armptr->servos[0][1].t_max / 4);

        // grab
        clawptr->target_angle = claw_close_a;
        time_sleep(armptr->servos[0][1].t_max / 4);

        // move to neutral position
        move_to_target = false;
        wristptr->t_max = armptr->servos[0][1].t_max / 2;
        armptr->stiffy();
        time_sleep(2);

        // release
        clawptr->target_angle = claw_open_a;
        time_sleep(armptr->servos[0][1].t_max / 4);
    }
}

void *arm_update_thread(void *argv) {
    while (receiving_commands) {
        if (move_to_target) {
            armptr->grab_safe(wristlen, target);
        }
        armptr->move_wrist(*wristptr);
    }
}

int main(int argc, char *argv[]) {
    if (pthread_mutex_init(&frameread_lock, NULL) != 0) {
        printf("\n mutex init has failed\n");
        return -1;
    }

    if (gpioInitialise() < 0)
        return -1;
    gpioSetSignalFunc(SIGINT, INThandler);
    gpioSetSignalFunc(SIGABRT, ABRThandler);

    servo_cam_x.setWidth(1500);
    servo_cam_y.setWidth(1500);

    printf("%d\n", laptop_port);
    pio::Arducam cam;
    cam.setQuality(4);

    cam_sock = connect_socket_blocking(laptop_port);
    arm_ctl_sock = connect_socket_blocking(laptop_port + 1);
    esp_ser_sock = connect_socket_blocking(laptop_port + 2);

    vec3 pole = {0, 0, 1};
    vec3 bonechain[] = {vec3{0, 0, 0}, vec3{0, 1, 0}, vec3{0, 1 + 6.9 / 17.2, 0},
                        vec3{0, 1 + 13.8 / 17.2, 0}};

    ik::IkChain arm(4, bonechain, pole);
    std::vector<pio::SmoothServo> servos[] = {
        std::vector<pio::SmoothServo>(), std::vector<pio::SmoothServo>(),
        std::vector<pio::SmoothServo>(), std::vector<pio::SmoothServo>()};
    servos[0].push_back(
        pio::SmoothServo(pio::ServoAngular(12, 500, 2500), {0, 1, 0}, bonechain[0], 1450));
    servos[0].push_back(
        pio::SmoothServo(pio::ServoAngular(13, 500, 2500), {1, 0, 0}, bonechain[0], 1600));
    servos[1].push_back(
        pio::SmoothServo(pio::ServoAngular(26, 500, 2500), {1, 0, 0}, bonechain[1], 1600));
    servos[2].push_back(
        pio::SmoothServo(pio::ServoAngular(16, 500, 2500), {-1, 0, 0}, bonechain[2], 1500));

    pio::SmoothServo wrist(pio::ServoAngular(6, 500, 2500));
    wrist.servo.setAngle(M_PI_2);
    wrist.t_max *= 0.5;
    wrist.target_angle = 0;
    // pthread_create(&wrist.tid, NULL, ik::move_servo_thread, &wrist);
    // wrist.tid = std::thread(ik::move_servo_thread, wrist);
    threads.push_back(std::thread(ik::move_servo_thread, &wrist));
    wristptr = &wrist;

    pio::SmoothServo claw(pio::ServoAngular(5, 500, 2500));
    claw.servo.setAngle(claw_open_a);
    claw.target_angle = claw_open_a;
    // pthread_create(&claw.tid, NULL, ik::move_servo_thread, &claw);
    // claw.tid = std::thread(ik::move_servo_thread, claw);
    threads.push_back(std::thread(ik::move_servo_thread, &claw));
    clawptr = &claw;

    ik::ServoChain phys_arm(arm, servos, &run, 172);
    phys_arm.servos[0][0].t_max *= 0.5;
    armptr = &phys_arm;

    phys_arm.servos[0][1].servo.setAngle(M_PI / 2);
    phys_arm.servos[1][0].servo.setAngle(0.1);
    phys_arm.servos[1][0].target_angle = 0;
    phys_arm.servos[2][0].servo.setAngle(M_PI);
    phys_arm.servos[2][0].target_angle = M_PI;

    start_read_data = 1;

    pthread_create(&cam_io_tid, NULL, cam_io_thread,
                   (void *)&cam); // send camera feed to laptop for processing

    pthread_create(&arm_ctl_tid, NULL, arm_ctl_thread, NULL);
    pthread_create(&arm_update_tid, NULL, arm_update_thread, NULL);

    pthread_create(&ser_tid, NULL, esp_serial_thread, NULL);
    pthread_create(&i2c_tid, NULL, esp_i2c_thread, NULL);

    while (receiving_commands) {
        int32_t pin, value;
        read_int(cam_sock, pin, true, 0);
        read_int(cam_sock, value, true, 0);

        gpioServo(pin, value);
    }

    pthread_join(cam_io_tid, NULL);
}