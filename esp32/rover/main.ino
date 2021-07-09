// #include <ArduinoOTA.h>
// #include <ESPmDNS.h>
// #include <WiFiUdp.h>
#include "xn_ota.h"
#include <analogWrite.h>

#include "MPU9250.h"
#include "i2c_periph.h"
#include "xn_current_sensor.h"
#include "xn_ik.h"
#include "xn_net.h"
#include "xn_peripherals.h"

using namespace xn;

int RotaryEncoder::cb_num = 0;
float WheelMotor::target_rpm = 23;

const int pin_servo = 13, pin_stepper_step = 14, pin_stepper_dir = 12,
          pin_claw_current_sensor = 36;

WiFiClient client;

Servo lidar_servo;
WheelMotor wheels[2];

MPU9250 mpu;
uint32_t lidar_data;
pthread_mutex_t lidar_mut = PTHREAD_MUTEX_INITIALIZER;
vec3 mpu_eulers;
std::thread tid[2];
HardwareSerial SerialTF(2);

CurrentSensor cur_sens;
std::thread cur_sens_tid;

void ota_thread() {
  while (true) {
    ArduinoOTA.handle();
    delay(1000);
  }
}

void update_gyro_thread() {
  while (true) {
    mpu.update();
    delay(20);
  }
}

// longitudinal redundancy check for i2c data
uint8_t checksum_LRC(uint8_t *buf, unsigned n) {
  int LRC = 0;
  for (int i = 0; i < n; i++) {
    LRC -= buf[i];
  }
  return (uint8_t)LRC;
}

/**
 *   Bytes are sometimes lost/garbled in the transfer, so 1 padding byte is
 * added at the beginning and end of each message as means for a validity check.
 *
 *   If a packet is invalid, read 1 byte at a time until the padding bytes are
 * in the right position
 */
void i2c_thread() {
  DUMP(i2c_slave_init());
  std::cout << "i2c starting\n";

  while (true) {
    static const int n = sizeof(float) + 2;
    uint8_t buf[n];
    buf[0] = '?';
    float v = cur_sens.getVolts();
    Serial.println(v);
    memcpy(&buf[1], &v, sizeof(v));

    buf[n - 1] = checksum_LRC(buf, n - 1);
    i2c_slave_write_buffer(I2C_SLAVE_NUM, (uint8_t *)buf, n,
                           1000 / portTICK_RATE_MS);
    delay(50);
  }
}

vec3 mpu_get_ypr(const MPU9250 &m) {
  return {m.getYaw(), m.getPitch(), m.getRoll()};
}

void lidar_scan_ex(const LidarInstructionEx &instr);

int tf_read_frame(HardwareSerial SerialTF) {
  int retval = -1;
  if (SerialTF.available() >= 9) {
    unsigned char packet[9];
    SerialTF.readBytes(packet, 9);

    while (!tfMiniDetectFrame(packet)) {
      while (SerialTF.available() == 0)
        delayMicroseconds(500);

      for (int i = 0; i < 8; i++)
        packet[i] = packet[i + 1];
      packet[8] = SerialTF.read();
    }

    if (packet[0] == 0x59 && packet[1] == 0x59) {
      retval = ((uint16_t)(packet[2])) | (((uint16_t)(packet[3])) << 8);
    }
  }
  return retval;
}

void setup() {
  Serial.begin(115200);

  wheels[0] = WheelMotor(5, 18, 33, 25);
  wheels[1] = WheelMotor(0, 4, 26, 27);

  ESP32PWM::allocateTimer(2);
  pinMode(pin_servo, OUTPUT);
  lidar_servo.attach(pin_servo);
  lidar_servo.writeMicroseconds(500);

  pinMode(pin_stepper_step, OUTPUT);
  pinMode(pin_stepper_dir, OUTPUT);
  digitalWrite(pin_stepper_dir, HIGH);

  pinMode(pin_stepper_step, OUTPUT);
  pinMode(pin_stepper_dir, OUTPUT);
  digitalWrite(pin_stepper_dir, HIGH);

  WiFi.mode(WIFI_STA);
  char *mac = new char[WiFi.macAddress().length() + 1];
  strcpy(mac, WiFi.macAddress().c_str());
  Serial.println(mac);
  delete[] mac;
  // PS4.begin(mac);
  WiFi.begin("looneytunes", "1F26107EDB");
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Couldn't get a wifi connection");
    delay(5000);
    ESP.restart();
  }
  WiFi.setSleep(false);

  OTA_init();
  tid[1] = std::thread(OTA_update_thread);

  wheels[0].encoder.init();
  wheels[1].encoder.init();

  analogReadResolution(12);
  cur_sens = CurrentSensor(pin_claw_current_sensor);
  cur_sens_tid = cur_sens.init();

  tid[0] = std::thread(i2c_thread);

  // Wire.begin();
  // delay(2000);
  // mpu.setup(0x68);
  // mpu.calibrateAccelGyro();
  // SerialTF.begin(115200);

  while (!client.connected()) {
    client.connect(laptop_ip, laptop_port);
    delay(1000);
    DUMP(cur_sens.getVolts());
    DUMP(cur_sens.getAmps());
  }
  // client.setTimeout(200);

  printf("%d available\n", client.available());

  printf("starting main loop\n");
  // pthread_create(&tid[0], NULL, update_gyro_thread, NULL);
  // pthread_create(&tid[0], NULL, i2c_thread, NULL);
}

void loop() {
  if (!client.connected()) {
    printf("disconnected\n");
    // while (client.connect(laptop_ip, laptop_port) != 1)
    //     delay(200);
  }

  printf("start time: %d\n", millis());
  MessageRaw msg;
  msg = read_message(client);

  int32_t ack = 1;
  send_buf(client, (char *)&ack, sizeof(ack));

  int8_t type = msg.buf[0];
  printf("type=%d\n", type);
  switch (type) {
  case INSTR_MOTOR: {
    MotorInstruction instr = *(MotorInstruction *)msg.buf;
    WheelMotor::move_motors(instr, wheels);
  } break;
  case INSTR_SENSOR_EX: {
    LidarInstructionEx instr = *(LidarInstructionEx *)msg.buf;
    lidar_scan_ex(instr);
  } break;
  case INSTR_LIDAR_READ: {
    uint16_t d = -1;
    while (SerialTF.available() >= 9 || d == -1) {
      d = tf_read_frame(SerialTF);
    }
    DUMP(d);
    send_buf(client, (char *)&d, sizeof(d));
  } break;
  case INSTR_LIDAR_STEP: {
    LidarStepInstruction instr = *(LidarStepInstruction *)msg.buf;
    if (instr.dir)
      digitalWrite(pin_stepper_dir, HIGH);
    else
      digitalWrite(pin_stepper_dir, LOW);
    step_motor(pin_stepper_step, 50);
  } break;
  case INSTR_LIDAR_SERVO: {
    LidarServoInstruction instr = *(LidarServoInstruction *)msg.buf;
    lidar_servo.writeMicroseconds(instr.width);
  } break;
  case INSTR_RESTART_ESP: {
    ESP.restart();
  } break;
  }
  delete[] msg.buf;

  printf("end time: %d\n", millis());
  // delay(100);
}

void lidar_scan_ex(const LidarInstructionEx &instr) {
  while (!(abs(mpu_get_ypr(mpu).y) < 10) || !(abs(mpu_get_ypr(mpu).z) > 170))
    delay(200);
  mpu_eulers = mpu_get_ypr(mpu);
  int32_t x = mpu_eulers.x, y = mpu_eulers.y, z = mpu_eulers.z;
  //  client.flush();
  DUMP(x);
  DUMP(y);
  DUMP(z);
  send_buf(client, (char *)&x, sizeof(x));
  send_buf(client, (char *)&y, sizeof(y));
  send_buf(client, (char *)&z, sizeof(z));
  int32_t step_count = 0;
  int step_incr = 1;
  bool dir = true;
  int32_t servo_width = instr.y_degrees_min;
  int servo_width_orig = servo_width;
  lidar_servo.writeMicroseconds(servo_width);

  if (instr.x_degrees_min < 0) {
    step_incr *= -1;
    digitalWrite(pin_stepper_dir, HIGH);
  }

  for (step_count = 0; step_count != instr.x_degrees_min;
       step_count += step_incr) {
    step_motor(pin_stepper_step, 50);
    delay(50);
  }

  step_incr = 1;
  digitalWrite(pin_stepper_dir, LOW);
  while (servo_width < instr.y_degrees_max && run) {
    if (client.connected()) {
      if (step_count + step_incr > instr.x_degrees_max ||
          step_count + step_incr < instr.x_degrees_min) {
        step_incr *= -1;
        dir = !dir;
        if (dir)
          digitalWrite(pin_stepper_dir, LOW);
        else
          digitalWrite(pin_stepper_dir, HIGH);
        servo_width += instr.y_degrees_per_step;
        lidar_servo.writeMicroseconds(servo_width);
        continue;
      }
      step_count += step_incr;
      step_motor(pin_stepper_step, 50);

      int d = -1;
      while (SerialTF.available() >= 9 || d == -1) {
        d = tf_read_frame(SerialTF);
      }
      int32_t d32 = d;
      send_buf(client, (char *)&d32, sizeof(int32_t));
      int32_t relstep = servo_width - servo_width_orig;
      send_buf(client, (char *)&step_count, sizeof(int32_t));
      send_buf(client, (char *)&relstep, sizeof(int32_t));
    } else {
      printf("disconnect while scanning\n");
      while (!client.connected())
        client.connect(laptop_ip, laptop_port);
      return;
    }
  }

  int32_t stop_ack = -1;
  send_buf(client, (char *)&stop_ack, sizeof(int32_t));

  lidar_servo.writeMicroseconds(servo_width_orig);
}