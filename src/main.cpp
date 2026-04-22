#include "imu.h"
#include "pid.h"

#include <Wire.h>
#include <Servo.h>

const sh2_SensorId_t IMU_REPORT = SH2_GAME_ROTATION_VECTOR;
const long IMU_REPORT_INTERVAL_US = 5000;

Servo myServo;
Servo ESC;

const float SERVO_CENTER_US  = 1150.0f;
const float SERVO_FULL_MIN_US = 500.0f;
const float SERVO_FULL_MAX_US = 2500.0f;
const float SERVO_MAX_DEFLECTION_RAD = PI / 4.0f;  // +/-45 deg about neutral
const float SERVO_US_PER_RAD = (SERVO_FULL_MAX_US - SERVO_FULL_MIN_US) / PI;
const float SERVO_MAX_DELTA_US = SERVO_MAX_DEFLECTION_RAD * SERVO_US_PER_RAD;
const float SERVO_MIN_US = SERVO_CENTER_US - SERVO_MAX_DELTA_US;
const float SERVO_MAX_US = SERVO_CENTER_US + SERVO_MAX_DELTA_US;
const float SERVO_CMD_GAIN   = 45.0f;  // Softer post-PID gain to reduce setpoint chatter
const float SERVO_MAX_STEP_US = 45.0f; // Rate limit each 10 ms update to calm oscillation
const float CONTROL_SIGN     = 1.0f;  // Flip to -1.0f if the servo corrects the wrong way

const float PID_OUT_MIN = -SERVO_MAX_DEFLECTION_RAD;
const float PID_OUT_MAX =  SERVO_MAX_DEFLECTION_RAD;

const unsigned long LOOP_MS  = 10;
const unsigned long PRINT_MS = 50;

const float Kp = 0.0307f;
const float Ki = 0.0113f;
const float Kd = 0.0186f;

static bool header_printed = false;

PID pid(Kp, Ki, Kd, PID_OUT_MIN, PID_OUT_MAX, LOOP_MS);

Quaternion q0;
bool calibrated = false;
float pitch = 0.0f;
float pitch_ref = 25.0f * PI / 180.0f;

void setup() {
  Serial.begin(115200);
  delay(1000);

  if (!bno.begin_I2C()) {
    Serial.println("ERR: BNO085 not found");
    while (1);
  }

  if (!bno.enableReport(IMU_REPORT, IMU_REPORT_INTERVAL_US)) {
    Serial.println("ERR: IMU report failed");
    while (1);
  }

  ESC.attach(5);
  myServo.attach(9, (int)SERVO_FULL_MIN_US, (int)SERVO_FULL_MAX_US);

  myServo.writeMicroseconds((int)SERVO_CENTER_US);
  ESC.writeMicroseconds(1000);

  delay(2000);
  Serial.println("Ready.");
}

void loop() {
  static unsigned long last_loop  = 0;
  static unsigned long last_print = 0;
  static float last_servo_us = SERVO_CENTER_US;
  unsigned long now = millis();

  if (now - last_loop < LOOP_MS) return;
  last_loop = now;

  if (bno.wasReset()) {
    if (!bno.enableReport(IMU_REPORT, IMU_REPORT_INTERVAL_US)) {
      Serial.println("ERR: IMU report re-enable failed");
      while (1);
    }

    calibrated = false;
    pitch = 0.0f;
    pid.reset();
    myServo.writeMicroseconds((int)SERVO_CENTER_US);
    last_servo_us = SERVO_CENTER_US;
    Serial.println("IMU reset, waiting to recalibrate.");
  }

  int raw_throttle = analogRead(A0);
  int esc_us = map(raw_throttle, 0, 1023, 1000, 2000);
  ESC.writeMicroseconds(esc_us);

  Quaternion q;
  if (!getQuat(q)) {
    return;
  }

  if (!calibrated) {
    q0 = q;
    calibrated = true;
    pitch = 0.0f;
    pid.reset();
    myServo.writeMicroseconds((int)SERVO_CENTER_US);
    last_servo_us = SERVO_CENTER_US;
    Serial.println("Calibrated.");
    return;
  }

  Quaternion q_rel = quatMultiply(quatInverse(q0), q);
  pitch = getPitchFromQuat(q_rel);

  float u_cmd = pid.compute(pitch, pitch_ref);
  float servo_delta_us = CONTROL_SIGN * SERVO_CMD_GAIN * u_cmd * SERVO_US_PER_RAD;
  float servo_us = SERVO_CENTER_US + servo_delta_us;
  servo_us = constrain(servo_us, last_servo_us - SERVO_MAX_STEP_US, last_servo_us + SERVO_MAX_STEP_US);
  servo_us = constrain(servo_us, SERVO_MIN_US, SERVO_MAX_US);
  myServo.writeMicroseconds((int)servo_us);
  last_servo_us = servo_us;

  if (now - last_print >= PRINT_MS) {
    last_print = now;
    if (!header_printed) {
      Serial.println("timestamp_ms,imu_pitch,u_cmd,servo_gain_servo_du,servo_cmd,esc_us");
      header_printed = true;
    }
    Serial.print(now);                Serial.print(",");
    Serial.print(pitch, 4);           Serial.print(",");
    Serial.print(u_cmd, 4);           Serial.print(",");
    Serial.print(SERVO_CMD_GAIN, 1);  Serial.print(",");
    Serial.print(servo_us);           Serial.print(",");
    Serial.println(esc_us);         
  }
}