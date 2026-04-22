#include "imu.h"

Adafruit_BNO08x bno;

static sh2_SensorValue_t sensorValue;

Quaternion quatInverse(Quaternion q) {
  return {q.w, -q.x, -q.y, -q.z};
}

Quaternion quatMultiply(Quaternion a, Quaternion b) {
  Quaternion r;
  r.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
  r.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
  r.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
  r.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
  return r;
}

float getPitchFromQuat(Quaternion q) {
  float gx = -(2.0f * (q.x*q.y + q.w*q.z));
  float gy = -(1.0f - 2.0f * (q.x*q.x + q.z*q.z));
  return atan2f(gx, -gy);
}

bool getQuat(Quaternion& q) {
  if (!bno.getSensorEvent(&sensorValue)) {
    return false;
  }

  if (sensorValue.sensorId != SH2_GAME_ROTATION_VECTOR) {
    return false;
  }

  q.w = sensorValue.un.gameRotationVector.real;
  q.x = sensorValue.un.gameRotationVector.i;
  q.y = sensorValue.un.gameRotationVector.j;
  q.z = sensorValue.un.gameRotationVector.k;
  return true;
}