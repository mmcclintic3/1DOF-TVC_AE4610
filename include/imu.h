#pragma once
#include <Adafruit_BNO08x.h>

extern Adafruit_BNO08x bno;

struct Quaternion {
  float w, x, y, z;
};

bool getQuat(Quaternion& q);
Quaternion quatInverse(Quaternion q);
Quaternion quatMultiply(Quaternion a, Quaternion b);
float getPitchFromQuat(Quaternion q);