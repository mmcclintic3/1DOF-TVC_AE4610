#pragma once
#include <Arduino.h>

class PID {
public:
  PID(float Kp, float Ki, float Kd,
      float out_min, float out_max,
      unsigned long sample_time_ms);

  float compute(float theta, float theta_ref);
  void  reset();

private:
  float kp, ki, kd;

  float prev_theta;
  float integral;
  float derivative;
  float prev_out;
  float alpha;

  float out_min, out_max;

  unsigned long sample_time_ms;
  unsigned long prev_time;
};