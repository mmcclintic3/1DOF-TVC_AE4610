#include "pid.h"

namespace {
const float DERIVATIVE_ALPHA = 0.75f;
const float ERROR_DEADBAND_RAD = 0.015f;      // About 0.9 deg
const float INTEGRAL_ACTIVE_BAND_RAD = 0.25f; // Integrate mainly near the target
const float INTEGRAL_DECAY_WHEN_FAR = 0.97f;
const float INTEGRAL_DECAY_IN_DEADBAND = 0.90f;
const float INTEGRAL_LIMIT = 0.35f;
}

PID::PID(float Kp, float Ki, float Kd,
         float out_min, float out_max,
         unsigned long sample_time_ms)
{
  kp = Kp;
  ki = Ki;
  kd = Kd;

  this->out_min = out_min;
  this->out_max = out_max;
  this->sample_time_ms = sample_time_ms;

  integral    = 0.0f;
  prev_theta  = 0.0f;
  prev_out    = 0.0f;
  derivative  = 0.0f;
  prev_time   = millis();

  // Derivative low-pass: 0=no filter, higher=more smoothing.
  alpha = DERIVATIVE_ALPHA;
}

void PID::reset() {
  integral   = 0.0f;
  derivative = 0.0f;
  prev_theta = 0.0f;
  prev_out   = 0.0f;
  prev_time  = millis();
}

float PID::compute(float theta, float theta_ref) {
  unsigned long now = millis();
  float dt = (now - prev_time) / 1000.0f;

  // Skip if called too soon
  if (dt < sample_time_ms / 1000.0f || dt <= 0.0f) {
    return prev_out;
  }

  float error = theta_ref - theta;
  float abs_error = fabsf(error);

  // Proportional
  float p_term = kp * error;
  if (abs_error < ERROR_DEADBAND_RAD) {
    p_term = 0.0f;
  }

  // Integral (adjust based on proximity to setpoint)
  if (abs_error < ERROR_DEADBAND_RAD) {
    integral *= INTEGRAL_DECAY_IN_DEADBAND;
  } else if (abs_error < INTEGRAL_ACTIVE_BAND_RAD) {
    integral += error * dt;
    integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  } else {
    integral *= INTEGRAL_DECAY_WHEN_FAR;
  }

  // Derivative on measurement (avoids kick on setpoint change)
  float dtheta_raw = (theta - prev_theta) / dt;
  derivative = alpha * derivative + (1.0f - alpha) * dtheta_raw;

  // PID sum
  float out = p_term + (ki * integral) - (kd * derivative);

  // Anti-windup: back-calculation
  // If saturated in the same direction as error, undo last integral step
  bool sat_high = (out > out_max);
  bool sat_low  = (out < out_min);
  out = constrain(out, out_min, out_max);

  if ((sat_high && error > 0.0f) || (sat_low && error < 0.0f)) {
    integral -= error * dt;
    integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  }

  prev_theta = theta;
  prev_time  = now;
  prev_out   = out;

  return out;
}
