#ifndef FRICTION_H
#define FRICTION_H
extern float friction_correction_integral=0;
const double polyCoeffs_high[4] = {
    42.94318923289474554394, 0.00690707214480905125, 0.00000068754010283506, 0.00000000112726056534};
const double polyCoeffs_low[4] = {
  42.94318923289474554394, 0.00690707214480905125, 0.00000068754010283506, 0.00000000112726056534};
float friction(float vel)
{
  float result = 0;
  int sign = 1;
  // if (vel < 0) sign = -1;
  vel = abs(vel);
  if(vel > 5600) vel = 5600;
  if (vel > 10000000) for (int i = 0; i < 4; i++)
  {
    result += polyCoeffs_high[i] * pow(vel, i);
  }
  else for (int i = 0; i < 4; i++)
  {
    result += polyCoeffs_low[i] * pow(vel, i);
  }
  return sign * result;
}
float friction_correction(float fill, float raw_acc) {
  // Static variables maintain their value between function calls.
  static float filtered_correction = 0;
  static float prev_accel_error = 0; // Stores the previous error for derivative calculation
  static float filtered_derivative = 0; // Stores the filtered derivative term

  // These are the gains you'll need to tune.
  const float ki_friction = 300.0f;
  const float kp_friction = 2000.0f;
  const float kd_friction = 200.0f;
  const float Ts = 0.01f;

  const float integrator_decay_factor = 0.9999f;
  
  const float filter_alpha = 0.2f;
  const float derivative_filter_alpha = 0.2f;

  const float MAX_ACCEL = 0.1f;
  const float INTEGRAL_MAX = 0.1f;
  const float INTEGRAL_MIN = -0.1f;

  float processed_acc = raw_acc;
  if (abs(raw_acc) > MAX_ACCEL) {
    processed_acc = 0;
  }

  float accel_error = 0;
  if (abs(processed_acc) > 0.0001 && abs(fill) > 5)
  {
    if (fill * processed_acc > 0) accel_error = processed_acc;
  }

  float derivative = (accel_error - prev_accel_error) / Ts;
  prev_accel_error = accel_error;

  filtered_derivative = filtered_derivative + derivative_filter_alpha * (derivative - filtered_derivative);

  friction_correction_integral *= integrator_decay_factor;
  friction_correction_integral += ki_friction * accel_error * Ts;

  // Anti-windup
  if (friction_correction_integral > INTEGRAL_MAX) {
    friction_correction_integral = INTEGRAL_MAX;
  } else if (friction_correction_integral < INTEGRAL_MIN) {
    friction_correction_integral = INTEGRAL_MIN;
  }

  float correction = (kp_friction * accel_error) + friction_correction_integral + (kd_friction * filtered_derivative);

  return correction;
}
#endif