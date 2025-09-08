#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class PIDController {
public:
  PIDController(float kp, float ki, float kd);

  // Calculates the PID output based on the error
  float calculate(float error);

  // Resets the controller's state (integral error, etc.)
  void reset();

  // Allows for tuning gains at runtime
  void setGains(float kp, float ki, float kd);

private:
  float kp;
  float ki;
  float kd;

  float integralError;
  float lastError;
  unsigned long lastTime;
};

#endif // PID_CONTROLLER_H
