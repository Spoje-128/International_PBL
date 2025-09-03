#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
  PIDController(float Kp, float Ki, float Kd);
  float calculate(float setpoint, float currentValue);
  void reset();

private:
  float kp, ki, kd;
  float integral;
  float previous_error;
  unsigned long previous_time;
};

#endif // PID_CONTROLLER_H
