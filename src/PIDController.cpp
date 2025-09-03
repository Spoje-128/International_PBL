#include "PIDController.h"
#include <Arduino.h>

PIDController::PIDController(float Kp, float Ki, float Kd) {
  kp = Kp;
  ki = Ki;
  kd = Kd;
  integral = 0;
  previous_error = 0;
  previous_time = millis();
}

float PIDController::calculate(float setpoint, float currentValue) {
  unsigned long current_time = millis();
  float dt = (current_time - previous_time) / 1000.0;
  previous_time = current_time;

  float error = setpoint - currentValue;
  integral += error * dt;
  float derivative = (error - previous_error) / dt;
  previous_error = error;

  return kp * error + ki * integral + kd * derivative;
}

void PIDController::reset() {
  integral = 0;
  previous_error = 0;
  previous_time = millis();
}
