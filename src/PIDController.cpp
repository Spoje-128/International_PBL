#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd) {
  setGains(kp, ki, kd);
  reset();
}

void PIDController::setGains(float p, float i, float d) {
  kp = p;
  ki = i;
  kd = d;
}

void PIDController::reset() {
  integralError = 0;
  lastError = 0;
  lastTime = 0;
}


float PIDController::calculate(float error) {
  unsigned long currentTime = millis();
  float dt = (lastTime > 0) ? (float)(currentTime - lastTime) / 1000.0 : 0;

  // To prevent instability on the first run or after a long pause
  if (dt <= 0 || dt > 0.5) {
    dt = 0;
  }

  // Integral term (with anti-windup)
  integralError += error * dt;
  // A generic anti-windup limit. This might need tuning.
  integralError = constrain(integralError, -250, 250);

  // Derivative term
  float derivative = (dt > 0) ? (error - lastError) / dt : 0;

  // Update state for next calculation
  lastError = error;
  lastTime = currentTime;

  // PID calculation
  float output = (kp * error) + (ki * integralError) + (kd * derivative);
  return output;
}
