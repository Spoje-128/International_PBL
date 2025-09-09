#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

class MotorControl {
public:
  MotorControl();
  void init();
  void setSpeeds(int left_pwm, int right_pwm);
  void forward(int speed);
  void backward(int speed);
  void turnLeft(int speed);
  void turnRight(int speed);
  void stop();
  void setMotorDirection(bool rightReversed, bool leftReversed);

private:
  // Pin definitions for L298N motor driver
  static const int ENA = 9;   // Right wheel speed (PWM)
  static const int ENB = 10;  // Left wheel speed (PWM)
  static const int IN1 = 4;   // Right wheel direction 1
  static const int IN2 = 5;   // Right wheel direction 2
  static const int IN3 = 6;   // Left wheel direction 1
  static const int IN4 = 7;   // Left wheel direction 2

  // Motor direction reversal flags
  bool rightMotorReversed;
  bool leftMotorReversed;

  // Low-level motor control functions
  void rightWheelForward(int speed);
  void rightWheelBackward(int speed);
  void rightWheelStop();
  void leftWheelForward(int speed);
  void leftWheelBackward(int speed);
  void leftWheelStop();
};

#endif // MOTOR_CONTROL_H
