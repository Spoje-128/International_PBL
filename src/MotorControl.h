#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

class MotorControl {
public:
  MotorControl();
  void init();
  void moveForward(int speed);
  void moveBackward(int speed);
  void turnLeft(int speed);
  void turnRight(int speed);
  void stopRobot();
  void robotTest();

private:
  // L298Nモータードライバのピン定義
  static const int ENA = 9;   // D9ピン - 右車輪モーター速度制御（PWM）
  static const int ENB = 10;  // D10ピン - 左車輪モーター速度制御（PWM）
  // OutputA（右車輪）制御用
  static const int IN1 = 4;   // D4ピン - 右車輪モーター制御ピン1
  static const int IN2 = 5;   // D5ピン - 右車輪モーター制御ピン2
  // OutputB（左車輪）制御用
  static const int IN3 = 6;   // D6ピン - 左車輪モーター制御ピン3
  static const int IN4 = 7;   // D7ピン - 左車輪モーター制御ピン4

  void rightWheelForward(int speed);
  void rightWheelBackward(int speed);
  void rightWheelStop();
  void leftWheelForward(int speed);
  void leftWheelBackward(int speed);
  void leftWheelStop();
};

#endif // MOTOR_CONTROL_H
