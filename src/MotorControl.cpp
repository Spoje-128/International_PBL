#include "MotorControl.h"

MotorControl::MotorControl() {}

void MotorControl::init() {
  // モーター制御ピンを出力として設定
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.println("L298N Dual Motor Control with PWM Ready");
  Serial.println("Right Wheel: ENA(D9), D4(IN1), D5(IN2)");
  Serial.println("Left Wheel: ENB(D10), D6(IN3), D7(IN4)");
}

void MotorControl::move(int baseSpeed, int correction) {
  int leftSpeed = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  // 速度をPWMの範囲内（0-255）に収める
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  leftWheelForward(leftSpeed);
  rightWheelForward(rightSpeed);
}

// 右車輪を前進させる関数
void MotorControl::rightWheelForward(int speed) {
  analogWrite(ENA, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

// 右車輪を後進させる関数
void MotorControl::rightWheelBackward(int speed) {
  analogWrite(ENA, speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

// 右車輪を停止させる関数
void MotorControl::rightWheelStop() {
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

// 左車輪を前進させる関数
void MotorControl::leftWheelForward(int speed) {
  analogWrite(ENB, speed);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// 左車輪を後進させる関数
void MotorControl::leftWheelBackward(int speed) {
  analogWrite(ENB, speed);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// 左車輪を停止させる関数
void MotorControl::leftWheelStop() {
  analogWrite(ENB, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// ロボットを停止させる関数
void MotorControl::stopRobot() {
  rightWheelStop();
  leftWheelStop();
  Serial.println("Robot Stopped");
}

void MotorControl::turnLeft(int speed) {
  rightWheelForward(speed);
  leftWheelBackward(speed);
}

void MotorControl::turnRight(int speed) {
  leftWheelForward(speed);
  rightWheelBackward(speed);
}

