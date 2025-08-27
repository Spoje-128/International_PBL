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

// ロボット全体を前進させる関数
void MotorControl::moveForward(int speed) {
  rightWheelForward(speed);
  leftWheelForward(speed);
  Serial.println("Moving Forward");
}
 
// ロボット全体を後進させる関数
void MotorControl::moveBackward(int speed) {
  rightWheelBackward(speed);
  leftWheelBackward(speed);
  Serial.println("Moving Backward");
}

// 左旋回する関数（右車輪前進、左車輪後進）
void MotorControl::turnLeft(int speed) {
  rightWheelForward(speed);
  leftWheelBackward(speed);
  Serial.println("Turning Left");
}

// 右旋回する関数（左車輪前進、右車輪後進）
void MotorControl::turnRight(int speed) {
  leftWheelForward(speed);
  rightWheelBackward(speed);
  Serial.println("Turning Right");
}

// ロボットを停止させる関数
void MotorControl::stopRobot() {
  rightWheelStop();
  leftWheelStop();
  Serial.println("Robot Stopped");
}

// ロボット動作テスト関数
void MotorControl::robotTest() {
  Serial.println("=== Robot Movement Test ===");
  
  // 前進テスト
  moveForward(255);
  delay(2000);
  
  stopRobot();
  delay(1000);
  
  // 後進テスト
  moveBackward(255);
  delay(2000);
  
  stopRobot();
  delay(1000);
  
  // // 左旋回テスト
  // turnLeft(150);
  // delay(1000);
  
  // stopRobot();
  // delay(1000);
  
  // // 右旋回テスト
  // turnRight(150);
  // delay(1000);
  
  // stopRobot();
  // delay(2000);
  
  Serial.println("Test cycle completed\n");
}
