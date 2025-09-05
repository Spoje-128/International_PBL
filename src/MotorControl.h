#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// 前方宣言
class GyroSensor;

class MotorControl {
public:
  MotorControl();
  void init();
  void move(int baseSpeed, int correction);
  void moveBackward(int speed);
  void turnLeft(int speed);
  void turnRight(int speed);
  void stopRobot();
  void setMotorDirection(bool rightReversed, bool leftReversed);
  
  // 角速度ベース制御関数（ジャイロフィードバック版）
  void moveWithAngularVelocity(float linearSpeed, float targetAngularVelocity); // [deg/s]
  void turnAtAngularVelocity(float targetAngularVelocity); // [deg/s] 正=右旋回、負=左旋回
  void moveStraight(float linearSpeed); // 直進（角速度=0）
  void setRobotParameters(float wheelbase, float wheelRadius); // ロボットパラメータ設定
  void setGyroSensor(GyroSensor* gyro); // ジャイロセンサー設定
  void updateAngularControl(); // 角速度制御の更新（ループ内で呼び出し）

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

  // モーター回転方向反転フラグ
  bool rightMotorReversed;
  bool leftMotorReversed;
  
  // ロボットの物理パラメータ
  float robotWheelbase;   // 車軸間距離 [cm]
  float robotWheelRadius; // 車輪半径 [cm]
  
  // 角速度制御用の定数とフィードバック制御
  static const int BASE_TURN_SPEED = 100;      // 基本旋回速度 [PWM値]
  
  // 定数は実装ファイルで定義
  static const float NOMINAL_TURN_RATE; // ノミナル旋回角速度 [deg/s]
  
  // ジャイロフィードバック制御用
  GyroSensor* gyroSensor;
  float targetAngularVelocity;
  float currentLinearSpeed;
  bool angularControlActive;
  unsigned long lastControlUpdate;
  
  // PID制御パラメータ
  float angularKp;        // 比例ゲイン
  float angularKi;        // 積分ゲイン  
  float angularKd;        // 微分ゲイン
  float angularErrorSum;  // 積分項
  float angularLastError; // 前回誤差

  void rightWheelForward(int speed);
  void rightWheelBackward(int speed);
  void rightWheelStop();
  void leftWheelForward(int speed);
  void leftWheelBackward(int speed);
  void leftWheelStop();
  
  // 角速度から車輪速度差を計算（フィードバック制御版）
  void calculateWheelSpeeds(float linearSpeed, float angularVelocityDeg, int& leftSpeed, int& rightSpeed);
  float angularPIDControl(float targetAngular, float currentAngular);
};

#endif // MOTOR_CONTROL_H
