#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// 前方宣言 / 前向声明
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
  
  // 角速度ベース制御関数（ジャイロフィードバック版） / 角速度基礎控制函數（陀螺儀反饋版）
  void moveWithAngularVelocity(float linearSpeed, float targetAngularVelocity); // [deg/s]
  void turnAtAngularVelocity(float targetAngularVelocity); // [deg/s] 正=右旋回、負=左旋回 / 正=右轉，負=左轉
  void moveStraight(float linearSpeed); // 直進（角速度=0） / 直行（角速度=0）
  void setRobotParameters(float wheelbase, float wheelRadius); // ロボットパラメータ設定 / 機器人參數設置
  void setGyroSensor(GyroSensor* gyro); // ジャイロセンサー設定 / 陀螺儀傳感器設置
  void updateAngularControl(); // 角速度制御の更新（ループ内で呼び出し） / 角速度控制更新（在循環中調用）
  
  // テスト用の直接モーター制御関数 / 測試用直接電機控制函數
  void testTurnLeftDirect(int pwmValue = 200);   // 直接左旋回テスト
  void testTurnRightDirect(int pwmValue = 200);  // 直接右旋回テスト

private:
  // L298Nモータードライバのピン定義 / L298N電機驅動器引腳定義
  static const int ENA = 9;   // D9ピン - 右車輪モーター速度制御（PWM） / D9引腳 - 右輪電機速度控制（PWM）
  static const int ENB = 10;  // D10ピン - 左車輪モーター速度制御（PWM） / D10引腳 - 左輪電機速度控制（PWM）
  // OutputA（右車輪）制御用 / OutputA（右輪）控制用
  static const int IN1 = 4;   // D4ピン - 右車輪モーター制御ピン1 / D4引腳 - 右輪電機控制引腳1
  static const int IN2 = 5;   // D5ピン - 右車輪モーター制御ピン2 / D5引腳 - 右輪電機控制引腳2
  // OutputB（左車輪）制御用 / OutputB（左輪）控制用
  static const int IN3 = 6;   // D6ピン - 左車輪モーター制御ピン3 / D6引腳 - 左輪電機控制引腳3
  static const int IN4 = 7;   // D7ピン - 左車輪モーター制御ピン4 / D7引腳 - 左輪電機控制引腳4

  // モーター回転方向反転フラグ / 電機旋轉方向反轉標誌
  bool rightMotorReversed;
  bool leftMotorReversed;
  
  // ロボットの物理パラメータ / 機器人物理參數
  float robotWheelbase;   // 車軸間距離 [cm] / 輪軸間距離 [cm]
  float robotWheelRadius; // 車輪半径 [cm] / 車輪半徑 [cm]
  
  // 角速度制御用の定数とフィードバック制御 / 角速度控制用常數和反饋控制
  static const int BASE_TURN_SPEED = 100;      // 基本旋回速度 [PWM値] / 基本轉向速度 [PWM值]

  // 定数は実装ファイルで定義 / 常數在實現文件中定義
  static const float NOMINAL_TURN_RATE; // ノミナル旋回角速度 [deg/s] / 標稱轉向角速度 [deg/s]
  
  // ジャイロフィードバック制御用 / 陀螺儀反饋控制用
  GyroSensor* gyroSensor;
  float targetAngularVelocity;
  float currentLinearSpeed;
  bool angularControlActive;
  unsigned long lastControlUpdate;
  
  // PID制御パラメータ / PID控制參數
  float angularKp;        // 比例ゲイン / 比例增益
  float angularKi;        // 積分ゲイン / 積分增益  
  float angularKd;        // 微分ゲイン / 微分增益
  float angularErrorSum;  // 積分項 / 積分項
  float angularLastError; // 前回誤差 / 上次誤差
  unsigned long lastPidUpdate; // PID計算用のタイムスタンプ / PID計算時間戳

  void rightWheelForward(int speed);
  void rightWheelBackward(int speed);
  void rightWheelStop();
  void leftWheelForward(int speed);
  void leftWheelBackward(int speed);
  void leftWheelStop();
  
  // 角速度から車輪速度差を計算（フィードバック制御版） / 從角速度計算車輪速度差（反饋控制版）
  void calculateWheelSpeeds(float linearSpeed, float angularVelocityDeg, int& leftSpeed, int& rightSpeed);
  float angularPIDControl(float targetAngular, float currentAngular);
};

#endif // MOTOR_CONTROL_H
