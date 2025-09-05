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
  
  // 角速度ベース制御関数（ジャイロフィードバック版） / 角速度基础控制函数（陀螺仪反馈版）
  void moveWithAngularVelocity(float linearSpeed, float targetAngularVelocity); // [deg/s]
  void turnAtAngularVelocity(float targetAngularVelocity); // [deg/s] 正=右旋回、負=左旋回 / 正=右转，负=左转
  void moveStraight(float linearSpeed); // 直進（角速度=0） / 直行（角速度=0）
  void setRobotParameters(float wheelbase, float wheelRadius); // ロボットパラメータ設定 / 机器人参数设置
  void setGyroSensor(GyroSensor* gyro); // ジャイロセンサー設定 / 陀螺仪传感器设置
  void updateAngularControl(); // 角速度制御の更新（ループ内で呼び出し） / 角速度控制更新（在循环中调用）

private:
  // L298Nモータードライバのピン定義 / L298N电机驱动器引脚定义
  static const int ENA = 9;   // D9ピン - 右車輪モーター速度制御（PWM） / D9引脚 - 右轮电机速度控制（PWM）
  static const int ENB = 10;  // D10ピン - 左車輪モーター速度制御（PWM） / D10引脚 - 左轮电机速度控制（PWM）
  // OutputA（右車輪）制御用 / OutputA（右轮）控制用
  static const int IN1 = 4;   // D4ピン - 右車輪モーター制御ピン1 / D4引脚 - 右轮电机控制引脚1
  static const int IN2 = 5;   // D5ピン - 右車輪モーター制御ピン2 / D5引脚 - 右轮电机控制引脚2
  // OutputB（左車輪）制御用 / OutputB（左轮）控制用
  static const int IN3 = 6;   // D6ピン - 左車輪モーター制御ピン3 / D6引脚 - 左轮电机控制引脚3
  static const int IN4 = 7;   // D7ピン - 左車輪モーター制御ピン4 / D7引脚 - 左轮电机控制引脚4

  // モーター回転方向反転フラグ / 电机旋转方向反转标志
  bool rightMotorReversed;
  bool leftMotorReversed;
  
  // ロボットの物理パラメータ / 机器人物理参数
  float robotWheelbase;   // 車軸間距離 [cm] / 轮轴间距离 [cm]
  float robotWheelRadius; // 車輪半径 [cm] / 车轮半径 [cm]
  
  // 角速度制御用の定数とフィードバック制御 / 角速度控制用常数和反馈控制
  static const int BASE_TURN_SPEED = 100;      // 基本旋回速度 [PWM値] / 基本转向速度 [PWM值]
  
  // 定数は実装ファイルで定義 / 常数在实现文件中定义
  static const float NOMINAL_TURN_RATE; // ノミナル旋回角速度 [deg/s] / 标称转向角速度 [deg/s]
  
  // ジャイロフィードバック制御用 / 陀螺仪反馈控制用
  GyroSensor* gyroSensor;
  float targetAngularVelocity;
  float currentLinearSpeed;
  bool angularControlActive;
  unsigned long lastControlUpdate;
  
  // PID制御パラメータ / PID控制参数
  float angularKp;        // 比例ゲイン / 比例增益
  float angularKi;        // 積分ゲイン / 积分增益  
  float angularKd;        // 微分ゲイン / 微分增益
  float angularErrorSum;  // 積分項 / 积分项
  float angularLastError; // 前回誤差 / 上次误差
  unsigned long lastPidUpdate; // PID計算用のタイムスタンプ / PID计算时间戳

  void rightWheelForward(int speed);
  void rightWheelBackward(int speed);
  void rightWheelStop();
  void leftWheelForward(int speed);
  void leftWheelBackward(int speed);
  void leftWheelStop();
  
  // 角速度から車輪速度差を計算（フィードバック制御版） / 从角速度计算车轮速度差（反馈控制版）
  void calculateWheelSpeeds(float linearSpeed, float angularVelocityDeg, int& leftSpeed, int& rightSpeed);
  float angularPIDControl(float targetAngular, float currentAngular);
};

#endif // MOTOR_CONTROL_H
