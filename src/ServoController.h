#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <Arduino.h>
#include <Servo.h>

class ServoController {
public:
  ServoController();
  void init();                            // サーボの初期化 / 舵机初始化
  void setAngle(int angle);               // 角度指定（0-180度） / 角度指定（0-180度）
  void sweep();                           // 簡単なスイープ動作（ブロッキング） / 简单扫描动作（阻塞）
  void startNonBlockingSweep();           // ノンブロッキングスイープ開始 / 非阻塞扫描开始
  void updateNonBlockingSweep();          // ノンブロッキングスイープ更新（ループ内で呼び出し） / 非阻塞扫描更新（在循环中调用）
  void stopNonBlockingSweep();            // ノンブロッキングスイープ停止 / 非阻塞扫描停止
  bool isSweeping();                      // スイープ中かどうか / 是否在扫描中
  int getCurrentAngle();                  // 現在の角度を取得 / 获取当前角度

private:
  static const int SERVO_PIN = 3;        // D3ピン（Timer2使用） / D3引脚（使用Timer2）
  
  Servo servo;                            // Servoライブラリのインスタンス / Servo库实例
  int currentAngle;                       // 現在の角度 / 当前角度
  
  // ノンブロッキングスイープ用の変数 / 非阻塞扫描用变量
  bool sweepActive;                       // スイープが有効かどうか / 扫描是否有效
  int sweepStep;                          // 現在のスイープステップ / 当前扫描步骤
  unsigned long sweepLastTime;            // 最後にスイープが更新された時間 / 最后更新扫描的时间
  unsigned long sweepStepDuration;       // 各ステップの持続時間 / 每个步骤的持续时间
};

#endif // SERVO_CONTROLLER_H
