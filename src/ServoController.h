#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <Arduino.h>
#include <Servo.h>

class ServoController {
public:
  ServoController();
  void init();                            // サーボの初期化 / 舵機初始化
  void setAngle(int angle);               // 角度指定（0-180度） / 角度指定（0-180度）
  void sweep();                           // 簡単なスイープ動作（ブロッキング） / 簡單掃描動作（阻塞）
  void startNonBlockingSweep();           // ノンブロッキングスイープ開始 / 非阻塞掃描開始
  void updateNonBlockingSweep();          // ノンブロッキングスイープ更新（ループ内で呼び出し） / 非阻塞掃描更新（在循環中調用）
  void stopNonBlockingSweep();            // ノンブロッキングスイープ停止 / 非阻塞掃描停止
  bool isSweeping();                      // スイープ中かどうか / 是否在掃描中
  int getCurrentAngle();                  // 現在の角度を取得 / 獲取當前角度

private:
  static const int SERVO_PIN = 3;        // D3ピン（Timer2使用） / D3引腳（使用Timer2）
  
  Servo servo;                            // Servoライブラリのインスタンス / Servo庫實例
  int currentAngle;                       // 現在の角度 / 當前角度
  
  // ノンブロッキングスイープ用の変数 / 非阻塞掃描用變量
  bool sweepActive;                       // スイープが有効かどうか / 掃描是否有效
  int sweepStep;                          // 現在のスイープステップ / 當前掃描步驟
  unsigned long sweepLastTime;            // 最後にスイープが更新された時間 / 最後更新掃描的時間
  unsigned long sweepStepDuration;       // 各ステップの持続時間 / 每個步驟的持續時間
};

#endif // SERVO_CONTROLLER_H
