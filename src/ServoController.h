#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <Arduino.h>
#include <Servo.h>

class ServoController {
public:
  ServoController();
  void init();                            // サーボの初期化
  void setAngle(int angle);               // 角度指定（0-180度）
  void sweep();                           // 簡単なスイープ動作
  int getCurrentAngle();                  // 現在の角度を取得

private:
  static const int SERVO_PIN = 3;        // D3ピン（Timer2使用）
  
  Servo servo;                            // Servoライブラリのインスタンス
  int currentAngle;                       // 現在の角度
};

#endif // SERVO_CONTROLLER_H
