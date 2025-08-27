#include <Arduino.h>
#include "MotorControl.h"

MotorControl motor;

void setup() {
  // シリアル通信の初期化（デバッグ用）
  Serial.begin(9600);
  motor.init();
}

void loop() {
  // ロボットの動作テストを実行
  motor.robotTest();
}
