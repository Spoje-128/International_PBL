#include <Arduino.h>
#include "MotorControl.h"
#include "PixyCam.h"
#include "PIDController.h"

MotorControl motor;
PixyCam pixy;
PIDController pid(0.1, 0.01, 0.05); // PIDゲイン（Kp, Ki, Kd）は要調整

const int BASE_SPEED = 150;   // ロボットの基本速度
const int SEARCH_SPEED = 100; // 探索時の旋回速度

// 探索モードの状態を管理する変数
enum SearchState {
  NOT_SEARCHING,
  TURNING_LEFT,
  PAUSING_AFTER_LEFT,
  TURNING_RIGHT,
  PAUSING_AFTER_RIGHT
};
SearchState searchState = NOT_SEARCHING;
unsigned long stateChangeTime = 0;

void setup() {
  Serial.begin(9600);
  motor.init();
  pixy.init();
}

void loop() {
  int32_t x_offset = pixy.getXOffset();
  unsigned long currentTime = millis();

  if (x_offset != -1) {
    // --- オブジェクトを検出した場合 ---
    if (searchState != NOT_SEARCHING) {
      // 探索モードだった場合は解除
      searchState = NOT_SEARCHING;
      pid.reset(); // PIDをリセット
      Serial.println("Object found, tracking started.");
    }
    
    // PID制御でオブジェクトを追跡
    float correction = pid.calculate(0, x_offset);
    motor.move(BASE_SPEED, correction);
    
    Serial.print("X Offset: ");
    Serial.print(x_offset);
    Serial.print(", Correction: ");
    Serial.println(correction);

  } else {
    // --- オブジェクトを検出しなかった場合（探索モード） ---
    switch (searchState) {
      case NOT_SEARCHING:
        // 探索モードを開始
        Serial.println("No object detected. Starting search...");
        motor.turnLeft(SEARCH_SPEED);
        searchState = TURNING_LEFT;
        stateChangeTime = currentTime;
        break;

      case TURNING_LEFT:
        // 1秒間、左に旋回
        if (currentTime - stateChangeTime > 1000) {
          motor.stopRobot();
          searchState = PAUSING_AFTER_LEFT;
          stateChangeTime = currentTime;
        }
        break;

      case PAUSING_AFTER_LEFT:
        // 0.5秒間、停止
        if (currentTime - stateChangeTime > 500) {
          motor.turnRight(SEARCH_SPEED);
          searchState = TURNING_RIGHT;
          stateChangeTime = currentTime;
        }
        break;

      case TURNING_RIGHT:
        // 1秒間、右に旋回
        if (currentTime - stateChangeTime > 1000) {
          motor.stopRobot();
          searchState = PAUSING_AFTER_RIGHT;
          stateChangeTime = currentTime;
        }
        break;

      case PAUSING_AFTER_RIGHT:
        // 0.5秒間、停止
        if (currentTime - stateChangeTime > 500) {
          // 再度、左旋回から探索を繰り返す
          motor.turnLeft(SEARCH_SPEED);
          searchState = TURNING_LEFT;
          stateChangeTime = currentTime;
        }
        break;
    }
  }
}
