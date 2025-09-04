#include <Arduino.h>

#include "MotorControl.h"
#include "PIDController.h"
#include "PixyCam.h"
#include "UltrasonicSensor.h"

MotorControl motor;
PixyCam pixy;
PIDController pid(0.01, 0.0, 0.0);  // PIDゲイン（Kp, Ki, Kd）は要調整
UltrasonicSensor ultrasonic;

const int BASE_SPEED   = 150;  // ロボットの基本速度
const int SEARCH_SPEED = 100;  // 探索時の旋回速度

// 探索モードの状態を管理する変数
enum SearchState { NOT_SEARCHING, TURNING_LEFT, PAUSING_AFTER_LEFT, TURNING_RIGHT, PAUSING_AFTER_RIGHT, MOVING_BACKWARD, PAUSING_AFTER_BACKWARD };
SearchState searchState       = NOT_SEARCHING;
unsigned long stateChangeTime = 0;

void setup() {
    Serial.begin(9600);
    motor.init();
    pixy.init();
    ultrasonic.init();

    // モーターの回転方向を設定（必要に応じてtrueに変更）
    // motor.setMotorDirection(false, false); // 両方とも正常
    // motor.setMotorDirection(true, false);  // 右モーターのみ逆転
    // motor.setMotorDirection(false, true);  // 左モーターのみ逆転
    // motor.setMotorDirection(true, true);   // 両方とも逆転

    Serial.println("=== Setup Test: Left/Right Turn ===");

    // 左旋回テスト
    Serial.println("Testing Left Turn...");
    motor.turnLeft(150);
    delay(1000);
    motor.stopRobot();
    delay(500);

    // 右旋回テスト
    Serial.println("Testing Right Turn...");
    motor.turnRight(150);
    delay(1000);
    motor.stopRobot();
    delay(500);

    Serial.println("Setup test completed. Starting main program...");

    motor.move(100, 0);
}

void loop() {
    int32_t x_offset          = pixy.getXOffset();
    unsigned long currentTime = millis();

    if (x_offset != -1) {
        // --- オブジェクトを検出した場合 ---
        if (searchState != NOT_SEARCHING) {
            // 探索モードだった場合は解除
            searchState = NOT_SEARCHING;
            pid.reset();  // PIDをリセット
            motor.stopRobot();
            // delay(3000);
            Serial.println("Object found, tracking started.");
        }

        // 超音波センサーで障害物をチェック
        float leftDist, centerDist, rightDist;
        ultrasonic.getAllDistances(leftDist, centerDist, rightDist);

        // 障害物回避の優先度が高い
        if (ultrasonic.isObstacleDetected(5.0)) {
            // 15cm以内に障害物がある場合
            Serial.println("Obstacle detected! Avoiding...");
            Serial.print("Distances - L: ");
            Serial.print(leftDist);
            Serial.print("cm, C: ");
            Serial.print(centerDist);
            Serial.print("cm, R: ");
            Serial.print(rightDist);
            Serial.println("cm");

            // 障害物回避動作（一時的に停止して左に旋回）
            motor.stopRobot();
            delay(200);
            motor.turnLeft(SEARCH_SPEED);
            delay(500);
            motor.stopRobot();
            delay(200);
        } else {
            // 障害物がない場合、通常のPID制御でオブジェクトを追跡
            float correction = pid.calculate(0, x_offset) / 60.0;
            motor.move(BASE_SPEED, correction);

            Serial.print("Tracking - X Offset: ");
            Serial.print(x_offset);
            Serial.print(", Correction: ");
            Serial.print(correction);
            Serial.print(", Distances - L: ");
            Serial.print(leftDist);
            Serial.print("cm, C: ");
            Serial.print(centerDist);
            Serial.print("cm, R: ");
            Serial.print(rightDist);
            Serial.println("cm");
        }

    } else {
        float leftDist, centerDist, rightDist;
        ultrasonic.getAllDistances(leftDist, centerDist, rightDist);
        // --- オブジェクトを検出しなかった場合（探索モード） ---
        switch (searchState) {
            case NOT_SEARCHING:
                // 探索モードを開始
                Serial.println("No object detected. Starting search...");
                motor.turnLeft(SEARCH_SPEED);
                searchState     = TURNING_LEFT;
                stateChangeTime = currentTime;
                break;

            case TURNING_LEFT:
                // 1秒間、左に旋回
                if (currentTime - stateChangeTime > 2000) {
                    motor.stopRobot();
                    searchState     = PAUSING_AFTER_LEFT;
                    stateChangeTime = currentTime;
                }
                break;

            case PAUSING_AFTER_LEFT:
                // 0.5秒間、停止
                if (currentTime - stateChangeTime > 500) {
                    motor.turnRight(SEARCH_SPEED);
                    searchState     = TURNING_RIGHT;
                    stateChangeTime = currentTime;
                }
                break;

            case TURNING_RIGHT:
                // 2秒間、右に旋回
                if (currentTime - stateChangeTime > 2000) {
                    motor.stopRobot();
                    searchState     = PAUSING_AFTER_RIGHT;
                    stateChangeTime = currentTime;
                }
                break;

            case PAUSING_AFTER_RIGHT:
                // 0.5秒間、停止
                if (currentTime - stateChangeTime > 500) {
                    // 後退動作を開始
                    motor.moveBackward(SEARCH_SPEED);
                    searchState     = MOVING_BACKWARD;
                    stateChangeTime = currentTime;
                }
                break;

            case MOVING_BACKWARD:
                // 1秒間、後退
                if (currentTime - stateChangeTime > 2000) {
                    motor.stopRobot();
                    searchState     = PAUSING_AFTER_BACKWARD;
                    stateChangeTime = currentTime;
                }
                break;

            case PAUSING_AFTER_BACKWARD:
                // 0.5秒間、停止
                if (currentTime - stateChangeTime > 500) {
                    // 再度、左旋回から探索を繰り返す
                    motor.turnLeft(SEARCH_SPEED);
                    searchState     = TURNING_LEFT;
                    stateChangeTime = currentTime;
                }
                break;
        }
    }
}
