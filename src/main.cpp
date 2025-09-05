#include <Arduino.h>

#include "MotorControl.h"
#include "PIDController.h"
#include "PixyCam.h"
#include "UltrasonicSensor.h"
#include "ServoController.h"
#include "GyroSensor.h"  // ジャイロセンサーを追加

MotorControl motor;
PixyCam pixy;
PIDController pid(8.0, 0.0, 0.3);  // PIDゲイン（Kp, Ki, Kd）は要調整
UltrasonicSensor ultrasonic;
ServoController servoCont;
GyroSensor gyro;  // ジャイロセンサーのインスタンス

const int BASE_SPEED   = 50;  // ロボットの基本速度（90→50に減速）
const int SEARCH_SPEED = 40;  // 探索時の旋回速度（90→40に減速）

// 探索モードの状態を管理する変数
enum SearchState { NOT_SEARCHING, TURNING_LEFT, PAUSING_AFTER_LEFT, TURNING_RIGHT, PAUSING_AFTER_RIGHT, MOVING_BACKWARD, PAUSING_AFTER_BACKWARD };
SearchState searchState       = NOT_SEARCHING;
unsigned long stateChangeTime = 0;

// 障害物回避の状態管理変数
enum ObstacleAvoidanceState { NO_AVOIDANCE, AVOIDING_BACKWARD, AVOIDING_TURN, AVOIDING_COMPLETE };
ObstacleAvoidanceState avoidanceState = NO_AVOIDANCE;
unsigned long avoidanceStartTime = 0;

void setup() {
    Serial.begin(9600);
    motor.init();
    pixy.init();
    ultrasonic.init();
    servoCont.init();
    
    // ジャイロセンサーの初期化
    if (!gyro.init()) {
        Serial.println("Gyro initialization failed!");
        while(1);
    }
    
    // ジャイロをモーター制御に関連付け
    motor.setGyroSensor(&gyro);

    // モーターの回転方向を設定（必要に応じてtrueに変更）
    // motor.setMotorDirection(false, false); // 両方とも正常
    // motor.setMotorDirection(true, false);  // 右モーターのみ逆転
    // motor.setMotorDirection(false, true);  // 左モーターのみ逆転
    // motor.setMotorDirection(true, true);   // 両方とも逆転

    Serial.println("=== Setup Test: Left/Right Turn ===");

    // 左旋回テスト
    Serial.println("Testing Left Turn...");
    motor.turnLeft(80);  // 150→80に減速
    delay(1000);
    motor.stopRobot();
    delay(500);

    // 右旋回テスト
    Serial.println("Testing Right Turn...");
    motor.turnRight(80);  // 150→80に減速
    delay(1000);
    motor.stopRobot();
    delay(500);

    // サーボテスト
    Serial.println("Testing Servo...");
    servoCont.sweep();
    Serial.println("Setup test completed. Starting main program...");

    motor.move(60, 0);  // 100→60に減速
}

void loop() {
    // **重要**: 角速度フィードバック制御を毎回更新
    motor.updateAngularControl();
    
    int32_t x_offset          = pixy.getXOffset();
    unsigned long currentTime = millis();

    if (x_offset != -1) {
        // --- オブジェクトを検出した場合 ---
        if (searchState != NOT_SEARCHING) {
            // 探索モードだった場合は解除
            searchState = NOT_SEARCHING;
            avoidanceState = NO_AVOIDANCE; // 障害物回避もリセット
            pid.reset();  // PIDをリセット
            motor.stopRobot();
            // delay(1000); // 3秒→1秒に短縮
            Serial.println("Object found, tracking started.");
        }

        // 超音波センサーで障害物をチェック
        float leftDist, centerDist, rightDist;
        ultrasonic.getAllDistances(leftDist, centerDist, rightDist);

        // 障害物回避中でない場合のみ、通常のPID制御でオブジェクトを追跡
        if (avoidanceState == NO_AVOIDANCE) {
            // 障害物検出チェック
            if (ultrasonic.isObstacleDetected(7.0)) {
                // 障害物が検出された場合、回避開始
                Serial.println("Obstacle detected during tracking! Starting avoidance...");
                Serial.print("Distances - L: ");
                Serial.print(leftDist);
                Serial.print("cm, C: ");
                Serial.print(centerDist);
                Serial.print("cm, R: ");
                Serial.print(rightDist);
                Serial.println("cm");

                avoidanceState = AVOIDING_BACKWARD;
                avoidanceStartTime = currentTime;
                motor.moveBackward(SEARCH_SPEED);
                Serial.println("Starting obstacle avoidance - moving backward");
            } else {
                // 障害物がない場合、PID制御でオブジェクトを追跡
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
        }
        
        // 障害物回避の状態管理（回避中の場合のみ実行）
        if (avoidanceState != NO_AVOIDANCE) {
            switch (avoidanceState) {
                case AVOIDING_BACKWARD:
                    if (currentTime - avoidanceStartTime > 1000) { // 1秒後退
                        avoidanceState = AVOIDING_TURN;
                        avoidanceStartTime = currentTime;
                        
                        // 障害物の位置に応じて旋回方向を決定
                        if (leftDist < 7.0) {
                            Serial.println("Turning right to avoid left obstacle");
                            motor.turnRight(SEARCH_SPEED);
                        } else if (rightDist < 7.0) {
                            Serial.println("Turning left to avoid right obstacle");
                            motor.turnLeft(SEARCH_SPEED);
                        } else {
                            Serial.println("Turning right to avoid center obstacle");
                            motor.turnRight(SEARCH_SPEED);
                        }
                    }
                    break;
                    
                case AVOIDING_TURN:
                    if (currentTime - avoidanceStartTime > 500) { // 0.5秒旋回
                        avoidanceState = AVOIDING_COMPLETE;
                        motor.stopRobot();
                        Serial.println("Obstacle avoidance completed");
                    }
                    break;
                    
                case AVOIDING_COMPLETE:
                    // 回避完了、通常動作に戻る
                    avoidanceState = NO_AVOIDANCE;
                    Serial.println("Returning to object tracking");
                    break;
            }
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
                // 0.2秒間、左に旋回
                if (currentTime - stateChangeTime > 220) {
                    motor.stopRobot();
                    searchState     = PAUSING_AFTER_LEFT;
                    stateChangeTime = currentTime;
                }
                break;

            case PAUSING_AFTER_LEFT:
                // 0.2秒間、停止
                if (currentTime - stateChangeTime > 220) {
                    // 探索中にロボットアームでスキャン
                    motor.turnRight(SEARCH_SPEED);
                    searchState     = TURNING_RIGHT;
                    stateChangeTime = currentTime;
                }
                break;

            case TURNING_RIGHT:
                // 0.5秒間、右に旋回
                if (currentTime - stateChangeTime > 220) {
                    motor.stopRobot();
                    searchState     = PAUSING_AFTER_RIGHT;
                    stateChangeTime = currentTime;
                }
                break;

            case PAUSING_AFTER_RIGHT:
                // 0.5秒間、停止（この時にサーボでスキャン）
                if (currentTime - stateChangeTime > 220) {
                    // 探索中にロボットアームでスキャン
                    // servoCont.setAngle(135); // 反対方向にスキャン
                    // 後退動作を開始
                    motor.moveBackward(SEARCH_SPEED);
                    searchState     = MOVING_BACKWARD;
                    stateChangeTime = currentTime;
                }
                break;

            case MOVING_BACKWARD:
                // 0.3秒間、後退
                if (currentTime - stateChangeTime > 220) {
                    motor.stopRobot();
                    searchState     = PAUSING_AFTER_BACKWARD;
                    stateChangeTime = currentTime;
                }
                break;

            case PAUSING_AFTER_BACKWARD:
                // 0.5秒間、停止
                if (currentTime - stateChangeTime > 220) {
                    // 再度、左旋回から探索を繰り返す
                    motor.turnLeft(SEARCH_SPEED);
                    searchState     = TURNING_LEFT;
                    stateChangeTime = currentTime;
                }
                break;
        }
    }
}
