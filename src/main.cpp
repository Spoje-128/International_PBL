#include <Arduino.h>

#include "MotorControl.h"
#include "PixyCam.h"
#include "UltrasonicSensor.h"
#include "ServoController.h"
#include "GyroSensor.h"  // ジャイロセンサーを追加 / 添加陀螺仪传感器

MotorControl motor;
PixyCam pixy;
UltrasonicSensor ultrasonic;
ServoController servoCont;
GyroSensor gyro;  // ジャイロセンサーのインスタンス / 陀螺仪传感器实例

const int BASE_SPEED   = 60;  // ロボットの基本速度（60→45に減速）/ 机器人基本速度（60→45减速）
const int SEARCH_SPEED = 50;  // 探索時の旋回速度（50→35に減速）/ 搜索时的旋转速度（50→35减速）

// デバッグ出力制御用変数 / 调试输出控制变量
unsigned long lastDebugTime = 0;
const unsigned long DEBUG_INTERVAL = 500; // 500ms間隔でデバッグ出力 / 500ms间隔调试输出

// サーボ制御用変数 / 舵机控制变量
bool objectTrackingActive = false;

// 探索モードの状態を管理する変数（シンプルな左右探索） / 管理搜索模式状态的变量（简单的左右搜索）
enum SearchState { NOT_SEARCHING, SEARCHING_LEFT, SEARCHING_RIGHT };
SearchState searchState       = NOT_SEARCHING;
unsigned long stateChangeTime = 0;

// 障害物回避の状態管理変数 / 障碍物回避状态管理变量
enum ObstacleAvoidanceState { NO_AVOIDANCE, AVOIDING_BACKWARD, AVOIDING_TURN, AVOIDING_COMPLETE };
ObstacleAvoidanceState avoidanceState = NO_AVOIDANCE;
unsigned long avoidanceStartTime = 0;

void setup() {
    Serial.begin(9600);
    motor.init();
    pixy.init();
    ultrasonic.init();
    servoCont.init();
    
    // ジャイロセンサーの初期化 / 陀螺仪传感器初始化
    if (!gyro.init()) {
        Serial.println("Gyro initialization failed!");
        while(1);
    }
    
    // ジャイロをモーター制御に関連付け / 将陀螺仪关联到电机控制
    motor.setGyroSensor(&gyro);

    // モーターの回転方向を設定（必要に応じてtrueに変更） / 设置电机旋转方向（必要时改为true）
    // motor.setMotorDirection(false, false); // 両方とも正常 / 两个都正常
    // motor.setMotorDirection(true, false);  // 右モーターのみ逆転 / 仅右电机反转
    // motor.setMotorDirection(false, true);  // 左モーターのみ逆転 / 仅左电机反转
    // motor.setMotorDirection(true, true);   // 両方とも逆転 / 两个都反转

    Serial.println("=== Initial Movement Sequence ===");
    
    // 1. 左に90度旋回 / 1. 向左旋转90度
    Serial.println("Step 1: Left turn 90 degrees");
    motor.turnLeft(45);
    delay(1500);  // 90度旋回のための時間 / 90度旋转所需时间
    motor.stopRobot();
    delay(500);
    
    // 2. 壁まで前進 / 2. 向前行进到墙壁
    Serial.println("Step 2: Move forward to wall");
    bool wallReached = false;
    while (!wallReached) {
        motor.updateAngularControl(); // 角速度制御更新 / 角速度控制更新
        
        float leftDist, centerDist, rightDist;
        ultrasonic.getAllDistances(leftDist, centerDist, rightDist);
        
        if (centerDist < 10.0) { // 壁まで10cm / 距墙壁10cm
            motor.stopRobot();
            wallReached = true;
            Serial.print("Wall reached at ");
            Serial.print(centerDist);
            Serial.println("cm");
        } else {
            motor.moveWithAngularVelocity(30.0, 0.0); // より低速で安定前進（40.0→30.0）/ 更低速稳定前进
        }
        delay(50); // 制御周期 / 控制周期
    }
    delay(1000);
    
    // 3. 右に180度回頭 / 3. 向右转头180度
    Serial.println("Step 3: Right turn 180 degrees");
    motor.turnRight(45);
    delay(3000);  // 180度旋回のための時間 / 180度旋转所需时间
    motor.stopRobot();
    delay(500);
    
    // 4. 再び壁まで前進 / 4. 再次向前行进到墙壁
    Serial.println("Step 4: Move forward to opposite wall");
    wallReached = false;
    while (!wallReached) {
        motor.updateAngularControl(); // 角速度制御更新 / 角速度控制更新
        
        float leftDist, centerDist, rightDist;
        ultrasonic.getAllDistances(leftDist, centerDist, rightDist);
        
        if (centerDist < 10.0) { // 壁まで10cm / 距墙壁10cm
            motor.stopRobot();
            wallReached = true;
            Serial.print("Opposite wall reached at ");
            Serial.print(centerDist);
            Serial.println("cm");
        } else {
            motor.moveWithAngularVelocity(30.0, 0.0); // より低速で安定前進（40.0→30.0）/ 更低速稳定前进
        }
        delay(50); // 制御周期 / 控制周期
    }
    delay(1000);

    Serial.println("Initial sequence completed. Starting main program...");
    motor.stopRobot();
}

void loop() {
    // **重要**: 角速度feedback制御を毎回更新 / **重要**: 每次更新角速度反馈控制
    motor.updateAngularControl();
    
    // **重要**: ノンブロッキングサーボ制御を毎回更新 / **重要**: 每次更新非阻塞舵机控制
    servoCont.updateNonBlockingSweep();
    
    int32_t x_offset          = pixy.getXOffset();
    unsigned long currentTime = millis();
    
    // 常に超音波センサーで障害物をチェック（並行動作） / 始终用超声波传感器检查障碍物（并行操作）
    float leftDist, centerDist, rightDist;
    ultrasonic.getAllDistances(leftDist, centerDist, rightDist);
    
    // 障害物検知処理（オブジェクト检出有無に関わらず実行） / 障碍物检测处理（无论是否检出对象都执行）
    // 閾値を5.0cmに下げる / 阈值降至5.0cm
    if (avoidanceState == NO_AVOIDANCE && ultrasonic.isObstacleDetected(5.0)) {
        // 障害物が検出された場合、回避開始 / 检测到障碍物时开始回避
        avoidanceState = AVOIDING_BACKWARD;
        avoidanceStartTime = currentTime;
        motor.moveBackward(SEARCH_SPEED);
        
        // 簡潔なデバッグ出力 / 简洁的调试输出
        Serial.print("AVOID START - L:");
        Serial.print(leftDist, 0);
        Serial.print(" C:");
        Serial.print(centerDist, 0);
        Serial.print(" R:");
        Serial.println(rightDist, 0);
    }

    if (x_offset != -1) {
        // --- オブジェクトを検出した場合 --- / --- 检测到对象的情况 ---
        if (searchState != NOT_SEARCHING) {
            // 探索モードだった場合は解除 / 如果是搜索模式则解除
            searchState = NOT_SEARCHING;
            motor.stopRobot();
            delay(1000);
            Serial.println("Object found, tracking started.");
        }
        
        // オブジェクト追跡中のサーボ制御 / 对象追踪中的舵机控制
        if (!objectTrackingActive) {
            objectTrackingActive = true;
            servoCont.startNonBlockingSweep();
        }

        // 障害物回避中でない場合のみ、通常のPID制御でオブジェクトを追跡 / 仅在非障碍物回避中时，用正常PID控制追踪对象
        if (avoidanceState == NO_AVOIDANCE) {
                // 障害物がない場合、比例制御でオブジェクトを追跡
                // When there are no obstacles, track the object with proportional control.
                // x_offsetから直接目標角速度を計算（外側PIDを撤廃）
                // Calculate target angular velocity directly from x_offset (outer PID removed)
                // K_angularは調整用ゲイン。x_offsetが最大(160)の時に目標角速度が約32deg/sになるように設定。
                // K_angular is a gain for adjustment. Set so that the target angular velocity is approx. 32deg/s when x_offset is max (160).
                const float K_angular = 0.75f;
                float targetAngularVelocity = K_angular * x_offset;

                // 距離に応じた前進速度調整 / 根据距离调整前进速度
                float forwardSpeed;
                if (abs(x_offset) < 80) {
                    // 中程度のずれ：中程度の速度
                    forwardSpeed = BASE_SPEED * 0.5;
                } else {
                    // 大きくずれている場合：低速で慎重に
                    forwardSpeed = BASE_SPEED * 0.3;
                }
                
                motor.moveWithAngularVelocity(forwardSpeed, targetAngularVelocity);
                
                // デバッグ出力（500ms間隔で制限）
                if (currentTime - lastDebugTime > DEBUG_INTERVAL) {
                    Serial.print("TRACK - X:");
                    Serial.print(x_offset);
                    Serial.print(" TargetAng:");
                    Serial.print(targetAngularVelocity, 1);
                    Serial.print(" Speed:");
                    Serial.print(forwardSpeed, 1);
                    Serial.print(" Dist L:");
                    Serial.print(leftDist, 0);
                    Serial.print(" C:");
                    Serial.print(centerDist, 0);
                    Serial.print(" R:");
                    Serial.println(rightDist, 0);
                    lastDebugTime = currentTime;
                }
            }
        
    } else {
        // --- オブジェクトを検出しなかった場合（探索モード） ---
        
        // オブジェクト追跡が非アクティブの場合、サーボを停止
        if (objectTrackingActive) {
            objectTrackingActive = false;
            servoCont.stopNonBlockingSweep();
        }
        
        // 障害物回避中でない場合のみ探索動作を実行
        if (avoidanceState == NO_AVOIDANCE) {
            switch (searchState) {
                case NOT_SEARCHING:
                    // 探索モードを開始
                    Serial.println("SEARCH START");
                    motor.turnLeft(SEARCH_SPEED);
                    searchState     = SEARCHING_LEFT;
                    stateChangeTime = currentTime;
                    break;

                case SEARCHING_LEFT:
                    // 1.5秒間、左に旋回
                    if (currentTime - stateChangeTime > 500) {
                        motor.turnRight(SEARCH_SPEED);
                        searchState     = SEARCHING_RIGHT;
                        stateChangeTime = currentTime;
                    }
                    break;

                case SEARCHING_RIGHT:
                    // 3.0秒間、右に旋回（左旋回分を戻す + 右探索）
                    if (currentTime - stateChangeTime > 500) {
                        motor.turnLeft(SEARCH_SPEED);
                        searchState     = SEARCHING_LEFT;
                        stateChangeTime = currentTime;
                    }
                    break;
            }
            
            // 探索中のデバッグ出力（500ms間隔で制限）
            if (currentTime - lastDebugTime > DEBUG_INTERVAL) {
                Serial.print("SEARCH - State:");
                Serial.print(searchState);
                Serial.print(" Dist L:");
                Serial.print(leftDist, 0);
                Serial.print(" C:");
                Serial.print(centerDist, 0);
                Serial.print(" R:");
                Serial.println(rightDist, 0);
                lastDebugTime = currentTime;
            }
        }
    }
    
    // 障害物回避の状態管理（常に実行 - 並行動作）
    if (avoidanceState != NO_AVOIDANCE) {
        switch (avoidanceState) {
            case AVOIDING_BACKWARD:
                if (currentTime - avoidanceStartTime > 100) { // 0.5秒→0.3秒に短縮
                    avoidanceState = AVOIDING_TURN;
                    avoidanceStartTime = currentTime;
                    
                    // 改善された障害物回避ロジック - 後退を最小限に
                    if (leftDist > rightDist && leftDist > 8.0) {
                        // 左に十分な空間がある場合
                        Serial.println("AVOID LEFT (left has more space)");
                        motor.turnLeft(SEARCH_SPEED);
                    } else if (rightDist > leftDist && rightDist > 8.0) {
                        // 右に十分な空間がある場合
                        Serial.println("AVOID RIGHT (right has more space)");
                        motor.turnRight(SEARCH_SPEED);
                    } else if (leftDist > 6.0) {
                        // 左にある程度の空間がある場合
                        Serial.println("AVOID LEFT (left available)");
                        motor.turnLeft(SEARCH_SPEED);
                    } else if (rightDist > 6.0) {
                        // 右にある程度の空間がある場合
                        Serial.println("AVOID RIGHT (right available)");
                        motor.turnRight(SEARCH_SPEED);
                    } else {
                        // 左右ともに狭い場合のみ、より短い後退
                        Serial.println("AVOID BRIEF BACKWARD (all sides tight)");
                        motor.moveBackward(SEARCH_SPEED);
                    }
                }
                break;
                
            case AVOIDING_TURN:
                if (currentTime - avoidanceStartTime > 300) { // 0.4秒旋回
                    avoidanceState = AVOIDING_COMPLETE;
                    motor.stopRobot();
                    Serial.println("AVOID COMPLETE");
                }
                break;
                
            case AVOIDING_COMPLETE:
                // 回避完了、通常動作に戻る
                avoidanceState = NO_AVOIDANCE;
                Serial.println("RETURN TO NORMAL");
                break;
        }
    }
}
