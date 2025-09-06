#include <Arduino.h>

#include "GyroSensor.h"  // ジャイロセンサーを追加 / 添加陀螺儀傳感器
#include "MotorControl.h"
#include "PixyCam.h"
#include "ServoController.h"
#include "UltrasonicSensor.h"

MotorControl motor;
PixyCam pixy;
UltrasonicSensor ultrasonic;
ServoController servoCont;
GyroSensor gyro;  // ジャイロセンサーのインスタンス / 陀螺儀傳感器實例

const int BASE_SPEED   = 120;  // ロボットの基本速度（確実に回転するために上げる）/ 機器人基本速度
const int SEARCH_SPEED = 120;  // 探索時の旋回速度（確実に回転するために上げる）/ 搜索時的旋轉速度

// デバッグ出力制御用変数 / 調試輸出控制變量
unsigned long lastDebugTime        = 0;
const unsigned long DEBUG_INTERVAL = 500;  // 500ms間隔でデバッグ出力 / 500ms間隔調試輸出

// サーボ制御用変数 / 舵機控制變量
bool objectTrackingActive = false;
unsigned long lastTargetSeenTime = 0; // 最後にターゲットを検出した時刻 / Last time target seen
const unsigned long TARGET_LOST_TIMEOUT_MS = 600; // 未検出継続で追跡終了 / Timeout to stop tracking

// 探索モードの状態を管理する変数（シンプルな左右探索） / 管理搜索模式狀態的變量（簡單的左右搜索）
enum SearchState { NOT_SEARCHING, SEARCHING_LEFT, SEARCHING_RIGHT };
SearchState searchState       = NOT_SEARCHING;
unsigned long stateChangeTime = 0;

// 障害物回避の状態管理変数 / 障礙物回避狀態管理變量
enum ObstacleAvoidanceState { NO_AVOIDANCE, AVOIDING_BACKWARD, AVOIDING_TURN, AVOIDING_EXTRA_TURN, AVOIDING_FORWARD, AVOIDING_STUCK_BACKWARD, AVOIDING_STUCK_PIVOT, AVOIDING_COMPLETE };
ObstacleAvoidanceState avoidanceState = NO_AVOIDANCE;
unsigned long avoidanceStartTime      = 0;
unsigned long avoidanceCompleteTime   = 0;  // 回避完了時刻を記録
// 自適応回避用変数 / Adaptive avoidance variables
int avoidanceAttempt = 0; // 連続試行回数
unsigned long dynBackwardDur = 600; // 動的後退時間
unsigned long dynTurnDur = 2000;    // 動的旋回時間
unsigned long dynExtraTurnDur = 1000; // 動的追加旋回
unsigned long dynForwardDur = 1500; // 動的前進
// 詰まり判定 / Stuck detection
static const float STUCK_CENTER_THRESH = 5.0f; // 中央5cm未満
static const float STUCK_SIDE_THRESH   = 8.0f; // 両側8cm未満
static const float STUCK_RELEASE_SIDE  = 12.0f; // 片側12cm超で解放
static const int   STUCK_MAX_PIVOTS    = 3; // 最大交互ピボット回数
int stuckPivotCount = 0;
bool stuckPivotLeft = true; // 次に回る方向（交互）
unsigned long stuckPivotStart = 0;
const unsigned long STUCK_PIVOT_DURATION = 700; // 各ピボット長さ(ms)
// クリア判定ヒステリシス / Clearance hysteresis
int clearConsecutiveCount = 0; // 連続クリアサンプル数
const int CLEAR_REQUIRED = 2; // 何回連続でクリアなら完了

// ================= ヘルパー定義（壁追従 + 障害物回避統合） / 輔助函數定義（牆面追隨 + 障礙回避整合） =================

// 壁追従用パラメータ / 牆面追隨參數
static const float WALL_SENSOR_ANGLE     = 26.5f;  // 取付角度
static const float WALL_FOLLOW_DISTANCE  = 10.0f;  // 目標距離
static const float WALL_DETECT_THRESHOLD = 25.0f;  // 壁判定
static const float WALL_FOLLOW_KP        = 0.8f;   // 比例ゲイン(過大補正抑制)
static const float WALL_FOLLOW_MAX_CORR  = 10.0f;  // 補正上限

// 障害物回避トリガ閾値（中央） / 障礙觸發閾值（中央）
static const float OBSTACLE_CENTER_TRIG = 6.0f;  // cm
static const float AVOID_CLEAR_CENTER = 14.0f; // この距離超で前方クリア判定
static const int   AVOID_MAX_ATTEMPTS = 4; // 最大連続試行

// 壁追従補正計算（相対ヨー角を加味） / 計算牆面追隨補正（含相對偏航角）
float computeWallFollowCorrection(float leftDist, float rightDist, float relativeYaw) {
    float leftActual  = 0.0f;
    float rightActual = 0.0f;
    if (leftDist > 0) leftActual = leftDist * sin(radians(WALL_SENSOR_ANGLE + relativeYaw));
    if (rightDist > 0) rightActual = rightDist * sin(radians(WALL_SENSOR_ANGLE - relativeYaw));

    bool leftWall  = (leftActual > 0 && leftActual < WALL_DETECT_THRESHOLD);
    bool rightWall = (rightActual > 0 && rightActual < WALL_DETECT_THRESHOLD);

    float corr = 0.0f;
    if (leftWall && !rightWall) {
        float err = leftActual - WALL_FOLLOW_DISTANCE;
        corr      = err * WALL_FOLLOW_KP;
    } else if (rightWall && !leftWall) {
        float err = rightActual - WALL_FOLLOW_DISTANCE;
        corr      = -err * WALL_FOLLOW_KP;  // 右側は符号反転
    }
    return constrain(corr, -WALL_FOLLOW_MAX_CORR, WALL_FOLLOW_MAX_CORR);
}

// 回避ステートマシン更新 / 回避狀態機更新
// 戻り値: true=回避中（他制御を上書き） / return true if avoidance active
bool updateObstacleAvoidance(float leftDist, float centerDist, float rightDist, unsigned long now, float &outLinear, float &outAngular) {
    if (avoidanceState == NO_AVOIDANCE) return false;  // 何もしない

    // 早期クリア判定 + ヒステリシス（後退/旋回フェーズ）
    if (avoidanceState == AVOIDING_TURN || avoidanceState == AVOIDING_EXTRA_TURN || avoidanceState == AVOIDING_BACKWARD) {
        if (centerDist > AVOID_CLEAR_CENTER) {
            if (++clearConsecutiveCount >= CLEAR_REQUIRED) {
                avoidanceState = AVOIDING_COMPLETE;
            }
        } else {
            clearConsecutiveCount = 0;
        }
    }

    switch (avoidanceState) {
        case AVOIDING_BACKWARD:
            if (now - avoidanceStartTime > dynBackwardDur) {
                avoidanceState     = AVOIDING_TURN;
                avoidanceStartTime = now;
                if (leftDist > rightDist) motor.turnLeft(200); else motor.turnRight(200);
            }
            break;
        case AVOIDING_TURN:
            if (now - avoidanceStartTime > dynTurnDur) {
                avoidanceState     = AVOIDING_EXTRA_TURN;
                avoidanceStartTime = now;
                if (leftDist > rightDist) motor.turnLeft(200); else motor.turnRight(200);
            }
            break;
        case AVOIDING_EXTRA_TURN:
            if (now - avoidanceStartTime > dynExtraTurnDur) {
                avoidanceState     = AVOIDING_FORWARD;
                avoidanceStartTime = now;
                motor.moveWithAngularVelocity(100, 0.0);
            }
            break;
        case AVOIDING_FORWARD:
            if (now - avoidanceStartTime > dynForwardDur) {
                bool stuck = (centerDist > 0 && centerDist < STUCK_CENTER_THRESH &&
                              leftDist > 0 && leftDist < STUCK_SIDE_THRESH &&
                              rightDist > 0 && rightDist < STUCK_SIDE_THRESH);
                if (stuck) {
                    avoidanceState     = AVOIDING_STUCK_BACKWARD; // まず後退 / reverse first
                    avoidanceStartTime = now;
                    motor.moveBackward(SEARCH_SPEED);
                    Serial.println("STUCK: backward stage start");
                } else if (centerDist > 0 && centerDist < AVOID_CLEAR_CENTER && avoidanceAttempt < AVOID_MAX_ATTEMPTS) {
                    avoidanceAttempt++;
                    dynBackwardDur  = (dynBackwardDur + 200 > 1200) ? 1200 : dynBackwardDur + 200;
                    dynTurnDur      = (dynTurnDur + 400 > 4000) ? 4000 : dynTurnDur + 400;
                    dynExtraTurnDur = (dynExtraTurnDur + 200 > 2000) ? 2000 : dynExtraTurnDur + 200;
                    dynForwardDur   = (dynForwardDur + 300 > 3000) ? 3000 : dynForwardDur + 300;
                    avoidanceState     = AVOIDING_BACKWARD;
                    avoidanceStartTime = now;
                    motor.moveBackward(SEARCH_SPEED);
                    Serial.print("AVOID RE-TRY #"); Serial.println(avoidanceAttempt);
                } else if (centerDist > 0 && centerDist < AVOID_CLEAR_CENTER && avoidanceAttempt >= AVOID_MAX_ATTEMPTS) {
                    Serial.println("AVOID FALLBACK 180 TURN");
                    motor.turnLeft(200);
                    avoidanceState     = AVOIDING_TURN;
                    avoidanceStartTime = now;
                    dynTurnDur = 3500; dynExtraTurnDur = 0; dynForwardDur = 1200; dynBackwardDur = 400;
                } else {
                    avoidanceState = AVOIDING_COMPLETE;
                    motor.stopRobot();
                }
            }
            break;
        case AVOIDING_STUCK_BACKWARD:
            if (now - avoidanceStartTime > 600) { // 余裕を少し長めに / give a bit more time
                avoidanceState   = AVOIDING_STUCK_PIVOT;
                stuckPivotCount  = 0;
                stuckPivotLeft   = (rightDist <= leftDist); // 広い側へ回頭
                stuckPivotStart  = now;
                if (stuckPivotLeft) motor.turnLeft(200); else motor.turnRight(200);
                Serial.println("STUCK: entering pivot sequence (after backward)");
            }
            break;
        case AVOIDING_STUCK_PIVOT: {
                if ((leftDist > STUCK_RELEASE_SIDE && leftDist < 400) || (rightDist > STUCK_RELEASE_SIDE && rightDist < 400)) {
                    Serial.println("STUCK: released by side opening");
                    avoidanceState = AVOIDING_COMPLETE;
                    motor.stopRobot();
                    break;
                }
                if (now - stuckPivotStart > STUCK_PIVOT_DURATION) {
                    stuckPivotCount++;
                    if (stuckPivotCount >= STUCK_MAX_PIVOTS) {
                        Serial.println("STUCK: max pivots reached -> fallback");
                        avoidanceState     = AVOIDING_TURN;
                        avoidanceStartTime = now;
                        motor.turnLeft(200);
                        dynTurnDur = 3000; dynExtraTurnDur = 0; dynForwardDur = 1200; dynBackwardDur = 400;
                        break;
                    }
                    stuckPivotLeft  = !stuckPivotLeft;
                    stuckPivotStart = now;
                    if (stuckPivotLeft) motor.turnLeft(200); else motor.turnRight(200);
                    Serial.print("STUCK: pivot #"); Serial.println(stuckPivotCount);
                }
            }
            break;
        case AVOIDING_COMPLETE:
            avoidanceState        = NO_AVOIDANCE;
            avoidanceCompleteTime = now;
            clearConsecutiveCount = 0;
            break;
        default:
            break;
    }
    // 回避中は個別に直接モーターを駆動しているので out* は現状未使用 / 直接驅動中
    return (avoidanceState != NO_AVOIDANCE);
}

// 回避トリガ判定（追跡中は閾値を下げる） / 回避觸發（追蹤時降低閾值）
void tryStartAvoidance(float leftDist, float centerDist, float rightDist, unsigned long now, bool tracking) {
    if (avoidanceState != NO_AVOIDANCE) return;
    // 追跡中は閾値を 4cm に低減 / 追蹤時閾值降到4cm
    float trig = tracking ? 4.0f : OBSTACLE_CENTER_TRIG;
    if ( (centerDist > 0 && centerDist < trig) || (leftDist > 0 && leftDist < trig) || (rightDist > 0 && rightDist < trig) ) {
        avoidanceState     = AVOIDING_BACKWARD;
        avoidanceStartTime = now;
    avoidanceAttempt = 0; // リセット
    // 動的時間リセット
    dynBackwardDur = 600; dynTurnDur = 2000; dynExtraTurnDur = 1000; dynForwardDur = 1500;
    clearConsecutiveCount = 0;
        motor.moveBackward(SEARCH_SPEED);
        Serial.print("AVOID START (center obstacle trig=");
        Serial.print(trig, 1);
        Serial.println(")");
    }
}

void setup() {
    Serial.begin(9600);
    motor.init();
    pixy.init();
    ultrasonic.init();
    servoCont.init();

    // ジャイロセンサーの初期化 / 陀螺儀傳感器初始化
    if (!gyro.init()) {
        Serial.println("Gyro initialization failed!");
        while (1);
    }

    // ジャイロをモーター制御に関連付け / 將陀螺儀關聯到電機控制
    motor.setGyroSensor(&gyro);

    // モーターの回転方向を設定（必要に応じてtrueに変更） / 設置電機旋轉方向（必要時改為true）
    // motor.setMotorDirection(false, false); // 両方とも正常 / 兩個都正常
    // motor.setMotorDirection(true, false);  // 右モーターのみ逆転 / 僅右電機反轉
    // motor.setMotorDirection(false, true);  // 左モーターのみ逆転 / 僅左電機反轉
    // motor.setMotorDirection(true, true);   // 両方とも逆転 / 兩個都反轉
    delay(5000);

    Serial.println("Starting main program...");
    motor.stopRobot();
}

void loop() {
    // 更新（角速度制御）
    motor.updateAngularControl();
    // サーボ更新は追跡アクティブ時のみ / Only update sweep while actively tracking
    if (objectTrackingActive) {
        servoCont.updateNonBlockingSweep();
    }

    unsigned long currentTime = millis();
    float leftDist, centerDist, rightDist;
    ultrasonic.getAllDistances(leftDist, centerDist, rightDist);

    // PIXYオフセット取得（追跡中判定のため先に取得） / 先取得PIXY偏移量以判定是否追蹤中
    int32_t x_offset = pixy.getXOffset(1);
    bool tracking    = (x_offset != -1);

    // 回避トリガ判定（追跡中は閾値縮小 6cm→4cm） / 嘗試啟動回避（追蹤時縮小觸發閾值）
    tryStartAvoidance(leftDist, centerDist, rightDist, currentTime, tracking);

    // 基本値
    float linearCmd  = 0.0f;
    float angularCmd = 0.0f;

    // 相対ヨー角（壁追従用）
    float relYaw = gyro.getRelativeYaw();

    // 回避更新（回避中なら以降は追跡/探索抑制）
    if (updateObstacleAvoidance(leftDist, centerDist, rightDist, currentTime, linearCmd, angularCmd)) {
        return;  // 回避処理が直接モーターを制御
    }

    // 壁追従補正
    float wallFollowCorrection = computeWallFollowCorrection(leftDist, rightDist, relYaw);
    // スピンスレッショルド（絶対角速度が異常に高い場合の安全用）
    float currentAngVel = gyro.getAngularVelocityZDegrees();
    static unsigned long spinStartTime = 0;
    const float SPIN_ABS_THRESHOLD = 120.0f; // 異常スピン判定角速度
    const unsigned long SPIN_PERSIST_MS = 300; // 継続時間
    bool spinning = false;
    if (fabs(currentAngVel) > SPIN_ABS_THRESHOLD) {
        if (spinStartTime == 0) spinStartTime = currentTime;
        if (currentTime - spinStartTime > SPIN_PERSIST_MS) spinning = true;
    } else {
        spinStartTime = 0;
    }

    // 上で取得済みの x_offset を使用 / 使用先前取得的 x_offset
    if (x_offset != -1) {
        lastTargetSeenTime = currentTime; // 検出時刻更新
        // --- オブジェクトを検出した場合 --- / --- 檢測到對象的情況 ---
        if (searchState != NOT_SEARCHING) {
            // 探索モードだった場合は解除 / 如果是搜索模式則解除
            searchState = NOT_SEARCHING;
            motor.stopRobot();
            // Serial.println("Object found, tracking started.");
        }

        // オブジェクト追跡中のサーボ制御 / 對象追蹤中的舵機控制
        if (!objectTrackingActive) {
            objectTrackingActive = true;
            servoCont.startNonBlockingSweep();
        }

        // 障害物回避中でない場合のみ、通常のPID制御でオブジェクトを追跡 / 僅在非障礙物回避中時，用正常PID控制追蹤對象
        if (avoidanceState == NO_AVOIDANCE) {
            // 障害物がない場合、比例制御でオブジェクトを追跡
            // When there are no obstacles, track the object with proportional control.
            // x_offsetから直接目標角速度を計算（外側PIDを撤廃）
            // Calculate target angular velocity directly from x_offset (outer PID removed)
            // K_angularは調整用ゲイン。x_offsetが最大(160)の時に目標角速度が約32deg/sになるように設定。
            // K_angular is a gain for adjustment. Set so that the target angular velocity is approx. 32deg/s when x_offset is max (160).
            const float K_angular       = 0.15f;
            float targetAngularVelocity = K_angular * (float)x_offset / 160.0 * 30.0;

            // 壁追従補正を加算
            targetAngularVelocity += wallFollowCorrection;

            // 距離に応じた前進速度調整 / 根據距離調整前進速度
            // float forwardSpeed;
            // if (abs(x_offset) < 80) {
            //     // 中程度のずれ：中程度の速度
            //     forwardSpeed = BASE_SPEED * 0.5;
            // } else {
            //     // 大きくずれている場合：低速で慎重に
            //     forwardSpeed = BASE_SPEED * 0.3;
            // }

            if (spinning) {
                // 緊急スピン抑制：角速度目標をゼロにし短時間停止
                motor.moveWithAngularVelocity(0.0f, 0.0f);
                if (currentTime - lastDebugTime > DEBUG_INTERVAL) {
                    Serial.println("[SAFETY] Spin detected -> emergency damping");
                }
            } else {
                // 通常追跡
                // 角速度コマンドを安全範囲へクランプ
                targetAngularVelocity = constrain(targetAngularVelocity, -40.0f, 40.0f);
                motor.moveWithAngularVelocity(BASE_SPEED * 0.6f, targetAngularVelocity);
            }

            // デバッグ出力（500ms間隔で制限）
            if (currentTime - lastDebugTime > DEBUG_INTERVAL) {
                // Serial.print("TRACK - X:");
                // Serial.print(x_offset);
                Serial.print(" TargetAng:");
                Serial.print(targetAngularVelocity, 1);
                // Serial.print(" Speed:");
                // Serial.print(forwardSpeed, 1);
                Serial.print(" Dist L:");
                Serial.print(leftDist, 0);
                Serial.print(" C:");
                Serial.print(centerDist, 0);
                Serial.print(" R:");
                Serial.println(rightDist, 0);
                lastDebugTime = currentTime;
            }

    } else {  // 物体未検出時：探索 + 壁追従 / 未檢測到物體時：搜索 + 牆面追隨
            // --- オブジェクトを検出しなかった場合（探索モード） --- / --- 未檢測到對象的情況（搜索模式） ---

            // オブジェクト追跡が非アクティブの場合、サーボを停止 / 對象追蹤非活躍時，停止舵機
            if (objectTrackingActive) {
                objectTrackingActive = false;
                servoCont.stopNonBlockingSweep();
            }

            // 障害物回避中でない場合のみ探索動作を実行 / 僅在非障礙物回避中時執行搜索動作
            if (avoidanceState == NO_AVOIDANCE) {
                switch (searchState) {
                    case NOT_SEARCHING:
                        // 回避完了から1秒待ってから探索開始 / 回避完成後等待1秒再開始搜索
                        if (avoidanceCompleteTime == 0 || currentTime - avoidanceCompleteTime > 1000) {
                            // Serial.println("SEARCH START");
                            // 壁追従補正を適用した探索モード / 應用牆面追隨補正的搜索模式
                            float searchAngularVelocity = -16.0 + wallFollowCorrection;  // 左旋回 + 壁追従 / 左旋轉 + 牆面追隨
                            motor.moveWithAngularVelocity(0, searchAngularVelocity);
                            searchState     = SEARCHING_LEFT;
                            stateChangeTime = currentTime;
                        }
                        break;

                    case SEARCHING_LEFT:
                        // 2.0秒間、左に旋回（500ms→2000msに延長）
                        if (currentTime - stateChangeTime > 2000) {
                            // 壁追従補正を適用した右旋回
                            float searchAngularVelocity = 16.0 + wallFollowCorrection;  // 右旋回 + 壁追従
                            // スピン中は探索角速度抑制
                            if (spinning) searchAngularVelocity = (searchAngularVelocity>0?8.0f:-8.0f);
                            motor.moveWithAngularVelocity(100, searchAngularVelocity);
                            searchState     = SEARCHING_RIGHT;
                            stateChangeTime = currentTime;
                        }
                        break;

                    case SEARCHING_RIGHT:
                        // 2.0秒間、右に旋回（500ms→2000msに延長）
                        if (currentTime - stateChangeTime > 2000) {
                            // 壁追従補正を適用した左旋回
                            float searchAngularVelocity = -16.0 + wallFollowCorrection;  // 左旋回 + 壁追従
                            if (spinning) searchAngularVelocity = (searchAngularVelocity>0?8.0f:-8.0f);
                            motor.moveWithAngularVelocity(100, searchAngularVelocity);
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
    } else { // ターゲット未検出 / Target not detected
        // タイムアウトでスイープ停止（即時停止防止の猶予）/ Stop sweep after grace period
        if (objectTrackingActive && (currentTime - lastTargetSeenTime > TARGET_LOST_TIMEOUT_MS)) {
            objectTrackingActive = false;
            servoCont.stopNonBlockingSweep();
        }
        // 以下は既存の探索ロジックに委ねる（上の探索ブロックへ到達しないため追加実装不要）
    }
}
