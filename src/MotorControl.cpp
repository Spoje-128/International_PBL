#include "MotorControl.h"
#include "GyroSensor.h"

// static const float の定義
const float MotorControl::NOMINAL_TURN_RATE = 20.0;  // 30.0→20.0に減速

MotorControl::MotorControl() {
  // モーター回転方向フラグを初期化（false = 正転）
  rightMotorReversed = false;
  leftMotorReversed = true;
  
  // ロボットの物理パラメータを初期化（デフォルト値）
  robotWheelbase = 20.0;   // 20cm (要調整)
  robotWheelRadius = 3.5;  // 3.5cm (要調整)
  
  // ジャイロフィードバック制御の初期化
  gyroSensor = nullptr;
  targetAngularVelocity = 0.0;
  currentLinearSpeed = 0.0;
  angularControlActive = false;
  lastControlUpdate = 0;
  
  // PID制御パラメータ（要調整）
  // 二重PID問題を解消するため、ゲインを現実的な値に見直し。
  // The gain is reviewed to a realistic value to eliminate the dual PID problem.
  angularKp = 20.0;        // 比例ゲイン / Proportional gain
  angularKi = 0.5;         // 積分ゲイン / Integral gain
  angularKd = 10.0;        // 微分ゲイン / Derivative gain
  angularErrorSum = 0.0;
  angularLastError = 0.0;
  lastPidUpdate = 0;
}

void MotorControl::init() {
  // モーター制御ピンを出力として設定
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.println("L298N Dual Motor Control with PWM Ready");
  Serial.print("Nominal turn rate: ");
  Serial.print(NOMINAL_TURN_RATE);
  Serial.println(" deg/s");
}

void MotorControl::move(int baseSpeed, int correction) {
  // 従来の関数を角速度ベースに変更
  // correctionを角速度補正として解釈（deg/s）
  float linearSpeedNormalized = baseSpeed / 150.0 * 60.0; // PWM値を線速度に正規化（100.0→60.0に減速）
  float angularVelocityCorrection = correction * 0.3; // 補正値を角速度に変換（0.5→0.3に減少）
  
  // 角速度ベース制御を使用
  moveWithAngularVelocity(linearSpeedNormalized, angularVelocityCorrection);
  
  Serial.print("Legacy move() - Base: ");
  Serial.print(baseSpeed);
  Serial.print(", Correction: ");
  Serial.print(correction);
  Serial.print(" -> Linear: ");
  Serial.print(linearSpeedNormalized);
  Serial.print(", Angular: ");
  Serial.print(angularVelocityCorrection);
  Serial.println(" deg/s");
}

void MotorControl::moveBackward(int speed) {
  // 後進も角速度ベースに変更
  float linearSpeedNormalized = -speed / 150.0 * 60.0; // 負の線速度で後進（100.0→60.0に減速）
  moveWithAngularVelocity(linearSpeedNormalized, 0.0); // 直進後退
  
  Serial.print("Legacy moveBackward() - Speed: ");
  Serial.print(speed);
  Serial.print(" -> Linear: ");
  Serial.print(linearSpeedNormalized);
  Serial.println(" (straight backward)");
}

// 右車輪を前進させる関数
void MotorControl::rightWheelForward(int speed) {
  analogWrite(ENA, speed);
  if (rightMotorReversed) {
    // 逆転フラグがtrueの場合、方向を反転
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
}

// 右車輪を後進させる関数
void MotorControl::rightWheelBackward(int speed) {
  analogWrite(ENA, speed);
  if (rightMotorReversed) {
    // 逆転フラグがtrueの場合、方向を反転
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
}

// 右車輪を停止させる関数
void MotorControl::rightWheelStop() {
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

// 左車輪を前進させる関数
void MotorControl::leftWheelForward(int speed) {
  analogWrite(ENB, speed);
  if (leftMotorReversed) {
    // 逆転フラグがtrueの場合、方向を反転
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
}

// 左車輪を後進させる関数
void MotorControl::leftWheelBackward(int speed) {
  analogWrite(ENB, speed);
  if (leftMotorReversed) {
    // 逆転フラグがtrueの場合、方向を反転
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
}

// 左車輪を停止させる関数
void MotorControl::leftWheelStop() {
  analogWrite(ENB, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// ロボットを停止させる関数
void MotorControl::stopRobot() {
  rightWheelStop();
  leftWheelStop();
  angularControlActive = false; // フィードバック制御も停止
  angularErrorSum = 0.0;        // 積分項をリセット
  angularLastError = 0.0;
  lastPidUpdate = 0;            // PID時間もリセット
  Serial.println("Robot Stopped (angular control deactivated)");
}

void MotorControl::turnLeft(int speed) {
  // 従来の関数を角速度ベースに変更
  // speedをノミナル角速度にスケーリング
  float angularVelocity = -NOMINAL_TURN_RATE * (speed / 150.0); // 負=左旋回
  turnAtAngularVelocity(angularVelocity);
  
  Serial.print("Legacy turnLeft() - Speed: ");
  Serial.print(speed);
  Serial.print(" -> Angular: ");
  Serial.print(angularVelocity);
  Serial.println(" deg/s (left turn)");
}

void MotorControl::turnRight(int speed) {
  // speedをノミナル角速度にスケーリング
  float angularVelocity = NOMINAL_TURN_RATE * (speed / 150.0); // 正=右旋回
  turnAtAngularVelocity(angularVelocity);
}

// モーターの回転方向を設定する関数
void MotorControl::setMotorDirection(bool rightReversed, bool leftReversed) {
  rightMotorReversed = rightReversed;
  leftMotorReversed = leftReversed;
  
  Serial.print("Motor direction set - Right: ");
  Serial.print(rightReversed ? "REVERSED" : "NORMAL");
  Serial.print(", Left: ");
  Serial.println(leftReversed ? "REVERSED" : "NORMAL");
}

// ===== 角速度ベース制御関数（ジャイロフィードバック版） =====

void MotorControl::setGyroSensor(GyroSensor* gyro) {
  gyroSensor = gyro;
  Serial.println("Gyro sensor attached for angular velocity feedback control");
}

void MotorControl::moveWithAngularVelocity(float linearSpeed, float targetAngularVel) {
  // 新しい目標が設定されたら、PIDの状態をリセットする
  // Reset PID state when a new target is set
  if (targetAngularVelocity != targetAngularVel || currentLinearSpeed != linearSpeed) {
      angularErrorSum = 0.0;
      angularLastError = 0.0;
      lastPidUpdate = 0;
  }

  targetAngularVelocity = targetAngularVel;
  currentLinearSpeed = linearSpeed;
  angularControlActive = true;
  
  Serial.print("Angular control activated - Target: ");
  Serial.print(targetAngularVelocity, 2);
  Serial.print(" deg/s, Linear: ");
  Serial.println(currentLinearSpeed);
}

void MotorControl::turnAtAngularVelocity(float targetAngularVel) {
  // その場旋回（linearSpeed = 0）
  moveWithAngularVelocity(0.0, targetAngularVel);
}

void MotorControl::moveStraight(float linearSpeed) {
  // 直進（角速度 = 0）
  moveWithAngularVelocity(linearSpeed, 0.0);
}

void MotorControl::updateAngularControl() {
  if (!angularControlActive || gyroSensor == nullptr) {
    return;
  }
  
  unsigned long currentTime = millis();
  if (currentTime - lastControlUpdate < 10) { // 100Hz制御
    return;
  }
  lastControlUpdate = currentTime;
  
  // 現在の角速度を取得
  float currentAngularVel = gyroSensor->getAngularVelocityZDegrees();
  Serial.print("Current Angular Velocity: ");
  Serial.println(currentAngularVel);
  
  // PID制御で補正値を計算
  float correction = angularPIDControl(targetAngularVelocity, currentAngularVel);
  
  // 基本速度を計算（従来の運動学ベース）
  int baseLeftSpeed, baseRightSpeed;
  calculateWheelSpeeds(currentLinearSpeed, targetAngularVelocity, baseLeftSpeed, baseRightSpeed);
  
  // フィードバック補正を適用
  int correctedLeftSpeed = constrain(baseLeftSpeed - (int)correction, -255, 255);
  int correctedRightSpeed = constrain(baseRightSpeed + (int)correction, -255, 255);
  
  // モーター制御（符号で方向、絶対値でPWM速度）
  if (correctedLeftSpeed >= 0) {
    leftWheelForward(abs(correctedLeftSpeed));    // 正：前進、PWMは絶対値
  } else {
    leftWheelBackward(abs(correctedLeftSpeed));   // 負：後進、PWMは絶対値
  }
  
  if (correctedRightSpeed >= 0) {
    rightWheelForward(abs(correctedRightSpeed));  // 正：前進、PWMは絶対値
  } else {
    rightWheelBackward(abs(correctedRightSpeed)); // 負：後進、PWMは絶対値
  }
  
  // デバッグ出力
  Serial.print("FB Control - Target: ");
  Serial.print(targetAngularVelocity, 2);
  Serial.print(", Current: ");
  Serial.print(currentAngularVel, 2);
  Serial.print(", Correction: ");
  Serial.print(correction, 2);
  Serial.print(", L: ");
  Serial.print(correctedLeftSpeed);
  Serial.print(", R: ");
  Serial.println(correctedRightSpeed);
}

void MotorControl::setRobotParameters(float wheelbase, float wheelRadius) {
  robotWheelbase = wheelbase;
  robotWheelRadius = wheelRadius;
  
  Serial.print("Robot parameters updated - Wheelbase: ");
  Serial.print(robotWheelbase);
  Serial.print("cm, Wheel radius: ");
  Serial.print(robotWheelRadius);
  Serial.println("cm");
}

void MotorControl::calculateWheelSpeeds(float linearSpeed, float angularVelocityDeg, int& leftSpeed, int& rightSpeed) {
  // 角速度をrad/sに変換
  float angularVelocityRad = angularVelocityDeg * PI / 180.0;
  
  // 差動2輪ロボットの運動学
  // v_left = v_linear - (wheelbase/2) * angular_velocity
  // v_right = v_linear + (wheelbase/2) * angular_velocity
  
  float wheelSpeedLeft = linearSpeed - (robotWheelbase / 2.0) * angularVelocityRad;
  float wheelSpeedRight = linearSpeed + (robotWheelbase / 2.0) * angularVelocityRad;
  
  // 車輪速度をPWM値に変換（簡単なスケーリング）
  // 基準：30deg/s → BASE_TURN_SPEED PWM
  float speedScale = BASE_TURN_SPEED / (NOMINAL_TURN_RATE * PI / 180.0 * robotWheelRadius);
  
  leftSpeed = constrain((int)(wheelSpeedLeft * speedScale), -255, 255);
  rightSpeed = constrain((int)(wheelSpeedRight * speedScale), -255, 255);
}

float MotorControl::angularPIDControl(float targetAngular, float currentAngular) {
  unsigned long currentTime = millis();
  float dt = 0.0;
  if (lastPidUpdate != 0) {
    dt = (currentTime - lastPidUpdate) / 1000.0; // 経過時間を秒単位に
  }
  lastPidUpdate = currentTime;

  // dtが0または異常に大きい場合は計算をスキップ
  if (dt <= 0 || dt > 0.5) {
    return 0.0;
  }

  float error = targetAngular - currentAngular;
  
  // 積分項（ワインドアップ対策付き）
  angularErrorSum += error * dt;
  angularErrorSum = constrain(angularErrorSum, -100.0, 100.0);
  
  // 微分項
  float derivative = (error - angularLastError) / dt;
  angularLastError = error;
  
  // PID出力
  float output = (angularKp * error) + (angularKi * angularErrorSum) + (angularKd * derivative);
  
  return constrain(output, -150.0, 150.0); // PWM補正値の制限を緩和
}

