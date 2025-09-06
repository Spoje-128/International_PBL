#include "GyroSensor.h"

GyroSensor::GyroSensor() : bno(Adafruit_BNO055(55, 0x29)) {
  currentAngularVelocityX = 0.0;
  currentAngularVelocityY = 0.0;
  currentAngularVelocityZ = 0.0;
  filteredAngularVelocityX = 0.0;
  filteredAngularVelocityY = 0.0;
  filteredAngularVelocityZ = 0.0;
  lastUpdateTime = 0;
  calibrationComplete = false;
  
  // ヨー角オフセット管理の初期化
  yawOffsetDegrees = 0.0;
  yawReferenceSet = false;
  yawResetTime = 0;
}

bool GyroSensor::init() {
  Serial.println("Initializing BNO055 Gyroscope...");
  
  if (!bno.begin()) {
    Serial.println("ERROR: No BNO055 detected! Check wiring or I2C address.");
    return false;
  }
  
  delay(1000); // センサー起動待ち
  
  // 外部水晶を使用（より高精度）
  bno.setExtCrystalUse(true);
  
  Serial.println("BNO055 Gyroscope initialized successfully");
  
  // 初期キャリブレーション状態をチェック
  checkSensorStatus();
  
  // 初期ヨー角基準値を設定（少し待ってから）
  delay(500);
  resetYawReference();
  
  return true;
}

void GyroSensor::update() {
  unsigned long currentTime = millis();
  
  // 更新間隔チェック
  if (currentTime - lastUpdateTime < UPDATE_INTERVAL) {
    return;
  }
  
  lastUpdateTime = currentTime;
  
  // 角速度データを取得（単位: deg/s）
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  
  // 参考コードに基づく座標系変換（deg/sをそのまま使用）
  float gx = -gyro.x(); // deg/s
  float gy =  gyro.y(); // deg/s
  float gz = -gyro.z(); // deg/s
  
  // LPF適用（参考コードと同じアルゴリズム）
  filteredAngularVelocityX = lowPassFilter(gx, filteredAngularVelocityX);
  filteredAngularVelocityY = lowPassFilter(gy, filteredAngularVelocityY);
  filteredAngularVelocityZ = lowPassFilter(gz, filteredAngularVelocityZ);
  
  // 現在値も更新（deg/s）
  currentAngularVelocityX = gx;
  currentAngularVelocityY = gy;
  currentAngularVelocityZ = gz;
  
  // デバッグ出力（必要に応じてコメントアウト）
  /*
  Serial.print("Gyro X: ");
  Serial.print(filteredAngularVelocityX, 4);
  Serial.print(" deg/s, Y: ");
  Serial.print(filteredAngularVelocityY, 4);
  Serial.print(" deg/s, Z: ");
  Serial.print(filteredAngularVelocityZ, 4);
  Serial.println(" deg/s");
  */
}

float GyroSensor::getAngularVelocityZ() {
  update(); // 最新データに更新
  return filteredAngularVelocityZ * PI / 180.0; // deg/s → rad/s に変換
}

float GyroSensor::getAngularVelocityZDegrees() {
  update(); // 最新データに更新
  return filteredAngularVelocityZ; // deg/s をそのまま返す
}

float GyroSensor::getAngularVelocityX() {
  update(); // 最新データに更新
  return filteredAngularVelocityX * PI / 180.0; // deg/s → rad/s に変換
}

float GyroSensor::getAngularVelocityY() {
  update(); // 最新データに更新
  return filteredAngularVelocityY * PI / 180.0; // deg/s → rad/s に変換
}

void GyroSensor::getAllAngularVelocities(float& x, float& y, float& z) {
  update(); // 最新データに更新
  x = filteredAngularVelocityX * PI / 180.0; // deg/s → rad/s
  y = filteredAngularVelocityY * PI / 180.0; // deg/s → rad/s
  z = filteredAngularVelocityZ * PI / 180.0; // deg/s → rad/s
}

float GyroSensor::getYaw() {
  // オイラー角（ヨー角）を取得 [度]
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  return euler.x(); // ヨー角（度）
}

float GyroSensor::getRelativeYaw() {
  // 相対ヨー角を取得（初期化時からの変化量）[度]
  float currentYaw = getYaw();
  
  // 基準値が未設定の場合は設定
  if (!yawReferenceSet) {
    resetYawReference();
    return 0.0;
  }
  
  // 自動リセット機能（長期ドリフト対策）
  unsigned long currentTime = millis();
  if (currentTime - yawResetTime > YAW_RESET_INTERVAL) {
    // 30秒経過し、かつロボットが静止している場合のみリセット
    if (abs(getAngularVelocityZDegrees()) < 2.0) { // 2度/秒以下で静止とみなす
      Serial.println("Auto-resetting yaw reference (drift compensation)");
      resetYawReference();
      return 0.0;
    }
  }
  
  // 相対角度を計算（-180 ~ +180度に正規化）
  float relativeYaw = normalizeYaw(currentYaw - yawOffsetDegrees);
  return relativeYaw;
}

void GyroSensor::resetYawReference() {
  // 現在のヨー角を基準値として設定
  yawOffsetDegrees = getYaw();
  yawReferenceSet = true;
  yawResetTime = millis();
  
  // Serial.print("Yaw reference reset to: ");
  // Serial.print(yawOffsetDegrees, 2);
  // Serial.println(" degrees");
}

float GyroSensor::getPitch() {
  // オイラー角（ピッチ角）を取得 [度]
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  return euler.y(); // ピッチ角（度）
}

float GyroSensor::getRoll() {
  // オイラー角（ロール角）を取得 [度]
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  return euler.z(); // ロール角（度）
}

bool GyroSensor::isTurning(float threshold) {
  update(); // 最新データに更新
  
  // Z軸の角速度の絶対値が閾値を超えているかチェック（deg/sで比較）
  float absAngularVelDeg = abs(filteredAngularVelocityZ);
  float thresholdDeg = threshold * 180.0 / PI; // rad/s → deg/s 変換
  
  if (absAngularVelDeg > thresholdDeg) {
    Serial.print("Robot is turning - Angular velocity Z: ");
    Serial.print(filteredAngularVelocityZ, 4);
    Serial.print(" deg/s (");
    Serial.print(getAngularVelocityZ(), 4);
    Serial.println(" rad/s)");
    return true;
  }
  
  return false;
}

void GyroSensor::calibrate() {
  Serial.println("Starting BNO055 calibration...");
  Serial.println("Please move the robot slowly in figure-8 patterns for calibration.");
  
  // キャリブレーション状態をリセット
  calibrationComplete = false;
  
  unsigned long calibrationStart = millis();
  const unsigned long CALIBRATION_TIMEOUT = 60000; // 60秒タイムアウト
  
  while (!calibrationComplete && (millis() - calibrationStart) < CALIBRATION_TIMEOUT) {
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    
    Serial.print("Calibration - Sys: ");
    Serial.print(system);
    Serial.print(", Gyro: ");
    Serial.print(gyro);
    Serial.print(", Accel: ");
    Serial.print(accel);
    Serial.print(", Mag: ");
    Serial.println(mag);
    
    // ジャイロのキャリブレーションが完了したらOK
    if (gyro >= 3 && system >= 2) {
      calibrationComplete = true;
      Serial.println("BNO055 calibration completed!");
    }
    
    delay(500);
  }
  
  if (!calibrationComplete) {
    Serial.println("WARNING: Calibration timeout. Sensor may not be fully calibrated.");
  }
}

bool GyroSensor::isCalibrated() {
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  
  // ジャイロのキャリブレーションが完了していればOK
  return (gyro >= 3);
}

void GyroSensor::reset() {
  Serial.println("Resetting gyro sensor...");
  
  // 値をリセット
  currentAngularVelocityX = 0.0;
  currentAngularVelocityY = 0.0;
  currentAngularVelocityZ = 0.0;
  filteredAngularVelocityX = 0.0;
  filteredAngularVelocityY = 0.0;
  filteredAngularVelocityZ = 0.0;
  calibrationComplete = false;
  
  // ヨー角基準値もリセット
  yawOffsetDegrees = 0.0;
  yawReferenceSet = false;
  yawResetTime = 0;
  
  Serial.println("Gyro sensor reset completed.");
}

float GyroSensor::lowPassFilter(float newValue, float previousValue, float alpha) {
  // 参考コードと同じアルゴリズム: filtered = alpha * new + (1-alpha) * previous
  return alpha * newValue + (1.0 - alpha) * previousValue;
}

bool GyroSensor::checkSensorStatus() {
  // センサーの動作状態をチェック
  uint8_t system_status, self_test_results, system_error;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);
  
  Serial.println("=== BNO055 System Status ===");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test Results: 0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error: 0x");
  Serial.println(system_error, HEX);
  
  // キャリブレーション状態もチェック
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  
  Serial.println("=== Calibration Status ===");
  Serial.print("System: ");
  Serial.print(system);
  Serial.print(", Gyro: ");
  Serial.print(gyro);
  Serial.print(", Accel: ");
  Serial.print(accel);
  Serial.print(", Mag: ");
  Serial.println(mag);
  
  if (gyro >= 3) {
    Serial.println("Gyroscope is calibrated and ready!");
    calibrationComplete = true;
    return true;
  } else {
    Serial.println("WARNING: Gyroscope needs calibration!");
    return false;
  }
}

float GyroSensor::normalizeYaw(float yaw) {
  // ヨー角を -180 ~ +180度の範囲に正規化
  while (yaw > 180.0) {
    yaw -= 360.0;
  }
  while (yaw < -180.0) {
    yaw += 360.0;
  }
  return yaw;
}
