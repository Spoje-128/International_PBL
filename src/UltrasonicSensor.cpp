#include "UltrasonicSensor.h"

UltrasonicSensor::UltrasonicSensor() {
  currentLeftDistance = 0.0;
  currentCenterDistance = 0.0;
  currentRightDistance = 0.0;
  lastMeasurementTime = 0;
  lastSensorUpdate = 0;
  currentSensorState = IDLE;
  measurementCount = 0;
  stateStartTime = 0;
}

void UltrasonicSensor::init() {
  // 左センサーのピン設定
  pinMode(LEFT_TRIG, OUTPUT);
  pinMode(LEFT_ECHO, INPUT);
  
  // 中央センサーのピン設定
  pinMode(CENTER_TRIG, OUTPUT);
  pinMode(CENTER_ECHO, INPUT);
  
  // 右センサーのピン設定
  pinMode(RIGHT_TRIG, OUTPUT);
  pinMode(RIGHT_ECHO, INPUT);
  
  Serial.println("Ultrasonic Sensors initialized for Arduino Mega");
  Serial.println("Left: Trig=22, Echo=23");
  Serial.println("Center: Trig=24, Echo=25");
  Serial.println("Right: Trig=26, Echo=27");
  Serial.println("All three sensors enabled");
}

float UltrasonicSensor::getLeftDistance() {
  updateSensors(); // 非ブロッキング更新を実行
  return currentLeftDistance;
}

float UltrasonicSensor::getCenterDistance() {
  updateSensors(); // 非ブロッキング更新を実行
  return currentCenterDistance;
}

float UltrasonicSensor::getRightDistance() {
  updateSensors(); // 非ブロッキング更新を実行
  return currentRightDistance;
}

void UltrasonicSensor::getAllDistances(float& left, float& center, float& right) {
  updateSensors(); // 非ブロッキング更新を実行
  
  // 最新の値を返す
  left = currentLeftDistance;
  center = currentCenterDistance;
  right = currentRightDistance;
}

bool UltrasonicSensor::isObstacleDetected(float threshold) {
  updateSensors(); // 非ブロッキング更新を実行

  float left = currentLeftDistance;
  float center = currentCenterDistance;
  float right = currentRightDistance;

  // いずれかのセンサーで閾値以下の距離を検出した場合、障害物ありと判定
  bool obstacleDetected = (left <= threshold || center <= threshold || right <= threshold || left == 400.0 || center == 400.0 || right == 400.0);

  if (obstacleDetected) {
    Serial.println("*** OBSTACLE DETECTED ***");
    if (left <= threshold) Serial.println("- Left sensor detected obstacle");
    if (center <= threshold) Serial.println("- Center sensor detected obstacle");
    if (right <= threshold) Serial.println("- Right sensor detected obstacle");
  }
  
  return obstacleDetected;
}

float UltrasonicSensor::measureDistance(int trigPin, int echoPin) {
  // Trigピンを確実にクリア
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5); // より長い待機時間
  
  // Trigピンを10マイクロ秒間HIGHに
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Echoピンからの応答時間を測定（タイムアウトを延長）
  unsigned long duration = pulseIn(echoPin, HIGH, 50000); // 50msタイムアウト
  
  // デバッグ出力
  // Serial.print("Trig:");
  // Serial.print(trigPin);
  // Serial.print(", Echo:");
  // Serial.print(echoPin);
  // Serial.print(", Duration:");
  // Serial.print(duration);
  
  // 距離を計算（音速: 約343m/s = 0.0343cm/μs）
  // 往復時間なので2で割る
  float distance = (duration * 0.0343) / 2.0;
  
  // 測定範囲外の場合は最大値を返す
  if (duration == 0 || distance <= 2.0 || distance > 400) {
    distance = 400.0; // 測定可能最大距離
    // Serial.print(", Distance: TIMEOUT/ERROR -> 400cm");
  } else {
    // Serial.print(", Distance:");
    // Serial.print(distance);
    // Serial.print("cm");
  }
  // Serial.println();
  
  return distance;
}

float UltrasonicSensor::filterDistance(float newDistance, float previousDistance) {
  // 初回測定の場合はそのまま返す
  if (previousDistance == 0.0) {
    return newDistance;
  }
  
  // 前回との差が大きすぎる場合はノイズとして除去
  float diff = abs(newDistance - previousDistance);
  if (diff > 50.0) {
    return previousDistance; // 前回の値を維持
  }
  
  // 移動平均フィルタを適用（重み付き平均）
  return (previousDistance * 0.7) + (newDistance * 0.3);
}

void UltrasonicSensor::updateSensors() {
  unsigned long currentTime = millis();
  
  // 前回の更新から十分時間が経過していない場合はスキップ
  if (currentTime - lastSensorUpdate < SENSOR_UPDATE_INTERVAL) {
    return;
  }
  
  lastSensorUpdate = currentTime;
  
  switch (currentSensorState) {
    case IDLE:
      // 次の測定サイクルを開始
      if (currentTime - lastMeasurementTime >= MEASUREMENT_INTERVAL) {
        currentSensorState = MEASURE_LEFT;
        measurementCount = 0;
        stateStartTime = currentTime;
      }
      break;
      
    case MEASURE_LEFT:
      if (measurementCount == 0) {
        // 左センサーの測定を開始
        tempMeasurements[measurementCount] = measureDistance(LEFT_TRIG, LEFT_ECHO);
        measurementCount++;
      } else if (currentTime - stateStartTime >= 5) {  // 5ms待機
        if (measurementCount < 3) {
          tempMeasurements[measurementCount] = measureDistance(LEFT_TRIG, LEFT_ECHO);
          measurementCount++;
        } else {
          // 3回測定完了、平均値を計算
          float sum = 0;
          int validCount = 0;
          for (int i = 0; i < 3; i++) {
            if (tempMeasurements[i] < 400.0) {
              sum += tempMeasurements[i];
              validCount++;
            }
          }
          currentLeftDistance = (validCount > 0) ? (sum / validCount) : 400.0;
          // Serial.print("current left distance: ");
          // Serial.print(currentLeftDistance);
          
          // 次の状態へ
          currentSensorState = MEASURE_CENTER;
          measurementCount = 0;
          stateStartTime = currentTime;
        }
      }
      break;
      
    case MEASURE_CENTER:
      if (measurementCount == 0) {
        // 中央センサーの測定を開始
        tempMeasurements[measurementCount] = measureDistance(CENTER_TRIG, CENTER_ECHO);
        measurementCount++;
      } else if (currentTime - stateStartTime >= 5) {  // 5ms待機
        if (measurementCount < 3) {
          tempMeasurements[measurementCount] = measureDistance(CENTER_TRIG, CENTER_ECHO);
          measurementCount++;
        } else {
          // 3回測定完了、平均値を計算
          float sum = 0;
          int validCount = 0;
          for (int i = 0; i < 3; i++) {
            if (tempMeasurements[i] < 400.0) {
              sum += tempMeasurements[i];
              validCount++;
            }
          }
          currentCenterDistance = (validCount > 0) ? (sum / validCount) : 400.0;
          // Serial.print(", current center distance: ");
          // Serial.print(currentCenterDistance);
          
          // 次の状態へ
          currentSensorState = MEASURE_RIGHT;
          measurementCount = 0;
          stateStartTime = currentTime;
        }
      }
      break;
      
    case MEASURE_RIGHT:
      if (measurementCount == 0) {
        // 右センサーの測定を開始
        tempMeasurements[measurementCount] = measureDistance(RIGHT_TRIG, RIGHT_ECHO);
        measurementCount++;
      } else if (currentTime - stateStartTime >= 5) {  // 5ms待機
        if (measurementCount < 3) {
          tempMeasurements[measurementCount] = measureDistance(RIGHT_TRIG, RIGHT_ECHO);
          measurementCount++;
        } else {
          // 3回測定完了、平均値を計算
          float sum = 0;
          int validCount = 0;
          for (int i = 0; i < 3; i++) {
            if (tempMeasurements[i] < 400.0) {
              sum += tempMeasurements[i];
              validCount++;
            }
          }
          currentRightDistance = (validCount > 0) ? (sum / validCount) : 400.0;
          // Serial.print(", current right distance: ");
          // Serial.print(currentRightDistance);
          
          // 測定完了、IDLEに戻る
          currentSensorState = IDLE;
          lastMeasurementTime = currentTime;
          
          // 結果を出力
          Serial.print("Non-blocking - L: ");
          Serial.print(currentLeftDistance);
          Serial.print("cm, C: ");
          Serial.print(currentCenterDistance);
          Serial.print("cm, R: ");
          Serial.print(currentRightDistance);
          Serial.println("cm");
        }
      }
      break;
  }
}
