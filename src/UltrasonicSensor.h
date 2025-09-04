#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <Arduino.h>

class UltrasonicSensor {
public:
  UltrasonicSensor();
  void init();
  float getLeftDistance();
  float getCenterDistance();
  float getRightDistance();
  void getAllDistances(float& left, float& center, float& right);
  bool isObstacleDetected(float threshold = 20.0); // デフォルト閾値20cm

private:
  // 左センサー用ピン
  static const int LEFT_TRIG = 2;
  static const int LEFT_ECHO = 3;
  
  // 中央センサー用ピン
  static const int CENTER_TRIG = 13;
  static const int CENTER_ECHO = 12;
  
  // 右センサー用ピン
  static const int RIGHT_TRIG = A1;
  static const int RIGHT_ECHO = A0;
  
  float measureDistance(int trigPin, int echoPin);
  float filterDistance(float newDistance, float previousDistance);
  void updateSensors(); // 非ブロッキング更新メソッド
  
  // 前回の測定値（ノイズフィルタ用）
  float previousLeftDistance;
  float previousCenterDistance;
  float previousRightDistance;
  
  // 現在の測定値（リアルタイム）
  float currentLeftDistance;
  float currentCenterDistance;
  float currentRightDistance;
  
  // 測定間隔制御用
  unsigned long lastMeasurementTime;
  unsigned long lastSensorUpdate;
  static const unsigned long MEASUREMENT_INTERVAL = 50; // 50ms間隔
  static const unsigned long SENSOR_UPDATE_INTERVAL = 10; // 10ms間隔で更新
  
  // センサー測定のステート管理
  enum SensorState {
    MEASURE_LEFT,
    MEASURE_CENTER, 
    MEASURE_RIGHT,
    IDLE
  };
  SensorState currentSensorState;
  int measurementCount;
  float tempMeasurements[3];
  unsigned long stateStartTime;
};

#endif // ULTRASONIC_SENSOR_H
