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
  bool isObstacleDetected(float threshold = 10.0); // デフォルト閾値10cm

private:
  // 左センサー用ピン (Arduino Mega)
  static const int LEFT_TRIG = 22;
  static const int LEFT_ECHO = 23;
  
  // 中央センサー用ピン (Arduino Mega)
  static const int CENTER_TRIG = 24;
  static const int CENTER_ECHO = 25;
  
  // 右センサー用ピン (Arduino Mega)
  static const int RIGHT_TRIG = 26;
  static const int RIGHT_ECHO = 27;
  
  float measureDistance(int trigPin, int echoPin);
  float filterDistance(float newDistance, float previousDistance);
  void updateSensors(); // 非ブロッキング更新メソッド
  
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
