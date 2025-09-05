#ifndef GYRO_SENSOR_H
#define GYRO_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

class GyroSensor {
public:
  GyroSensor();
  bool init();                          // センサーの初期化
  void update();                        // 角速度データの更新
  float getAngularVelocityZ();          // Z軸角速度取得（ヨーレート）[rad/s]
  float getAngularVelocityZDegrees();   // Z軸角速度取得（度/秒）[deg/s]
  float getAngularVelocityX();          // X軸角速度取得[rad/s]
  float getAngularVelocityY();          // Y軸角速度取得[rad/s]
  void getAllAngularVelocities(float& x, float& y, float& z); // 全軸取得[rad/s]
  bool isTurning(float threshold = 0.1); // 旋回中かどうか判定
  void calibrate();                     // キャリブレーション実行
  bool isCalibrated();                  // キャリブレーション完了確認
  void reset();                         // センサーのリセット

private:
  Adafruit_BNO055 bno;                  // BNO055センサーインスタンス
  
  // 現在の角速度値
  float currentAngularVelocityX;
  float currentAngularVelocityY;
  float currentAngularVelocityZ;
  
  // フィルタリング用（全軸対応）
  float filteredAngularVelocityX;
  float filteredAngularVelocityY;
  float filteredAngularVelocityZ;
  
  // 測定間隔制御
  unsigned long lastUpdateTime;
  static const unsigned long UPDATE_INTERVAL = 10; // 10ms間隔（100Hz）
  
  // キャリブレーション状態
  bool calibrationComplete;
  
  // フィルタリング関数（参考コードに合わせてalpha=0.1）
  float lowPassFilter(float newValue, float previousValue, float alpha = 0.1);
  
  // センサー状態確認
  bool checkSensorStatus();
};

#endif // GYRO_SENSOR_H
