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
  float getYaw();                       // ヨー角取得（度）[deg] - オイラー角
  float getRelativeYaw();               // 相対ヨー角取得（初期化時からの変化量）[deg]
  float getPitch();                     // ピッチ角取得（度）[deg] - オイラー角
  float getRoll();                      // ロール角取得（度）[deg] - オイラー角
  void resetYawReference();             // ヨー角基準値をリセット（現在位置を0度とする）
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
  
  // ヨー角オフセット管理（ドリフト対策）
  float yawOffsetDegrees;               // 初期化時のヨー角オフセット
  bool yawReferenceSet;                 // ヨー角基準値が設定済みかどうか
  unsigned long yawResetTime;           // 最後にヨー角基準をリセットした時刻
  static const unsigned long YAW_RESET_INTERVAL = 30000; // 30秒ごとに基準値更新を検討
  
  // フィルタリング関数（参考コードに合わせてalpha=0.1）
  float lowPassFilter(float newValue, float previousValue, float alpha = 0.1);
  
  // センサー状態確認
  bool checkSensorStatus();
  
  // ヨー角の正規化（-180 ~ +180度に収める）
  float normalizeYaw(float yaw);
};

#endif // GYRO_SENSOR_H
