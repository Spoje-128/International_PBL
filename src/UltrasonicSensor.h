#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <Arduino.h>

class UltrasonicSensor {
public:
  UltrasonicSensor();
  void init();

  // Reads all three sensors sequentially and applies a Low-Pass Filter.
  void readDistances(float& left, float& center, float& right);

private:
  // Using pins that DO NOT conflict with Serial communication for reliable operation.
  static const int LEFT_TRIG_PIN = 22;
  static const int LEFT_ECHO_PIN = 23;
  static const int CENTER_TRIG_PIN = 24;
  static const int CENTER_ECHO_PIN = 25;
  static const int RIGHT_TRIG_PIN = 26;
  static const int RIGHT_ECHO_PIN = 27;

  // Low-Pass Filter (LPF) constant
  static constexpr float LPF_ALPHA = 0.6;

  // Variables to store the previous filtered distance for the LPF
  float filteredLeft;
  float filteredCenter;
  float filteredRight;

  // Helper function to measure distance with a timeout
  float measureDistance(int trigPin, int echoPin);
};

#endif // ULTRASONIC_SENSOR_H
