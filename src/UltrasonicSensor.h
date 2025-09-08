#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <Arduino.h>

class UltrasonicSensor {
public:
  UltrasonicSensor();
  void init();

  // Reads all three sensors sequentially.
  // This is a blocking function, but uses a short timeout to keep the loop fast.
  void readDistances(float& left, float& center, float& right);

private:
  // WARNING: Using pins from pinAssignment.md. Pins 0 & 1 conflict with Serial.
  static const int LEFT_TRIG_PIN = 0;
  static const int LEFT_ECHO_PIN = 1;
  static const int CENTER_TRIG_PIN = 20;
  static const int CENTER_ECHO_PIN = 21;
  static const int RIGHT_TRIG_PIN = A1;
  static const int RIGHT_ECHO_PIN = A0;

  // Helper function to measure distance with a timeout
  float measureDistance(int trigPin, int echoPin);
};

#endif // ULTRASONIC_SENSOR_H
