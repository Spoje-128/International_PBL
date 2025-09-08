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
  // Pins that do not conflict with Serial communication
  static const int LEFT_TRIG_PIN = 22;
  static const int LEFT_ECHO_PIN = 23;
  static const int CENTER_TRIG_PIN = 24;
  static const int CENTER_ECHO_PIN = 25;
  static const int RIGHT_TRIG_PIN = 26;
  static const int RIGHT_ECHO_PIN = 27;

  // Helper function to measure distance with a timeout
  float measureDistance(int trigPin, int echoPin);
};

#endif // ULTRASONIC_SENSOR_H
