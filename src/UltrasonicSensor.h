#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <Arduino.h>

class UltrasonicSensor {
public:
  UltrasonicSensor();
  void init();
  // Reads all three sensors and returns distances in cm
  void readDistances(float& left, float& center, float& right);

private:
  // Pins based on pinAssignment.md for MEGA
  // WARNING: Pins 0 and 1 are the primary Serial (TX/RX) on an Arduino Mega.
  // Using them for the left ultrasonic sensor means you CANNOT use Serial.print()
  // for debugging at the same time. The code will function, but debugging via the
  // USB serial monitor will be unreliable or impossible.
  static const int LEFT_TRIG_PIN = 0;
  static const int LEFT_ECHO_PIN = 1;
  
  static const int CENTER_TRIG_PIN = 20;
  static const int CENTER_ECHO_PIN = 21;
  
  // A0 and A1 are analog pins, but can be used as digital pins
  static const int RIGHT_TRIG_PIN = A1;
  static const int RIGHT_ECHO_PIN = A0;
  
  // Helper function to measure distance for a single sensor
  float measureDistance(int trigPin, int echoPin);
};

#endif // ULTRASONIC_SENSOR_H
