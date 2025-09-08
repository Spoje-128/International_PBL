#include "UltrasonicSensor.h"

// Speed of sound in cm/microsecond
const float SOUND_SPEED_CM_US = 0.0343;

UltrasonicSensor::UltrasonicSensor() {}

void UltrasonicSensor::init() {
  pinMode(LEFT_TRIG_PIN, OUTPUT);
  pinMode(LEFT_ECHO_PIN, INPUT);
  pinMode(CENTER_TRIG_PIN, OUTPUT);
  pinMode(CENTER_ECHO_PIN, INPUT);
  pinMode(RIGHT_TRIG_PIN, OUTPUT);
  pinMode(RIGHT_ECHO_PIN, INPUT);
  Serial.println("UltrasonicSensor Initialized");
}

void UltrasonicSensor::readDistances(float& left, float& center, float& right) {
  left = measureDistance(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
  // Add a small delay between measurements to prevent echo interference
  delay(10);
  center = measureDistance(CENTER_TRIG_PIN, CENTER_ECHO_PIN);
  delay(10);
  right = measureDistance(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);
}

float UltrasonicSensor::measureDistance(int trigPin, int echoPin) {
  // Clear the trigger pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Send a 10 microsecond pulse to trigger the sensor
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read the echo pin. pulseIn returns the duration of the pulse in microseconds.
  // The timeout is set to 30000 microseconds (approx. 5 meters, well beyond sensor range)
  long duration = pulseIn(echoPin, HIGH, 30000);
  
  // Calculate the distance
  // The sound wave travels to the obstacle and back, so we divide by 2
  float distance = (duration * SOUND_SPEED_CM_US) / 2.0;
  
  // Return 0 if the sensor timed out (no object in range)
  return distance > 0 ? distance : 0;
}
