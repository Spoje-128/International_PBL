#include "UltrasonicSensor.h"

// Speed of sound in cm/microsecond
const float SOUND_SPEED_CM_US = 0.0343;
// Timeout for pulseIn. 20000 us is ~344 cm, a safe max range.
// The key is that it prevents pulseIn from blocking forever if no echo is received.
const unsigned long SENSOR_TIMEOUT_US = 20000;

UltrasonicSensor::UltrasonicSensor() : filteredLeft(0), filteredCenter(0), filteredRight(0) {}

void UltrasonicSensor::init() {
  pinMode(LEFT_TRIG_PIN, OUTPUT);
  pinMode(LEFT_ECHO_PIN, INPUT);
  pinMode(CENTER_TRIG_PIN, OUTPUT);
  pinMode(CENTER_ECHO_PIN, INPUT);
  pinMode(RIGHT_TRIG_PIN, OUTPUT);
  pinMode(RIGHT_ECHO_PIN, INPUT);
  Serial.println("UltrasonicSensor Initialized (Fast-Blocking with LPF)");
}

void UltrasonicSensor::readDistances(float& left, float& center, float& right) {
  // 1. Read raw distances
  float rawLeft = measureDistance(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
  delay(1);
  float rawCenter = measureDistance(CENTER_TRIG_PIN, CENTER_ECHO_PIN);
  delay(1);
  float rawRight = measureDistance(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);

  // 2. Apply Low-Pass Filter
  // If the sensor times out (returns 0), we don't want to drag the filtered value to 0.
  // Instead, we just keep the previous filtered value.
  if (rawLeft > 0) {
    filteredLeft = (LPF_ALPHA * rawLeft) + ((1.0 - LPF_ALPHA) * filteredLeft);
  }
  if (rawCenter > 0) {
    filteredCenter = (LPF_ALPHA * rawCenter) + ((1.0 - LPF_ALPHA) * filteredCenter);
  }
  if (rawRight > 0) {
    filteredRight = (LPF_ALPHA * rawRight) + ((1.0 - LPF_ALPHA) * filteredRight);
  }

  // 3. Return the filtered values
  left = filteredLeft;
  center = filteredCenter;
  right = filteredRight;
}

float UltrasonicSensor::measureDistance(int trigPin, int echoPin) {
  // Send the trigger pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read the echo pulse with a timeout.
  long duration = pulseIn(echoPin, HIGH, SENSOR_TIMEOUT_US);
  
  // Calculate distance, or return 0 if timed out.
  if (duration > 0) {
    return (duration * SOUND_SPEED_CM_US) / 2.0;
  } else {
    return 0;
  }
}
