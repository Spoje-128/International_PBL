#include "UltrasonicSensor.h"

// Speed of sound in cm/microsecond
const float SOUND_SPEED_CM_US = 0.0343;
// Timeout for pulseIn. 20000 us is ~344 cm, a safe max range.
// The key is that it prevents pulseIn from blocking forever if no echo is received.
const unsigned long SENSOR_TIMEOUT_US = 20000;

UltrasonicSensor::UltrasonicSensor() {}

void UltrasonicSensor::init() {
  pinMode(LEFT_TRIG_PIN, OUTPUT);
  pinMode(LEFT_ECHO_PIN, INPUT);
  pinMode(CENTER_TRIG_PIN, OUTPUT);
  pinMode(CENTER_ECHO_PIN, INPUT);
  pinMode(RIGHT_TRIG_PIN, OUTPUT);
  pinMode(RIGHT_ECHO_PIN, INPUT);
  Serial.println("UltrasonicSensor Initialized (Fast-Blocking)");
}

void UltrasonicSensor::readDistances(float& left, float& center, float& right) {
  // Read each sensor sequentially. A small delay between pings prevents interference.
  left = measureDistance(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
  delay(1);
  center = measureDistance(CENTER_TRIG_PIN, CENTER_ECHO_PIN);
  delay(1);
  right = measureDistance(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);
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
