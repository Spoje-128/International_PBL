#include "ServoController.h"

ServoController::ServoController() {}

void ServoController::init() {
  servo.attach(SERVO_PIN);
  reset(); // Start in the reset position
  Serial.println("ServoController Initialized");
}

void ServoController::attack() {
  servo.write(ATTACK_ANGLE);
  Serial.println("Servo: Attack!");
}

void ServoController::reset() {
  servo.write(RESET_ANGLE);
  Serial.println("Servo: Reset.");
}
