#include "MotorControl.h"

MotorControl::MotorControl() {
  // Set default motor directions (false = normal, true = reversed)
  // This might need to be adjusted based on wiring.
  rightMotorReversed = false;
  leftMotorReversed = true; // A common configuration
  
  // Initialize PWM values
  currentLeftPWM = 0;
  currentRightPWM = 0;
}

void MotorControl::init() {
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stop(); // Ensure motors are stopped initially
  Serial.println("MotorControl Initialized");
}

void MotorControl::setMotorDirection(bool rightReversed, bool leftReversed) {
  rightMotorReversed = rightReversed;
  leftMotorReversed = leftReversed;
}

// Private low-level functions
void MotorControl::rightWheelForward(int speed) {
  if (rightMotorReversed) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  analogWrite(ENA, speed);
}

void MotorControl::rightWheelBackward(int speed) {
  if (rightMotorReversed) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  analogWrite(ENA, speed);
}

void MotorControl::rightWheelStop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
}

void MotorControl::leftWheelForward(int speed) {
  if (leftMotorReversed) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  analogWrite(ENB, speed);
}

void MotorControl::leftWheelBackward(int speed) {
  if (leftMotorReversed) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  analogWrite(ENB, speed);
}

void MotorControl::leftWheelStop() {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}

// Public high-level functions
void MotorControl::stop() {
  leftWheelStop();
  rightWheelStop();
  currentLeftPWM = 0;
  currentRightPWM = 0;
}

void MotorControl::setSpeeds(int left_pwm, int right_pwm) {
  // Store current PWM values for debugging
  currentLeftPWM = left_pwm;
  currentRightPWM = right_pwm;
  
  // Handle left motor
  if (left_pwm > 0) {
    leftWheelForward(constrain(left_pwm, 0, 255));
  } else if (left_pwm < 0) {
    leftWheelBackward(constrain(abs(left_pwm), 0, 255));
  } else {
    leftWheelStop();
  }

  // Handle right motor
  if (right_pwm > 0) {
    rightWheelForward(constrain(right_pwm, 0, 255));
  } else if (right_pwm < 0) {
    rightWheelBackward(constrain(abs(right_pwm), 0, 255));
  } else {
    rightWheelStop();
  }
}

void MotorControl::forward(int speed) {
  setSpeeds(speed, speed);
}

void MotorControl::backward(int speed) {
  setSpeeds(-speed, -speed);
}

void MotorControl::turnLeft(int speed) {
  setSpeeds(-speed, speed);
}

void MotorControl::turnRight(int speed) {
  setSpeeds(speed, -speed);
}
