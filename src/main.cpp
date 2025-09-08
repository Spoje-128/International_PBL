#include <Arduino.h>

// ====================================================================================
// !! WARNING !!
// This code is configured to use pins 0 and 1 for the left ultrasonic sensor,
// as specified in pinAssignment.md. These are the main serial communication pins
// on the Arduino Mega. You will not be able to use the Serial Monitor for
// debugging while the sensor is connected to these pins.
// ====================================================================================

#include "MotorControl.h"
#include "UltrasonicSensor.h"
#include "PixyCam.h"
#include "ServoController.h"

// --- Tuning Constants ---
// Speeds (PWM range 130-255 as per requirements)
const int SEARCH_SPEED = 140;
const int TRACKING_BASE_SPEED = 160;
const int TURN_SPEED = 150;
const int MAX_SPEED = 255;

// Distances (in cm)
const float COLLISION_THRESHOLD_FRONT = 15.0;
const float COLLISION_THRESHOLD_SIDE = 10.0;

// PIXY - Target Tracking
const uint8_t TARGET_SIGNATURE = 1;
const int PIXY_CENTER_X = PixyCam::PIXY_FRAME_WIDTH / 2;
const int TARGET_DEADZONE_X = 20; // Pixels from center to consider "centered"
const int TARGET_ATTACK_AREA = 12000; // PIXY block area to trigger attack
const float KP = 0.4; // Proportional gain for steering correction

// --- Global Objects ---
MotorControl motors;
UltrasonicSensor ultrasonic;
PixyCam pixy;
ServoController servo;
Block targetBlock; // To store data of the detected target

// --- State/Timing Variables ---
unsigned long lastActionTime = 0;
const unsigned long SEARCH_TURN_DURATION = 1500; // ms
bool searchingLeft = true;

// --- Helper Functions ---
void printDistances(float l, float c, float r) {
  Serial.print("Dist L:");
  Serial.print(l, 0);
  Serial.print(" C:");
  Serial.print(c, 0);
  Serial.print(" R:");
  Serial.println(r, 0);
}

// --- Setup ---
void setup() {
  Serial.begin(9600);
  while (!Serial); // Wait for serial port to connect

  Serial.println("Robot starting up...");

  motors.init();
  ultrasonic.init();
  pixy.init();
  servo.init();

  // A small delay to ensure all sensors are stable
  delay(1000);
  Serial.println("Initialization complete. Starting main loop.");
}

// --- Main Loop ---
void loop() {
  // 1. Read all sensor data
  float leftDist, centerDist, rightDist;
  ultrasonic.readDistances(leftDist, centerDist, rightDist);
  bool targetVisible = pixy.getBestBlock(TARGET_SIGNATURE, targetBlock);

  // 2. Behavior-based control logic (highest priority first)

  // --- Behavior 1: Collision Avoidance ---
  if ((centerDist > 0 && centerDist < COLLISION_THRESHOLD_FRONT) ||
      (leftDist > 0 && leftDist < COLLISION_THRESHOLD_SIDE) ||
      (rightDist > 0 && rightDist < COLLISION_THRESHOLD_SIDE)) {

    Serial.println("--- AVOIDING COLLISION ---");
    printDistances(leftDist, centerDist, rightDist);

    // Stop forward motion
    motors.stop();

    // Simple avoidance: back up a little, then turn towards the most open space
    motors.backward(TURN_SPEED);
    delay(400); // Back up for a short duration

    if (leftDist > rightDist) {
      Serial.println("Turning left (more space)");
      motors.turnLeft(TURN_SPEED);
    } else {
      Serial.println("Turning right (more space)");
      motors.turnRight(TURN_SPEED);
    }
    delay(600); // Turn for a short duration
    motors.stop();
    return; // End this loop iteration
  }

  // --- Behavior 2: Target Tracking and Attack ---
  else if (targetVisible) {
    Serial.print("--- TRACKING TARGET ---, ");
    Serial.print("X:");
    Serial.print(targetBlock.x);
    Serial.print(" Area:");
    Serial.println(targetBlock.width * targetBlock.height);

    int area = targetBlock.width * targetBlock.height;

    // Check if target is close enough to attack
    if (area > TARGET_ATTACK_AREA) {
      Serial.println("!!! ATTACKING TARGET !!!");
      motors.stop();
      servo.attack();
      delay(1000); // Wait for servo to swing
      servo.reset();
      delay(500);

      // After attacking, turn a bit to look for a new target
      motors.turnLeft(TURN_SPEED);
      delay(1000);
      motors.stop();
      return;
    }

    // If not close enough, continue tracking
    int error = targetBlock.x - PIXY_CENTER_X;

    // If target is centered, move straight forward
    if (abs(error) <= TARGET_DEADZONE_X) {
      motors.forward(TRACKING_BASE_SPEED);
    }
    // If target is not centered, use P-controller to steer
    else {
      int speedCorrection = (int)(KP * error);

      int leftSpeed = TRACKING_BASE_SPEED - speedCorrection;
      int rightSpeed = TRACKING_BASE_SPEED + speedCorrection;

      // Constrain motor speeds to valid PWM range
      leftSpeed = constrain(leftSpeed, 130, MAX_SPEED);
      rightSpeed = constrain(rightSpeed, 130, MAX_SPEED);

      motors.setSpeeds(leftSpeed, rightSpeed);
    }
  }

  // --- Behavior 3: Search Mode ---
  else {
    Serial.println("--- SEARCHING ---");
    // Simple search: move forward and turn left/right periodically
    motors.forward(SEARCH_SPEED);

    if (millis() - lastActionTime > SEARCH_TURN_DURATION) {
      if (searchingLeft) {
        Serial.println("Searching: Turning left");
        motors.turnLeft(SEARCH_SPEED);
      } else {
        Serial.println("Searching: Turning right");
        motors.turnRight(SEARCH_SPEED);
      }
      // Invert the search direction for next time
      searchingLeft = !searchingLeft;
      lastActionTime = millis();
      // Let it turn for a bit
      delay(500);
    }
  }
}
