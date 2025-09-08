#include <Arduino.h>

// ====================================================================================
// !! WARNING !!
// Per user instruction, this code is configured to use pins 0 and 1 for the left
// ultrasonic sensor. These are the main serial communication pins on the
// Arduino Mega. You will not be able to use the Serial Monitor for debugging
// while the sensor is connected to these pins.
// ====================================================================================

#include "MotorControl.h"
#include "UltrasonicSensor.h"
#include "PixyCam.h"
#include "ServoController.h"
#include "PIDController.h"

// --- Tuning Constants ---
// Speeds (PWM range 130-255 as per requirements)
const int SEARCH_SPEED = 140;
const int TRACKING_BASE_SPEED = 160;
const int TURN_SPEED = 150;
const int MAX_SPEED = 255;

// Distances (in cm)
const float COLLISION_THRESHOLD_FRONT_DEFAULT = 10.0;
const float COLLISION_THRESHOLD_FRONT_TRACKING = 3.0; // Allow getting closer to the target
const float COLLISION_THRESHOLD_SIDE = 7.0;

// Wall Following
const float WALL_TARGET_DISTANCE = 12.0;
const float WALL_DETECT_THRESHOLD = 35.0;
const float WALL_FOLLOW_KP = 2.5;
const float SIN_26_5_DEG = 0.4462;

// PIXY - Target Tracking
const uint8_t TARGET_SIGNATURE = 1;
const int PIXY_CENTER_X = PixyCam::PIXY_FRAME_WIDTH / 2;
const int TARGET_DEADZONE_X = 20;
const int TARGET_ATTACK_AREA = 12000;

// PID gains for steering correction
float KP = 0.4;
float KI = 0.02;
float KD = 0.1;

// --- Global Objects ---
MotorControl motors;
UltrasonicSensor ultrasonic;
PixyCam pixy;
ServoController servo;
Block targetBlock;
PIDController pidController(KP, KI, KD);

// --- State/Timing Variables ---
String robotState = "STARTING";

// --- Debugging ---
unsigned long lastDebugPrintTime = 0;
const unsigned long DEBUG_PRINT_INTERVAL = 500;

// --- Function Prototypes ---
void handleSerialTuning();
void printDebugInfo(float l, float c, float r, bool targetVisible, float front_thresh);

// --- Setup ---
void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("Robot starting up...");
  motors.init();
  ultrasonic.init();
  pixy.init();
  servo.init();

  Serial.println("Initialization complete. Starting main loop.");
  Serial.println("Send 'pX.X', 'iX.X', or 'dX.X' to tune PID gains.");
}

// --- Main Loop ---
void loop() {
  handleSerialTuning();

  float leftDist, centerDist, rightDist;
  ultrasonic.readDistances(leftDist, centerDist, rightDist);
  bool targetVisible = pixy.getBestBlock(TARGET_SIGNATURE, targetBlock);

  // Dynamically adjust front collision threshold if a target is visible
  float current_front_threshold = targetVisible ? COLLISION_THRESHOLD_FRONT_TRACKING : COLLISION_THRESHOLD_FRONT_DEFAULT;

  // --- Behavior 1: Collision Avoidance (Highest Priority) ---
  if ((centerDist > 0 && centerDist < current_front_threshold) ||
      (leftDist > 0 && leftDist < COLLISION_THRESHOLD_SIDE) ||
      (rightDist > 0 && rightDist < COLLISION_THRESHOLD_SIDE)) {

    robotState = "AVOIDING";
    motors.stop();
    motors.backward(TURN_SPEED);
    delay(400);

    if (leftDist > rightDist) {
      motors.turnLeft(TURN_SPEED);
    } else {
      motors.turnRight(TURN_SPEED);
    }
    delay(600);
    motors.stop();
    pidController.reset();
    return;
  }
  // --- Behavior 2: Target Tracking and Attack ---
  else if (targetVisible) {
    int area = targetBlock.m_width * targetBlock.m_height;

    if (area > TARGET_ATTACK_AREA) {
      robotState = "ATTACKING";
      motors.stop();
      servo.attack();
      delay(1000);
      servo.reset();
      delay(500);
      motors.turnLeft(TURN_SPEED);
      delay(1000);
      motors.stop();
      pidController.reset();
      return;
    }

    robotState = "TRACKING";
    int error = targetBlock.m_x - PIXY_CENTER_X;

    if (abs(error) <= TARGET_DEADZONE_X) {
      motors.forward(TRACKING_BASE_SPEED);
      pidController.reset();
    } else {
      int speedCorrection = (int)pidController.calculate(error);
      int leftSpeed = constrain(TRACKING_BASE_SPEED - speedCorrection, 130, MAX_SPEED);
      int rightSpeed = constrain(TRACKING_BASE_SPEED + speedCorrection, 130, MAX_SPEED);
      motors.setSpeeds(leftSpeed, rightSpeed);
    }
  }
  // --- Behavior 3: Search / Wall Following (Lowest Priority) ---
  else {
    pidController.reset();
    int leftSpeed = SEARCH_SPEED;
    int rightSpeed = SEARCH_SPEED;

    bool leftWall = (leftDist > 0 && leftDist < WALL_DETECT_THRESHOLD);

    if (leftWall) {
        // "Left-hand rule": always prioritize following the left wall.
        robotState = "WALL_FOLLOW_L";
        float perp_dist_L = leftDist * SIN_26_5_DEG;
        float error = WALL_TARGET_DISTANCE - perp_dist_L;
        int correction = (int)(WALL_FOLLOW_KP * error);
        leftSpeed -= correction;
        rightSpeed += correction;
    } else {
        // No left wall found, turn right to find one.
        robotState = "SEARCH (FIND WALL)";
        leftSpeed = TURN_SPEED;
        rightSpeed = -TURN_SPEED; // Turn right
    }

    int finalLeft = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
    int finalRight = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);
    motors.setSpeeds(finalLeft, finalRight);
  }

  if (millis() - lastDebugPrintTime > DEBUG_PRINT_INTERVAL) {
    printDebugInfo(leftDist, centerDist, rightDist, targetVisible, current_front_threshold);
    lastDebugPrintTime = millis();
  }
}

void handleSerialTuning() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() < 2) return;

    char gainType = input.charAt(0);
    float value = input.substring(1).toFloat();

    switch (gainType) {
      case 'p': case 'P': KP = value; break;
      case 'i': case 'I': KI = value; break;
      case 'd': case 'D': KD = value; break;
      default: Serial.println("Invalid gain type."); return;
    }
    pidController.setGains(KP, KI, KD);
    pidController.reset();
    Serial.print("Gains set: P="); Serial.print(KP);
    Serial.print(", I="); Serial.print(KI);
    Serial.print(", D="); Serial.println(KD);
  }
}

void printDebugInfo(float l, float c, float r, bool targetVisible, float front_thresh) {
    Serial.print("State: "); Serial.print(robotState);
    Serial.print(" | Ultra (L,C,R): ");
    Serial.print(l, 0); Serial.print(", ");
    Serial.print(c, 0); Serial.print(", ");
    Serial.print(r, 0);
    Serial.print(" | Front Thresh: "); Serial.print(front_thresh, 1);

    if (robotState == "TRACKING") {
        Serial.print(" | Target (X,Area): ");
        Serial.print(targetBlock.m_x); Serial.print(", ");
        Serial.print(targetBlock.m_width * targetBlock.m_height);
    }
    Serial.println();
}
