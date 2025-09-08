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
const float COLLISION_THRESHOLD_FRONT = 15.0;
const float COLLISION_THRESHOLD_SIDE = 10.0;

// Wall Following
const float WALL_TARGET_DISTANCE = 15.0; // Target perpendicular distance from wall (cm)
const float WALL_DETECT_THRESHOLD = 40.0; // Max distance to consider a wall reading valid
const float WALL_FOLLOW_KP = 2.5; // Proportional gain for wall following
const float SIN_26_5_DEG = 0.4462; // sin(26.5 deg)

// PIXY - Target Tracking
const uint8_t TARGET_SIGNATURE = 1;
const int PIXY_CENTER_X = PixyCam::PIXY_FRAME_WIDTH / 2;
const int TARGET_DEADZONE_X = 20; // Pixels from center to consider "centered"
const int TARGET_ATTACK_AREA = 12000; // PIXY block area to trigger attack
const float KP = 0.4; // Proportional gain
const float KI = 0.02; // Integral gain
const float KD = 0.1; // Derivative gain

// --- Global Objects ---
MotorControl motors;
UltrasonicSensor ultrasonic;
PixyCam pixy;
ServoController servo;
Block targetBlock;
PIDController pidController(KP, KI, KD);

// --- State/Timing Variables ---
unsigned long lastSearchTurnTime = 0;
const unsigned long SEARCH_TURN_INTERVAL = 1500; // ms
bool searchingLeft = true;

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
}

// --- Main Loop ---
void loop() {
  // 1. Read all sensor data
  float leftDist, centerDist, rightDist;
  ultrasonic.readDistances(leftDist, centerDist, rightDist);
  bool targetVisible = pixy.getBestBlock(TARGET_SIGNATURE, targetBlock);

  // --- Behavior 1: Collision Avoidance (Highest Priority) ---
  if ((centerDist > 0 && centerDist < COLLISION_THRESHOLD_FRONT) ||
      (leftDist > 0 && leftDist < COLLISION_THRESHOLD_SIDE) ||
      (rightDist > 0 && rightDist < COLLISION_THRESHOLD_SIDE)) {

    Serial.println("--- AVOIDING COLLISION ---");
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
    pidController.reset(); // Reset PID state after avoidance
    return;
  }

  // --- Behavior 2: Target Tracking and Attack ---
  else if (targetVisible) {
    int area = targetBlock.width * targetBlock.height;

    if (area > TARGET_ATTACK_AREA) {
      Serial.println("!!! ATTACKING TARGET !!!");
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

    Serial.println("--- TRACKING TARGET ---");
    int error = targetBlock.x - PIXY_CENTER_X;

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
    pidController.reset(); // Reset PID if no target is visible
    int leftSpeed = SEARCH_SPEED;
    int rightSpeed = SEARCH_SPEED;

    // Determine base action: search turn or move forward
    if (millis() - lastSearchTurnTime > SEARCH_TURN_INTERVAL) {
        lastSearchTurnTime = millis();
        searchingLeft = !searchingLeft;
        Serial.println("--- SEARCHING (TURNING) ---");
        if (searchingLeft) {
            leftSpeed = -TURN_SPEED;
            rightSpeed = TURN_SPEED;
        } else {
            leftSpeed = TURN_SPEED;
            rightSpeed = -TURN_SPEED;
        }
    }
    // else, default is forward (leftSpeed = rightSpeed = SEARCH_SPEED)

    // If moving forward, check for walls and apply correction
    bool isMovingForward = (leftSpeed == rightSpeed);
    if(isMovingForward) {
        float perp_dist_L = leftDist * SIN_26_5_DEG;
        float perp_dist_R = rightDist * SIN_26_5_DEG;
        bool leftWall = (leftDist > 0 && leftDist < WALL_DETECT_THRESHOLD);
        bool rightWall = (rightDist > 0 && rightDist < WALL_DETECT_THRESHOLD);
        int correction = 0;

        if (leftWall && !rightWall) {
            Serial.println("--- FOLLOWING LEFT WALL ---");
            float error = WALL_TARGET_DISTANCE - perp_dist_L;
            correction = (int)(WALL_FOLLOW_KP * error);
        } else if (!leftWall && rightWall) {
            Serial.println("--- FOLLOWING RIGHT WALL ---");
            float error = perp_dist_R - WALL_TARGET_DISTANCE;
            correction = (int)(WALL_FOLLOW_KP * error);
        } else if (leftWall && rightWall) {
            Serial.println("--- CENTERING BETWEEN WALLS ---");
            float error = perp_dist_R - perp_dist_L;
            correction = (int)(WALL_FOLLOW_KP * error * 0.5);
        } else {
            Serial.println("--- SEARCHING (NO WALLS) ---");
        }
        leftSpeed -= correction;
        rightSpeed += correction;
    }
    // Constrain speeds, allowing for negative values for turning
    int finalLeft = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
    int finalRight = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);
    motors.setSpeeds(finalLeft, finalRight);
  }
}
