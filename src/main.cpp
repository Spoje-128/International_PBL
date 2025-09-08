#include <Arduino.h>
#include "MotorControl.h"
#include "UltrasonicSensor.h"
#include "PixyCam.h"
#include "ServoController.h"
#include "PIDController.h"
#include "WallFollowingController.h"

// --- Tuning Constants ---
const int TRACKING_BASE_SPEED = 160;
const int TURN_SPEED = 150;
const int MAX_SPEED = 255;

const float COLLISION_THRESHOLD_FRONT_DEFAULT = 12.0; // Slightly increased for cylinders
const float COLLISION_THRESHOLD_FRONT_TRACKING = 4.0; // Allow getting closer to the target
const float COLLISION_THRESHOLD_SIDE = 7.0;

const uint8_t TARGET_SIGNATURE = 1;
const int PIXY_CENTER_X = PixyCam::PIXY_FRAME_WIDTH / 2;
const int TARGET_DEADZONE_X = 20;
const int TARGET_ATTACK_AREA = 12000;

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
WallFollowingController wallFollower(&motors);

// --- Stuck Detection ---
unsigned long lastSignificantMoveTime = 0;
float lastStuckCheckLeftDist = 0;
float lastStuckCheckCenterDist = 0;
float lastStuckCheckRightDist = 0;
const unsigned long STUCK_TIMEOUT = 2000; // 2 seconds
const float STUCK_DISTANCE_TOLERANCE = 1.0; // 1 cm

// --- Debugging ---
unsigned long lastDebugPrintTime = 0;
const unsigned long DEBUG_PRINT_INTERVAL = 500;
String debugState = "STARTING";

// --- Function Prototypes ---
void handleSerialTuning();
void printDebugInfo(float l, float c, float r, bool targetVisible);
bool handleStuckCondition(float l, float c, float r, int currentL, int currentR);
bool handleAttack(bool targetVisible);

// New architecture prototypes
int calculateCorrection(bool targetVisible, float l, float c, float r);


// --- Setup ---
void setup() {
  Serial.begin(9600);
  motors.init();
  ultrasonic.init();
  pixy.init();
  servo.init();
  Serial.println("Initialization complete. Starting main loop.");
  Serial.println("Send 'pX.X', 'iX.X', or 'dX.X' to tune PID gains.");
}

// --- Main Loop: Base Speed + Correction Model ---
void loop() {
  handleSerialTuning();

  // 1. Read Sensors
  float leftDist, centerDist, rightDist;
  ultrasonic.readDistances(leftDist, centerDist, rightDist);
  bool targetVisible = pixy.getBestBlock(TARGET_SIGNATURE, targetBlock);

  // 2. Handle Terminal States (Stuck, Attack)
  if (handleStuckCondition(leftDist, centerDist, rightDist, motors.getLeftSpeed(), motors.getRightSpeed())) {
    return; // Escape maneuver was executed
  }
  if (handleAttack(targetVisible)) {
    return; // Stop processing further behaviors if attacking
  }

  // 3. Determine base speed and correction
  const int SEARCH_BASE_SPEED = 140;
  int baseSpeed = targetVisible ? TRACKING_BASE_SPEED : SEARCH_BASE_SPEED;
  int turnCorrection = calculateCorrection(targetVisible, leftDist, centerDist, rightDist);

  // 4. Set Motor Speeds
  int leftSpeed = baseSpeed - turnCorrection;
  int rightSpeed = baseSpeed + turnCorrection;

  // 5. Scale and Constrain Outputs for Motors
  int motorL, motorR;
  if (leftSpeed > 0) {
    motorL = map(leftSpeed, 1, MAX_SPEED, 130, MAX_SPEED);
  } else if (leftSpeed < 0) {
    motorL = map(leftSpeed, -MAX_SPEED, -1, -MAX_SPEED, -130);
  } else {
    motorL = 0;
  }

  if (rightSpeed > 0) {
    motorR = map(rightSpeed, 1, MAX_SPEED, 130, MAX_SPEED);
  } else if (rightSpeed < 0) {
    motorR = map(rightSpeed, -MAX_SPEED, -1, -MAX_SPEED, -130);
  } else {
    motorR = 0;
  }

  motors.setSpeeds(motorL, motorR);


  // 5. Debug Print
  if (millis() - lastDebugPrintTime > DEBUG_PRINT_INTERVAL) {
    printDebugInfo(leftDist, centerDist, rightDist, targetVisible);
    lastDebugPrintTime = millis();
  }
}


// --- State Handlers ---

bool handleStuckCondition(float l, float c, float r, int currentL, int currentR) {
  // We only check for being stuck if the motors are supposed to be moving.
  if (currentL != 0 || currentR != 0) {
    // Check if sensor readings have changed significantly
    if (abs(l - lastStuckCheckLeftDist) > STUCK_DISTANCE_TOLERANCE ||
        abs(c - lastStuckCheckCenterDist) > STUCK_DISTANCE_TOLERANCE ||
        abs(r - lastStuckCheckRightDist) > STUCK_DISTANCE_TOLERANCE) {
      // Robot is moving, update timestamp and readings
      lastSignificantMoveTime = millis();
      lastStuckCheckLeftDist = l;
      lastStuckCheckCenterDist = c;
      lastStuckCheckRightDist = r;
    }
  } else {
    // Motors are stopped, so we are not "stuck", reset timer.
    lastSignificantMoveTime = millis();
  }

  // Check if the timeout has been exceeded
  if (millis() - lastSignificantMoveTime > STUCK_TIMEOUT) {
    debugState = "ESCAPING";
    Serial.println("STUCK! Executing escape maneuver.");
    // Execute a decisive backup and turn
    motors.backward(200);
    delay(500);
    motors.turnLeft(200);
    delay(800);
    motors.stop();
    // Reset timers to prevent immediate re-triggering
    lastSignificantMoveTime = millis();
    pidController.reset();
    return true; // Escape maneuver was performed
  }

  return false; // Not stuck
}


// --- New Main Logic ---

int calculateCorrection(bool targetVisible, float l, float c, float r) {
    // 1. Obstacle Avoidance (Highest Priority)
    // Front sensor is critical
    if (c > 0 && c < COLLISION_THRESHOLD_FRONT_DEFAULT) {
        debugState = "AVOID_FRONT";
        // Turn towards the side with more space
        if (l > r) {
            return -150; // Turn Right Sharply
        } else {
            return 150;  // Turn Left Sharply
        }
    }
    // Side sensors
    if (l > 0 && l < COLLISION_THRESHOLD_SIDE) {
        debugState = "AVOID_LEFT";
        // Turn right, away from left wall. Proportional control.
        int correction = (int)(10.0 * (COLLISION_THRESHOLD_SIDE - l));
        return -correction; // Negative correction turns right
    }
    if (r > 0 && r < COLLISION_THRESHOLD_SIDE) {
        debugState = "AVOID_RIGHT";
        // Turn left, away from right wall.
        int correction = (int)(10.0 * (COLLISION_THRESHOLD_SIDE - r));
        return correction; // Positive correction turns left
    }

    // 2. Target Tracking
    if (targetVisible) {
        debugState = "TRACKING";
        int error = targetBlock.m_x - PIXY_CENTER_X;
        // PID controller calculates the necessary correction
        return (int)pidController.calculate(error);
    }

    // 3. Searching
    debugState = "SEARCHING";
    pidController.reset();
    // Gentle right turn to search
    return -30;
}


// --- Behavior Implementations (Old - Kept for reference) ---

/*
bool handleCollision(float l, float c, float r, bool targetVisible) {
  // ... (Old code commented out)
}
*/

bool handleAttack(bool targetVisible) {
  if (targetVisible && (targetBlock.m_width * targetBlock.m_height > TARGET_ATTACK_AREA)) {
    debugState = "ATTACKING";
    motors.stop();
    servo.attack();
    delay(1000);
    servo.reset();
    delay(500);
    motors.turnLeft(TURN_SPEED);
    delay(1000);
    motors.stop();
    pidController.reset();
    pixy.resetTracking(); // Forget the old target
    return true;
  }
  return false;
}

/*
void handleTracking() {
  // ... (Old code commented out)
}

void handleSearching(float l, float r) {
  // ... (Old code commented out)
}
*/

// --- Utility Functions ---

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

void printDebugInfo(float l, float c, float r, bool targetVisible) {
    // Simplified debug output for the new architecture
    Serial.print("State: ");
    if (targetVisible) {
      Serial.print("TRACKING");
    } else {
      Serial.print("SEARCHING");
    }
    Serial.print(" | Ultra (L,C,R): ");
    Serial.print(l, 0); Serial.print(", ");
    Serial.print(c, 0); Serial.print(", ");
    Serial.print(r, 0);

    if (targetVisible) {
        Serial.print(" | Target(X,A): ");
        Serial.print(targetBlock.m_x); Serial.print(", ");
        Serial.print(targetBlock.m_width * targetBlock.m_height);
    }
    Serial.println();
}
