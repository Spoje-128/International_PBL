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
void calculateMotorOutputs(int& finalLeft, int& finalRight, bool targetVisible, float l, float c, float r);
void calculateTrackingOutput(int& left, int& right, bool targetVisible);
void calculateAvoidanceOutput(int& left, int& right, float l_dist, float c_dist, float r_dist);


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

// --- Main Loop: Refactored for Behavior Blending ---
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

  // 3. Calculate and Blend Behaviors
  int finalLeft = 0;
  int finalRight = 0;
  calculateMotorOutputs(finalLeft, finalRight, targetVisible, leftDist, centerDist, rightDist);

  // 4. Scale and Constrain Outputs for Motors
  // This logic maps the blended output to the motor's effective PWM range.
  int motorL, motorR;
  if (finalLeft > 0) {
    motorL = map(finalLeft, 1, MAX_SPEED, 130, MAX_SPEED);
  } else if (finalLeft < 0) {
    motorL = map(finalLeft, -MAX_SPEED, -1, -MAX_SPEED, -130);
  } else {
    motorL = 0;
  }

  if (finalRight > 0) {
    motorR = map(finalRight, 1, MAX_SPEED, 130, MAX_SPEED);
  } else if (finalRight < 0) {
    motorR = map(finalRight, -MAX_SPEED, -1, -MAX_SPEED, -130);
  } else {
    motorR = 0;
  }

  // 5. Actuate Motors
  motors.setSpeeds(motorL, motorR);


  // 5. Debug Print
  if (millis() - lastDebugPrintTime > DEBUG_PRINT_INTERVAL) {
    printDebugInfo(leftDist, centerDist, rightDist, targetVisible);
    lastDebugPrintTime = millis();
  }
}

// --- Behavior Blending ---

void calculateMotorOutputs(int& finalLeft, int& finalRight, bool targetVisible, float l, float c, float r) {
    int trackL = 0, trackR = 0;
    int avoidL = 0, avoidR = 0;

    calculateTrackingOutput(trackL, trackR, targetVisible);
    calculateAvoidanceOutput(avoidL, avoidR, l, c, r);

    // --- Dynamic Weighting ---
    float avoidWeight = 0.0;
    float min_dist = 1000.0;
    if (l > 0) min_dist = min(min_dist, l);
    if (c > 0) min_dist = min(min_dist, c);
    if (r > 0) min_dist = min(min_dist, r);

    if (min_dist < COLLISION_THRESHOLD_FRONT_DEFAULT) {
        // As the robot gets closer to an object, the weight of avoidance increases quadratically
        avoidWeight = 1.0 - (min_dist / COLLISION_THRESHOLD_FRONT_DEFAULT);
        avoidWeight *= avoidWeight;
    }

    float trackWeight = 1.0 - avoidWeight;

    // --- Blending ---
    // We blend the 'desire' vectors from each behavior
    finalLeft = (int)(trackWeight * trackL + avoidWeight * avoidL);
    finalRight = (int)(trackWeight * trackR + avoidWeight * avoidR);

    // TODO: Add stuck detection logic here in Step 5

    // The final values are constrained in the main loop after all calculations.
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


// --- Behavior Implementations (New) ---

void calculateTrackingOutput(int& left, int& right, bool targetVisible) {
    if (targetVisible) {
        debugState = "TRACKING";
        int error = targetBlock.m_x - PIXY_CENTER_X;

        // Use PID controller to get a turn correction value
        int speedCorrection = (int)pidController.calculate(error);

        // The base speed is forward, and we adjust each wheel based on the correction
        left = TRACKING_BASE_SPEED - speedCorrection;
        right = TRACKING_BASE_SPEED + speedCorrection;

    } else {
        debugState = "SEARCHING";
        pidController.reset(); // Reset PID when no target is in sight
        // Gentle right turn to search for targets
        left = 150;
        right = 135;
    }
}

void calculateAvoidanceOutput(int& left, int& right, float l_dist, float c_dist, float r_dist) {
    left = 0;
    right = 0;
    int repulsion_strength = 0;

    // Center sensor has highest priority and generates a sharp turn
    if (c_dist > 0 && c_dist < COLLISION_THRESHOLD_FRONT_DEFAULT) {
        // Obstacle directly ahead. Generate a strong turn command.
        // We turn based on which side has more space, or default to left.
        if (l_dist > r_dist) {
            left = -TURN_SPEED; // Turn right
            right = TURN_SPEED;
        } else {
            left = TURN_SPEED; // Turn left
            right = -TURN_SPEED;
        }
        return; // This is a critical avoidance, override side sensors
    }

    // Side sensors generate proportional repulsion
    if (l_dist > 0 && l_dist < COLLISION_THRESHOLD_SIDE) {
        repulsion_strength = (int)(100.0 * (1.0 - (l_dist / COLLISION_THRESHOLD_SIDE)));
        left -= repulsion_strength;  // Slow down left wheel to turn right
        right += repulsion_strength; // Speed up right wheel to turn right
    }

    if (r_dist > 0 && r_dist < COLLISION_THRESHOLD_SIDE) {
        repulsion_strength = (int)(100.0 * (1.0 - (r_dist / COLLISION_THRESHOLD_SIDE)));
        left += repulsion_strength;  // Speed up left wheel to turn left
        right -= repulsion_strength; // Slow down right wheel to turn left
    }
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
