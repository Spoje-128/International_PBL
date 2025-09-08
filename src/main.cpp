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

// --- Debugging ---
unsigned long lastDebugPrintTime = 0;
const unsigned long DEBUG_PRINT_INTERVAL = 500;
String debugState = "STARTING";

// --- Function Prototypes ---
void handleSerialTuning();
void printDebugInfo(float l, float c, float r, bool targetVisible, float f_thresh);
bool handleCollision(float l, float c, float r, bool targetVisible);
bool handleAttack(bool targetVisible);
void handleTracking();
void handleSearching(float l, float r);

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

// --- Main Loop: High-Level Coordinator ---
void loop() {
  handleSerialTuning();

  float leftDist, centerDist, rightDist;
  ultrasonic.readDistances(leftDist, centerDist, rightDist);
  bool targetVisible = pixy.getBestBlock(TARGET_SIGNATURE, targetBlock);

  if (handleCollision(leftDist, centerDist, rightDist, targetVisible)) return;
  if (handleAttack(targetVisible)) return;

  if (targetVisible) {
    handleTracking();
  } else {
    handleSearching(leftDist, rightDist);
  }

  if (millis() - lastDebugPrintTime > DEBUG_PRINT_INTERVAL) {
    printDebugInfo(leftDist, centerDist, rightDist, targetVisible,
                   targetVisible ? COLLISION_THRESHOLD_FRONT_TRACKING : COLLISION_THRESHOLD_FRONT_DEFAULT);
    lastDebugPrintTime = millis();
  }
}

// --- Behavior Implementations ---

bool handleCollision(float l, float c, float r, bool targetVisible) {
  float front_thresh = targetVisible ? COLLISION_THRESHOLD_FRONT_TRACKING : COLLISION_THRESHOLD_FRONT_DEFAULT;
  bool front_coll = (c > 0 && c < front_thresh);
  bool side_coll = (l > 0 && l < COLLISION_THRESHOLD_SIDE) || (r > 0 && r < COLLISION_THRESHOLD_SIDE);

  if (front_coll || side_coll) {
    debugState = "AVOIDING";
    motors.stop();
    motors.backward(TURN_SPEED);
    delay(400);

    // If the collision is mainly from the front (like a cylinder),
    // perform a decisive 90-degree turn to clear it.
    if (front_coll && !(side_coll)) {
        motors.turnLeft(TURN_SPEED);
        delay(800); // 800ms should be roughly a 90-degree turn
    }
    // Otherwise, for side collisions or corner cases, turn toward the most open space.
    else {
        if (l > r) motors.turnLeft(TURN_SPEED);
        else motors.turnRight(TURN_SPEED);
        delay(600);
    }

    motors.stop();
    pidController.reset();
    return true;
  }
  return false;
}

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

void handleTracking() {
  debugState = "TRACKING";
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

void handleSearching(float l, float r) {
  debugState = "SEARCH/WALL_FOLLOW";
  pidController.reset();
  pixy.resetTracking(); // Ensure we don't have a stale target lock
  wallFollower.execute(l, r);
}

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

void printDebugInfo(float l, float c, float r, bool targetVisible, float f_thresh) {
    Serial.print("State: "); Serial.print(debugState);
    Serial.print(" | Ultra (L,C,R): ");
    Serial.print(l, 0); Serial.print(", ");
    Serial.print(c, 0); Serial.print(", ");
    Serial.print(r, 0);
    Serial.print(" | F_Thresh: "); Serial.print(f_thresh, 1);
    if (debugState == "TRACKING") {
        Serial.print(" | Target(X,A): ");
        Serial.print(targetBlock.m_x); Serial.print(", ");
        Serial.print(targetBlock.m_width * targetBlock.m_height);
    }
    Serial.println();
}
