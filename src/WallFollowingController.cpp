#include "WallFollowingController.h"
#include <Arduino.h> // For Serial

WallFollowingController::WallFollowingController(MotorControl* motorControl) : motors(motorControl) {}

void WallFollowingController::execute(float leftDist, float rightDist) {
  int leftSpeed = SEARCH_SPEED;
  int rightSpeed = SEARCH_SPEED;

  bool leftWall = (leftDist > 0 && leftDist < WALL_DETECT_THRESHOLD);

  if (leftWall) {
    // "Left-hand rule": always prioritize following the left wall.
    Serial.println("State: WALL_FOLLOW_L");
    float perp_dist_L = leftDist * SIN_26_5_DEG;
    float error = WALL_TARGET_DISTANCE - perp_dist_L;
    int correction = (int)(WALL_FOLLOW_KP * error);
    leftSpeed -= correction;
    rightSpeed += correction;
  } else {
    // No left wall found, turn right to find one.
    Serial.println("State: SEARCH (FIND WALL)");
    leftSpeed = TURN_SPEED;
    rightSpeed = -TURN_SPEED; // Turn right
  }

  int finalLeft = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  int finalRight = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);
  motors->setSpeeds(finalLeft, finalRight);
}
