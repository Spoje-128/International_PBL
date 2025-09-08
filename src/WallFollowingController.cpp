#include "WallFollowingController.h"
#include <Arduino.h> // For Serial

WallFollowingController::WallFollowingController(MotorControl* motorControl) : motors(motorControl) {}

void WallFollowingController::execute(float leftDist, float rightDist) {
  int leftSpeed = SEARCH_SPEED;
  int rightSpeed = SEARCH_SPEED;

  bool leftWall = (leftDist > 0 && leftDist < WALL_DETECT_THRESHOLD);
  bool rightWall = (rightDist > 0 && rightDist < WALL_DETECT_THRESHOLD);

  if (leftWall) {
    // Case 1: Left wall is detected. Follow it. (Highest priority)
    Serial.println("State: WALL_FOLLOW_L");
    float perp_dist_L = leftDist * SIN_26_5_DEG;
    float error = WALL_TARGET_DISTANCE - perp_dist_L;
    int correction = (int)(WALL_FOLLOW_KP * error);
    leftSpeed -= correction;
    rightSpeed += correction;
  } else if (rightWall) {
    // Case 2: No left wall, but there is a right wall.
    // Move forward and curve gently to the left to eventually find a wall on the left.
    Serial.println("State: SEARCH (CURVE LEFT)");
    leftSpeed = SEARCH_SPEED * 0.7; // Move left wheel slower to curve left
    rightSpeed = SEARCH_SPEED;
  } else {
    // Case 3: No walls on either side.
    // Turn on the spot to find a wall.
    Serial.println("State: SEARCH (SPIN RIGHT)");
    leftSpeed = TURN_SPEED;
    rightSpeed = -TURN_SPEED; // Turn right
  }

  int finalLeft = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  int finalRight = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);
  motors->setSpeeds(finalLeft, finalRight);
}
