#ifndef WALL_FOLLOWING_CONTROLLER_H
#define WALL_FOLLOWING_CONTROLLER_H

#include "MotorControl.h"

class WallFollowingController {
public:
  WallFollowingController(MotorControl* motorControl);

  // Calculates and sets motor speeds for wall-following or searching.
  void execute(float leftDist, float rightDist);

private:
  MotorControl* motors;

  // Constants for wall following behavior
  static const int SEARCH_SPEED = 140;
  static const int TURN_SPEED = 150;
  static const int MAX_SPEED = 255;
  static constexpr float WALL_TARGET_DISTANCE = 12.0;
  static constexpr float WALL_DETECT_THRESHOLD = 35.0;
  static constexpr float WALL_FOLLOW_KP = 2.5;
  static constexpr float SIN_26_5_DEG = 0.4462;
};

#endif // WALL_FOLLOWING_CONTROLLER_H
