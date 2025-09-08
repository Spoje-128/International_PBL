#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <Arduino.h>
#include <Servo.h>

class ServoController {
public:
  ServoController();
  void init();

  // Moves the servo to the "attack" position
  void attack();

  // Resets the servo to the "resting" position
  void reset();

private:
  // Pin for the servo, as per pinAssignment.md
  static const int SERVO_PIN = 3;
  
  // The angle for the attack position (0 deg)
  static const int ATTACK_ANGLE = 0;

  // The angle for the resting position (90 deg, swung up)
  static const int RESET_ANGLE = 90;

  Servo servo;
};

#endif // SERVO_CONTROLLER_H
