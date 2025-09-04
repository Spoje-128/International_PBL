#ifndef WALL_FOLLOWING_CONTROLLER_H
#define WALL_FOLLOWING_CONTROLLER_H

#include <Arduino.h>
#include "MotorControl.h"
#include "UltrasonicSensor.h"

class WallFollowingController {
public:
  WallFollowingController();
  void init(MotorControl* motorController, UltrasonicSensor* ultrasonicSensor);
  void update();
  void setParameters(float y_ref, float K_gain, float v_base);
  void setVehiclePose(float x, float y, float phi);
  
  // Core functions
  void updateSensorPosition();
  float computeControl();
  void setMotorSpeeds(float v_left, float v_right);
  
  // Getters for debugging
  float getSensorX() const { return x_s; }
  float getSensorY() const { return y_s; }
  float getHeadingRate() const { return heading_rate; }
  
private:
  // Sensor geometry (in mm, converted to m internally)
  static constexpr float SENSOR_OFFSET_X = 0.065f;  // 65mm forward
  static constexpr float SENSOR_OFFSET_Y = 0.047f;  // 47mm left  
  static constexpr float SENSOR_ANGLE = 36.0f * PI / 180.0f; // 36 degrees in radians
  
  // Vehicle state
  float x_m, y_m;    // Vehicle position (m)
  float phi;         // Vehicle heading (radians)
  
  // Sensor position in global frame
  float x_s, y_s;    // Sensor position (m)
  
  // Control parameters
  float y_ref;       // Reference y-coordinate (m)
  float K;           // Proportional gain
  float v_base;      // Base forward velocity (m/s)
  
  // Control output
  float heading_rate; // d(phi)/dt
  
  // Hardware interfaces
  MotorControl* motor;
  UltrasonicSensor* ultrasonic;
  
  // Conversion constants
  static constexpr float WHEEL_BASE = 0.15f;        // Distance between wheels (m)
  static constexpr float MAX_SPEED_MPS = 0.3f;      // Maximum speed (m/s)
  static constexpr int MAX_PWM = 255;               // Maximum PWM value
  
  // Helper functions
  float constrainSpeed(float speed);
  int speedToPWM(float speed);
};

#endif // WALL_FOLLOWING_CONTROLLER_H
