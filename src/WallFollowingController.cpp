#include "WallFollowingController.h"

WallFollowingController::WallFollowingController() :
  x_m(0.0f), y_m(0.0f), phi(0.0f),
  x_s(0.0f), y_s(0.0f),
  y_ref(0.5f), K(2.0f), v_base(0.1f),
  heading_rate(0.0f),
  motor(nullptr), ultrasonic(nullptr) {
}

void WallFollowingController::init(MotorControl* motorController, UltrasonicSensor* ultrasonicSensor) {
  motor = motorController;
  ultrasonic = ultrasonicSensor;
  
  Serial.println("Wall Following Controller initialized");
  Serial.print("Sensor offset: X=");
  Serial.print(SENSOR_OFFSET_X * 1000);
  Serial.print("mm, Y=");
  Serial.print(SENSOR_OFFSET_Y * 1000);
  Serial.println("mm");
  Serial.print("Sensor angle: ");
  Serial.print(SENSOR_ANGLE * 180.0f / PI);
  Serial.println(" degrees");
}

void WallFollowingController::setParameters(float y_ref_val, float K_gain, float v_base_val) {
  y_ref = y_ref_val;
  K = K_gain;
  v_base = v_base_val;
  
  Serial.println("Wall following parameters updated:");
  Serial.print("y_ref = ");
  Serial.print(y_ref);
  Serial.println("m");
  Serial.print("K = ");
  Serial.println(K);
  Serial.print("v_base = ");
  Serial.print(v_base);
  Serial.println("m/s");
}

void WallFollowingController::setVehiclePose(float x, float y, float phi_rad) {
  x_m = x;
  y_m = y;
  phi = phi_rad;
}

void WallFollowingController::updateSensorPosition() {
  // Transform sensor local position to global coordinates
  // Local sensor position relative to vehicle center
  float sensor_local_x = SENSOR_OFFSET_X;
  float sensor_local_y = SENSOR_OFFSET_Y;
  
  // Rotation matrix transformation
  float cos_phi = cos(phi);
  float sin_phi = sin(phi);
  
  // Global sensor position
  x_s = x_m + cos_phi * sensor_local_x - sin_phi * sensor_local_y;
  y_s = y_m + sin_phi * sensor_local_x + cos_phi * sensor_local_y;
  
  // Get sensor measurement and update y_s based on wall detection
  float sensorDistance = ultrasonic->getLeftDistance() / 1000.0f; // Convert mm to m
  
  // If we have a valid sensor reading, use it to estimate the sensor's y-coordinate
  // The sensor points at an angle, so we need to account for that
  if (sensorDistance < 4.0f) { // Valid reading (less than 4m)
    float sensor_global_angle = phi + SENSOR_ANGLE;
    
    // The sensor detects the wall at y = d
    // The sensor reading gives us the perpendicular distance to the wall
    float wall_y = y_s + sensorDistance * sin(sensor_global_angle);
    
    // Update sensor y-coordinate based on wall detection
    y_s = wall_y - sensorDistance * sin(sensor_global_angle);
  }
}

float WallFollowingController::computeControl() {
  // Update sensor position first
  updateSensorPosition();
  
  // Control law: d(phi)/dt = K * (y_ref - y_s)
  heading_rate = K * (y_ref - y_s);
  
  // Limit heading rate to reasonable values
  heading_rate = constrain(heading_rate, -2.0f, 2.0f); // rad/s
  
  return heading_rate;
}

void WallFollowingController::setMotorSpeeds(float v_left, float v_right) {
  // Constrain speeds to safe limits
  v_left = constrainSpeed(v_left);
  v_right = constrainSpeed(v_right);
  
  // Convert speeds to PWM values
  int pwm_left = speedToPWM(abs(v_left));
  int pwm_right = speedToPWM(abs(v_right));
  
  // Set motor directions and speeds
  if (v_left >= 0 && v_right >= 0) {
    // Forward motion
    motor->move(max(pwm_left, pwm_right), pwm_right - pwm_left);
  } else if (v_left < 0 && v_right < 0) {
    // Backward motion
    motor->moveBackward(max(pwm_left, pwm_right));
  } else {
    // Turning in place
    if (v_left > v_right) {
      motor->turnRight(max(pwm_left, pwm_right));
    } else {
      motor->turnLeft(max(pwm_left, pwm_right));
    }
  }
}

void WallFollowingController::update() {
  // Compute control signal
  float heading_rate = computeControl();
  
  // Convert heading rate to differential wheel velocities
  // v_left  = v_base - control_factor * heading_rate
  // v_right = v_base + control_factor * heading_rate
  float control_factor = WHEEL_BASE / 2.0f; // Convert angular to linear velocity
  
  float v_left = v_base - control_factor * heading_rate;
  float v_right = v_base + control_factor * heading_rate;
  
  // Set motor speeds
  setMotorSpeeds(v_left, v_right);
  
  // Debug output
  Serial.print("WallFollow - Sensor pos: (");
  Serial.print(x_s, 3);
  Serial.print(", ");
  Serial.print(y_s, 3);
  Serial.print("), y_error: ");
  Serial.print(y_ref - y_s, 3);
  Serial.print(", heading_rate: ");
  Serial.print(heading_rate, 3);
  Serial.print(", speeds: L=");
  Serial.print(v_left, 3);
  Serial.print(", R=");
  Serial.print(v_right, 3);
  Serial.println();
}

float WallFollowingController::constrainSpeed(float speed) {
  return constrain(speed, -MAX_SPEED_MPS, MAX_SPEED_MPS);
}

int WallFollowingController::speedToPWM(float speed) {
  // Linear mapping from speed (m/s) to PWM (0-255)
  float pwm_float = (speed / MAX_SPEED_MPS) * MAX_PWM;
  return constrain((int)pwm_float, 0, MAX_PWM);
}
