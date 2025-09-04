#include "ServoController.h"

ServoController::ServoController() {
  currentAngle = 90;  // 初期角度90度（中央）
}

void ServoController::init() {
  servo.attach(SERVO_PIN);  // D11ピンにサーボを接続
  delay(100);               // サーボの初期化待ち
  setAngle(90);             // 初期位置（中央）を正しく設定
  delay(500);               // 初期位置への移動完了を待つ
  
  Serial.println("Servo Controller initialized");
  Serial.print("Servo Pin: D");
  Serial.println(SERVO_PIN);
  Serial.println("Initial position: 90 degrees");
}

void ServoController::setAngle(int angle) {
  // 角度を0-180度に制限
  angle = constrain(angle, 0, 180);
  
  servo.write(angle);
  currentAngle = angle;
  
  Serial.print("Servo angle: ");
  Serial.print(angle);
  Serial.println(" degrees");
}

void ServoController::sweep() {
  Serial.println("Starting servo sweep...");
  
  // 0度に移動
  servo.write(0);
  currentAngle = 0;
  Serial.println("Servo -> 0 degrees");
  delay(800);  // もう少し長く待つ
  
  // 180度に移動
  servo.write(180);
  currentAngle = 180;
  Serial.println("Servo -> 180 degrees");
  delay(800);
  
  // 60度に移動
  servo.write(0);
  currentAngle = 60;
  Serial.println("Servo -> 60 degrees");
  delay(600);
  
  // 120度に移動
  servo.write(180);
  currentAngle = 120;
  Serial.println("Servo -> 120 degrees");
  delay(600);
  
  // 中央に戻る
  servo.write(90);
  currentAngle = 90;
  Serial.println("Servo -> 90 degrees (center)");
  delay(500);
  
  Serial.println("Sweep completed.");
}

int ServoController::getCurrentAngle() {
  return currentAngle;
}
