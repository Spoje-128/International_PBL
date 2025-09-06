#include "ServoController.h"

ServoController::ServoController() {
  currentAngle = 90;  // 初期角度90度（中央）
  
  // ノンブロッキングスイープの初期化
  sweepActive = false;
  sweepStep = 0;
  sweepLastTime = 0;
  sweepStepDuration = 600; // 各ステップ600ms
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

// ノンブロッキングスイープ開始
void ServoController::startNonBlockingSweep() {
  sweepActive = true;
  sweepStep = 0;
  sweepLastTime = millis();
  Serial.println("Non-blocking sweep started");
}

// ノンブロッキングスイープ更新
void ServoController::updateNonBlockingSweep() {
  if (!sweepActive) return;
  
  unsigned long currentTime = millis();
  
  // 前のステップから十分な時間が経過していない場合は何もしない
  if (currentTime - sweepLastTime < sweepStepDuration) {
    return;
  }
  
  // スイープステップを実行
  switch (sweepStep) {
    case 0: // 0度に移動（振り下ろし）
      setAngle(0);
      sweepStepDuration = 400;
      Serial.println("Sweep: Strike down (0°)");
      break;
      
    case 1: // 180度に移動（振り上げ）
      setAngle(180);
      sweepStepDuration = 400;
      Serial.println("Sweep: Strike up (180°)");
      break;
      
    case 2: // 0度に移動（振り下ろし）
      setAngle(0);
      sweepStepDuration = 400;
      Serial.println("Sweep: Strike down (0°)");
      break;
      
    case 3: // 180度に移動（振り上げ）
      setAngle(180);
      sweepStepDuration = 400;
      Serial.println("Sweep: Strike up (180°)");
      break;
      
    case 4: // 中央に戻る
      setAngle(90);
      sweepStepDuration = 400;
      Serial.println("Sweep: Return center (90°)");
      break;
      
    default:
      // スイープ完了、最初に戻る
      sweepStep = -1; // 次のステップで0になる
      sweepStepDuration = 200; // 少し長めの間隔
      break;
  }
  
  sweepStep++;
  sweepLastTime = currentTime;
}

// ノンブロッキングスイープ停止
void ServoController::stopNonBlockingSweep() {
  sweepActive = false;
  setAngle(90); // 中央に戻す
  Serial.println("Non-blocking sweep stopped");
}

// スイープ中かどうか
bool ServoController::isSweeping() {
  return sweepActive;
}
