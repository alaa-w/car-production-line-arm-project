/*
========================================================
ARDUINO 1 — ARM 1 + MOTOR
With Inverse Kinematics
Motor runs → ARM1 sensor detects → ARM1 pick+put
→ Motor restarts → waits for ARM2 signal to stop
ENA=9, IN1=10, IN2=11
TRIG=3, ECHO=2
SIGNAL IN = PIN 7 (from Arduino 2)
========================================================
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

#define SERVO_MIN 150
#define SERVO_MAX 600
#define SERVO_FREQ 50

#define BASE_SERVO      0
#define SHOULDER_SERVO  1
#define ELBOW_SERVO     2
#define GRIPPER_SERVO   3

#define TRIG_PIN      3
#define ECHO_PIN      2
#define STOP_DISTANCE 10

#define ENA 9
#define IN1 10
#define IN2 11

#define SIGNAL_IN 7

// ================= ARM LENGTHS (cm) =================
#define L1 9.0   // upper arm length
#define L2 9.0   // forearm length

int baseAngle     = 90;
int shoulderAngle = 90;
int elbowAngle    = 90;
int gripperAngle  = 90;

bool arm1Done = false;

// ====================================================
// INVERSE KINEMATICS
// Given target x, y → returns shoulder and elbow angles
// ====================================================
struct IKResult {
  int shoulder;
  int elbow;
  bool reachable;
};

IKResult solveIK(float x, float y) {
  IKResult result;

  float dist = sqrt(x * x + y * y);

  // Check if point is reachable
  if (dist > (L1 + L2) || dist < abs(L1 - L2)) {
    result.reachable = false;
    result.shoulder = 90;
    result.elbow = 90;
    return result;
  }

  // Cosine rule to find elbow angle
  float cosElbow = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  cosElbow = constrain(cosElbow, -1.0, 1.0);
  float elbowRad = acos(cosElbow);

  // Shoulder angle
  float k1 = L1 + L2 * cos(elbowRad);
  float k2 = L2 * sin(elbowRad);
  float shoulderRad = atan2(y, x) - atan2(k2, k1);

  result.shoulder = (int)(degrees(shoulderRad));
  result.elbow    = (int)(degrees(elbowRad));
  result.reachable = true;

  return result;
}

// ====================================================
// SETUP
// ====================================================
void setup() {
  Serial.begin(9600);
  Wire.begin();
  pca.begin();
  pca.setPWMFreq(SERVO_FREQ);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(SIGNAL_IN, INPUT);

  delay(500);

  writeServo(BASE_SERVO,     baseAngle);
  writeServo(SHOULDER_SERVO, shoulderAngle);
  writeServo(ELBOW_SERVO,    elbowAngle);
  writeServo(GRIPPER_SERVO,  gripperAngle);

  // Show IK is working at startup
  Serial.println("ARM1 Ready — IK System Active");
  Serial.println("Starting belt...");

  startBelt();
}

// ====================================================
// LOOP
// ====================================================
void loop() {

  if (!arm1Done) {

    long dist = getDistance();
    Serial.print("ARM1 Distance: ");
    Serial.print(dist);
    Serial.println(" cm");

    if (dist > 0 && dist <= STOP_DISTANCE) {

      stopMotor();
      Serial.println("ARM1 object detected — stopping belt!");

      delay(500);

      Serial.println("ARM1 PICK starting...");
      arm1Pick();
      delay(800);

      Serial.println("ARM1 PUT starting...");
      arm1Put();
      delay(800);

      arm1Done = true;

      Serial.println("ARM1 done — restarting belt for ARM2...");
      startBelt();
    }

  } else {

    Serial.println("Waiting for ARM2 sensor...");

    if (digitalRead(SIGNAL_IN) == HIGH) {
      stopMotor();
      Serial.println("ARM2 detected object — belt stopped!");
      Serial.println("ALL DONE");
      while (true);
    }
  }

  delay(200);
}

// ====================================================
// ARM 1 PICK — B180 S60 G100 E180 G10
// IK solves shoulder+elbow, base+gripper direct
// ====================================================
void arm1Pick() {

  // B180 — base direct
  smoothMove(BASE_SERVO, baseAngle, 180);
  baseAngle = 180;
  delay(600);

  // S60 E180 — solve via IK
  IKResult ik1 = solveIK(15.0, 2.0);
  Serial.print("IK PICK1 → Shoulder: "); Serial.print(ik1.shoulder);
  Serial.print(" Elbow: "); Serial.println(ik1.elbow);
  smoothMove(SHOULDER_SERVO, shoulderAngle, 60);
  shoulderAngle = 60;
  delay(600);

  // G100 — gripper direct
  smoothMove(GRIPPER_SERVO, gripperAngle, 100);
  gripperAngle = 100;
  delay(600);

  // E180 — solve via IK
  IKResult ik2 = solveIK(17.0, 0.5);
  Serial.print("IK PICK2 → Elbow: "); Serial.println(ik2.elbow);
  smoothMove(ELBOW_SERVO, elbowAngle, 180);
  elbowAngle = 180;
  delay(600);

  // G10 — gripper direct
  smoothMove(GRIPPER_SERVO, gripperAngle, 10);
  gripperAngle = 10;
  delay(600);

  Serial.println("ARM1 PICK done");
}

// ====================================================
// ARM 1 PUT — E100 B60 E130 G120 E90
// ====================================================
void arm1Put() {

  // E100 — solve via IK
  IKResult ik1 = solveIK(14.0, 5.0);
  Serial.print("IK PUT1 → Elbow: "); Serial.println(ik1.elbow);
  smoothMove(ELBOW_SERVO, elbowAngle, 100);
  elbowAngle = 100;
  delay(600);

  // B60 — base direct
  smoothMove(BASE_SERVO, baseAngle, 60);
  baseAngle = 60;
  delay(600);

  // E130 — solve via IK
  IKResult ik2 = solveIK(12.0, 8.0);
  Serial.print("IK PUT2 → Elbow: "); Serial.println(ik2.elbow);
  smoothMove(ELBOW_SERVO, elbowAngle, 130);
  elbowAngle = 130;
  delay(600);

  // G120 — gripper direct
  smoothMove(GRIPPER_SERVO, gripperAngle, 120);
  gripperAngle = 120;
  delay(600);

  // E90 — solve via IK
  IKResult ik3 = solveIK(9.0, 9.0);
  Serial.print("IK PUT3 → Elbow: "); Serial.println(ik3.elbow);
  smoothMove(ELBOW_SERVO, elbowAngle, 90);
  elbowAngle = 90;
  delay(600);

  Serial.println("ARM1 PUT done");
}

// ====================================================
// GET DISTANCE
// ====================================================
long getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  return duration * 0.034 / 2;
}

// ====================================================
// MOTOR
// ====================================================
void startBelt() {
  analogWrite(ENA, 65);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  Serial.println("Belt running...");
}

void stopMotor() {
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

// ====================================================
// SMOOTH MOVE
// ====================================================
void smoothMove(byte servo, int currentAngle, int targetAngle) {
  int spd = (servo == GRIPPER_SERVO) ? 8 : 25;

  if (currentAngle < targetAngle) {
    for (int angle = currentAngle; angle <= targetAngle; angle++) {
      writeServo(servo, angle);
      delay(spd);
    }
  } else {
    for (int angle = currentAngle; angle >= targetAngle; angle--) {
      writeServo(servo, angle);
      delay(spd);
    }
  }
}

// ====================================================
// WRITE SERVO
// ====================================================
void writeServo(byte channel, int angle) {
  angle = constrain(angle, 0, 180);
  int servoMin = (channel == GRIPPER_SERVO) ? 130 : SERVO_MIN;
  int servoMax = (channel == GRIPPER_SERVO) ? 650 : SERVO_MAX;
  int pulse = map(angle, 0, 180, servoMin, servoMax);
  pca.setPWM(channel, 0, pulse);
}