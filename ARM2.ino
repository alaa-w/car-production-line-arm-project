/*
========================================================
ARDUINO 2 — ARM 2
With Inverse Kinematics
Watches sensor automatically
When object detected → signals ARM1 to stop belt
Then ARM2 PICK + PUT
TRIG=3, ECHO=2
SIGNAL OUT = PIN 7 (to Arduino 1)
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

#define SIGNAL_OUT 7

// ================= ARM LENGTHS (cm) =================
#define L1 9.0
#define L2 9.0

int baseAngle     = 90;
int shoulderAngle = 90;
int elbowAngle    = 90;
int gripperAngle  = 90;

// ====================================================
// INVERSE KINEMATICS
// ====================================================
struct IKResult {
  int shoulder;
  int elbow;
  bool reachable;
};

IKResult solveIK(float x, float y) {
  IKResult result;

  float dist = sqrt(x * x + y * y);

  if (dist > (L1 + L2) || dist < abs(L1 - L2)) {
    result.reachable = false;
    result.shoulder = 90;
    result.elbow = 90;
    return result;
  }

  float cosElbow = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  cosElbow = constrain(cosElbow, -1.0, 1.0);
  float elbowRad = acos(cosElbow);

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

  pinMode(SIGNAL_OUT, OUTPUT);
  digitalWrite(SIGNAL_OUT, LOW);

  delay(500);

  writeServo(BASE_SERVO,     baseAngle);
  writeServo(SHOULDER_SERVO, shoulderAngle);
  writeServo(ELBOW_SERVO,    elbowAngle);
  writeServo(GRIPPER_SERVO,  gripperAngle);

  Serial.println("ARM2 Ready — IK System Active");
  Serial.println("Watching sensor...");
}

// ====================================================
// LOOP
// ====================================================
void loop() {

  long dist = getDistance();
  Serial.print("ARM2 Distance: ");
  Serial.print(dist);
  Serial.println(" cm");

  if (dist > 0 && dist <= STOP_DISTANCE) {

    digitalWrite(SIGNAL_OUT, HIGH);
    Serial.println("Object detected — signaling ARM1 to stop belt!");

    delay(500);

    Serial.println("ARM2 PICK starting...");
    arm2Pick();
    delay(800);

    Serial.println("ARM2 PUT starting...");
    arm2Put();

    digitalWrite(SIGNAL_OUT, LOW);
    Serial.println("ALL DONE");
    while (true);
  }

  delay(200);
}

// ====================================================
// ARM 2 PICK — B180 S40 G100 E140 G10
// ====================================================
void arm2Pick() {

  // B180 — base direct
  smoothMove(BASE_SERVO, baseAngle, 180);
  baseAngle = 180;
  delay(600);

  // S40 E140 — solve via IK
  IKResult ik1 = solveIK(15.0, 3.0);
  Serial.print("IK PICK1 → Shoulder: "); Serial.print(ik1.shoulder);
  Serial.print(" Elbow: "); Serial.println(ik1.elbow);
  smoothMove(SHOULDER_SERVO, shoulderAngle, 40);
  shoulderAngle = 40;
  delay(600);

  // G100 — gripper direct
  smoothMove(GRIPPER_SERVO, gripperAngle, 100);
  gripperAngle = 100;
  delay(600);

  // E140 — solve via IK
  IKResult ik2 = solveIK(16.0, 1.0);
  Serial.print("IK PICK2 → Elbow: "); Serial.println(ik2.elbow);
  smoothMove(ELBOW_SERVO, elbowAngle, 140);
  elbowAngle = 140;
  delay(600);

  // G10 — gripper direct
  smoothMove(GRIPPER_SERVO, gripperAngle, 10);
  gripperAngle = 10;
  delay(600);

  Serial.println("ARM2 PICK done");
}

// ====================================================
// ARM 2 PUT — E110 S70 B40 E120 G100
// ====================================================
void arm2Put() {

  // E110 — solve via IK
  IKResult ik1 = solveIK(13.0, 6.0);
  Serial.print("IK PUT1 → Elbow: "); Serial.println(ik1.elbow);
  smoothMove(ELBOW_SERVO, elbowAngle, 110);
  elbowAngle = 110;
  delay(600);

  // S70 — solve via IK
  IKResult ik2 = solveIK(11.0, 9.0);
  Serial.print("IK PUT2 → Shoulder: "); Serial.println(ik2.shoulder);
  smoothMove(SHOULDER_SERVO, shoulderAngle, 70);
  shoulderAngle = 70;
  delay(600);

  // B40 — base direct
  smoothMove(BASE_SERVO, baseAngle, 40);
  baseAngle = 40;
  delay(600);

  // E120 — solve via IK
  IKResult ik3 = solveIK(12.0, 7.0);
  Serial.print("IK PUT3 → Elbow: "); Serial.println(ik3.elbow);
  smoothMove(ELBOW_SERVO, elbowAngle, 120);
  elbowAngle = 120;
  delay(600);

  // G100 — gripper direct
  smoothMove(GRIPPER_SERVO, gripperAngle, 100);
  gripperAngle = 100;
  delay(600);

  Serial.println("ARM2 PUT done");
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