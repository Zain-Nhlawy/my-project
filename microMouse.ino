#include <NewPing.h>
#include <Encoder.h>

#define TRIG_LEFT A5
#define ECHO_LEFT A4

#define TRIG_FRONT A3
#define ECHO_FRONT A2

#define TRIG_RIGHT A0
#define ECHO_RIGHT A1

#define MAX_DISTANCE 50

NewPing sonarLeft(TRIG_LEFT, ECHO_LEFT, MAX_DISTANCE);
NewPing sonarFront(TRIG_FRONT, ECHO_FRONT, MAX_DISTANCE);
NewPing sonarRight(TRIG_RIGHT, ECHO_RIGHT, MAX_DISTANCE);

#define ENC1_A 12
#define ENC1_B 2
#define ENC2_A 3
#define ENC2_B 4

Encoder encoder1(ENC1_A, ENC1_B);
Encoder encoder2(ENC2_A, ENC2_B);

#define ENA 5
#define IN1 6
#define IN2 7

#define ENB 10
#define IN3 8
#define IN4 9
#define LeftTrack 11
#define RightTrack A6


const long PULSES_PER_TURN = 778;
const float WHEEL_DIAMETER = 4.02;
const float WHEEL_DISTANCE = 8.04;

const float targetDistance = 4.2;

const float kp = 0.24;
const float kd = 0.2;
const float ki = 0;

float integral = 0;
float lastError = 0;

int baseSpeedA = 50;
int baseSpeedB = 40;
const int baseDelay = 5;

void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);


  pinMode(LeftTrack, INPUT);
  pinMode(RightTrack, INPUT);

  Forward();
}

void loop() {
  long count1 = encoder1.read();
  long count2 = encoder2.read();

  float frontDistance = sonarFront.ping_cm();

  if (frontDistance == 0) frontDistance = MAX_DISTANCE;

  int LeftSensorState = digitalRead(LeftTrack);
  int RightSensorState = analogRead(RightTrack);

  if (LeftSensorState) {
    turnLeft(90.0);

  } else if (frontDistance >= 7) {
    leftPID();
  } else if (RightSensorState > 500) {
    turnRight(90.0);
  } else {
    turnLeft(180.0);
  }

  Serial.println(RightSensorState);

  delay(baseDelay);
}

void movelitte() {
  Wait(0.5);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, baseSpeedA);
  analogWrite(ENB, baseSpeedB);
  delay(750);
}

void leftPID() {
  float leftDistance = sonarLeft.ping_cm();

  if (leftDistance == 0) leftDistance = MAX_DISTANCE;

  // float error = targetDistance - leftDistance;
  float error = targetDistance - leftDistance;
  integral = 0.7 * integral + ki * error;
  float derivative = error - lastError;
  float control = kp * error + kd * derivative + integral;

  int speedA = baseSpeedA + control;
  int speedB = baseSpeedB - control;

  speedA = constrain(speedA, 0, 255);
  speedB = constrain(speedB, 0, 255);

  analogWrite(ENA, speedA);
  analogWrite(ENB, speedB);

  lastError = error;
}

void moveDistance(float distanceCm) {

  const float pi = 3.14159265;
  float wheelCircumference = pi * WHEEL_DIAMETER;
  long targetPulses = (long)((PULSES_PER_TURN * distanceCm) / wheelCircumference + 0.5);

  encoder1.write(0);
  encoder2.write(0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  while (true) {
    long pulses1 = abs(encoder1.read()) / 1.45;
    long pulses2 = abs(encoder2.read());

    if (pulses1 >= targetPulses && pulses2 >= targetPulses) break;

    long error = pulses1 - pulses2;

    integral = 0.7 * integral + ki * error;
    float derivative = error - lastError;
    float control = kp * error + kd * derivative + integral;
    int adjA = baseSpeedA + control;
    int adjB = baseSpeedB - control;

    adjA = constrain(adjA, 0, 255);
    adjB = constrain(adjB, 0, 255);

    analogWrite(ENA, adjA);
    analogWrite(ENB, adjB);
  }
  Wait(0.5);

}


void rightPID() {

  float rightDistance = sonarRight.ping_cm();

  if (rightDistance == 0) rightDistance = MAX_DISTANCE;

  // float error = targetDistance - leftDistance;
  float error = targetDistance - rightDistance;
  integral = 0.7 * integral + ki * error;
  float derivative = error - lastError;
  float control = kp * error + kd * derivative + integral;


  int speedA = baseSpeedA + control;
  int speedB = baseSpeedB - control;

  speedA = constrain(speedA, 0, 255);
  speedB = constrain(speedB, 0, 255);

  analogWrite(ENA, speedA);
  analogWrite(ENB, speedB);

  lastError = error;
}

void Forward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void Wait(float seconds) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  delay(seconds * 1000);
}
void turnRight(float degree) {
  Wait(0.5);
  const float pi = 3.14159265;
  float wheelCircumference = pi * WHEEL_DIAMETER;
  float turnDistance = 2.0 * pi * (WHEEL_DISTANCE / 2.0) * (degree / 360.0);
  long targetPulses = (long)((PULSES_PER_TURN * turnDistance) / wheelCircumference + 0.5);

  encoder1.write(0);
  encoder2.write(0);

  Serial.print("Target pulses: ");
  Serial.println(targetPulses);

  while (abs(encoder1.read()) / 1.6 < targetPulses && abs(encoder2.read()) < targetPulses) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);


    long error2 = encoder1.read() - encoder2.read();
    int speedA = 75 + kp * error2;
    int speedB = 55 - kp * error2;

    speedA = constrain(speedA, 0, 255);
    speedB = constrain(speedB, 0, 255);

    analogWrite(ENA, speedA);
    analogWrite(ENB, speedB);
  }

  Wait(0.5);
 moveDistance(18);
  Serial.println("Turn complete!");
}


void turnLeft(float degree) {
  Wait(0.5);
  const float pi = 3.14159265;
  float wheelCircumference = pi * WHEEL_DIAMETER;
  float turnDistance = 2.0 * pi * (WHEEL_DISTANCE / 2.0) * (degree / 360.0);
  long targetPulses = (long)((PULSES_PER_TURN * turnDistance) / wheelCircumference + 0.5);


  encoder1.write(0);
  encoder2.write(0);


  Serial.print("Target pulses: ");
  Serial.println(targetPulses);

  while (abs(encoder1.read()) / 1.45 < targetPulses && abs(encoder2.read()) < targetPulses) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    long error2 = encoder1.read() - encoder2.read();
    int speedA = 75 + kp * error2;
    int speedB = 55 - kp * error2;

    speedA = constrain(speedA, 0, 255);
    speedB = constrain(speedB, 0, 255);

    analogWrite(ENA, speedA);
    analogWrite(ENB, speedB);
  }

  Wait(0.5);
  moveDistance(18);
  Serial.println("Turn complete!");
}

//////
void Stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}