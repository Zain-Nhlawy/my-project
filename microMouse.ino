#include <NewPing.h>
#include <Encoder.h>

#define TRIG_LEFT   A5
#define ECHO_LEFT   A4

#define TRIG_FRONT  A3
#define ECHO_FRONT  A2

#define TRIG_RIGHT  A0
#define ECHO_RIGHT  A1

#define MAX_DISTANCE 50

NewPing sonarLeft(TRIG_LEFT, ECHO_LEFT, MAX_DISTANCE);
NewPing sonarFront(TRIG_FRONT, ECHO_FRONT, MAX_DISTANCE);
NewPing sonarRight(TRIG_RIGHT, ECHO_RIGHT, MAX_DISTANCE);

#define ENC1_A 13  
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


const long PULSES_PER_TURN = 420;    
const float WHEEL_DIAMETER = 4.0;
const float WHEEL_DISTANCE = 8.0;

const float targetDistance = 7;

const float kp = 2.2;
const float kd = 0.15;
const float ki = 0.001;

float integral = 0;
float lastError = 0;

int baseSpeed = 50;
const int baseDelay = 5;

void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Forward();
}

void loop() {
  long count1 = encoder1.read();
  long count2 = encoder2.read();

  float leftDistance = sonarLeft.ping_cm();
  float frontDistance = sonarFront.ping_cm();
  float rightDistance = sonarRight.ping_cm();

  if (leftDistance == 0) leftDistance = MAX_DISTANCE;
  if (frontDistance == 0) frontDistance = MAX_DISTANCE;
  if (rightDistance == 0) rightDistance = MAX_DISTANCE;

  // float error = targetDistance - leftDistance;
float error = targetDistance - leftDistance;
  integral = 0.7 * integral + ki * error;
  float derivative = error - lastError;
  float control = kp * error + kd * derivative + integral;


  int speedA = baseSpeed*1.435 + control;
  int speedB = baseSpeed - control;

  speedA = constrain(speedA, 0, 255);
  speedB = constrain(speedB, 0, 255);

  analogWrite(ENA, speedA);
  analogWrite(ENB, speedB);

  lastError = error;

  if (frontDistance < 10) {
    turnRightShort();
  }

  // if (rightDistance > 15) {
  //   turnLeft(90.0);
  // }

  Serial.print("Left: "); Serial.print(leftDistance);
  Serial.print("  A: ");  Serial.print(count1);
  Serial.print("  B: ");  Serial.println(count2);

  delay(baseDelay);
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

void turnRightShort() {
  Wait(0.5);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 150);
  analogWrite(ENB, 150);
  delay(400);
  Wait(0.3);
  Forward();
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

  while (abs(encoder1.read()) < targetPulses && abs(encoder2.read()) < targetPulses) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    long error2 = encoder1.read() - encoder2.read();
    int speedA = baseSpeed + kp * error2;
    int speedB = baseSpeed - kp * error2;

    speedA = constrain(speedA, 0, 255);
    speedB = constrain(speedB, 0, 255);

    analogWrite(ENA, speedA);
    analogWrite(ENB, speedB);
  }

  Wait(0.5);
  Stop();
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

  while (abs(encoder1.read()) < targetPulses && abs(encoder2.read()) < targetPulses) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    long error2 = encoder1.read() - encoder2.read();
    int speedA = baseSpeed + kp * error2;
    int speedB = baseSpeed - kp * error2;

    speedA = constrain(speedA, 0, 255);
    speedB = constrain(speedB, 0, 255);

    analogWrite(ENA, speedA);
    analogWrite(ENB, speedB);
  }

  Wait(0.5);
  Stop();
  Serial.println("Turn complete!");
}

//////
void Stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
