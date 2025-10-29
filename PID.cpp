#include "PID.h"
#include "ultrasonic.h"
#include "motor.h"
#include "Print.h"
#include <math.h>
const long PULSES_PER_TURN = 778;
const float WHEEL_DIAMETER = 4.02;
const float WHEEL_DISTANCE = 8.04;

const float targetDistance = 4;

const float kp = 0.38;
const float kd = 0.24;
const float ki = 0;

float integral = 0;
float lastError = 0;

int baseSpeedL = 60;
int baseSpeedR = 55;
const int baseDelay = 5;


void UpdatePID()
{
   long count1 = -encoder1.read();
  long count2 = -encoder2.read();

  float frontDistance = FrontUltra.ping_cm();
  float leftDistance =  LeftUltra.ping_cm();

  if (frontDistance == 0) frontDistance = MAX_DISTANCE;

  // int LeftSensorState = digitalRead(LeftTrack);
  // int RightSensorState = analogRead(RightTrack);
  // delay(100);
// leftPIDforSeconds(3);
// leftPID();
}



void movelitte() {
  Wait(0.5);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENL, baseSpeedL);
  analogWrite(ENR, baseSpeedR);
  delay(750);
}

void leftPIDforSeconds(int seconds) {
  unsigned long startTime = millis(); // وقت البداية
  unsigned long duration = seconds * 1000; // نحول الثواني إلى ميلي ثانية

  while (millis() - startTime < duration) {
    float leftDistance =FrontUltrasonicDistanceCM();
    if (leftDistance == 0) leftDistance = MAX_DISTANCE;

    float error = targetDistance - leftDistance;
    integral = 0.7 * integral + ki * error;
    float derivative = error - lastError;
    float control = kp * error + kd * derivative + integral;

    SpeedL = baseSpeedL + control;
    SpeedR = baseSpeedR - control;

    SpeedL = constrain(SpeedL, 0, 255);
    SpeedR = constrain(SpeedR, 0, 255);

    analogWrite(ENL, SpeedL);
    analogWrite(ENR, SpeedR);
    lastError = error;

    delay(baseDelay); 
  }

  Stop(); 
}


void leftPID() {
  float leftDistance = LeftUltrasonicDistanceCM();

  if (leftDistance == 0) leftDistance = MAX_DISTANCE;

  // float error = targetDistance - leftDistance;
  float error = targetDistance - leftDistance;
  integral = 0.7 * integral + ki * error;
  float derivative = error - lastError;
  float control = kp * error + kd * derivative + integral;
 control=1.6*ceil(control);
   SpeedL = baseSpeedL + abs(control);
   SpeedR = baseSpeedR - abs(control);

  SpeedL = constrain(SpeedL, 0, 255);
  SpeedR = constrain(SpeedR, 0, 255);
  Forward();
  analogWrite(ENL, SpeedL);
  analogWrite(ENR, SpeedR);

  PrintWithDelay(200,String(control));
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
    long pulses1 = abs(encoder1.read());
    long pulses2 = abs(encoder2.read());
    if (pulses1 >= targetPulses && pulses2 >= targetPulses) break;

    long error = pulses1*1.85 - pulses2;

    integral = 0.7 * integral + ki * error;
    float derivative = error - lastError;
    float control = kp * error + kd * derivative + integral;
    int adjA = 60 + control;
    int adjB = 40 - control;

    adjA = constrain(adjA, 0, 255);
    adjB = constrain(adjB, 0, 255);

    analogWrite(ENL, adjA);
    analogWrite(ENR, adjB);
    lastError = error;
  }
  Wait(0.5);
}


void rightPID() {

  float rightDistance = RightUltrasonicDistanceCM();

  if (rightDistance == 0) rightDistance = MAX_DISTANCE;

  float error = targetDistance - rightDistance;
  integral = 0.7 * integral + ki * error;
  float derivative = error - lastError;
  float control = kp * error + kd * derivative + integral;


  int speedA = baseSpeedL - control;
  int speedB = baseSpeedR + control;

  speedA = constrain(speedA, 0, 255);
  speedB = constrain(speedB, 0, 255);

  analogWrite(ENL, speedA);
  analogWrite(ENR, speedB);

  lastError = error;
}


void turnRight(float degree) {

  float wheelCircumference = PI * WHEEL_DIAMETER;
  float turnDistance = 2.0 * PI * (WHEEL_DISTANCE / 2.0) * (degree / 360.0);
  long targetPulses = (long)((PULSES_PER_TURN * turnDistance) / wheelCircumference + 0.5);

  encoder1.write(0);
  encoder2.write(0);

  Serial.print("Target pulses: ");
  Serial.println(targetPulses);

  while (abs(encoder1.read()) / 1.64 < targetPulses && abs(encoder2.read()) < targetPulses) {

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
      
    long error = encoder1.read() - encoder2.read();
    int speedA = 80 - kp * error; 
    int speedB = 60 + kp * error; 

    speedA = constrain(speedA, 0, 255);
    speedB = constrain(speedB, 0, 255);

    analogWrite(ENL, speedA);
    analogWrite(ENR, speedB);
  }

  Wait(1);
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

  while (abs(encoder1.read()) / 1.64 < targetPulses && abs(encoder2.read()) < targetPulses) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    long error2 = encoder1.read() - encoder2.read();
    int speedA = 80 + kp * error2;
    int speedB = 60 - kp * error2;

    speedA = constrain(speedA, 0, 255);
    speedB = constrain(speedB, 0, 255);

    analogWrite(ENL, speedA);
    analogWrite(ENR, speedB);
  }

  Wait(0.5);
  Serial.println("Turn complete!");
}




void pid ()
{
  Forward();
  int distance = LeftUltrasonicDistanceCM() ;
  int error = targetDistance - LeftUltrasonicDistanceCM();
  
  if(distance==0) distance=MAX_DISTANCE;
  if(targetDistance >distance)
  {
    analogWrite(ENL,baseSpeedL+error);
    analogWrite(ENR,baseSpeedR-error);
  }
  else
{   analogWrite(ENR,baseSpeedR+error);
    analogWrite(ENL,baseSpeedL-error);
}
PrintWithDelay(200,String(error));

  
}


