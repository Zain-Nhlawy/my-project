#pragma once 
#include "motor.h"

extern const long PULSES_PER_TURN ;
extern const float WHEEL_DIAMETER ;
extern const float WHEEL_DISTANCE ;

extern const float targetDistance ;

extern const float kp ;
extern const float kd ;
extern const float ki ;

extern float integral ;
extern float lastError ;

extern int baseSpeedL ;
extern int baseSpeedR ;
extern const int baseDelay;


void UpdatePID();
void movelitte();

void leftPIDforSeconds(int seconds);
void leftPID();
void moveDistance(float distanceCm);
void rightPID();
void turnRight();
void turnLeft();
void pid();

void pid_encoders();


