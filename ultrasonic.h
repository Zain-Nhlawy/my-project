#pragma once 
#include <Arduino.h>
#include <NewPing.h>

#define MAX_DISTANCE 50
#define FrontWallDistance 7

#define TRIG_LEFT A5
#define ECHO_LEFT A4

#define TRIG_FRONT A3
#define ECHO_FRONT A2

#define TRIG_RIGHT A0
#define ECHO_RIGHT A1


uint8_t FrontUltrasonicDistanceCM();
uint8_t RightUltrasonicDistanceCM();
uint8_t LeftUltrasonicDistanceCM();
extern NewPing LeftUltra;
extern NewPing FrontUltra;
extern NewPing RightUltra;

void DebugUltrasonic();
void DebugUltrasonic(char n );




