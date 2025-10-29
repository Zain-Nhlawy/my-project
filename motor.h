#pragma once 
#include <Encoder.h>
#define ENC1_A 12
#define ENC1_B 2
#define ENC2_A 3
#define ENC2_B 4

#define ENL 5
#define IN1 6
#define IN2 7

#define ENR 10
#define IN3 8
#define IN4 9

extern Encoder encoder1;
extern Encoder encoder2;
extern uint8_t SpeedR;
extern uint8_t SpeedL;


void Forward();
void Wait (float Seconds);
void Stop();
void goForward();
void Init_Motors();

void Wait(float seconds) ;

void Debug_Encoders();
void Debug_Encoders(char n );

