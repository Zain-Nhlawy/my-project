#include "motor.h"


Encoder encoder1(ENC1_A, ENC1_B);
Encoder encoder2(ENC2_A, ENC2_B);

uint8_t SpeedR=0;
uint8_t SpeedL=0;

void Init_Motors()
{
  pinMode(ENL, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENR, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

}




void Forward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}



void Stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}


void goForward()
{
  Forward();
}




void Wait(float seconds) {
  Stop();
  delay(seconds * 1000);
  Forward();
}




void Debug_Encoders()
{
  Serial.print("Left Encoder : ");
  Serial.print(encoder2.read());
  Serial.print("      Right Encoder : ");
  
  Serial.print(encoder1.read());
  Serial.print("        difference : ");
  Serial.print(encoder1.read()-encoder2.read());
}



void Debug_Encoders(char n )
{
  Debug_Encoders();
  Serial.println();
  
}
