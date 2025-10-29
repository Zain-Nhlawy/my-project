#include "ultrasonic.h"




 NewPing LeftUltra(TRIG_LEFT, ECHO_LEFT, MAX_DISTANCE);
 NewPing FrontUltra(TRIG_FRONT, ECHO_FRONT, MAX_DISTANCE);
 NewPing RightUltra(TRIG_RIGHT, ECHO_RIGHT, MAX_DISTANCE);


uint8_t FrontUltrasonicDistanceCM()
{
  return FrontUltra.ping_cm();
}


 uint8_t RightUltrasonicDistanceCM()
 {
  return RightUltra.ping_cm();
 }

 uint8_t LeftUltrasonicDistanceCM()
{
  return LeftUltra.ping_cm();
}

void DebugUltrasonic()
{
  Serial.print("Left Ultra : ");
  Serial.print(LeftUltrasonicDistanceCM());
  Serial.print("        Front Ultra : ");
  Serial.print(FrontUltrasonicDistanceCM());
  Serial.print("        Right Ultra : ");
  Serial.print(RightUltrasonicDistanceCM());
}

void DebugUltrasonic(char n)
{
  DebugUltrasonic();
  Serial.println();
}
