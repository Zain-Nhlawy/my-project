//0 mean there is wall
//1 mean no wall
#include "Tracker.h"
#include <Arduino.h>
void Init_Track()
{
  pinMode(LeftTrack,INPUT);
  pinMode(RightTrack,INPUT);
}

uint8_t ReadRightTrack()
{
  
  return analogRead(RightTrack)>500?0:1;
}

uint8_t ReadLeftTrack()
{
  return !digitalRead(LeftTrack);
}

void DebugTrackers()
{
  Serial.print(" Left Track ");
  Serial.print(ReadLeftTrack());

  Serial.print("     Right Track ");
  Serial.println(ReadRightTrack());

}