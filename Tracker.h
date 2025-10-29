#pragma once 
#include <Arduino.h>

#define LeftTrack 11
#define RightTrack A6

void Init_Track();

uint8_t ReadRightTrack();
uint8_t ReadLeftTrack();

void DebugTrackers();



