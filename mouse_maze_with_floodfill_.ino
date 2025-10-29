#include <Arduino.h>
#include "API.h"
#include "solver.h"
#include "queue.h"
#include "motor.h"
#include "Tracker.h"
#include "PID.h"
// #include "ultrasonic.h"
void setup()
{
Serial.begin(9600);
Init_Motors();
Init_Track();

}

void loop()
{
  // DebugTrackers();
  
  // Debug_Encoders('n');
  // leftPID();
  // DebugUltrasonic('n');
// Forward();
//   analogWrite(ENL,baseSpeedL);
//   analogWrite(ENR,baseSpeedR);

// Debug_Encoders('n');
// pid();
leftPID();
  
}
