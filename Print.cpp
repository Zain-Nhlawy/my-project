#include <Arduino.h>
#include "Print.h"
unsigned long previousMillis = 0;  // will store last time LED was updated

// constants won't change:
const long interval = 1000;  // interval at which to blink (milliseconds)


void PrintWithDelay(unsigned long time,String msg)
{
   unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

Serial.println(msg) ;

  }
}