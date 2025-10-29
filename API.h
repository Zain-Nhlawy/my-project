#pragma once
#include "ultrasonic.h"
// #include <NewPing.h>
#include <Arduino.h>
#define SquareRoot 6
uint8_t API_mazeWidth();
uint8_t API_mazeHeight();

uint8_t API_wallFront();
uint8_t API_wallRight();
uint8_t API_wallLeft();

void API_moveForward();  // Returns 0 if crash, else returns 1
void API_turnRight();
void API_turnLeft();

void API_setWall(int x, int y, char direction);
void API_clearWall(int x, int y, char direction);

void API_setColor(int x, int y, char color);
void API_clearColor(int x, int y);
void API_clearAllColor();

void API_setText(int x, int y, int val);
void API_clearText(int x, int y);
void API_clearAllText();

int API_wasReset();
void API_ackReset();

void debug_log(char* text);
void debug_int(int num);
void debug_coord(int x, int y);
