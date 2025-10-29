#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "API.h"
#include "Tracker.h"
#include <Arduino.h>
#define BUFFER_SIZE 32

// int getInteger(char* command) {
//     printf("%s\n", command);
//     fflush(stdout);
//     char response[BUFFER_SIZE];
//     fgets(response, BUFFER_SIZE, stdin);
//     int value = atoi(response);
//     return value;
// }

// int getBoolean(char* command) {
//     printf("%s\n", command);
//     fflush(stdout);
//     char response[BUFFER_SIZE];
//     fgets(response, BUFFER_SIZE, stdin);
//     int value = (strcmp(response, "true\n") == 0);
//     return value;
// }

int getAck(char* command) {
    printf("%s\n", command);
    fflush(stdout);
    char response[BUFFER_SIZE];
    fgets(response, BUFFER_SIZE, stdin);
    int success = (strcmp(response, "ack\n") == 0);
    return success;
}

uint8_t API_mazeWidth() {
    return SquareRoot;
}

uint8_t API_mazeHeight() {
    return SquareRoot;
}

uint8_t API_wallFront() {
    return (FrontUltrasonicDistanceCM()<=FrontWallDistance);
}

uint8_t API_wallRight() {
    return ReadRightTrack();
}

uint8_t API_wallLeft() {
    return ReadLeftTrack();
}
void API_moveForward() {
  
}

void API_turnRight() {
    getAck("turnRight");
}

void API_turnLeft() {
    getAck("turnLeft");
}

void API_setWall(int x, int y, char direction) {
    // printf("setWall %d %d %c\n", x, y, direction);
    // fflush(stdout);
    Serial.print("There's a wall to the : ");
    Serial.println(direction);
    
}

void API_clearWall(int x, int y, char direction) {
    // printf("clearWall %d %d %c\n", x, y, direction);
    // fflush(stdout);
}

void API_setColor(int x, int y, char color) {
    // printf("setColor %d %d %c\n", x, y, color);
    // fflush(stdout);
}

void API_clearColor(int x, int y) {
    // printf("clearColor %d %d\n", x, y);
    // fflush(stdout);
}

void API_clearAllColor() {
    // printf("clearAllColor\n");
    // fflush(stdout);
}

void API_setText(int x, int y, int val) {
    // printf("setText %d %d %d\n", x, y, val);
    // fflush(stdout);
}

void API_clearText(int x, int y) {
    // printf("clearText %d %d\n", x, y);
    // fflush(stdout);
}

void API_clearAllText() {
    // printf("clearAllText\n");
    // fflush(stdout);
}

int API_wasReset() {
    // return getBoolean("wasReset");
}

void API_ackReset() {
    getAck("ackReset");
}

void debug_log(char* text) {
    // fprintf(stderr, "%s\n", text);
    // fflush(stderr);
}

void debug_int(int num) {
    // fprintf(stderr, "%d\n", num);
    // fflush(stderr);
}

void debug_coord(int x, int y) {
    // fprintf(stderr, "(%d, %d)\n", x, y);
    // fflush(stderr);
}