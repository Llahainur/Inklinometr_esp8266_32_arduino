#include <Wire.h>
#include <Arduino.h>
#include <Kalman.h>

void getValues(int16_t &accX,
int16_t &accY,
int16_t &accZ,
int16_t &tempRaw,
int16_t &gyroX,
int16_t &gyroY,
int16_t &gyroZ,
float &temp,
uint8_t &IMUAddress);

// void calculateAngles(int16_t &accX,
// int16_t &accY,
// int16_t &accZ,
// int16_t &tempRaw,
// int16_t &gyroX,
// int16_t &gyroY,
// int16_t &gyroZ,
// float &accXangle, // Angle calculate using the accelerometer
// float &accYangle,
// float &accZangle,
// float &temp);