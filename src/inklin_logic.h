#include <Wire.h>
#include <Arduino.h>
#include <Kalman.h>
#include <MPU6050.h>


#define BUFFER_SIZE 100


void getValues(int16_t &accX,
int16_t &accY,
int16_t &accZ,
int16_t &tempRaw,
int16_t &gyroX,
int16_t &gyroY,
int16_t &gyroZ,
float &temp,
uint8_t &IMUAddress);

void calculateAngles(int16_t &accX,
int16_t &accY,
int16_t &accZ,
int16_t &tempRaw,
int16_t &gyroX,
int16_t &gyroY,
int16_t &gyroZ,
float &accXangle, // Angle calculate using the accelerometer
float &accYangle,
float &accZangle,
float &temp,
float &gyroXangle, // Angle calculate using the gyro
float &gyroYangle,
float &gyroZangle,
float &kalAngleX, // Calculate the angle using a Kalman filter
float &kalAngleY,
float &kalAngleZ,
uint32_t &timer,
Kalman &kalmanX,
Kalman &kalmanY,
Kalman &kalmanZ);

void Calibrate(MPU6050 mpu);


// calculateAngles(accX,accY,accZ, tempRaw,gyroX,gyroY,gyroZ,accXangle,accYangle,accZangle,temp,gyroXangle, 
// gyroYangle,gyroZangle,kalAngleX,kalAngleY,kalAngleZ,timer,kalmanX,kalmanY,kalmanZ);