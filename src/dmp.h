//#include "I2Cdev.h"
//#include <MPU6050.h>
#include "MPU6050_6Axis_MotionApps20.h"

void DMP_setup(MPU6050 mpu);

void getDMP_angles(float &x, float &y, float &z, MPU6050 &mpu);