#include "inklin_logic.h"

void getValues(int16_t &accX,
int16_t &accY,
int16_t &accZ,
int16_t &tempRaw,
int16_t &gyroX,
int16_t &gyroY,
int16_t &gyroZ,
float &temp,
uint8_t &IMUAddress) {
  /* Update all the values */
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(IMUAddress, 14, true); // request a total of 14 registers
  accX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  accY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  accZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  tempRaw = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gyroX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyroY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyroZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  temp = ((float)tempRaw + 12412.0) / 340.0;
}

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
// float &temp) {
//   /* Calculate the angls based on the different sensors and algorithm */
//   accZangle = (atan2(accX, accY) + PI) * RAD_TO_DEG;
//   accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG;
//   accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;
//   float gyroXrate = ((float)gyroX / 32768 * 250.0);//попробовать увеличить диапазон, как следстствие уменьшить ошибку шума
//   float gyroYrate = ((float)gyroY / 32768 * 250.0);
//   float gyroZrate = ((float)gyroZ / 32768 * 250.0);
//   // Calculate gyro angle without any filter
//   gyroXangle += gyroXrate * ((float)(micros() - timer) / 1000000);
//   gyroYangle += gyroYrate * ((float)(micros() - timer) / 1000000);
//   gyroZangle += gyroZrate * ((float)(micros() - timer) / 1000000);
//   kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (float)(micros() - timer) / 1000000);
//   kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (float)(micros() - timer) / 1000000);
//   kalAngleZ = kalmanZ.getAngle(accZangle, gyroZrate, (float)(micros() - timer) / 1000000);
  
//   timer = micros();
// }
