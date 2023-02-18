#include "inklin_logic.h"

void getValues(
int16_t &accX,
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
Kalman &kalmanZ) {
  /* Calculate the angls based on the different sensors and algorithm */
  // accZangle = (atan2(accX, accY) + PI) * RAD_TO_DEG;
  // accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG;
  // accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;
  accXangle = (atan2(accX, sqrt(pow(accY,2)+pow(accZ,2)))) * RAD_TO_DEG;
  accYangle = (atan2(accY, sqrt(pow(accX,2)+pow(accZ,2)))) * RAD_TO_DEG;
  accZangle = (atan2(accZ, sqrt(pow(accX,2)+pow(accY,2)))) * RAD_TO_DEG;
  float gyroXrate = ((float)gyroX / 32768 * 250.0);//попробовать увеличить диапазон, как следстствие уменьшить ошибку шума
  float gyroYrate = ((float)gyroY / 32768 * 250.0);
  float gyroZrate = ((float)gyroZ / 32768 * 250.0);
  // Calculate gyro angle without any filter
  gyroXangle += gyroXrate * ((float)(micros() - timer) / 1000000);
  gyroYangle += gyroYrate * ((float)(micros() - timer) / 1000000);
  gyroZangle += gyroZrate * ((float)(micros() - timer) / 1000000);
  kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (float)(micros() - timer) / 1000000);
  kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (float)(micros() - timer) / 1000000);
  kalAngleZ = kalmanZ.getAngle(accZangle, gyroZrate, (float)(micros() - timer) / 1000000);
  
  timer = micros();
}

// ======= ФУНКЦИЯ КАЛИБРОВКИ =======
void Calibrate(MPU6050 mpu) {
  long offsets[6];
  long offsetsOld[6];
  int16_t mpuGet[6];

  // используем стандартную точность
  mpu.setFullScaleAccelRange(0);
  mpu.setFullScaleGyroRange(0);

  // обнуляем оффсеты
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  delay(10);
  Serial.println("Calibration start. It will take about 5 seconds");
  for (byte n = 0; n < 10; n++) {     // 10 итераций калибровки
    for (byte j = 0; j < 6; j++) {    // обнуляем калибровочный массив
      offsets[j] = 0;
    }
    for (byte i = 0; i < 100 + BUFFER_SIZE; i++) { // делаем BUFFER_SIZE измерений для усреднения
      mpu.getMotion6(&mpuGet[0], &mpuGet[1], &mpuGet[2], &mpuGet[3], &mpuGet[4], &mpuGet[5]);
      if (i >= 99) {                         // пропускаем первые 99 измерений
        for (byte j = 0; j < 6; j++) {
          offsets[j] += (long)mpuGet[j];   // записываем в калибровочный массив
        }
      }
    }
    for (byte i = 0; i < 6; i++) {
      offsets[i] = offsetsOld[i] - ((long)offsets[i] / BUFFER_SIZE); // учитываем предыдущую калибровку
      if (i == 2) offsets[i] += 16384;                               // если ось Z, калибруем в 16384
      offsetsOld[i] = offsets[i];
    }
    // ставим новые оффсеты
    mpu.setXAccelOffset(offsets[0] / 8);
    mpu.setYAccelOffset(offsets[1] / 8);
    mpu.setZAccelOffset(offsets[2] / 8);
    mpu.setXGyroOffset(offsets[3] / 4);
    mpu.setYGyroOffset(offsets[4] / 4);
    mpu.setZGyroOffset(offsets[5] / 4);
    delay(2);
  }

#ifdef SERVER
  server.send(200, "text/html", SendHTML_onCalibration(offsets[0] / 8, offsets[1] / 8,offsets[2]/8, offsets[3] / 4, offsets[4] / 4, offsets[5] / 4));
#endif
    // выводим в порт
    Serial.println("Calibration end. Your offsets:");
    Serial.println("accX accY accZ gyrX gyrY gyrZ");
    Serial.print(mpu.getXAccelOffset()); Serial.print(", ");
    Serial.print(mpu.getYAccelOffset()); Serial.print(", ");
    Serial.print(mpu.getZAccelOffset()); Serial.print(", ");
    Serial.print(mpu.getXGyroOffset()); Serial.print(", ");
    Serial.print(mpu.getYGyroOffset()); Serial.print(", ");
    Serial.print(mpu.getZGyroOffset()); Serial.println(" ");
    Serial.println(" ");

}