#include "dmp.h"

uint8_t fifoBuffer[45];         // буфер

void DMP_setup(MPU6050 mpu) {
  //Wire.begin();
  //Wire.setClock(1000000UL);   // разгоняем шину на максимум
  // инициализация DMP
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
}

void getDMP_angles(float &x, float &y, float &z, MPU6050 &mpu) {
  static uint32_t tmr;
  if (millis() - tmr >= 11) {  // таймер на 11 мс (на всякий случай)
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      // переменные для расчёта (ypr можно вынести в глобал)
      Quaternion q;
      VectorFloat gravity;
      float ypr[3];
      // расчёты
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      // выводим результат в радианах (-3.14, 3.14)
      z=degrees(ypr[0]); // вокруг оси Z
      y=degrees(ypr[1]); // вокруг оси Y
      x=degrees(ypr[2]); // вокруг оси X
      // для градусов можно использовать degrees()
      tmr = millis();  // сброс таймера
    }
  }
}