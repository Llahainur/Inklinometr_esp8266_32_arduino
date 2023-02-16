#define SERVER //Добавлять ли веб-сервер
//or
//#define MODBUS //Используем Модбас
//or
//#define CLIENT //Отправляем данные на веб сервер https://radioprog.ru/post/1119
//доделать веб клиент



#ifdef SERVER
//#define EXPERIMENTAL //длинный html с автообновлением если включено
#define WIFI_ACP //если датчик - центральный, делаем его точкой доступа
//or
//#define WIFI_CLI //если датчик - клиент другой сети
#endif

const int count=1000;// количество измерений, от которых берется среднее

#ifdef WIFI_ACP 
#define IP 1 //IP адрес датчика. Должен быть уникален. 192.168.1.IP
#endif

#ifndef WIFI_ACP 
#define CONNECT_TO_HOME //подключить к домашней сети
#define IP 200 //IP адрес датчика. Должен быть уникален. 192.168.1.IP
#endif

#ifdef CLIENT
const char* host = "192.168.1.1";
#endif

//#include "calibrate.h"
//#define COMPL_K 0.05
#include <Arduino.h>
#include <MPU6050.h>
#include <Wire.h> 
#include <Kalman.h>
#include <ESP8266WiFi.h>
#include "constants.h"

#include <ModbusRTU.h>
#include <string>

#include "server_html.h"


#ifdef CONNECT_TO_HOME
/* SSID и пароль домашней сети */
const char* ssid = "Tenda";       // SSID
const char* password = "Nikol1204Svet";  // пароль
#endif

#ifndef CONNECT_TO_HOME
/* Установите здесь свои SSID и пароль */
const char* ssid = "NodeMCU";       // SSID
const char* password = "12345678";  // пароль
#endif

/* Настройки IP адреса */
IPAddress local_ip(192,168,1,IP);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);
#ifdef SERVER
ESP8266WebServer server(80);
#endif

MPU6050 mpu;

#ifdef CLIENT
WiFiClient client;
int port = 80;
#endif

namespace vals{
Kalman kalmanX;
Kalman kalmanY;
Kalman kalmanZ;
uint8_t IMUAddress = 0x68;

/* IMU Data */
int16_t accX;
int16_t accY;
int16_t accZ;
int16_t tempRaw;
int16_t gyroX;
int16_t gyroY;
int16_t gyroZ;
float accXangle; // Angle calculate using the accelerometer
float accYangle;
float accZangle;
float temp;
float gyroXangle = 180; // Angle calculate using the gyro
float gyroYangle = 180;
float gyroZangle = 180;
float compAngleX = 180; // Calculate the angle using a Kalman filter
float compAngleY = 180;
float compAngleZ = 180;
float kalAngleX; // Calculate the angle using a Kalman filter
float kalAngleY;
float kalAngleZ;
uint32_t timer;

float sumX=0;
float sumY=0;
float sumZ=0;
float avX=0;
float avY=0;
float avZ=0;
float dX;
float dY;
float dZ;
float dsX;
float dsY;
float dsZ;
int i=0;
int nomer_izmerenia = 0;

#define BUFFER_SIZE 100
int16_t ax, ay, az;
int16_t gx, gy, gz;
}

using namespace vals;

// ======= ФУНКЦИЯ КАЛИБРОВКИ =======
void handle_onCalibrate() {
  long offsets[6];
  long offsetsOld[6];
  int16_t mpuGet[6];

  // используем стандартную точность
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

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


#ifdef SERVER
void handle_OnConnect(){
  //Serial.write("connection");
  if(not (avX==avY and avY==avZ and avZ==0)){
    server.send(200, "text/html", SendHTML(avX, avY, avZ, dX, dY, dZ, nomer_izmerenia));
  }
  else{
    server.send(200, "text/plain", "Loading...Wait for 30s");
  }
}
#endif

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  kalmanX.setAngle(180); // Set starting angle
  kalmanY.setAngle(180);
  kalmanZ.setAngle(180);
  timer = micros();
  
#ifdef WIFI_ACP
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);
#endif

#ifdef WIFI_CLI
  //WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.config(local_ip, gateway, subnet);
  Serial.println(WiFi.localIP()); 
    while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
#endif

  delay(100);

#ifdef SERVER
  server.on("/", handle_OnConnect);
  server.on("/calibrate", handle_onCalibrate);
  //server.onNotFound(handle_NotFound);  
  
  server.begin();
  Serial.println("HTTP server started");
#endif

  mpu.setFullScaleAccelRange(3);//-2..2 g/s
  //mpu.setFullScaleAccelRange(0);//-16..16 g/s // вернуть, если что-то с погрешностью пойдет не так
  mpu.setFullScaleGyroRange(0);//-250..250 deg/sec
}


void getValues() {
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

void calculateAngles() {
  /* Calculate the angls based on the different sensors and algorithm */
  accZangle = (atan2(accX, accY) + PI) * RAD_TO_DEG;
  accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG;
  accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;
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


void loop() {
  #ifdef SERVER
  server.handleClient();
  #endif
  
  uint32_t looptime = micros();
  getValues();
  calculateAngles();
  // Serial.println(micros() - looptime);
  Serial.print(kalAngleX,4); Serial.print(" ");
  Serial.print(kalAngleY,4); Serial.print(" ");
  Serial.print(kalAngleZ,4); Serial.print(" ");
  // Serial.print(temp,4);
  Serial.println();
  
  sumX+=kalAngleX;
  sumY+=kalAngleY;
  sumZ+=kalAngleZ;
  dsX+=abs(kalAngleX-avX);
  dsY+=abs(kalAngleY-avY);
  dsZ+=abs(kalAngleZ-avZ);
  i++;
  
  if(i == count){  
    nomer_izmerenia++;
    avX = sumX/count;
    avY = sumY/count;
    avZ = sumZ/count;
    dX = dsX/count;
    dY = dsY/count;
    dZ = dsZ/count;
    sumX=0;
    sumY=0;
    sumZ=0;
    i=0;

  #ifdef CLIENT
      WiFiClient client;
      HTTPClient http;
      String url = "https://192.168.1.1/params/";
      url += "?avX=";
      url += avX;
      url += "?avY=";
      url += avY;
      url += "?avZ=";
      url += avZ;      
      // Your Domain name with URL path or IP address with path
      http.begin(client, url.c_str());
  
      // If you need Node-RED/server authentication, insert user and password below
      //http.setAuthorization("REPLACE_WITH_SERVER_USERNAME", "REPLACE_WITH_SERVER_PASSWORD");
        
      // Send HTTP GET request
      int httpResponseCode = http.GET();

  #endif
  }
   
  delay(1); // The accelerometer's maximum samples rate is 1kHz
}


