//#define SERVER //Добавлять ли веб-сервер
//or
//#define CLIENT //Отправляем данные на веб сервер (https://radioprog.ru/post/1119), не реализовано
//доделать веб клиент
//or
#define WIRED //Используем Модбас

#ifdef SERVER
#define WIFI_ACP //если датчик - центральный, делаем его точкой доступа
//or
//#define WIFI_CLI //если датчик - клиент другой сети
#endif
//#define NOCMD

const int count=1000;// количество измерений, от которых берется среднее

float noize = 0.05;//уровень шума, градусов

#define ADDR_ARR {'1','0','0'}
#ifndef WIFI_ACP 
//#define CONNECT_TO_HOME //подключить к домашней сети
//#define IP 2 //IP адрес датчика. Должен быть уникален. 192.168.1.IP
#endif

#ifdef WIFI_ACP 
#define IP 1 //IP адрес датчика. Должен быть уникален. 192.168.1.IP
#endif

#ifdef CLIENT
const char* host = "192.168.1.1";
#endif

#include <Arduino.h>
#include <Wire.h> 
#include <Kalman.h>
#include <RS485.h>
//#include <CmdLine.h>
// #include <ESP8266WiFi.h>

// #include <server_html.h>
#include <inklin_logic.h>
#include <i2c.h>
//#include <Cmd_Iface.h>

//CmdLine cmdline(Serial);

MPU6050 mpu;

SoftwareSerial RS485(RO, DI);//RO DI

uint8_t IMUAddress = 0x68;

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

// /* Настройки IP адреса */
// IPAddress local_ip(192,168,1,IP);
// IPAddress gateway(192,168,1,1);
// IPAddress subnet(255,255,255,0);

Kalman kalmanX;
Kalman kalmanY;
Kalman kalmanZ;

//float Q_bias = 0.0001f;
#ifdef SERVER
ESP8266WebServer server(80);
#endif



#ifdef CLIENT
WiFiClient client;
int port = 80;
#endif

namespace vals{
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
float gyroXangle = 0; // Angle calculate using the gyro
float gyroYangle = 0;
float gyroZangle = 0;
float compAngleX = 0; // Calculate the angle using a Kalman filter
float compAngleY = 0;
float compAngleZ = 0;
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

int16_t ax, ay, az;
int16_t gx, gy, gz;
}

using namespace vals;

// ======= ХЭНДЛЕР КАЛИБРОВКИ =======
void handle_onCalibrate() {
  Calibrate(mpu);
  }


#ifdef SERVER
void handle_OnConnect(){
  //Serial.write("connection");
  if(not (avX==0 and avY==0 and avZ==0)){
    server.send(200, "text/html", SendHTML(avX, avY, avZ, dX, dY, dZ, nomer_izmerenia,timer/1000000, dX,dY,dZ, temp));    
  }
  else{
    server.send(200, "text/plain", "Loading...Wait for 30s");
  }
}
#endif

void print_addr(){
  int ar[3]=ADDR_ARR;
  //RS485.print("address: ");
  for(int i=0;i<3;i++){Serial.write(ar[i]);}
  //RS485.println();
};

void setup() {
  
  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);

  Serial.begin(9600);//для serial, 2400
  //Serial.begin(2400);//для rs
  Wire.begin();
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  //Serial.println("INIT");

  #ifdef WIRED
  RS485_mode(1);
  delay(5);
  Serial.println("INIT");
  #endif
  
  // getValues(accX,accY,accZ,tempRaw,gyroX,gyroY,gyroZ,temp,IMUAddress);
  // calculateAngles(accX,accY,accZ, tempRaw,gyroX,gyroY,gyroZ,accXangle,accYangle,accZangle,temp,gyroXangle, 
  // gyroYangle,gyroZangle,kalAngleX,kalAngleY,kalAngleZ,timer,kalmanX,kalmanY,kalmanZ);

  //Serial.println(accXangle);
  kalmanX.setAngle(0); // Set starting angle
  kalmanY.setAngle(0);
  kalmanZ.setAngle(0);
  // kalmanX.setQbias(Q_bias);
  // kalmanY.setQbias(Q_bias);
  // kalmanZ.setQbias(Q_bias);
  
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
  WiFi.setAutoReconnect(true);
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
#endif

  //delay(100);

#ifdef SERVER
  server.on("/", handle_OnConnect);
  server.on("/calibrate", handle_onCalibrate);
  //server.onNotFound(handle_NotFound);  
  
  server.begin();
  Serial.println("HTTP server started");
#endif

  mpu.setFullScaleAccelRange(3);//-2..2 g/s
  ////mpu.setFullScaleAccelRange(0);//-16..16 g/s // вернуть, если что-то с погрешностью пойдет не так
  mpu.setFullScaleGyroRange(0);//-250..250 deg/sec
  //Serial.println("INIT_DONE");

  print_addr();
  Serial.println();
  RS485_mode(0);
}

void loop() {
  RS485_mode(1);
  Serial.println("INIT");
  RS485_mode(0);
  #ifdef SERVER
  server.handleClient();
  #endif

  uint32_t looptime = micros();
  getValues(accX,accY,accZ,tempRaw,gyroX,gyroY,gyroZ,temp,IMUAddress);
  calculateAngles(accX,accY,accZ, tempRaw,gyroX,gyroY,gyroZ,accXangle,accYangle,accZangle,temp,gyroXangle, 
  gyroYangle,gyroZangle,kalAngleX,kalAngleY,kalAngleZ,timer,kalmanX,kalmanY,kalmanZ);


  #ifndef WIRED
  // Serial.println(micros() - looptime);
  //Serial.print(kalAngleX,4); Serial.print(" ");
  // Serial.print(kalAngleY,4); Serial.print(" ");
  // Serial.print(kalAngleZ,4); Serial.print(" ");
  // Serial.print(temp,4);
  // Serial.print(avX,4); Serial.print(" ");
  // Serial.print(dX,4); Serial.print(" ");
  // Serial.println();
  #endif
  
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
    #ifdef NOCMD
    RS485_mode(1);
    Serial.print(timer/1000000,0); Serial.print(" ");
    Serial.print(avX,4); Serial.print(" ");
    Serial.print(avY,4); Serial.print(" ");
    Serial.print(avZ,4); Serial.print(" ");
    Serial.println();

    RS485_mode(0);
    #endif

    sumX=0;
    sumY=0;
    sumZ=0;
    dsX=0;
    dsY=0;
    dsZ=0;
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

  #ifdef WIRED
  char data[5];
  char addr[3]=ADDR_ARR;
  char cmd='f';
  bool flag=1;
  int const len=4;

  //if(Serial.available()>0){Serial.println("got");}

  if (Serial.available() > 3){
    for(int i = 0; i < len; i++){
      data[i] = Serial.read();//пакет - 4 буквы. Первые 3 - адрес, последняя - команда. 
      
      if (i<len-1){
        if(addr[i]!=data[i]){;
          flag = 0;
          Serial.print(data[i]);
        }
      }
      if(i==len-1){
        cmd=data[i];
        //Serial.println(cmd);
      }
    }
    if (flag){
    if(cmd=='g'){
      RS485_mode(1);
      Serial.print(timer/1000000); Serial.print(" ");
      Serial.print(avX,4); Serial.print(" ");
      Serial.print(avY,4); Serial.print(" ");
      Serial.print(avZ,4); Serial.print(" ");
      Serial.println();
      flag = 0;
      RS485_mode(0);
    }
    else if(cmd=='n'){
      RS485_mode(1);
      Serial.print("accxAng ");
      Serial.print(accXangle);  Serial.print(" accX ");
      Serial.print(accX);       Serial.print(" accY ");
      Serial.print(accY);       Serial.print(" kalmX ");
      Serial.print(kalAngleX);  Serial.print(" N ");
      Serial.print(nomer_izmerenia); Serial.print(" ");
      Serial.println();
      flag = 0;
      RS485_mode(0);
      }
      else if(cmd=='p'){
      RS485_mode(1);
      Serial.print("ping ");
      print_addr();
      Serial.println();
      flag = 0;
      RS485_mode(0);
      }
    else if (cmd=='c'){
      RS485_mode(1);
      Serial.println("CALIBRAION");
      print_addr();Serial.println();
      Calibrate(mpu);
      flag = 0;
      RS485_mode(0);
    }
    else if (cmd=='i'){
      RS485_mode(1);
      i2c_test();
      print_addr();
      Serial.println();
      flag = 0;
      RS485_mode(0);
    }
    else{
      RS485_mode(1);
      Serial.println("input error ");
      Serial.print(cmd);
      RS485_mode(0);
    }
    }
    }
    
  #endif
  delay(1); // The accelerometer's maximum samples rate is 1kHz
}


