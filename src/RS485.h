#include <Arduino.h>
#include <SoftwareSerial.h>

#define DI 8  //Driver In 
#define RO 9  //Receiver Out
#define RE 10 //Receiver Enable
#define DE 11 //Driver Enable     

void RS485_mode(bool mode);