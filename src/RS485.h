#include <Arduino.h>
#include <SoftwareSerial.h>

#define DI 8
#define RO 9
#define RE 10
#define DE 11           // Подключаем библиотеку SoftwareSerial

void RS485_mode(bool mode, int re, int de);