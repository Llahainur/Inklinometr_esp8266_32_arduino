#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>

String SendHTML_onCalibration(float xA,float yA,float zA, float xG,float yG,float zG);

String SendHTML(float x,float y,float z, float dX, float dY, float dZ, int nomer_izmerenia);