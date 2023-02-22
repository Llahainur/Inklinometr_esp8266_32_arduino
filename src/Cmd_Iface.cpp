#include <Cmd_Iface.h>

////////////////////////////////////////////////////////////////////////////////////////////////////
void printHelpCmd(const char *arg) {
  Serial.println("List of commands:");
  Serial.println("help                    print this help");
  Serial.println("setLED HIGH|LOW      set LED on 13 pin to HIGH or LOW");
  Serial.println("getTime                returns runtime");
}

