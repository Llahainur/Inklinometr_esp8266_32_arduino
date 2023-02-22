#include <Cmd_Iface.h>

////////////////////////////////////////////////////////////////////////////////////////////////////
void printHelpCmd(const char *arg) {
  Serial.println("List of commands:");
  Serial.println("help                    помощь");
  Serial.println("getAngle      получить показания датчика");
  Serial.println("startCalibrate      откалибровать датчик");
}

