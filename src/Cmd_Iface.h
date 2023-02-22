#include <CmdLine.h>
//#include <RS485.h>

//SoftwareSerial RS485(RO, DI);//RO DI

//CmdLine cmdline(Serial);

void printHelpCmd(const char *arg);
void getAngleCmd(const char *arg);
void startCalibrateCmd(const char *arg);

const cmd_t commands[] = {
  {"help", printHelpCmd},
  {"getAngle", getAngleCmd},
  {"startCalibrate", startCalibrateCmd},
};

