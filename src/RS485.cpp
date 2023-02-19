#include <RS485.h>

/*0-get, 1-send*/
void RS485_mode(bool mode, int re, int de){
  pinMode(re, OUTPUT);
  pinMode(de, OUTPUT);
  digitalWrite(re, mode);
  digitalWrite(de, mode);

  digitalWrite(RE, mode);
  digitalWrite(DE, mode);
}

