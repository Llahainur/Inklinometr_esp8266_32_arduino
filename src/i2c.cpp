#include "i2c.h"

void i2c_test(SoftwareSerial RS485)
  {
  int nDevices;
  byte error, address;

  RS485.println("Scanning I2C bus...\n");


  nDevices = 0;

  RS485.print("   00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F");


  for(address = 0; address < 128; address++ )
  {
    if((address % 0x10) == 0)
    {
      RS485.println();
      if(address < 16)
        RS485.print('0');
      RS485.print(address, 16);
      RS485.print(" ");
    }
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);error = Wire.endTransmission();


    if (error == 0)
    {
      if (address<16)
         RS485.print("0");
      RS485.print(address, HEX);

      nDevices++;
    }
    else
    {
      RS485.print("--");
    }

    RS485.print(" ");
    delay(1);
  }
  RS485.println();

  if (nDevices == 0)
     RS485.println("No I2C devices found\n");
   else
   {

     RS485.print("Found ");
     RS485.print(nDevices);
     RS485.println(" device(s) ");
   }           // wait 5 seconds for next scan

  }