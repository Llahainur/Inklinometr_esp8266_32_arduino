#include <RS485.h>

/*0-get, 1-send*/
void RS485_mode(bool mode){
  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);
  digitalWrite(RE, mode);//RE is the 'Receiver Enable' pin and must be pulled low whenever you want to be able to receive data.
  digitalWrite(DE, mode);//DE is the 'Driver Enable' pin and must be pulled high while you're transmitting data.
}

/*
DE is the 'Driver Enable' pin and must be pulled high while you're transmitting data. Depending on your micro and how you're using its interrupts you may need to be careful about when you pull it back low - check that all of the bits are completely finished first or you risk truncating the end of your message. You must pull it back low before you'll be able to receive anything.
RE is the 'Receiver Enable' pin and must be pulled low whenever you want to be able to receive data.
You'll notice that the DE and RE pins have opposite polarity. DE is active-high and RE is active-low. So you can tie them together and control them from one pin if you want to - high means you want to transmit (DE active, RE inactive), and low means you want to receive (RE active, DE inactive).
Another possibility is to tie RE to ground and only control DE. You would use this configuration is you want to be able to listen to yourself talk. This would be useful in cases where there could be multiple masters talking on your RS-485 bus and you need to check that what you think you're sending is not being corrupted by another transmission occurring at the same time.As Tut correctly points out, this is not a reliable method of collision detection
If you're only ever going to be either transmitting or receiving then you could tie both DE and RE high (permanent transmit) or both low (permanent receive).*/

