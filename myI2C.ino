#include <util/twi.h>

#include "I2C.h"

void setup()
{
  myI2C_init(0);
  Serial.begin(115200U);
}


void loop()
{
  uint8_t buf[16];
  buf[0]=0x75;
  Serial.println("send");
  myI2C_writeTo(0x68,buf,1,MYI2C_WAIT|MYI2C_NOSTOP);
  Serial.println("receive");
  myI2C_readFrom(0x68,buf,1,MYI2C_WAIT);
  Serial.println(buf[0],16);
  delay(1000);
}
