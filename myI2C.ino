#include <util/twi.h>

#include "I2C.h"

void setup()
{
  myI2C_init(1);
  Serial.begin(115200U);
}

uint8_t outdata = 0;

uint16_t rx_bytes = 0;
uint16_t tx_bytes = 0;

uint8_t slaveHandler(uint8_t *data, uint8_t flags)
{
  if (flags & MYI2C_SLAVE_ISTX) {
    *data = outdata++;
    tx_bytes++;
    return 1;
  } else {
    if (flags & MYI2C_SLAVE_ISFIRST) {
      Serial.println("!!");
    }
    Serial.println(*data, 16);
    rx_bytes++;
    return 1;
  }
}

void slave()
{
  Serial.println("slave mode");
  myI2C_slaveSetup(32, 0, 0, slaveHandler);
  while(1) {
    delay(1000);
    Serial.print("Stats TX: ");
    Serial.print(tx_bytes);
    Serial.print(" RX: ");
    Serial.println(rx_bytes);
  }
}

uint8_t buf[8] = {0x11,0x22,0x33,0x44,0x77,0x88,0x99,0xff};
uint8_t rbuf[8];

void master()
{
  Serial.println("master mode");

  while(1) {
    Serial.print("sending, status =");
    Serial.println(myI2C_writeTo(32, buf, 8, MYI2C_WAIT));
    delay(100);
    Serial.print("reading, status =");
    Serial.println(myI2C_readFrom(32, rbuf, 8, MYI2C_WAIT));
    for (int i=0;i<8;i++) {
      Serial.print(rbuf[i],16);
      Serial.print(' ');
    }
    Serial.println();
    delay(300);
  }
}



void loop()
{
  pinMode(3,INPUT);
  digitalWrite(3,HIGH);
  delay(1);
  if (digitalRead(3)) {
    slave();
  } else {
    master();
  }
}
