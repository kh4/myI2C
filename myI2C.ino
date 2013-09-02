#include <util/twi.h>

#define I2C_FREQ 400000

// flag values
#define MYI2C_WAIT     0x01 // do not return until transfer is done
#define MYI2C_NOSTOP   0x02 // do not release bus after transaction

#define MYI2C_SLAVE_ISTX    0x01 // slave handler should 'send' a byte
#define MYI2C_SLAVE_ISFIRST 0x02 // first byte after addressing (normally register number)
// slave handler routine, should return desired ACK status
// *data  - pointter to fetch/store data
// flags  - see above
// RETVAL 0 - no more data wanted/available
//        1 - OK to continue transfer
uint8_t (*myI2C_slaveHandler)(uint8_t *data, uint8_t flags) = NULL;

// Internal state data
volatile uint8_t myI2C_slarw;    // slave address & RW bit, used in master mode
volatile uint8_t *myI2C_dataptr; // TX/RX data ptr
volatile uint8_t myI2C_datacnt;  // data countter
#define MYI2C_DONTSTOP 0x01 // do not release bus after operation
#define MYI2C_REPSTART 0x02 // going to use repeated start on next transfer
#define MYI2C_BUSY     0x04 // transfer ongoing
volatile uint8_t myI2C_flags;

void myI2C_init(uint8_t enable_pullup)
{
  digitalWrite(SDA, enable_pullup?1:0);
  digitalWrite(SCL, enable_pullup?1:0);

  myI2C_datacnt=0;
  myI2C_flags=0;

  TWSR |= ~(_BV(TWPS0)|_BV(TWPS1));
  TWBR = ((F_CPU / I2C_FREQ) - 16) / 2;

  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
}

void myI2C_slaveSetup(uint8_t address, uint8_t mask, uint8_t enable_gcall,
                      uint8_t (*handler)(uint8_t *data, uint8_t flags))
{
  TWAR  = (address << 1) | (enable_gcall?1:0);
  TWAMR = (mask << 1);
  myI2C_slaveHandler = handler;
}

void myI2C_reply(uint8_t ack)
{
  if(ack) {
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);
  } else {
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT);
  }
}

void myI2C_stop(void)
{
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTO);
  while(TWCR & _BV(TWSTO)) {
    continue;
  }
}

void myI2C_releaseBus(void)
{
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT);
}

// ISR //
SIGNAL(TWI_vect)
{
  //  Serial.println(TW_STATUS,16);
  switch(TW_STATUS) {
    // All Master
  case TW_START:     // sent start condition
  case TW_REP_START: // sent repeated start condition
    // copy device address and r/w bit to output register and ack
    TWDR = myI2C_slarw;
    myI2C_reply(1);
    break;

    // Master Transmitter
  case TW_MT_SLA_ACK:  // slave receiver acked address
  case TW_MT_DATA_ACK: // slave receiver acked data
    // if there is data to send, send it, otherwise stop
    if(myI2C_datacnt--) {
      // copy data to output register and ack
      TWDR = *(myI2C_dataptr++);
      myI2C_reply(1);
    } else {
      myI2C_flags &= ~MYI2C_BUSY;
      if (myI2C_flags&MYI2C_DONTSTOP) {
        myI2C_flags &= ~MYI2C_DONTSTOP;
        myI2C_flags |= MYI2C_REPSTART;
        TWCR = _BV(TWINT) | _BV(TWSTA)| _BV(TWEN);
      } else {
        myI2C_stop();
      }
    }
    break;
  case TW_MT_SLA_NACK:  // address sent, nack received
    //twi_error = TW_MT_SLA_NACK;
    myI2C_flags &= ~MYI2C_BUSY;
    myI2C_stop();
    break;
  case TW_MT_DATA_NACK: // data sent, nack received
    //twi_error = TW_MT_DATA_NACK;
    myI2C_flags &= ~MYI2C_BUSY;
    myI2C_stop();
    break;
  case TW_MT_ARB_LOST: // lost bus arbitration
    //twi_error = TW_MT_ARB_LOST;
    myI2C_flags &= ~MYI2C_BUSY;
    myI2C_releaseBus();
    break;

    // Master Receiver
  case TW_MR_DATA_ACK: // data received, ack sent
    // put byte into buffer
    *(myI2C_dataptr++) = TWDR;
    myI2C_datacnt--;
  case TW_MR_SLA_ACK:  // address sent, ack received
    // ack if more bytes are expected, otherwise nack
    if (myI2C_datacnt) {
      myI2C_reply(1);
    } else {
      myI2C_reply(0);
    }
    break;
  case TW_MR_DATA_NACK: // data received, nack sent
    // put final byte into buffer
    *(myI2C_dataptr++) = TWDR;
    myI2C_datacnt--;
    myI2C_flags &= ~MYI2C_BUSY;
    if (myI2C_flags&MYI2C_DONTSTOP) {
      myI2C_flags &= ~MYI2C_DONTSTOP;
      myI2C_flags |= MYI2C_REPSTART;
      TWCR = _BV(TWINT) | _BV(TWSTA)| _BV(TWEN);
    } else {
      myI2C_stop();
    }
    break;
  case TW_MR_SLA_NACK: // address sent, nack received
    myI2C_flags &= ~MYI2C_BUSY;
    myI2C_stop();
    break;
    // TW_MR_ARB_LOST handled by TW_MT_ARB_LOST case

    // Slave Receiver
  case TW_SR_SLA_ACK:   // addressed, returned ack
  case TW_SR_GCALL_ACK: // addressed generally, returned ack
  case TW_SR_ARB_LOST_SLA_ACK:   // lost arbitration, returned ack
  case TW_SR_ARB_LOST_GCALL_ACK: // lost arbitration, returned ack
    myI2C_datacnt = 0;
    myI2C_reply(1);
    break;
  case TW_SR_DATA_ACK:       // data received, returned ack
  case TW_SR_GCALL_DATA_ACK: // data received generally, returned ack
    if (myI2C_slaveHandler) {
      uint8_t data=TWDR;
      myI2C_reply(myI2C_slaveHandler(&data, (myI2C_datacnt++==0)?MYI2C_SLAVE_ISFIRST:0));
    } else {
      myI2C_reply(0);
    }
    break;
  case TW_SR_STOP: // stop or repeated start condition received
    myI2C_stop();
    myI2C_releaseBus();
    break;
  case TW_SR_DATA_NACK:       // data received, returned nack
  case TW_SR_GCALL_DATA_NACK: // data received generally, returned nack
    // nack back at master
    myI2C_reply(0);
    break;

    // Slave Transmitter
  case TW_ST_SLA_ACK:          // addressed, returned ack
  case TW_ST_ARB_LOST_SLA_ACK: // arbitration lost, returned ack
    // just fall thru to data sending
  case TW_ST_DATA_ACK: // byte sent, ack returned

    if (myI2C_slaveHandler) {
      uint8_t data,ret;
      ret = myI2C_slaveHandler(&data, MYI2C_SLAVE_ISTX);
      TWDR = data;
      myI2C_reply(ret); // reply with ACK or NACK depending on slave callback
    } else {
      myI2C_reply(0);
    }
    break;
  case TW_ST_DATA_NACK: // received nack, we are done
  case TW_ST_LAST_DATA: // received ack, but we are done already!
    // ack future responses
    myI2C_reply(1);
    break;

    // All
  case TW_NO_INFO:   // no state information
    break;
  case TW_BUS_ERROR: // bus error, illegal stop/start
    //twi_error = TW_BUS_ERROR;
    myI2C_stop();
    break;
  }
}

void myI2C_wait()
{
  while (myI2C_flags & MYI2C_BUSY) {
    continue;
  }
}

uint8_t myI2C_writeTo(uint8_t address, uint8_t* data, uint8_t length, uint8_t flags)
{
  myI2C_wait();
  myI2C_datacnt = length;
  myI2C_dataptr = data;
  myI2C_slarw = TW_WRITE | (address << 1);

  myI2C_flags |= MYI2C_BUSY | ((flags&MYI2C_NOSTOP)?MYI2C_DONTSTOP:0);

  if (myI2C_flags & MYI2C_REPSTART) {
    myI2C_flags &= ~MYI2C_REPSTART;
    TWDR = myI2C_slarw;
    TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE);
  } else {
    TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE) | _BV(TWSTA);
  }
  if (flags & MYI2C_WAIT) {
    myI2C_wait();
  }
  return 0;
}


uint8_t myI2C_readFrom(uint8_t address, uint8_t* data, uint8_t length, uint8_t flags)
{
  myI2C_wait();

  myI2C_datacnt = length;
  myI2C_dataptr = data;
  myI2C_slarw = TW_READ | (address << 1);

  myI2C_flags |= MYI2C_BUSY | ((flags&MYI2C_NOSTOP)?MYI2C_DONTSTOP:0);

  if (myI2C_flags & MYI2C_REPSTART) {
    myI2C_flags &= ~MYI2C_REPSTART;
    TWDR = myI2C_slarw;
    TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE);
  } else {
    TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE) | _BV(TWSTA);
  }

  if (flags & MYI2C_WAIT) {
    myI2C_wait();
  }
  return 0;
}

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
