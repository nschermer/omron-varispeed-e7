#include "memobus.h"

/* SoftwareSerial or Serial1 (HW serial) on the MEGA32U4 boards */
#if defined(__AVR_ATmega32U4__)
#define RS485 Serial1
#else
#define RS485_SW_SERIAL
#include <SoftwareSerial.h>
SoftwareSerial RS485(RS485_RX, RS485_TX);
#endif

#define RS485_TRANSMIT HIGH
#define RS485_RECEIVE  LOW

/* memobus message spacing timer */
unsigned long MemobusSpacingTimer = 0;

void memobus_setup () {
  pinMode(RS485_RTS, OUTPUT);
  RS485.begin(MEMOBUS_BITRATE);
  RS485.setTimeout(MEMOBUS_TIMEOUT);
  while (!RS485);
}

#if 0
void memobus_print_buffer(byte * buf, int len) {
  for (byte i = 0; i < len; i++ ) {
    Serial.print(buf[i], HEX);
    Serial.print(F(" "));
  }
  Serial.println();
}
#endif

void memobus_check_spacing_delay() {
  unsigned long expired;

  /* check if we need to delay to make sure the new command is
     MEMOBUS_WAIT_TIME after the last response message */
  expired = millis() - MemobusSpacingTimer;
  if (expired < MEMOBUS_WAIT_TIME) {
    delay(MEMOBUS_WAIT_TIME - expired);
  }
}

bool memobus_write_register(uint16_t address, uint16_t data) {
  byte msg[11];
  byte buf[8] = { 0 };
  uint16_t quantity = 1; /* nr of registers */
  uint16_t crc;
  int retval;

  msg[0] = MEMOBUS_SLAVE_ID;
  msg[1] = 0x10; /* write memory */
  msg[2] = highByte(address);
  msg[3] = lowByte(address);
  msg[4] = highByte(quantity);
  msg[5] = lowByte(quantity);
  msg[6] = lowByte(quantity * 2); /* nr high/low data fields */
  msg[7] = highByte(data);
  msg[8] = lowByte(data);

  crc = memobus_msg_crc16 (msg, 9);
  msg[9] = lowByte(crc); /* crc has swapped bytes */
  msg[10] = highByte(crc);

  /* check if we need to sleep */
  memobus_check_spacing_delay();

  /* send request */
  digitalWrite(RS485_RTS, RS485_TRANSMIT);
  RS485.write(msg, sizeof(msg));
  RS485.flush();

  delay(MEMOBUS_WAIT_TIME);

  /* read reply */
  digitalWrite(RS485_RTS, RS485_RECEIVE);
  retval = RS485.readBytes(buf, sizeof (buf));
  if (retval < (int) sizeof (buf)) {
    error_handler(__LINE__);
    return false;
  }

  /* store new last response timestamp */
  MemobusSpacingTimer = millis();

  /* check for error */
  if (buf[0] != MEMOBUS_SLAVE_ID || buf[1] == 0x90) {
    error_handler(__LINE__);
    return false;
  }

  /* validate reply */
  crc = memobus_msg_crc16 (buf, sizeof(buf) - 2);
  if (lowByte(crc) != buf[6] || highByte(crc) != buf[7]) {
    error_handler(__LINE__);
    return false;
  }

  /* check if all values are written (first 6 fields) */
  return memcmp(msg, buf, 6) == 0;
}

bool memobus_read_register(uint16_t start_address, uint16_t quantity, uint16_t *ret_status) {
  byte msg[8];
  uint16_t crc;
  byte buf[64] = { 0 };
  size_t bufsize;
  uint16_t i;
  uint16_t offset;
  int retval;

  msg[0] = MEMOBUS_SLAVE_ID;
  msg[1] = 0x03; /* read memory */
  msg[2] = highByte(start_address);
  msg[3] = lowByte(start_address);
  msg[4] = highByte(quantity);
  msg[5] = lowByte(quantity);

  crc = memobus_msg_crc16 (msg, 6);
  msg[6] = lowByte(crc); /* crc has swapped bytes */
  msg[7] = highByte(crc);

  /* buffer reply size (response message has 5 items + 2 for each register) */
  bufsize = 5 + 2 * quantity;

  /* check if our static buffer is bug enough */
  if (bufsize > sizeof(buf)) {
    error_handler(__LINE__);
    return false;
  }

  /* check if we need to sleep */
  memobus_check_spacing_delay();

  /* send request */
  digitalWrite(RS485_RTS, RS485_TRANSMIT);
  RS485.write(msg, sizeof(msg));
  RS485.flush();

  delay(MEMOBUS_WAIT_TIME);

  /* read reply */
  digitalWrite(RS485_RTS, RS485_RECEIVE);
  retval = RS485.readBytes(buf, bufsize);
  if (retval < (int) bufsize) {
    //error_handler(__LINE__);
    return false;
  }

  /* store new last response timestamp */
  MemobusSpacingTimer = millis();

  if (buf[0] != MEMOBUS_SLAVE_ID || buf[1] == 0x83) {
    error_handler(__LINE__);
    return false;
  }

  /* validate reply */
  crc = memobus_msg_crc16 (buf, bufsize - 2);
  if (lowByte(crc) != buf[bufsize - 2] || highByte(crc) != buf[bufsize - 1]) {
    //error_handler(__LINE__);
    return false;
  }

  /* store data in return register */
  for (i = 0; i < quantity; i++) {
    offset = i * 2 + 3;
    /* merge higher and lower bit in integer */
    ret_status[i] = buf[offset] << 8 | buf[offset + 1];
  }

  return true;
}

#if MEMOBUS_LOOPBACK
boolean memobus_loopback_test(void) {
  uint16_t crc;
  byte buf[8] = {0 };
  byte msg[8];
  long rnd;
  int retval;

  /* we insert random data in the msg */
  rnd = random();

  /* create the loopback request */
  msg[0] = MEMOBUS_SLAVE_ID;
  msg[1] = 0x08; /* loopback test */
  msg[2] = highByte(rnd);
  msg[3] = lowByte(rnd);
  msg[4] = highByte(rnd + 1);
  msg[5] = lowByte(rnd + 1);

  /* complete crc */
  crc = memobus_msg_crc16(msg, 6);
  msg[6] = lowByte(crc); /* crc has swapped bytes */
  msg[7] = highByte(crc);

  /* check if we need to sleep */
  memobus_check_spacing_delay();

  /* send request */
  digitalWrite(RS485_RTS, RS485_TRANSMIT);
  RS485.write(msg, sizeof(msg));
  RS485.flush();

  delay(MEMOBUS_WAIT_TIME);

  /* read reply */
  digitalWrite(RS485_RTS, RS485_RECEIVE);
  retval = RS485.readBytes(buf, sizeof(buf));
  if (retval < (int) sizeof(buf)) {
    error_handler(__LINE__);
    return false;
  }

  /* store new last response timestamp */
  MemobusSpacingTimer = millis();

  /* check for error */
  if (buf[0] != MEMOBUS_SLAVE_ID || buf[1] == 0x90)
    return false;

  /* with loopback; message and reply are identical */
  return memcmp(msg, buf, sizeof (buf)) == 0;
}
#endif

uint16_t memobus_msg_crc16(byte * buf, size_t len)
{
  uint16_t crc = 0xFFFF;

  for (size_t pos = 0; pos < len; pos++) {
    crc ^= (uint16_t) buf[pos];       // XOR byte into least sig. byte of crc

    for (int i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      } else {                        // Else LSB is not set
        crc >>= 1;                    // Just shift right
      }
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return crc;
}
