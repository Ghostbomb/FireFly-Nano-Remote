#include <utils.h>

// CRC-8 - based on the CRC8 formulas by Dallas/Maxim
// code released under the therms of the GNU GPL 3.0 license
byte CRC8(const void *data, byte len) {

  byte crc = 0x00;

  for (int i = 0; i < len; i++) {
    byte extract =  *((byte*) data + i);

    for (byte tempI = 8; tempI; tempI--) {
      byte sum = (crc ^ extract) & 0x01;
      crc >>= 1;
      if (sum) {
        crc ^= 0x8C;
      }
      extract >>= 1;
    }
  }
  return crc;
}

unsigned long millisSince(unsigned long time) {
  return millis() - time;
}

double secondsSince(unsigned long time) {
  return millisSince(time) / 1000.0;
}

void printStruct(const void * data, byte len) {

  for(int i = 0; i < len; i++){
    byte c =  *((byte*) data + i);
    Serial.print(c, HEX);
    Serial.print(" ");
  }
}
