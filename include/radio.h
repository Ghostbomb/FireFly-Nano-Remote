#ifndef _RADIO_H
#define _RADIO_H

// #include <LoRa.h>

// Change to allowed frequency
#define RF_FREQ 915.0

#ifdef ESP32

  void initRadio();

#elif ARDUINO_SAMD_ZERO

  #include <RH_RF69.h>
  void initRadio(RH_RF69 &rdio);

#endif

#endif
