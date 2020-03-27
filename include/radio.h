#ifndef _RADIO_H
#define _RADIO_H

// #include <LoRa.h>
#include <globals.h>

// Change to allowed frequency
#ifdef RFM_USA
#define RF_FREQ 915.0
#else
#ifdef RFM_EU
#define RF_FREQ 415.0
#endif
#endif

#ifdef ESP32

  void initRadio();

#elif ARDUINO_SAMD_ZERO

  #include <RH_RF69.h>
  void initRadio(RH_RF69 &rdio);

#endif

#endif
