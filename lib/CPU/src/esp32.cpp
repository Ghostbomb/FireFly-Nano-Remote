/*
    ESP32 related functions
*/
#ifdef ESP32

#include "CPU.h"

#include <esp_sleep.h>

namespace CPU {

  void sleep(int pin) {

    esp_sleep_enable_ext0_wakeup((gpio_num_t)pin, 0);
    esp_deep_sleep_start();

    // After waking the code continues
    // to execute from this point.
  }

  void reset() {
    // NVIC_SystemReset();
  }

  void delay(int ms) {
    vTaskDelay(ms);
  }

  uint32_t getID() {
    //The chip ID is essentially its MAC address(length: 6 bytes).
    uint64_t chipID = ESP.getEfuseMac();

    // print High 2 bytes
    Serial.printf("ESP32 Chip ID = %04X",(uint16_t)(chipID >> 32));
    // print Low 4 bytes.
    Serial.printf("%08X\n",(uint32_t)chipID);

    return (uint32_t)chipID;
  }
}

#endif
