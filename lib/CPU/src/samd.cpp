/*
    SAMD related functions
*/
#include "CPU.h"

#ifdef ARDUINO_SAMD_ZERO

namespace CPU {

  void sleep(int pin) {

    USBDevice.standby();

    delay(200);

    // Set sleep mode to deep sleep
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    //Enter sleep mode and wait for interrupt (WFI)
    __DSB();
    __WFI();

    // After waking the code continues
    // to execute from this point.

    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    USBDevice.attach();

  }

  void reset() {
    NVIC_SystemReset();
  }

  uint64_t combine(uint32_t low, uint32_t high) {
    return (((uint64_t) high) << 32) | ((uint64_t) low);
  }

  /*  Each SAMD device has a unique 128-bit serial number which is a concatenation
      of four 32-bit words contained at the following addresses:
      Word 0: 0x0080A00C Word 1: 0x0080A040
      Word 2: 0x0080A044 Word 3: 0x0080A048
   */
  uint32_t getID() {

    volatile uint32_t val1, val2, val3, val4;
    volatile uint32_t *ptr1 = (volatile uint32_t *)0x0080A00C;
    val1 = *ptr1;
    volatile uint32_t *ptr = (volatile uint32_t *)0x0080A040;
    val2 = *ptr;
    ptr++;
    val3 = *ptr;
    ptr++;
    val4 = *ptr;

    Serial.print("Chip ID: 0x");
    char buf[33];
    sprintf(buf, "%8x%8x%8x%8x", val1, val2, val3, val4);
    Serial.println(buf);

    // val2 and val 3 is not unique
    return val1;
  }

  void delay(int ms) {
    delay(ms);
  }

}

#endif
