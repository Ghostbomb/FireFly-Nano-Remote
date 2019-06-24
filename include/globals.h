#ifndef _CONST_H
#define _CONST_H

#include <Arduino.h>
#include <datatypes.h>

// #define FAKE_UART // Comment out after pairing the remote and connecting VESC

// #define DEBUG // Uncomment DEBUG if you need to debug the remote
// #define DEBUG_OP //extra Debugging, basiccely showing everything its doing

#define REVISION_ID "$ID$"
#define FW_VERSION "development"
// #define RECEIVER_SCREEN
const COMM_PACKET_ID VESC_COMMAND = COMM_GET_VALUES; // VESC
// const COMM_PACKET_ID VESC_COMMAND = COMM_GET_UNITY_VALUES; // Enertion Unity

/*
  Endless ride - when remote is off and speed is over 12 km/h for 3 seconds,
  cruise control will be activated when speed drops below 12 km/h.

  Slide the board backwards while standing on it or foot brake
  to produce a spike in the current and stop the board.
*/

#define MilesSetup     //UNCOMMENT for Miles per hour. Then change number below to miles per hour if uncommented(defined)
#define InvertTrigger HIGH   //(LOW or HIGH) Change to LOW or HIGH if trigger is soldered wrong or menu/cruise control is acting strange

//Auto Cruise
const bool  AUTO_CRUISE_ON = false;     // disabled by default
const float C_PUSHING_SPEED = 12.0;       // Pushing Speed to initiate the Auto Cruise
const float PUSHING_TIME = 3.0;         // seconds
const float CRUISE_CURRENT_SPIKE = 5.0; // Amps

// boad will stop after 30s if current is low
const float AUTO_CRUISE_TIME = 30.0;    // seconds
const float CRUISE_CURRENT_LOW = 5.0;   // Amps

// auto stop if remote is off and speed is over 20 km/h
const float C_MAX_PUSHING_SPEED = 20.0;   // this should be in mph if MilesSetup is defined

// Auto stop (in seconds)
const float AUTO_BRAKE_TIME = 5.0;    // time to apply the full brakes
const int AUTO_BRAKE_RELEASE = 5;     // time to release brakes after the full stop

// UART
const int UART_SPEED = 115200;
const uint16_t uartPullInterval = 150;
const int UART_TIMEOUT = 10; // 10ms for 115200 bauds, 100ms for 9600 bauds
const int REMOTE_RX_TIMEOUT = 20; // ms

const int REMOTE_LOCK_TIMEOUT = 10; // seconds to lock throttle when idle
const int REMOTE_SLEEP_TIMEOUT = 180; // seconds to go to sleep mode

// turn off display if battery < 15%
const int DISPLAY_BATTERY_MIN = 0; //15 default. If 0 then make sure you MONITOR your battery

// VESC current, for graphs only | Take directly from VESC Tool
const int MOTOR_MIN = -45;
const int MOTOR_MAX = 45;
const int BATTERY_MIN = -12;
const int BATTERY_MAX = 20;

// default board configuration
const int C_MAX_SPEED = 25;       // Max Speed
const int C_MAX_RANGE = 10;       // MAX RANGE
const int BATTERY_CELLS = 10;
const float BATTERY_VOLTAGE_CUTOFF_START = 36;   // "Battery Voltage Cutoff Start" Should come directly from VESC Tool
const float BATTERY_VOLTAGE_CUTOFF_END   = 34;     // "Battery Voltage Cutoff End"   Should come directly from VESC Tool
const int BATTERY_TYPE = 1;     // 0: LI-ION | 1: LIPO
const int MOTOR_POLES = 14;
const int WHEEL_DIAMETER = 90;
const int WHEEL_PULLEY = 36;
const int MOTOR_PULLEY = 15;

//MPH calculations DON'T CHANGE
#ifdef MilesSetup
const float PUSHING_SPEED = (C_PUSHING_SPEED);
const float MAX_PUSHING_SPEED = (C_MAX_PUSHING_SPEED); 
const int MAX_SPEED = (C_MAX_SPEED);
const int MAX_RANGE = (C_MAX_RANGE);
const char DISTANCE_UNIT[3] = "mi";
const char ALT_DISTANCE_UNIT[3] = "mi";
const char SMALL_DISTANCE_UNIT[3] = "ft";
const char SPEED_UNIT[4] = "mph";
#endif

//KMH calculations DON'T CHANGE
#ifndef MilesSetup
const float PUSHING_SPEED = C_PUSHING_SPEED;
const float MAX_PUSHING_SPEED = C_MAX_PUSHING_SPEED;
const int MAX_SPEED = C_MAX_SPEED;
const int MAX_RANGE = C_MAX_RANGE;
const char DISTANCE_UNIT[3] = "km";
const char ALT_DISTANCE_UNIT[2] = "k";
const char SMALL_DISTANCE_UNIT[3] = "m";
const char SPEED_UNIT[5] = "km/h";
#endif

const float minCellVoltage = (BATTERY_VOLTAGE_CUTOFF_END/BATTERY_CELLS);

// MIN_BATTERY_VOLTAGE_START
// MIN_BATTERY_VOLTAGE_END

#define VERSION 2

// Remote > receiver
struct RemotePacket {
  uint32_t address;
  // --------------  // keep 4 byte alignment!
  uint8_t  version;  // 1
  uint8_t  command;	 // Throttle | Light | Settings
  uint8_t  data;     // e.g. throttle value
  uint8_t  counter;
  // --------------
};

// commands
const uint8_t SET_THROTTLE  = 1;
const uint8_t SET_CRUISE    = 2;

const uint8_t GET_CONFIG    = 3;
const uint8_t SET_STATE     = 4;


// state machine
enum AppState {
  IDLE,       // remote is not connected //state 0
  NORMAL,     //state 1
  PUSHING,
  CRUISE,
  ENDLESS,
  CONNECTED,  // riding with connected remote
  CONNECTING,
  MENU,
  STOPPING,   // emergency brake when remote has disconnected
  STOPPED,
  PAIRING,
  UPDATE,     // update over WiFi
  COASTING    // waiting for board to slowdown
};

// Receiver > remote  3 bytes
struct ReceiverPacket {
  uint8_t type;
  uint8_t chain;	// CRC from RemotePacket
  uint8_t state;   // Mode: Pairing, BT, ...
  uint8_t r2;
};

// responses type
const uint8_t ACK_ONLY  = 1;
const uint8_t TELEMETRY = 2;
const uint8_t CONFIG    = 3;
const uint8_t BOARD_ID  = 4;

struct InfoPacket {
  ReceiverPacket header;
  // --------------  // keep 4 byte alignment!
  int32_t id;
  // --------------
  uint8_t r0;
  uint8_t r1;
  uint16_t r2;
  // --------------
  uint16_t r3;
  uint16_t r4;
  // --------------
};

const int PACKET_SIZE = sizeof(InfoPacket);
const int CRC_SIZE = 1;

// New VESC values 12 + 3
struct TelemetryPacket {
  ReceiverPacket header;
  // -----------------  // keep 4 byte alignment!
  uint16_t speed;       // km/h * 100
  uint8_t tempMotor;
  uint8_t tempFET;
  // -----------------
  uint16_t voltage;     // volts * 100
  uint16_t distance;    // km * 100 - 10m accuracy
  // -----------------
  int16_t motorCurrent; // motor amps * 100
  int16_t inputCurrent; // battery amps * 100
  // -----------------

  uint16_t f2w(float f) { return f * 100; } // pack float
  float w2f(uint16_t w) { return float(w) / 100; }; // unpack float

  int16_t f2wi(float f) { return f * 100; } // pack float
  float w2fi(int16_t w) { return float(w) / 100; }; // unpack float

  #ifdef MilesSetup
  float getSpeed() { return (w2f(speed)); }
  void setSpeed(float f) { speed = (f2w(f)*0.621371); }
  #endif
  #ifndef MilesSetup
  float getSpeed() { return w2f(speed); }
  void setSpeed(float f) { speed = f2w(f); }
  #endif

  float getVoltage() { return w2f(voltage); }
  void setVoltage(float f) { voltage = f2w(f); }

  #ifdef MilesSetup
  float getDistance() { return (w2f(distance)); }
  void setDistance(float f) { distance = (f2w(f)*0.621371); }
  #endif
  #ifndef MilesSetup
  float getDistance() { return w2f(distance); }
  void setDistance(float f) { distance = f2w(f); }
  #endif
  float getMotorCurrent() { return w2fi(motorCurrent); }
  void setMotorCurrent(float f) { motorCurrent = f2wi(f); }

  float getInputCurrent() { return w2fi(inputCurrent); }
  void setInputCurrent(float f) { inputCurrent = f2wi(f); }
};

// board setting
struct ConfigPacket {
  ReceiverPacket header;
  // -------------------  // keep 4 byte alignment!
  uint8_t  maxSpeed;	    // m/s
  uint8_t  maxRange;      // km
  uint8_t  batteryCells;
  uint8_t  batteryType;   // 0: Li-ion | 1: LiPo
  // -------------------
  uint8_t  motorPoles;
  uint8_t  wheelDiameter;
  uint8_t  wheelPulley;
  uint8_t  motorPulley;
  // -------------------
  int16_t r1;  // battery amps * 100
  int16_t r2;
  // -------------------
  float getMaxSpeed() { return (maxSpeed) / 100; }
  void setMaxSpeed(float f) { maxSpeed = f * 100; }
};

const int default_throttle = 127;

#ifdef DEBUG
  #define debug(x) Serial.println (x)
#else
  #define debug(x)
#endif
#ifdef DEBUG_OP
  #define debug_E(x) Serial.println (x)
#else
  #define debug_E(x)
#endif


#endif
