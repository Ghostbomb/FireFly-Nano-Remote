#include <Arduino.h>
#include <datatypes.h>

// To compile for FOCBOX Unity, uncomment the following line.
// #define FOCBOX_UNITY

/*
  Different Looks for the Main page of the remote. Define which version you want by uncommenting or commenting out the following
  Go here to see differences: 
*/
// #define MAINPAGE_LITE

// #define PWM_MODE

// - Choose frequency:
// #define RFM_EU                    // RFM_EU for 415Mhz in Europe
#define RFM_USA                 // RFM_USA for 915Mhz in USA and AUS


// UNCOMMENT for Miles per hour. Then change number below to miles per hour if uncommented(defined)
#define MilesSetup

// Set to  true invert the deadman switch just in case it was soldered on wrong
#define InvertTrigger true
// #define POLICE_MODE        //TODO:
#define VOLTAGE_DIVIDER

/*
  Endless ride - when remote is off and speed is over 12 km/h for 3 seconds,
  cruise control will be activated when speed drops below 12 km/h.

  Slide the board backwards while standing on it or foot brake
  to produce a spike in the current and stop the board.
*/

//Auto Cruise
const bool AUTO_CRUISE_ON = false;      // disabled by default
const float C_PUSHING_SPEED = 7.0;      // Pushing Speed to initiate the Auto Cruise
const float PUSHING_TIME = 3.0;         // seconds
const float CRUISE_CURRENT_SPIKE = 5.0; // Amps

// board will stop after 30s if current is low
const float AUTO_CRUISE_TIME = 30.0;  // seconds
const float CRUISE_CURRENT_LOW = 5.0; // Amps

// auto stop if remote is off and speed is over 20 km/h
const float C_MAX_PUSHING_SPEED = 20.0; // this should be in mph if MilesSetup is defined

// Auto stop (in seconds)
const float AUTO_BRAKE_TIME = 5.0; // time to apply the full brakes
const int AUTO_BRAKE_RELEASE = 5;  // time to release brakes after the full stop

// UART
const int UART_SPEED = 115200;
const uint16_t uartPullInterval = 150;
const int UART_TIMEOUT = 10;          // 10ms for 115200 bauds, 100ms for 9600 bauds
const int REMOTE_RX_TIMEOUT = 20;     // ms
const int MIN_SPEED_FOR_MENU = 2;     //affects at what speed the remote is going to vibrate like mad and what speed you can use the menu.

const int REMOTE_LOCK_TIMEOUT = 10;   // seconds to lock throttle when idle
const int REMOTE_SLEEP_TIMEOUT = 180; // seconds to go to sleep mode

const float VOLTAGE_MULTIPLIER = 0.001388;

// turn off display if battery < 15%
const int DISPLAY_BATTERY_MIN = 15; //15 default. If 0 then make sure you MONITOR your battery

// VESC current, for graphs only | Take directly from VESC Tool
const int MOTOR_MIN = -60;
const int MOTOR_MAX = 60;
const int BATTERY_MIN = -17;
const int BATTERY_MAX = 30;

// default board configuration
const int C_MAX_SPEED = 20;                    // Max Speed
const int C_MAX_RANGE = 15;                    // MAX RANGE
const int BATTERY_CELLS = 5;
const float BATTERY_VOLTAGE_CUTOFF_START = 18; // "Battery Voltage Cutoff Start" Should come directly from VESC Tool
const float BATTERY_VOLTAGE_CUTOFF_END = 17;   // "Battery Voltage Cutoff End"   Should come directly from VESC Tool
const int BATTERY_TYPE = 1;                    // 0: LI-ION | 1: LIPO
const int MOTOR_POLES = 14;
const int WHEEL_DIAMETER = 90;
const int WHEEL_PULLEY = 36;
const int MOTOR_PULLEY = 15;