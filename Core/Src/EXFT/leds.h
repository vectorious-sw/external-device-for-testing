#pragma once

#include <hwdrivers.h>
#include "protocolApp.h"
#include "inet.h"
#include "rtc.h"
#include "vmicapplayer.h"
#include "i2cwork.h"
#include "timers.h"
#include "common.h"

#define LEDS_DELAY_TIME_FOR_INIT 500
#define LEDS_ORANGE_GREEN_PWM 94

// G L O B A L   T Y P E S   A N D    D E F I N I T I O N S 

typedef enum  { // MSB->(G_PWM | B | G | R)<-LSB
  LEDS_COLOR_OFF = 0, 
  LEDS_COLOR_WHITE = 7, 
  LEDS_COLOR_RED = 1, 
  LEDS_COLOR_GREEN = 2, 
  LEDS_COLOR_BLUE = 4,
  LEDS_COLOR_YELLOW = 3,
  LEDS_COLOR_MAGENTA = 5,
  LEDS_COLOR_CYAN = 6,
  LEDS_COLOR_ORANGE = 11 // Same as YELLOW but green is PWM 
} LedsColor_T;

typedef enum 
{
  LEDS_ON,
  LEDS_FAULT,
  LEDS_OFF
} LedsIndicationState_T;

typedef enum  {
  /********MAIN PATTERNS/********/
  LEDS_MAIN_PATTERN_IDLE, 
  LEDS_MAIN_PATTERN_WAKEUP,
  LEDS_MAIN_PATTERN_PROGRESS_SLOW,
  LEDS_MAIN_PATTERN_PROGRESS_RAPID, 
  LEDS_MAIN_PATTERN_SUCCESS,
  LEDS_MAIN_PATTERN_FAIL,
  LEDS_MAIN_PATTERN_MEAS_START,
  LEDS_MAIN_PATTERN_FULLBATT,
  LEDS_MAIN_PATTERN_LOWBATT,
  LEDS_MAIN_PATTERN_CRITBATT,
  LEDS_MAIN_PATTERN_MEAS_CHARGING,
  /********PORT PATTERNS********/
  LEDS_PORT_PATTERN_IDLE,
  LEDS_PORT_PATTERN_FULL,
  LEDS_PORT_PATTERN_CHARGING,
  LEDS_PORT_PATTERN_FAULT,
} LedsPatterns_T;

typedef enum {
  LEDS_TARGET_MAIN,
  LEDS_TARGET_PORT
} LedTarget_T;

#pragma pack(1)
typedef struct
{
  HwdriversGpiosT Red;
  HwdriversGpiosT Green;
  HwdriversGpiosT Blue;
} LedsHW_T;

#pragma pack(1)
typedef struct
{
  LedsColor_T Color;
  uint16_t NumberOfCycles;
  uint16_t CycleTime; //1ms ticks
  uint8_t DutyCycle; // 0-100% of CycleTime
  LedTarget_T LedTarget; //Which led to change - main or plug
}LedsCycle_T;

static const LedsCycle_T ledsPatternTable[] =  // (Color, NumberOfCycles, CycleTime, DutyCycle, LedTarget)
{
  /********************** MAIN LEDS **********************/
  {LEDS_COLOR_OFF, 1, 100, 50, LEDS_TARGET_MAIN}, //LEDS_MAIN_PATTERN_IDLE
  {LEDS_COLOR_BLUE, 1, 6000, 50, LEDS_TARGET_MAIN}, //LEDS_MAIN_PATTERN_WAKEUP
  {LEDS_COLOR_GREEN, 999, 2000, 50, LEDS_TARGET_MAIN}, //LEDS_MAIN_PATTERN_PROGRESS_SLOW
  {LEDS_COLOR_GREEN, 999, 1000, 50, LEDS_TARGET_MAIN}, //LEDS_MAIN_PATTERN_PROGRESS_RAPID
  {LEDS_COLOR_GREEN, 1, 6000, 50, LEDS_TARGET_MAIN}, //LEDS_MAIN_PATTERN_SUCCESS
  {LEDS_COLOR_RED, 7, 600, 50, LEDS_TARGET_MAIN}, //LEDS_MAIN_PATTERN_FAIL
  {LEDS_COLOR_GREEN, 2, 100, 50, LEDS_TARGET_MAIN}, //  LEDS_MAIN_PATTERN_MEAS_START
  {LEDS_COLOR_GREEN, 1, 6000, 50, LEDS_TARGET_MAIN}, // LEDS_MAIN_PATTERN_FULLBATT
  {LEDS_COLOR_ORANGE, 1, 6000, 50, LEDS_TARGET_MAIN}, // LEDS_MAIN_PATTERN_LOWBATT
  {LEDS_COLOR_RED, 1, 6000, 50, LEDS_TARGET_MAIN}, // LEDS_MAIN_PATTERN_CRITBATT
  {LEDS_COLOR_RED, 1, 200, 50, LEDS_TARGET_MAIN}, //LEDS_MAIN_PATTERN_MEAS_CHARGING
  /********************** USB PORT LEDS **********************/
  {LEDS_COLOR_OFF, 1, 1, 100, LEDS_TARGET_PORT}, //LEDS_PORT_PATTERN_IDLE
  {LEDS_COLOR_GREEN, 1, 6000, 50, LEDS_TARGET_PORT}, //LEDS_PORT_PATTERN_FULL
  {LEDS_COLOR_BLUE, 1, 6000, 50, LEDS_TARGET_PORT}, //LEDS_PORT_PATTERN_CHARGING
  {LEDS_COLOR_RED, 1, 1, 100, LEDS_TARGET_PORT} //LEDS_PORT_PATTERN_FAULT
};

static const LedsHW_T ledsHWTargetTable[] = // (Red, Green, Blue)
{
  {HWDRIVERS_MAIN_RED_LED, HWDRIVERS_MAIN_GREEN_LED, HWDRIVERS_MAIN_BLUE_LED}, // LEDS_TARGET_MAIN
  {HWDRIVERS_USB_RED_LED, HWDRIVERS_USB_GREEN_LED, HWDRIVERS_USB_BLUE_LED} // LEDS_TARGET_PORT
};



// G L O B A L  P R O T O T Y P E S 
ReturnCode_T ledsInit(void);
void ledsMainPatternTimerTimeoutCallback();
void ledsPortPatternTimerTimeoutCallback();
ReturnCode_T ledsPatternSet(LedsPatterns_T pattern);
ReturnCode_T ledsColorSet(LedsColor_T color, LedTarget_T target);
LedsIndicationState_T ledsIndicationStateGet();
