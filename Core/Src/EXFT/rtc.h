#ifndef RTC_H
#define RTC_H

#include <hwdrivers.h>
#include "stm32h7xx.h"
#include <time.h>


typedef enum  {RTC_WAKEUP_REASON_IDLE=0, RTC_WAKEUP_REASON_TIMED_WAKEUP = 1, RTC_WAKEUP_REASON_ALARM_A = 2, RTC_WAKEUP_REASON_ALARM_B = 3,  RTC_WAKEUP_REASON_PUSH_BUTTON=4, RTC_WAKEUP_REASON_CHARGER_EVENT = 5, RTC_WAKEUP_REASON_UNKNOWN=7} rtcWakeupReasonT;


typedef struct{
uint32_t LastExceptionEpochTime;
uint32_t Crc32;
} rtcLastExceptionEpoch_T;



int rtcInit();
void rtcTimeDateUpdate();
ReturnCode_T rtcEpochTimeStampGet( uint32_t* ReturnedEpochPtr, uint8_t* Returned10mSecCountPtr);
void rtcResetDateTime();
rtcWakeupReasonT rtcSleepStart();


void rtcEpochTimeAndDateSet(time_t now);
uint32_t rtcEpochGet();
uint8_t rtcUsbPluggedStateGet();
uint8_t rtcSubSecondGet();
rtcWakeupReasonT rtcWakeupReasonGet();
void rtcDateTimeConsolePrint();
extern bool PBDebounceSample;
#endif
