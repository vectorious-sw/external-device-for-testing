#pragma once

#include <hwdrivers.h>
#include "common.h"
#include "protocolApp.h"
#include "inet.h"
#include "rtc.h"
#include "vmicapplayer.h"

// G L O B A L   T Y P E S   A N D    D E F I N I T I O N S 

//Temperature is in format : 80.00 celsius.
#define MAX_TEMPERATURE 8000
#define USB_MIN_VOLTAGE 4800
#define AUDIT_BELT_HUMAN_CHECK_TRIES 100


typedef enum { 
  AUDIT_BV_OK,
  AUDIT_BV_CROSSED_UB,
  AUDIT_BV_CROSSED_UB_SENDED,
  AUDIT_BV_CROSSED_LB,
  AUDIT_BV_CROSSED_LB_SENDED
} auditBvState_T;

 auditBvState_T auditBatteryStatusGet();

 typedef enum { 
  AUDIT_TEMPERATURE_NORMAL,
  AUDIT_TEMPERATURE_HIGH,
} auditTemperatureState_T;


typedef enum{AUDIT_NURSE_BUFFER_FIRST_BYTE, AUDIT_NURSE_BUFFER_REST_BYTES} AuditNurseModeBufferState_T;

typedef enum{AUDIT_NURSE_MODE_ENABLED, AUDIT_NURSE_MODE_DISABLED} AuditNurseModeStatus_T;

typedef enum  {AUDIT_BATTERY_RED_OFF = 0, AUDIT_BATTERY_RED_ON = 1} AuditBatteryLedState_T; 

typedef enum {AUDIT_BELT_HUMAN_CHECK_IDLE ,AUDIT_BELT_OPEN_HYS, AUDIT_BELT_OPEN, AUDIT_NO_HUMAN_HYS, AUDIT_BELT_NO_HUMAN, AUDIT_BELT_HUMAN_OK_HYS, AUDIT_BELT_HUMAN_OK} AuditBeltHumanCheck_T;

// L O C A L    T Y P E S   A N D    D E F I N I T I O N S  

// G L O B A L  P R O T O T Y P E S 
void auditInit(void);
auditTemperatureState_T auditTxTemperatureOverheatCheck();
void auditPowerTelemetryUpdate();
ReturnCode_T auditAmbiantTemperatueProcessing();
ReturnCode_T auditBatteryLevelCheck();
ReturnCode_T auditBeltHumanStatusProcessing();
auditTemperatureState_T auditAbiantTemperatureStateGet();
AuditNurseModeStatus_T auditNurseModeStatusGet();
AuditBeltHumanCheck_T auditBeltHumanStatusGet();
uint32_t auditNurseModeLastPluggedOutTimeGet();
uint32_t auditNurseModeLastPluggedInTimeGet();
ReturnCode_T auditManageBufferNurseMode(uint8_t newByte);
uint8_t  auditUsbVoltaGet();

