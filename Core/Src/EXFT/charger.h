#pragma once

#include <hwdrivers.h>
#include "common.h"
#include "protocolApp.h"
#include "inet.h"
#include "rtc.h"
#include "vmicapplayer.h"
#include "pccommAppLayer.h"
#include "Leds.h"
#include "measure.h"

// G L O B A L   T Y P E S   A N D    D E F I N I T I O N S
typedef enum {
	CHARGER_CONTROL_DISABLE, CHARGER_CONTROL_ENABLE
} ChargerControl_T;
typedef enum {
	CHARGER_STATUS_DISABLED, CHARGER_STATUS_ENABLED
} ChargerEnableStatus_T;
typedef enum {
	CHARGER_INDICATION_SHOW_OFF = 0,
	CHARGER_INDICATION_SHOW_ON = 1,
	CHARGER_INDICATION_SHOW_OFF_WAIT = 2
} chargerShowIndicationState_T;
typedef enum {
	CHARGER_DC_PLUG_DETACHED = 0,
	CHARGER_DC_PLUG_INSERTED = 1,
	CHARGER_DC_PLUG_INSERTED_WITH_POLLING = 2,
	CHARGER_DC_PLUG_BETWEEN_THRESHOLD = 3
} chargerDcPlugStatus_T;
typedef enum {
	CHARGER_PLUG_DISCONNECTED,
	CHARGER_PLUG_CONNECTED,
	CHARGER_PLUG_CONNECTED_SENT,
	CHARGER_PLUG_DISCONNECTED_EVENT
} chargerPlugState_T;
typedef enum {
	CHARGER_INDICATION_NOT_CHARGING,
	CHARGER_INDICATION_CHARGING,
	CHARGER_INDICATION_POLLING_DISABLE
} chargerIndicationState_T;
typedef enum {
	CHARGER_REQ_INDICATION_OFF = 0, CHARGER_REQ_INDICATION_ON = 1
} chargerReq_T;
typedef enum {
	CHARGER_NOT_CHARGING, CHARGER_CHARGING
} chargerChargingState_T;
typedef enum {
	CHARGER_CONFIG_STATE0,
	CHARGER_CONFIG_STATE1,
	CHARGER_CONFIG_STATE2,
	CHARGER_CONFIG_STATE3
} chargerConfigStateT;
typedef enum {
	CHARGER_REG_VBUS_L = 0x1C,
	CHARGER_REG_VBUS_H = 0x1B,
	CHARGER_REG_CHG_STATUS2 = 0x0C,
	CHARGER_REG_CHG_STATUS1 = 0x0B,
	CHARGER_REG_VBAT_H = 0x1D,
	CHARGER_REG_VBAT_L = 0x1E,
	CHARGER_REG_FAULT = 0x0E,
	CHARGER_REG_NTC = 0x0D
} chargerRegId_t;
typedef enum {
	CHARGER_FAULT_NONE,
	CHARGER_FAULT_DETECTED,
	CHARGER_FAULT_PRESENT,
	CHARGER_FAULT_CLEARED
} chargerFaultState_T;

typedef enum {
	CHARGER_BQ_NOT_CHARGING,
	CHARGER_BQ_TRICKLE,
	CHARGER_BQ_PRE_CHARGE,
	CHARGER_BQ_FAST_CHARGE,
	CHARGER_BQ_TAPER_CHARGE,
	CHARGER_BQ_TOPOFF,
	CHARGER_BQ_DONE_CHARGE
} chargerBQChargeStatus_T;
typedef enum {
	CHARGER_BQ_PLUG_NOINPUT,
	CHARGER_BQ_PLUG_SDP,
	CHARGER_BQ_PLUG_CDP,
	CHARGER_BQ_PLUG_DCP,
	CHARGER_BQ_PLUG_POORSRC,
	CHARGER_BQ_PLUG_UNKNOWN,
	CHARGER_BQ_PLUG_NONSTD,
	CHARGER_BQ_PLUG_OTG
} chargerBQPlugStatus_T;
#define CHARGER_DC_PLUG_DETECTION_FILTER_THRESHOLD /*1*/ 100
#define CHARGER_MIN_VOLTAGE 2900
#define CHARGER_VOLTAGE_HYSTERESIS 150
#define CHARGER_CHANGE_LED_INDICATION_THRESHOLD 50
#define CHARGER_TURN_INDICATION_OFF_THRESHOLD 10
#define CHARGER_PB_ABORT_SHOW_INDICATION_THRESHOLD 200

// L O C A L    T Y P E S   A N D    D E F I N I T I O N S

#define CHARGER_BQ25882_CHARGER_I2C_ADDRESS_LOW 0xD6 // SAD[0] = 0
#define CHARGER_BQ25882_VOLT_MSB_BITVAL 256          // 256mV per bit
#define CHARGER_BQ25882_VOLT_LSB_BITVAL 1            // 1mV per bit

// #define PS_CONNECTED

extern bool chargerFinishedUpdating;

// G L O B A L  P R O T O T Y P E S
ReturnCode_T chargerInit(void);
ReturnCode_T chargerControl(ChargerControl_T ChargerControl);
chargerChargingState_T chargerChargingStatusGet();
chargerDcPlugStatus_T chargerDcPlugStatusGet();
ReturnCode_T chargerDcPlugStateProcessing();
ReturnCode_T chargerProcessing();
chargerShowIndicationState_T chargerShowIndicationStateGet();
ReturnCode_T chargerRequestSend(chargerReq_T chragerRequest);
ReturnCode_T chargerLedsLevelRefresh();
ReturnCode_T chargerShowIndication();
void chargerRefreshUSBInit();
chargerPlugState_T chargerPlugStateGet();
uint8_t chargerRegGet(chargerRegId_t regId);
chargerBQChargeStatus_T chargerChargeStatusRegGet();
uint16_t chargerUSBVoltageGet();
bool chargerFaultFlagDetect();
void chargerFaultIndicationCheck();
ReturnCode_T chargerReqSchedule();
ReturnCode_T chargerReqCallBack(uint8_t LedIndex,
		i2cworkSchedulerFunctionParameter_T *ReturnedCommandPtr);
ReturnCode_T chargerPostCallBack(uint8_t LedIndex,
		i2cworkSchedulerFunctionParameter_T *PostFunctionPtr);

chargerBQPlugStatus_T chargerBQPlugDetectionStatusGet();
void chargerResetFirstMeasureFlag();
bool chargerGetFirstMeasureFlag();
