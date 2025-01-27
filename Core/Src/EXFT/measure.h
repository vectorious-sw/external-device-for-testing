#pragma once

#include <hwdrivers.h>
#include "protocolApp.h"
#include "inet.h"
#include "rtc.h"
#include "vmicapplayer.h"
#include "generic.h"
#include "measure.h"
#include "audit.h"
#include "math.h"

#include "common.h"
#include "sensor.h"


// G L O B A L   T Y P E S   A N D    D E F I N I T I O N S 
typedef enum{
  MEASURE_PB_IDLE, MEASURE_PB_START,
  MEASURE_END_OF_MEASUREMENT,
  MEASURE_TIME_EVENT, 
  MEASURE_FSM_QUEUE_ENTRY_TYPE_TIMEOUT, 
  MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_SAMPLE,
  MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_STATUS_SOURCE_0,
  MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_STATUS_SOURCE_1
} MeasureReq_T;

typedef enum{ 
MEASURE_IDLE                                                    = 0, 
MEASURE_PUSH_BUTTON_BOUNCING_DELAY                              = 1,
MEASURE_CHECK_PLUG_IN_NURSE_MODE                                = 2,
MEASURE_NURSE_MODE_STATE                                        = 3,
MEASURE_NURSE_MODE_END_WAIT                                     = 4,
MEASURE_CHECK_USB_CONNECTED                                     = 5,
MEASURE_CHECK_BELT_AND_HUMAN_STATUS                             = 6,
MEASURE_DELAY_BEFORE_START                                      = 30,
MEASURE_MINIMAL_ENERGY_SEARCH                                   = 27,
MEASURE_MINIMAL_ENERGY_WAIT                                     = 7, 
MEASURE_MINIMAL_ENERGY_AUTO_RESONANCE_WAIT                      = 8, 
MEASURE_MINIMAL_ENERGY_IC127_SOURCE_SELECT_SAMPLE_WAIT          = 28, 
MEASURE_MINIMAL_ENERGY_IC127_SOURCE_SELECT_COMPLETED_WAIT       = 9, 
MEASURE_MINIMAL_ENERGY_T127_SOURCE_SELECT_SAMPLE_WAIT           = 29,
MEASURE_MINIMAL_ENERGY_T127_SOURCE_SELECT_COMPLETED_WAIT        = 10,
MEASURE_VMIC_ID_LSB_WAIT                                        = 11, 
MEASURE_VMIC_ID_MSB_WAIT                                        = 12, 
MEASURE_VMIC_INTERNAL_CAP_1_MEASURE_WAIT                        = 13,
MEASURE_VMIC_INTERNAL_CAP_2_MEASURE_WAIT                        = 14,
MEASURE_VMIC_INTERNAL_CAP_3_MEASURE_WAIT                        = 15,
MEASURE_REFERENCE_1_WAIT                                        = 16,
MEASURE_TEMPERATURE_1_MEASUREMENT_WAIT                          = 17,
MEASURE_TEMPERATURE_2_MEASUREMENT_WAIT                          = 18,
MEASURE_REFERENCE_2_WAIT                                        = 19,
MEASURE_VMIC_INTERNAL_CAP_4_MEASURE_WAIT                        = 20,
MEASURE_VMIC_INTERNAL_CAP_5_MEASURE_WAIT                        = 21,
MEASURE_VMIC_INTERNAL_CAP_6_MEASURE_WAIT                        = 22,
MEASURE_VMIC_REAL_MEMS_WAIT                                     = 23,
MEASURE_PUBLISH_RESULTS_WAIT                                    = 24,
MEASURE_END_TEST_WAIT                                           = 25,
MEASURE_DEBUG_FOR_NURSE_MODE                                    = 26,
MEASURE_AUTORESONANCE_INITIAL_CONVERGE_WAIT                     = 31

} MeasureFsmState_T;

typedef enum{MEASURE_TASK_QUEUE_SEND_SOURCE_NORMAL, MEASURE_TASK_QUEUE_SEND_SOURCE_ISR} MeasureTaskQueueSourceT;

typedef enum{MEASURE_STABILITY_FSM_RETURNED_STATUS_IDLE, MEASURE_STABILITY_FSM_RETURNED_STATUS_INPROGRESS, MEASURE_STABILITY_FSM_RETURNED_STATUS_STABLE, MEASURE_STABILITY_FSM_RETURNED_STATUS_FAILED} MeasureStabilityFsmReturnedStatusT;

typedef enum{MEASURE_MINIMAL_ENERGY_IDLE, MEASURE_MINIMAL_ENERGY_LOWER_SCALE_INCREASE, MEASURE_MINIMAL_ENERGY_LOWER_SCALE_DECREASE, MEASURE_MINIMAL_ENERGY_HIGHER_SCALE_INCREASE, MEASURE_MINIMAL_ENERGY_HIGHER_SCALE_DECREASE, MEASURE_MINIMAL_ENERGY_LOW_ENERGY, MEASURE_MINIMAL_ENERGY_SEARCH_DONE, MEASURE_MINIMAL_ENERGY_FAILED} MeasureMinimalEnergyFsmStateT;

typedef enum{MEASURE_POWER_LEVEL0 = 0, MEASURE_POWER_LEVEL1 = 1, MEASURE_POWER_LEVEL2 = 2, MEASURE_POWER_LEVEL3 = 3, MEASURE_POWER_LEVEL4 = 4, MEASURE_POWER_LEVEL5 = 5} MeasurePowerLevelT; 

typedef enum{MEASURE_FAILURE_NONE = 0, MEASURE_FAILURE_SHORT_PRESS = 1, MEASURE_FAILURE_OVER_TEMPERATURE = 2, MEASURE_FAILURE_LOW_BATTERY = 3, MEASURE_FAILURE_CHARGING_PLUG_CONNECTED = 4, MEASURE_FAILURE_BELT_OPEN = 5, MEASURE_FAILURE_MAX_PSL_REACHED = 6, MEASURE_FAILURE_STABILITY_P2P_MAX_ITERATIONS_REACHED = 7, MEASURE_FAILURE_STABILITY_AVERAGE_MAX_ITERATIONS_REACHED = 8, MEASURE_FAILURE_IMPLANT_ID_ZERO = 9, MEASURE_FAILURE_SWITCH_SOURCE = 10, MEASURE_FAILURE_TOTAL_TIMEOUT = 11, MEASURE_FAILURE_STATE_TIMEOUT = 12 } MeasureFailureReasonT;

#define MEASURE_MIN_PSK_LEVEL                                     20


// L O C A L    T Y P E S   A N D    D E F I N I T I O N S  
#pragma pack(1)
typedef  struct
{
  MeasureReq_T MeasureRequest; 
  uint8_t       ParamState;
  uint8_t       FreeTxBufferFlag;
} measureReqQueueEntry_T;

#pragma pack(1)
typedef  struct
{
  uint32_t Ic1;
  uint32_t Ref1;
  uint32_t T1;
  uint32_t T2;
  uint32_t Ref2;
  uint32_t Ic2;
} MeasurementStructT;




#define         MEASURE_FSM_TIMEOUT_SET(QueueWaitTimeoutIn1mSec)                  \
                {                                                                               \
                  xTimerChangePeriod( MeasureFsmTimerHandler, QueueWaitTimeoutIn1mSec, 100);        \
                }                                                                               \


// G L O B A L  P R O T O T Y P E S 
ReturnCode_T measureInit(void);
portTASK_FUNCTION(measureTask, pvParameters );
ReturnCode_T measureTaskEventSend(MeasureReq_T NotificationToCommTask , MeasureTaskQueueSourceT IsrOrNonIsrSource);
void measureEndOfSourceSelectionTxCommandCallback();
void measureStabilityFsmInit();
MeasureStabilityFsmReturnedStatusT  measureStabilityFsm();
uint32_t measurementSerialIdGet();
ReturnCode_T buzzAndVibrateForFail();
MeasureFsmState_T measurementStateGet();
ReturnCode_T measureNurseModeIndication(uint32_t peakToPeakSignalMeasure, uint32_t peakToPeakMeasure);
MeasurePowerLevelT measureConvertP2PtoLevel(uint32_t peakToPeakSignalMeasure, uint32_t peakToPeakMeasure);
uint32_t measureCalcBuzzerFreqByParam(uint32_t param);
uint8_t measureMeasurementInProgressGet();
ReturnCode_T minimalEnergyMeasureIncreaseLimit(uint8_t increaseNum);
ReturnCode_T measureIncreaseAndSetPsl(uint8_t increaseNum);
ReturnCode_T measureDecreaseAndSetPsl(uint8_t decreaseNum);
ReturnCode_T measureModemStateTurnOff();
void measureTimeoutTimerCallback();
ReturnCode_T measureInitializeAccelParams();
void measureMinimalEnergyFsmInit();
ReturnCode_T measureProgressLedsSequence(MeasureFsmState_T newState);

MeasureMinimalEnergyFsmStateT measureMinimalEnergyFsm(uint32_t peakToPeakSignalMeasure);

