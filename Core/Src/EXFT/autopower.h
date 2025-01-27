#ifndef AUTOPOWER_H
#define	AUTOPOWER_H
#include "common.h"

// G L O B A L   T Y P E S   A N D    D E F I N I T I O N S 
#define  AUTOPOWER_RELAYS_EXCITATION_DELAY    2       // 2 Miliseconds 
#define  AUTOPOWER_BACK_TO_IDLE_CYCLES        5       // 5 Miliseconds 
#define  AUTOPOWER_CURRENT_MON_FACTOR         1
typedef enum {AUTOPOWER_BELT_STATUS_NOT_CONNECTED, AUTOPOWER_BELT_STATUS_OVERFOLDED,AUTOPOWER_BELT_STATUS_NOT_ON_BODY, AUTOPOWER_BELT_STATUS_CONNECTED} AutopowerBeltStatusT;
  
  
  
 
  
  
// G L O B A L  P R O T O T Y P E S 

ReturnCode_T autopowerInit();
ReturnCode_T autopowerPulseSkipLevelSet( uint8_t PulseSkipLevel);

// L O C A L    T Y P E S   A N D    D E F I N I T I O N S  
#define AUTOPOWER_MAX_RELAYS_VAL                    511
#define AUTOPOWER_MIN_RELAYS_VAL                    0
#define AUTOPOWER_DUCCESSIVE_CNT_THR_A              8
#define AUTOPOWER_DUCCESSIVE_CNT_THR_B              12
#define AUTOPOWER_DUCCESSIVE_CNT_THR_C              16
#define AUTOPOWER_MAX_PWM_POWER_VAL                 1000    
#define AUTOPOWER_MAX_PSL_STEPS       				180
#define AUTOPOWER_PSL_OFFSET						30

#pragma pack(1)
typedef    struct
{
  uint8_t       Capture;
  uint8_t       Period;
  uint16_t      Power;
} PwmLutEntry_T;



typedef  struct 
{
  uint32_t      Voltage;
  uint32_t      Current;
  uint16_t      Power;
  uint8_t       ByPassEnabled;
} autopowerRfDc2DcOutput_T;



// L O C A L    P R O T O T Y P E S
ReturnCode_T autopowerControl(uint8_t ControlByte);
ReturnCode_T autopowerManaualRelaysSet(uint8_t RelaysState);
ReturnCode_T autopowerTxCurrent1mAGet(uint16_t *ReturnedRealTimeCurrent1mAPtr);
ReturnCode_T autopowerTransmitterCurrentGoalSet(uint16_t NewTransmitterCurrentGoalValue);
ReturnCode_T autopowerTransmitterCurrentGoalGet(uint16_t *TransmitterCurrentGoalPtr);
ReturnCode_T autopowerPulseSkipLevelGet(uint8_t *ReturnedPulseSkipLevelPtr);
ReturnCode_T autopowerManualModePulseSkipLevelIncrease();
ReturnCode_T autopowerManualModePulseSkipLevelDecrease();
uint8_t      autopowerMaxLutIndexGet();
uint16_t     autopowerRealTimeXmtrCurrentGet();

ReturnCode_T autopowerEffectiveBeltResistanceGet( uint16_t *ReturnedResistance);
ReturnCode_T autopowerPwmTemporaryControl(uint8_t Control);
ReturnCode_T autopowerManaualPwmSet(uint8_t AutopowerLutNewIndex);
uint16_t autopowerBeltCurrentMilliAmpGet();
uint8_t  autopowerRfDc2DcFailureStateGet();

#endif
