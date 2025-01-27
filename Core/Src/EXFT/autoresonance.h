#ifndef AUTORESONANCE_H
#define	AUTORESONANCE_H
#include "common.h"

// G L O B A L   T Y P E S   A N D    D E F I N I T I O N S 
#define  AUTORESONANCE_RELAYS_EXCITATION_DELAY    10       // 2 Miliseconds 
#define  AUTORESONANCE_BACK_TO_IDLE_CYCLES        20       // 100 Miliseconds 
#define  AUTORESONANCE_2_SEGNEMTS_BELT_MIN_RELALY 1
#define  AUTORESONANCE_3_SEGNEMTS_BELT_MIN_RELALY 1
#define  AUTORESONANCE_4_SEGNEMTS_BELT_MIN_RELALY 1




// G L O B A L  T Y P E S 
typedef enum  { AUTORESONANCE_CONTROL_OFF, AUTORESONANCE_CONTROL_ON, AUTORESONANCE_CONTROL_DELAYED_ON }  AutoresonanceControl_T;

typedef enum  { AUTORESONANCE_INIT_REQ_AFTER_POWERUP, AUTORESONANCE_INIT_REQ_AFTER_TX_ON }  AutoresonanceInitRequest_T;



// G L O B A L  P R O T O T Y P E S 

void autoresonanceFsmGlue(void *v);
ReturnCode_T autoresonanceFsm(void);
uint16_t  autoresonanceRelaysCapaticanceGet();
uint16_t  autoresonanceRelaysStateGet();
uint8_t  autoresonanceMinimalRelaysStateGet();
ReturnCode_T  autoresonanceFrequencyCompareGet();
ReturnCode_T autoresonanceControl(AutoresonanceControl_T ControlByte);
ReturnCode_T autoresonanceInit(AutoresonanceInitRequest_T InitRequestType);
uint16_t  autoresonanceRelaysCapaticanceGet();
ReturnCode_T autoresonanceResetP2P();
uint16_t autoresonanceP2PGet();
void autoresonanceActivationTimerSchedule( uint16_t Time);
void autoresonanceActivationTimerInterruptCallback();

// L O C A L    T Y P E S   A N D    D E F I N I T I O N S  
typedef enum  { AUTORESONANCE_STATE_DELAYED_ON, AUTORESONANCE_STATE_IDLE, AUTORESONANCE_STATE_INCREMENTING, AUTORESONANCE_STATE_INCREMENTING_RELAYS_ACTIVATION, AUTORESONANCE_STATE_DECREMENTING, AUTORESONANCE_STATE_DECREMENTING_RELAYS_ACTIVATION, AUTORESONANCE_STATE_LAST, AUTORESONANCE_STATE_STABLE}  AutoresonanceState_T;
#define AUTORESONANCE_MAX_RELAYS_VAL                    255
#define AUTORESONANCE_DUCCESSIVE_CNT_THR_A              8
#define AUTORESONANCE_DUCCESSIVE_CNT_THR_B              12
#define AUTORESONANCE_DUCCESSIVE_CNT_THR_C              16
#define AUTORESONANCE_INITIAL_RELAY_STATE               1
// 2% of 6.780Mhz in Khz
#define AUTORESONANCE_ALLOWED_FREQUENCY_MEASURED_DELTA                     ((HSE_VALUE/4)*2/(100*1000))


// L O C A L    P R O T O T Y P E S
ReturnCode_T StateChange(AutoresonanceState_T NewState);
ReturnCode_T StepGearShift(AutoresonanceState_T State, uint16_t* ReturnedRelayStepPtr);
ReturnCode_T autoresonanceManaualRelaysSet(uint16_t RelaysState);
uint16_t  autoresonanceRelaysStateGet();
ReturnCode_T autoresonanceIsIdle();
ReturnCode_T autoresonanceUpdateP2P(uint16_t newValue);




#endif
