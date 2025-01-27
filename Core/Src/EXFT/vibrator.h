#ifndef VIBRATOR_H
#define	VIBRATOR_H
#include <hwdrivers.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "timers.h"

#include "common.h"
#include "common.h"
#include "i2cwork.h"
#include "vlapconfig.h"
#include "vlapmain.h"


// G L O B A L   T Y P E S   A N D    D E F I N I T I O N S 
// VIBRATOR parameters. 

typedef enum {VIBRATOR_STATE_IDLE, VIBRATOR_STATE_ON_WAIT, VIBRATOR_STATE_OFF_WAIT} vibratorQueueEntryState_T;
typedef enum {VIBRATOR_OPCODE_TIMER, VIBRATOR_OPCODE_ACTIVATE} vibratorOpcodeQueue_T;
typedef enum {VIBRATOR_OPTION1 = 0, VIBRATOR_OPTION2 = 1, VIBRATOR_OPTION3 = 2, VIBRATOR_OPTION4 = 3, VIBRATOR_OPTION5 = 4, VIBRATOR_NURSE_MODE = 5} vibratorOption_T;

typedef struct 
{
  vibratorOpcodeQueue_T       opcode;
  uint8_t                     dutyCycle;
  uint16_t                    cycleTime;
  uint8_t                     numberOfCycles;  
} vibratorParamQueueEntryT;

// G L O B A L   T Y P E S   A N D    D E F I N I T I O N S  
static const vibratorParamQueueEntryT vibratorTypesTable[] = 
  {    
     //VIBRATOR_OPTION1
    {VIBRATOR_OPCODE_ACTIVATE, 50, 100, 2},
   
     //VIBRATOR_OPTION2
    {VIBRATOR_OPCODE_ACTIVATE, 100, 750, 1},
    
    //VIBRATOR_OPTION3
    {VIBRATOR_OPCODE_ACTIVATE, 50, 1000, 2},
    
    //VIBRATOR_OPTION4
    {VIBRATOR_OPCODE_ACTIVATE, 30, 500, 1},
    
    //VIBRATOR_OPTION5
    {VIBRATOR_OPCODE_ACTIVATE, 50, 100, 1},
    
    //VIBRATOR_NURSE_MODE
    {VIBRATOR_OPCODE_ACTIVATE, 50, 500, 1},    
  };

// E X T E R N A L S   
extern QueueHandle_t vibratorRequestQueueHandle;

// G L O B A L  P R O T O T Y P E S 
// G L O B A L S 

ReturnCode_T vibratorInit();
ReturnCode_T vibratorPatternSet(vibratorParamQueueEntryT parameters);
ReturnCode_T vibratorTaskEventSend(vibratorOpcodeQueue_T NotificationToVibratorTask);
void vibratorTimerTimeoutCallback();
void vibratorRequestToQueueAdd(vibratorOption_T vibratorOption_T);
#endif
