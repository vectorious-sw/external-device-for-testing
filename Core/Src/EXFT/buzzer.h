#ifndef BUZZER_H
#define	BUZZER_H
#include "FreeRTOS.h"
#include "queue.h"
#include "timers.h"

#include "common.h"
#include "common.h"
#include "i2cwork.h"
#include "vlapconfig.h"
#include "vlapmain.h"


// G L O B A L   T Y P E S   A N D    D E F I N I T I O N S 
// Buzzer parameters. 

typedef enum {BUZZER_STATE_IDLE, BUZZER_STATE_ON_WAIT, BUZZER_STATE_OFF_WAIT, BUZZER_STATE_ON, BUZZER_STATE_OFF} buzzerQueueEntryState_T;
typedef enum {BUZZER_OPCODE_TIMER, BUZZER_OPCODE_ACTIVATE} buzzerOpcodeQueue_T;
typedef enum {BUZZER_BEEPING_OPTION1 = 0, BUZZER_BEEPING_OPTION2 = 1, BUZZER_BEEPING_OPTION3 = 2, BUZZER_BEEPING_OPTION4 = 3, BUZZER_BEEPING_ON_POWER_UP = 4 } buzzerBeepingOption_T;

typedef struct 
{
  buzzerOpcodeQueue_T         opcode;
  uint32_t                    frequencyHz;
  uint8_t                     dutyCycle;
  uint16_t                    cycleTime;
  uint8_t                     numberOfCycles;  
} buzzerParamQueueEntryT;

// G L O B A L   T Y P E S   A N D    D E F I N I T I O N S  
static const buzzerParamQueueEntryT buzzerTypesTable[] = 
  {    
     //BUZZER_BEEPING_OPTION1
    {BUZZER_OPCODE_ACTIVATE, 3200, 50, 100, 2},
    
     //BUZZER_BEEPING_OPTION2
    {BUZZER_OPCODE_ACTIVATE, 2600, 100, 750, 1},
    
    //BUZZER_BEEPING_OPTION3
    {BUZZER_OPCODE_ACTIVATE, 960, 50, 1000, 2},
    
    //BUZZER_BEEPING_OPTION4
    {BUZZER_OPCODE_ACTIVATE, 960, 30, 500, 1},   
    
    //BUZZER_BEEPING_ON_POWER_UP
    {BUZZER_OPCODE_ACTIVATE, 3200, 50, 100, 1},
  };

// E X T E R N A L S   
extern QueueHandle_t buzzerRequestQueueHandle;

// G L O B A L  P R O T O T Y P E S 
// G L O B A L S 

ReturnCode_T buzzerInit();
ReturnCode_T buzzerPatternSet(buzzerParamQueueEntryT parameters);
ReturnCode_T buzzerTaskEventSend(buzzerOpcodeQueue_T NotificationToBuzzerTask);
void buzzerTimerTimeoutCallback();
void buzzerRequestToQueueAdd(buzzerBeepingOption_T buzzerBeepingOption);
ReturnCode_T buzzerFreqSet(uint32_t frequencymHz);
ReturnCode_T buzzerOn(uint32_t frequencymHz);
ReturnCode_T buzzerOff();
#endif
