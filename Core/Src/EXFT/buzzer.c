#include "common.h"
#include "buzzer.h"
#include "config.h"

//D E F I N E S
#define BUZZER_PRIMARY_FREQUENCY (uint32_t) 1000000 // (72MHz / 36) / 2

// G L O B A L S 
QueueHandle_t buzzerRequestQueueHandle;
TimerHandle_t buzzerTimerHandler;
uint8_t buzzerCounter;
uint8_t buzzerNumberOfCycles;
uint16_t buzzerTimeOn;
uint16_t buzzerTimeOff;
buzzerQueueEntryState_T buzzerState;
//TIM_TimeBaseInitTypeDef BuzzerTIM_TimeBaseStructure;
// C O N S T S
//TIM_TypeDef *BUZZER_TIM = TIM5;

/******************************************************************************
* @brief  ReturnCode_T buzzerInit(void)
* @param  
* @retval 
******************************************************************************/
ReturnCode_T buzzerInit()
{
  //Init BuzzerTIM_TimeBaseStructure only one time.
//  TIM_TimeBaseStructInit(&BuzzerTIM_TimeBaseStructure);
  
  // Create input queue  
  buzzerRequestQueueHandle = xQueueCreate(5, sizeof(buzzerParamQueueEntryT));
  
  // Create the task
  xTaskCreate(buzzerTask, buzzerTaskName, buzzerTaskSTACK_SIZE, NULL,  buzzerTaskPriority, ( TaskHandle_t * ) NULL );
  // Create timer.
  buzzerTimerHandler =  xTimerCreate("buzzerTimer",  portTICK_PERIOD_MS, pdFALSE, (void *)0, buzzerTimerTimeoutCallback);

  // Allways return OK
  return(RETURNCODE_OK);
}

/******************************************************************************
* @brief  portTASK_FUNCTION(buzzerTask, pvParameters )
* @param  buzzerTask, pvParameters
* @retval 
******************************************************************************/
portTASK_FUNCTION(buzzerTask, pvParameters )
{
  buzzerParamQueueEntryT QueueEntry;
  
  while(1)
  {
    // block task till new queue entry is received or timeout 
    BaseType_t  QueueState = xQueueReceive(buzzerRequestQueueHandle, &QueueEntry, 10000);
    // Check if there is new message.
    if(QueueState == pdTRUE)
    {
      switch(buzzerState)
      {
      case BUZZER_STATE_IDLE:
        switch (QueueEntry.opcode)
        {
        case BUZZER_OPCODE_ACTIVATE:
          if(configConfigurationDb.BuzzerControl)
          {
            //Initialize parameters.
            buzzerPatternSet(QueueEntry);
            buzzerCounter = 0; 
            if (buzzerTimeOn > 0)
            {
             // TIM_CCxCmd(BUZZER_TIM, TIM_Channel_1, TIM_CCx_Enable);
             TIM_CCxChannelCmd(TIM5, TIM_CHANNEL_1, TIM_CCx_ENABLE);
              xTimerChangePeriod(buzzerTimerHandler, buzzerTimeOn, 100); 
              __HAL_TIM_ENABLE(&htim5);
            }
            else
            {
              //If buzzerTimeOn is less then 1, use only one milliSec so callback will be called.
              xTimerChangePeriod(buzzerTimerHandler, 1, 100);
              __HAL_TIM_ENABLE(&htim5);
            }
            //Switch to next state.
            buzzerState = BUZZER_STATE_OFF_WAIT;
          }
          break;
        case BUZZER_OPCODE_TIMER:
          break;
        }
        break;

        case BUZZER_STATE_OFF_WAIT:
         switch (QueueEntry.opcode)
          {
          case BUZZER_OPCODE_ACTIVATE:
            break;
          case BUZZER_OPCODE_TIMER:
            if (buzzerTimeOff > 0)
            {
 //             TIM_CCxCmd(BUZZER_TIM, TIM_Channel_1, TIM_CCx_Disable);
                TIM_CCxChannelCmd(TIM5, TIM_CHANNEL_1, TIM_CCx_DISABLE);
              xTimerChangePeriod(buzzerTimerHandler, buzzerTimeOff, 100);
            }
            else
            {
              //If buzzerTimeOff is less then 1, use only one milliSec so callback will be called.
              xTimerChangePeriod(buzzerTimerHandler, 1, 100);
            }
            //Switch to next state.
            buzzerState = BUZZER_STATE_ON_WAIT;
            break;
          }
         break;

        case BUZZER_STATE_ON_WAIT:
         switch (QueueEntry.opcode)
          {
          case BUZZER_OPCODE_ACTIVATE:
            break;
          case BUZZER_OPCODE_TIMER:
            buzzerCounter++;
            if (buzzerCounter == buzzerNumberOfCycles)
            {       
            	TIM_CCxChannelCmd(TIM5, TIM_CHANNEL_1, TIM_CCx_DISABLE);
            	buzzerState = BUZZER_STATE_IDLE;
            }
            else
            {
              if (buzzerTimeOn > 0)
              {
                 	TIM_CCxChannelCmd(TIM5, TIM_CHANNEL_1, TIM_CCx_ENABLE);
                    xTimerChangePeriod(buzzerTimerHandler, buzzerTimeOn, 100);
              }
              else
              {
                //If timeOn is less then 1, use only one milliSec so callback will be called.
                xTimerChangePeriod(buzzerTimerHandler, 1, 100); 
              }

              buzzerState = BUZZER_STATE_OFF_WAIT;
            }   
            break;
          }
         break;
         
      case BUZZER_STATE_ON:
      	TIM_CCxChannelCmd(TIM5, TIM_CHANNEL_1, TIM_CCx_ENABLE);
      	__HAL_TIM_ENABLE(&htim5);
        break;
        
      case BUZZER_STATE_OFF:
      	TIM_CCxChannelCmd(TIM5, TIM_CHANNEL_1, TIM_CCx_DISABLE);
      	__HAL_TIM_DISABLE(&htim5);
        buzzerState = BUZZER_STATE_IDLE;
        break;
      }    
    }
  }
  
}

/******************************************************************************
* @brief  ReturnCode_T buzzerFreqSet(uint16_t frequencymHz)
* @param  frequencymHz
* @retval 
******************************************************************************/
ReturnCode_T buzzerFreqSet(uint32_t frequencymHz)
{
	TIM_Base_InitTypeDef TIM_TimeBaseStructure;


	uint32_t calcPeriod = ((BUZZER_PRIMARY_FREQUENCY * 1000) /  frequencymHz) - 1;



	__HAL_TIM_SET_AUTORELOAD(&htim5,  calcPeriod);



#if 0

	// TODO: Not HAL
TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
TIM_TimeBaseStructure.Period = calcPeriod;
TIM_TimeBaseStructure.Prescaler = 1;
TIM_TimeBaseStructure.ClockDivision  = 0;
TIM_TimeBaseStructure.CounterMode = TIM_COUNTERMODE_UP;
TIM_Base_SetConfig(&htim5, &TIM_TimeBaseStructure);



  TIM_TimeBaseStructInit(&BuzzerTIM_TimeBaseStructure);
  BuzzerTIM_TimeBaseStructure.TIM_Period = calcPeriod;
  BuzzerTIM_TimeBaseStructure.TIM_Prescaler = 1;
  BuzzerTIM_TimeBaseStructure.TIM_ClockDivision = 0;
  BuzzerTIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(BUZZER_TIM, &BuzzerTIM_TimeBaseStructure);
#endif

  
  return(RETURNCODE_OK);
}

/******************************************************************************
* @brief  ReturnCode_T buzzerPatternSet(buzzerParamQueueEntryT parameters)
* @param  parameters
* @retval 
******************************************************************************/
ReturnCode_T buzzerPatternSet(buzzerParamQueueEntryT parameters)
{
  buzzerFreqSet(parameters.frequencyHz * 1000);

  buzzerNumberOfCycles = parameters.numberOfCycles;
  //Calculate time on and time off.
  buzzerTimeOn = parameters.cycleTime * parameters.dutyCycle / 100;
  buzzerTimeOff = parameters.cycleTime * (100 - parameters.dutyCycle) / 100;
  
  // Always return OK
  return(RETURNCODE_OK);
}

/******************************************************************************
* @brief  void buzzerTimerTimeoutCallback()
* @param  
* @retval 
******************************************************************************/
void buzzerTimerTimeoutCallback()
{
  buzzerTaskEventSend(BUZZER_OPCODE_TIMER);
}

/******************************************************************************
* @brief  ReturnCode_T buzzerTaskEventSend(buzzerOpcodeQueue_T NotificationToBuzzerTask)
* @param  NotificationToBuzzerTask
* @retval 
******************************************************************************/
ReturnCode_T buzzerTaskEventSend(buzzerOpcodeQueue_T NotificationToBuzzerTask)
{
  buzzerParamQueueEntryT  buzzerTaskQueueEntry;
  
  // Fill the queue entry
  buzzerTaskQueueEntry.opcode = NotificationToBuzzerTask;
  
  // Enqueue the event to the buzzerTask input queue
  xQueueSend(buzzerRequestQueueHandle, &buzzerTaskQueueEntry, 0);
  
  return(RETURNCODE_OK);
}


/******************************************************************************
* @brief  void buzzerRequestToQueueAdd(buzzerBeepingOption_T buzzerBeepingOption)
* @param  
* @retval 
******************************************************************************/
void buzzerRequestToQueueAdd(buzzerBeepingOption_T buzzerBeepingOption)
{
  buzzerParamQueueEntryT request = buzzerTypesTable[buzzerBeepingOption];
  //Add request to queue.
  xQueueSend(buzzerRequestQueueHandle, &request, 0);
}

/******************************************************************************
* @brief  ReturnCode_T buzzerOn(uint32_t frequencymHz)
* @param  frequencymHz
* @retval 
******************************************************************************/
ReturnCode_T buzzerOn(uint32_t frequencymHz)
{
  buzzerParamQueueEntryT  buzzerTaskQueueEntry;
  
  buzzerFreqSet(frequencymHz);
  buzzerState = BUZZER_STATE_ON;
  xQueueSend(buzzerRequestQueueHandle, &buzzerTaskQueueEntry, 0);
  
  return(RETURNCODE_OK);
}

/******************************************************************************
* @brief  ReturnCode_T buzzerOff()
* @param  
* @retval 
******************************************************************************/
ReturnCode_T buzzerOff()
{
  buzzerParamQueueEntryT  buzzerTaskQueueEntry;
  buzzerState = BUZZER_STATE_OFF;
  xQueueSend(buzzerRequestQueueHandle, &buzzerTaskQueueEntry, 0);
  return(RETURNCODE_OK);
}
