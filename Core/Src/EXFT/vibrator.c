#include "vibrator.h"

// G L O B A L S 
QueueHandle_t vibratorRequestQueueHandle;
TimerHandle_t vibratorTimerHandler;
uint8_t vibratorCounter;
uint8_t vibratorNumberOfCycles;
uint16_t vibratorTimeOn;
uint16_t vibratorTimeOff;
vibratorQueueEntryState_T vibratorState;

/******************************************************************************
* @brief  ReturnCode_T vibratorInit()
* @param  
* @retval 
******************************************************************************/
ReturnCode_T vibratorInit()
{
   // Create input queue  
  vibratorRequestQueueHandle = xQueueCreate(5, sizeof(vibratorParamQueueEntryT));
  
  // Create the task
  xTaskCreate(vibratorTask, vibratorTaskName, vibratorTaskSTACK_SIZE, NULL,  vibratorTaskPriority, ( TaskHandle_t * ) NULL );
  // Create timer.
  vibratorTimerHandler =  xTimerCreate("vibratorTimer",  portTICK_PERIOD_MS, pdFALSE, (void *)0, vibratorTimerTimeoutCallback);

  // Allways return OK
  return(RETURNCODE_OK);
}

/******************************************************************************
* @brief  portTASK_FUNCTION(vibratorTask, pvParameters )
* @param  vibratorTask, pvParameters
* @retval 
******************************************************************************/
portTASK_FUNCTION(vibratorTask, pvParameters )
{
  vibratorParamQueueEntryT QueueEntry;
  
  while(1)
  {
    // block task till new queue entry is received or timeout 
    BaseType_t  QueueState = xQueueReceive(vibratorRequestQueueHandle, &QueueEntry, 10000);
    // Check if there is new message.
    if(QueueState == pdTRUE)
    {
      switch(vibratorState)
      {
      case VIBRATOR_STATE_IDLE:
        switch (QueueEntry.opcode)
        {
        case VIBRATOR_OPCODE_ACTIVATE:
          if(configConfigurationDb.vibratorControl)
          {
            //Initialize parameters.
            vibratorPatternSet(QueueEntry);
            vibratorCounter = 0; 
            if (vibratorTimeOn > 0)
            {
              hwdriversGpioBitWrite(HWDRIVERS_VIBRATOR, 1);
              xTimerChangePeriod(vibratorTimerHandler, vibratorTimeOn, 100); 
            }
            else
            {
              //If vibratorTimeOn is less then 1, use only one milliSec so callback will be called.
              xTimerChangePeriod(vibratorTimerHandler, 1, 100); 
            }
            //Switch to next state.
            vibratorState = VIBRATOR_STATE_OFF_WAIT;
          }
          break;
        case VIBRATOR_OPCODE_TIMER:
          break;
        }
        break;

        case VIBRATOR_STATE_OFF_WAIT:
         switch (QueueEntry.opcode)
          {
          case VIBRATOR_OPCODE_ACTIVATE:
            break;
          case VIBRATOR_OPCODE_TIMER:
            if (vibratorTimeOff > 0)
            {
              hwdriversGpioBitWrite(HWDRIVERS_VIBRATOR, 0);
              xTimerChangePeriod(vibratorTimerHandler, vibratorTimeOff, 100);
            }
            else
            {
              //If vibratorTimeOff is less then 1, use only one milliSec so callback will be called.
              xTimerChangePeriod(vibratorTimerHandler, 1, 100);
            }
            //Switch to next state.
            vibratorState = VIBRATOR_STATE_ON_WAIT;
            break;
          }
         break;

        case VIBRATOR_STATE_ON_WAIT:
         switch (QueueEntry.opcode)
          {
          case VIBRATOR_OPCODE_ACTIVATE:
            break;
          case VIBRATOR_OPCODE_TIMER:
            vibratorCounter++;
            if (vibratorCounter == vibratorNumberOfCycles)
            {       
              hwdriversGpioBitWrite(HWDRIVERS_VIBRATOR, 0);
              vibratorState = VIBRATOR_STATE_IDLE;
            }
            else
            {
              if (vibratorTimeOn > 0)
              {
                hwdriversGpioBitWrite(HWDRIVERS_VIBRATOR, 1);
 
                xTimerChangePeriod(vibratorTimerHandler, vibratorTimeOn, 100); 
              }
              else
              {
                //If timeOn is less then 1, use only one milliSec so callback will be called.
                xTimerChangePeriod(vibratorTimerHandler, 1, 100); 
              }

              vibratorState = VIBRATOR_STATE_OFF_WAIT;
            }   
            break;
          }
         break;
      }    
    }
  }
  
}

/******************************************************************************
* @brief  ReturnCode_T vibratorPatternSet(vibratorParamQueueEntryT parameters)
* @param  parameters
* @retval 
******************************************************************************/
ReturnCode_T vibratorPatternSet(vibratorParamQueueEntryT parameters)
{  
  vibratorNumberOfCycles = parameters.numberOfCycles;
  //Calculate time on and time off.
  vibratorTimeOn = parameters.cycleTime * parameters.dutyCycle / 100;
  vibratorTimeOff = parameters.cycleTime * (100 - parameters.dutyCycle) / 100;
  
  // Allways return OK
  return(RETURNCODE_OK);
}

/******************************************************************************
* @brief  void vibratorTimerTimeoutCallback()
* @param  
* @retval 
******************************************************************************/
void vibratorTimerTimeoutCallback()
{
  vibratorTaskEventSend(VIBRATOR_OPCODE_TIMER);
}

/******************************************************************************
* @brief  ReturnCode_T vibratorTaskEventSend(vibratorOpcodeQueue_T NotificationToVibratorTask)
* @param  NotificationToVibratorTask
* @retval 
******************************************************************************/
ReturnCode_T vibratorTaskEventSend(vibratorOpcodeQueue_T NotificationToVibratorTask)
{
  vibratorParamQueueEntryT  vibratorTaskQueueEntry;
  
  // Fill the queue entry
  vibratorTaskQueueEntry.opcode = NotificationToVibratorTask;
  
  // Enqueue the event to the vibratorTask input queue
  xQueueSend(vibratorRequestQueueHandle, &vibratorTaskQueueEntry, 0);
  
  return(RETURNCODE_OK);
}

/******************************************************************************
* @brief  void vibratorRequestToQueueAdd(vibratorOption_T vibratorOption)
* @param  vibratorOption
* @retval 
******************************************************************************/
void vibratorRequestToQueueAdd(vibratorOption_T vibratorOption)
{
  vibratorParamQueueEntryT request = vibratorTypesTable[vibratorOption];
  //Add request to queue.
  xQueueSend(vibratorRequestQueueHandle, &request, 0);
}