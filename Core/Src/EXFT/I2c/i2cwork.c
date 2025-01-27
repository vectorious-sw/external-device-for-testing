#include "common.h"
#include "hwdrivers.h"
#include "vlapConfig.h"
#include "semphr.h"
#include "i2cwork.h"
#include "timers.h"
#include "leds.h"
#include "vlapMain.h"

// L O C A L    D E F I N I T I O N S
#define I2CWORK_SCHEDULER_NUMBER_OF_ENTRIES     4
#define I2CWORK_QUEUE_TIME_EVENT_TIMEOUT 40
#define I2CWORK_PS_OFF_DELAY 400

// L O C A L    P R O T O T Y P E S
static void i2cworkTimerCallBack(TimerHandle_t pxTimer);
void I2C1_Setup_Job(uint8_t job_, volatile uint8_t* data);
void startJob(volatile uint8_t direction,
              volatile uint8_t *pBuffer,
              volatile uint8_t deviceAddr,
              volatile uint8_t addr,
              volatile uint16_t numBytes,
              volatile I2CREQ *i2cReq);



// M O D U L E   G L O B A L S
// I2c Scheduler array     
i2cworkSchedulerDataBaseEntry_T I2cSchedulerArray[I2CWORK_SCHEDULER_NUMBER_OF_ENTRIES];
// Number of active array entries
uint8_t I2cSchedulerArrayActiveEntries;

// I2C Scheduler Queue handler
QueueHandle_t i2cworlSchedulerQueueHandle;
// 
I2CREQ i2cworkReq;
// I2C Scheduler state var
i2cworkSchedulerState_T i2cWorkSchedulerState;
// I2C scan index
uint8_t I2cSchedulerArrayIndex;
// TODO: Remove 
volatile SemaphoreHandle_t i2c1ReadSemaphore = NULL; // semaphore to perform read transaction 
//QueueHandle_t i2cReqQ;
volatile uint32_t QueueErrorCounter=0;
i2cworkDuringMessageState_T i2cworkDuringMessageState = I2CWORK_NOT_DURING_MESSAGE;

uint8_t i2cSupplyOnDelay = 0;
uint8_t i2cSupplyOffDelay = 0;
uint16_t ErrorCounter;

volatile I2C_Job_Type I2C_jobs[2];


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
*                  
*                  
* Output         : None
* Return         : None
*******************************************************************************/
ReturnCode_T i2cworkSchedulerInit()
{

	uint8_t i;
  uint8_t* Dptr = (uint8_t*)I2cSchedulerArray;
  // Clear the array
  for(i=0; i<sizeof(I2cSchedulerArray)/sizeof(i2cworkSchedulerDataBaseEntry_T); i++)
    Dptr[i] = 0;

  I2cSchedulerArrayActiveEntries = 0;
  // Create the schedule queue  
  i2cworlSchedulerQueueHandle = xQueueCreate(3, sizeof(i2cworlSchedulerQueueEntry_T));
  // Set the initial scheduler state
  i2cWorkSchedulerState = I2CWORK_SCHEDULER_STATE_TIME_WAIT;
  // Configure the I2C
  i2cworkConfig();
  // CReate the task
  xTaskCreate(i2cworkSchedulerTask, i2cserverName, i2cserverSTACK_SIZE, NULL, 
              i2cserverPriority, ( TaskHandle_t * ) NULL );
  // Initiate timer to stimuli the I2C scheduler time events
  xTimerStart(xTimerCreate("I2cScheduler", I2CWORK_QUEUE_TIME_EVENT_TIMEOUT, pdTRUE, (void *)0, i2cworkTimerCallBack), 0); 
  
//  i2c1ReadSemaphore = xSemaphoreCreateCounting(2, 1);
  ErrorCounter = 0;
  // Allways return OK
  return(RETURNCODE_OK);
}

ReturnCode_T i2cworkSchedulerRemoveAllLeds()
{
  uint8_t i;
  ReturnCode_T MyReturnCode = RETURNCODE_OK;
  // Search for empty scheduler entry in the data base, We search only for the Period field, Period of 0 indicates empty entry
  for(i=0; i<I2CWORK_SCHEDULER_NUMBER_OF_ENTRIES; i++)
  {
    if (I2cSchedulerArray[i].MyEntry.workType == I2CWORK_LEDS_REQUEST)
    {
      // Remove this index from I2C scheduler array.
      I2cSchedulerArray[i].MyEntry.Period = 0;
      if(I2cSchedulerArray[i].Counter)
        I2cSchedulerArray[I2cSchedulerArrayIndex].Counter--;   
    }
    
  }
  // Return 
  return(MyReturnCode);
  
}


/*******************************************************************************
* Function Name  : ReturnCode_T i2cworkSchedulerRegister( i2cworkSchedulerEntry_T* SchedulerEntryPtr)
* Description    : 
* Input          : 
*                  
*                  
* Output         : None
* Return         : None
*******************************************************************************/
ReturnCode_T i2cworkSchedulerRegister( i2cworkSchedulerEntry_T* SchedulerEntryPtr)
{
  uint8_t i;
  ReturnCode_T MyReturnCode = RETURNCODE_ERROR;
  // The new Period field's value must not be zero
  if(SchedulerEntryPtr->Period)
  {
    // Search for empty scheduler entry in the data base, We search only for the Period field, Period of 0 indicates empty entry
    for(i=0; i<I2CWORK_SCHEDULER_NUMBER_OF_ENTRIES; i++)
    {
      if ((I2cSchedulerArray[i].MyEntry.LedIndex == SchedulerEntryPtr->LedIndex) && (I2cSchedulerArray[i].MyEntry.Period) 
          && (I2cSchedulerArray[i].MyEntry.workType == I2CWORK_LEDS_REQUEST) && (SchedulerEntryPtr->workType == I2CWORK_LEDS_REQUEST))
      {
        // Entry found, Copy the source data to the destination entry
        memcpy((uint8_t*)(&(I2cSchedulerArray[i].MyEntry)), (uint8_t*)(SchedulerEntryPtr), sizeof(i2cworkSchedulerEntry_T));
        if(I2cSchedulerArray[i].MyEntry.Mode == I2CWORK_REQMODE_PERIODIC)
        {
          // Sets the "Counter" field to the new "Period" field
          I2cSchedulerArray[i].Counter = I2cSchedulerArray[i].MyEntry.Period;
        }
        else
        {
          // One Shot
          // Force the Counter to 1 so only one transaction will be precessed
          I2cSchedulerArray[i].Counter = 1;
        }
        I2cSchedulerArrayActiveEntries++;
        // Break the outer for loop, we have found empty entry 
        break;
      }
      else
      {
        if(!I2cSchedulerArray[i].MyEntry.Period)
        {
            I2cSchedulerArrayActiveEntries++;
        	// Entry found, Copy the source data to the destination entry
          memcpy((uint8_t*)(&(I2cSchedulerArray[i].MyEntry)), (uint8_t*)(SchedulerEntryPtr), sizeof(i2cworkSchedulerEntry_T));
          if(I2cSchedulerArray[i].MyEntry.Mode == I2CWORK_REQMODE_PERIODIC)
          {
            // Sets the "Counter" field to the new "Period" field
            I2cSchedulerArray[i].Counter = I2cSchedulerArray[i].MyEntry.Period;
          }
          else
          {
            // One Shot
            // Force the Counter to 1 so only one transaction will be precessed
            I2cSchedulerArray[i].Counter = 1;
          }
          // Break the outer for loop, we have found empty entry 
          break;
        }
        
      }      
    }
    // Check if the search reached the end of the array or empty entry was found
    if(i==I2CWORK_SCHEDULER_NUMBER_OF_ENTRIES)
      MyReturnCode = RETURNCODE_NO_EMPTY_ENTRIES;
    else
      MyReturnCode = RETURNCODE_OK;
  }
  else
    MyReturnCode = RETURNCODE_UNSUPPORTED;
  
  // Return 
  return(MyReturnCode);
}

// For debug
char PrintBuff[100];
uint8_t iter;
/*******************************************************************************
* Function Name  : portTASK_FUNCTION(i2cworkSchedulerTask, pvParameters )
* Description    : 
* Input          : 
*                  
*                  
* Output         : None
* Return         : None
*******************************************************************************/
portTASK_FUNCTION(i2cworkSchedulerTask, pvParameters )
{
  ReturnCode_T MyReturnCode = RETURNCODE_ERROR;
  i2cworlSchedulerQueueEntry_T QueueEntry;
  i2cworkSchedulerFunctionParameter_T ReturnedCommand;
  

  
  while(1)
  {
    // Wait for queue event
    if (xQueueReceive(i2cworlSchedulerQueueHandle, &QueueEntry, 3000) == pdTRUE)
    {
      switch( i2cWorkSchedulerState)
      {
      case I2CWORK_SCHEDULER_STATE_TIME_WAIT:
        // Time event will fall through to the next state for scanning pending registered entries
        if(QueueEntry.Evnet != I2CWORK_QUEUE_TIME_EVENT)
          break;
      case I2CWORK_SCHEDULER_STATE_SCAN:
        switch(QueueEntry.Evnet)
        {
        default:
          // Unexpected Event
          break;
        case I2CWORK_QUEUE_TIME_EVENT:
          // Time event starts the array scan from the beginning, while CALLSELF continues the scan
          I2cSchedulerArrayIndex = 0;
        case I2CWORK_QUEUE_I2C_CALLSELF_EVENT:
        	break;
          // Fall through to continue the scan: Coming from I2CWORK_SCHEDULER_STATE_TIME_WAIT, or from I2CWORK_SCHEDULER_STATE_I2C_COMPLETION_WAIT
        }
		for(;I2cSchedulerArrayIndex<I2CWORK_SCHEDULER_NUMBER_OF_ENTRIES; I2cSchedulerArrayIndex++)
		{
		  if(I2cSchedulerArray[I2cSchedulerArrayIndex].MyEntry.Period && !I2cSchedulerArray[I2cSchedulerArrayIndex].Counter)
		  {
			// Prepare for the next period
			I2cSchedulerArray[I2cSchedulerArrayIndex].Counter = I2cSchedulerArray[I2cSchedulerArrayIndex].MyEntry.Period;
			// If Pre processing function registered, call the function
			if( I2cSchedulerArray[I2cSchedulerArrayIndex].MyEntry.I2cWorkerPreTranscationCallBackPtr)
			  MyReturnCode = (*I2cSchedulerArray[I2cSchedulerArrayIndex].MyEntry.I2cWorkerPreTranscationCallBackPtr)(I2cSchedulerArray[I2cSchedulerArrayIndex].MyEntry.LedIndex,  &ReturnedCommand);
			// If the function returned OK, process the I2C request
			if(MyReturnCode == RETURNCODE_OK)
			{
			  // Fill the Request structure
			  i2cworkReq.deviceAddr               = ReturnedCommand.I2cSlaveAddress;
			  i2cworkReq.regAddr                  = ReturnedCommand.SlaveRegAddress;
			  i2cworkReq.numBytes                 = ReturnedCommand.TranscationLength;
			  i2cworkReq.pBuffer                  = ReturnedCommand.TransactionDataPtr;
			  i2cworkReq.InitiatingQueueHandle    = i2cworlSchedulerQueueHandle;
			  // Select the command registered in the database
			  if(ReturnedCommand.Cmd == I2CWORK_TRANSACTION_CMD_WRITE)
			  {
				i2cworkDuringMessageState = I2CWORK_DURING_MESSAGE;

				// Initiate I2C Write transaction
				i2cworkBufferWrite(I2C1, &i2cworkReq);

				// Change the state to wait for the I2C transaction end (Initiated by I2C1 completion interrupt)
				i2cWorkSchedulerState = I2CWORK_SCHEDULER_STATE_I2C_COMPLETION_WAIT;
				// Break the outer for loop and move to the new state
				break;
			  }
			  else
				if(ReturnedCommand.Cmd == I2CWORK_TRANSACTION_CMD_READ)
				{
				  i2cworkDuringMessageState = I2CWORK_DURING_MESSAGE;

				  // Initiate I2C Read transaction
				  i2cworkBufferRead(I2C1, &i2cworkReq);
				  // Change the state to wait for the I2C transcation end (Initiated by completion interrupt)
				  i2cWorkSchedulerState = I2CWORK_SCHEDULER_STATE_I2C_COMPLETION_WAIT;
				  // Break the outer for loop and move to the new state
				  break;
				}
				else
				  if(ReturnedCommand.Cmd == I2CWORK_TRANSACTION_CMD_IDLE)
				  {
					// Remove this index from I2C scheduler array.
					I2cSchedulerArray[I2cSchedulerArrayIndex].MyEntry.Period = 0;
					if(I2cSchedulerArray[I2cSchedulerArrayIndex].Counter)
					  I2cSchedulerArray[I2cSchedulerArrayIndex].Counter--;
				  }
				  else
					if(ReturnedCommand.Cmd == I2CWORK_TRANSACTION_CMD_SKIP)
					{
					  //Skip this call, do nothing.
					}
			}
		  }
		  else
			if(I2cSchedulerArray[I2cSchedulerArrayIndex].Counter)
			  I2cSchedulerArray[I2cSchedulerArrayIndex].Counter--;
		}
        break;
        
      case I2CWORK_SCHEDULER_STATE_I2C_COMPLETION_WAIT:   
        switch(QueueEntry.Evnet)
        {
        case I2CWORK_QUEUE_TIME_EVENT:
#if 1
          //xSemaphoreGive(i2c1ReadSemaphore); 
          
          ErrorCounter++;
          sprintf(PrintBuff, "I2CWORK_QUEUE_TIME_EVENT!!!!!, I2C timeout counter: %d", ErrorCounter);
          
//          if((ErrorCounter > 30)&&(!measureMeasurementInProgressGet()))
//          {
//            tracerHardFaultExceptionImageSave(TRACER_GENERATE_SW_HW_RESET_I2C_TIMEOUT);
//            tracerSwHwResetGenerate(TRACER_GENERATE_SW_HW_RESET_I2C_TIMEOUT);
//          }
          
          vlapmainDebugLog(PrintBuff);
          __HAL_I2C_DISABLE(&hi2c1);
          // I2C_DMACmd(I2C1, DISABLE);
          
          // Possible I2C bus busy state since one of the I2C peripherals asserted the bus, Turn off the power rails powering
//          hwdriversGpioBitWrite(HWDRIVERS_UI_ENABLE,0);
          // TODO : TURN I2C LINES OFF
        //  hwdriversI2CLinesGpioConfigure(HWDRIVERS_I2C_OUTPUT_LOW);

          hwdriversI2CLinesGpioConfigure(HWDRIVERS_I2C_OUTPUT_LOW);
          i2cworkConfig();
          // Self trigger the task to continue the scan, move to wait for the power to settel 
          i2cWorkSchedulerState = I2CWORK_SCHEDULER_STATE_I2C_FAULT_PS_OFF_WAIT;
#endif
          break;
        case I2CWORK_QUEUE_I2C_CALLSELF_EVENT:
          // Error: I2C must complete before the next time event
          break;
        case I2CWORK_QUEUE_I2C_COMPLETION_EVENT:
          // If completion callback is registered in the database, call the callback with the results of the I2C read command
          // If Pre processing function registered, call the function
          if( I2cSchedulerArray[I2cSchedulerArrayIndex].MyEntry.I2cWorkerPostTranscationCallBackPtr)
            MyReturnCode = (*I2cSchedulerArray[I2cSchedulerArrayIndex].MyEntry.I2cWorkerPostTranscationCallBackPtr)( I2cSchedulerArray[I2cSchedulerArrayIndex].MyEntry.LedIndex, &ReturnedCommand);
          // Anyway, go continue the scan.
          i2cWorkSchedulerState = I2CWORK_SCHEDULER_STATE_SCAN;
          // Self trigger the task to continue the scan
          QueueEntry.Evnet = I2CWORK_QUEUE_I2C_CALLSELF_EVENT;
          xQueueSend(i2cworlSchedulerQueueHandle, &QueueEntry, 0);
          // If Single Shote, Clear the Period field marking the entry Empty
          if(I2cSchedulerArray[I2cSchedulerArrayIndex].MyEntry.Mode == I2CWORK_REQMODE_ONESHOT)
          {
            I2cSchedulerArray[I2cSchedulerArrayIndex].MyEntry.Period = 0;
            I2cSchedulerArray[I2cSchedulerArrayIndex].Counter = 0;          
          }
          
          i2cworkDuringMessageState = I2CWORK_NOT_DURING_MESSAGE;
        }
        break;
        
      case I2CWORK_SCHEDULER_STATE_I2C_FAULT_PS_OFF_WAIT:
        switch(QueueEntry.Evnet)
        {
        case I2CWORK_QUEUE_TIME_EVENT:
#if 0
          if(i2cSupplyOffDelay == 0)
          {
            for(iter = 0; iter < 18; iter++)
            {
              if(iter%2)
                hwdriversI2CLinesGpioConfigure(HWDRIVERS_I2C_OUTPUT_TOGGLE_HIGH);
              else
                hwdriversI2CLinesGpioConfigure(HWDRIVERS_I2C_OUTPUT_TOGGLE_LOW);
            }
          }
          i2cSupplyOffDelay++;
#endif
          break;
        default:
          break;
        }
        
        if(i2cSupplyOffDelay * I2CWORK_QUEUE_TIME_EVENT_TIMEOUT >= I2CWORK_PS_OFF_DELAY)
        {         
          i2cSupplyOffDelay = 0;
          //i2cworkConfig();

//          hwdriversGpioBitWrite(HWDRIVERS_UI_ENABLE,1);
          // TODO : TURN I2C LINES ON
 //         hwdriversI2CLinesGpioConfigure(HWDRIVERS_I2C_LINES_AF);


          __HAL_I2C_ENABLE(&hi2c1);

//          I2C_DMACmd(I2C1, ENABLE);

          // Self trigger the task to continue the scan, move to wait for the power to settel 
          i2cWorkSchedulerState = I2CWORK_SCHEDULER_STATE_I2C_FAULT_PS_ON_WAIT;
        }
        else
        	i2cSupplyOffDelay++;

        break;
        
      case I2CWORK_SCHEDULER_STATE_I2C_FAULT_PS_ON_WAIT:
        switch(QueueEntry.Evnet)
        {
        case I2CWORK_QUEUE_TIME_EVENT:
          i2cSupplyOnDelay++;
          if(__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY))
//          if(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
          {
            i2cWorkSchedulerState = I2CWORK_SCHEDULER_STATE_I2C_COMPLETION_WAIT;
            i2cSupplyOnDelay = 0;
          }
          break;
        default:
          break;
        }
        
        if(i2cSupplyOnDelay * I2CWORK_QUEUE_TIME_EVENT_TIMEOUT >= LEDS_DELAY_TIME_FOR_INIT)
        {                     
            __HAL_I2C_ENABLE(&hi2c1);

            i2cworkConfig();

        	i2cSupplyOnDelay = 0;
                    
          // Anyway, go continue the scan.
          i2cWorkSchedulerState = I2CWORK_SCHEDULER_STATE_SCAN;
          // Self trigger the task to continue the scan
          QueueEntry.Evnet = I2CWORK_QUEUE_I2C_CALLSELF_EVENT;
          xQueueSend(i2cworlSchedulerQueueHandle, &QueueEntry, 0);
          // If Single Shot, Clear the Period field marking the entry Empty
          if(I2cSchedulerArray[I2cSchedulerArrayIndex].MyEntry.Mode == I2CWORK_REQMODE_ONESHOT)
          {
            I2cSchedulerArray[I2cSchedulerArrayIndex].MyEntry.Period = 0;
            I2cSchedulerArray[I2cSchedulerArrayIndex].Counter = 0;          
          }
        }
              
        break;
        
      }      
    }
    else
    {
      // Queue Rx error 
      QueueErrorCounter++;
    }
  }
  
}


i2cworkDuringMessageState_T i2cworkDuringMessageStateGet()
{
  return(i2cworkDuringMessageState);
}


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
*                  
*                  
* Output         : None
* Return         : None
*******************************************************************************/
void i2cworkConfig(void)
{

	  /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x601015E9;
    hi2c1.Init.OwnAddress1 = 102;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	  {
	    Error_Handler();
	  }

#if 0

	I2C_InitTypeDef  I2C_InitStructure;

  
  // gpio initialization is done in gpioTableActivate()
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);  
  I2C_DeInit(I2C1);
  
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DUTYCYCLE;
  I2C_InitStructure.I2C_OwnAddress1 = 0x33;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_ClockSpeed = I2C1_SPEED;
  //I2C_InitStructure.I2C_ClockSpeed = 5000;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  
  I2C_Init(I2C1, &I2C_InitStructure);
  
  I2C_Cmd(I2C1, ENABLE);  
  
  //
  // wa:
  // cant work with i2c due to busy bit on. 
  // change pin output-type from open-drain (as in gpiotable PB8 and PB9) to push-pull.
  // it happens to be working :)
  //
  if (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
  {
    GPIO_OTYPER_SET_PP(GPIOB, 8); // sync with gpioTable.h
    GPIO_OTYPER_SET_PP(GPIOB, 9); // ""
  }
  
//  /* Configure the I2C event priority */
//  NVIC_InitStructure.NVIC_IRQChannel                   = I2C1_EV_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = INTERRUPT_PRIORITY_VALID_FROMISR_API;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
//  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);

#endif
#if 1
  if(GPIOB->BSRR & I2C_FLAG_BUSY)
  {
	  GPIOB->OTYPER &= ~(1<<8);
	  GPIOB->OTYPER &= ~(1<<9);
  }
#endif

}


/*******************************************************************************
* Function Name  : static void voltageCheckCallback(TimerHandle_t pxTimer)
* Description    : 
* Input          : 
*                  
*                  
* Output         : 
* Return         : None
*******************************************************************************/
static void i2cworkTimerCallBack(TimerHandle_t pxTimer)
{
  i2cworlSchedulerQueueEntry_T MyQueueEntry;
  
  // Periodically send timer event to the scheduler queue   
  MyQueueEntry.Evnet = I2CWORK_QUEUE_TIME_EVENT;
  xQueueSend(i2cworlSchedulerQueueHandle, &MyQueueEntry, 0);
  
}


/*******************************************************************************
* Function Name  : i2cworkBufferRead
* Description    : read a numByteToRead bytes from I2C Bus
* Input          : deviceAddr is the I2C address of the device
*                  readAddr is the register address you want to read from
*                  numByteToRead is the number of bytes to read
* Output         : pBuffer is the buffer that contains bytes read
* Return         : None
*******************************************************************************/
void i2cworkBufferRead(I2C_TypeDef *I2Cx, I2CREQ *i2cReq)
{
	HAL_I2C_Mem_Read_DMA(&hi2c1, i2cReq->deviceAddr, i2cReq->regAddr, I2C_MEMADD_SIZE_8BIT, i2cReq->pBuffer, i2cReq->numBytes);


	startJob(0, i2cReq->pBuffer, i2cReq->deviceAddr,i2cReq->regAddr, i2cReq->numBytes, i2cReq);

}

/*******************************************************************************
* Function Name  : i2cworkBufferWrite
* Description    : write a Byte to I2C Bus
* Input          : deviceAddr is the I2C address of the device
*                  WriteAddr is the register address you want to write to
*                  pBuffer contains bytes to write
* Output         : None
* Return         : None
*******************************************************************************/
void i2cworkBufferWrite(I2C_TypeDef *I2Cx, I2CREQ *i2cReq)
{
  
	// TODO: i2cReq->regAddr NOT SUPPORTED

	HAL_I2C_Mem_Write_DMA(&hi2c1, i2cReq->deviceAddr, i2cReq->regAddr, I2C_MEMADD_SIZE_8BIT, i2cReq->pBuffer, i2cReq->numBytes);

	startJob(0, i2cReq->pBuffer, i2cReq->deviceAddr, i2cReq->regAddr, i2cReq->numBytes, i2cReq);
}


uint32_t i2cworkintErroCounter;



void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	  BaseType_t ret;
	  BaseType_t xHigherPriorityTaskWoken  = pdFALSE;
    i2cworlSchedulerQueueEntry_T QueueEntry;
     QueueEntry.Evnet = I2CWORK_QUEUE_I2C_COMPLETION_EVENT;
     ret = xQueueSendToBackFromISR(I2C_jobs[0].i2cReq->InitiatingQueueHandle, &QueueEntry, &xHigherPriorityTaskWoken);
     if (ret != pdTRUE)
     {
       i2cworkintErroCounter++;
     }
}




void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	  BaseType_t ret;
	  BaseType_t xHigherPriorityTaskWoken  = pdFALSE;
	i2cworlSchedulerQueueEntry_T QueueEntry;
     QueueEntry.Evnet = I2CWORK_QUEUE_I2C_COMPLETION_EVENT;
     ret = xQueueSendToBackFromISR(I2C_jobs[0].i2cReq->InitiatingQueueHandle, &QueueEntry, &xHigherPriorityTaskWoken);
     if (ret != pdTRUE)
     {
       i2cworkintErroCounter++;
     }
}


void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	BaseType_t ret;
	BaseType_t xHigherPriorityTaskWoken  = pdFALSE;
	i2cworlSchedulerQueueEntry_T QueueEntry;
	QueueEntry.Evnet = I2CWORK_QUEUE_I2C_COMPLETION_EVENT;
	ret = xQueueSendToBackFromISR(I2C_jobs[0].i2cReq->InitiatingQueueHandle, &QueueEntry, &xHigherPriorityTaskWoken);
	if (ret != pdTRUE)
	{
	 i2cworkintErroCounter++;
	}
}







/**
*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
*                  
*                  
* Output         : None
* Return         : None
*******************************************************************************/
void i2c1EndTransactionFromISR(void)
{


  //  vdbg("i2c end transaction\n");
  
  
}

/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
*                  
*                  
* Output         : None
* Return         : None
*******************************************************************************/
void i2cRespFree(I2CRESP *i2cResp)
{
  //
  // you must free the response resources:
  // data, and the response-object which was  allocated by the sender
  //
  if (i2cResp->validStruct != VALID_STRUCT)
    return;
  if (i2cResp->data)
    vPortFree((void *)i2cResp->data);   
  if (i2cResp->this)
    vPortFree((void *)i2cResp->this);
}

/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
*                  
*                  
* Output         : None
* Return         : None
*******************************************************************************/
void i2cRespFreeFromISR(I2CRESP *i2cResp)
{
  //
  // you must free the response resources:
  // data, and the response-object which was  allocated by the sender
  //
  if (i2cResp->validStruct != VALID_STRUCT)
    return;
  if (i2cResp->data)
    vPortFree((void *)i2cResp->data);   
  if (i2cResp->this)
    vPortFree((void *)i2cResp->this);
}

/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
*                  
*                  
* Output         : None
* Return         : None
*******************************************************************************/
uint8_t i2cRespValid(I2CRESP *i2cResp)
{
  return (i2cResp->validStruct == VALID_STRUCT)?1:0;
}


/*******************************************************************************
 * Function Name  : i2cworkSyncByteWrite
 * Description    : write a Byte to I2C Bus
 * Input          : deviceAddr is the I2C address of the device
 *                  WriteAddr is the register address you want to write to
 *                  pBuffer contains bytes to write
 * Output         : None
 * Return         : None
 *******************************************************************************/
uint8_t i2cworkSyncByteWrite(I2C_TypeDef *I2Cx, uint8_t *pBuffer, uint8_t deviceAddress, uint8_t WriteAddr)
{


	HAL_I2C_Mem_Write(&hi2c1, deviceAddress, WriteAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, 1, 10);
//	HAL_I2C_Mem_Write(&hi2c1, deviceAddress, 0, I2C_MEMADD_SIZE_8BIT, pBuffer, Count);



#if 0
	uint32_t i2c_timeout;

  // /* While the bus is busy * /
  i2c_timeout = I2C_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
  {
    if (!(i2c_timeout--))
      return i2cworkSyncTimeoutCallback(I2Cx);
  }

  /* Send STRAT condition */
  I2C_GenerateSTART(I2Cx, ENABLE);

  /* Test on EV5 and clear it */
  i2c_timeout = I2C_SHORT_TIMEOUT;
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
  {
    if (!(i2c_timeout--))
      return i2cworkSyncTimeoutCallback(I2Cx);
  }


  /* Send EEPROM address for write */
  I2C_Send7bitAddress(I2Cx, deviceAddress, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  i2c_timeout = I2C_SHORT_TIMEOUT;
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
    if (!(i2c_timeout--))
    {
      I2C_GenerateSTOP(I2Cx, ENABLE);
      return i2cworkSyncTimeoutCallback(I2Cx);
    }
  }


  /* Send the EEPROM's internal address to write to : only one byte Address */
  I2C_SendData(I2Cx, WriteAddr);

  /* Test on EV8 and clear it */
  i2c_timeout = I2C_LONG_TIMEOUT;
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
    if (!(i2c_timeout--))
    {
      I2C_GenerateSTOP(I2Cx, ENABLE);
      return i2cworkSyncTimeoutCallback(I2Cx);
    }
  }

  /* Send the byte to be written */
  I2C_SendData(I2Cx, *pBuffer);

  /* Test on EV8 and clear it */
  i2c_timeout = I2C_LONG_TIMEOUT;
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
    if (!(i2c_timeout--))
    {
      I2C_GenerateSTOP(I2Cx, ENABLE);
      return i2cworkSyncTimeoutCallback(I2Cx);
    }
  }

  /* Send STOP condition */
  I2C_GenerateSTOP(I2Cx, ENABLE);

#endif

  return 1;
}



/**
 * @brief  This function sets the data pointer on a job
 * @param : job number, pointer to data
 * @retval : None
 */
void I2C1_Setup_Job(uint8_t job_, volatile uint8_t* data)
{
  if(job_<2)
    I2C_jobs[job_].data_pointer=data;
}


void startJob(volatile uint8_t direction,
              volatile uint8_t *pBuffer,
              volatile uint8_t deviceAddr,
              volatile uint8_t addr,
              volatile uint16_t numBytes,
              volatile I2CREQ *i2cReq)
{
  I2C_jobs[0].direction = direction;
  I2C_jobs[0].data_pointer = pBuffer;
  I2C_jobs[0].address = deviceAddr;
  I2C_jobs[0].subaddress = addr;
  I2C_jobs[0].bytes = numBytes;
  I2C_jobs[0].i2cReq = i2cReq;

}

