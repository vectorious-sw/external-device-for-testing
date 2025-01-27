
#include "stm32h7xx.h"

#include <stdlib.h>
#include <math.h>
#include <arm_math.h>
#include <hwdrivers.h>
#include "FreeRTOS.h"
#include "task.h"
#include "vlapConfig.h"
#include "queue.h"
#include "timers.h"
#include "audit.h"
#include "iir.h"
#include "spiflashdisk.h"
#include "protocolapp.h"
#include "spiflash.h"
#include "crc32.h"
#include "rtc.h"

// L O C A L   D E F I N I T I O N S 
#define         SPIFLASHDISK_FSM_STATE_GET spiflashdiskFsmState
#define         SPIFLASHDISK_SPIFLASHDISK_QUEUE_NUMBER_OF_ENTRIES           10
#define         SPIFLASHDISK_SPIFLASHDISK_QUEUE_MAX_ALLOWED_NEW_SPIFLASHDISK  3
#if(SPIFLASHDISK_SPIFLASHDISK_QUEUE_MAX_ALLOWED_NEW_SPIFLASHDISK >= SPIFLASHDISK_SPIFLASHDISK_QUEUE_NUMBER_OF_ENTRIES)
  #error "SPIFLASHDISK_SPIFLASHDISK_QUEUE_NUMBER_OF_ENTRIES must be higher than SPIFLASHDISK_SPIFLASHDISK_QUEUE_MAXIMAL_ALLOWED_NEW_SPIFLASHDISK"
#endif

     

// L O C A L   P R O T O T Y P E S 
void spiflashdiskSpiCompletionCallBack();
void SpiflashdiskFsmTimerTimeoutCallback();


// G L O B A L S
volatile  spiflashdiskFsmStateT spiflashdiskFsmState;
volatile  QueueHandle_t spiflashdiskQueueHandle;
TimerHandle_t SpiflashdiskFsmTimerHandler;
uint8_t SpiFlashDiskBuffer[SPIFLASH_SPI_PAGESIZE];
spiflashdiskCurrentEventDescriptorT spiflashdiskCurrentEventDescriptor; 
 




/******************************************************************************
* @brief  ReturnCode_T spiflashdiskInit()
* @param  
* @retval 
******************************************************************************/
ReturnCode_T spiflashdiskInit()
{
  spiflashdiskQueueEntryT EventRequest;
  
  // Create input queue  
  spiflashdiskQueueHandle = xQueueCreate(SPIFLASHDISK_SPIFLASHDISK_QUEUE_NUMBER_OF_ENTRIES, sizeof(spiflashdiskQueueEntryT));
  // Set the initial scheduler state
  spiflashdiskFsmState = SPIFLASHDISK_FSM_STATE_IDLE;
  // Create the task
  xTaskCreate(spiflashdiskTask, spiflashdiskTaskName, spiflashdiskTaskSTACK_SIZE, NULL, 
              spiflashdiskTaskPriority, ( TaskHandle_t * ) NULL );
  
  // Create timer for the spiflashdisk FSM and start it for the first time
  SpiflashdiskFsmTimerHandler =  xTimerCreate("SpiflashdiskTaskFsmTimer",  portTICK_PERIOD_MS, pdFALSE, (void *)0, SpiflashdiskFsmTimerTimeoutCallback);

  return(RETURNCODE_OK);
}




/******************************************************************************
* @brief  portTASK_FUNCTION(spiflashdiskTask, pvParameters )
* @param  
* @retval 
******************************************************************************/
portTASK_FUNCTION(spiflashdiskTask, pvParameters )
{
  spiflashdiskQueueEntryT QueueEntry;
  spiflashdiskFsmStimuliT Stimuli;
  ReturnCode_T MyReturnCode;
    
  while(1)
  {
    // block task till new queue entry is received, reception timeouts will not be handled 
    BaseType_t  QueueState = xQueueReceive(spiflashdiskQueueHandle, &QueueEntry, 30000 );
    // Convert the queue status to FSM Stimuli
    Stimuli = QueueEntry.FsmStimuli;
    if(QueueState == pdTRUE)
    {
      // Handle the new stimuli
      switch(SPIFLASHDISK_FSM_STATE_GET)   
      {
       case SPIFLASHDISK_FSM_STATE_IDLE:
        switch(Stimuli)
        {
        case SPIFLASHDISK_FSM_STIMULI_WRITE:
 //           hwdriversGpioBitWrite(HWDRIVERS_PB14_USB_TEST, 0);
            // New event received, Check if the log memory is full  
            if(QueueEntry.WriteBufferPtr && QueueEntry.WriteBufferLength)
            {
              spiflashdiskCurrentEventDescriptor.NumberOfPagesToWrite                   = QueueEntry.WriteBufferLength / SPIFLASH_SPI_PAGESIZE;
              spiflashdiskCurrentEventDescriptor.CurrentPage                            = QueueEntry.SpiAddress / SPIFLASH_SPI_PAGESIZE;
              spiflashdiskCurrentEventDescriptor.NumberOfEmptyBytesInFirstPage          = SPIFLASH_SPI_PAGESIZE - (QueueEntry.SpiAddress  % SPIFLASH_SPI_PAGESIZE);
              spiflashdiskCurrentEventDescriptor.NewEventMsgPtr                         = QueueEntry.WriteBufferPtr;
              spiflashdiskCurrentEventDescriptor.AllocatedMessagePtr                    = QueueEntry.WriteBufferPtr;
              spiflashdiskCurrentEventDescriptor.NewEventMsgLength                      = QueueEntry.WriteBufferLength;
              spiflashdiskCurrentEventDescriptor.BytesLeftToBeWritten                   = QueueEntry.WriteBufferLength;
              spiflashdiskCurrentEventDescriptor.CompletionCallBackPtr                  = QueueEntry.CompletionCallBackPtr;     
              // Check if the head pointer is page aligned, Non page aligned requires specific handling for the first page
              if(QueueEntry.SpiAddress % SPIFLASH_SPI_PAGESIZE)
              {
                // Non aligned spi head Ptr, Read the page containing the head ptr
                // Read the page holding the head pointer
                SPIFLASHDISK_FSM_STATE_CHANGE(SPIFLASHDISK_FSM_STATE_SPIREAD_NONALIGNED_FIRST_PAGE_WAIT, 100);
                spiflashReqEnqueue(SPIFLASH_CMD_READ, spiflashdiskCurrentEventDescriptor.CurrentPage * SPIFLASH_SPI_PAGESIZE , (uint8_t*)SpiFlashDiskBuffer, 0, sizeof(SpiFlashDiskBuffer), spiflashdiskSpiCompletionCallBack, false);
              }
              else
              {
                /// aligned head pointer
                if(QueueEntry.WriteBufferLength < SPIFLASH_SPI_PAGESIZE)
                {
                  // New buffer write request aligined to page 
                  memcpy(SpiFlashDiskBuffer, QueueEntry.WriteBufferPtr, QueueEntry.WriteBufferLength);
                  // Aligned Spi Flash head pointer, write first page
                  spiflashReqEnqueue(SPIFLASH_CMD_PROG_VIA_BUFF_1, spiflashdiskCurrentEventDescriptor.CurrentPage * SPIFLASH_SPI_PAGESIZE, 0, (uint8_t*)SpiFlashDiskBuffer, sizeof(SpiFlashDiskBuffer), spiflashdiskSpiCompletionCallBack, false);
                  SPIFLASHDISK_FSM_STATE_CHANGE(SPIFLASHDISK_FSM_STATE_SPIWRITE_SINGLE_ALIGNED_PAGE_COMPLETION, 100);
                }
                else
                {
                  // New buffer write request aligned to page
                  memcpy(SpiFlashDiskBuffer, QueueEntry.WriteBufferPtr, SPIFLASH_SPI_PAGESIZE);
                  SPIFLASHDISK_FSM_STATE_CHANGE(SPIFLASHDISK_FSM_STATE_SPIWRITE_ALL_PAGES_WAIT, 100);
                  // Aligned Spi Flash head pointer, write first page
                  spiflashReqEnqueue(SPIFLASH_CMD_PROG_VIA_BUFF_1, spiflashdiskCurrentEventDescriptor.CurrentPage * SPIFLASH_SPI_PAGESIZE, 0, (uint8_t*)SpiFlashDiskBuffer, sizeof(SpiFlashDiskBuffer), spiflashdiskSpiCompletionCallBack, false);
                  spiflashdiskCurrentEventDescriptor.NewEventMsgPtr += SPIFLASH_SPI_PAGESIZE;
                  spiflashdiskCurrentEventDescriptor.BytesLeftToBeWritten                   = QueueEntry.WriteBufferLength - SPIFLASH_SPI_PAGESIZE;
                }
              }
          }
          break;
        case SPIFLASHDISK_FSM_STIMULI_SPI_COMPLETION:
        case SPIFLASHDISK_FSM_STIMULI_TIMEOUT:
          // Must be a mistake having "non Event" queue item in IDLE state
          // Remove the item and contine 
          xQueueReceive(spiflashdiskQueueHandle, &QueueEntry, 10);
          break;
        }
        break;
        
        
        
        
      case  SPIFLASHDISK_FSM_STATE_SPIREAD_NONALIGNED_FIRST_PAGE_WAIT:
        switch(Stimuli)
        {
        case SPIFLASHDISK_FSM_STIMULI_SPI_COMPLETION:
          {
            // Check if this event entry can fit into the current page or not
            if(spiflashdiskCurrentEventDescriptor.NewEventMsgLength >= spiflashdiskCurrentEventDescriptor.NumberOfEmptyBytesInFirstPage)
            {
              // Fill data till the top of the current page  
              memcpy(SpiFlashDiskBuffer + (SPIFLASH_SPI_PAGESIZE - spiflashdiskCurrentEventDescriptor.NumberOfEmptyBytesInFirstPage), spiflashdiskCurrentEventDescriptor.NewEventMsgPtr, spiflashdiskCurrentEventDescriptor.NumberOfEmptyBytesInFirstPage); 
              spiflashdiskCurrentEventDescriptor.BytesLeftToBeWritten =  spiflashdiskCurrentEventDescriptor.NewEventMsgLength - spiflashdiskCurrentEventDescriptor.NumberOfEmptyBytesInFirstPage;
              spiflashdiskCurrentEventDescriptor.NewEventMsgPtr += spiflashdiskCurrentEventDescriptor.NumberOfEmptyBytesInFirstPage;
            }
            else
            {
              // Log memory head pointer is aligned with spiFlash page
              // Fill the top part of the (First) spiflash page just read with new data from the new message (From head pointer till the next spiFlash page boundry)  
              memcpy(SpiFlashDiskBuffer + (spiflashdiskCurrentEventDescriptor.CurrentSpiFlashAddress%SPIFLASH_SPI_PAGESIZE), spiflashdiskCurrentEventDescriptor.NewEventMsgPtr, spiflashdiskCurrentEventDescriptor.NewEventMsgLength);
              spiflashdiskCurrentEventDescriptor.BytesLeftToBeWritten -= spiflashdiskCurrentEventDescriptor.NewEventMsgLength;
            }
            spiflashReqEnqueue(SPIFLASH_CMD_PROG_VIA_BUFF_1, spiflashdiskCurrentEventDescriptor.CurrentPage * SPIFLASH_SPI_PAGESIZE, 0, (uint8_t*)SpiFlashDiskBuffer, sizeof(SpiFlashDiskBuffer), spiflashdiskSpiCompletionCallBack, false);
            // Write the first page to the spiFlash
            SPIFLASHDISK_FSM_STATE_CHANGE(SPIFLASHDISK_FSM_STATE_SPIWRITE_ALL_PAGES_WAIT, 100);
          } // msg build end
          break;
        case SPIFLASHDISK_FSM_STIMULI_TIMEOUT:
          xQueueReceive(spiflashdiskQueueHandle, &QueueEntry, 10);
          SPIFLASHDISK_FSM_STATE_CHANGE(SPIFLASHDISK_FSM_STATE_IDLE, 100);
          break;
        case SPIFLASHDISK_FSM_STIMULI_WRITE:
          // We do not remove queue spiflashdisk items from queue as we wait for SPI_COMPLETION spiflashdisk to be sent to the from of the queue 
          break;
        }
        break;
        
      case  SPIFLASHDISK_FSM_STATE_SPIWRITE_ALL_PAGES_WAIT:
        switch(Stimuli)
        {
        case SPIFLASHDISK_FSM_STIMULI_SPI_COMPLETION:
          xQueueReceive(spiflashdiskQueueHandle, &QueueEntry, 10);
          // At this point we are page aligned and waiting for prev. page write operation to complete
          // Point to next page
          spiflashdiskCurrentEventDescriptor.CurrentPage++;
          if(spiflashdiskCurrentEventDescriptor.BytesLeftToBeWritten)
          {
            if(spiflashdiskCurrentEventDescriptor.BytesLeftToBeWritten >  SPIFLASH_SPI_PAGESIZE)
            {  
              // Copy page to local memory
              memcpy(SpiFlashDiskBuffer, spiflashdiskCurrentEventDescriptor.NewEventMsgPtr, sizeof(SpiFlashDiskBuffer) );
              // 
              spiflashdiskCurrentEventDescriptor.NewEventMsgPtr += SPIFLASH_SPI_PAGESIZE;
              spiflashdiskCurrentEventDescriptor.BytesLeftToBeWritten -= SPIFLASH_SPI_PAGESIZE;
              // Send to SpiFlash, 
              spiflashReqEnqueue(SPIFLASH_CMD_PROG_VIA_BUFF_1, spiflashdiskCurrentEventDescriptor.CurrentPage * SPIFLASH_SPI_PAGESIZE, 0, (uint8_t*)SpiFlashDiskBuffer, sizeof(SpiFlashDiskBuffer), spiflashdiskSpiCompletionCallBack, false);
              SPIFLASHDISK_FSM_STATE_CHANGE(SPIFLASHDISK_FSM_STATE_SPIWRITE_ALL_PAGES_WAIT, 100);
            }
            else
            {
              // Clear the local page - to 0xff
              memset(SpiFlashDiskBuffer, 0xff, spiflashdiskCurrentEventDescriptor.BytesLeftToBeWritten); 
              // Last page, Copy the BytesLeftToBeWritten to local memory
              memcpy(SpiFlashDiskBuffer, spiflashdiskCurrentEventDescriptor.NewEventMsgPtr, sizeof(SpiFlashDiskBuffer) );
              spiflashReqEnqueue(SPIFLASH_CMD_PROG_VIA_BUFF_1, spiflashdiskCurrentEventDescriptor.CurrentPage * SPIFLASH_SPI_PAGESIZE, 0, (uint8_t*)SpiFlashDiskBuffer, spiflashdiskCurrentEventDescriptor.BytesLeftToBeWritten, spiflashdiskSpiCompletionCallBack, false);
              spiflashdiskCurrentEventDescriptor.BytesLeftToBeWritten = 0;
              SPIFLASHDISK_FSM_STATE_CHANGE(SPIFLASHDISK_FSM_STATE_SPIWRITE_ALL_PAGES_WAIT, 100);
            }
          }
          else
          {
            // Last page was written 
            spiflashdiskCurrentEventDescriptor.CompletionCallBackPtr();
            //hwdriversGpioBitWrite(HWDRIVERS_PB14_USB_TEST, 1);
            SPIFLASHDISK_FSM_STATE_CHANGE(SPIFLASHDISK_FSM_STATE_IDLE, 10);
          }
          break;
        case SPIFLASHDISK_FSM_STIMULI_TIMEOUT:
          SPIFLASHDISK_FSM_STATE_CHANGE(SPIFLASHDISK_FSM_STATE_IDLE, 100);
          break;
        case SPIFLASHDISK_FSM_STIMULI_WRITE:
          // We do not remove queue spiflashdisk items from queue as we wait for SPI_COMPLETION spiflashdisk to be sent to the from of the queue 
          break;
        }
        break;
        
      case  SPIFLASHDISK_FSM_STATE_SPIWRITE_SINGLE_ALIGNED_PAGE_COMPLETION:
        switch(Stimuli)
        {
        case SPIFLASHDISK_FSM_STIMULI_SPI_COMPLETION:
          spiflashdiskCurrentEventDescriptor.CompletionCallBackPtr();
          //hwdriversGpioBitWrite(HWDRIVERS_PB14_USB_TEST, 1);
          SPIFLASHDISK_FSM_STATE_CHANGE(SPIFLASHDISK_FSM_STATE_IDLE, 10);
          break;
        case SPIFLASHDISK_FSM_STIMULI_TIMEOUT:
          SPIFLASHDISK_FSM_STATE_CHANGE(SPIFLASHDISK_FSM_STATE_IDLE, 100);
          break;
        case SPIFLASHDISK_FSM_STIMULI_WRITE:
          // We do not remove queue spiflashdisk items from queue as we wait for SPI_COMPLETION spiflashdisk to be sent to the from of the queue 
          break;
        }
        break;
      } // switch  
    }
    else
    {
    }
  } // while
  
}




/******************************************************************************
* @brief ReturnCode_T SpiflashdiskFsmTimerTimeoutCallback()
* @param  
* @retval 
******************************************************************************/
void SpiflashdiskFsmTimerTimeoutCallback()
{
  spiflashdiskQueueEntryT  SpiflashdiskTaskQueueEntry;
  
  // Fill the queue entry
  SpiflashdiskTaskQueueEntry.FsmStimuli = SPIFLASHDISK_FSM_STIMULI_TIMEOUT;
  SpiflashdiskTaskQueueEntry.WriteBufferPtr   = 0;
  SpiflashdiskTaskQueueEntry.WriteBufferLength = 0;
  
  // Enqueue the event to the commTask input queue
  xQueueSendToFront(spiflashdiskQueueHandle, &SpiflashdiskTaskQueueEntry, 0);
}

/******************************************************************************
* @brief ReturnCode_T spiflashdiskSpiCompletionCallBack()
* @param  
* @retval 
******************************************************************************/
void spiflashdiskSpiCompletionCallBack()
{
  spiflashdiskQueueEntryT  SpiflashdiskTaskQueueEntry;
  
  // Fill the queue entry
  SpiflashdiskTaskQueueEntry.FsmStimuli = SPIFLASHDISK_FSM_STIMULI_SPI_COMPLETION;
  SpiflashdiskTaskQueueEntry.WriteBufferPtr   = 0;
  SpiflashdiskTaskQueueEntry.WriteBufferLength = 0;
      
  // Enqueue the event to the commTask input queue
  xQueueSendToFront(spiflashdiskQueueHandle, &SpiflashdiskTaskQueueEntry, 0);
}



/******************************************************************************
* @brief ReturnCode_T spiflashdiskReqEnqueue(SpiflashReq_T OpCode, uint32_t SpiFlashAddress, uint8_t *RxDestinationPtr, uint8_t *TxSourcePtr, uint16_t OperationBytesCount, QueueHandle_t *CompletionHandlerFunctionPtr, uint8_t FreeTxBufferFlag)

* @param  
* @retval 
******************************************************************************/
ReturnCode_T spiflashdiskReqEnqueue(spiflashdiskFsmStimuliT OpCode, uint32_t SpiFlashAddress, uint8_t *RxDestinationPtr, uint8_t *TxSourcePtr, uint16_t OperationBytesCount, void(*CompletionCallBackPtr)(), uint8_t FreeTxBufferFlag)
{
  spiflashdiskQueueEntryT Request;
  uint16_t i;

   
  // Send spi read request 
  Request.SpiAddress            = SpiFlashAddress;
  Request.WriteBufferPtr        = TxSourcePtr;
  Request.WriteBufferLength     = OperationBytesCount;
  Request.FsmStimuli            = OpCode;
  Request.CompletionCallBackPtr = CompletionCallBackPtr;     
  Request.FreeTxBufferFlag      = FreeTxBufferFlag;
  
  // send the request to spi flash 
  xQueueSend(spiflashdiskQueueHandle, &Request, 0);  
  return RETURNCODE_OK;
}  

