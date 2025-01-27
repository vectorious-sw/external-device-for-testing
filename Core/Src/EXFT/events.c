
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
#include "events.h"
#include "protocolapp.h"
#include "spiflash.h"
#include "crc32.h"
#include "rtc.h"
#include "vlapmain.h"

// L O C A L   D E F I N I T I O N S 
#define         EVENTS_FSM_STATE_GET eventsFsmState
#define         EVENTS_EVENTS_QUEUE_NUMBER_OF_ENTRIES           10
#define         EVENTS_EVENTS_QUEUE_MAX_ALLOWED_NEW_EVENTS  3
#if(EVENTS_EVENTS_QUEUE_MAX_ALLOWED_NEW_EVENTS >= EVENTS_EVENTS_QUEUE_NUMBER_OF_ENTRIES)
  #error "EVENTS_EVENTS_QUEUE_NUMBER_OF_ENTRIES must be higher than EVENTS_EVENTS_QUEUE_MAXIMAL_ALLOWED_NEW_EVENTS"
#endif

#define         EVENTS_FSM_STATE_CHANGE(NewState, QueueWairTimeoutIn10mSec)     \
{                                                                               \
  eventsFsmState=NewState;                                                    \
    xTimerChangePeriod( EventsFsmTimerHandler, QueueWairTimeoutIn10mSec, 100);      \
}                                                                               \
  
  
#define         EVENTS_FSM_STATE_CHANGE_NO_TIMEOUT(NewState)     \
  {                                                                               \
    eventsFsmState=NewState;                                                    \
  }                                                                               \
    
#define         EVENTS_FSM_TIMEOUT_SET(QueueWairTimeoutIn10mSec)                \
    {                                                                             \
      xTimerChangePeriod( EventsFsmTimerHandler, QueueWairTimeoutIn10mSec, 100);      \
    }                                                                             \
      
#define         EVENTS_FSM_STATE_IDLE()                \
      {                                                                             \
        eventsFsmState = EVENTS_FSM_STATE_IDLE; \
        xTimerStop( EventsFsmTimerHandler, 0);      \
      }                                                                             
      
      
      // G L O B A L S
      //#pragma location=".vlapbootFirstTimeAfterJlinkProgrammingSection"
      

//#pragma location=".eventsLogMemoryPointersSection"
eventsLogMemoryPointers_T eventsLogMemoryPointersStructure __attribute__( ( section( "eventsLogMemoryPointersSection") ) );


volatile  eventsFsmStateT eventsFsmState;
volatile  QueueHandle_t eventsQueueHandle;
// Events Queue wait timeout
volatile  uint32_t eventsQueueWaitTimeout;
// This descriptor will hold the last event.  
eventsCurrentEventDescriptorT eventsCurrentEventDescriptor; 
eventsLogMemoryReadWriteOperation_T LogMemoryWriteOperation;
// Events FSM timer, handles the FSM timeouts
TimerHandle_t EventsFsmTimerHandler;
// Indicates if there is an event that is currently in the saving process
eventsPendingStatusT eventsPendingEventsStatus = EVENTS_NO_PENDING_EVENTS;
uint32_t EventsBuildFailCounter;

SemaphoreHandle_t eventsNewEventSavedSemaphoreHandle;


// L O C A L   P R O T O T Y P E S 

uint8_t SpiBuffer[SPIFLASH_SPI_PAGESIZE];
eventsLogMemoryStateBitMap_t LogMemoryStateBitMap;
void eventsSpiCompletionCallBack();
void EventsFsmTimerTimeoutCallback();
void  eventsMemoryStatePrint(uint8_t CallSource, PROTOCOLAPP_COMMANDS_T QueueOpCode);


/******************************************************************************
* @brief  ReturnCode_T eventsInit()
* @param  
* @retval 
******************************************************************************/
ReturnCode_T eventsInit()
{
  eventsQueueEntryT EventRequest;

  // Create input queue  
  eventsQueueHandle = xQueueCreate(EVENTS_EVENTS_QUEUE_NUMBER_OF_ENTRIES, sizeof(eventsQueueEntryT));
  // Set the initial scheduler state
  eventsFsmState = EVENTS_FSM_STATE_POINTERS_RESTORE_WAIT;
  // Sets the initial input queue wait timeout (1 second)
  eventsQueueWaitTimeout = 1000 / portTICK_PERIOD_MS; 
  // Create the task
  xTaskCreate(eventsTask, eventsTaskName, eventsTaskSTACK_SIZE, NULL, 
              eventsPriority, ( TaskHandle_t * ) NULL );
  
  // Create timer for the events FSM and start it for the first time
  EventsFsmTimerHandler =  xTimerCreate("EventsTaskFsmTimer",  portTICK_PERIOD_MS, pdFALSE, (void *)0, EventsFsmTimerTimeoutCallback);

  // xTimerStart(EventsFsmTimerHandler, 100);
  
  eventsLogMemoryPointersStructure.eventsSpiFlashAddressHead           = *(__IO uint32_t *) (D3_BKPSRAM_BASE + 0);
  eventsLogMemoryPointersStructure.eventsSpiFlashAddressHeadPrev       = *(__IO uint32_t *) (D3_BKPSRAM_BASE + 4);
  eventsLogMemoryPointersStructure.eventsSpiFlashAddressTail           = *(__IO uint32_t *) (D3_BKPSRAM_BASE + 8);
  eventsLogMemoryPointersStructure.eventsPendingMessagesInLogMemory    = *(__IO uint32_t *) (D3_BKPSRAM_BASE + 12);
  eventsLogMemoryPointersStructure.eventsFailedTxMessagesCounter       = *(__IO uint32_t *) (D3_BKPSRAM_BASE + 16);
  eventsLogMemoryPointersStructure.eventsTotalStoredNumberCounter      = *(__IO uint32_t *) (D3_BKPSRAM_BASE + 20);
  eventsLogMemoryPointersStructure.Crc32                               = *(__IO uint32_t *) (D3_BKPSRAM_BASE + 24);

  //HAL_PWR_DisableBkUpAccess();

  // Calculate the CRC32 BCC for the structure read 
  uint32_t CalculatedCrc32 = crc32BuffCalc( (uint8_t*) &eventsLogMemoryPointersStructure, 0, sizeof(eventsLogMemoryPointersStructure)-4);
  // Confirm the stored Crc32 matches the calculated, if not, try reading from the spiFlash systemPagse
  if( CalculatedCrc32 != eventsLogMemoryPointersStructure.Crc32)
  {
    // CRC32 failed error msg
    eventsMemoryStatePrint(2, 0xffff);
    // Read the last saved pointers from the nonvolatile memory
    spiflashReqEnqueue(SPIFLASH_CMD_READ, SPIFLASH_SYSTEM_START, (uint8_t*)&eventsLogMemoryPointersStructure, 0, sizeof(eventsLogMemoryPointersStructure), eventsSpiCompletionCallBack, false);
  }
  else
  {
    // RTC backup memory, Log memory pointers Crc32 OK 
    eventsMemoryStatePrint(3, 0xffff);
    // Log memory pointers ok, erase the SPI Flash pointers so it would be prepare for the PVD write
    eventsLogMemoryPointersToNvmErase();
    // Memory pointers are ok, safe to got to IDLE state
    EventRequest.FsmStimuli      = EVENTS_LOG_MEM_POINTERS_RESTORED; 
    EventRequest.Channel         = 0;
    EventRequest.messageOpCode   = 0;   
    EventRequest.PayloadLength   = 0;
    EventRequest.PayloadPtr      = 0;
    EventRequest.PayloadMemFreeRequired = 0;
    EventRequest.RequrstOrCommandSessionId = 0;
    xQueueSendToBack(eventsQueueHandle, &EventRequest, 0);
  }
  
  LogMemoryStateBitMap.LogMemoryTail1Operating = true;
  LogMemoryStateBitMap.LogMemoryTail2Operating = true;
  
  eventsNewEventSavedSemaphoreHandle = xSemaphoreCreateBinary();

  EventsBuildFailCounter = 0;
  return(RETURNCODE_OK);
}


void eventsJlinkProgrammaingLogMemoryPointersSet()
{
  eventsLogMemoryPointersStructure.eventsSpiFlashAddressHead = SPIFLASH_ZONE_EVENTS_START; 
  eventsLogMemoryPointersStructure.eventsSpiFlashAddressTail = SPIFLASH_ZONE_EVENTS_START;
  eventsLogMemoryPointersStructure.eventsPendingMessagesInLogMemory = 0;
  eventsLogMemoryPointersStructure.eventsFailedTxMessagesCounter = 0;
  eventsLogMemoryPointersStructure.eventsTotalStoredNumberCounter = 0;
  eventsLogMemoryPointersStructure.eventsSpiFlashAddressHeadPrev = 0;
  eventsLogMemoryPointersStructure.Crc32 = crc32BuffCalc( (uint8_t*) &eventsLogMemoryPointersStructure, 0, sizeof(eventsLogMemoryPointersStructure)-4);
  
  taskENTER_CRITICAL();
  
  *(__IO uint32_t *) (D3_BKPSRAM_BASE + 0)  = eventsLogMemoryPointersStructure.eventsSpiFlashAddressHead;
  *(__IO uint32_t *) (D3_BKPSRAM_BASE + 4)  = eventsLogMemoryPointersStructure.eventsSpiFlashAddressHeadPrev;
  *(__IO uint32_t *) (D3_BKPSRAM_BASE + 8)  = eventsLogMemoryPointersStructure.eventsSpiFlashAddressTail;
  *(__IO uint32_t *) (D3_BKPSRAM_BASE + 12) = eventsLogMemoryPointersStructure.eventsPendingMessagesInLogMemory;
  *(__IO uint32_t *) (D3_BKPSRAM_BASE + 16) = eventsLogMemoryPointersStructure.eventsFailedTxMessagesCounter;
  *(__IO uint32_t *) (D3_BKPSRAM_BASE + 20) = eventsLogMemoryPointersStructure.eventsTotalStoredNumberCounter;
  *(__IO uint32_t *) (D3_BKPSRAM_BASE + 24) = eventsLogMemoryPointersStructure.Crc32;

  //HAL_PWR_DisableBkUpAccess();
  
  taskEXIT_CRITICAL();
}


/******************************************************************************
* @brief  portTASK_FUNCTION(eventsTask, pvParameters )
* @param  
* @retval 
******************************************************************************/
portTASK_FUNCTION(eventsTask, pvParameters )
{
  eventsQueueEntryT QueueEntry;
  eventsFsmStimuliT Stimuli;
  uint8_t *ReturnedMsgPtr = 0;
  uint8_t ReturnedMsgPad = 0;
  uint16_t ReturnedMsgLength = 0;
  ReturnCode_T MyReturnCode;
  eventsEventsLogMemoryStateT eventsEventsLogMemoryState;
  uint16_t eventsReturnedEncryptedMsgLength;
  uint32_t CalculatedCrc32;
  
  while(1)
  {
    // block task till new queue entry is received, reception timeouts will not be handled 
    BaseType_t  QueueState = xQueuePeek(eventsQueueHandle, &QueueEntry, 30000 );
    // Convert the queue status to FSM Stimuli
    Stimuli = QueueEntry.FsmStimuli;
    if(QueueState == pdTRUE)
    {
      // Handle the new stimuli
      switch(eventsFsmState)
      {
      case EVENTS_FSM_STATE_POINTERS_RESTORE_WAIT:
        // Remove the queue item from queue
        xQueueReceive(eventsQueueHandle, &QueueEntry, 10);
        switch(Stimuli)
        {
        case EVENTS_LOG_MEM_POINTERS_RESTORED:
          EVENTS_FSM_STATE_CHANGE_NO_TIMEOUT(EVENTS_FSM_STATE_IDLE);
          break;
        case EVENTS_FSM_STIMULI_SPI_COMPLETION:
        	// Calculate the CRC32 BCC for the structure read
        	CalculatedCrc32 = crc32BuffCalc( (uint8_t*) &eventsLogMemoryPointersStructure, 0, sizeof(eventsLogMemoryPointersStructure)-4);
        	// Confirm the stored Crc32 matches the calculated, if not, clear the pointers, calc Crc32 and store into the backup SRAM
          if( CalculatedCrc32 != eventsLogMemoryPointersStructure.Crc32)
          {
            eventsMemoryStatePrint(4, 0xffff);
            eventsJlinkProgrammaingLogMemoryPointersSet();
            // Reset time and time
            rtcResetDateTime();
          }
          else
          {
            eventsMemoryStatePrint(5, 0xffff);
          }
          // Erase the SPI Flash pointers so it would be prepare for the PVD write
          eventsLogMemoryPointersToNvmErase();
          EVENTS_FSM_STATE_CHANGE_NO_TIMEOUT(EVENTS_FSM_STATE_IDLE);
          break;
        deafult:
          break;
        }
        break;
        
      case EVENTS_FSM_STATE_IDLE:
        switch(Stimuli)
        {
        case EVENTS_FSM_STIMULI_NEW_EVENT:
          // Remove the queue item from queue
          xQueueReceive(eventsQueueHandle, &QueueEntry, 10);
          // ***
//          hwdriversGpioBitWrite(HWDRIVERS_PB15_USB_TEST, 0);
          // New event received, Check if the log memory is full  
          if( eventsLogMemoryFullCheck(QueueEntry.PayloadLength) == RETURNCODE_LOGMEMORY_NOTFULL)
          {
            eventsMemoryStatePrint(0, QueueEntry.messageOpCode);
            // Use the eventsCurrentEventDescriptor to Allocate memory and fill it with relevant protocol fields, the buffer includes the SPI buffer header
            MyReturnCode = protocolappMessageBuild(QueueEntry.Channel, QueueEntry.RequrstOrCommandSessionId, QueueEntry.messageOpCode, QueueEntry.PayloadPtr, QueueEntry.PayloadLength, QueueEntry.PayloadMemFreeRequired ,&ReturnedMsgPtr, &ReturnedMsgLength, &ReturnedMsgPad);
            if (MyReturnCode == RETURNCODE_OK)
            {
              // Encrypt the message before saving it in the flash
              ReturnCode_T ReturnCodeEncrypt = aescbcEncryptByChunks(ReturnedMsgPtr + sizeof(spiflashLogMemoryHeaderT), ReturnedMsgLength - sizeof(spiflashLogMemoryHeaderT) - sizeof(aescbcProtcolHeader_t) - ReturnedMsgPad, &eventsReturnedEncryptedMsgLength);
              eventsReturnedEncryptedMsgLength += sizeof(spiflashLogMemoryHeaderT);
              // Check if the message build was successful and it returned the pointer, No need to free memory if return code is not OK
              if((ReturnCodeEncrypt == RETURNCODE_OK) && eventsReturnedEncryptedMsgLength && ReturnedMsgPtr)
              {
                // Calculate CRC on decrypted event
                ((spiflashLogMemoryHeaderT*)ReturnedMsgPtr)->CRC32 = crc32BuffCalc((uint8_t*)(ReturnedMsgPtr + sizeof(spiflashLogMemoryHeaderT)), 0, ReturnedMsgLength - sizeof(spiflashLogMemoryHeaderT));

                eventsCurrentEventDescriptor.NumberOfPagesToWrite                   = eventsReturnedEncryptedMsgLength / SPIFLASH_SPI_PAGESIZE;
                eventsCurrentEventDescriptor.CurrentPage                            = (eventsLogMemoryPointersStructure.eventsSpiFlashAddressHead - SPIFLASH_ZONE_EVENTS_START) / SPIFLASH_SPI_PAGESIZE;
                eventsCurrentEventDescriptor.NumberOfEmptyBytesInFirstPage          = SPIFLASH_SPI_PAGESIZE - (eventsLogMemoryPointersStructure.eventsSpiFlashAddressHead % SPIFLASH_SPI_PAGESIZE);
                eventsCurrentEventDescriptor.NewEventMsgPtr                         = ReturnedMsgPtr;
                eventsCurrentEventDescriptor.AllocatedMessagePtr                    = ReturnedMsgPtr;
                eventsCurrentEventDescriptor.NewEventMsgLength                      = eventsReturnedEncryptedMsgLength;
                eventsCurrentEventDescriptor.BytesLeftToBeWritten                   = eventsReturnedEncryptedMsgLength;
                // Check if the head pointer is page aligned, Non page aligned requires specific handling for the first page
                if(eventsLogMemoryPointersStructure.eventsSpiFlashAddressHead % SPIFLASH_SPI_PAGESIZE)
                {
                  // Non aligned spi head Ptr, Read the page containing the head ptr
                  // Read the page holding the head pointer
                  EVENTS_FSM_STATE_CHANGE(EVENTS_FSM_STATE_SPIREAD_NONALIGNED_FIRST_PAGE_WAIT, 100);
                  spiflashReqEnqueue(SPIFLASH_CMD_READ, eventsCurrentEventDescriptor.CurrentPage * SPIFLASH_SPI_PAGESIZE , (uint8_t*)SpiBuffer, 0, sizeof(SpiBuffer), eventsSpiCompletionCallBack, false);
                }
                else
                {
                  /// aligned head pointer
                  if(eventsReturnedEncryptedMsgLength < SPIFLASH_SPI_PAGESIZE)
                  {
                    // New buffer write request aligned to page
                    memcpy(SpiBuffer, ReturnedMsgPtr, eventsReturnedEncryptedMsgLength);
                    // Aligned Spi Flash head pointer, write first page
                    spiflashReqEnqueue(SPIFLASH_CMD_PROG_VIA_BUFF_1, eventsCurrentEventDescriptor.CurrentPage * SPIFLASH_SPI_PAGESIZE, 0, (uint8_t*)SpiBuffer, sizeof(SpiBuffer), eventsSpiCompletionCallBack, false);
                    EVENTS_FSM_STATE_CHANGE(EVENTS_FSM_STATE_SPIWRITE_SINGLE_ALIGNED_PAGE_COMPLETION, 100);
                  }
                  else
                  {
                    // New buffer write request aligned to page
                    memcpy(SpiBuffer, ReturnedMsgPtr, SPIFLASH_SPI_PAGESIZE);
                    EVENTS_FSM_STATE_CHANGE(EVENTS_FSM_STATE_SPIWRITE_ALL_PAGES_WAIT, 100);
                    // Aligned Spi Flash head pointer, write first page
                    spiflashReqEnqueue(SPIFLASH_CMD_PROG_VIA_BUFF_1, eventsCurrentEventDescriptor.CurrentPage * SPIFLASH_SPI_PAGESIZE, 0, (uint8_t*)SpiBuffer, sizeof(SpiBuffer), eventsSpiCompletionCallBack, false);
                    eventsCurrentEventDescriptor.NewEventMsgPtr += SPIFLASH_SPI_PAGESIZE;
                    eventsCurrentEventDescriptor.BytesLeftToBeWritten                   = eventsReturnedEncryptedMsgLength - SPIFLASH_SPI_PAGESIZE;
                  }
                }
              }
            }
            else
            	EventsBuildFailCounter++;
          }
          else
            eventsPendingEventsStatus = EVENTS_NO_PENDING_EVENTS;
          break;
        case EVENTS_FSM_STIMULI_SPI_COMPLETION:
        case EVENTS_FSM_STIMULI_TIMEOUT:
          // Must be a mistake having "non Event" queue item in IDLE state
          // Remove the item and continue
          eventsPendingEventsStatus = EVENTS_NO_PENDING_EVENTS;
          xQueueReceive(eventsQueueHandle, &QueueEntry, 10);
          break;
        default:
        	 xQueueReceive(eventsQueueHandle, &QueueEntry, 10);
        	break;
        }
        break;
        
        
        
        
      case  EVENTS_FSM_STATE_SPIREAD_NONALIGNED_FIRST_PAGE_WAIT:
        switch(Stimuli)
        {
        case EVENTS_FSM_STIMULI_SPI_COMPLETION:
          {
            xQueueReceive(eventsQueueHandle, &QueueEntry, 10);
            // Check if this event entry can fit into the current page or not
            if(eventsCurrentEventDescriptor.NewEventMsgLength >= eventsCurrentEventDescriptor.NumberOfEmptyBytesInFirstPage)
            {
              // Fill data till the top of the current page  
              memcpy(SpiBuffer + (SPIFLASH_SPI_PAGESIZE - eventsCurrentEventDescriptor.NumberOfEmptyBytesInFirstPage), eventsCurrentEventDescriptor.NewEventMsgPtr, eventsCurrentEventDescriptor.NumberOfEmptyBytesInFirstPage); 
              eventsCurrentEventDescriptor.BytesLeftToBeWritten =  eventsCurrentEventDescriptor.NewEventMsgLength - eventsCurrentEventDescriptor.NumberOfEmptyBytesInFirstPage;
              eventsCurrentEventDescriptor.NewEventMsgPtr += eventsCurrentEventDescriptor.NumberOfEmptyBytesInFirstPage;
            }
            else
            {
              // Log memory head pointer is aligned with spiFlash page
              // Fill the top part of the (First) spiflash page just read with new data from the new message (From head pointer till the next spiFlash page boundry)  
              memcpy(SpiBuffer + (eventsLogMemoryPointersStructure.eventsSpiFlashAddressHead%SPIFLASH_SPI_PAGESIZE), eventsCurrentEventDescriptor.NewEventMsgPtr, eventsCurrentEventDescriptor.NewEventMsgLength);
              eventsCurrentEventDescriptor.BytesLeftToBeWritten -= eventsCurrentEventDescriptor.NewEventMsgLength;
            }
            spiflashReqEnqueue(SPIFLASH_CMD_PROG_VIA_BUFF_1, eventsCurrentEventDescriptor.CurrentPage * SPIFLASH_SPI_PAGESIZE, 0, (uint8_t*)SpiBuffer, sizeof(SpiBuffer), eventsSpiCompletionCallBack, false);
            // Write the first page to the spiFlash
            EVENTS_FSM_STATE_CHANGE(EVENTS_FSM_STATE_SPIWRITE_ALL_PAGES_WAIT, 100);
          } // msg build end
          break;
        case EVENTS_FSM_STIMULI_TIMEOUT:
            if(eventsCurrentEventDescriptor.AllocatedMessagePtr)
              vPortFree(eventsCurrentEventDescriptor.AllocatedMessagePtr);
            vlapmainDebugLog(" timeout in EVENTS_FSM_STATE_SPIREAD_NONALIGNED_FIRST_PAGE_WAIT, free allocated memory");
          eventsPendingEventsStatus = EVENTS_NO_PENDING_EVENTS;
          xQueueReceive(eventsQueueHandle, &QueueEntry, 10);
          //EVENTS_FSM_STATE_CHANGE(EVENTS_FSM_STATE_IDLE, 100);
          EVENTS_FSM_STATE_IDLE();
          break;
        case EVENTS_FSM_STIMULI_NEW_EVENT:
          // We do not remove queue events items from queue as we wait for SPI_COMPLETION events to be sent to the from of the queue 
          break;
        default:
          break;

        }
        break;
        
      case  EVENTS_FSM_STATE_SPIWRITE_ALL_PAGES_WAIT:
        switch(Stimuli)
        {
        case EVENTS_FSM_STIMULI_SPI_COMPLETION:
          xQueueReceive(eventsQueueHandle, &QueueEntry, 10);
          // At this point we are page aligned and waiting for prev. page write operation to complete
          // Point to next page
          eventsCurrentEventDescriptor.CurrentPage++;
          // Handle crossing the top of the reagon allocated for events in the SPI flash 
          if(eventsCurrentEventDescriptor.CurrentPage > (SPIFLASH_ZONE_EVENTS_START + SPIFLASH_ZONE_EVENTS_SIZE))
            eventsCurrentEventDescriptor.CurrentPage = SPIFLASH_ZONE_EVENTS_START;
          if(eventsCurrentEventDescriptor.BytesLeftToBeWritten)
          {
            if(eventsCurrentEventDescriptor.BytesLeftToBeWritten >  SPIFLASH_SPI_PAGESIZE)
            {  
              // Copy page to local memory
              memcpy(SpiBuffer, eventsCurrentEventDescriptor.NewEventMsgPtr, sizeof(SpiBuffer) );
              // 
              eventsCurrentEventDescriptor.NewEventMsgPtr += SPIFLASH_SPI_PAGESIZE;
              eventsCurrentEventDescriptor.BytesLeftToBeWritten -= SPIFLASH_SPI_PAGESIZE;
              // Send to SpiFlash, 
              spiflashReqEnqueue(SPIFLASH_CMD_PROG_VIA_BUFF_1, eventsCurrentEventDescriptor.CurrentPage * SPIFLASH_SPI_PAGESIZE, 0, (uint8_t*)SpiBuffer, sizeof(SpiBuffer), eventsSpiCompletionCallBack, false);
              EVENTS_FSM_STATE_CHANGE(EVENTS_FSM_STATE_SPIWRITE_ALL_PAGES_WAIT, 100);
            }
            else
            {
              // Clear the local page - to 0xff
              memset(SpiBuffer, 0xff, eventsCurrentEventDescriptor.BytesLeftToBeWritten); 
              // Last page, Copy the BytesLeftToBeWritten to local memory
              memcpy(SpiBuffer, eventsCurrentEventDescriptor.NewEventMsgPtr, sizeof(SpiBuffer) );
              spiflashReqEnqueue(SPIFLASH_CMD_PROG_VIA_BUFF_1, eventsCurrentEventDescriptor.CurrentPage * SPIFLASH_SPI_PAGESIZE, 0, (uint8_t*)SpiBuffer, eventsCurrentEventDescriptor.BytesLeftToBeWritten, eventsSpiCompletionCallBack, false);
              eventsCurrentEventDescriptor.BytesLeftToBeWritten = 0;
              EVENTS_FSM_STATE_CHANGE(EVENTS_FSM_STATE_SPIWRITE_ALL_PAGES_WAIT, 100);
            }
          }
          else
          {
            // Last page was written 
            // Handle head pointer increment at the last page
            eventsQueuePointerIndexUpdate(EVENTS_LOGMEMORY_HEAD, &eventsEventsLogMemoryState, eventsCurrentEventDescriptor.NewEventMsgLength, false);
            // Free the allocated event block
            if(eventsCurrentEventDescriptor.AllocatedMessagePtr)
              vPortFree(eventsCurrentEventDescriptor.AllocatedMessagePtr);
            vlapmainDebugLog(" Free allocated memory after event has been written to SPI");
            // ***
            eventsPendingEventsStatus = EVENTS_NO_PENDING_EVENTS;
            xSemaphoreGive(eventsNewEventSavedSemaphoreHandle);
//            hwdriversGpioBitWrite(HWDRIVERS_PB15_USB_TEST, 1);
            //EVENTS_FSM_STATE_CHANGE(EVENTS_FSM_STATE_IDLE, /*10*/ 500);
            EVENTS_FSM_STATE_IDLE();
          }
          break;
        case EVENTS_FSM_STIMULI_TIMEOUT:
          eventsPendingEventsStatus = EVENTS_NO_PENDING_EVENTS;
          if(eventsCurrentEventDescriptor.AllocatedMessagePtr)
            vPortFree(eventsCurrentEventDescriptor.AllocatedMessagePtr);
          vlapmainDebugLog(" SpiFlash write timeout, EVENTS_FSM_STATE_SPIWRITE_ALL_PAGES_WAIT, Free allocated memory");
          EVENTS_FSM_STATE_IDLE();
          break;
        case EVENTS_FSM_STIMULI_NEW_EVENT:
          // We do not remove queue events items from queue as we wait for SPI_COMPLETION events to be sent to the from of the queue 
          break;
        default:
        	break;
        }
        break;
        
      case  EVENTS_FSM_STATE_SPIWRITE_SINGLE_ALIGNED_PAGE_COMPLETION:
        switch(Stimuli)
        {
        case EVENTS_FSM_STIMULI_SPI_COMPLETION:
          xQueueReceive(eventsQueueHandle, &QueueEntry, 10);
          // Last page was written 
          // Handle head pointer increment at the last page
          eventsQueuePointerIndexUpdate(EVENTS_LOGMEMORY_HEAD, &eventsEventsLogMemoryState, eventsCurrentEventDescriptor.NewEventMsgLength, false);
          // Free the allocated event block
          if(eventsCurrentEventDescriptor.AllocatedMessagePtr)
            vPortFree(eventsCurrentEventDescriptor.AllocatedMessagePtr);
          //            vlapmainDebugLog(" Free allocated memory after event has been written to SPI");
          // ***
          eventsPendingEventsStatus = EVENTS_NO_PENDING_EVENTS;
          xSemaphoreGive(eventsNewEventSavedSemaphoreHandle);
//          hwdriversGpioBitWrite(HWDRIVERS_PB15_USB_TEST, 1);
          //EVENTS_FSM_STATE_CHANGE(EVENTS_FSM_STATE_IDLE, 10);
          EVENTS_FSM_STATE_IDLE();
          break;
        case EVENTS_FSM_STIMULI_TIMEOUT:
          eventsPendingEventsStatus = EVENTS_NO_PENDING_EVENTS;
          if(eventsCurrentEventDescriptor.AllocatedMessagePtr)
            vPortFree(eventsCurrentEventDescriptor.AllocatedMessagePtr);
          vlapmainDebugLog(" timeout in EVENTS_FSM_STATE_SPIWRITE_SINGLE_ALIGNED_PAGE_COMPLETION, free allocated memory");
           //EVENTS_FSM_STATE_CHANGE(EVENTS_FSM_STATE_IDLE, 100);
          EVENTS_FSM_STATE_IDLE();
          break;
        case EVENTS_FSM_STIMULI_NEW_EVENT:
          // We do not remove queue events items from queue as we wait for SPI_COMPLETION events to be sent to the from of the queue 
          break;
        default:
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
* @brief ReturnCode_T eventsEventWrite(uartdllChannel_t Channel, uint32_t RequrstOrCommandSessionId, PROTOCOLAPP_COMMANDS_T messageOpCode, char * PayloadPtr, uint16_t PayloadLength, uint8_t PayloadMemFreeRequired)
* @param  
* @retval 
******************************************************************************/
ReturnCode_T eventsEventWrite(uartdllChannel_t Channel, uint32_t RequrstOrCommandSessionId, PROTOCOLAPP_COMMANDS_T messageOpCode, char * PayloadPtr, uint16_t PayloadLength, uint8_t PayloadMemFreeRequired)
{
  eventsQueueEntryT EventRequest;
  uint8_t *ReturnedMsgPtr = 0;
  ReturnCode_T MyReturnCode = RETURNCODE_ERROR;
  
  // TODO: 
  // return(RETURNCODE_ERROR);
  
  if( eventsLogMemoryFullCheck(PayloadLength) == RETURNCODE_LOGMEMORY_NOTFULL)
  {
    // Fill the request queue entry
    EventRequest.FsmStimuli      = EVENTS_FSM_STIMULI_NEW_EVENT; 
    EventRequest.Channel         = Channel;
    EventRequest.messageOpCode   = messageOpCode;   
    EventRequest.PayloadLength   = PayloadLength;
    EventRequest.PayloadPtr      = PayloadPtr;
    EventRequest.PayloadMemFreeRequired = PayloadMemFreeRequired;
    EventRequest.RequrstOrCommandSessionId = RequrstOrCommandSessionId;
    // ***
//    hwdriversGpioBitWrite(HWDRIVERS_PB15_USB_TEST,1);
    // send the request to the events queue back 
    if (uxQueueSpacesAvailable(eventsQueueHandle) > EVENTS_EVENTS_QUEUE_MAX_ALLOWED_NEW_EVENTS)
    {
      eventsPendingEventsStatus = EVENTS_PENDING_EVENTS;
      xQueueSendToBack(eventsQueueHandle, &EventRequest, 0);
      MyReturnCode = RETURNCODE_OK;
    }
    else
    {
      
      vlapmainDebugLog(">>>>>> Events queue full\n\r");
      // Free the allocated memory if the queue send failed, this message is lost
      vPortFree(ReturnedMsgPtr);
      // Increment Lost Tx message counter.
      eventsLogMemoryPointersStructure.eventsFailedTxMessagesCounter++;
      // Return Error 
      MyReturnCode = RETURNCODE_ERROR;
    }
  }
    
#if 0
    for(uint16_t i=0; i< PayloadLength+16; i+=16)
    {
      uint8_t *  Ptr = pvPortMalloc(200);
      uint16_t MyLength = sprintf(Ptr, "SpiPage = %05x,  %05x ** %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \r\n", EventRequest.PayloadPtr, i, EventRequest.PayloadPtr[i], EventRequest.PayloadPtr[i+1], EventRequest.PayloadPtr[i+2], EventRequest.PayloadPtr[i+3], EventRequest.PayloadPtr[i+4], EventRequest.PayloadPtr[i+5], EventRequest.PayloadPtr[i+6], EventRequest.PayloadPtr[i+7], EventRequest.PayloadPtr[i+8], EventRequest.PayloadPtr[i+9], EventRequest.PayloadPtr[i+10], EventRequest.PayloadPtr[i+11], EventRequest.PayloadPtr[i+12], EventRequest.PayloadPtr[i+13], EventRequest.PayloadPtr[i+14], EventRequest.PayloadPtr[i+15]);
      uartdllTxQueueEnqueue(UARTDLL_UART_6_RS485, Ptr, MyLength, true);
      vTaskDelay(10);
    }
#endif
    
   return(MyReturnCode);
  
}



uint16_t eventsEventInProgressStateGet()
{
  return( uxQueueMessagesWaiting(eventsQueueHandle));
}



/******************************************************************************
* @brief  ReturnCode_T eventsQueueStatusAndTailEntryAddressGet(eventsEventsLogMemoryStateT *EventsLogMemoryStatePtr, uint32_t  *QueueEntryPtr )
* @param  
* @retval 
******************************************************************************/
ReturnCode_T eventsQueueStatusAndTailEntryAddressGet(eventsEventsLogMemoryStateT *EventsLogMemoryStatePtr, uint32_t *QueueEntryPtr )
{
  if(LogMemoryStateBitMap.LogMemoryTail1Operating)
  {
    if(eventsLogMemoryPointersStructure.eventsSpiFlashAddressHead == eventsLogMemoryPointersStructure.eventsSpiFlashAddressTail)
    {
      *EventsLogMemoryStatePtr = EVENTS_QUEUESTATE_EMPTY;
      *QueueEntryPtr = 0;
//      eventsLogMemoryPointersStructure.eventsPendingMessagesInLogMemory = 0;
      //      *(__IO uint32_t *) (D3_BKPSRAM_BASE_BASE + 12) = eventsPendingMessagesInLogMemory;
    }
    else
    {
      // Return back NOT EMPTY and point the returned address to the new tail location
      *EventsLogMemoryStatePtr = EVENTS_QUEUESTATE_NOTEMPTY;
      *QueueEntryPtr = eventsLogMemoryPointersStructure.eventsSpiFlashAddressTail;
    }
  }
  else
    *EventsLogMemoryStatePtr = EVENTS_QUEUESTATE_EMPTY;
  
  return(RETURNCODE_OK);
}






uint8_t pointersUpdateLog[100];
/******************************************************************************
* @brief  ReturnCode_T eventsQueuePointerIndexUpdate(eventsLogMemoryPointerTypeT LogMemoryPointerType, eventsEventsLogMemoryStateT *EventsLogMemoryStatePtr, uint32_t NewEntrySize)
* @param  
* @retval 
******************************************************************************/
ReturnCode_T eventsQueuePointerIndexUpdate(eventsLogMemoryPointerTypeT LogMemoryPointerType, eventsEventsLogMemoryStateT *EventsLogMemoryStatePtr, uint32_t NewEntrySize, bool eventSkipped)
{
  uint32_t *LogMemoryIndexPtr;
  
  taskENTER_CRITICAL();
  
  switch(LogMemoryPointerType)
  {
  case EVENTS_LOGMEMORY_HEAD:
    LogMemoryIndexPtr = &eventsLogMemoryPointersStructure.eventsSpiFlashAddressHead;
    // Prev head before incrementing, used for back reference prev entry in the log memory 
    eventsLogMemoryPointersStructure.eventsSpiFlashAddressHeadPrev = eventsLogMemoryPointersStructure.eventsSpiFlashAddressHead;
    // Increment log messages counter
    eventsLogMemoryPointersStructure.eventsPendingMessagesInLogMemory++;
    // Count total messages
    eventsLogMemoryPointersStructure.eventsTotalStoredNumberCounter++;
    break;
  case EVENTS_LOGMEMORY_TAIL:
    LogMemoryIndexPtr = &eventsLogMemoryPointersStructure.eventsSpiFlashAddressTail;
    // Decrement log messages counter
    if(eventsLogMemoryPointersStructure.eventsPendingMessagesInLogMemory)
    {
      eventsLogMemoryPointersStructure.eventsPendingMessagesInLogMemory--;
      if (eventSkipped)
        eventsLogMemoryPointersStructure.eventsPendingMessagesInLogMemory--;
    }
    break;
  }
  
  // Point to the next entry and handle log event  pointer boundary cases.
  sprintf((char*)pointersUpdateLog, "Update type = %ld, NewEntrySize = %d", LogMemoryPointerType, NewEntrySize);
  vlapmainDebugLog(pointersUpdateLog);
  
  *LogMemoryIndexPtr += NewEntrySize;
  if(*LogMemoryIndexPtr >= SPIFLASH_ZONE_EVENTS_END)
  {
    *LogMemoryIndexPtr = (SPIFLASH_ZONE_EVENTS_START + (NewEntrySize - (SPIFLASH_ZONE_EVENTS_END - (*LogMemoryIndexPtr - NewEntrySize)) ));
  }
  
  *(__IO uint32_t *) (D3_BKPSRAM_BASE + 0 ) = eventsLogMemoryPointersStructure.eventsSpiFlashAddressHead;
  *(__IO uint32_t *) (D3_BKPSRAM_BASE + 4 ) = eventsLogMemoryPointersStructure.eventsSpiFlashAddressHeadPrev;
  *(__IO uint32_t *) (D3_BKPSRAM_BASE + 8 ) = eventsLogMemoryPointersStructure.eventsSpiFlashAddressTail;
  *(__IO uint32_t *) (D3_BKPSRAM_BASE + 12) = eventsLogMemoryPointersStructure.eventsPendingMessagesInLogMemory;
  *(__IO uint32_t *) (D3_BKPSRAM_BASE + 16) = eventsLogMemoryPointersStructure.eventsFailedTxMessagesCounter;
  *(__IO uint32_t *) (D3_BKPSRAM_BASE + 20) = eventsLogMemoryPointersStructure.eventsTotalStoredNumberCounter;

  uint32_t MyCrc32 = crc32BuffCalc( (uint8_t*) &eventsLogMemoryPointersStructure, 0, sizeof(eventsLogMemoryPointersStructure)-4);
  eventsLogMemoryPointersStructure.Crc32 = MyCrc32;
  *(__IO uint32_t *) (D3_BKPSRAM_BASE + 24) = eventsLogMemoryPointersStructure.Crc32;

  //HAL_PWR_DisableBkUpAccess();
  taskEXIT_CRITICAL();

  switch(LogMemoryPointerType)
    {
    case EVENTS_LOGMEMORY_HEAD:
    	  //The OpCode field is not used
    	  eventsMemoryStatePrint(6, 0);
     break;
    case EVENTS_LOGMEMORY_TAIL:
      break;
    }

  
  return(RETURNCODE_OK);
}


/******************************************************************************
* @brief  uint32_t eventsQueueTailIndexGet()
* @param  
* @retval 
******************************************************************************/
uint32_t eventsQueueTailIndexGet()
{
  return(eventsLogMemoryPointersStructure.eventsSpiFlashAddressTail);
}


/******************************************************************************
* @brief  uint32_t eventsQueuePrevHeadIndexGet()
* @param  
* @retval 
******************************************************************************/
uint32_t eventsQueuePrevHeadIndexGet()
{
  return(eventsLogMemoryPointersStructure.eventsSpiFlashAddressHeadPrev);
}

/******************************************************************************
* @brief  uint32_t eventsPendingEventsStatusGet()
* @param  
* @retval 
******************************************************************************/
uint32_t eventsPendingEventsStatusGet()
{
  //return(eventsLogMemoryPointersStructure.eventsPendingMessagesInLogMemory || (eventsFsmState != EVENTS_FSM_STATE_IDLE ));
  return(eventsLogMemoryPointersStructure.eventsPendingMessagesInLogMemory);
}



/**************************************************************************
* >>> ReturnCode_T eventsLogMemoryCmd( ProtocolappLogMemoryStructCmd_t* LogMemoryCmdPtr )
*
*
*
***************************************************************************/
ReturnCode_T eventsLogMemoryCmd( ProtocolappLogMemoryStructCmd_t* LogMemoryCmdPtr )
{
  switch(LogMemoryCmdPtr->LogMemId)
  {
  case 0:
    LogMemoryStateBitMap.LogMemoryTail1Operating = LogMemoryCmdPtr->TailPtr1State;
    break;
  case 1:
    LogMemoryStateBitMap.LogMemoryTail2Operating = LogMemoryCmdPtr->TailPtr2State;;
    break;
  } 
  
  return(RETURNCODE_OK);
}



/******************************************************************************
* @brief  ReturnCode_T eventsLogMemoryFullCheck(uint16_t NewMessageEntryLength)
* @param  
* @retval 
******************************************************************************/
ReturnCode_T eventsLogMemoryFullCheck(uint16_t NewMessageEntryLength)
{
  ReturnCode_T MyReturnCode = RETURNCODE_LOGMEMORY_FULL;  
  
  volatile uint32_t Temp = 0;
  
  // Initial condition where both pointers are zero is considered no full
  
  if(eventsLogMemoryPointersStructure.eventsSpiFlashAddressHead == eventsLogMemoryPointersStructure.eventsSpiFlashAddressTail)
    MyReturnCode = RETURNCODE_LOGMEMORY_NOTFULL;
  else
  {
    if(eventsLogMemoryPointersStructure.eventsSpiFlashAddressHead > eventsLogMemoryPointersStructure.eventsSpiFlashAddressTail)
    {
      Temp = ((SPIFLASH_ZONE_EVENTS_END - eventsLogMemoryPointersStructure.eventsSpiFlashAddressHead) + eventsLogMemoryPointersStructure.eventsSpiFlashAddressTail);
      
      if( ((SPIFLASH_ZONE_EVENTS_END - eventsLogMemoryPointersStructure.eventsSpiFlashAddressHead) + eventsLogMemoryPointersStructure.eventsSpiFlashAddressTail) > (uint32_t)NewMessageEntryLength)
        MyReturnCode = RETURNCODE_LOGMEMORY_NOTFULL;
      else
        MyReturnCode = RETURNCODE_LOGMEMORY_FULL;
    }
    else
    {
      if( (eventsLogMemoryPointersStructure.eventsSpiFlashAddressTail - eventsLogMemoryPointersStructure.eventsSpiFlashAddressHead) > (uint32_t)NewMessageEntryLength)
        MyReturnCode = RETURNCODE_LOGMEMORY_NOTFULL;
      else
        MyReturnCode = RETURNCODE_LOGMEMORY_FULL;
    }
  }
  return(MyReturnCode);
}


/******************************************************************************
* @brief ReturnCode_T EventsFsmTimerTimeoutCallback()
* @param  
* @retval 
******************************************************************************/
void EventsFsmTimerTimeoutCallback()
{
  eventsQueueEntryT  EventsTaskQueueEntry;
  
  // Fill the queue entry
  EventsTaskQueueEntry.FsmStimuli = EVENTS_FSM_STIMULI_TIMEOUT;
  EventsTaskQueueEntry.PayloadPtr   = 0;
  EventsTaskQueueEntry.PayloadLength = 0;
  
  // Enqueue the event to the commTask input queue
  xQueueSendToFront(eventsQueueHandle, &EventsTaskQueueEntry, 0);
}

/******************************************************************************
* @brief ReturnCode_T eventsSpiCompletionCallBack()
* @param  
* @retval 
******************************************************************************/
void eventsSpiCompletionCallBack()
{
  eventsQueueEntryT  EventsTaskQueueEntry;
  
  // Fill the queue entry
  EventsTaskQueueEntry.FsmStimuli = EVENTS_FSM_STIMULI_SPI_COMPLETION;
  EventsTaskQueueEntry.PayloadPtr   = 0;
  EventsTaskQueueEntry.PayloadLength = 0;
      
  // Enqueue the event to the commTask input queue
  xQueueSendToFront(eventsQueueHandle, &EventsTaskQueueEntry, 0);
}



/******************************************************************************
* @brief void  eventsMemoryStatePrint()
* @param  
* @retval 
******************************************************************************/
void  eventsMemoryStatePrint(uint8_t CallSource, PROTOCOLAPP_COMMANDS_T QueueOpCode)
{
  uint8_t DebugBuffer[300];
  
  switch(CallSource)
  {
  case 0:
    sprintf(DebugBuffer, "Event Created: Head=%Lx Tail=%x PendingMsg=%ld OpCode=%ld TotalEvents=%ld FailedTxMessagesCounter=%lx", eventsLogMemoryPointersStructure.eventsSpiFlashAddressHead, eventsLogMemoryPointersStructure.eventsSpiFlashAddressTail, eventsLogMemoryPointersStructure.eventsPendingMessagesInLogMemory, QueueOpCode, eventsLogMemoryPointersStructure.eventsTotalStoredNumberCounter, eventsLogMemoryPointersStructure.eventsFailedTxMessagesCounter);
    break;
  case 1:
    sprintf(DebugBuffer, "Event Sent: Head=%lx Tail=%x PendingMsg=%ld OpCode=%ld TotalEvents=%ld FailedTxMessagesCounter=%lx", eventsLogMemoryPointersStructure.eventsSpiFlashAddressHead, eventsLogMemoryPointersStructure.eventsSpiFlashAddressTail, eventsLogMemoryPointersStructure.eventsPendingMessagesInLogMemory, QueueOpCode, eventsLogMemoryPointersStructure.eventsTotalStoredNumberCounter, eventsLogMemoryPointersStructure.eventsFailedTxMessagesCounter);
    break;
  case 2:
    sprintf(DebugBuffer, "Log Mem pointers Crc32 failed, Try recovering from SpiFlash SystemPage: Head=%lx Tail=%lx PendingMsg=%ld  TotalEvents=%ld FailedTxMessagesCounter=%x", eventsLogMemoryPointersStructure.eventsSpiFlashAddressHead, eventsLogMemoryPointersStructure.eventsSpiFlashAddressTail, eventsLogMemoryPointersStructure.eventsPendingMessagesInLogMemory, eventsLogMemoryPointersStructure.eventsTotalStoredNumberCounter, eventsLogMemoryPointersStructure.eventsFailedTxMessagesCounter);
    break;
  case 3:
    sprintf(DebugBuffer, "Log Mem pointers Crc32 ok, : Head=%x Tail=%lx PendingMsg=%ld  TotalEvents=%d FailedTxMessagesCounter=%lx", eventsLogMemoryPointersStructure.eventsSpiFlashAddressHead, eventsLogMemoryPointersStructure.eventsSpiFlashAddressTail, eventsLogMemoryPointersStructure.eventsPendingMessagesInLogMemory, eventsLogMemoryPointersStructure.eventsTotalStoredNumberCounter, eventsLogMemoryPointersStructure.eventsFailedTxMessagesCounter);
    break;
  case 4:
    sprintf(DebugBuffer, "Could not recover from SpiFlash - HardReset Pointers, : Head=%lx Tail=%x PendingMsg=%ld  TotalEvents=%ld FailedTxMessagesCounter=%ux", eventsLogMemoryPointersStructure.eventsSpiFlashAddressHead, eventsLogMemoryPointersStructure.eventsSpiFlashAddressTail, eventsLogMemoryPointersStructure.eventsPendingMessagesInLogMemory, eventsLogMemoryPointersStructure.eventsTotalStoredNumberCounter, eventsLogMemoryPointersStructure.eventsFailedTxMessagesCounter);
    break;
  case 5:
    sprintf(DebugBuffer, "Log Mem recovered successfully from Spi Flash, : Head=%lx Tail=%lx PendingMsg=%ld  TotalEvents=%ld FailedTxMessagesCounter=%lx", eventsLogMemoryPointersStructure.eventsSpiFlashAddressHead, eventsLogMemoryPointersStructure.eventsSpiFlashAddressTail, eventsLogMemoryPointersStructure.eventsPendingMessagesInLogMemory, eventsLogMemoryPointersStructure.eventsTotalStoredNumberCounter, eventsLogMemoryPointersStructure.eventsFailedTxMessagesCounter);
    break;
  case 6:
    sprintf(DebugBuffer, "Event was Created: Head=%Lx Tail=%x PendingMsg=%ld TotalEvents=%ld FailedTxMessagesCounter=%lx", eventsLogMemoryPointersStructure.eventsSpiFlashAddressHead, eventsLogMemoryPointersStructure.eventsSpiFlashAddressTail, eventsLogMemoryPointersStructure.eventsPendingMessagesInLogMemory, eventsLogMemoryPointersStructure.eventsTotalStoredNumberCounter, eventsLogMemoryPointersStructure.eventsFailedTxMessagesCounter);
    break;
  default:
    strcpy(DebugBuffer, "");
  }
  
  vlapmainDebugLog(DebugBuffer);
}


void eventsLogMemoryPointersToNvmErase()
{
  spiflashReqEnqueue(SPIFLASH_CMD_PAGE_ERASE, SPIFLASH_SYSTEM_START, 0, (uint8_t*)&eventsLogMemoryPointersStructure, sizeof(eventsLogMemoryPointersStructure), 0, false);
}


void eventsLogMemoryPointersToNvmSavePVD()
{
 // hwdriversGpioBitWrite(HWDRIVERS_PB15_USB_TEST, 0);
  spiflashBlockingWriteWithoutErase(SPIFLASH_SYSTEM_START, sizeof(eventsLogMemoryPointersStructure), (uint8_t*)&eventsLogMemoryPointersStructure);
  //hwdriversGpioBitWrite(HWDRIVERS_PB15_USB_TEST, 1);
}

//void eventsLogMemoryPointersToNvmSave()
//{
//  // TODO: Consider for the future to use here the blocking method
//  //spiflashBlockingWrite(SPIFLASH_SYSTEM_START, sizeof(eventsLogMemoryPointersStructure), (uint8_t*)&eventsLogMemoryPointersStructure);
//  spiflashReqEnqueue(SPIFLASH_CMD_PROG_VIA_BUFF_1, SPIFLASH_SYSTEM_START, 0, (uint8_t*)&eventsLogMemoryPointersStructure, sizeof(eventsLogMemoryPointersStructure), 0, false);
//}


eventsFsmStateT eventsStateGet()
{
  return(eventsFsmState);
}

eventsPendingStatusT eventsPendingStatusGet()
{
  return eventsPendingEventsStatus;
}
/******************************************************************************
* @brief Returns the distance between head & tail pointers
* @retval Number of bytes between pointers
******************************************************************************/
uint32_t eventsGetEventsArea()
{
  if (eventsLogMemoryPointersStructure.eventsSpiFlashAddressHead > eventsLogMemoryPointersStructure.eventsSpiFlashAddressTail)
    return (eventsLogMemoryPointersStructure.eventsSpiFlashAddressHead - eventsLogMemoryPointersStructure.eventsSpiFlashAddressTail);
  else
    return (SPIFLASH_ZONE_EVENTS_END - eventsLogMemoryPointersStructure.eventsSpiFlashAddressTail + eventsLogMemoryPointersStructure.eventsSpiFlashAddressHead);
}
