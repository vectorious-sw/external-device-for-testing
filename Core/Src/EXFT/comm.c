#include "stdlib.h"
#include "comm.h"
#include "hwdrivers.h"
#include "vlapconfig.h"
#include "timers.h"
#include "pccommAppLayer.h"
#include "events.h"
#include "spiflash.h"
#include "ble.h"
#include "events.h"
#include "vlapmain.h"
#include "task.h"
#include "measure.h"
#include "i2cwork.h"
#include "config.h"
#include "buzzer.h"
#include "vibrator.h"
#include "charger.h"
#include "tracer.h"
#include "protocolapp.h"
#include "aescbc.h"
#include "leds.h"

// D E F I N I T I O N S 
#define COMM_TASK_NO_EVENTS_BLE_CONNECTED_TIMEOUT     5000
// The taking the gateway BLE client to detect master was disconnected
#define COMM_TASK_POWEROFF_DISCONNECT_TIME            30000
#define COMM_SPI_FLASH_BUFFER_LENGTH                  11000
#define COMM_GATEWAY_RETRY_COUNT                      5
#define COMM_SERVER_RESPONSE_RETRY_COUNT              5
#define COMM_CONFIG_SET_THRESHOLD                     10
#define COMM_COAP_MAX_MTU_SIZE                        950
typedef enum { COMM_PRE_CONFIGURED_STATE = 0, COMM_CONFIGURED_STATE = 1}  commConfigurationState_T;


// L O C A L   P R O T Y P E S 
void commSpiCompletionCallBack();
void commFsmTimerTimeoutCallback( );
void eventBsipStatusUpdate(uint16_t PendingBytes, uint32_t PendingMessages);
void commGoToIdle(uint16_t Delay);
void commDecryptAndUpdateGeneralHeader(uint8_t Source);
void commSkipCorruptedEvent(commSpiCorruption_T type, uint32_t* address, uint8_t* corruptedEventHeader);
uint32_t commGetCorruptedEventSkipSize(uint32_t address, uint16_t msgSize);



// G L O B A L S 
uint32_t commLastCommFsmTimeout;
commFsmStateT commFsmStatePrev;
commFsmStateT commFsmState;
QueueHandle_t commFsmQueueHandle;
commCurrentEventDescriptorT commCurrentEventDescriptor; 
commCurrentEventDescriptorT commCurrentEventDescriptorBackupForRetry; 
commCorruptedEventDescriptorT commCorruptedEventDescriptor; 
TimerHandle_t CommFsmTimerHandler;
uint32_t BleChunkCounter;
// Queue wait timeout
volatile  uint32_t commQueueWaitTimeout;
// TODO: Consider changing to 9000
uint8_t flashReadBuffer[COMM_SPI_FLASH_BUFFER_LENGTH]  __attribute__ ((aligned (4)));
uint32_t flashReadHeaderCopy[4];
volatile uint32_t AckCounter=0;
uint16_t commStatusSyncCounter;
commSleepState_T commSleepState;
uint8_t GatewayReconnectionRetryCounter;
uint8_t NoServerResponseRetryCounter;
bool syncSearchOccured = false;
commConfigurationState_T commConfigurationState;
ProtocolappHeader_t commPendingProtocolHeaderEventPtr;



/******************************************************************************
* @brief  void commInit(void)
* @param  
* @retval 
******************************************************************************/
ReturnCode_T commInit()
{
  // Create input queue  
  commFsmQueueHandle = xQueueCreate(30, sizeof(commFsmQueueEntryT));
  commFsmState = COMM_FSM_STATE_IDLE;
  commSleepState = COMM_SLEEP_STATE_NORMAL;
  GatewayReconnectionRetryCounter = COMM_GATEWAY_RETRY_COUNT;
  NoServerResponseRetryCounter = COMM_SERVER_RESPONSE_RETRY_COUNT;
  
  // Create the task
  xTaskCreate(commTask, commTaskName, commTaskSTACK_SIZE, NULL,  commTaskPriority, ( TaskHandle_t * ) NULL );
  // Generate first time n timeout event 
  
  CommFsmTimerHandler =  xTimerCreate("CommTaskFsmTimer",  portTICK_PERIOD_MS, pdFALSE, (void *)0, commFsmTimerTimeoutCallback);
  xTimerStart(CommFsmTimerHandler, 100);
  // Start the commFsm
  COMM_FSM_STATE_CHANGE(COMM_FSM_STATE_IDLE, 200);
  
  commConfigurationState = COMM_PRE_CONFIGURED_STATE;
  

  
  // Allways return OK
  return(RETURNCODE_OK);
}

uint16_t ReturnedEncryptedMsgLength;
uint8_t* ReturnedEncryptedMsgPtr;

uint8_t *ReturnedTelitM2mStringPtr;
uint16_t ReturnedTelitM2mStringLength;
uint16_t  BsipMessageIndex;
uint16_t commConfigSetThreshold = 0;
uint8_t* outputDecryptProtocolHeaderPtr = flashReadBuffer;



volatile bool commA;
volatile bool commB;
volatile bool commC;



/******************************************************************************
* @brief  portTASK_FUNCTION(commFsm, pvParameters )
* @param  
* @retval 
******************************************************************************/
portTASK_FUNCTION(commTask, pvParameters )
{
  commFsmQueueEntryT QueueEntry;
  eventsEventsLogMemoryStateT eventsEventsLogMemoryState;
  uint32_t NewEventEntryAddress;
  eventsEventsLogMemoryStateT commEventsLogMemoryState;
  uint8_t   *MsgPtr;
  uint16_t  MsgLength;
  uint16_t NumberOfPayloadBytes;
  static volatile int MallocProblemCounter;
  uint8_t ReturnedMsgPad = 0;
  uint16_t ackTimeout = 0;
  aescbcProtcolHeader_t *aescbcProtocolHeaderPtr;
  uint8_t *ReturnedMsgPtr;
  
  
 
    // Block the task till configuration is ready
  switch(commConfigurationState)
  {
  case COMM_PRE_CONFIGURED_STATE:
    xSemaphoreTake( configConfigurationValidSemaphoreHandle, portMAX_DELAY );
    xSemaphoreGive( configConfigurationValidSemaphoreHandle);
    commConfigurationState = COMM_CONFIGURED_STATE;
    break;
  case COMM_CONFIGURED_STATE:
  default:
    break;
  }

  
  // powerup event with exception data
  eventsEventWrite(0, 0, PROTOCOLAPP_GENERAL_SYSTEM_UP_EVENT, (char*)tracerExceptionDataPtrGet(), sizeof(ProtocolappPowerUpEventResetInfo_T), 0);
  //eventsEventWrite(0, 0, PROTOCOLAPP_GENERAL_SYSTEM_UP_EVENT, (char*)0, 0, 0);
  buzzerRequestToQueueAdd(BUZZER_BEEPING_ON_POWER_UP);
  vibratorRequestToQueueAdd(VIBRATOR_OPTION5);
  
  while(1)
  {
    // block task till new queue entry is received or timeout 
    BaseType_t  QueueState = xQueueReceive(commFsmQueueHandle, &QueueEntry, 10000);
    // Convert the queue status to FSM Stimuli
    if( (QueueState == pdTRUE) && (QueueEntry.ActionType < COMM_FSM_QUEUE_ENTRY_TYPE_LAST_ENTRY))
    {
 #if COMM_IS_BLE    
      // If measurement starts, Abort the comm session and turn off the ble to avoid BLE SOC mallfunction due to belt transmission 
      if(QueueEntry.ActionType == COMM_FSM_QUEUE_ENTRY_TYPE_COMM_ABORT)
      {
        commGoToIdle(3000);
      }
#endif      
      // Handle the new stimuli
      switch(COMM_FSM_STATE_GET)
      {
        // COMM_FSM_STATE_IDLE
      case COMM_FSM_STATE_IDLE:
        switch(QueueEntry.ActionType)
        {
        case COMM_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:
          // Messages will be read and send only after measurement ends, when the BLE is not at BOOT state and transmission is disabled
          if(!measureMeasurementInProgressGet() && (bleGetBootState() == BLE_BOOT_DISABLE) && (!pccpmmAppLayerStruct.BeltTransmitterRegisters.TransmitterEnable))
		  {
            // Check the state of the Log memory and return the address of the next entry in the spi flash
            eventsQueueStatusAndTailEntryAddressGet(&eventsEventsLogMemoryState, &NewEventEntryAddress);
            switch(eventsEventsLogMemoryState)
            {
            case EVENTS_QUEUESTATE_EMPTY:
              // Check if there is a pending message that still didn't get saved
              if(eventsPendingStatusGet()){
            	  COMM_FSM_TIMEOUT_SET(100);
            	  break;
              }
              
              bleControl(BLE_CONTROL_DISABLE);
              // Take this path to go into sleep
              if(!eventsPendingEventsStatusGet()){
                bleControl(BLE_CONTROL_DISABLE);
                COMM_FSM_STATE_CHANGE(COMM_FSM_STATE_APP_CONFIG_RESPONSE_WAIT, 5000);
              }  else {
              	COMM_FSM_TIMEOUT_SET(300);
              }
              break;
            case EVENTS_QUEUESTATE_NOTEMPTY:
            case EVENTS_QUEUESTATE_FULL:
            	 bleControl(BLE_CONTROL_DISABLE);
            	 vTaskDelay(1000);
            	 bleControl(BLE_CONTROL_ENABLE);
              
  #if COMM_IS_BLE    
              COMM_FSM_STATE_CHANGE(COMM_FSM_STATE_BLE_SOC_WARMUP, 100);
  #else
              COMM_FSM_STATE_CHANGE(COMM_FSM_STATE_CLIENT_SETUP_TIME_WAIT, 200);
  #endif
              break;
            }
          }
		  COMM_FSM_TIMEOUT_SET(100);
           break;
        default:
          break;
        }
        break;
        
        // COMM_FSM_STATE_BLE_SOC_WARMUP
      case COMM_FSM_STATE_BLE_SOC_WARMUP:
        switch(QueueEntry.ActionType)
        {
        case COMM_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:
          // Configure the BLE Soc using BSIP and start advertisement.
          // Allocate memory for the full message (DLL + application
          MsgLength = sizeof(bleBsipDllHeader_t) + sizeof(bleAdvertisingControl_t) + sizeof(bleBsipDllEndOfMessage_t);
          // The buffer will be freed by the low level USB
          MsgPtr = pvPortMalloc(MsgLength);
          if(MsgPtr)
          {
            ((bleAdvertisingControl_t*)(MsgPtr+sizeof(bleBsipDllHeader_t)))->PayloadApplicationLayerOperationCode = BLE_BSIP_OPCODE_ADVERTISING_CONTROL;
            ((bleAdvertisingControl_t*)(MsgPtr+sizeof(bleBsipDllHeader_t)))->AdvertisementControl = true;
            uint8_t * BytePtr = protocolappUniqueIdPtrGet();
            // Update the Bsip read response data struct
            ((bleAdvertisingControl_t*)(MsgPtr+sizeof(bleBsipDllHeader_t)))->PayloadApplicationLayerOperationCode = BLE_BSIP_OPCODE_ADVERTISING_CONTROL;
            uint16_t AdvertisingStringLength = sprintf((char*)((bleAdvertisingControl_t*)(MsgPtr+sizeof(bleBsipDllHeader_t)))->AdvertisementString, "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
			BytePtr[0], BytePtr[1], BytePtr[2], BytePtr[3], BytePtr[4], BytePtr[5], BytePtr[6], BytePtr[7], BytePtr[8], BytePtr[9], BytePtr[10], BytePtr[11]);       
            // Send BSIP protocol message, The allocated message will be freed by the UART DMA Tx ISR (The uart connected to the BLE)
            bleBsipDllMessageSend(MsgPtr, sizeof(bleBsipDllHeader_t) + AdvertisingStringLength + sizeof(bleBsipDllEndOfMessage_t));
            vTaskDelay(10);
            vlapmainDebugLog("modem connection wait for 30 seconds");
            bleBsipDllRxFsmIdleSet();
            // Change state to wait for the BLE advertisement ack
            COMM_FSM_STATE_CHANGE(COMM_FSM_STATE_ADVERTISE_BLE_CONNECTION_WAIT, 30000);
          }
          else
        	  commGoToIdle(100);
          break;
        default:
          break;
        }
        break;
        
        
        
        
        // COMM_FSM_STATE_ADVERTISE_BLE_CONNECTION_WAIT
      case COMM_FSM_STATE_ADVERTISE_BLE_CONNECTION_WAIT:
        switch(QueueEntry.ActionType)
        {
        case COMM_FSM_QUEUE_ENTRY_TYPE_BLE_CONNECTED: 
          commStatusSyncCounter = 0;
          vlapmainDebugLog("modem connected,  Client setup time delay");
          // BLE Client connection request was accepted, BLE is connected
          COMM_FSM_STATE_CHANGE(COMM_FSM_STATE_CLIENT_SETUP_TIME_WAIT, /*400*/ /*10*/ 1100);
          break;
        case COMM_FSM_QUEUE_ENTRY_TYPE_BLE_DISCONNECTED:
          // BLE Was disconnected
          commGoToIdle(100);
          break;
        case COMM_FSM_QUEUE_ENTRY_TYPE_BLE_TIMEOUT:
          break;
        case COMM_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:
          // Take this path to go into sleep
          COMM_FSM_STATE_CHANGE(COMM_FSM_STATE_APP_CONFIG_RESPONSE_WAIT, 5000);
          break;
        default:
          break;
        }
        break;
 
      // COMM_FSM_STATE_CLIENT_SETUP_TIME_WAIT
      case COMM_FSM_STATE_CLIENT_SETUP_TIME_WAIT:
        switch(QueueEntry.ActionType)
        {
        case COMM_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:       
          // Read the current page (The page containing the current tail pointer), The NewEventEntryAddress in this case points to the message's spiFlashLogMemory Header.
          spiflashReqEnqueue(SPIFLASH_CMD_READ, NewEventEntryAddress, (uint8_t*)flashReadBuffer, 0, sizeof(spiflashLogMemoryHeaderT), commSpiCompletionCallBack, false);
          vlapmainDebugLog("start Read first page");
          COMM_FSM_STATE_CHANGE(COMM_FSM_STATE_SPIREAD_FIRST_PAGE_WAIT, 1000 );
          break;
        case COMM_FSM_QUEUE_ENTRY_TYPE_BLE_DISCONNECTED:
          // BLE Was disconnected
          commGoToIdle(100);
          break;
        default:
          break;
        }
        break;

       
      case COMM_FSM_STATE_SPIREAD_SYNC_SEARCH:

        switch(QueueEntry.ActionType)
        {
          case COMM_FSM_QUEUE_ENTRY_TYPE_SPIFLASH_READ_COMPLETED:
            syncSearchOccured = true;
            uint8_t *flashReadBufferPtr = flashReadBuffer;
            bool endOfEventZone = (NewEventEntryAddress + SPIFLASH_SPI_PAGESIZE >= SPIFLASH_ZONE_EVENTS_END); 
            uint16_t readSize = endOfEventZone ? SPIFLASH_ZONE_EVENTS_END - NewEventEntryAddress : SPIFLASH_SPI_PAGESIZE; 
            // Page read - sync size
            readSize = (readSize > SPIFLASH_SYNC_PATTERN_SIZE) ? (readSize - SPIFLASH_SYNC_PATTERN_SIZE) : readSize; 
            uint32_t validEventSkipSize = commGetCorruptedEventSkipSize(NewEventEntryAddress, 0);
            // If out of bounds, reset
            if ((eventsGetEventsArea() < validEventSkipSize) || (eventsPendingEventsStatusGet() <= 1)) 
            {
              commGoToIdle(100);
              break;
            }
            uint16_t i = 0;
            // Search in whole page for SYNC
            for (i = 0; i <= readSize; i++)
            {
              uint32_t syncPattern = SPIFLASH_SYNC_PATTERN;
              if (memcmp(flashReadBufferPtr, &syncPattern, SPIFLASH_SYNC_PATTERN_SIZE) == 0)
              {
                NewEventEntryAddress = NewEventEntryAddress + i;
                COMM_FSM_STATE_CHANGE(COMM_FSM_STATE_SPIREAD_FIRST_PAGE_WAIT, 10);
                spiflashReqEnqueue(SPIFLASH_CMD_READ, NewEventEntryAddress, (uint8_t*)flashReadBuffer, 0, sizeof(spiflashLogMemoryHeaderT), commSpiCompletionCallBack, false);
                break;
              }
              flashReadBufferPtr++;
            }
            if (i > readSize) // header not found in read page
            {
            	// Read a whole page to scan again, offset read by 1
              NewEventEntryAddress = endOfEventZone ? SPIFLASH_ZONE_EVENTS_START : NewEventEntryAddress + readSize - (SPIFLASH_SYNC_PATTERN_SIZE - 1);
              COMM_FSM_STATE_CHANGE(COMM_FSM_STATE_SPIREAD_SYNC_SEARCH, 1000);
              spiflashReqEnqueue(SPIFLASH_CMD_READ, NewEventEntryAddress, (uint8_t*)(flashReadBuffer), 0, SPIFLASH_SPI_PAGESIZE, commSpiCompletionCallBack, false);
            }
          break;

          case COMM_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:
            commGoToIdle(100);
            break;
          default:
            commGoToIdle(100);
            break;
        }
      break;        
        
        
        // COMM_FSM_STATE_SPIREAD_FIRST_PAGE_WAIT
      case COMM_FSM_STATE_SPIREAD_FIRST_PAGE_WAIT:
        switch(QueueEntry.ActionType)
        {
        case COMM_FSM_QUEUE_ENTRY_TYPE_SPIFLASH_READ_COMPLETED:
          // Check if the spiFlash header is correct
          // TODO: This cannot handle a header that splits between START / END, ticket EDF-483 
          if ((((spiflashLogMemoryHeaderT*)flashReadBuffer)->SyncPattern == SPIFLASH_SYNC_PATTERN) && (((spiflashLogMemoryHeaderT*)flashReadBuffer)->EntrySize) && (((spiflashLogMemoryHeaderT*)flashReadBuffer)->EntrySize)<= COMM_SPI_FLASH_BUFFER_LENGTH)
          {
  //        sprintf(PrintBuff, "First Page Read Head: %08x, Tail: %08x", eventsQueuePrevHeadIndexGet(), eventsQueueTailIndexGet());
  //        vlapmainDebugLog(PrintBuff);
            // Fill the descriptor with this read session info  
            commCurrentEventDescriptor.CurrentPage         = (NewEventEntryAddress - SPIFLASH_ZONE_EVENTS_START);
            commCurrentEventDescriptor.BytesLeftToBeRead   = (uint16_t)((spiflashLogMemoryHeaderT*)flashReadBuffer)->EntrySize;
            commCurrentEventDescriptor.CurrentSpiAddress   = NewEventEntryAddress;
            commCurrentEventDescriptor.NewEventMsgLength   = (uint16_t)(((spiflashLogMemoryHeaderT*)flashReadBuffer)->EntrySize + sizeof(spiflashLogMemoryHeaderT));
            // Save a copy of the descriptor for retries
            memcpy(&commCurrentEventDescriptorBackupForRetry, &commCurrentEventDescriptor, sizeof(commCurrentEventDescriptorT));
            memcpy(flashReadHeaderCopy, flashReadBuffer, sizeof(spiflashLogMemoryHeaderT));
            BsipMessageIndex = 0;
            // Read one complete message and place it in sizeof(DllHeader_t) offset from the spiflash buffer
            spiflashReqEnqueue(SPIFLASH_CMD_READ, NewEventEntryAddress+sizeof(spiflashLogMemoryHeaderT), (uint8_t*)(flashReadBuffer /*+ sizeof(aescbcProtcolHeader_t*/+sizeof(DllHeader_t)), 0, commCurrentEventDescriptor.BytesLeftToBeRead, commSpiCompletionCallBack, false);
#if COMM_IS_BLE
            COMM_FSM_STATE_CHANGE(COMM_FSM_STATE_SPIREAD_BLE_READ_REQ_WAIT, 1000/*10000*/);
#else
            COMM_FSM_STATE_CHANGE(COMM_FSM_STATE_SPIREAD_BLE_READ_REQ_WAIT, 1000);
#endif
            BleChunkCounter = 0;
          }
          else // Header is wrong, no sync pattern, search for header
          {
            commSkipCorruptedEvent(COMM_SPI_CORRUPTION_SYNC, &NewEventEntryAddress, flashReadBuffer);
          }
          break;
        case COMM_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:
          // Message's first page read operation failed, go to idle
          commGoToIdle(100);
          break;
        default:
        	break;
        }
        break;
        
        // COMM_FSM_STATE_SPIREAD_BLE_READ_REQ_WAIT:
      case COMM_FSM_STATE_SPIREAD_BLE_READ_REQ_WAIT:
        switch(QueueEntry.ActionType)
        {
        case COMM_FSM_QUEUE_ENTRY_TYPE_SPIFLASH_READ_COMPLETED:
          // Check if event is corrupt
          if (crc32BuffCalc((flashReadBuffer + sizeof(DllHeader_t)), 0, commCurrentEventDescriptor.BytesLeftToBeRead) != ((spiflashLogMemoryHeaderT*)flashReadHeaderCopy)->CRC32)
          {
            commSkipCorruptedEvent(COMM_SPI_CORRUPTION_BAD_CRC, &NewEventEntryAddress, flashReadHeaderCopy);
            break;
          }
          // Insert the transmission time into the event message overwriting the defult time which matches the event time by default 
          aescbcProtocolHeaderPtr = (aescbcProtcolHeader_t *)(flashReadBuffer+(sizeof(DllHeader_t)));
          aescbcProtocolHeaderPtr->TransmissionTimeStamp       = HTONL(rtcEpochGet());     
          aescbcProtocolHeaderPtr->TransmissionTimeStamp10mSec = 0; 
          // Encrypt the message read from the spiFlash
          // Insert the transmission time into the event message overwriting the defult time which matches the event time by default 
          // the Encryption will be done over the flashReadbuffer assuming the message includes DllHeader_t + aescbcProtcolHeader_t + Payload, the encyption will return the encrypted msg length
          // Please note the resulted buffer may contain padded bytes as required by the encryption algorithm
          //ReturnCode_T ReturnCode = aescbcEncryptByChunks(flashReadBuffer, commCurrentEventDescriptor.BytesLeftToBeRead, &ReturnedEncryptedMsgLength);
          
          uint16_t readMessageLength = commCurrentEventDescriptor.BytesLeftToBeRead + sizeof(DllHeader_t) + sizeof(DllEndOfMessage_t);
          // Add the DLL layer (Sync and CRC)
          uartdllCompleteMEssageBuild(flashReadBuffer, readMessageLength);


        case COMM_FSM_QUEUE_ENTRY_TYPE_BLE_READ_REQ:
#if !COMM_IS_BLE
        case COMM_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:
#endif     
  	  vlapmainDebugLog("Read Request");
          BleChunkCounter++;
          if(readMessageLength > BLE_UARTDLL_CHUNK_SIZE)
           // Allocate memory for the full message (DLL + application
            NumberOfPayloadBytes = BLE_UARTDLL_CHUNK_SIZE;
          else
          {
     	  vlapmainDebugLog("Read Last");
            NumberOfPayloadBytes = readMessageLength;
          }		
#if !COMM_IS_BLE
          uartdllTxQueueEnqueue(UARTDLL_UART_2_CELLMODEM, flashReadBuffer + BsipMessageIndex, readMessageLength, false);
          readMessageLength = 0;
#endif
#if COMM_IS_BLE   
          //   
          MsgLength = sizeof(bleBsipDllHeader_t) + sizeof(bleReadResponse_t) + NumberOfPayloadBytes + sizeof(bleBsipDllEndOfMessage_t);
          // Allocate memory for the full message (DLL + application
          // The buffer will be freed by the low level driver
          MsgPtr = pvPortMalloc(MsgLength);
          if(MsgPtr)
          {
            // Update the Bsip read response darta struct
            ((bleReadResponse_t*)(MsgPtr+sizeof(bleBsipDllHeader_t)))->PayloadApplicationLayerOperationCode = BLE_BSIP_OPCODE_READ_RESP;
            ((bleReadResponse_t*)(MsgPtr+sizeof(bleBsipDllHeader_t)))->PendingBytesCurrentMessage = TYPES_ENDIAN16_CHANGE(readMessageLength);
            ((bleReadResponse_t*)(MsgPtr+sizeof(bleBsipDllHeader_t)))->PendingMessagesInLogMemory = TYPES_ENDIAN32_CHANGE(eventsPendingEventsStatusGet());
            commStatusSyncCounter++;
            ((bleReadResponse_t*)(MsgPtr+sizeof(bleBsipDllHeader_t)))->StatusSyncCounter = TYPES_ENDIAN16_CHANGE(commStatusSyncCounter);
            memcpy(MsgPtr+sizeof(bleBsipDllHeader_t) + sizeof(bleReadResponse_t), flashReadBuffer + BsipMessageIndex, NumberOfPayloadBytes);
            // Send BSIP protocol message, The allocated message buffer will be freed by the UART DMA Tx ISR
            bleBsipDllMessageSend(MsgPtr, MsgLength);

            vTaskDelay(30);
            // Increment index
            readMessageLength -= NumberOfPayloadBytes;
            BsipMessageIndex += NumberOfPayloadBytes;
#endif
                       
            if(BsipMessageIndex > COMM_COAP_MAX_MTU_SIZE)
              ackTimeout = 20000;
            else
              ackTimeout = 20000;
            
            if(readMessageLength)
            {
#if COMM_IS_BLE
            COMM_FSM_STATE_CHANGE(COMM_FSM_STATE_SPIREAD_BLE_READ_REQ_WAIT, 10000);
#else
            COMM_FSM_STATE_CHANGE(COMM_FSM_STATE_SPIREAD_BLE_READ_REQ_WAIT, 10000);
#endif
            }
            else
            {
              // Last chunk of data was sent to the BLE SOC via Bsip, go wait for ack from host
              readMessageLength = 0;
#if 0
              // Printout the contents of the buffer before sending it 
              for(uint16_t i=0; i< commCurrentEventDescriptor.NewEventMsgLength; i+=16)
              {
                uint8_t *  Ptr = pvPortMalloc(200);
                uint16_t MyLength = sprintf(Ptr, "%05x -- %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \r\n", i, flashReadBuffer[i], flashReadBuffer[i+1], flashReadBuffer[i+2], flashReadBuffer[i+3], flashReadBuffer[i+4], flashReadBuffer[i+5], flashReadBuffer[i+6], flashReadBuffer[i+7], flashReadBuffer[i+8], flashReadBuffer[i+9], flashReadBuffer[i+10], flashReadBuffer[i+11], flashReadBuffer[i+12], flashReadBuffer[i+13], flashReadBuffer[i+14], flashReadBuffer[i+15]);
                uartdllTxQueueEnqueue(UARTDLL_UART_6_RS485, Ptr, MyLength, false);
                vTaskDelay(10);
              }
#endif 
              commDecryptAndUpdateGeneralHeader(0);

              // Waiting 5 Sec for Ack was marginal so I've moved to 10 Sec
#if COMM_IS_BLE
              COMM_FSM_STATE_CHANGE(COMM_FSM_STATE_APP_ACK_WAIT, ackTimeout);                
#else
              COMM_FSM_STATE_CHANGE(COMM_FSM_STATE_APP_ACK_WAIT, 1000);              
#endif
              vlapmainDebugLog("App Ack Wait");
            }
#if COMM_IS_BLE
          }
          else
          {
              // Malloc problem
              MallocProblemCounter++;
          }
#endif          
          break;
#if COMM_IS_BLE
        case COMM_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:
#endif
        case COMM_FSM_QUEUE_ENTRY_TYPE_BLE_DISCONNECTED:
          bleControl(BLE_CONTROL_DISABLE);
          //commGoToIdle(COMM_TASK_POWEROFF_DISCONNECT_TIME);
          commGoToIdle(100);
          break;
        default:
          break;
        }
        break;
        
        
      case COMM_FSM_STATE_APP_ACK_WAIT:
          switch(QueueEntry.ActionType)
          {
          case COMM_FSM_QUEUE_ENTRY_TYPE_SPIFLASH_READ_COMPLETED:
          case COMM_FSM_QUEUE_ENTRY_TYPE_BLE_CONNECTED:       
          case COMM_FSM_QUEUE_ENTRY_TYPE_BLE_READ_REQ:
          case COMM_FSM_QUEUE_ENTRY_TYPE_BLE_TIMEOUT:
            break;
          case COMM_FSM_QUEUE_ENTRY_TYPE_BLE_DISCONNECTED:
            commGoToIdle(100);
            break;
            
          case COMM_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:
            // NACK received, retry sending the last event
            // restor the event transmission descriptor
            vlapmainDebugLog("Ack Timeout");
            memcpy(&commCurrentEventDescriptor, &commCurrentEventDescriptorBackupForRetry,sizeof(commCurrentEventDescriptorT));
            BsipMessageIndex = 0;
            NoServerResponseRetryCounter--;
            if(NoServerResponseRetryCounter)
            {
            // Read the current page (The page containing the current tail pointer), The NewEventEntryAddress in this case points to the message's spiFlashLogMemory Header.
            spiflashReqEnqueue(SPIFLASH_CMD_READ, NewEventEntryAddress, (uint8_t*)flashReadBuffer, 0, sizeof(spiflashLogMemoryHeaderT), commSpiCompletionCallBack, false);
#if COMM_IS_BLE
            COMM_FSM_STATE_CHANGE(COMM_FSM_STATE_SPIREAD_FIRST_PAGE_WAIT, 10000);
#else
            COMM_FSM_STATE_CHANGE(COMM_FSM_STATE_SPIREAD_FIRST_PAGE_WAIT, 10000);
#endif
            }
            else
            // Take this path to go into sleep
            COMM_FSM_STATE_CHANGE(COMM_FSM_STATE_APP_CONFIG_RESPONSE_WAIT, 5000);

            break;
          case COMM_FSM_QUEUE_ENTRY_TYPE_APP_NACK:
            // NACK received, retry sending the last event
            vlapmainDebugLog("NAck");
          case COMM_FSM_QUEUE_ENTRY_TYPE_APP_INVALID_ACK:
            // restor the event transmission descriptor
            memcpy(&commCurrentEventDescriptor, &commCurrentEventDescriptorBackupForRetry,sizeof(commCurrentEventDescriptorT));
            BsipMessageIndex = 0;
            NoServerResponseRetryCounter--;
            if(NoServerResponseRetryCounter)
            {
              // Read the current page (The page containing the current tail pointer), The NewEventEntryAddress in this case points to the message's spiFlashLogMemory Header.
              spiflashReqEnqueue(SPIFLASH_CMD_READ, NewEventEntryAddress, (uint8_t*)flashReadBuffer, 0, sizeof(spiflashLogMemoryHeaderT), commSpiCompletionCallBack, false);
              COMM_FSM_STATE_CHANGE(COMM_FSM_STATE_SPIREAD_FIRST_PAGE_WAIT, 10000);
            }
            else
              // Take this path to go into sleep
              COMM_FSM_STATE_CHANGE(COMM_FSM_STATE_APP_CONFIG_RESPONSE_WAIT, 5000);
            break;
          case COMM_FSM_QUEUE_ENTRY_TYPE_APP_ACK:
            vlapmainDebugLog("Ack");
            // Reset the server response retry as Ack indicates server is respnsive again
            NoServerResponseRetryCounter = COMM_SERVER_RESPONSE_RETRY_COUNT;
            AckCounter++;
            // Create event alerting a sync mismatch occured
            if (syncSearchOccured)
            {
              // Update log memory skipping the corrupted events
              vlapmainDebugLog("Event skipped!");
              uint32_t validEventSkipSize = commGetCorruptedEventSkipSize(NewEventEntryAddress, commCurrentEventDescriptor.NewEventMsgLength);
              eventsQueuePointerIndexUpdate(EVENTS_LOGMEMORY_TAIL, &commEventsLogMemoryState, validEventSkipSize, true);
              commCorruptedEventDescriptor.EndingSpiFlashAddress = NewEventEntryAddress;
              eventsEventWrite(0, 0, PROTOCOLAPP_GENERAL_CORRUPTED_EVENT, (char*)&commCorruptedEventDescriptor, sizeof(commCorruptedEventDescriptorT), 0);
              syncSearchOccured = false;
            }
            else 
            {
              eventsQueuePointerIndexUpdate(EVENTS_LOGMEMORY_TAIL, &commEventsLogMemoryState, commCurrentEventDescriptor.NewEventMsgLength, false);
            }
            // Check for more pending events
            // Update the log memory tail index
            vlapmainDebugLog("Event sent");
            //eventsMemoryStatePrint(1, TransmittedOpCode);
            // Check for more pending events, returns the Log memory status and new event header address if relevant
            eventsQueueStatusAndTailEntryAddressGet(&eventsEventsLogMemoryState, &NewEventEntryAddress);
            // Check the new Log memory status
            switch(eventsEventsLogMemoryState)
            {
            case EVENTS_QUEUESTATE_EMPTY:
              // No more events to send, Answer the read request with 0 payload bytes and updated status
              COMM_FSM_STATE_CHANGE(COMM_FSM_STATE_BLE_CONNECTED_EVENTS_WAIT , COMM_TASK_NO_EVENTS_BLE_CONNECTED_TIMEOUT);
              break;
            case EVENTS_QUEUESTATE_NOTEMPTY:
            case EVENTS_QUEUESTATE_FULL:
              // Read the current page (The page containing the current tail pointer), The NewEventEntryAddress in this case points to the message's spiFlashLogMemory Header.
              spiflashReqEnqueue(SPIFLASH_CMD_READ, NewEventEntryAddress, (uint8_t*)flashReadBuffer, 0, sizeof(spiflashLogMemoryHeaderT), commSpiCompletionCallBack, false);
              COMM_FSM_STATE_CHANGE(COMM_FSM_STATE_SPIREAD_FIRST_PAGE_WAIT, 1000);
              break;
//            case COMM_FSM_QUEUE_ENTRY_TYPE_BLE_TIMEOUT:
//              break;
            default:
              break;
            }
          break;
            default:
            	break;
        }
        break;
        
        //COMM_FSM_STATE_BLE_CONNECTED_EVENTS_WAIT
      case COMM_FSM_STATE_BLE_CONNECTED_EVENTS_WAIT:
        switch(QueueEntry.ActionType)
        {
        case COMM_FSM_QUEUE_ENTRY_TYPE_BLE_CONNECTED:       
        case COMM_FSM_QUEUE_ENTRY_TYPE_BLE_TIMEOUT:
          break;
        case COMM_FSM_QUEUE_ENTRY_TYPE_BLE_DISCONNECTED:
          commGoToIdle(100);
          break;
        case COMM_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:
          // Periodical Check for more pending logged events
          eventsQueueStatusAndTailEntryAddressGet(&eventsEventsLogMemoryState, &NewEventEntryAddress);
          switch(eventsEventsLogMemoryState)
          {
          case EVENTS_QUEUESTATE_EMPTY:
            // Send Config request event
            vlapmainDebugLog("Config Request Event");
            // Build  Configuration download request message
            //uint8_t *ReturnedMsgPtr = 0;
            uint16_t ReturnedMsgLength;
            protocolappMessageBuildnNotViaFlash(0, 0, PROTOCOLAPP_CONFIGURATION_DOWNLOAD_REQ, 0, 0,flashReadBuffer, &ReturnedMsgLength, &ReturnedMsgPad);
            // the Encryption will be done over the flashReadbuffer assuming the message includes DllHeader_t + aescbcProtcolHeader_t + Payload, the encyption will return the encrypted msg length
            // Please note the resulted buffer may contain padded bytes as required by the encryption algorithm
            ReturnCode_T ReturnCode = aescbcEncryptByChunks(flashReadBuffer + sizeof(DllHeader_t), ReturnedMsgLength - ReturnedMsgPad, &ReturnedEncryptedMsgLength);
            ReturnedEncryptedMsgLength += (sizeof(DllHeader_t) + sizeof(DllEndOfMessage_t));
            if(ReturnCode != RETURNCODE_OK)
            {
              commGoToIdle(100);
              break;
            }
            else
              uartdllCompleteMEssageBuild(flashReadBuffer, ReturnedEncryptedMsgLength);
#if !COMM_IS_BLE
            uartdllTxQueueEnqueue(UARTDLL_UART_2_CELLMODEM, flashReadBuffer, ReturnedEncryptedMsgLength, false);
            // Free the application protocol allocated buffer as it was already coppied to the BLE message buffer
            //              if(ReturnedMsgPtr)
            //               vPortFree(ReturnedMsgPtr);
            // Give time for the BLE message to propogate
            vTaskDelay(2000);
            // Go wait for Config Ack
            COMM_FSM_STATE_CHANGE(COMM_FSM_STATE_APP_CONFIG_RESPONSE_WAIT, 5000/*40000*/);
#endif
#if COMM_IS_BLE   
            // 
            MsgLength = sizeof(bleBsipDllHeader_t) + sizeof(bleReadResponse_t) + ReturnedEncryptedMsgLength + sizeof(bleBsipDllEndOfMessage_t);
            // Allocate memory for the full message.
            // The buffer will be freed by the low level UART driver
            MsgPtr = pvPortMalloc(MsgLength);
            if(MsgPtr)
            {
              // Update the Bsip read response data struct
              ((bleReadResponse_t*)(MsgPtr+sizeof(bleBsipDllHeader_t)))->PayloadApplicationLayerOperationCode = BLE_BSIP_OPCODE_READ_RESP;
              // We force the configuration message length so the gateway will read it as if it is normal message
              ((bleReadResponse_t*)(MsgPtr+sizeof(bleBsipDllHeader_t)))->PendingBytesCurrentMessage = TYPES_ENDIAN16_CHANGE(ReturnedEncryptedMsgLength);
              // We force the pending events in log memory to 1 to make the gateway to think there is one more event 
              ((bleReadResponse_t*)(MsgPtr+sizeof(bleBsipDllHeader_t)))->PendingMessagesInLogMemory = TYPES_ENDIAN32_CHANGE(1);
              commStatusSyncCounter++;
              ((bleReadResponse_t*)(MsgPtr+sizeof(bleBsipDllHeader_t)))->StatusSyncCounter = TYPES_ENDIAN16_CHANGE(commStatusSyncCounter);
              memcpy(MsgPtr+sizeof(bleBsipDllHeader_t) + sizeof(bleReadResponse_t), flashReadBuffer, ReturnedEncryptedMsgLength);
              // Send BSIP protocol message, The allocated message buffer will be freed by the UART DMA Tx ISR
              bleBsipDllMessageSend(MsgPtr, MsgLength);
                
              commDecryptAndUpdateGeneralHeader(1);
              // Free the application protocol allocated buffer as it was already copied to the BLE message buffer
              //               if(ReturnedMsgPtr)
              //                 vPortFree(ReturnedMsgPtr);
              // Give time for the BLE message to propagate
              vTaskDelay(30);
              
              // Go wait for Config Ack
              COMM_FSM_STATE_CHANGE(COMM_FSM_STATE_APP_CONFIG_RESPONSE_WAIT, 5000/*40000*/);
            }
            else
            {
              // Free the application protocol allocated buffer even if the BLE buffer allocation failed
              if(ReturnedMsgPtr)
                vPortFree(ReturnedMsgPtr);
              // No memory to allocate, go to idle
              vlapmainDebugLog("Config Request Event - No memory to allocate");
              commGoToIdle(100);
            }
#endif          
            break;
          case EVENTS_QUEUESTATE_NOTEMPTY:
          case EVENTS_QUEUESTATE_FULL:
            // Read the current page (The page containing the current tail pointer), The NewEventEntryAddress in this case points to the message's spiFlashLogMemory Header.
            spiflashReqEnqueue(SPIFLASH_CMD_READ, NewEventEntryAddress, (uint8_t*)flashReadBuffer, 0, sizeof(spiflashLogMemoryHeaderT), commSpiCompletionCallBack, false);
            COMM_FSM_STATE_CHANGE(COMM_FSM_STATE_SPIREAD_FIRST_PAGE_WAIT, 100);
            break;
          default:
            break;
          }
          default:
        	  break;
        }
        break;
        
      case COMM_FSM_STATE_APP_CONFIG_RESPONSE_WAIT:
        switch(QueueEntry.ActionType)
        {
        case COMM_FSM_QUEUE_ENTRY_TYPE_BLE_CONNECTED:       
        case COMM_FSM_QUEUE_ENTRY_TYPE_BLE_TIMEOUT:
          break;
        case COMM_FSM_QUEUE_ENTRY_TYPE_NO_NEW_CONFIG_RESP:
          // Send disconnection request
          MsgLength = sizeof(bleBsipDllHeader_t) + sizeof(bleDisconnectionReq_t) + sizeof(bleBsipDllEndOfMessage_t);
          MsgPtr = pvPortMalloc(MsgLength);
          if(MsgPtr)
          {
            ((bleDisconnectionReq_t*)(MsgPtr+sizeof(bleBsipDllHeader_t)))->PayloadApplicationLayerOperationCode = BLE_BSIP_OPCODE_DISCONNECTION_REQ;
          }
          
          bleBsipDllMessageSend(MsgPtr, MsgLength);
          
          
          vlapmainDebugLog("Server indicates no new configuration pending, go to sleep");
          //eventBsipStatusUpdate(0,0);
          break;
        case COMM_FSM_QUEUE_ENTRY_TYPE_CONFIG_RESP:
          vlapmainDebugLog("Config Response received, waiting for config set...");
          // Set wait for config set threshold
          commConfigSetThreshold = COMM_CONFIG_SET_THRESHOLD;
          break;
        case COMM_FSM_QUEUE_ENTRY_TYPE_CONFIG_SET_DONE:
          // Wait for config set confirm
          if(commConfigSetThreshold)
          {
            vlapmainDebugLog("Config set is done!");
            // Set comm config threshold back to zero
            commConfigSetThreshold = 0;
            // Notify config update
            eventsEventWrite(0,0,PROTOCOLAPP_CONFIGURATION_CHANGE_EVENT,0,0,0);
            
            // Send message that just entered with opcode PROTOCOLAPP_CONFIGURATION_CHANGE_EVENT
            eventsQueueStatusAndTailEntryAddressGet(&eventsEventsLogMemoryState, &NewEventEntryAddress);
            // Check the new Log memory status
            switch(eventsEventsLogMemoryState)
            {
            case EVENTS_QUEUESTATE_EMPTY:
              // No more events to send, Answer the read request with 0 payload bytes and updated status
              COMM_FSM_STATE_CHANGE(COMM_FSM_STATE_BLE_CONNECTED_EVENTS_WAIT , /*COMM_TASK_NO_EVENTS_BLE_CONNECTED_TIMEOUT*/ 3000);
              break;
            case EVENTS_QUEUESTATE_NOTEMPTY:
            case EVENTS_QUEUESTATE_FULL:
              // Read the current page (The page containing the current tail pointer), The NewEventEntryAddress in this case points to the message's spiFlashLogMemory Header.
              spiflashReqEnqueue(SPIFLASH_CMD_READ, NewEventEntryAddress, (uint8_t*)flashReadBuffer, 0, sizeof(spiflashLogMemoryHeaderT), commSpiCompletionCallBack, false);
              COMM_FSM_STATE_CHANGE(COMM_FSM_STATE_SPIREAD_FIRST_PAGE_WAIT, 1000);
              break;
            case COMM_FSM_QUEUE_ENTRY_TYPE_BLE_TIMEOUT:
              break;
            default:
              break;
            }       
          }
          
          break;
        case COMM_FSM_QUEUE_ENTRY_TYPE_SUSPEND_DURING_FWUPGRADE:
              COMM_FSM_STATE_CHANGE(COMM_FSM_STATE_APP_CONFIG_RESPONSE_WAIT, 50000);
              break;
          // This is what the server sends now 
        case COMM_FSM_QUEUE_ENTRY_TYPE_APP_ACK:
          break;
          // Config timeout 
          break;
        case COMM_FSM_QUEUE_ENTRY_TYPE_BLE_DISCONNECTED:  
        case COMM_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:
          // Wait for config set confirm
          if(commConfigSetThreshold)
          {
            commConfigSetThreshold--;
            break;
          }
          
          vlapmainDebugLog("Config Response Timeout, no pending events");
          bleControl(BLE_CONTROL_DISABLE);
          //eventBsipStatusUpdate(0,0);
          
          uint8_t enableSleepPolling = 1;
          if ((pccommnapplayerConnectivityStateGet() == PCCOMMAPPLAYER_CONNECTIVITY_STATE_CONNECTED) && !configConfigurationDb.EnableSleepWhilePolling)
          {
            enableSleepPolling = 0;
          }
#ifdef  SLEEP_ENABLED
          if (!measureMeasurementInProgressGet() /*&& !eventsPendingEventsStatusGet() */ &&
              i2cworkDuringMessageStateGet() == I2CWORK_NOT_DURING_MESSAGE && enableSleepPolling &&
              (ledsIndicationStateGet() != LEDS_ON) && HAL_GPIO_ReadPin(PB_IN_GPIO_Port, PB_IN_Pin) &&
              eventsStateGet() == EVENTS_FSM_STATE_IDLE &&
			        chargerFinishedUpdating)
          {
            vlapmainDebugLog("go to sleep mode");
            xSemaphoreTake( eventsNewEventSavedSemaphoreHandle, 1); // Take any non-relevant sempahore gives
            // Delay and let the message to be queued out 
            vTaskDelay(100);
            commSleepState = COMM_SLEEP_STATE_STOP_MODE;
            hwdriversVinGpioConfigure(COMM_SLEEP_STATE_STOP_MODE);
            measureModemStateTurnOff();
            //gpioTableActivate(ALL_GPIO_PINS_SLEEP);
            rtcSleepStart(); // Start sleep 
            // Sleep ended
            //gpioTableActivate(ALL_GPIO_PINS);
            
            switch(rtcWakeupReasonGet())
            {

            case RTC_WAKEUP_REASON_IDLE:
            	break;
            case RTC_WAKEUP_REASON_TIMED_WAKEUP:
            case RTC_WAKEUP_REASON_ALARM_A:
            case RTC_WAKEUP_REASON_ALARM_B:
            case RTC_WAKEUP_REASON_UNKNOWN:
              break;
            case RTC_WAKEUP_REASON_CHARGER_EVENT:
                  xSemaphoreTake( eventsNewEventSavedSemaphoreHandle, 5000);
                  break;
            case RTC_WAKEUP_REASON_PUSH_BUTTON:
              if(chargerDcPlugStatusGet() == CHARGER_DC_PLUG_INSERTED)
              {
                chargerRequestSend(CHARGER_REQ_INDICATION_ON);
              }
              else
              {
                if (!hwdriversGpioBitRead(HWDRIVERS_PUSH_BUTTON_IN))
                  measureTaskEventSend(MEASURE_PB_START, MEASURE_TASK_QUEUE_SEND_SOURCE_NORMAL);  
              }
              break;
            }
            
            
            commSleepState = COMM_SLEEP_STATE_NORMAL;
            hwdriversVinGpioConfigure(COMM_SLEEP_STATE_NORMAL);

           }
#endif           
          GatewayReconnectionRetryCounter = COMM_GATEWAY_RETRY_COUNT;
          commGoToIdle(100);
          break;
        default:
        	break;
        }
        break;
      } // Switch
    } // If true
    else
    {
      char PrintBuff[50];
      // Mailbox Rcv Timeout
      sprintf(PrintBuff, "Comm State = %d", COMM_FSM_STATE_GET);
      vlapmainDebugLog(PrintBuff);
    }
  } // While
}


void commGoToIdle(uint16_t Delay)
{
  bleControl(BLE_CONTROL_DISABLE);
  NoServerResponseRetryCounter = COMM_SERVER_RESPONSE_RETRY_COUNT;
  COMM_FSM_STATE_CHANGE(COMM_FSM_STATE_IDLE, Delay);
}


commSleepState_T commSleepModeStateGet()
{
 return(commSleepState);
} 


commFsmStateT commStateGet()
{
  return(COMM_FSM_STATE_GET);
}

void eventBsipStatusUpdate(uint16_t PendingBytes, uint32_t PendingMessages)
{
    uint8_t   *MsgPtr;
    uint16_t  MsgLength;


    MsgLength = sizeof(bleBsipDllHeader_t) + sizeof(bleReadResponse_t) + sizeof(bleBsipDllEndOfMessage_t);
    // Allocate memory for the full message.
    // The buffer will be freed by the low level UART driver
    MsgPtr = pvPortMalloc(MsgLength);
    if(MsgPtr)
    {
      // Update the Bsip read response darta struct
      ((bleReadResponse_t*)(MsgPtr+sizeof(bleBsipDllHeader_t)))->PayloadApplicationLayerOperationCode = BLE_BSIP_OPCODE_READ_RESP;
      // We force the configuration message length so the gateway will read it as if it is normal message
      ((bleReadResponse_t*)(MsgPtr+sizeof(bleBsipDllHeader_t)))->PendingBytesCurrentMessage = TYPES_ENDIAN16_CHANGE(PendingBytes);
      // We force the pending events in log memory to 1 to make the gateway to think there is one more event 
      ((bleReadResponse_t*)(MsgPtr+sizeof(bleBsipDllHeader_t)))->PendingMessagesInLogMemory = TYPES_ENDIAN32_CHANGE(PendingMessages);
      commStatusSyncCounter++;
      ((bleReadResponse_t*)(MsgPtr+sizeof(bleBsipDllHeader_t)))->StatusSyncCounter = TYPES_ENDIAN16_CHANGE(commStatusSyncCounter);
      // Send BSIP protocol message, The allocated message buffer will be freed by the UART DMA Tx ISR
      bleBsipDllMessageSend(MsgPtr, MsgLength);
      // Give time for the BLE message to propogate
      vTaskDelay(2000);
    }
}

/******************************************************************************
* @brief  void  commFsmTimerTimeoutCallback()
* @param  
* @retval 
******************************************************************************/
void commFsmTimerTimeoutCallback( )
{
  commTaskEventSend(COMM_FSM_QUEUE_ENTRY_TYPE_TIMEOUT);
}




/******************************************************************************
* @brief ReturnCode_T commSpiCompletionCallBack()
* @param  
* @retval 
******************************************************************************/
void commSpiCompletionCallBack()
{
  commFsmQueueEntryT CommRequest;
  
  CommRequest.DataLength        = 0;
  CommRequest.DataPtr   = 0;   
  CommRequest.ActionType   = COMM_FSM_QUEUE_ENTRY_TYPE_SPIFLASH_READ_COMPLETED;
  
  xQueueSend(commFsmQueueHandle, &CommRequest, 0);
}

/******************************************************************************
*** ReturnCode_T commTaskEventSend(commFsmQueueEntryActionType_T NotificationToCommTask )
* @brief  // Sends notifications to the commTask   
* @param  
* @retval 
******************************************************************************/
ReturnCode_T commTaskEventSend(commFsmQueueEntryActionType_T NotificationToCommTask )
{
  commFsmQueueEntryT  CommTaskQueueEntry;
  
  // Fill the queue entry
  CommTaskQueueEntry.ActionType = NotificationToCommTask;
  CommTaskQueueEntry.DataPtr    = 0;
  CommTaskQueueEntry.DataLength = 0;
  
  // Enqueue the event to the commTask input queue
  xQueueSend(commFsmQueueHandle, &CommTaskQueueEntry, 0);
  
  return(RETURNCODE_OK);
}




/******************************************************************************
*** ReturnCode_T ReturnCode_T commMessageSendNotViaFlash(uartdllChannel_t Channel, uint32_t RequrstOrCommandSessionId, PROTOCOLAPP_COMMANDS_T messageOpCode, char * PayloadRelatedDataPtr, uint16_t PayloadRelatedDataLength, uint8_t FreeMemoryFlag)
* @brief  // Sends notifications to the commTask   
* @param  
* @retval 
******************************************************************************/
ReturnCode_T commMessageSendNotViaFlash(uartdllChannel_t Channel, uint32_t RequrstOrCommandSessionId, PROTOCOLAPP_COMMANDS_T messageOpCode, uint8_t* PayloadRelatedDataPtr, uint16_t PayloadRelatedDataLength, uint8_t FreeMemoryFlag)
{

  uint8_t *ReturnedMsgPtr = 0;
  uint16_t ReturnedMsgLength;
  uint8_t ReturnedAddedPad;
  uint16_t ReturnedEncryptedMsgLength;
  uint8_t   *MsgPtr;
  uint16_t  MsgLength;


               // Build  Configuration download request message 
              protocolappMessageBuildnNotViaFlash(0, 0, messageOpCode, (char*)PayloadRelatedDataPtr, PayloadRelatedDataLength, flashReadBuffer, &ReturnedMsgLength, &ReturnedAddedPad);
              // the Encryption will be done over the flashReadbuffer assuming the message includes DllHeader_t + aescbcProtcolHeader_t + Payload, the encyption will return the encrypted msg length
             
                            // Please note the resulted buffer may contain padded bytes as required by the encryption algorithm
              ReturnCode_T ReturnCode = aescbcEncryptByChunks(flashReadBuffer + sizeof(DllHeader_t), ReturnedMsgLength - ReturnedAddedPad, &ReturnedEncryptedMsgLength);
              ReturnedEncryptedMsgLength += (sizeof(DllHeader_t) + sizeof(DllEndOfMessage_t));
              
              // Please note the resulted buffer may contain padded bytes as required by the encryption algorithm
              //ReturnCode_T ReturnCode = aescbcEncryptByChunks(flashReadBuffer, ReturnedMsgLength, &ReturnedEncryptedMsgLength);
              if(ReturnCode != RETURNCODE_OK)
              {
              }
              else
              uartdllCompleteMEssageBuild(flashReadBuffer, ReturnedEncryptedMsgLength);
#if !COMM_IS_BLE
              uartdllTxQueueEnqueue(UARTDLL_UART_2_CELLMODEM, flashReadBuffer, ReturnedEncryptedMsgLength, false);
              // Free the application protocol allocated buffer as it was already coppied to the BLE message buffer
              if(FreeMemoryFlag && ReturnedMsgPtr)
                vPortFree(ReturnedMsgPtr);
              // Give time for the BLE message to propogate
              vTaskDelay(50);
              // Go wait for Config Ack
#endif
#if COMM_IS_BLE   
              // 
              MsgLength = sizeof(bleBsipDllHeader_t) + sizeof(bleReadResponse_t) + ReturnedEncryptedMsgLength + sizeof(bleBsipDllEndOfMessage_t);
              // Allocate memory for the full message.
              // The buffer will be freed by the low level UART driver
              MsgPtr = pvPortMalloc(MsgLength);
              if(MsgPtr)
              {
                // Update the Bsip read response data struct
                ((bleReadResponse_t*)(MsgPtr+sizeof(bleBsipDllHeader_t)))->PayloadApplicationLayerOperationCode = BLE_BSIP_OPCODE_READ_RESP;
                // We force the configuration message length so the gateway will read it as if it is normal message
                ((bleReadResponse_t*)(MsgPtr+sizeof(bleBsipDllHeader_t)))->PendingBytesCurrentMessage = TYPES_ENDIAN16_CHANGE(ReturnedEncryptedMsgLength);
                // We force the pending events in log memory to 1 to make the gateway to think there is one more event 
                ((bleReadResponse_t*)(MsgPtr+sizeof(bleBsipDllHeader_t)))->PendingMessagesInLogMemory = TYPES_ENDIAN32_CHANGE(1);
                commStatusSyncCounter++;
                ((bleReadResponse_t*)(MsgPtr+sizeof(bleBsipDllHeader_t)))->StatusSyncCounter = TYPES_ENDIAN16_CHANGE(commStatusSyncCounter);
                memcpy(MsgPtr+sizeof(bleBsipDllHeader_t) + sizeof(bleReadResponse_t), flashReadBuffer, ReturnedEncryptedMsgLength);
                // Send BSIP protocol message, The allocated message buffer will be freed by the UART DMA Tx ISR
                bleBsipDllMessageSend(MsgPtr, MsgLength);
                
                commDecryptAndUpdateGeneralHeader(2);
                
                // Give time for the BLE message to propogate
                vTaskDelay(50);
                // Go wait for Config Ack
                COMM_FSM_STATE_CHANGE(COMM_FSM_STATE_APP_CONFIG_RESPONSE_WAIT, 50000/*40000*/);
              }
              else
              {
                // Free the application protocol allocated buffer even if the BLE buffer allocation failed
                if(ReturnedMsgPtr)
                  vPortFree(ReturnedMsgPtr);
                // No memory to allocate, go to idle
                vlapmainDebugLog("Config Request Event - No memory to allocate");
              }
#endif
              return(RETURNCODE_OK);
}

void commDecryptAndUpdateGeneralHeader(uint8_t Source)
{
	// The size of the decryption should be a product of the key size
  uint16_t protocolappHeaderDecryptSize = (sizeof(ProtocolappHeader_t) / sizeof(configProductionDb.UpStreamAesCbcKey)) * sizeof(configProductionDb.UpStreamAesCbcKey);
  if(sizeof(ProtocolappHeader_t) % sizeof(configProductionDb.UpStreamAesCbcKey))
  {
    protocolappHeaderDecryptSize += sizeof(configProductionDb.UpStreamAesCbcKey);
  }
  // Decrypt and don't verify CRC because it only decrypts the protocol header
  aescbcDecrypt(flashReadBuffer + sizeof(DllHeader_t), outputDecryptProtocolHeaderPtr, protocolappHeaderDecryptSize, configProductionDb.UpStreamAesCbcKey, 0);  
  memcpy((void *) &commPendingProtocolHeaderEventPtr, outputDecryptProtocolHeaderPtr, sizeof(ProtocolappHeader_t));
}

ProtocolappHeader_t * commPendingProtocolHeaderEventPtrGet()
{
  return &commPendingProtocolHeaderEventPtr;
}

void commSkipCorruptedEvent(commSpiCorruption_T type, uint32_t* address, uint8_t* corruptedEventHeader)
{
  if (!syncSearchOccured)
  {
    commCorruptedEventDescriptor = (commCorruptedEventDescriptorT){0};
    commCorruptedEventDescriptor.CurrentPage  = (*address - SPIFLASH_ZONE_EVENTS_START) / SPIFLASH_SPI_PAGESIZE;
    commCorruptedEventDescriptor.CurrentSpiFlashAddress  = *address;
    commCorruptedEventDescriptor.SyncPattern = ((spiflashLogMemoryHeaderT*)corruptedEventHeader)->SyncPattern;
    commCorruptedEventDescriptor.EntrySize = (uint16_t)((spiflashLogMemoryHeaderT*)corruptedEventHeader)->EntrySize;
    commCorruptedEventDescriptor.PrevEntryAddress = ((spiflashLogMemoryHeaderT*)corruptedEventHeader)->PrevEntryAddress;
    commCorruptedEventDescriptor.MessagesInMemory = eventsPendingEventsStatusGet();
    commCorruptedEventDescriptor.CorruptionType = type;
  }
  COMM_FSM_STATE_CHANGE(COMM_FSM_STATE_SPIREAD_SYNC_SEARCH, 1000);
  if (*address >= SPIFLASH_ZONE_EVENTS_END)
  {
    *address = SPIFLASH_ZONE_EVENTS_START;
  }
  else
  {
    ++(*address);
  }
  spiflashReqEnqueue(SPIFLASH_CMD_READ, (*address), (uint8_t*)(flashReadBuffer), 0, SPIFLASH_SPI_PAGESIZE, commSpiCompletionCallBack, false);
}

uint32_t commGetCorruptedEventSkipSize(uint32_t address, uint16_t msgSize)
{
  // Skipping event to start of SPI event area (was at the end of the circular buffer)
  if (address < commCorruptedEventDescriptor.CurrentSpiFlashAddress) 
  {
    return (address - SPIFLASH_ZONE_EVENTS_START) + ( (SPIFLASH_ZONE_EVENTS_END - commCorruptedEventDescriptor.CurrentSpiFlashAddress)) + msgSize;
  }
  else 
  {
    return (address - commCorruptedEventDescriptor.CurrentSpiFlashAddress + msgSize);
  }
}
