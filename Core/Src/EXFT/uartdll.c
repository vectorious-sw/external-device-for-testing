#include <hwdrivers.h>
#include "uartDll.h"
#include "protocolApp.h"
#include "crc32.h"
#include "inet.h"
#include "transQ.h"
#include "timers.h"



// L O C A L    D E F I N I T I O N S
#define MODEM_CR              13
#define MODEM_RX_BUFFER_SIZE        30
//#define UARTDLL_INTER_CHAR_TIMEOUT      1000
#define uartdll_RX_BUFF_LENGTH        1200

uint16_t UARTDLL_INTER_CHAR_TIMEOUT = 1000;

// L O C A L    P R O T O T Y P E S
void TimeoutHandler(uartdllChannel_t Channel);
void  uartRxDellFsmTimerCallback(TimerHandle_t pxExpiredTimer );


// G L O B A L S

// Queue Control Blocks

QueueHandle_t UartTransQueueHandle1;
QueueHandle_t UartTransQueueHandle2;
QueueHandle_t UartTransQueueHandle3;
QueueHandle_t UartTransQueueHandle6;


uint8_t uartUsart1MemFreeNeeded;
uint8_t uartUsart6MemFreeNeeded;

uint32_t  uartdllAccumulatedTxCrc32;

char modemRxBuffer[MODEM_RX_BUFFER_SIZE];
// Modem Rx buffer index
uint8_t modemRxBufferIndex=0;
// Modem FSM states
uartdllRxState_t uartdllRxState[UARTDLL_NUMBER_OF_CHANNLES];
uint8_t uartdllRcvBuffer[UARTDLL_NUMBER_OF_CHANNLES][uartdll_RX_BUFF_LENGTH];
uint16_t uartdllRcvBuffIndex[UARTDLL_NUMBER_OF_CHANNLES];
uint16_t uartdllMessageLength[UARTDLL_NUMBER_OF_CHANNLES];
// The accumulated CRC32 word being built along the reception of the buffer 
uint32_t uartdllAccumulatedRxCrc32[UARTDLL_NUMBER_OF_CHANNLES];
// These variables manages the reception of the CRC32 bytes sent at the end of each message.
uint8_t uartdllReceivedCrc32BytesCounter[UARTDLL_NUMBER_OF_CHANNLES];
uint32_t uartdllReceivedCrc32[UARTDLL_NUMBER_OF_CHANNLES];

// uartdll RxFSm timer handler IDs
const uint8_t uartDllRxFsmTimerIdAraay[] = {UARTDLL_UART_1_VTASGUI, UARTDLL_UART_2_CELLMODEM, UARTDLL_UART_3_BLE, UARTDLL_UART_6_RS485};
// uart dll rx fsm timer handlers array
TimerHandle_t uartDllRxFsmTimerHandler[UARTDLL_NUMBER_OF_CHANNLES];

// uartdll reception statistics counters
uartdllStatisticsCounters_t uartdllStatisticsCounters[UARTDLL_NUMBER_OF_CHANNLES];

uartdllFsmState_T uartdllFsmState;
QueueHandle_t uartdllFsmQueueHandle;



volatile char myInChar;
volatile uartdllFsmStimuli_t myStimuli;
volatile uartdllChannel_t myChannel;


volatile char myBuff[300];
volatile uint32_t myBuffIndex;

/***********************************************************************************
 *** ReturnCode_T uartdllInit()
 *
 *
 *
 *
 ***********************************************************************************/
ReturnCode_T uartdllInit()
{
  // Default state after power up
  uartdllRxState[0]=UARTDLL_FSM_IDLE;
  uartdllRxState[1]=UARTDLL_FSM_IDLE;
  uartdllRxState[2]=UARTDLL_FSM_IDLE;
  uartdllRxState[3]=UARTDLL_FSM_IDLE;
  
  // Create uartdll task to 
  
  // Create uartdll message queueu
  
    // Create input queue  
  uartdllFsmQueueHandle = xQueueCreate(30, sizeof(uartdllFsmQueueEntryT));
  uartdllFsmState = UARTDLL_FSM_STATE_IDLE;

  
 
  uartDllRxFsmTimerHandler[UARTDLL_UART_1_VTASGUI]    =  xTimerCreate("Uart1DllTimer", portTICK_PERIOD_MS, pdFALSE,(void *)&uartDllRxFsmTimerIdAraay[UARTDLL_UART_1_VTASGUI]  , uartRxDellFsmTimerCallback);
  uartDllRxFsmTimerHandler[UARTDLL_UART_2_CELLMODEM]  =  xTimerCreate("Uart2DllTimer", portTICK_PERIOD_MS, pdFALSE,(void *)&uartDllRxFsmTimerIdAraay[UARTDLL_UART_2_CELLMODEM], uartRxDellFsmTimerCallback);
  uartDllRxFsmTimerHandler[UARTDLL_UART_3_BLE]        =  xTimerCreate("Uart3DllTimer", portTICK_PERIOD_MS, pdFALSE,(void *)&uartDllRxFsmTimerIdAraay[UARTDLL_UART_3_BLE]      , uartRxDellFsmTimerCallback);
  uartDllRxFsmTimerHandler[UARTDLL_UART_6_RS485]      =  xTimerCreate("Uart6DllTimer", portTICK_PERIOD_MS, pdFALSE,(void *)&uartDllRxFsmTimerIdAraay[UARTDLL_UART_6_RS485]    , uartRxDellFsmTimerCallback);

 
  
  // Create the task
  xTaskCreate(uartdllTask, uartdllTaskName, uartdllTaskSTACK_SIZE, NULL, 
              uartdllTaskPriority, ( TaskHandle_t * ) NULL );
  // Allways return OK
  //myBuffIndex = 0;
  
  // Initialize the UART Tx Queue for channels 1 and 2
  
  if (transQInit(20, &UartTransQueueHandle1) != TRANSQ_OK)
    return(RETURNCODE_ERROR);
  else
    if (transQInit(20, &UartTransQueueHandle2) != TRANSQ_OK)
      return(RETURNCODE_ERROR);
    else
      if (transQInit(20, &UartTransQueueHandle3) != TRANSQ_OK)
        return(RETURNCODE_ERROR);
      else
        if (transQInit(20, &UartTransQueueHandle6) != TRANSQ_OK)
          return(RETURNCODE_ERROR);
        else
          return(RETURNCODE_OK);

}


/******************************************************************************
* @brief  void UARTDLL_STATE_CHANGE( uartdllChannel_t Channel, uartdllRxState_t NextState, uint16_t Timeout)
* @param  
* @retval 
******************************************************************************/
void UARTDLL_STATE_CHANGE( uartdllChannel_t Channel, uartdllRxState_t NextState, uint16_t Timeout)
{
  uartdllRxState[Channel] = NextState;
  xTimerChangePeriod(uartDllRxFsmTimerHandler[Channel], Timeout, 100);
}


/******************************************************************************
* @brief  portTASK_FUNCTION(uartdllFsm, pvParameters )
* @param  
* @retval 
******************************************************************************/
portTASK_FUNCTION(uartdllTask, pvParameters )
{
  uartdllFsmQueueEntryT QueueEntry;
  uartdllFsmState_T Stimuli;
  uint16_t i;
  
  while(1)
  {
    // block task till new queue entry is received or timeout 
    BaseType_t  QueueState = xQueueReceive(uartdllFsmQueueHandle, &QueueEntry, 10000);
    // Convert the queue status to FSM Stimuli
    if(QueueState == pdTRUE)
    {
      if(QueueEntry.ActionType < UARTDLL_FSM_QUEUE_ENTRY_TYPE_LAST_ENTRY) 
        Stimuli = UARTDLL_FSM_STIMULI_NEW_TOKEN_RECVD;
      else
        Stimuli = UARTDLL_FSM_STIMULI_TIMEOUT;  
    }
    else
      Stimuli = UARTDLL_FSM_STIMULI_TIMEOUT;  
    
      switch(Stimuli)
      {
      case UARTDLL_FSM_STIMULI_NEW_TOKEN_RECVD:
        for(i=0; i<QueueEntry.DataLength; i++)
          uartDllRxFsm(UARTDLL_UART_3_BLE, UARTDLL_FSM_STIMULI_INCOMMING_CHAR, *(QueueEntry.DataPtr+i));
        vPortFree(QueueEntry.DataPtr);
        break;
      case UARTDLL_FSM_STIMULI_TIMEOUT:
        break;
      }
  } // While
}




/******************************************************************************
*** ReturnCode_T uartdllTaskEventSend(uartdllFsmQueueEntryActionType_T NotificationToUartdllTask, uint8_t * DatePtr, uint16 DataLength)
* @brief  // Sends notifications to the uartdllTask   
* @param  
* @retval 
******************************************************************************/
ReturnCode_T uartdllTaskEventSend(uartdllFsmQueueEntryActionType_T NotificationToUartdllTask, uint8_t * DatePtr, uint16_t DataLength)
{
  uartdllFsmQueueEntryT  UartdllTaskQueueEntry;
  
  // Fill the queue entry
  UartdllTaskQueueEntry.ActionType = NotificationToUartdllTask;
  UartdllTaskQueueEntry.DataPtr    = DatePtr;
  UartdllTaskQueueEntry.DataLength = DataLength;
 
  // Enqueue the event to the uartdllTask input queue
  xQueueSend(uartdllFsmQueueHandle, &UartdllTaskQueueEntry, 0);

  return(RETURNCODE_OK);
}

/***********************************************************************************
 *** ReturnCode_T  uartDllRxFsm( uartdllChannel_t Channel, uartdllFsmStimuli_t stimuli, char  inChar )
 *
 *
 *
 *
 ***********************************************************************************/
ReturnCode_T  uartDllRxFsm( uartdllChannel_t Channel, uartdllFsmStimuli_t stimuli, char  inChar )
{
  
  myInChar  = inChar;
  myStimuli = stimuli;
  myChannel = Channel;
#if 1
  myBuff[myBuffIndex] = inChar;
  myBuffIndex++;
  if(myBuffIndex >= sizeof(myBuff))
    myBuffIndex = 0;
#endif
   
  
  xTimerStop(uartDllRxFsmTimerHandler[Channel],100);
  
  switch(uartdllRxState[Channel])
  {
    // I D L E
  case  UARTDLL_FSM_IDLE:
    if(stimuli==UARTDLL_FSM_STIMULI_INCOMMING_CHAR)
      if(inChar==UARTDLL_SYNC_WORD_MSB)
      {
        // UARTDLL_SYNC_WORD_MSB found, message probably started
        UARTDLL_STATE_CHANGE(Channel, UARTDLL_FSM_SYNC_LSB_WAIT, UARTDLL_INTER_CHAR_TIMEOUT);
      }
    break;
    // WAIT SYNC LSByte   
  case  UARTDLL_FSM_SYNC_LSB_WAIT:
    switch(stimuli)
    {
    case UARTDLL_FSM_STIMULI_INCOMMING_CHAR:
      if(inChar==UARTDLL_SYNC_WORD_LSB)
      {
        // New message, reset the buffer index
        uartdllRcvBuffIndex[Channel]=0;
        // UARTDLL_SYNC_WORD_MSB found, message probably started
        UARTDLL_STATE_CHANGE(Channel, UARTDLL_FSM_MSG_LENGTH_MSB_WAIT, UARTDLL_INTER_CHAR_TIMEOUT);
      }
      else
        // Sync LSB not following the SYNC MSB, Go to idle
        TimeoutHandler(Channel);
      break;
    case UARTDLL_FSM_STIMULI_TIMOUT:
      TimeoutHandler(Channel);
      break;
    }
    break;
    // WAIT Length MSB  
  case  UARTDLL_FSM_MSG_LENGTH_MSB_WAIT:
    switch(stimuli)
    {
    case UARTDLL_FSM_STIMULI_INCOMMING_CHAR:
      // Store the char into the rx buffer
      uartdllRcvBuffer[Channel][uartdllRcvBuffIndex[Channel]++]=inChar;
      // The received message length MSByte is placed in the MSByte of the length uint16
      uartdllMessageLength[Channel] = (((uint16_t)inChar)<<8);
      // Go wait for the message length
      UARTDLL_STATE_CHANGE(Channel, UARTDLL_FSM_MSG_LENGTH_LSB_WAIT, UARTDLL_INTER_CHAR_TIMEOUT);
      break;
    case UARTDLL_FSM_STIMULI_TIMOUT:
      TimeoutHandler(Channel);
      break;
    }
    break;
    // WAIT Length LSB  
  case  UARTDLL_FSM_MSG_LENGTH_LSB_WAIT:
    switch(stimuli)
    {
    case UARTDLL_FSM_STIMULI_INCOMMING_CHAR:
      // Store the char into the rx buffer
      uartdllRcvBuffer[Channel][uartdllRcvBuffIndex[Channel]++]=inChar;
      // The received message length LSByte is placed in the LSByte of the length uint16
      uartdllMessageLength[Channel]+=inChar;
      // Init the received CRC32 bytes counter to be later used when receiving the 4 CRC32 bytes
      uartdllReceivedCrc32BytesCounter[Channel]= sizeof(uint32_t);
      // Set the initials accumulated CRC32 vlaue for this session
      uartdllAccumulatedRxCrc32[Channel] = crc32Init();
      // Special case for zero message length or message length larger than the rcv buffer, just go to IDLE and cancle the reception
      if(!uartdllMessageLength[Channel]  ||  (uartdllMessageLength[Channel] > sizeof(uartdllRcvBuffer[Channel])))
         UARTDLL_STATE_CHANGE(Channel, UARTDLL_FSM_IDLE, 1);
      else
        // Go wait for payload
         UARTDLL_STATE_CHANGE(Channel, UARTDLL_FSM_MSG_PAYLOAD_RCV_WAIT, UARTDLL_INTER_CHAR_TIMEOUT);
      break;
    case UARTDLL_FSM_STIMULI_TIMOUT:
      TimeoutHandler(Channel);
      break;
    }
    break;
    // WAIT PAYLOAD  
  case  UARTDLL_FSM_MSG_PAYLOAD_RCV_WAIT:
    switch(stimuli)
    {
    case UARTDLL_FSM_STIMULI_INCOMMING_CHAR:
      // Accumulate CRC32 based on the last known value
      uartdllAccumulatedRxCrc32[Channel] = crc32ByteCalc(uartdllAccumulatedRxCrc32[Channel], inChar);
      // Store the char into the rx buffer
      uartdllRcvBuffer[Channel][uartdllRcvBuffIndex[Channel]++]=inChar;
      // Decrement message length
      if (uartdllMessageLength[Channel])
        uartdllMessageLength[Channel]--;
      // Is it the last byte of this message
      if(!uartdllMessageLength[Channel])
      {
        // 4 CRC32 bytes, 3 to 0
        uartdllReceivedCrc32BytesCounter[Channel]=UARTDLL_NUMBER_OF_CRC_BYTES;
        // Go wait for checksum
        UARTDLL_STATE_CHANGE(Channel, UARTDLL_FSM_MSG_CRC32_3_WAIT, UARTDLL_INTER_CHAR_TIMEOUT);
       }
      else
      {
          UARTDLL_STATE_CHANGE(Channel, UARTDLL_FSM_MSG_PAYLOAD_RCV_WAIT, UARTDLL_INTER_CHAR_TIMEOUT);
      }
      break;
    case UARTDLL_FSM_STIMULI_TIMOUT:
      TimeoutHandler(Channel);
      break;
    }
    break;
    // WAIT CRC32  bytes
  case  UARTDLL_FSM_MSG_CRC32_3_WAIT:
    UARTDLL_STATE_CHANGE(Channel, UARTDLL_FSM_MSG_CRC32_2_WAIT, UARTDLL_INTER_CHAR_TIMEOUT);
    // Clear the received CRC32 byte
    uartdllReceivedCrc32[Channel] = 0;
  case  UARTDLL_FSM_MSG_CRC32_2_WAIT:
    UARTDLL_STATE_CHANGE(Channel, UARTDLL_FSM_MSG_CRC32_1_WAIT, UARTDLL_INTER_CHAR_TIMEOUT);
  case  UARTDLL_FSM_MSG_CRC32_1_WAIT:
    UARTDLL_STATE_CHANGE(Channel, UARTDLL_FSM_MSG_CRC32_0_WAIT, UARTDLL_INTER_CHAR_TIMEOUT);
  case  UARTDLL_FSM_MSG_CRC32_0_WAIT:
    // Because of the common case we need to check if the stimuli is not a timeout before
    if(stimuli == UARTDLL_FSM_STIMULI_TIMOUT)
    {
      TimeoutHandler(Channel);
    }
    switch(uartdllRxState[Channel])
    {
    case  UARTDLL_FSM_MSG_CRC32_3_WAIT:
      UARTDLL_STATE_CHANGE(Channel, UARTDLL_FSM_MSG_CRC32_2_WAIT, UARTDLL_INTER_CHAR_TIMEOUT);
      // Clear the received CRC32 byte
      uartdllReceivedCrc32[Channel] = 0;
      break;
    case  UARTDLL_FSM_MSG_CRC32_2_WAIT:
      UARTDLL_STATE_CHANGE(Channel, UARTDLL_FSM_MSG_CRC32_1_WAIT, UARTDLL_INTER_CHAR_TIMEOUT);
      break;
    case  UARTDLL_FSM_MSG_CRC32_1_WAIT:
      UARTDLL_STATE_CHANGE(Channel, UARTDLL_FSM_MSG_CRC32_0_WAIT, UARTDLL_INTER_CHAR_TIMEOUT);
      break;
    }
    switch(stimuli)
    {
    case UARTDLL_FSM_STIMULI_INCOMMING_CHAR:
      // Decrement the Received CRC32 bytes counter
      if(uartdllReceivedCrc32BytesCounter[Channel]) 
        uartdllReceivedCrc32BytesCounter[Channel]--;
      // Form the CRC32 word byte by byte till the forth byte is received 
      uartdllReceivedCrc32[Channel]|=(inChar<<(uartdllReceivedCrc32BytesCounter[Channel])*8); 
      // In case this is the last byte, Test the calculated CRC32 against the received CRC32 
      if(!uartdllReceivedCrc32BytesCounter[Channel])
      {
        // does the CRC32 match, if so, call the application layer
        if(uartdllReceivedCrc32[Channel]== ~uartdllAccumulatedRxCrc32[Channel])
        {
          // Update reception statistics, Good packets
          uartdllStatisticsCounterUpdate(Channel, UARTDLL_RCV_STATSTICS_GOOD_PACKETS_COUNTER);
          uint32_t messageLength = (*(uartdllRcvBuffer[Channel]) * 256) + *(uartdllRcvBuffer[Channel] + 1);
          // Call the application layer, The two sync bytes are not stored, so the pointer points to the encryption header start
          protocolappHandlePacket(Channel, uartdllRcvBuffer[Channel]+2, messageLength);
        }
        else
        {
          // Update reception statistics, Errored packets
          uartdllStatisticsCounterUpdate(Channel, UARTDLL_RCV_STATSTICS_CRC_ERROR_COUNTER );
        }       
         //UARTDLL_STATE_CHANGE(Channel, UARTDLL_FSM_IDLE, UARTDLL_INTER_CHAR_TIMEOUT);
        uartdllRxState[Channel] = UARTDLL_FSM_IDLE;
      }
      break;
    case UARTDLL_FSM_STIMULI_TIMOUT:
      TimeoutHandler(Channel);
      break;
    }
    break;
    
  }
  return(RETURNCODE_OK);
}




void  uartRxDellFsmTimerCallback(TimerHandle_t pxExpiredTimer )
{
  uint8_t MyBuffer[100]; 
  uartdllChannel_t Channel = *(uartdllChannel_t*)pvTimerGetTimerID(pxExpiredTimer);
  

  switch(Channel)
  {
  case UARTDLL_UART_1_VTASGUI:
	  break;
  case UARTDLL_UART_2_CELLMODEM:
	  break;
  case UARTDLL_UART_3_BLE:
	  if((uartdllRxState[Channel] != UARTDLL_FSM_IDLE) && (uartdllRxState[Channel] != UARTDLL_FSM_MSG_CRC32_0_WAIT))
	  {
	    sprintf(MyBuffer, "uartdll timeout Channel: %d State: %d\n\r", Channel, uartdllRxState[Channel]);
	    vlapmainDebugLog(MyBuffer);
	  }
	  uartDllRxFsm(Channel, UARTDLL_FSM_STIMULI_TIMOUT, 0);
	break;
  case UARTDLL_UART_6_RS485:
	  break;
  }
}


/***********************************************************************************
 *** void TimeoutHandler()
 *
 *
 *
 *
 ***********************************************************************************/
void TimeoutHandler(uartdllChannel_t Channel)
{
  // Inter char timeout, message reception aborted, go to idle
  // Update UARTDLL reception statistics
  uartdllStatisticsCounterUpdate(Channel, UARTDLL_RCV_STATSTICS_INTER_CHAR_TIMEOUT_COUNTER);
  // go wait for next message
  uartdllRxState[Channel]=UARTDLL_FSM_IDLE;
}

/***********************************************************************************
 *** ReturnCode_T uartdllStatisticsCounterUpdate(uartdllRxStatistics_t counterId)
 *
 *
 *
 *
 *
 ***********************************************************************************/
ReturnCode_T uartdllStatisticsCounterUpdate(uartdllChannel_t Channel, uartdllRxStatistics_t counterId)
{
  // Pointo to the channel's entry
  uartdllStatisticsCounters_t* Ptr = &uartdllStatisticsCounters[Channel];  
  
  switch(counterId)
  {
  case UARTDLL_RCV_STATSTICS_INTER_CHAR_TIMEOUT_COUNTER:
    Ptr->interCharTimeOutCounter++;
    break;
  case UARTDLL_RCV_STATSTICS_CRC_ERROR_COUNTER:
    Ptr->crcErrorCounter++;
    break;
  case UARTDLL_RCV_STATSTICS_GOOD_PACKETS_COUNTER:
    Ptr->RxMessageCounter++;
    break;
  default:
    break;
  }
  return(RETURNCODE_OK);
}


/***********************************************************************************
 *** ReturnCode_T uartdllStatisticsCounterClear(uartdllChannel_t Channel, uartdllRxStatistics_t counterId)
 *
 *
 *
 *
 ***********************************************************************************/
ReturnCode_T uartdllStatisticsCounterClear(uartdllChannel_t Channel, uartdllRxStatistics_t counterId)
{
  uint8_t i;
  // Pointo to the channel's entry
  uint8_t* ClearPtr = (uint8_t*)&uartdllStatisticsCounters[Channel];  
  // Pointo to the channel's entry
  uartdllStatisticsCounters_t* Ptr = &uartdllStatisticsCounters[Channel];  
  
  switch(counterId)
  {
  case UARTDLL_RCV_STATSTICS_INTER_CHAR_TIMEOUT_COUNTER:
    Ptr->crcErrorCounter = 0;
    break;
  case UARTDLL_RCV_STATSTICS_CRC_ERROR_COUNTER:
    Ptr->crcErrorCounter = 0;
    break;
  case UARTDLL_RCV_STATSTICS_GOOD_PACKETS_COUNTER:
    Ptr->RxMessageCounter = 0;
    break;
  case UARTDLL_RCV_STATSTICS_ALL_COUNTERS:
    for(i=0; i<sizeof(uartdllStatisticsCounters[Channel]); i++)
      *(ClearPtr++) = 0;
    break;
  default:
    break;
  }
  return(RETURNCODE_OK);
}



/**************************************************************************
 *** ReturnCode_T uartdllTxQueueEnqueue(uartdllChannel_t Channel, uint8_t * DataPtr, uint16_t Length )
 *
 *
 *
 *
 ***************************************************************************/
ReturnCode_T uartdllTxQueueEnqueue(uartdllChannel_t Channel, uint8_t * DataPtr, uint16_t Length, uint8_t MemoryFreeNeeded )
{
  // Assume status is OK
  ReturnCode_T ReturnCode = RETURNCODE_OK;
  ReturnCode_T ReturnCode1 = RETURNCODE_OK;
  
  QueueHandle_t myQueue = NULL;  

  // Switch over Channel as each UART has its Tx queue
  switch(Channel)
  {
  case UARTDLL_UART_1_VTASGUI:
    myQueue = UartTransQueueHandle1;
    break;
  case UARTDLL_UART_2_CELLMODEM:
    myQueue = UartTransQueueHandle2;
    break;
  case UARTDLL_UART_3_BLE:
    myQueue = UartTransQueueHandle3;
    break;
 case UARTDLL_UART_6_RS485:
    myQueue = UartTransQueueHandle6;
    break;
  }
  
  //if(Length < 1023)
  if(Length < 4500)
  {
    // Enqueue to channel 0 queue, handle queue exceptions by returning ERROR status
    if(transQEnqueue(myQueue, DataPtr, Length, MemoryFreeNeeded) != TRANSQ_OK)
      ReturnCode = RETURNCODE_ERROR;
    else
      ReturnCode = RETURNCODE_OK;
    
    return(ReturnCode);
  }
  else
  {
    // When length is larger than 1023 we need to issue two DMA transmissions
    if(transQEnqueue(myQueue, DataPtr, 1023, MemoryFreeNeeded ) != TRANSQ_OK)
      ReturnCode = RETURNCODE_ERROR;
    else
      ReturnCode = RETURNCODE_OK;
    
    if(transQEnqueue(myQueue, DataPtr+1023, Length-1023, MemoryFreeNeeded) != TRANSQ_OK)
      ReturnCode1 = RETURNCODE_ERROR;
    else
      ReturnCode1 = RETURNCODE_OK;
    
    if(ReturnCode == ReturnCode1)
      return(ReturnCode);
    else
      return(RETURNCODE_ERROR);
  }
}



/**************************************************************************
 *** ReturnCode_T uartdllMessageBuild(uartdllChannel_t Channel, uint8_t OpCode, char * PayloadPtr, uint16_t PayloadLength, )
 *
 *
 *
 *
 ***************************************************************************/
ReturnCode_T uartdllMessageSend(uartdllChannel_t Channel, uartdllTxPhaseT TxPhase, uint8_t * MessagePtr, uint16_t MessageLength, uint16_t TxChunkSize)
{
  DllHeader_t* myDllHeaderPtr;
  DllEndOfMessage_t* myDllEndOfMessagePtr = NULL;
  // The payload buffer length = The message length - Dll header and Dll End  of message
  uint16_t PayloadLength = MessageLength - sizeof(DllHeader_t) - sizeof(DllEndOfMessage_t);


  switch(TxPhase)
  {
  case UARTDLL_TX_FIRST_CHUNK:
      // Point to the DLL header 
      myDllHeaderPtr = (DllHeader_t*)MessagePtr;
      // Point to the DLL postfix header (The CRC32)
      myDllEndOfMessagePtr = (DllEndOfMessage_t*)(MessagePtr + MessageLength - sizeof(myDllEndOfMessagePtr));
      
      // Fill the Dll message
      myDllHeaderPtr->Sync0 = UARTDLL_SYNC_WORD_MSB;
      myDllHeaderPtr->Sync1 = UARTDLL_SYNC_WORD_LSB;
      // The payload length excluding the DLL header
      myDllHeaderPtr->PayloadLength = HTONS(PayloadLength);
      // Init the Tx CRC32 accumulator 
      uartdllAccumulatedTxCrc32 = crc32Init();
      // Accumulate CRC32 for this chunk
      uartdllAccumulatedTxCrc32 =  crc32BuffAccumulate(uartdllAccumulatedTxCrc32, MessagePtr, sizeof(DllHeader_t), TxChunkSize);
      break;
  case UARTDLL_TX_MIDDLE_CHUNK:
          uartdllAccumulatedTxCrc32 =  crc32BuffAccumulate(uartdllAccumulatedTxCrc32, MessagePtr, sizeof(DllHeader_t), TxChunkSize);
    break;
  case UARTDLL_TX_LAST_CHUNK:
      myDllEndOfMessagePtr->CRC32 = HTONL(uartdllAccumulatedTxCrc32);   
    break;
  }
  
  
  // Enqueue the application message to the Tx queue, 
  if(uartdllTxQueueEnqueue(Channel, MessagePtr, TxChunkSize, true) == RETURNCODE_OK)
    return(RETURNCODE_OK);
  else
  {
    // Failed, Queue is full
    return(RETURNCODE_ERROR);
  }
}




/**************************************************************************
 *** ReturnCode_T uartdllCompleteMEssageBuild(uint8_t * MessagePtr, uint16_t MessageLength)
 *
 *
 *
 *
 ***************************************************************************/
ReturnCode_T uartdllCompleteMEssageBuild(uint8_t * MessagePtr, uint16_t MessageLength)
{
  ReturnCode_T MyReturn;
  DllHeader_t* myDllHeaderPtr;
  DllEndOfMessage_t* myDllEndOfMessagePtr;
  
  if(MessageLength)
  {
    // The payload buffer length = The message length - Dll header and Dll End  of message
    uint16_t PayloadLength = MessageLength - sizeof(DllHeader_t) - sizeof(DllEndOfMessage_t);
    
    // Point to the DLL header 
    myDllHeaderPtr = (DllHeader_t*)MessagePtr;
    // Point to the DLL postfix header (The CRC32)
    myDllEndOfMessagePtr = (DllEndOfMessage_t*)(MessagePtr + MessageLength - sizeof(myDllEndOfMessagePtr));
    
    // Fill the Dll message
    myDllHeaderPtr->Sync0 = UARTDLL_SYNC_WORD_MSB;
    myDllHeaderPtr->Sync1 = UARTDLL_SYNC_WORD_LSB;
    // The payload length excluding the DLL header
    myDllHeaderPtr->PayloadLength = HTONS(PayloadLength);
    
    // Calculate CheckSum and fill the Dll End Of Message
    uint32_t Crc32Result =  crc32BuffCalc(MessagePtr, sizeof(DllHeader_t), PayloadLength);
    myDllEndOfMessagePtr->CRC32 = HTONL(Crc32Result);  
    MyReturn = RETURNCODE_OK;
  }
  else
    MyReturn = RETURNCODE_ERROR;
  
  return(MyReturn);
}


uint32_t uartdllDllMessageProcessCrc32QAccumulator;

/**************************************************************************
 *** ReturnCode_T uartdllDllMessageProcess(uartdllTxPhaseT TxPhase, uint8_t * MessagePtr, uint16_t MessageLength, uint16_t TxChunkSize)
 *
 *
 *
 *
 ***************************************************************************/
ReturnCode_T uartdllDllMessageProcess(uartdllTxPhaseT TxPhase, uint8_t * MessagePtr, uint16_t MessageLength, uint16_t TxChunkSize)
{
  DllHeader_t* myDllHeaderPtr;
  DllEndOfMessage_t* myDllEndOfMessagePtr = NULL;
  // The payload buffer length = The message length - Dll header and Dll End  of message
  uint16_t PayloadLength = MessageLength - sizeof(DllHeader_t) - sizeof(DllEndOfMessage_t);
  

  switch(TxPhase)
  {
  case UARTDLL_TX_FIRST_CHUNK:
      // Point to the DLL header 
      myDllHeaderPtr = (DllHeader_t*)MessagePtr;
      // Point to the DLL postfix header (The CRC32)
      myDllEndOfMessagePtr = (DllEndOfMessage_t*)(MessagePtr + MessageLength - sizeof(myDllEndOfMessagePtr));
      
      // Fill the Dll message
      myDllHeaderPtr->Sync0 = UARTDLL_SYNC_WORD_MSB;
      myDllHeaderPtr->Sync1 = UARTDLL_SYNC_WORD_LSB;
      // The payload length excluding the DLL header
      myDllHeaderPtr->PayloadLength = HTONS(PayloadLength);
      // Accumulate CRC32 for the first cunck including the init part.
      uartdllAccumulatedTxCrc32 =  crc32BuffAccumulate(crc32Init(), MessagePtr, sizeof(DllHeader_t), TxChunkSize);
      break;
  case UARTDLL_TX_MIDDLE_CHUNK:
          uartdllAccumulatedTxCrc32 =  crc32BuffAccumulate(uartdllAccumulatedTxCrc32, MessagePtr, sizeof(DllHeader_t), TxChunkSize);
    break;
  case UARTDLL_TX_LAST_CHUNK:
      myDllEndOfMessagePtr->CRC32 = HTONL(uartdllAccumulatedTxCrc32);   
    break;
  }
  
  return(RETURNCODE_ERROR);
}


