#include <hwdrivers.h>
#include "vlapConfig.h"
#include "ble.h"
#include "timers.h"
#include "comm.h"
#include "pccommAppLayer.h"
#include "fwupgrade.h"



// L O C A L    D E F I N I T I O N S
#define BLE_CHECKSUM_INITIAL_VALUE      0x5a
#define BLE_PROGRAM_DATA_WAIT_TIMEOUT   60000

// L O C A L    P R O T O T Y P E S
ReturnCode_T bleRxDllFsmTimerSchedule(uint32_t time);
void bleRxDllFsmTimeoutHandler();
ReturnCode_T bleRxDllFsmTimerRemove();
portTASK_FUNCTION(bleTask, pvParameters );
ReturnCode_T commTaskEventSend(commFsmQueueEntryActionType_T NotificationToCommTask );
ReturnCode_T bleBsipDllMessageSend(uint8_t* MessagePtr, uint16_t MessageLength);
ReturnCode_T bleBsipApplicationLayerHandlePacket(uint8_t *MsgPtr, uint16_t Length);
ReturnCode_T bleBsipCheckSumCalc(uint8_t *MessagePtr, uint16_t PayloadLength, uint8_t *ReturnedCheckSumPtr );
ReturnCode_T bleTimeoutHandler();
void bleProgramSetIdle();

static void bleProgramFsmTimerCallback(TimerHandle_t pxTimer);

// M O D U L E   G L O B A L S
bleRxFsmState_T bleBsipDllRxFsmState;
bleRxBootloaderFsmState_T bleBootloaderRxFsmState;
uint8_t bleDllRcvBuffer[100];
uint16_t bleDllRcvBuffIndex;
uint16_t bleDllMessageLengthTemp;
uint16_t bleDllMessageLength;
uint8_t BleCheckSum;
TimerHandle_t BleRxDllFsmTimer;
TimerHandle_t bleProgramFsmTimerHandler;
// bleTask
QueueHandle_t bleTaskQueueHandle;
bleTaskFsmState_T bleTaskFsmState;
bleTaskFsmStimuli_T Stimuli;
uint16_t BleTaskMsecDelay;

volatile uint32_t bleCheckSumErrorCounter;

uint16_t bleVersionNumber = 0;
BleBootState_T bleBootState = BLE_BOOT_DISABLE;
bleProgramFsmState_T bleProgramState = BLE_PROGRAM_FSM_IDLE;
void(*bleProgramRegisteredCallback)(char* receivedData, uint8_t receivedlength);
uint8_t bleBootloaderSendBuffer[257];
uint8_t* bleProgramData;
uint8_t bleBootloaderExpectedLength;
uint8_t bleProgramPrintBuffer[100]; 
uint8_t bleProgramReceivedOpcode = 0;
uint8_t BleConnectedState;
/******************************************************************************
* @brief  ReturnCode_T bleControl( BleControl_T BleControl)
* @param  
* @retval 
******************************************************************************/
ReturnCode_T bleControl( BleControl_T BleControl)
{
  ReturnCode_T MyReturnCode = RETURNCODE_OK; 
  
  switch(BleControl)
  {
  case BLE_CONTROL_DISABLE:
    // If the BLE chip currently at boot state - it cannot be turned off
    if(bleBootState == BLE_BOOT_ENABLE)
      return MyReturnCode;
    // Enable the BLE power supply
    hwdriversGpioBitWrite(HWDRIVERS_BLE_POWER_ENABLE, 0);
    // De assert the BLE reset
    hwdriversGpioBitWrite(HWDRIVERS_BLE_RESET, 0 );
    hwdriversBleUartTxPinControl(HWDRIVERS_UART3_TX_PIN_GPIO_ZERO);
    break;
  case BLE_CONTROL_ENABLE:
    hwdriversBleUartTxPinControl(HWDRIVERS_UART3_TX_PIN_UART_MODE);
    // Enable the BLE power supply
    hwdriversGpioBitWrite(HWDRIVERS_BLE_POWER_ENABLE, 1);
    // De assert the BLE reset
    hwdriversGpioBitWrite(HWDRIVERS_BLE_RESET, 1);
    //vTaskDelay(50);
    // ReInit BLE Rx DMA - as the reset stopped the DMA
    //hwdriversReInitUartBleRxDma();
//    bleBsipDllRxFsmIdleSet();
    break;
  default:
    MyReturnCode = RETURNCODE_UNSUPPORTED;
  }
  
  return(MyReturnCode);
}



/******************************************************************************
* @brief  void bleInit()
* @param  
* @retval 
******************************************************************************/
ReturnCode_T bleInit(void)
{
  // By default the ble module is powered off
  bleControl(BLE_CONTROL_ENABLE);
  // Default state after power up
  bleBsipDllRxFsmState = BLE_BSIP_RX_DLL_FSM_STATE_IDLE; 
  // Create bleTask input queue  
  bleTaskQueueHandle = xQueueCreate(3, sizeof(bleTaskFsmQueueEntryActionType_T));
  // Init the FSM state
  bleTaskFsmState = BLE_TASK_FSM_STATE_IDLE;
  // Create the task
  xTaskCreate(bleTask, "bleTask", 1000, NULL, 1, ( TaskHandle_t * ) NULL );
  // Set initiail queue wait timeout
  BleTaskMsecDelay  = 10;
  //hwdriversGpioBitWrite(HWDRIVERS_BLE_BOOT, 0);
  bleSetBootState(BLE_BOOT_DISABLE);
    
  bleCheckSumErrorCounter = 0;
  
  bleVersionNumber = 0;
  bleBootState = BLE_BOOT_DISABLE;
  BleConnectedState = 0;

  hwdriversReInitUartBleRxDma();

  bleProgramFsmTimerHandler =  xTimerCreate("bleBootloaderTimer", portTICK_PERIOD_MS, pdFALSE, (void *)0, bleProgramFsmTimerCallback);
  BleRxDllFsmTimer = xTimerCreate("blerXdLL", portTICK_PERIOD_MS, pdFALSE, (void *)0, bleRxDllFsmTimeoutHandler);
  // Return success
  return(RETURNCODE_OK);
}

uint32_t bleProcessorVersionGet()
{
  return bleVersionNumber;
}

uint8_t *MsgPtrTemp;

/******************************************************************************
*** ReturnCode_T bleBsipApplicationLayerHandlePacket(uint8_t *MsgPtr, uint16_t Length)
* @brief  BSIP Req from BLE clinet via the BLE Soc  
* @param  
* @retval 
******************************************************************************/
ReturnCode_T bleBsipApplicationLayerHandlePacket(uint8_t *MsgPtr, uint16_t Length)
{

  MsgPtrTemp = MsgPtr;
  switch( ((bleAdvertisingControl_t*)(MsgPtr + sizeof(bleBsipDllHeader_t)))->PayloadApplicationLayerOperationCode)
  {
  case BLE_BSIP_OPCODE_READ_REQ:
    if(BleConnectedState){
	  // Payload Read request from the client, In BLE connected state, assuming 20 bytes are requested to be sent from the log memory to the BLE clinet, The call
	  commTaskEventSend(COMM_FSM_QUEUE_ENTRY_TYPE_BLE_READ_REQ);
    }
    break;
  case BLE_BSIP_OPCODE_WRITE_REQ:
    if(BleConnectedState)
    {
	  if(Length > 2)
		{
		  // Allocate memory for the incoming buffer
		  uint8_t * MyPtr = pvPortMalloc(Length);
		  if(MyPtr)
		  {
			// Copy the buffer to the allocated buff pointer
			memcpy(MyPtr, (MsgPtr+sizeof(bleBsipDllHeader_t)+sizeof(bleWriteReq_t)), Length-1);

			// Send the buffer to the uartDllRxFsm
			uartdllTaskEventSend(UARTDLL_FSM_STIMULI_NEW_TOKEN_RECVD, MyPtr, Length-1);
		  }
		}
    }
    break;
  case BLE_BSIP_OPCODE_BLE_STATUS_CHANGE:
    switch(((bleStatusChange_t*)(MsgPtr+sizeof(bleBsipDllHeader_t)))->UpdatedStatus)
    {
    case 0:
      commTaskEventSend(COMM_FSM_QUEUE_ENTRY_TYPE_BLE_DISCONNECTED);
      BleConnectedState = 0;
      break;
    case 1:
      commTaskEventSend(COMM_FSM_QUEUE_ENTRY_TYPE_BLE_CONNECTED);
      BleConnectedState = 1;
      break;
    default:
      break;
    }
    break;
  case BLE_BSIP_OPCODE_PROCESSOR_STATUS:
    //bleBsipProcessorStatus_t
    bleVersionNumber = ((bleBsipProcessorStatus_t*)(MsgPtr+sizeof(bleBsipDllHeader_t)))->VersionNumber;
    pccpmmAppLayerStruct.BoardSystemRegisters.BleFirmwareVersion = TYPES_ENDIAN16_CHANGE(((bleBsipProcessorStatus_t*)(MsgPtr+sizeof(bleBsipDllHeader_t)))->VersionNumber);
    break;
  default:
    break;
  }


  return(RETURNCODE_OK); 
}



/******************************************************************************
* @brief  ReturnCode_T bleRxFsm(bleDllFsmStimuli_T stimuli, char inChar)
* @param  
* @retval 
******************************************************************************/
ReturnCode_T bleRxFsm(bleDllFsmStimuli_T stimuli, char inChar)
{
  // Distinguish between the two parsing types when the BLE chip in BOOT mode or not
  switch(bleBootState)
  {
  case BLE_BOOT_DISABLE:
    return bleBsipDllRxFsm(stimuli, inChar);
    break;
  case BLE_BOOT_ENABLE:
    return bleBootloaderRxFsm(stimuli, inChar);
    break;
  }
  return(RETURNCODE_OUT_OF_RANGE); 
}

/******************************************************************************
* @brief  ReturnCode_T bleBootloaderRxFsm(bleDllFsmStimuli_T stimuli, char  inChar)
* @param  
* @retval 
******************************************************************************/
ReturnCode_T bleBootloaderRxFsm(bleDllFsmStimuli_T stimuli, char  inChar)
{
  // Any incoming char removes pending timer
//  if(stimuli==BLE_RX_DLL_FSM_STIMULI_INCOMMING_CHAR)
//  {
//    // New received char removes the pending timer guarding the current state
//    bleRxDllFsmTimerRemove();
//  }
  
  switch(bleBootloaderRxFsmState)
  {
    // I D L E
  case  BLE_BOOTLOADER_RX_FSM_STATE_IDLE:
    if(stimuli==BLE_RX_DLL_FSM_STIMULI_INCOMMING_CHAR)
    {
      if(inChar==BLE_BOOTLOADER_ACK)
      {
        bleProgramFsm(BLE_PROGRAM_STIMULI_RECEIVED_ACK, (char*)inChar, 1, 0);
      }
      else if(inChar==BLE_BOOTLOADER_NACK)
      {
        bleProgramFsm(BLE_PROGRAM_STIMULI_RECEIVED_NACK, (char*)inChar, 1, 0);
      }
    }
    // For now, ignore all other cases - we will receive only ACK or NACK
    break;   
    // WAIT Length  
  case  BLE_BOOTLOADER_RX_FSM_STATE_MSG_LENGTH_WAIT:
    switch(stimuli)
    {
    case BLE_RX_DLL_FSM_STIMULI_INCOMMING_CHAR:
      // Store the char into the rx buffer
      bleDllRcvBuffer[bleDllRcvBuffIndex++]=inChar;
      // The received message length LSByte is placed in the LSByte of the length uint16
      bleDllMessageLengthTemp+=inChar;
      bleDllMessageLength = bleDllMessageLengthTemp;
      // Check the received message length: length 0 or length higher than buffer size will force FSM to Idle.
      if(!bleDllMessageLength || (bleDllMessageLengthTemp >= sizeof(bleDllRcvBuffer)) )
        bleBsipDllRxFsmState=BLE_BSIP_RX_DLL_FSM_STATE_IDLE;
      else
      {
        // Start inter char timeout timer
        bleRxDllFsmTimerSchedule(10);
        // Set the initials accumulated CRC32 value for this session
        BleCheckSum = BLE_CHECKSUM_INITIAL_VALUE;
        // Go wait for payload
        bleBsipDllRxFsmState=BLE_BSIP_RX_DLL_FSM_STATE_MSG_PAYLOAD_RCV_WAIT;
      }
      break;
    case BLE_RX_DLL_FSM_STIMULI_TIMOUT:
      bleTimeoutHandler();
      break;
    }
    break;
    
    // WAIT PAYLOAD  
  case  BLE_BOOTLOADER_RX_FSM_STATE_MSG_PAYLOAD_RCV_WAIT:
    switch(stimuli)
    {
    case BLE_RX_DLL_FSM_STIMULI_INCOMMING_CHAR:
      // Accumulate CRC32 based on the last known value
      BleCheckSum += inChar;
      // Start inter char timeout timer
      bleRxDllFsmTimerSchedule(10);
      // Store the char into the rx buffer
      bleDllRcvBuffer[bleDllRcvBuffIndex++]=inChar;
      // Decrement message length
      if (bleDllMessageLengthTemp)
        bleDllMessageLengthTemp--;
      // Is it the last byte of this message
      if(!bleDllMessageLengthTemp)
      {
        // Go wait for checksum
        bleBsipDllRxFsmState=BLE_BSIP_RX_DLL_FSM_STATE_MSG_CHECKSUM_WAIT;
      }
      break;
    case BLE_RX_DLL_FSM_STIMULI_TIMOUT:
      bleTimeoutHandler();
      break;
    }
    break;    
  }
  return(RETURNCODE_OK);
}



void bleBsipDllRxFsmIdleSet()
{
	  bleBsipDllRxFsmState = BLE_BSIP_RX_DLL_FSM_STATE_IDLE;
}


/******************************************************************************
* @brief  ReturnCode_T  bleBsipDllRxFsm(bleDllFsmStimuli_T stimuli, char  inChar)
* @param  
* @retval 
******************************************************************************/
ReturnCode_T  bleBsipDllRxFsm(bleDllFsmStimuli_T stimuli, char  inChar)
{
  
  // Any incoming char removes pending timer
  if(stimuli==BLE_RX_DLL_FSM_STIMULI_INCOMMING_CHAR)
  {
    // New received char removes the pending timer guarding the current state
    bleRxDllFsmTimerRemove();
  }
  
  
  switch(bleBsipDllRxFsmState)
  {
    // I D L E
  case  BLE_BSIP_RX_DLL_FSM_STATE_IDLE:
    if(stimuli==BLE_RX_DLL_FSM_STIMULI_INCOMMING_CHAR)
      if(inChar==BLE_BSIP_SYNC_WORD)
      {
        // BLE_SYNC_WORD_MSB found, message probably started
        // Start inter char timeout timer
        // Store the char into the rx buffer
        bleDllRcvBuffIndex = 0;
        bleDllRcvBuffer[bleDllRcvBuffIndex++]=inChar;
        bleRxDllFsmTimerSchedule(10);
        // Go wait for Sync LSB
        bleBsipDllRxFsmState=BLE_BSIP_RX_DLL_FSM_STATE_MSG_LENGTH_MSB_WAIT;
      }
    break;
    
    // WAIT Length MSB  
  case  BLE_BSIP_RX_DLL_FSM_STATE_MSG_LENGTH_MSB_WAIT:
    switch(stimuli)
    {
    case BLE_RX_DLL_FSM_STIMULI_INCOMMING_CHAR:
      // Store the char into the rx buffer
      bleDllRcvBuffer[bleDllRcvBuffIndex++]=inChar;
      // The received message length MSByte is placed in the MSByte of the length uint16
      bleDllMessageLengthTemp = (((uint16_t)inChar)<<8);
      // Store the char into the rx buffer
      bleDllRcvBuffer[bleDllRcvBuffIndex]=inChar;
      // Start inter char timeout timer
      bleRxDllFsmTimerSchedule(10);
      // Go wait for message the length
      bleBsipDllRxFsmState=BLE_BSIP_RX_DLL_FSM_STATE_MSG_LENGTH_LSB_WAIT;
      break;
    case BLE_RX_DLL_FSM_STIMULI_TIMOUT:
      bleTimeoutHandler();
      break;
    }
    break;
    
    // WAIT Length LSB  
  case  BLE_BSIP_RX_DLL_FSM_STATE_MSG_LENGTH_LSB_WAIT:
    switch(stimuli)
    {
    case BLE_RX_DLL_FSM_STIMULI_INCOMMING_CHAR:
      // Store the char into the rx buffer
      bleDllRcvBuffer[bleDllRcvBuffIndex++]=inChar;
      // The received message length LSByte is placed in the LSByte of the length uint16
      bleDllMessageLengthTemp+=inChar;
      bleDllMessageLength = bleDllMessageLengthTemp;
      // Check the received message length: length 0 or length higher than buffer size will force FSM to Idle.
      if(!bleDllMessageLength || (bleDllMessageLengthTemp >= sizeof(bleDllRcvBuffer)) )
        bleBsipDllRxFsmState=BLE_BSIP_RX_DLL_FSM_STATE_IDLE;
      else
      {
        // Start inter char timeout timer
        bleRxDllFsmTimerSchedule(10);
        // Set the initials accumulated CRC32 vlaue for this session
        BleCheckSum = BLE_CHECKSUM_INITIAL_VALUE;
        // Go wait for payload
        bleBsipDllRxFsmState=BLE_BSIP_RX_DLL_FSM_STATE_MSG_PAYLOAD_RCV_WAIT;
      }
      break;
    case BLE_RX_DLL_FSM_STIMULI_TIMOUT:
      bleTimeoutHandler();
      break;
    }
    break;
    
    // WAIT PAYLOAD  
  case  BLE_BSIP_RX_DLL_FSM_STATE_MSG_PAYLOAD_RCV_WAIT:
    switch(stimuli)
    {
    case BLE_RX_DLL_FSM_STIMULI_INCOMMING_CHAR:
      // Accumulate CRC32 based on the last known value
      BleCheckSum += inChar;
      // Start inter char timeout timer
      bleRxDllFsmTimerSchedule(10);
      // Store the char into the rx buffer
      bleDllRcvBuffer[bleDllRcvBuffIndex++]=inChar;
      // Decrement message length
      if (bleDllMessageLengthTemp)
        bleDllMessageLengthTemp--;
      // Is it the last byte of this message
      if(!bleDllMessageLengthTemp)
      {
        // Go wait for checksum
        bleBsipDllRxFsmState=BLE_BSIP_RX_DLL_FSM_STATE_MSG_CHECKSUM_WAIT;
      }
      break;
    case BLE_RX_DLL_FSM_STIMULI_TIMOUT:
      bleTimeoutHandler();
      break;
    }
    break;
    
    // WAIT for Check Sum
  case  BLE_BSIP_RX_DLL_FSM_STATE_MSG_CHECKSUM_WAIT:
    switch(stimuli)
    {
    case BLE_RX_DLL_FSM_STIMULI_INCOMMING_CHAR:
      // does the CheckSum match, if so, call the application layer
      if(BleCheckSum == inChar)
      {
        // Call the application layer
        bleBsipApplicationLayerHandlePacket(bleDllRcvBuffer, bleDllMessageLength);
      }
      else
      {
        // Update reception statistics, Errored packets
        bleCheckSumErrorCounter++;
      }       
      bleBsipDllRxFsmState=BLE_BSIP_RX_DLL_FSM_STATE_IDLE;
      break;
    case BLE_RX_DLL_FSM_STIMULI_TIMOUT:
      bleTimeoutHandler();
      break;
    }
    break;
    
  }
  return(RETURNCODE_OK);
}



/***********************************************************************************
*** void bleFsmTimerCallback(void)
*
*
*
*
***********************************************************************************/
void bleDllRxFsmTimerCallback()
{
 // uartDllRxFsm(UARTDLL_UART_6_RS485,UARTDLL_FSM_STIMULI_TIMOUT, 0);
}


/***********************************************************************************
*** ReturnCode_T bleRxDllFsmTimerSchedule(uint32_t time)
*
*
*
*
***********************************************************************************/
ReturnCode_T bleRxDllFsmTimerSchedule(uint32_t time)
{
  
  if(xTimerIsTimerActive(BleRxDllFsmTimer))
  {
    // Assuming the timer is already stopped, Change the timer period
    xTimerChangePeriod(BleRxDllFsmTimer, pdMS_TO_TICKS( time ), 100 ); 
    // Start the timer
    xTimerStart(BleRxDllFsmTimer, 100);  
  }
  return(RETURNCODE_OK);
}


/***********************************************************************************
*** ReturnCode_T bleRxDllFsmTimerRemove()
*
*
*
*
***********************************************************************************/
ReturnCode_T bleRxDllFsmTimerRemove()
{
  
  // Stop the timer
  xTimerStop(BleRxDllFsmTimer, 100);  
  
  return(RETURNCODE_OK);
}


/***********************************************************************************
*** void bleRxDllFsmTimeoutHandler()
*
*
*
*
***********************************************************************************/
void bleRxDllFsmTimeoutHandler()
{
  // Inter char timeout, message reception aborted, go to idle
  // Update UARTDLL reception statistics
//  bleStatisticsCounterUpdate(UARTDLL_RCV_STATSTICS_INTER_CHAR_TIMEOUT_COUNTER);
  // go wait for next message
  bleBsipDllRxFsmState=BLE_BSIP_RX_DLL_FSM_STATE_IDLE;
}



/*******************************************************************************
* Function Name  : portTASK_FUNCTION(bleflashTask, pvParameters )
* Description    : 
* Input          : 
*                  
*                  
* Output         : None
* Return         : None
*******************************************************************************/
portTASK_FUNCTION(bleTask, pvParameters )
{
  bleTaskFsmQueueEntry_T BleTaskQueueEntry;
  
  while(1)
  {
    // Wait for queue event
    BaseType_t  QueueState = xQueueReceive(bleTaskQueueHandle, &BleTaskQueueEntry, BleTaskMsecDelay);
    {
      // Convert the queue status to FSM Stimuli
      if(QueueState == pdTRUE)
        Stimuli = BLE_TASK_FSM_STIMULI_NEW_TOKEN_RECVD;
      else
        Stimuli = BLE_TASK_FSM_STIMULI_TIMOUT;  
      
      switch(bleTaskFsmState)
      {
      //==============================
      case BLE_TASK_FSM_STATE_IDLE:
        switch(Stimuli)
        {
        case BLE_TASK_FSM_STIMULI_NEW_TOKEN_RECVD:
          switch(BleTaskQueueEntry.ActionType)
          {
          case BLE_TASK_QUEUE_ENTRY_COMMAND_START:
              // Turn Off the BLE module 
              bleControl(BLE_CONTROL_DISABLE);
              vTaskDelay(1000);
              // Turn On the BLE module 
              bleControl(BLE_CONTROL_ENABLE);
              // Go wait for ON response
              BLE_TASK_FSM_STATE_CHANGE(BLE_TASK_FSM_STATE_ON_WAIT, 1000);
            break;
          case BLE_TASK_QUEUE_ENTRY_COMMAND_STOP:
              // Turn Off the BLE module 
              bleControl(BLE_CONTROL_DISABLE);
              // Send Disconnected event to the commTask
              commTaskEventSend(COMM_FSM_QUEUE_ENTRY_TYPE_BLE_TIMEOUT);
            break;
          default:
        	  break;

          }
          break;
        case BLE_TASK_FSM_STIMULI_TIMOUT:
          BLE_TASK_FSM_STATE_CHANGE(BLE_TASK_FSM_STATE_IDLE, 100);
          break;
        }
        break;

      //==============================
      case BLE_TASK_FSM_STATE_ON_WAIT:
        switch(Stimuli)
        {
        case BLE_TASK_FSM_STIMULI_NEW_TOKEN_RECVD:
          switch(BleTaskQueueEntry.ActionType)
          {
          case BLE_TASK_QUEUE_ENTRY_RESP_BSIP_BLE_ON_STATUS:
              // Send BLE configuration ( Service, Charectaristics, Properties, Advertisement)  
              // Go wait BLE Configuration  config Ack
              BLE_TASK_FSM_STATE_CHANGE(BLE_TASK_FSM_STATE_CONFIG_ACK_WAIT, 100);
            break;
          case BLE_TASK_QUEUE_ENTRY_COMMAND_STOP:
            break;
          }
          break;
        case BLE_TASK_FSM_STIMULI_TIMOUT:
          BLE_TASK_FSM_STATE_CHANGE(BLE_TASK_FSM_STATE_IDLE, 100);
          break;
        default:
        	break;
        }
        break;
      
      //==============================
      case BLE_TASK_FSM_STATE_CONFIG_ACK_WAIT:
        switch(Stimuli)
        {
        case BLE_TASK_FSM_STIMULI_NEW_TOKEN_RECVD:
          switch(Stimuli)
          {
          case BLE_TASK_FSM_STIMULI_NEW_TOKEN_RECVD:
            switch(BleTaskQueueEntry.ActionType)
            {
            case BLE_TASK_QUEUE_ENTRY_RESP_BSIP_CONFIG_ACK:
              // Command the BLE module to start advertising 
              // Go wait for ON response
              BLE_TASK_FSM_STATE_CHANGE(BLE_TASK_FSM_STATE_CONNECTION_WAIT, 10000);
              break;
            case BLE_TASK_QUEUE_ENTRY_COMMAND_STOP:
              break;
            default:
            	break;
            }
            break;
          case BLE_TASK_FSM_STIMULI_TIMOUT:
            BLE_TASK_FSM_STATE_CHANGE(BLE_TASK_FSM_STATE_IDLE, 100);
            break;
          }
          break;
        case BLE_TASK_FSM_STIMULI_TIMOUT:
          BLE_TASK_FSM_STATE_CHANGE(BLE_TASK_FSM_STATE_IDLE, 100);
          break;
        }
        break;
        
      //==============================
      case BLE_TASK_FSM_STATE_CONNECTION_WAIT:
        switch(Stimuli)
        {
        case BLE_TASK_FSM_STIMULI_NEW_TOKEN_RECVD:
          break;
        case BLE_TASK_FSM_STIMULI_TIMOUT:
          BLE_TASK_FSM_STATE_CHANGE(BLE_TASK_FSM_STATE_IDLE, 100);
          break;
        }
        break;

      //==============================
      case BLE_TASK_FSM_STATE_CONNECTED:
        switch(Stimuli)
        {
        case BLE_TASK_FSM_STIMULI_NEW_TOKEN_RECVD:
          break;
        case BLE_TASK_FSM_STIMULI_TIMOUT:
          BLE_TASK_FSM_STATE_CHANGE(BLE_TASK_FSM_STATE_IDLE, 100);
          break;
        }
        break;

      //==============================
      default:
        BleTaskMsecDelay = 35/portTICK_PERIOD_MS;
        break;
      }
    }
  }
  
}
  
  
  
  /**************************************************************************
  *** ReturnCode_T bleBsipDllMessageSend(uint8_t *MessagePtr, uint16_t MessageLength)
  * 
  *  We assume the pointed message buffer has the length of the complete message including the 
  *  DLL part (
  *
  ***************************************************************************/
  ReturnCode_T bleBsipDllMessageSend(uint8_t *MessagePtr, uint16_t MessageLength)
  {
    ReturnCode_T MyErrorReturn;
    bleBsipDllHeader_t* myDllHeaderPtr;
    bleBsipDllEndOfMessage_t* myDllEndOfMessagePtr;
    uint8_t BsipCheckSum;
 
    // The payload buffer length = The message length - Dll header and Dll End  of message
    uint16_t PayloadLength = MessageLength - sizeof(bleBsipDllHeader_t) - sizeof(bleBsipDllEndOfMessage_t);
    
    
    // Point to the DLL header 
    myDllHeaderPtr = (bleBsipDllHeader_t*)MessagePtr;
    // Point to the DLL postfix header (The CheckSum)
    myDllEndOfMessagePtr = (bleBsipDllEndOfMessage_t*)(MessagePtr + MessageLength - sizeof(bleBsipDllEndOfMessage_t));
    
    // Fill the Dll message
    myDllHeaderPtr->Sync = BLE_BSIP_SYNC_WORD;
    // The payload length excluding the DLL header (From the byte after the mength excluding the checkSum byte)
    myDllHeaderPtr->PayloadLengthMsb = (uint8_t)(PayloadLength>>8);
    myDllHeaderPtr->PayloadLengthLsb = (uint8_t) PayloadLength;
    // Clear the CheckSum
    BsipCheckSum = 0;
    // Accumulate checkSum for this block, excluding the header length
    if( bleBsipCheckSumCalc(MessagePtr + sizeof(bleBsipDllHeader_t), PayloadLength, &BsipCheckSum) == RETURNCODE_OK)
    {
      myDllEndOfMessagePtr->CheckSum = BsipCheckSum; 
      // Enqueue the application message to the Tx queue, 
      if(uartdllTxQueueEnqueue(UARTDLL_UART_3_BLE, MessagePtr, MessageLength, true) == RETURNCODE_OK)
        MyErrorReturn = RETURNCODE_OK;
      else
      {
        // Failed, Queue is full
        MyErrorReturn = RETURNCODE_TX_QUEUE_FULL;
      }
    }
    else
    {
      MyErrorReturn = RETURNCODE_ERROR;
    }
    
    return(MyErrorReturn);
  }
  
  
 /**************************************************************************
  *** ReturnCode_T bleBsipCheckSumCalc(uint8_t *MessagePtr, uint16_t PayloadLength, uint8_t *ReturnedCheckSumPtr )
  * 
  *  
  *
  ***************************************************************************/
ReturnCode_T bleBsipCheckSumCalc(uint8_t *MessagePtr, uint16_t PayloadLength, uint8_t *ReturnedCheckSumPtr )
{
  uint8_t MyCheckSum = BLE_CHECKSUM_INITIAL_VALUE;
  uint16_t i;
  
  for(i=0; i<PayloadLength; i++)
     MyCheckSum += *(MessagePtr+i);
  
  *ReturnedCheckSumPtr  = MyCheckSum;
  
  return(RETURNCODE_OK);
}
  

 /**************************************************************************
  *** ReturnCode_T bleTimeoutHandler()
  * 
  *  
  *
  ***************************************************************************/
ReturnCode_T bleTimeoutHandler()
{
  return(RETURNCODE_OK);
}


/******************************************************************************
*** ReturnCode_T bleTaskEventSend(bleTaskFsmQueueEntryActionType_T NotificationToBleTask )
* @brief  // Sends notifications to the bleTask   
* @param  
* @retval 
******************************************************************************/
ReturnCode_T bleTaskEventSend(bleTaskFsmQueueEntryActionType_T NotificationToBleTask )
{
  bleTaskFsmQueueEntry_T  BleTaskFsmQueueEntry;
  
  // Fill the queue entry
  BleTaskFsmQueueEntry.ActionType = NotificationToBleTask;
  BleTaskFsmQueueEntry.DataPtr    = 0;
  BleTaskFsmQueueEntry.DataLength = 0;
 
  // Enqueue the event to the bleTask input queue
  xQueueSend(bleTaskQueueHandle, &BleTaskFsmQueueEntry, 0);

  return(RETURNCODE_OK);
}

/******************************************************************************
*** ReturnCode_T bleSetBootState(BleBootState_T state)
* @brief  Sets the boot state of the BLE chip  
* @param  
* @retval 
******************************************************************************/
ReturnCode_T bleSetBootState(BleBootState_T state)
{
  switch(state)
  {
  case BLE_BOOT_DISABLE:
    hwdriversGpioBitWrite(HWDRIVERS_BLE_BOOT, 0);
    bleBootState = state;
    break;
  case BLE_BOOT_ENABLE:
    bleControl(BLE_CONTROL_DISABLE);
    // Set the bleBootState now to mark that the device already cannot use the 
    // BLE chip anymore
    bleBootState = state;
    // Let the BLE chip to discharge
    vTaskDelay(200);
    hwdriversGpioBitWrite(HWDRIVERS_BLE_BOOT, 1);
    // If we want to set the boot state to high - it means that we want to program
    // it. Make sure that the BLE chip is always on and mark to comm task that it 
    // cannot use the BLE chip right now
    bleControl(BLE_CONTROL_ENABLE);
    // Let the BLE chip to turn on
    vTaskDelay(200);
    break;
  default:
    break;
  }

  return(RETURNCODE_OK);
}

/******************************************************************************
*** BleBootState_T bleGetBootState()
* @brief  Returns the BLE boot state
* @param  
* @retval 
******************************************************************************/
BleBootState_T bleGetBootState()
{
  return bleBootState;
}

/******************************************************************************
*** static void bleProgramFsmTimerCallback(TimerHandle_t pxTimer)
* @brief  Ble program FSM timeout handler
* @param  
* @retval 
******************************************************************************/
static void bleProgramFsmTimerCallback(TimerHandle_t pxTimer)
{
  if(bleProgramState != BLE_PROGRAM_FSM_IDLE)
  {
    sprintf((char *)bleProgramPrintBuffer, "BLE program timeout, State: %d", bleProgramState);
    vlapmainDebugLog((char *)bleProgramPrintBuffer);
    bleProgramFsm(BLE_PROGRAM_STIMULI_TIMEOUT, 0, 0, 0);
  }
}

/******************************************************************************
*** void bleProgramSetIdle()
* @brief  Sets the Ble program FSM back to idle
* @param  
* @retval 
******************************************************************************/
void bleProgramSetIdle()
{
  bleProgramState = BLE_PROGRAM_FSM_IDLE;
  // Disable the boot mode
  bleSetBootState(BLE_BOOT_DISABLE);
  // Stop the timeout timer
  xTimerStop(bleProgramFsmTimerHandler, 0);
}

/******************************************************************************
*** void ReturnCode_T bleProgramFsm(bleProgramFsmStimuli_T stimuli, char* data, uint16_t length, void(*ReceivedCallBackPtr)(char* receivedData, uint8_t receivedlength))
* @brief  Ble program FSM
* @param  
* @retval 
******************************************************************************/
ReturnCode_T bleProgramFsm(bleProgramFsmStimuli_T stimuli, char* data, uint16_t length, void(*ReceivedCallBackPtr)(char* receivedData, uint8_t receivedlength))
{
  ReturnCode_T MyErrorReturn = RETURNCODE_ERROR;
  uint8_t bleProgramDataLength;

  switch(bleProgramState)
  {
  case BLE_PROGRAM_FSM_IDLE:
    switch(stimuli)
    {
    case BLE_PROGRAM_STIMULI_RECEIVED_DATA:
    case BLE_PROGRAM_STIMULI_TIMEOUT:
    case BLE_PROGRAM_STIMULI_RECEIVED_ACK:
    case BLE_PROGRAM_STIMULI_RECEIVED_NACK:
      break;
    case BLE_PROGRAM_STIMULI_SEND_DATA:
      bleProgramReceivedOpcode = (uint8_t) data[0];
      // Check if received sync request is valid - if it is not, drop the request
      if((bleProgramReceivedOpcode != BLE_BOOTLOADER_SYNC_WORD) || (length != 1))
      {
        return RETURNCODE_UNSUPPORTED;
      }
      // Stop the FW upgrade in case that it is currently downloading
      fwupgradeEventSend(FWUPGRADE_OPCODE_STOP_DOWNLOAD, 0, 0);
      // Send abort message to the commTask to abort the current communication session
      commTaskEventSend(COMM_FSM_QUEUE_ENTRY_TYPE_COMM_ABORT);
      // Enable boot state 
      bleSetBootState(BLE_BOOT_ENABLE);
      // Send waited byte to the BLE chip bootloader for auto - baud rate initialization
      // and wait for acknowledgment
      // Save registered callback
      bleProgramRegisteredCallback = ReceivedCallBackPtr;
      // Enqueue the application message to the Tx queue, 
      if(uartdllTxQueueEnqueue(UARTDLL_UART_3_BLE, (uint8_t *)data, length, false) == RETURNCODE_OK)
      {
        MyErrorReturn = RETURNCODE_OK;
        BLE_PROGRAM_FSM_STATE_CHANGE(BLE_PROGRAM_FSM_INIT_ACK_WAIT, 1000);
      }
      else
      {
        // Failed, Queue is full
        MyErrorReturn = RETURNCODE_TX_QUEUE_FULL;
      }
      break;
    default:
      break;
    }
    break;
  case BLE_PROGRAM_FSM_INIT_ACK_WAIT:
    switch(stimuli)
    {
    case BLE_PROGRAM_STIMULI_RECEIVED_DATA:
    case BLE_PROGRAM_STIMULI_SEND_DATA:
      break;
    case BLE_PROGRAM_STIMULI_RECEIVED_ACK:
      sprintf((char*)bleProgramPrintBuffer, "BLE bootloader baud rate synced, ready for usage");
      vlapmainDebugLog((char*)bleProgramPrintBuffer);
      if(bleProgramRegisteredCallback)
        // Notify that ACK was received
        bleProgramRegisteredCallback(data, length);
      // Recevied ACK - baud rate synced, we can start using the bootloader now
      // Set timeout for 1 minute, if no data received to send at this time - abort the operation
      BLE_PROGRAM_FSM_STATE_CHANGE(BLE_PROGRAM_FSM_READY, 60000);
      break;
    case BLE_PROGRAM_STIMULI_RECEIVED_NACK:
      sprintf((char*)bleProgramPrintBuffer, "BLE bootloader baud rate synced failed - received NACK");
      vlapmainDebugLog((char*)bleProgramPrintBuffer);
      if(bleProgramRegisteredCallback)
        // Notify that NACK was received
        bleProgramRegisteredCallback(data, length);
      // Don't break - we still want to go to idle
    case BLE_PROGRAM_STIMULI_TIMEOUT:
      // Received NACK - go back to idle
      bleProgramSetIdle();
      break;
    default:
      break;
    }
    break;
  case BLE_PROGRAM_FSM_READY:
    switch(stimuli)
    {
    case BLE_PROGRAM_STIMULI_RECEIVED_DATA:
    case BLE_PROGRAM_STIMULI_RECEIVED_ACK:
    case BLE_PROGRAM_STIMULI_RECEIVED_NACK:
      break;
    case BLE_PROGRAM_STIMULI_SEND_DATA:
      bleProgramRegisteredCallback = ReceivedCallBackPtr;
      // If size is smaller then 3 (smallest command) mark that this request isn't valid
      if(length < 3)
      {
        if(bleProgramRegisteredCallback)
          // Notify that there was a problem
          bleProgramRegisteredCallback(0, 0);
        // Retriger self state
        BLE_PROGRAM_FSM_STATE_CHANGE(BLE_PROGRAM_FSM_READY, BLE_PROGRAM_DATA_WAIT_TIMEOUT);
        break;
      }
      // Save the pointer to the data received
      bleProgramData = (uint8_t*)data;
      // First byte is the opcode
      bleProgramReceivedOpcode = (uint8_t) data[0];
      // Send opcode (2 bytes) for the write command and wait for ACK
      if(uartdllTxQueueEnqueue(UARTDLL_UART_3_BLE, (uint8_t *)bleProgramData, 2, false) == RETURNCODE_OK)
      {
        MyErrorReturn = RETURNCODE_OK;
        // Go and wait for ACK for the sent data
        BLE_PROGRAM_FSM_STATE_CHANGE(BLE_PROGRAM_FSM_WRITE_OPCODE_ACK_WAIT, 1000);
      }
      else
      {
        if(bleProgramRegisteredCallback)
          // Notify that there was a problem
          bleProgramRegisteredCallback(0, 0);
        // Retriger self state
        BLE_PROGRAM_FSM_STATE_CHANGE(BLE_PROGRAM_FSM_READY, BLE_PROGRAM_DATA_WAIT_TIMEOUT);
        // Failed, Queue is full
        MyErrorReturn = RETURNCODE_TX_QUEUE_FULL;
        break;
      }
      break;
    case BLE_PROGRAM_STIMULI_TIMEOUT:
      // Process timeout
      bleProgramSetIdle();
      break;
    default:
      break;
    }
    break;
  case BLE_PROGRAM_FSM_WRITE_OPCODE_ACK_WAIT:
    switch(stimuli)
    {
    case BLE_PROGRAM_STIMULI_RECEIVED_DATA:
      break;
    case BLE_PROGRAM_STIMULI_SEND_DATA:
      sprintf((char*)bleProgramPrintBuffer, "BLE program still waiting for ACK, cannot receive data right now");
      vlapmainDebugLog((char*)bleProgramPrintBuffer);
      break;
    case BLE_PROGRAM_STIMULI_RECEIVED_ACK:
      switch(bleProgramReceivedOpcode)
      {
      case BLE_ERASE_OPCODE:
        // For now, this application only supports mass erase
        // Send address (4 bytes) and checksum byte for the write command and wait for ACK
        MyErrorReturn = uartdllTxQueueEnqueue(UARTDLL_UART_3_BLE, (uint8_t *)bleProgramData + 2, 2, false);
        break;
      case BLE_WRITE_OPCODE:
      case BLE_GO_OPCODE:
        // Send address (4 bytes) and checksum byte for the write command and wait for ACK
        MyErrorReturn = uartdllTxQueueEnqueue(UARTDLL_UART_3_BLE, (uint8_t *)bleProgramData + 2, 5, false);
        break;
      default:
        break;
      }
      if(MyErrorReturn == RETURNCODE_OK)
      {
        // Go and wait for ACK for the sent data
        BLE_PROGRAM_FSM_STATE_CHANGE(BLE_PROGRAM_FSM_WRITE_ADDRESS_ACK_WAIT, 1000);
      }
      else
      {
        if(bleProgramRegisteredCallback)
          // Notify that there was a problem
          bleProgramRegisteredCallback(0, 0);
        // Retriger self state
        BLE_PROGRAM_FSM_STATE_CHANGE(BLE_PROGRAM_FSM_READY, BLE_PROGRAM_DATA_WAIT_TIMEOUT);
        // Failed, Queue is full
        MyErrorReturn = RETURNCODE_TX_QUEUE_FULL;
        break;
      }
      break;
    case BLE_PROGRAM_STIMULI_RECEIVED_NACK:
      sprintf((char*)bleProgramPrintBuffer, "BLE program NACK received for sent data, failed! wait for next data package");
      vlapmainDebugLog((char*)bleProgramPrintBuffer);
      // Go and wait for the next data package at ready mode, set timeout for a minute
      BLE_PROGRAM_FSM_STATE_CHANGE(BLE_PROGRAM_FSM_READY, BLE_PROGRAM_DATA_WAIT_TIMEOUT);
      break;
    case BLE_PROGRAM_STIMULI_TIMEOUT:
      // Process timeout
      bleProgramSetIdle();
      break;
    default:
      break;
    }
    break;
  case BLE_PROGRAM_FSM_WRITE_ADDRESS_ACK_WAIT:
    switch(stimuli)
    {
    case BLE_PROGRAM_STIMULI_RECEIVED_DATA:
      break;
    case BLE_PROGRAM_STIMULI_SEND_DATA:
      sprintf((char*)bleProgramPrintBuffer, "BLE program still waiting for ACK, cannot receive data right now");
      vlapmainDebugLog((char*)bleProgramPrintBuffer);
      break;
    case BLE_PROGRAM_STIMULI_RECEIVED_ACK:
      switch(bleProgramReceivedOpcode)
      {
      case BLE_ERASE_OPCODE:
        if(bleProgramRegisteredCallback)
          // Notify that ACK was received
          bleProgramRegisteredCallback(data, length);
        sprintf((char*)bleProgramPrintBuffer, "BLE mass erase ACK received");
        vlapmainDebugLog((char*)bleProgramPrintBuffer);
        // Go and wait for the next data package at ready mode, set timeout for a minute
        BLE_PROGRAM_FSM_STATE_CHANGE(BLE_PROGRAM_FSM_READY, BLE_PROGRAM_DATA_WAIT_TIMEOUT);
        break;
      case BLE_WRITE_OPCODE:
        // Get size byte (size couldn't be bigger then 256 bytes due to bootlaoder protocol)
        bleProgramDataLength = (uint8_t) (bleProgramData[7]);
        // Skip the first 7 bytes (opcode + address bytes + xor byte) and add one to size (xor of data) 
        if(uartdllTxQueueEnqueue(UARTDLL_UART_3_BLE, (uint8_t *)bleProgramData + 7, bleProgramDataLength + 3, false) == RETURNCODE_OK)
        {
          MyErrorReturn = RETURNCODE_OK;
          // Go and wait for ACK for the sent data
          BLE_PROGRAM_FSM_STATE_CHANGE(BLE_PROGRAM_FSM_WRITE_DATA_ACK_WAIT, 1000);
        }
        else
        {
          if(bleProgramRegisteredCallback)
            // Notify that there was a problem
            bleProgramRegisteredCallback(0, 0);
          // Retriger self state
          BLE_PROGRAM_FSM_STATE_CHANGE(BLE_PROGRAM_FSM_READY, BLE_PROGRAM_DATA_WAIT_TIMEOUT);
          // Failed, Queue is full
          MyErrorReturn = RETURNCODE_TX_QUEUE_FULL;
          break;
        }
        break;
      case BLE_GO_OPCODE:
        if(bleProgramRegisteredCallback)
          // Notify that ACK was received
          bleProgramRegisteredCallback(data, length);
        sprintf((char*)bleProgramPrintBuffer, "BLE Go command ACK received, programming done - go back to idle");
        vlapmainDebugLog((char*)bleProgramPrintBuffer);
        // Go back to Idle
        bleProgramSetIdle();
        break;
      default:
        break;
      }
      break;
    case BLE_PROGRAM_STIMULI_RECEIVED_NACK:
      sprintf((char*)bleProgramPrintBuffer, "BLE program NACK received for sent data, failed! wait for next data package");
      vlapmainDebugLog((char*)bleProgramPrintBuffer);
      // Go and wait for the next data package at ready mode, set timeout for a minute
      BLE_PROGRAM_FSM_STATE_CHANGE(BLE_PROGRAM_FSM_READY, BLE_PROGRAM_DATA_WAIT_TIMEOUT);
      break;
    case BLE_PROGRAM_STIMULI_TIMEOUT:
      // Process timeout
      bleProgramSetIdle();
      break;
    default:
      break;
    }
    break;
  case BLE_PROGRAM_FSM_WRITE_DATA_ACK_WAIT:
    switch(stimuli)
    {
    case BLE_PROGRAM_STIMULI_RECEIVED_DATA:
      break;
    case BLE_PROGRAM_STIMULI_SEND_DATA:
      sprintf((char*)bleProgramPrintBuffer, "BLE program still waiting for ACK, cannot receive data right now");
      vlapmainDebugLog((char*)bleProgramPrintBuffer);
      break;
    case BLE_PROGRAM_STIMULI_RECEIVED_ACK:
      if(bleProgramRegisteredCallback)
        // Notify that ACK was received
        bleProgramRegisteredCallback(data, length);
      sprintf((char*)bleProgramPrintBuffer, "BLE program ACK received for sent data");
      vlapmainDebugLog((char*)bleProgramPrintBuffer);
      // Go and wait for the next data package at ready mode, set timeout for a minute
      BLE_PROGRAM_FSM_STATE_CHANGE(BLE_PROGRAM_FSM_READY, BLE_PROGRAM_DATA_WAIT_TIMEOUT);
      break;
    case BLE_PROGRAM_STIMULI_RECEIVED_NACK:
      if(bleProgramRegisteredCallback)
        // Notify that NACK was received
        bleProgramRegisteredCallback(data, length);
      sprintf((char*)bleProgramPrintBuffer, "BLE program NACK received for sent data, failed! wait for next data package");
      vlapmainDebugLog((char*)bleProgramPrintBuffer);
      // Go and wait for the next data package at ready mode, set timeout for a minute
      BLE_PROGRAM_FSM_STATE_CHANGE(BLE_PROGRAM_FSM_READY, BLE_PROGRAM_DATA_WAIT_TIMEOUT);
      break;
    case BLE_PROGRAM_STIMULI_TIMEOUT:
      // Process timeout
      bleProgramSetIdle();
      break;
    default:
      break;
    }
    break;
  default:
    break;
  }
  return MyErrorReturn;
}

#if 0

/******************************************************************************
*** ReturnCode_T bleTaskEventSend(bleTaskFsmQueueEntryActionType_T NotificationToBleTask )
* @brief  // Sends notifications to the bleTask   
* @param  
* @retval 
******************************************************************************/
ReturnCode_T bleBsipTaskEventSend(bleBsipOpCode_T BsipCommnad )
{
  
  switch(BsipCommnad)
  {
  case BLE_BSIP_OPCODE_READ_RESP:
    MsgLength = sizeof(bleBsipDllHeader_t) + sizeof(bleReadResponse_t) + sizeof(bleBsipDllEndOfMessage_t);
    // Allocate memory for the full message.
    // The buffer will be freed by the low level USB
    MsgPtr = pvPortMalloc(MsgLength);
    // Configurare the BLE Soc using BSIP and start advertisement.
    if(MsgPtr)
    {
      // Update the Bsip read response darta struct
      ((bleReadResponse_t*)(MsgPtr+sizeof(bleBsipDllHeader_t)))->PayloadApplicationLayerOperationCode = BLE_BSIP_OPCODE_READ_RESP;
      ((bleReadResponse_t*)(MsgPtr+sizeof(bleBsipDllHeader_t)))->PendingBytesCurrentMessage = NumberOfPayloadBytes;
      ((bleReadResponse_t*)(MsgPtr+sizeof(bleBsipDllHeader_t)))->PendingMessagesInLogMemory = eventsPendingEventsStatusGet();
      ((bleReadResponse_t*)(MsgPtr+sizeof(bleBsipDllHeader_t)))->ConfigurationVersionId = configVersionIdGet();
    }
    break;
  case BLE_BSIP_OPCODE_WRITE_RESP:
    MsgLength = sizeof(bleBsipDllHeader_t) + sizeof(bleWriteResponse_t) + sizeof(bleBsipDllEndOfMessage_t);
    // Allocate memory for the full message.
    // The buffer will be freed by the low level USB
    MsgPtr = pvPortMalloc(MsgLength);
    // Configurare the BLE Soc using BSIP and start advertisement.
    if(MsgPtr)
    {
      // Update the Bsip read response darta struct
      ((bleWriteResponse_t*)(MsgPtr+sizeof(bleBsipDllHeader_t)))->PayloadApplicationLayerOperationCode = BLE_BSIP_OPCODE_WRITE_RESP;
      ((bleWriteResponse_t*)(MsgPtr+sizeof(bleBsipDllHeader_t)))->WriteResponseStatus = 1;
    }
    break;
  case BLE_BSIP_OPCODE_ADVERTISING_CONTROL:
    MsgLength = sizeof(bleBsipDllHeader_t) + sizeof(bleAdvertisingControl_t) + sizeof(bleBsipDllEndOfMessage_t);
    // Allocate memory for the full message.
    // The buffer will be freed by the low level USB
    MsgPtr = pvPortMalloc(MsgLength);
    // Configure the BLE Soc using BSIP and start advertisement.
    if(MsgPtr)
    {
      uint8_t * BytePtr = protocolappUniqueIdPtrGet();
      // Update the Bsip read response darta struct
      ((bleAdvertisingControl_t*)(MsgPtr+sizeof(bleBsipDllHeader_t)))->PayloadApplicationLayerOperationCode = BLE_BSIP_OPCODE_ADVERTISING_CONTROL;
      sprintf(((bleAdvertisingControl_t*)(MsgPtr+sizeof(bleBsipDllHeader_t)))->AdvertisementString, "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
      BytePtr[0], BytePtr[1], BytePtr[2], BytePtr[3], BytePtr[4], BytePtr[5], BytePtr[6], BytePtr[7], BytePtr[8], BytePtr[9], BytePtr[10], BytePtr[11])        
    }
    
    break;
  case BLE_BSIP_OPCODE_NOTIFICATION_WRITE:
    break;
  case BLE_BSIP_OPCODE_DISCONNECTION_REQ:
    MsgLength = sizeof(bleBsipDllHeader_t) + sizeof(bleDisconnectionReq_t) + sizeof(bleBsipDllEndOfMessage_t);
    if(MsgPtr)
    {
      ((bleDisconnectionReq_t*)(MsgPtr+sizeof(bleBsipDllHeader_t)))->PayloadApplicationLayerOperationCode = BLE_BSIP_OPCODE_DISCONNECTION_REQ;
    }

    break;
  }
  
  
  
  
  
  // Send BSIP protocol message, The allocated message buffer will be freed by the UART DMA Tx ISR
  bleBsipDllMessageSend(MsgPtr, MsgLength);
  vTaskDelay(100);
}


#endif

