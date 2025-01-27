#pragma once

#include "protocolApp.h"


// Spiflashdisk task FSM states definition
typedef enum { SPIFLASHDISK_FSM_STATE_POINTERS_RESTORE_WAIT, SPIFLASHDISK_FSM_STATE_IDLE, SPIFLASHDISK_FSM_STATE_SPIWRITE_SINGLE_ALIGNED_PAGE_COMPLETION, SPIFLASHDISK_FSM_STATE_SPIREAD_NONALIGNED_FIRST_PAGE_WAIT, SPIFLASHDISK_FSM_STATE_SPIWRITE_ALL_PAGES_WAIT} spiflashdiskFsmStateT;
// Spiflashdisk FSM stimulis 
//typedef enum  { SPIFLASHDISK_FSM_STIMULI_NEW_TOKEN_RECVD, SPIFLASHDISK_FSM_STIMULI_TIMEOUT } spiflashdiskFsmStimuliT;
typedef enum  { SPIFLASHDISK_FSM_STIMULI_WRITE, SPIFLASHDISK_FSM_STIMULI_SPI_COMPLETION, SPIFLASHDISK_FSM_STIMULI_TIMEOUT} spiflashdiskFsmStimuliT;


// spiflashdisk Queue Element  
typedef struct 
{
  spiflashdiskFsmStimuliT FsmStimuli;
  uint32_t SpiAddress;
  uint8_t* WriteBufferPtr;
  uint32_t WriteBufferLength;
  void (*CompletionCallBackPtr)();
  uint8_t       FreeTxBufferFlag;
} spiflashdiskQueueEntryT;


typedef struct 
{
  uint8_t *NewEventMsgPtr;
  uint8_t *AllocatedMessagePtr;
  uint16_t NewEventMsgLength;
  uint32_t BytesLeftToBeWritten;
  uint32_t NumberOfPagesToWrite;
  uint32_t NumberOfEmptyBytesInFirstPage;
  uint32_t CurrentPage;
  uint32_t CurrentSpiFlashAddress;
  void (*CompletionCallBackPtr)();
} spiflashdiskCurrentEventDescriptorT;


#define         SPIFLASHDISK_FSM_STATE_CHANGE(NewState, QueueWaitTimeoutIn10mSec)       \
{                                                                                       \
  spiflashdiskFsmState=NewState;                                                        \
  xTimerChangePeriod(SpiflashdiskFsmTimerHandler, QueueWaitTimeoutIn10mSec, 100);       \
}                                                                                       \
  
  
#define         SPIFLASHDISK_FSM_STATE_CHANGE_NO_TIMEOUT(NewState)                      \
  {                                                                                     \
    spiflashdiskFsmState=NewState;                                                      \
  }                                                                                     \
    
    
    
    
    
    
    // P R O T O T Y P E S 
    ReturnCode_T spiflashdiskInit();
    ReturnCode_T spiflashdiskReqEnqueue(spiflashdiskFsmStimuliT OpCode, uint32_t SpiFlashAddress, uint8_t *RxDestinationPtr, uint8_t *TxSourcePtr, uint16_t OperationBytesCount, void(*CompletionCallBackPtr)(), uint8_t FreeTxBufferFlag);
    
    
    
    