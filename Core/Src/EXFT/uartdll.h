
#ifndef UARTDLL_H
#define UARTDLL_H

#include "FreeRTOS.h"
#include "queue.h"

#include "common.h"


// T Y P E S
typedef enum {UARTDLL_FSM_IDLE, UARTDLL_FSM_SYNC_LSB_WAIT, UARTDLL_FSM_MSG_LENGTH_MSB_WAIT, UARTDLL_FSM_MSG_LENGTH_LSB_WAIT, UARTDLL_FSM_MSG_PAYLOAD_RCV_WAIT, UARTDLL_FSM_MSG_CRC32_3_WAIT, UARTDLL_FSM_MSG_CRC32_2_WAIT, UARTDLL_FSM_MSG_CRC32_1_WAIT, UARTDLL_FSM_MSG_CRC32_0_WAIT} uartdllRxState_t;
typedef enum {UARTDLL_RCV_STATSTICS_ALL_COUNTERS, UARTDLL_RCV_STATSTICS_INTER_CHAR_TIMEOUT_COUNTER, UARTDLL_RCV_STATSTICS_CRC_ERROR_COUNTER, UARTDLL_RCV_STATSTICS_GOOD_PACKETS_COUNTER} uartdllRxStatistics_t;
// uartdll fsm global vars and definitions
typedef enum {UARTDLL_FSM_STIMULI_INCOMMING_CHAR, UARTDLL_FSM_STIMULI_TIMOUT } uartdllFsmStimuli_t;
// uartdll Tx phase
typedef enum {UARTDLL_TX_FIRST_CHUNK, UARTDLL_TX_MIDDLE_CHUNK, UARTDLL_TX_LAST_CHUNK} uartdllTxPhaseT;

#define UARTDLL_SYNC_WORD_MSB       'V'
#define UARTDLL_SYNC_WORD_LSB       'E'
#define UARTDLL_NUMBER_OF_CHANNLES      4
#define UARTDLL_PROTOCOL_OVERHEAD_LENGTH          7
#define UARTDLL_NUMBER_OF_CRC_BYTES       4

// We need this parameter to be multiplication of notification size(x20), 
// add it with 15 bytes(bsip headers) and to be less then 4000 bytes(max buffer at BLE processor)
#define BLE_UARTDLL_CHUNK_SIZE             3980

// DLL Header
#pragma pack(1)
typedef  struct   {
  uint8_t  Sync0; // "V"
  uint8_t  Sync1; // "E"        
  uint16_t PayloadLength; // Payloadload length in Bytes
} DllHeader_t;

// DLL End Of Message 
#pragma pack(1)
typedef struct  {
  uint32_t CRC32; // MSByte first CRC32 calculated one byte after the Message Length polynomial 0x04C11DB7
} DllEndOfMessage_t;


// uartdll FSM stimulis 
typedef enum  { UARTDLL_FSM_STIMULI_NEW_TOKEN_RECVD, UARTDLL_FSM_STIMULI_TIMEOUT, UARTDLL_FSM_QUEUE_ENTRY_TYPE_LAST_ENTRY} uartdllFsmQueueEntryActionType_T;
typedef enum { UARTDLL_FSM_STATE_IDLE} uartdllFsmState_T;


// uartdll FSM Queue Element  
typedef struct 
{
  uartdllFsmQueueEntryActionType_T ActionType;
  uint8_t                       *DataPtr;
  uint16_t                      DataLength;
} uartdllFsmQueueEntryT;




typedef enum{UARTDLL_UART_1_VTASGUI, UARTDLL_UART_2_CELLMODEM, UARTDLL_UART_3_BLE, UARTDLL_UART_6_RS485} uartdllChannel_t;

typedef struct
{
  uint32_t interCharTimeOutCounter;
  uint32_t crcErrorCounter;
  uint32_t RxMessageCounter;
  uint32_t TxMessageCounter;
}  uartdllStatisticsCounters_t;

extern uartdllStatisticsCounters_t uartdllStatisticsCounters[UARTDLL_NUMBER_OF_CHANNLES];
// P R O T O T Y P E S
void      uartdllFsmTimerCallback(void);
ReturnCode_T    uartDllRxFsm( uartdllChannel_t Channel, uartdllFsmStimuli_t stimuli, char  inChar );
ReturnCode_T  uartdllInit();
void      uartdllFsmTimerCallback();
ReturnCode_T  uartdllStatisticsCounterClear(uartdllChannel_t Channel, uartdllRxStatistics_t counterId);
ReturnCode_T  uartdllStatisticsCounterUpdate(uartdllChannel_t Channel, uartdllRxStatistics_t counterId);
ReturnCode_T uartdllTxQueueEnqueue(uartdllChannel_t Channel, uint8_t * DataPtr, uint16_t Length, uint8_t MemoryFreeNeeded);
ReturnCode_T uartdllMessageSend(uartdllChannel_t Channel, uartdllTxPhaseT TxPhase, uint8_t * MessagePtr, uint16_t MessageLength, uint16_t TxChunkSize);
ReturnCode_T uartdllInit();
ReturnCode_T uartdllDllMessageProcess(uartdllTxPhaseT TxPhase, uint8_t * MessagePtr, uint16_t MessageLength, uint16_t TxChunkSize);
ReturnCode_T uartdllCompleteMEssageBuild(uint8_t * MessagePtr, uint16_t MessageLength);
ReturnCode_T uartdllTaskEventSend(uartdllFsmQueueEntryActionType_T NotificationToUartdllTask, uint8_t * DatePtr, uint16_t DataLength);

// E X T E R N A L S
// Queue control blocks for uart channels 0 and 1

extern QueueHandle_t UartTransQueueHandle1;
extern QueueHandle_t UartTransQueueHandle2;
extern QueueHandle_t UartTransQueueHandle3;
extern QueueHandle_t UartTransQueueHandle6;


extern uint8_t uartUsart1MemFreeNeeded;
extern uint8_t uartUsart3MemFreeNeeded;
extern uint8_t uartUsart6MemFreeNeeded;


#endif  /* UARTDLL_H */

