#ifndef COMM_H
#define	COMM_H
#include "FreeRTOS.h"
#include "queue.h"
#include "timers.h"

#include "common.h"
#include "uartdll.h"
#include "protocolapp.h"

extern TimerHandle_t CommFsmTimerHandlerPtr;

// G L O B A L   T Y P E S   A N D    D E F I N I T I O N S 
// comm FSM states
typedef enum { COMM_FSM_STATE_IDLE, COMM_FSM_STATE_BLE_SOC_WARMUP, COMM_FSM_STATE_ADVERTISE_BLE_CONNECTION_WAIT, COMM_FSM_STATE_CLIENT_SETUP_TIME_WAIT, COMM_FSM_STATE_SPIREAD_FIRST_PAGE_WAIT, COMM_FSM_STATE_SPIREAD_BLE_READ_REQ_WAIT, COMM_FSM_STATE_APP_ACK_WAIT, COMM_FSM_STATE_APP_CONFIG_RESPONSE_WAIT, COMM_FSM_STATE_BLE_CONNECTED_EVENTS_WAIT, COMM_FSM_STATE_SPIREAD_SYNC_SEARCH} commFsmStateT;
// comm FSM stimulis 

// comm FSM Queue Entry action type
typedef enum  {COMM_FSM_QUEUE_ENTRY_TYPE_SELF, COMM_FSM_QUEUE_ENTRY_TYPE_SPIFLASH_READ_COMPLETED, COMM_FSM_QUEUE_ENTRY_TYPE_BLE_CONNECTED, COMM_FSM_QUEUE_ENTRY_TYPE_BLE_READ_REQ, COMM_FSM_QUEUE_ENTRY_TYPE_BLE_DISCONNECTED, COMM_FSM_QUEUE_ENTRY_TYPE_BLE_TIMEOUT, COMM_FSM_QUEUE_ENTRY_TYPE_APP_ACK, COMM_FSM_QUEUE_ENTRY_TYPE_APP_NACK, COMM_FSM_QUEUE_ENTRY_TYPE_APP_INVALID_ACK, COMM_FSM_QUEUE_ENTRY_TYPE_TIMEOUT, COMM_FSM_QUEUE_ENTRY_TYPE_NO_NEW_CONFIG_RESP, COMM_FSM_QUEUE_ENTRY_TYPE_CONFIG_RESP, COMM_FSM_QUEUE_ENTRY_TYPE_COMM_ABORT, COMM_FSM_QUEUE_ENTRY_TYPE_CONFIG_SET_DONE, COMM_FSM_QUEUE_ENTRY_TYPE_SUSPEND_DURING_FWUPGRADE, COMM_FSM_QUEUE_ENTRY_TYPE_LAST_ENTRY} commFsmQueueEntryActionType_T;


// comm FSM Queue Entry action type
typedef enum  {COMM_SLEEP_STATE_NORMAL, COMM_SLEEP_STATE_STOP_MODE} commSleepState_T;

// comm Event corruption type
typedef enum  {COMM_SPI_CORRUPTION_SYNC, COMM_SPI_CORRUPTION_BAD_CRC} commSpiCorruption_T;

// comm FSM Queue Element  
typedef struct 
{
  commFsmQueueEntryActionType_T ActionType;
  uint8_t                       *DataPtr;
  uint16_t                      DataLength;
} commFsmQueueEntryT;

#define         COMM_FSM_STATE_CHANGE(NewState, QueueWairTimeoutIn10mSec)                  \
                {                                                                               \
                  commLastCommFsmTimeout = QueueWairTimeoutIn10mSec;                            \
                  commFsmStatePrev = commFsmState;                                                \
                  commFsmState=NewState;                                                     \
                xTimerChangePeriod( CommFsmTimerHandler, QueueWairTimeoutIn10mSec, 100);      \
                }                                                                              

#define         COMM_FSM_TIMEOUT_SET(QueueWairTimeoutIn10mSec)                  \
                {                                                                               \
                  xTimerChangePeriod( CommFsmTimerHandler, QueueWairTimeoutIn10mSec, 100);        \
                }                                                                               \


                  
#define         COMM_FSM_STATE_GET commFsmState




#define         COMM_FSM_INDEX_GET commIndex
#define         COMM_FSM_INDEX_SET(NewIndexVlaue) commIndex = NewIndexVlaue
#define         COMM_FSM_INDEX_INCREMENT commIndex++


#define         COMM_RXDLL_STATE_CHANGE(NewState, QueueWairTimeoutIn10mSec)                 \
                {                                                                                \
                  commRxDllState=NewState;                                                  \
                  commQueueWaitTimeout=(10*QueueWairTimeoutIn10mSec/portTICK_PERIOD_MS);       \
                }                                                                                \
  

typedef struct 
{
uint16_t NewEventMsgLength;
uint16_t BytesLeftToBeRead;
uint32_t NumberOfPagesToRead;
uint32_t CurrentSpiAddress;
uint32_t CurrentPage;
uint32_t CurrentSpiFlashAddress;
} commCurrentEventDescriptorT;

typedef struct 
{
uint32_t SyncPattern;
uint16_t EntrySize;     
uint32_t MessagesInMemory;
uint32_t CurrentPage;
uint32_t PrevEntryAddress;
uint32_t CurrentSpiFlashAddress;
uint32_t EndingSpiFlashAddress;
commSpiCorruption_T CorruptionType;
} commCorruptedEventDescriptorT;

#define COMM_TASK_SAMPLING_PERIOD       100


// E X T E R N A L S   
extern QueueHandle_t commFsmQueueHandle;

// G L O B A L  P R O T O T Y P E S 
ReturnCode_T commInit();
ReturnCode_T commTaskEventSend(commFsmQueueEntryActionType_T NotificationToCommTask );
commSleepState_T commSleepModeStateGet();
commFsmStateT commStateGet();
ReturnCode_T commMessageSendNotViaFlash(uartdllChannel_t Channel, uint32_t RequrstOrCommandSessionId, PROTOCOLAPP_COMMANDS_T messageOpCode, uint8_t* PayloadRelatedDataPtr, uint16_t PayloadRelatedDataLength, uint8_t FreeMemoryFlag);
ProtocolappHeader_t * commPendingProtocolHeaderEventPtrGet();

#endif
