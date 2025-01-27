#pragma once

#include "protocolApp.h"

typedef enum { 
	EVH_VBAT_HIGH,
	EVH_VBAT_LOW,
	EVH_VBAT_RESTORED_FROM_HIGH,
	EVH_VBAT_RESTORED_FROM_LOW,
} EVENT_TYPE;


#pragma pack(1)
typedef  struct sf_elog_typ
{
	ProtocolappHeader_t             ProtocolHeader;
	ProtocolappUpStreamHeader_t     UpStreamHeader;
        uint32_t                        FlashAddr;
} SF_ELOG;
 
#pragma pack(1)
typedef  struct {
    uint8_t LogMemoryTail1Operating:1;
    uint8_t LogMemoryTail2Operating:1;
  } eventsLogMemoryStateBitMap_t;


typedef enum { EVENTS_QUEUESTATE_EMPTY, EVENTS_QUEUESTATE_NOTEMPTY, EVENTS_QUEUESTATE_FULL} eventsEventsLogMemoryStateT;

#pragma pack(1)
typedef  struct {
    uint32_t Position;   
    uint32_t LengthTillEndOfLogBuffer;
    uint32_t LengthFromStartOfLogBuffer;
  } eventsLogMemoryReadWriteOperation_T;


typedef  struct {
    uint32_t eventsSpiFlashAddressHead; 
    uint32_t eventsSpiFlashAddressHeadPrev; 
    uint32_t eventsSpiFlashAddressTail;
    uint32_t eventsPendingMessagesInLogMemory;
    uint32_t eventsFailedTxMessagesCounter;
    uint32_t eventsTotalStoredNumberCounter;
    uint32_t Crc32;
  } eventsLogMemoryPointers_T;


// Events task FSM states definition
typedef enum { EVENTS_FSM_STATE_POINTERS_RESTORE_WAIT, EVENTS_FSM_STATE_IDLE, EVENTS_FSM_STATE_SPIWRITE_SINGLE_ALIGNED_PAGE_COMPLETION, EVENTS_FSM_STATE_SPIREAD_NONALIGNED_FIRST_PAGE_WAIT, EVENTS_FSM_STATE_SPIWRITE_ALL_PAGES_WAIT} eventsFsmStateT;
// Events FSM stimulis 
//typedef enum  { EVENTS_FSM_STIMULI_NEW_TOKEN_RECVD, EVENTS_FSM_STIMULI_TIMEOUT } eventsFsmStimuliT;
typedef enum  { EVENTS_FSM_STIMULI_NEW_EVENT, EVENTS_FSM_STIMULI_SPI_COMPLETION, EVENTS_FSM_STIMULI_TIMEOUT, EVENTS_LOG_MEM_POINTERS_RESTORED } eventsFsmStimuliT;
// LogMemory Head/Tail Enum 
typedef enum  { EVENTS_LOGMEMORY_HEAD, EVENTS_LOGMEMORY_TAIL} eventsLogMemoryPointerTypeT;
typedef enum  { EVENTS_NO_PENDING_EVENTS = 0, EVENTS_PENDING_EVENTS = 1 } eventsPendingStatusT;

// events Queue Element  
typedef struct 
{
uartdllChannel_t Channel;
eventsFsmStimuliT FsmStimuli;
uint32_t RequrstOrCommandSessionId;
PROTOCOLAPP_COMMANDS_T messageOpCode;
char * PayloadPtr;
uint16_t PayloadLength;
uint8_t PayloadMemFreeRequired;
} eventsQueueEntryT;


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

} eventsCurrentEventDescriptorT;



extern volatile QueueHandle_t eventsQueueHandle;

extern  eventsLogMemoryPointers_T eventsLogMemoryPointersStructure __attribute__( ( section( ".noinit") ) );

extern SemaphoreHandle_t eventsNewEventSavedSemaphoreHandle;


// P R O T O T Y P E S 
ReturnCode_T eventsQueuePointerIndexUpdate(eventsLogMemoryPointerTypeT LogMemoryPointerType, eventsEventsLogMemoryStateT *EventsLogMemoryStatePtr, uint32_t NewEntrySize, bool eventSkipped);
ReturnCode_T eventsEventWrite(uartdllChannel_t Channel, uint32_t RequrstOrCommandSessionId, PROTOCOLAPP_COMMANDS_T messageOpCode, char * PayloadPtr, uint16_t PayloadLength, uint8_t PayloadMemFreeRequired);
ReturnCode_T eventsQueueStatusAndTailEntryGet(eventsEventsLogMemoryStateT *EventsLogMemoryStatePtr, uint32_t *NewEntryAddressPtr);
ReturnCode_T eventsLogMemoryCmd( ProtocolappLogMemoryStructCmd_t* LogMemoryCmdPtr);
ReturnCode_T eventsLogMemoryFullCheck(uint16_t NewMessageEntryLength);
uint32_t eventsPendingEventsStatusGet();
uint32_t eventsQueueTailIndexGet();
uint32_t eventsQueuePrevHeadIndexGet();
ReturnCode_T eventsInit();
uint32_t eventsPendingEventsStatusGet();
ReturnCode_T eventsQueueStatusAndTailEntryAddressGet(eventsEventsLogMemoryStateT *EventsLogMemoryStatePtr, uint32_t *QueueEntryPtr);
void eventsFirstTimePointersClear();
void eventsJlinkProgrammaingLogMemoryPointersSet();
void  eventsMemoryStatePrint(uint8_t CallSource, PROTOCOLAPP_COMMANDS_T QueueOpCode);
uint16_t eventsEventInProgressStateGet();
//void eventsLogMemoryPointersToNvmSave();
void eventsLogMemoryPointersToNvmSavePVD();
void eventsLogMemoryPointersToNvmErase();
eventsFsmStateT eventsStateGet();
eventsPendingStatusT eventsPendingStatusGet();
uint32_t eventsGetEventsArea();

