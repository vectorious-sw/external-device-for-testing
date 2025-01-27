#pragma once

#include <hwdrivers.h>
#include "FreeRTOS.h"

#include "common.h"
#include "protocolApp.h"
#include "inet.h"
#include "rtc.h"
#include "vmicapplayer.h"

// G L O B A L   T Y P E S   A N D    D E F I N I T I O N S 
typedef enum  {BLE_CONTROL_DISABLE, BLE_CONTROL_ENABLE } BleControl_T;
typedef enum  {BLE_BOOT_DISABLE, BLE_BOOT_ENABLE } BleBootState_T;

// BLE Task FSM States 
typedef enum {BLE_TASK_FSM_STATE_IDLE, BLE_TASK_FSM_STATE_ON_WAIT, BLE_TASK_FSM_STATE_CONFIG_ACK_WAIT , BLE_TASK_FSM_STATE_CONNECTION_WAIT, BLE_TASK_FSM_STATE_CONNECTED} bleTaskFsmState_T;
// ble Task FSM Stimulies
typedef enum {BLE_TASK_FSM_STIMULI_NEW_TOKEN_RECVD, BLE_TASK_FSM_STIMULI_TIMOUT } bleTaskFsmStimuli_T;
// BLE Task FSM Queue Entry action type
typedef enum  {BLE_TASK_QUEUE_ENTRY_COMMAND_START, BLE_TASK_QUEUE_ENTRY_COMMAND_STOP, BLE_TASK_QUEUE_ENTRY_RESP_BSIP_BLE_ON_STATUS, BLE_TASK_QUEUE_ENTRY_RESP_BSIP_CONFIG_ACK, BLE_TASK_QUEUE_ENTRY_RESP_BSIP_TIMEOUT, BLE_TASK_QUEUE_ENTRY_RESP_BSIP_CONNECTED} bleTaskFsmQueueEntryActionType_T;
// BLE Task Input Queue Element  
typedef struct 
{
  bleTaskFsmQueueEntryActionType_T ActionType;
  uint8_t                          *DataPtr;
  uint16_t                         DataLength;
} bleTaskFsmQueueEntry_T;


// BLE Task state control macros
#define         BLE_TASK_FSM_STATE_CHANGE(NewState, QueueWairTimeoutIn10mSec)                  \
                {                                                                               \
                  bleTaskFsmState=NewState;                                                     \
                  BleTaskMsecDelay=(QueueWairTimeoutIn10mSec/portTICK_PERIOD_MS);        \
                }                                                                               \

#define         BLE_TASK_FSM_TIMEOUT_SET(QueueWairTimeoutIn10mSec)                  \
                {                                                                               \
                  BleTaskMsecDelay=(QueueWairTimeoutIn10mSec/portTICK_PERIOD_MS);        \
                }                                                                               \

                  
#define         BLE_FSM_STATE_GET bleFsmState

                  
#define         BLE_PROGRAM_FSM_STATE_CHANGE(NewState, QueueWairTimeoutIn1mSec)                  \
                {                                                                               \
                bleProgramState=NewState;                                                     \
                xTimerChangePeriod( bleProgramFsmTimerHandler, QueueWairTimeoutIn1mSec, 100);      \
                }                                                                               \
                  

// BLE BSIP Rx DLL 
typedef enum {BLE_BSIP_RX_DLL_FSM_STATE_IDLE, BLE_BSIP_RX_DLL_FSM_STATE_SYNC_WAIT, BLE_BSIP_RX_DLL_FSM_STATE_MSG_LENGTH_MSB_WAIT, BLE_BSIP_RX_DLL_FSM_STATE_MSG_LENGTH_LSB_WAIT, BLE_BSIP_RX_DLL_FSM_STATE_MSG_PAYLOAD_RCV_WAIT, BLE_BSIP_RX_DLL_FSM_STATE_MSG_CHECKSUM_WAIT} bleRxFsmState_T;
// ble fsm global vars and definitions
typedef enum {BLE_RX_DLL_FSM_STIMULI_INCOMMING_CHAR, BLE_RX_DLL_FSM_STIMULI_TIMOUT } bleDllFsmStimuli_T;
// BLE program process
typedef enum {BLE_PROGRAM_FSM_IDLE, BLE_PROGRAM_FSM_INIT_ACK_WAIT, BLE_PROGRAM_FSM_READY, BLE_PROGRAM_FSM_WRITE_OPCODE_ACK_WAIT, BLE_PROGRAM_FSM_WRITE_ADDRESS_ACK_WAIT, BLE_PROGRAM_FSM_WRITE_DATA_ACK_WAIT } bleProgramFsmState_T;
// BLE program stimuli
typedef enum {BLE_PROGRAM_STIMULI_RECEIVED_DATA, BLE_PROGRAM_STIMULI_RECEIVED_ACK, BLE_PROGRAM_STIMULI_RECEIVED_NACK, BLE_PROGRAM_STIMULI_SEND_DATA, BLE_PROGRAM_STIMULI_TIMEOUT } bleProgramFsmStimuli_T;
// BLE Bootloader RX
typedef enum {BLE_BOOTLOADER_RX_FSM_STATE_IDLE, BLE_BOOTLOADER_RX_FSM_STATE_MSG_LENGTH_WAIT, BLE_BOOTLOADER_RX_FSM_STATE_MSG_PAYLOAD_RCV_WAIT} bleRxBootloaderFsmState_T;

#define BLE_BSIP_SYNC_WORD       0x16
#define BLE_BOOTLOADER_SYNC_WORD 0x7f
#define BLE_BOOTLOADER_ACK       0x79
#define BLE_BOOTLOADER_NACK      0x1f
#define BLE_GO_OPCODE            0x21
#define BLE_ERASE_OPCODE         0x43
#define BLE_WRITE_OPCODE         0x31
// BSIP DLL Header
#pragma pack(1)
typedef  struct   {
  uint8_t  Sync; // 0x16
  uint8_t  PayloadLengthMsb; // Payload length in Bytes - Msb part
  uint8_t  PayloadLengthLsb; // Payload length in Bytes - Lsb part
} bleBsipDllHeader_t;

// BSIP DLL End Of Message 
#pragma pack(1)
typedef  struct  {
  uint8_t CheckSum; // 
} bleBsipDllEndOfMessage_t;


// ble BSIP Operational Codes
typedef enum {BLE_BSIP_OPCODE_READ_REQ=0x00, BLE_BSIP_OPCODE_WRITE_REQ=0x01, BLE_BSIP_OPCODE_BLE_STATUS_CHANGE=0x02, BLE_BSIP_OPCODE_PROCESSOR_STATUS=0x03, BLE_BSIP_OPCODE_READ_RESP=0x80, BLE_BSIP_OPCODE_WRITE_RESP=0x81, BLE_BSIP_OPCODE_ADVERTISING_CONTROL=0x82, BLE_BSIP_OPCODE_NOTIFICATION_WRITE=0x84, BLE_BSIP_OPCODE_DISCONNECTION_REQ=0x85  } bleBsipOpCode_T;

// BLE_BSIP_OPCODE_ADVERTISING_CONTROL
#pragma pack(1)
typedef  struct   {
  uint8_t  PayloadApplicationLayerOperationCode;
  uint8_t  AdvertisementControl;        // 0-Stop, 1-Start
  uint8_t AdvertisementString[50];      // String to be sent as part of the Advertisement
} bleAdvertisingControl_t;

#pragma pack(1)
typedef  struct   {
  uint8_t  PayloadApplicationLayerOperationCode;
} bleDisconnectionReq_t;

// BLE_BSIP_OPCODE_WRITE_RESP
#pragma pack(1)
typedef  struct   {
  uint8_t  PayloadApplicationLayerOperationCode;
  uint8_t  WriteResponseStatus;         // 0-Nack, 1-Ack
} bleWriteResponse_t;

// BLE_BSIP_OPCODE_BLE_STATUS_CHANGE
#pragma pack(1)
typedef  struct   {
  uint8_t  PayloadApplicationLayerOperationCode;
  uint8_t  UpdatedStatus;               // 0-BLE Disconneted, 1-BLE Connected
} bleStatusChange_t;

// BLE_BSIP_OPCODE_READ_REQ
#pragma pack(1)
typedef  struct   {
  uint8_t  PayloadApplicationLayerOperationCode;
  uint8_t  NumberOfRequestedBytes;      // 0-BLE Disconneted, 1-BLE Connected
} bleReadRequest_t;

// BLE_BSIP_OPCODE_WRITE_REQ
#pragma pack(1)
typedef  struct   {
  uint8_t  PayloadApplicationLayerOperationCode;
} bleWriteReq_t;

// BLE_BSIP_OPCODE_PROCESSOR_STATUS
#pragma pack(1)
typedef  struct {
  uint8_t  PayloadApplicationLayerOperationCode;
  uint16_t VersionNumber;
} bleBsipProcessorStatus_t;

// BLE_BSIP_OPCODE_READ_RESP
#pragma pack(1)
typedef  struct   {
  uint8_t  PayloadApplicationLayerOperationCode;
  uint16_t PendingBytesCurrentMessage;
  uint32_t PendingMessagesInLogMemory;
  uint16_t StatusSyncCounter;
  uint16_t Tbd;
  // Followed by payload bytes 
} bleReadResponse_t;

// L O C A L    T Y P E S   A N D    D E F I N I T I O N S  

// G L O B A L  P R O T O T Y P E S 
ReturnCode_T bleInit(void);
ReturnCode_T bleControl( BleControl_T BleControl);
ReturnCode_T  bleRxFsm(bleDllFsmStimuli_T stimuli, char  inChar);
ReturnCode_T  bleBsipDllRxFsm(bleDllFsmStimuli_T stimuli, char  inChar);
ReturnCode_T  bleBootloaderRxFsm(bleDllFsmStimuli_T stimuli, char  inChar);
ReturnCode_T bleTaskEventSend(bleTaskFsmQueueEntryActionType_T NotificationToBleTask );
ReturnCode_T bleBsipDllMessageSend(uint8_t* MessagePtr, uint16_t MessageLength);
uint32_t bleProcessorVersionGet();
ReturnCode_T bleSetBootState(BleBootState_T state);
BleBootState_T bleGetBootState();
ReturnCode_T bleProgramFsm(bleProgramFsmStimuli_T stimuli, char* data, uint16_t length, void(*ReceivedCallBackPtr)(char* receivedData, uint8_t receivedlength));
void bleBsipDllRxFsmIdleSet();

