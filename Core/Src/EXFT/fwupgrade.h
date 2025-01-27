#ifndef FWUPGRADE_H
#define	FWUPGRADE_H
#include "FreeRTOS.h"
#include "queue.h"
#include "timers.h"

#include "common.h"
#include "common.h"
#include "i2cwork.h"
#include "vlapconfig.h"
#include "vlapmain.h"


// G L O B A L   T Y P E S   A N D    D E F I N I T I O N S 
// Fwupgrade parameters. 

typedef enum {FWUPGRADE_STATE_IDLE, FWUPGRADE_STATE_FILE_HEADER_CHUNK_WAIT, FWUPGRADE_STATE_FILE_HEADER_SPI_WRITE_WAIT, FWUPGRADE_STATE_CHUNK_WAIT, FWUPGRADE_STATE_SPI_FLASH_WRITE_WAIT, FWUPGRADE_STATE_SPI_FLASH_WRITE_LAST_WAIT, FWUPGRADE_STATE_SPI_FLASH_FWUPGRADE_CONTROL_WRITE_WAIT} fwupgradeState_T;
typedef enum {FWUPGRADE_OPCODE_TIMEOUT, FWUPGRADE_OPCODE_START, FWUPGRADE_SPI_OPERATION_COMPLETE, FWUPGRADE_OPCODE_NEW_CHUNK, FWUPGRADE_OPCODE_FUG_CRC32_OK, FWUPGRADE_OPCODE_FUG_CRC32_FAILED, FWUPGRADE_OPCODE_STOP_DOWNLOAD} fwupgradeOpcode_T;
typedef enum uint32_t {FWUPGRADE_SYSTEM_STATE_NORMAL=1, FWUPGRADE_SYSTEM_STATE_UPGRADE_IN_ORDER=2} FwupgradeSystemState_T;

typedef enum { FWUPGRADE_FUG_CRC32_CHECK_STATE_IDLE, FWUPGRADE_FUG_CRC32_CHECK_STATE_HEADER_READ_WAIT, FWUPGRADE_FUG_CRC32_CHECK_STATE_SPIREAD_WAIT, FWUPGRADE_STATE_SPI_FLASH_CRC32_CALC_LAST_WAIT, FWUPGRADE_STATE_SPI_FLASH_CRC32_FWUPGRADE_CONTROL_WRITE_WAIT} FwupgradeStateSpiFugCrcCheckT;
typedef enum { FWUPGRADE_FUG_CRC32_CHECK_OPCODE_STIMULI_START, FWUPGRADE_FUG_CRC32_CHECK_OPCODE_STIMULI_SPI_COMPLETE, FWUPGRADE_FUG_CRC32_CHECK_OPCODE_STIMULI_TIMOUT} FwupgradeStateSpiFugCrcStimuliT;
typedef enum { FWUPGRADE_FUG_CRC32_CHECK_WITHOUT_RESET, FWUPGRADE_FUG_CRC32_CHECK_RESET} FwupgradeSpiFugCrcResetT;

// PROTOCOLAPP_FW_UPGRADE_START_RESP
#pragma pack(1)
typedef    struct
{
  uint8_t  SleepSuspendFlag;
  uint32_t ServerInfo2;
  uint32_t ServerInfo1;
  uint32_t ServerInfo0;
  uint8_t  TBD[19];
  // Follwed by payload 
} fwupgradeStartResp_T;


// PROTOCOLAPP_FW_UPGRADE_CHUNK_REQ
#pragma pack(1)
typedef    struct
{
  uint32_t FileOffset;
  uint32_t ChunkLength;
  uint32_t ServerInfo2;
  uint32_t ServerInfo1;
  uint32_t ServerInfo0;
  uint8_t  TBD[12];
  // Follwed by payload 
} fwupgradeChunkReqDataStruct_T;




#pragma pack(1)
typedef    struct
{
	fwupgradeOpcode_T       Opcode;
	uint8_t*                ChunkPtr;
	uint16_t                ChunkLength;
} fwupgradeQueueEntryT;

#define         FWUPGRADE_FSM_STATE_CHANGE(NewState, QueueWaitTimeoutIn10mSec)                  \
                {                                                                               \
                fwupgradeState=NewState;                                                     \
                xTimerChangePeriod( fwupgradeTimerHandler, QueueWaitTimeoutIn10mSec, 100);      \
                }                                                                              
#define         FWUPGRAD_FSM_TIMEOUT_SET(QueueWaitTimeoutIn10mSec)                  \
                {                                                                               \
                  xTimerChangePeriod( fwupgradeTimerHandler, QueueWaitTimeoutIn10mSec, 100);        \
                }                                                                               \

                  
                  
// FUG CRC check                   
#define         FWUPGRADE_FUG_CRC_CHECK_FSM_STATE_CHANGE_WITH_TIMEOUT(NewState, QueueWaitTimeoutIn10mSec)                  \
                {                                                                               \
                FugCrcCheckState = NewState;                                                     \
                xTimerChangePeriod( fwupgradeSpiFugCrcCheckTimerHandler, QueueWaitTimeoutIn10mSec, 100);      \
                } 
               
  
                  
                  
#pragma pack(1)
typedef   struct
{
  uint32_t    FileHeaderLength;
  uint32_t    FileImageCrc32;
  uint32_t    FileImageLength;
  uint32_t    FileCreationEpoch;
  uint32_t    FileVersion;
  uint32_t    FilesSubVersion;
  uint32_t    HwVersion;
  uint32_t    HwSubVersion;
  uint8_t     PaddingDifference;
  uint8_t     InitializationVector[16];
  uint8_t     Tbd[10];
} fwupgradeFileHeader_T;                  
                  
typedef struct __attribute__((packed))
{
	uint8_t	 FirstTimeFlag;
	uint16_t BootloaderVersion;
	uint8_t	 TBD;
}  fwupgradeVersionControlInfo_T;

#pragma pack(1)
typedef   struct
{
	FwupgradeSystemState_T  SystemUpgradeState;
	uint32_t    FwUpgradeEpochTime;		// The recorder Epoch time when the application have fully received the new FUG, verified its integrity and starts a reset to call the bootloader
	uint32_t    RevocationResetCounter;	// This counter holds the bootloader reset count. This count is relevant only if we are within the Revocation period
	fwupgradeVersionControlInfo_T Info;
}  fwupgradeControlBlock_T;


#pragma pack(1)
typedef  struct
{
	fwupgradeControlBlock_T    	VersionControlBlock;
	uint32_t    						Crc32;
}  fwupgradeVersionControlBlockInNvm_T;


// E X T E R N A L S   

// G L O B A L  P R O T O T Y P E S 
// G L O B A L S 

ReturnCode_T fwupgradeInit();
ReturnCode_T fwupgradeEventSend(fwupgradeOpcode_T opcode, uint8_t* ChunkPtr, uint16_t ChunkLength);
void fwupgradeStateReset();
fwupgradeState_T fwupgradeStateGet();
uint8_t fwupgradeFirstTimeFlagStateGet();
void   fwupgradeFirstTimeFlagSet(uint8_t FirstTimeFlag);
void fwupgradeSpiFugCrcCheckStart(uint32_t SpiAddress, FwupgradeSpiFugCrcResetT ResetRequest, void(*CompletionCallBackPtr)(int8_t Success));
uint32_t fwupgradeBootloaderVersionGet();


#endif
