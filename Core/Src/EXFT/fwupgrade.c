#include "common.h"
#include "fwupgrade.h"
#include "config.h"
#include "spiflash.h"
#include "inet.h"
#include "crc32.h"
#include "spiflashdisk.h"
#include "charger.h"
#include "audit.h"

//D E F I N E S
#define FWUPGRADE_FWUPGRADE_CHUNK_LENGTH   (uint32_t) 1000
#define FWUPGRADE_CHUNK_TIMEOUT /*6000*/ 12000
#define FWUPGRADE_CHUNK_MAX_RETRY 5

// G L O B A L S 
QueueHandle_t fwupgradeRequestQueueHandle;
TimerHandle_t fwupgradeTimerHandler;
fwupgradeState_T fwupgradeState;
fwupgradeFileHeader_T MyFileHeader;
fwupgradeChunkReqDataStruct_T MyFwUpgradeManagementBlock;
uint32_t CurrentOffset;
//uint8_t* CurrentAllocateSpiBufferPtr;
uint8_t CurrentAllocateSpiBufferPtr[FWUPGRADE_FWUPGRADE_CHUNK_LENGTH];

uint32_t CurrentSpiFlashAddress;
uint32_t SpiFlashAddressStart;
FwupgradeSpiFugCrcResetT CrcCheckResetRequest;
uint8_t* LastChunkMsgPtr;
uint32_t Crc32Accumulator;
fwupgradeVersionControlBlockInNvm_T FwUpgradeControlBlockInNvm;
uint32_t OffsetWithinPage;
uint32_t LaskChunkLength;
fwupgradeStartResp_T MyLocalFwUpgradeStartRequest;
// FUG CRC32 test
FwupgradeStateSpiFugCrcCheckT FugCrcCheckState; 
TimerHandle_t fwupgradeSpiFugCrcCheckTimerHandler;
fwupgradeFileHeader_T* fwupgradeSpiFugCrcCheckHeaderPtr;
void(*fwupgradeSpiFugCrcCheckCallBackPtr)(int8_t Success);


// P R O T O T Y P E S 
void fwupgradeTimerTimeoutCallback();
void fwupgradeSpiCompleteCallback();
void  fwupgradeFileHeaderOrderFix(fwupgradeFileHeader_T* FwUpgradeFileHeader);
void  fwupgradeFileHeaderOrderFix(fwupgradeFileHeader_T* FwUpgradeFileHeader);
void  fwupgradehunkReqHeaderOrderFix(fwupgradeChunkReqDataStruct_T* FwUpgradeChunkReqHeaderPtr);
void fwupgradeSpiFugCrcCheck(FwupgradeStateSpiFugCrcStimuliT Stimuli);
void fwupgradeSpiFugCrcCheckTimeoutCallback();
void fwupgradeSpiFugCrcCheckCompletionCallBack(int8_t CompletionStatus);
void fwupgradeSpiFugCrcCheckSpiCompleteCallBack();



uint32_t TestArray[300];
uint8_t TestArrayIndex = 0; 

uint16_t ChunkCounter;

uint8_t fwupgradeChunkRetry = 0;

/******************************************************************************
* @brief  ReturnCode_T fwupgradeInit(void)
* @param  
* @retval 
******************************************************************************/
ReturnCode_T fwupgradeInit()
{
  // Create input queue  
  fwupgradeRequestQueueHandle = xQueueCreate(5, sizeof(fwupgradeQueueEntryT));
  
  // Create the task
  xTaskCreate(fwupgradeTask, fwupgradeTaskName, fwupgradeTaskSTACK_SIZE, NULL,  fwupgradeTaskPriority, ( TaskHandle_t * ) NULL );
  // Create timer.
  fwupgradeTimerHandler =  xTimerCreate("fwupgradeTimer",  portTICK_PERIOD_MS, pdFALSE, (void *)0, fwupgradeTimerTimeoutCallback);
  
  // Allways return OK
  return(RETURNCODE_OK);
}

/******************************************************************************
* @brief  portTASK_FUNCTION(fwupgradeTask, pvParameters )
* @param  fwupgradeTask, pvParameters
* @retval 
******************************************************************************/
portTASK_FUNCTION(fwupgradeTask, pvParameters )
{
  fwupgradeQueueEntryT QueueEntry;
  ReturnCode_T MyReturnCode;
  
  char PrintBuff[100];
  
  while(1)
  {   
    // block task till new queue entry is received or timeout 
    BaseType_t  QueueState = xQueueReceive(fwupgradeRequestQueueHandle, &QueueEntry, 10000);
    // Check if there is new message.
    if(QueueState == pdTRUE)
    {
      if( (chargerPlugStateGet() != CHARGER_PLUG_DISCONNECTED) && (auditBatteryStatusGet() != AUDIT_BV_CROSSED_LB_SENDED))
      //if((chargerPlugStateGet() == CHARGER_PLUG_DISCONNECTED)&& (auditBatteryStatusGet() != AUDIT_BV_CROSSED_UB_SENDED))
        
      {
        switch(fwupgradeState)
        {
        case FWUPGRADE_STATE_IDLE:
          switch (QueueEntry.Opcode)
          {
          case FWUPGRADE_OPCODE_START:
            ChunkCounter = 0;
            // FW Upgrade start request received form the server 
            // MyLocalFwUpgradeStartRequest
            memcpy((uint8_t*)&MyLocalFwUpgradeStartRequest, QueueEntry.ChunkPtr , sizeof(fwupgradeChunkReqDataStruct_T));
            MyLocalFwUpgradeStartRequest.ServerInfo0 = NTOHL(MyLocalFwUpgradeStartRequest.ServerInfo0);
            MyLocalFwUpgradeStartRequest.ServerInfo1 = NTOHL(MyLocalFwUpgradeStartRequest.ServerInfo1);
            MyLocalFwUpgradeStartRequest.ServerInfo2 = NTOHL(MyLocalFwUpgradeStartRequest.ServerInfo2);
            memcpy((uint8_t*)&MyFileHeader, QueueEntry.ChunkPtr + sizeof(fwupgradeChunkReqDataStruct_T), sizeof(fwupgradeFileHeader_T));
            // Ask for the first chunk
            MyFwUpgradeManagementBlock.FileOffset = 0;
            CurrentSpiFlashAddress = SPIFLASH_UPGRADE_START;
            // TODO: 
            MyFwUpgradeManagementBlock.ChunkLength = NTOHL(sizeof(fwupgradeFileHeader_T));
            MyFwUpgradeManagementBlock.ServerInfo0 = NTOHL(MyLocalFwUpgradeStartRequest.ServerInfo0);
            MyFwUpgradeManagementBlock.ServerInfo1 = NTOHL(MyLocalFwUpgradeStartRequest.ServerInfo1);
            MyFwUpgradeManagementBlock.ServerInfo2 = NTOHL(MyLocalFwUpgradeStartRequest.ServerInfo2);
            
            if(QueueEntry.ChunkPtr)
              vPortFree(QueueEntry.ChunkPtr);

            commMessageSendNotViaFlash(0, 0, PROTOCOLAPP_FW_UPGRADE_CHUNK_REQ, (uint8_t*)&MyFwUpgradeManagementBlock, sizeof(MyFwUpgradeManagementBlock), 0);
            // Send Config request event
            vlapmainDebugLog("Request FUG header");
            FWUPGRADE_FSM_STATE_CHANGE(FWUPGRADE_STATE_FILE_HEADER_CHUNK_WAIT, /*30000*/ FWUPGRADE_CHUNK_TIMEOUT);  
            break;
          case FWUPGRADE_OPCODE_NEW_CHUNK:
          case FWUPGRADE_OPCODE_TIMEOUT:
          case FWUPGRADE_OPCODE_STOP_DOWNLOAD:
            if(QueueEntry.ChunkPtr)
              vPortFree(QueueEntry.ChunkPtr);
            break;
          case FWUPGRADE_SPI_OPERATION_COMPLETE:
          case FWUPGRADE_OPCODE_FUG_CRC32_OK:
          case FWUPGRADE_OPCODE_FUG_CRC32_FAILED:
			break;
          }
          break;
          
          
        case FWUPGRADE_STATE_FILE_HEADER_CHUNK_WAIT:
          switch (QueueEntry.Opcode)
          {
          case FWUPGRADE_OPCODE_START:
            if(QueueEntry.ChunkPtr)
              vPortFree(QueueEntry.ChunkPtr);
            break;
          case FWUPGRADE_OPCODE_NEW_CHUNK:
            vlapmainDebugLog("Got the header");
            // Copy the contents of the received file header to the local management data structure.
            memcpy((uint8_t*)&MyFileHeader, QueueEntry.ChunkPtr + sizeof(fwupgradeChunkReqDataStruct_T), sizeof(fwupgradeFileHeader_T));
            if(QueueEntry.ChunkPtr)
              vPortFree(QueueEntry.ChunkPtr);
            if(NTOHL(MyFileHeader.FileHeaderLength))
            {
              // Write to the spiFlash 
              spiflashdiskReqEnqueue(SPIFLASHDISK_FSM_STIMULI_WRITE        , CurrentSpiFlashAddress, 0, (uint8_t*)&MyFileHeader, sizeof(fwupgradeFileHeader_T), &fwupgradeSpiCompleteCallback, 0);
              FWUPGRADE_FSM_STATE_CHANGE(FWUPGRADE_STATE_FILE_HEADER_SPI_WRITE_WAIT, 100);  
            }
            else
              FWUPGRADE_FSM_STATE_CHANGE(FWUPGRADE_STATE_IDLE, 100);  
            break;
          case FWUPGRADE_OPCODE_TIMEOUT:
            if(QueueEntry.ChunkPtr)
              vPortFree(QueueEntry.ChunkPtr);
            vlapmainDebugLog("Header timeout");
            FWUPGRADE_FSM_STATE_CHANGE(FWUPGRADE_STATE_IDLE, 100);  
            break;
          case FWUPGRADE_OPCODE_STOP_DOWNLOAD:
            if(QueueEntry.ChunkPtr)
              vPortFree(QueueEntry.ChunkPtr);
            FWUPGRADE_FSM_STATE_CHANGE(FWUPGRADE_STATE_IDLE, 100);  
            break;
          default:
        	  break;
          }
          break;
          
          
          
          
        case FWUPGRADE_STATE_FILE_HEADER_SPI_WRITE_WAIT:
          switch (QueueEntry.Opcode)
          {
          case FWUPGRADE_SPI_OPERATION_COMPLETE:
            vlapmainDebugLog("Header written to flash");
            // Fix the header endianees
            MyFileHeader.FileHeaderLength  = NTOHL( MyFileHeader.FileHeaderLength);
            MyFileHeader.FileImageCrc32    = NTOHL( MyFileHeader.FileImageCrc32);   
            MyFileHeader.FileImageLength   = NTOHL( MyFileHeader.FileImageLength);  
            MyFileHeader.FileCreationEpoch = NTOHL( MyFileHeader.FileCreationEpoch);
            MyFileHeader.FileVersion       = NTOHL( MyFileHeader.FileVersion);      
            MyFileHeader.FilesSubVersion   = NTOHL( MyFileHeader.FilesSubVersion);  
            MyFileHeader.HwVersion         = NTOHL( MyFileHeader.HwVersion);        
            MyFileHeader.HwSubVersion      = NTOHL( MyFileHeader.HwSubVersion); 
            
            if(MyFileHeader.FileHeaderLength)
            {
              CurrentSpiFlashAddress += sizeof(fwupgradeFileHeader_T);
              CurrentOffset = sizeof(MyFileHeader);
              MyFwUpgradeManagementBlock.FileOffset = HTONL(CurrentOffset);
              MyFwUpgradeManagementBlock.ChunkLength = HTONL(FWUPGRADE_FWUPGRADE_CHUNK_LENGTH);
              MyFwUpgradeManagementBlock.ServerInfo0 = NTOHL(MyLocalFwUpgradeStartRequest.ServerInfo0);
              MyFwUpgradeManagementBlock.ServerInfo1 = NTOHL(MyLocalFwUpgradeStartRequest.ServerInfo1);
              MyFwUpgradeManagementBlock.ServerInfo2 = NTOHL(MyLocalFwUpgradeStartRequest.ServerInfo2);
              commMessageSendNotViaFlash(0, 0, PROTOCOLAPP_FW_UPGRADE_CHUNK_REQ, (uint8_t*)&MyFwUpgradeManagementBlock, sizeof(MyFwUpgradeManagementBlock), 0);
              sprintf(PrintBuff, "Request Chunk=  %id, Offset= %id",   ChunkCounter, (uint32_t)CurrentOffset);
              vlapmainDebugLog(PrintBuff);
              FWUPGRADE_FSM_STATE_CHANGE(FWUPGRADE_STATE_CHUNK_WAIT, /*30000*/ FWUPGRADE_CHUNK_TIMEOUT);  
            }
            else
              FWUPGRADE_FSM_STATE_CHANGE(FWUPGRADE_STATE_IDLE, 100);  
            break;
          case FWUPGRADE_OPCODE_TIMEOUT:
          case FWUPGRADE_OPCODE_START:
          case FWUPGRADE_OPCODE_NEW_CHUNK:
          case FWUPGRADE_OPCODE_STOP_DOWNLOAD:
            FWUPGRADE_FSM_STATE_CHANGE(FWUPGRADE_STATE_IDLE, 100);  
            break;
          default:
        	  break;
          }
          break;
          
          
          
        case FWUPGRADE_STATE_CHUNK_WAIT:
          switch (QueueEntry.Opcode)
          {
          case FWUPGRADE_OPCODE_START:
            break;
          case FWUPGRADE_OPCODE_NEW_CHUNK:
            if(QueueEntry.ChunkPtr)
            {
              LastChunkMsgPtr = QueueEntry.ChunkPtr;
              vlapmainDebugLog("New chunk received");
              //         fwupgradehunkReqHeaderOrderFix( (fwupgradeChunkReqDataStruct_T*)LastChunkMsgPtr);
              //           uint32_t ChunkLength = (((fwupgradeChunkReqDataStruct_T*)(LastChunkMsgPtr))->ChunkLength);
              //           uint32_t FileOffset =  (((fwupgradeChunkReqDataStruct_T*)(LastChunkMsgPtr))->FileOffset);
              //           uint32_t FileOffset = CurrentOffset;
              uint32_t ChunkLength  = FWUPGRADE_FWUPGRADE_CHUNK_LENGTH;
              // Start writing the chunk to the spiFlash
              spiflashdiskReqEnqueue(SPIFLASHDISK_FSM_STIMULI_WRITE      , CurrentSpiFlashAddress, 0, LastChunkMsgPtr + sizeof(fwupgradeChunkReqDataStruct_T), FWUPGRADE_FWUPGRADE_CHUNK_LENGTH, &fwupgradeSpiCompleteCallback, 0);
              // Initialize chunk retries for the next chunk download
              fwupgradeChunkRetry = 0;
              FWUPGRADE_FSM_STATE_CHANGE(FWUPGRADE_STATE_SPI_FLASH_WRITE_WAIT, 2000);
            }
            break;
          case FWUPGRADE_OPCODE_TIMEOUT:
            if(fwupgradeChunkRetry == FWUPGRADE_CHUNK_MAX_RETRY)
            {
              // Retries reached to max tries, abort FW upgrade
              vlapmainDebugLog("Chunk reception timeout, max retries reached, abort FW upgrade");
              FWUPGRADE_FSM_STATE_CHANGE(FWUPGRADE_STATE_IDLE, 100);  
              // There was a timeout - abort
              commTaskEventSend(COMM_FSM_QUEUE_ENTRY_TYPE_TIMEOUT);
              break;
            }
            else
            {
              // Retry sending the chunk request, simulates end of spi  
              vlapmainDebugLog("Chunk reception timeout, Retry asking for next chunk");
              commMessageSendNotViaFlash(0, 0, PROTOCOLAPP_FW_UPGRADE_CHUNK_REQ, (uint8_t*)&MyFwUpgradeManagementBlock, sizeof(MyFwUpgradeManagementBlock), 0);
            }

            fwupgradeChunkRetry++;
            FWUPGRADE_FSM_STATE_CHANGE(FWUPGRADE_STATE_CHUNK_WAIT, /*20000*/ FWUPGRADE_CHUNK_TIMEOUT);  
            break;
          case FWUPGRADE_OPCODE_STOP_DOWNLOAD:
            FWUPGRADE_FSM_STATE_CHANGE(FWUPGRADE_STATE_IDLE, 100);  
            break;
          default:
        	  break;
          }
          break;
          
          
          
        case FWUPGRADE_STATE_SPI_FLASH_WRITE_WAIT:
          switch (QueueEntry.Opcode)
          {
          case FWUPGRADE_SPI_OPERATION_COMPLETE:
            if(MyFileHeader.FileImageLength > (CurrentOffset - sizeof(fwupgradeFileHeader_T)))
            {
              if((MyFileHeader.FileImageLength - (CurrentOffset /*+*/ - sizeof(fwupgradeFileHeader_T)) > FWUPGRADE_FWUPGRADE_CHUNK_LENGTH))
              {
                CurrentSpiFlashAddress += FWUPGRADE_FWUPGRADE_CHUNK_LENGTH;
                CurrentOffset += FWUPGRADE_FWUPGRADE_CHUNK_LENGTH;
                
                MyFwUpgradeManagementBlock.FileOffset = HTONL(CurrentOffset);
                MyFwUpgradeManagementBlock.ChunkLength = HTONL(FWUPGRADE_FWUPGRADE_CHUNK_LENGTH);
                commMessageSendNotViaFlash(0, 0, PROTOCOLAPP_FW_UPGRADE_CHUNK_REQ, (uint8_t*)&MyFwUpgradeManagementBlock, sizeof(MyFwUpgradeManagementBlock), 0);
                ChunkCounter++;
                sprintf(PrintBuff, "Request Chunk=  %ld, Offset= %ld",   ChunkCounter, CurrentOffset);
                vlapmainDebugLog(PrintBuff);
                // Free the allocated buffer
                if(LastChunkMsgPtr)
                  vPortFree(LastChunkMsgPtr);
                FWUPGRADE_FSM_STATE_CHANGE(FWUPGRADE_STATE_CHUNK_WAIT, /*20000*/ FWUPGRADE_CHUNK_TIMEOUT);  
              }
              else
              {
                uint32_t lastChunkSize = MyFileHeader.FileImageLength + sizeof(fwupgradeFileHeader_T) - CurrentOffset;
                // Last chunk to write to spiFlash
                MyFwUpgradeManagementBlock.FileOffset = HTONL(CurrentOffset);
                MyFwUpgradeManagementBlock.ChunkLength = HTONL(lastChunkSize);
                commMessageSendNotViaFlash(0, 0, PROTOCOLAPP_FW_UPGRADE_CHUNK_REQ, (uint8_t*)&MyFwUpgradeManagementBlock, sizeof(MyFwUpgradeManagementBlock), 0);
                sprintf(PrintBuff, "Request Last chunk, Current offset= %ld, size= %ld", CurrentOffset, lastChunkSize);
                vlapmainDebugLog(PrintBuff);
                // Mark as the last chunk 
                CurrentOffset += lastChunkSize;
                // Free the allocated buffer
                if(LastChunkMsgPtr)
                  vPortFree(LastChunkMsgPtr);
                FWUPGRADE_FSM_STATE_CHANGE(FWUPGRADE_STATE_CHUNK_WAIT, /*20000*/ FWUPGRADE_CHUNK_TIMEOUT);  
              }
            }
            else
            {
              if(LastChunkMsgPtr)
                vPortFree(LastChunkMsgPtr);
              fwupgradeSpiFugCrcCheckStart(SPIFLASH_UPGRADE_START, FWUPGRADE_FUG_CRC32_CHECK_RESET, fwupgradeSpiFugCrcCheckCompletionCallBack);
              FWUPGRADE_FSM_STATE_CHANGE(FWUPGRADE_STATE_SPI_FLASH_WRITE_LAST_WAIT, 5000);  
            }
            break;
          case FWUPGRADE_OPCODE_STOP_DOWNLOAD:
            if(LastChunkMsgPtr)
              vPortFree(LastChunkMsgPtr);
            FWUPGRADE_FSM_STATE_CHANGE(FWUPGRADE_STATE_IDLE, 100); 
            break;
          default:
            break;
          }
          break;
          
          
        case FWUPGRADE_STATE_SPI_FLASH_WRITE_LAST_WAIT:
          switch (QueueEntry.Opcode)
          {
          case FWUPGRADE_OPCODE_FUG_CRC32_OK:
            vlapmainDebugLog("FUG CRC32 OK");
            // We expect that the external will reset with the new version so we don't need to go to idle
            break;
          case FWUPGRADE_OPCODE_FUG_CRC32_FAILED:
            // There was a problem during the FUG CRC check - go back to idle
            FWUPGRADE_FSM_STATE_CHANGE(FWUPGRADE_STATE_IDLE, 100);  
            // There was a CRC error - abort
            commTaskEventSend(COMM_FSM_QUEUE_ENTRY_TYPE_TIMEOUT);
            break;
          default:
            break;
          }
          break; 
          default:
        	  break;
                  
        }
      }
    }
  }
}











/******************************************************************************
* @brief  void fwupgradeTimerTimeoutCallback()
* @param  
* @retval 
******************************************************************************/
void fwupgradeTimerTimeoutCallback()
{
  fwupgradeEventSend(FWUPGRADE_OPCODE_TIMEOUT, 0, 0);
}


uint32_t AllocationErrorCounter __attribute__( ( section( ".noinit") ) );
uint32_t LastChunkLength __attribute__( ( section( ".noinit") ) );

fwupgradeQueueEntryT fwupgradeMyQueueEntry;

/******************************************************************************
* @brief  ReturnCode_T fwupgradeEventSend(fwupgradeOpcode_T opcode, uint8_t* ChunkPtr, uint16_t ChunkLength)
* @param  
* @retval 
******************************************************************************/
ReturnCode_T fwupgradeEventSend(fwupgradeOpcode_T Opcode, uint8_t* ChunkPtr, uint16_t ChunkLength)
{
  ReturnCode_T MyeturnedCode = RETURNCODE_OK;
  
  if(fwupgradeRequestQueueHandle)
  {
    if(ChunkPtr && ChunkLength)
    {
      uint8_t* AllocatedChunkPtr = pvPortMalloc(ChunkLength);
      LastChunkLength = ChunkLength;
      
      if(AllocatedChunkPtr)
      {
        memcpy(AllocatedChunkPtr, ChunkPtr, ChunkLength);
        fwupgradeMyQueueEntry.Opcode = Opcode;
        fwupgradeMyQueueEntry.ChunkPtr = AllocatedChunkPtr;
        fwupgradeMyQueueEntry.ChunkLength = ChunkLength;
        xQueueSend(fwupgradeRequestQueueHandle, &fwupgradeMyQueueEntry, 10);
        MyeturnedCode = RETURNCODE_OK;
      }
      else
      {
        AllocationErrorCounter++;
        MyeturnedCode = RETURNCODE_ERROR;
      }
    }
    else
    {
      fwupgradeMyQueueEntry.Opcode = Opcode;
      fwupgradeMyQueueEntry.ChunkPtr = 0;
      fwupgradeMyQueueEntry.ChunkLength = 0;
      xQueueSend(fwupgradeRequestQueueHandle, &fwupgradeMyQueueEntry, 10);
      MyeturnedCode = RETURNCODE_OK;
    }
  }
  else
    MyeturnedCode = RETURNCODE_ERROR;
  
  
  return(MyeturnedCode);
}




void  fwupgradeFileHeaderOrderFix(fwupgradeFileHeader_T* FwUpgradeFileHeader)
{
  FwUpgradeFileHeader->FileHeaderLength         =      NTOHL(FwUpgradeFileHeader->FileHeaderLength); 
  FwUpgradeFileHeader->FileImageCrc32           =      NTOHL(FwUpgradeFileHeader->FileImageCrc32);   
  FwUpgradeFileHeader->FileImageLength          =      NTOHL(FwUpgradeFileHeader->FileImageLength);  
  FwUpgradeFileHeader->FileCreationEpoch        =      NTOHL(FwUpgradeFileHeader->FileCreationEpoch);
  FwUpgradeFileHeader->FileVersion              =      NTOHL(FwUpgradeFileHeader->FileVersion);      
  FwUpgradeFileHeader->FilesSubVersion          =      NTOHL(FwUpgradeFileHeader->FilesSubVersion);  
  FwUpgradeFileHeader->HwVersion                =      NTOHL(FwUpgradeFileHeader->HwVersion);        
  FwUpgradeFileHeader->HwSubVersion             =      NTOHL(FwUpgradeFileHeader->HwSubVersion);     
  
}

void  fwupgradehunkReqHeaderOrderFix(fwupgradeChunkReqDataStruct_T* FwUpgradeChunkReqHeaderPtr)
{
  FwUpgradeChunkReqHeaderPtr->ChunkLength              =      NTOHL(FwUpgradeChunkReqHeaderPtr->ChunkLength); 
  FwUpgradeChunkReqHeaderPtr->FileOffset               =      NTOHL(FwUpgradeChunkReqHeaderPtr->FileOffset);   
}



void fwupgradeStateReset()
{
  fwupgradeState = FWUPGRADE_STATE_IDLE;
  fwupgradeChunkRetry = 0;
}


uint8_t fwupgradeFirstTimeFlagStateGet()
{
  spiflashBlockingRead(SPIFLASH_FWUPGRADECONTROL_START, sizeof(FwUpgradeControlBlockInNvm), (uint8_t*)&FwUpgradeControlBlockInNvm);
  // Check if CRC of the version control block is valid to determine the first time flag parameter
  if(crc32BuffCalc((uint8_t*)&FwUpgradeControlBlockInNvm, 0, sizeof(fwupgradeControlBlock_T)) == FwUpgradeControlBlockInNvm.Crc32)
    return(FwUpgradeControlBlockInNvm.VersionControlBlock.Info.FirstTimeFlag);
  else
    return 0;
}


void   fwupgradeFirstTimeFlagSet(uint8_t FirstTimeFlag)
{
  FwUpgradeControlBlockInNvm.VersionControlBlock.Info.FirstTimeFlag = FirstTimeFlag;
  FwUpgradeControlBlockInNvm.Crc32 = crc32BuffCalc((uint8_t*)&FwUpgradeControlBlockInNvm, 0, sizeof(fwupgradeControlBlock_T));
  spiflashBlockingWrite(SPIFLASH_FWUPGRADECONTROL_START, sizeof(FwUpgradeControlBlockInNvm), (uint8_t*)&FwUpgradeControlBlockInNvm);
}

/******************************************************************************
* @brief  void fwupgradeSpiCompleteCallback()
* @param  
* @retval 
******************************************************************************/
void fwupgradeSpiCompleteCallback()
{
  fwupgradeEventSend(FWUPGRADE_SPI_OPERATION_COMPLETE, 0, 0);
}


/******************************************************************************
* @brief void fwupgradeSpiFugCrcCheckStart(uint32_t SpiAddress, FwupgradeSpiFugCrcResetT ResetRequest, void(*CompletionCallBackPtr)(int8_t Success))
* @param  
* @retval 
******************************************************************************/
void fwupgradeSpiFugCrcCheckStart(uint32_t SpiAddress, FwupgradeSpiFugCrcResetT ResetRequest, void(*CompletionCallBackPtr)(int8_t Success))
{
  if(((SpiAddress == SPIFLASH_UPGRADE_START) || (SpiAddress == SPIFLASH_VANILLA_START)) && CompletionCallBackPtr && (FugCrcCheckState == FWUPGRADE_FUG_CRC32_CHECK_STATE_IDLE))
  {
    // Register the callback pointer
    fwupgradeSpiFugCrcCheckCallBackPtr = CompletionCallBackPtr;
    SpiFlashAddressStart = CurrentSpiFlashAddress = SpiAddress; 
    // Register the reset request at the end of the CRC check
    CrcCheckResetRequest = ResetRequest;
    fwupgradeSpiFugCrcCheck(FWUPGRADE_FUG_CRC32_CHECK_OPCODE_STIMULI_START);
  }
}



/******************************************************************************
* @brief void fwupgradeSpiFugCrcCheck(FwupgradeStateSpiFugCrcStimuliT Stimuli)
* @param  
* @retval 
******************************************************************************/
void fwupgradeSpiFugCrcCheck(FwupgradeStateSpiFugCrcStimuliT Stimuli)
{
  switch(FugCrcCheckState)
  {
  case FWUPGRADE_FUG_CRC32_CHECK_STATE_IDLE:
    switch(Stimuli)
    {
    case FWUPGRADE_FUG_CRC32_CHECK_OPCODE_STIMULI_START:
      // Check the SpiFlash address is the downloaded image or the Vanilla image
      if(((SpiFlashAddressStart == SPIFLASH_UPGRADE_START) || (SpiFlashAddressStart == SPIFLASH_VANILLA_START)) && fwupgradeSpiFugCrcCheckCallBackPtr)
      {
        // Allocate timer to manage the process timeouts
        fwupgradeSpiFugCrcCheckTimerHandler = xTimerCreate("fwupgradeCrcCheckTimer",  portTICK_PERIOD_MS, pdFALSE, (void *)0, fwupgradeSpiFugCrcCheckTimeoutCallback);
        if(fwupgradeSpiFugCrcCheckTimerHandler)
        {
          vlapmainDebugLog("Start CRC32 calculation");
          fwupgradeSpiFugCrcCheckHeaderPtr = (fwupgradeFileHeader_T*)pvPortMalloc(sizeof(fwupgradeFileHeader_T));
          if(fwupgradeSpiFugCrcCheckHeaderPtr)
          {
            spiflashReqEnqueue(SPIFLASH_CMD_READ, CurrentSpiFlashAddress, (uint8_t*)fwupgradeSpiFugCrcCheckHeaderPtr, 0,sizeof(fwupgradeFileHeader_T), &fwupgradeSpiFugCrcCheckSpiCompleteCallBack, 0);
            FWUPGRADE_FUG_CRC_CHECK_FSM_STATE_CHANGE_WITH_TIMEOUT(FWUPGRADE_FUG_CRC32_CHECK_STATE_HEADER_READ_WAIT, 100);  
          }
          else
          {
            // We could could not allocate memory for the header
            fwupgradeSpiFugCrcCheckCallBackPtr(-1);
          }
        }
        else
        {
          // We could could not allocate timer, Call the callback with false response
          fwupgradeSpiFugCrcCheckCallBackPtr(-2);
        }
      }
      else
      {
        // Illegal SpiFlash address
        fwupgradeSpiFugCrcCheckCallBackPtr(-3);
      }
      break;
    default:     
      // Illegal Request
      fwupgradeSpiFugCrcCheckCallBackPtr(-4);
      break;
    }
    break;
    
  case FWUPGRADE_FUG_CRC32_CHECK_STATE_HEADER_READ_WAIT:
    switch(Stimuli)
    {
    case FWUPGRADE_FUG_CRC32_CHECK_OPCODE_STIMULI_SPI_COMPLETE:
      // Allocate timer to manage the process timeouts
      //vlapmainDebugLog("Start CRC32 calculation");
      fwupgradeFileHeaderOrderFix(fwupgradeSpiFugCrcCheckHeaderPtr);
      SpiFlashAddressStart = CurrentSpiFlashAddress = SpiFlashAddressStart+sizeof(fwupgradeFileHeader_T); 
      Crc32Accumulator = crc32Init();
      memset(CurrentAllocateSpiBufferPtr, 0, FWUPGRADE_FWUPGRADE_CHUNK_LENGTH);
      spiflashReqEnqueue(SPIFLASH_CMD_READ, CurrentSpiFlashAddress, CurrentAllocateSpiBufferPtr, 0,FWUPGRADE_FWUPGRADE_CHUNK_LENGTH, &fwupgradeSpiFugCrcCheckSpiCompleteCallBack, 0);
      FWUPGRADE_FUG_CRC_CHECK_FSM_STATE_CHANGE_WITH_TIMEOUT(FWUPGRADE_FUG_CRC32_CHECK_STATE_SPIREAD_WAIT, 100);  
      break;
    default:
      // Illegal Request
      fwupgradeSpiFugCrcCheckCallBackPtr(-4);
      break;
    }
    break;
    
  case FWUPGRADE_FUG_CRC32_CHECK_STATE_SPIREAD_WAIT:
    switch (Stimuli)
    {
    case FWUPGRADE_FUG_CRC32_CHECK_OPCODE_STIMULI_SPI_COMPLETE:
      // Next chunk
      //vlapmainDebugLog("Next Chunk");
      Crc32Accumulator =  crc32BuffAccumulate(Crc32Accumulator, CurrentAllocateSpiBufferPtr, 0, FWUPGRADE_FWUPGRADE_CHUNK_LENGTH);
      TestArray[TestArrayIndex] =  Crc32Accumulator;
      TestArrayIndex++;
      CurrentSpiFlashAddress += FWUPGRADE_FWUPGRADE_CHUNK_LENGTH;
      LaskChunkLength = fwupgradeSpiFugCrcCheckHeaderPtr->FileImageLength - (CurrentSpiFlashAddress - SpiFlashAddressStart);
      if(LaskChunkLength > FWUPGRADE_FWUPGRADE_CHUNK_LENGTH)
      {
        memset(CurrentAllocateSpiBufferPtr, 0, FWUPGRADE_FWUPGRADE_CHUNK_LENGTH);
        spiflashReqEnqueue(SPIFLASH_CMD_READ, CurrentSpiFlashAddress, CurrentAllocateSpiBufferPtr, 0,FWUPGRADE_FWUPGRADE_CHUNK_LENGTH, &fwupgradeSpiFugCrcCheckSpiCompleteCallBack, 0);
        FWUPGRADE_FUG_CRC_CHECK_FSM_STATE_CHANGE_WITH_TIMEOUT(FWUPGRADE_FUG_CRC32_CHECK_STATE_SPIREAD_WAIT, 300);  
      }
      else
      {
        memset(CurrentAllocateSpiBufferPtr, 0, FWUPGRADE_FWUPGRADE_CHUNK_LENGTH);
        spiflashReqEnqueue(SPIFLASH_CMD_READ, CurrentSpiFlashAddress, CurrentAllocateSpiBufferPtr, 0, LaskChunkLength, &fwupgradeSpiFugCrcCheckSpiCompleteCallBack, 0);
        FWUPGRADE_FUG_CRC_CHECK_FSM_STATE_CHANGE_WITH_TIMEOUT(FWUPGRADE_STATE_SPI_FLASH_CRC32_CALC_LAST_WAIT, 300);  
      }
      break;
    default:
      break;
    }
    break;
    
    
  case FWUPGRADE_STATE_SPI_FLASH_CRC32_CALC_LAST_WAIT:
    switch (Stimuli)
    {
    case FWUPGRADE_FUG_CRC32_CHECK_OPCODE_STIMULI_SPI_COMPLETE:
      TestArray[TestArrayIndex] =  Crc32Accumulator;
      TestArrayIndex++;
      // Last chunk received, Verify the CRC32 
      Crc32Accumulator =  crc32BuffAccumulate(Crc32Accumulator, CurrentAllocateSpiBufferPtr, 0, LaskChunkLength);
      TestArray[TestArrayIndex] =  Crc32Accumulator;
      TestArrayIndex++;
      if(~Crc32Accumulator == fwupgradeSpiFugCrcCheckHeaderPtr->FileImageCrc32)
      {
        // Free header Ptr
        vPortFree(fwupgradeSpiFugCrcCheckHeaderPtr);
        // If there is no need for reset - call the callback function and return to idle
        if(CrcCheckResetRequest == FWUPGRADE_FUG_CRC32_CHECK_WITHOUT_RESET)
        {
          // Free the timer heandler
          xTimerDelete(fwupgradeSpiFugCrcCheckTimerHandler, 10);
          fwupgradeSpiFugCrcCheckCallBackPtr(1);
        }
        else if(CrcCheckResetRequest == FWUPGRADE_FUG_CRC32_CHECK_RESET)
        {
          vlapmainDebugLog("FUG CRC32 OK");
          fwupgradeSpiFugCrcCheckCallBackPtr(1);
          // Mark the bootloader state as pending for boot 
          FwUpgradeControlBlockInNvm.VersionControlBlock.SystemUpgradeState = FWUPGRADE_SYSTEM_STATE_UPGRADE_IN_ORDER;
          spiflashdiskReqEnqueue(SPIFLASHDISK_FSM_STIMULI_WRITE, SPIFLASH_FWUPGRADECONTROL_START, 0, (uint8_t*)&FwUpgradeControlBlockInNvm, sizeof(FwUpgradeControlBlockInNvm), &fwupgradeSpiFugCrcCheckSpiCompleteCallBack, 0);
          // Image CRC is fine, Reset and start the bootloader
          FWUPGRADE_FUG_CRC_CHECK_FSM_STATE_CHANGE_WITH_TIMEOUT(FWUPGRADE_STATE_SPI_FLASH_CRC32_FWUPGRADE_CONTROL_WRITE_WAIT, 100); 
          // Break the switch case to wait for the next completion callack
          break;
        }
      }
      else
      {
        vlapmainDebugLog("FUG CRC32 FAILED");
        // Free header Ptr
        vPortFree(fwupgradeSpiFugCrcCheckHeaderPtr);
        // Free the timer heandler
        xTimerDelete(fwupgradeSpiFugCrcCheckTimerHandler, 10);
        fwupgradeSpiFugCrcCheckCallBackPtr(0);
      }
      // Set back to idle
      FugCrcCheckState = FWUPGRADE_FUG_CRC32_CHECK_STATE_IDLE;
      break;
    default:
      break;
    }
    break; 
    
  case FWUPGRADE_STATE_SPI_FLASH_CRC32_FWUPGRADE_CONTROL_WRITE_WAIT:
    switch (Stimuli)
    {
    case FWUPGRADE_FUG_CRC32_CHECK_OPCODE_STIMULI_SPI_COMPLETE:
    	HAL_FLASH_Unlock();
	    __HAL_FLASH_CLEAR_FLAG_BANK1(FLASH_FLAG_EOP_BANK1 | FLASH_FLAG_OPERR_BANK1 | FLASH_FLAG_WRPERR_BANK1 |
	                           FLASH_FLAG_STRBERR_BANK1 | FLASH_FLAG_INCERR_BANK1 | FLASH_FLAG_PGSERR_BANK1);

    	 FLASH_WaitForLastOperation(1000, FLASH_BANK_1);
    	//FLASH_EraseSector(FLASH_Sector_1, VoltageRange_3);
    	NVIC_SystemReset();
      break;
    default:
      break;
    }
    break;
  }
}

/******************************************************************************
* @brief void fwupgradeSpiFugCrcCheckTimeoutCallback()
* @param  
* @retval 
******************************************************************************/
void fwupgradeSpiFugCrcCheckTimeoutCallback()
{
  fwupgradeSpiFugCrcCheck(FWUPGRADE_FUG_CRC32_CHECK_OPCODE_STIMULI_TIMOUT);
}

/******************************************************************************
* @brief void fwupgradeSpiFugCrcCheckCompletionCallBack()
* @param  
* @retval 
******************************************************************************/
void fwupgradeSpiFugCrcCheckSpiCompleteCallBack()
{
  fwupgradeSpiFugCrcCheck(FWUPGRADE_FUG_CRC32_CHECK_OPCODE_STIMULI_SPI_COMPLETE);
}

/******************************************************************************
* @brief void fwupgradeSpiFugCrcCheckCompletionCallBack()
* @param  
* @retval 
******************************************************************************/
void fwupgradeSpiFugCrcCheckCompletionCallBack(int8_t CompletionStatus)
{
  
  // TODO: Add enum for return error codes
  if(CompletionStatus == 1)
    fwupgradeEventSend(FWUPGRADE_OPCODE_FUG_CRC32_OK, 0, 0);
  else
    fwupgradeEventSend(FWUPGRADE_OPCODE_FUG_CRC32_FAILED, 0, 0);
}

/******************************************************************************
* @brief fwupgradeState_T fwupgradeStateGet()
* @param  
* @retval 
******************************************************************************/
fwupgradeState_T fwupgradeStateGet()
{
  return fwupgradeState;
}

/******************************************************************************
* @brief fwupgradeState_T fwupgradeBootloaderVersionGet()
* @param  
* @retval 
******************************************************************************/
uint32_t fwupgradeBootloaderVersionGet()
{
  return FwUpgradeControlBlockInNvm.VersionControlBlock.Info.BootloaderVersion;
}
