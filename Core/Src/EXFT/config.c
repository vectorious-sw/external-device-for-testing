#include "stm32h7xx.h"
#include <stdlib.h>
#include <math.h>
#include <arm_math.h>
#include <hwdrivers.h>
#include "FreeRTOS.h"
#include "task.h"
#include "vlapConfig.h"
#include "queue.h"
#include "timers.h"
#include "audit.h"
#include "iir.h"
#include "configPrefs.h"
#include "crc32.h"
#include "config.h"
#include "configdefaults.h"
#include "vlapmain.h"

volatile const configNvramProductionDb_t configNvramProductionFlashDb __attribute__( ( section( ".productionDBSection") ) );

// G L O B A L S 
QueueHandle_t configRequestQueueHandle;
TimerHandle_t ConfigFsmTimerHandler;

configConfigurationDb_t configConfigurationDb;
configProductionDb_t configProductionDb;
SemaphoreHandle_t configConfigurationValidSemaphoreHandle;
configFsmState_T configFsmState;
//uint8_t * GlobalPtr;

// L O C A L   P R O T O T Y P E S 
void configFsmTimerTimeoutCallback();
void configSpiCompletionCallBack();
void configSpiConfigUpdateCompletionWriteCallBack();
void configSpiConfigUpdateCompletionReadCallBack();
void configSpiCompletionReadCallBack();
void configSpiCompletionWriteCallBack();
ReturnCode_T configEraseMcuFlashSector(uint32_t Sector);


/******************************************************************************
* @brief  ReturnCode_T configInit(uint8_t FirstTimeAfterJlinkProgramming)
* @param  
* @retval 
******************************************************************************/
ReturnCode_T configInit(uint8_t FirstTimeAfterJlinkProgramming)
{
  // Create input queue  
  configRequestQueueHandle = xQueueCreate(5, sizeof(configFsmQueueEntryType_T));
  // Create the task
  xTaskCreate(configTask, configTaskName, configTaskSTACK_SIZE, NULL,  configTaskPriority, ( TaskHandle_t * ) NULL );
  // Create timer.
  ConfigFsmTimerHandler =  xTimerCreate("configTimer",  portTICK_PERIOD_MS, pdFALSE, (void *)0, configFsmTimerTimeoutCallback);

  
  // Creates binary semaphore to flag other depended activities that the configuration data struture is not ready for use
  configConfigurationValidSemaphoreHandle = xSemaphoreCreateBinary();
  
  // Copies the production Db from the flash to RAM
  configCopyProductionDbFromFlash();
  
  //aescbcTest();

  configFsmState = CONFIG_STATE_APP_CONFIG_IDLE;



  // Initial task state
  configTaskEventSend(CONFIG_FSM_QUEUE_ENTRY_TYPE_NORMAL_CONFIG);



  //memcpy(&configConfigurationDb, (uint8_t*) &configDefaultConfigStruct, sizeof(configConfigurationDb_t));


  // Allways return OK
  return(RETURNCODE_OK);
}



uint8_t *ConfigConfigTempBuffPtr;
uint8_t *ConfigConfigEncryptedTempBuffPtr;
/******************************************************************************
* @brief  portTASK_FUNCTION(configTask, pvParameters )
* @param  configTask, pvParameters
* @retval 
******************************************************************************/
portTASK_FUNCTION(configTask, pvParameters )
{
  configFsmQueueEntry_T QueueEntry;
  uint32_t Crc32;
  
  while(1)
  {
    // block task till new queue entry is received or timeout 
    BaseType_t  QueueState = xQueueReceive(configRequestQueueHandle, &QueueEntry, 10000);
    // Check if there is new message.
    if(QueueState == pdTRUE)
    {
      switch(configFsmState)
      {
      case CONFIG_STATE_APP_CONFIG_IDLE:
        ConfigConfigEncryptedTempBuffPtr = pvPortMalloc(sizeof(configNvramConfigurationEncryptedDb_t));
        if(!ConfigConfigEncryptedTempBuffPtr)
          break;
        switch (QueueEntry.EntryQueueType)
        {
          case CONFIG_FSM_QUEUE_ENTRY_TYPE_AFTER_JLINK_PROGRAMMING:
            vlapmainDebugLog("Configuration CRC failed, load defaults");
            if(ConfigConfigEncryptedTempBuffPtr)
            {
              // Copy the applicative default values to the local buffer
              memcpy((uint8_t*)&((configNvramConfigurationEncryptedDb_t*)ConfigConfigEncryptedTempBuffPtr)->NvramConfigDb.NvramConfigImage, (uint8_t*) &configDefaultConfigStruct.NvramConfigImage, sizeof(configConfigurationDb_t));
              // Calculate CRC32 for the default and write into the local buffer
              Crc32 =  crc32BuffCalc((uint8_t*)&((configNvramConfigurationEncryptedDb_t*)ConfigConfigEncryptedTempBuffPtr)->NvramConfigDb.NvramConfigImage, 0, sizeof(configConfigurationDb_t));
              ((configNvramConfigurationEncryptedDb_t*)ConfigConfigEncryptedTempBuffPtr)->NvramConfigDb.Crc32 = Crc32;
              // Encrypt the new config sturct
              aescbcEncryptConfig((configNvramConfigurationEncryptedDb_t *)ConfigConfigEncryptedTempBuffPtr);
              Crc32 =  crc32BuffCalc((uint8_t*)((configNvramConfigurationEncryptedDb_t*)ConfigConfigEncryptedTempBuffPtr), 0, sizeof(configNvramConfigurationEncryptedDb_t) - 4);
              // Calculate the new CRC for the encrypted config sturct
              ((configNvramConfigurationEncryptedDb_t*)ConfigConfigEncryptedTempBuffPtr)->Crc32 = Crc32;
              // write request without callback as we want to set also the production config defaults
//              spiflashReqEnqueue(SPIFLASH_CMD_PROG_VIA_BUFF_1, SPIFLASH_CONFIG_START, 0, (uint8_t*)ConfigConfigEncryptedTempBuffPtr, sizeof(configNvramConfigurationEncryptedDb_t), configSpiConfigUpdateCompletionWriteCallBack, false);
              CONFIG_FSM_STATE_CHANGE(CONFIG_STATE_APP_CONFIG_FIRST_TIME_AFTER_JLINK_PROG, 1000);
            }
              break;
          case CONFIG_FSM_QUEUE_ENTRY_TYPE_NORMAL_CONFIG:

//              spiflashReqEnqueue(SPIFLASH_CMD_READ, SPIFLASH_CONFIG_START, (uint8_t*)ConfigConfigEncryptedTempBuffPtr, 0, sizeof(configNvramConfigurationEncryptedDb_t), configSpiCompletionReadCallBack, false);
              CONFIG_FSM_STATE_CHANGE(CONFIG_STATE_APP_CONFIG_SPI_READ_WAIT, 1000);
              break;
        default:
              break;
        }
        break;
        
      case CONFIG_STATE_APP_CONFIG_FIRST_TIME_AFTER_JLINK_PROG:
        switch (QueueEntry.EntryQueueType)
        {
        case CONFIG_FSM_QUEUE_ENTRY_TYPE_SPIFLASH_WRITE_COMPLETED:
//          spiflashReqEnqueue(SPIFLASH_CMD_READ, SPIFLASH_CONFIG_START, (uint8_t*)ConfigConfigEncryptedTempBuffPtr, 0, sizeof(configNvramConfigurationEncryptedDb_t), configSpiCompletionReadCallBack, false);
          CONFIG_FSM_STATE_CHANGE(CONFIG_STATE_APP_CONFIG_SPI_READ_WAIT, 1000);
          break;
        case CONFIG_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:
        default:
          CONFIG_FSM_STATE_CHANGE(CONFIG_STATE_NOT_READY, 10);
          break;
        }
        break;

        
      case CONFIG_STATE_APP_CONFIG_SPI_READ_WAIT:
        switch (QueueEntry.EntryQueueType)
        {
        case CONFIG_FSM_QUEUE_ENTRY_TYPE_SPIFLASH_READ_COMPLETED:
          Crc32 = crc32BuffCalc((uint8_t*)((configNvramConfigurationEncryptedDb_t*)ConfigConfigEncryptedTempBuffPtr), 0, sizeof(configNvramConfigurationEncryptedDb_t) - 4);
          // Check if the encryption struct is valid
          if(Crc32 == (((configNvramConfigurationEncryptedDb_t*)ConfigConfigEncryptedTempBuffPtr)->Crc32))
          {
            // Decrypt it
            aescbcDecryptConfig((configNvramConfigurationEncryptedDb_t *)ConfigConfigEncryptedTempBuffPtr);
            Crc32 = crc32BuffCalc((uint8_t*)&((configNvramConfigurationEncryptedDb_t*)ConfigConfigEncryptedTempBuffPtr)->NvramConfigDb.NvramConfigImage, 0, sizeof(configConfigurationDb_t));
            // Check that the CRC of the configuration Db itself is valid after decryption
            if(Crc32 == ((configNvramConfigurationEncryptedDb_t*)ConfigConfigEncryptedTempBuffPtr)->NvramConfigDb.Crc32)
            {
              memcpy(&configConfigurationDb, (uint8_t*)&((configNvramConfigurationEncryptedDb_t*)ConfigConfigEncryptedTempBuffPtr)->NvramConfigDb.NvramConfigImage, sizeof(configConfigurationDb_t));


              // The "Semaphore Give" will release all the processes waiting for configuration to be ready
              // In our case the processes are: auditTask, measurementTask, commTask
              xSemaphoreGive(configConfigurationValidSemaphoreHandle);
              CONFIG_FSM_STATE_CHANGE(CONFIG_STATE_READY, 10);
            }
            else
            {
              CONFIG_FSM_STATE_CHANGE(CONFIG_STATE_APP_CONFIG_IDLE, 10);
              configTaskEventSend(CONFIG_FSM_QUEUE_ENTRY_TYPE_AFTER_JLINK_PROGRAMMING);
              //CONFIG_FSM_STATE_CHANGE(CONFIG_STATE_NOT_READY, 10);
            }
          }
          else
          {
            CONFIG_FSM_STATE_CHANGE(CONFIG_STATE_APP_CONFIG_IDLE, 10);
            configTaskEventSend(CONFIG_FSM_QUEUE_ENTRY_TYPE_AFTER_JLINK_PROGRAMMING);
            //CONFIG_FSM_STATE_CHANGE(CONFIG_STATE_NOT_READY, 10);
          }
          if(ConfigConfigEncryptedTempBuffPtr)
            vPortFree(ConfigConfigEncryptedTempBuffPtr);
          break;
        case CONFIG_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:
        case CONFIG_FSM_QUEUE_ENTRY_TYPE_APPLICATION_CONFIG_UPDATE:
        default:
            vPortFree(ConfigConfigEncryptedTempBuffPtr);
            CONFIG_FSM_STATE_CHANGE(CONFIG_STATE_NOT_READY, 10);
          break;
        }
        break;

      case CONFIG_STATE_READY:
        switch (QueueEntry.EntryQueueType)
        {
        case CONFIG_FSM_QUEUE_ENTRY_TYPE_APPLICATION_CONFIG_UPDATE:
          // Encrypt the configuration
          aescbcEncryptConfig((configNvramConfigurationEncryptedDb_t *)ConfigConfigEncryptedTempBuffPtr);
          // Calculate the CRC32 config sturct(without the last 4 bytes of CRC itself)
          Crc32 =  crc32BuffCalc((uint8_t*)(((configNvramConfigurationEncryptedDb_t*)ConfigConfigEncryptedTempBuffPtr)), 0, sizeof(configNvramConfigurationEncryptedDb_t) - 4);
          ((configNvramConfigurationEncryptedDb_t*)ConfigConfigEncryptedTempBuffPtr)->Crc32 = Crc32;
          // write request without callback as we want to set also the production config defaults
//          spiflashReqEnqueue(SPIFLASH_CMD_PROG_VIA_BUFF_1, SPIFLASH_CONFIG_START, 0, (uint8_t*)ConfigConfigEncryptedTempBuffPtr, sizeof(configNvramConfigurationEncryptedDb_t), configSpiConfigUpdateCompletionWriteCallBack, false);
          CONFIG_FSM_STATE_CHANGE(CONFIG_STATE_APP_CONFIG_UPDATE_SPI_WRITE_WAIT, 10);
          break;
        case CONFIG_FSM_QUEUE_ENTRY_TYPE_SPIFLASH_READ_COMPLETED:
          break;
        case CONFIG_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:
          break;
        default:
        	break;
        }
        break;
                
      case CONFIG_STATE_APP_CONFIG_UPDATE_SPI_WRITE_WAIT:
        switch (QueueEntry.EntryQueueType)
        {
        case CONFIG_FSM_QUEUE_ENTRY_TYPE_SPIFLASH_WRITE_COMPLETED:
//            spiflashReqEnqueue(SPIFLASH_CMD_READ, SPIFLASH_CONFIG_START, (uint8_t*)ConfigConfigEncryptedTempBuffPtr, 0, sizeof(configNvramConfigurationEncryptedDb_t), configSpiConfigUpdateCompletionReadCallBack, false);
            CONFIG_FSM_STATE_CHANGE(CONFIG_STATE_APP_CONFIG_UPDATE_SPI_READ_WAIT, 0);
          break;
        case CONFIG_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:
          break;
        case CONFIG_FSM_QUEUE_ENTRY_TYPE_APPLICATION_CONFIG_UPDATE:
        default:
          break;
        }
        break;
        

      case CONFIG_STATE_APP_CONFIG_UPDATE_SPI_READ_WAIT:
        switch (QueueEntry.EntryQueueType)
        {
        case CONFIG_FSM_QUEUE_ENTRY_TYPE_SPIFLASH_READ_COMPLETED:
          Crc32 =  crc32BuffCalc((uint8_t*)(((configNvramConfigurationEncryptedDb_t*)ConfigConfigEncryptedTempBuffPtr)), 0, sizeof(configNvramConfigurationEncryptedDb_t) - 4);
          if(Crc32 == (((configNvramConfigurationEncryptedDb_t*)ConfigConfigEncryptedTempBuffPtr)->Crc32))
          {
            memcpy(&configConfigurationDb, (uint8_t*)ConfigConfigTempBuffPtr, sizeof(configConfigurationDb_t));
            CONFIG_FSM_STATE_CHANGE(CONFIG_STATE_READY, 10);
            // Notify comm that config is set
//            commTaskEventSend(COMM_FSM_QUEUE_ENTRY_TYPE_CONFIG_SET_DONE);
          }
          else
          {
            CONFIG_FSM_STATE_CHANGE(CONFIG_STATE_NOT_READY, 10);
          }
          if(ConfigConfigEncryptedTempBuffPtr)
            vPortFree(ConfigConfigEncryptedTempBuffPtr);
          if(ConfigConfigTempBuffPtr)
            vPortFree(ConfigConfigTempBuffPtr);
          break;
        case CONFIG_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:
          break;
        case CONFIG_FSM_QUEUE_ENTRY_TYPE_APPLICATION_CONFIG_UPDATE:
        default:
          break;
        }
        break;


        
      case CONFIG_STATE_NOT_READY:
        switch (QueueEntry.EntryQueueType)
        {
        case CONFIG_FSM_QUEUE_ENTRY_TYPE_APPLICATION_CONFIG_UPDATE:
          break;
        case CONFIG_FSM_QUEUE_ENTRY_TYPE_SPIFLASH_READ_COMPLETED:
          break;
        case CONFIG_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:
          break;
        default:
        	break;
        }
        break;

        case SPI_READ_WAIT:
        	break;
      }
    }
  
  }
}



/******************************************************************************
* @brief  void  configFsmTimerTimeoutCallback()
* @param  
* @retval 
******************************************************************************/
void configFsmTimerTimeoutCallback( )
{
  configTaskEventSend(CONFIG_FSM_QUEUE_ENTRY_TYPE_TIMEOUT);
}


/******************************************************************************
* @brief ReturnCode_T configSpiCompletionReadCallBack()
* @param  
* @retval 
******************************************************************************/
void configSpiCompletionReadCallBack()
{
   configFsmQueueEntry_T QueueEntry;
  
   QueueEntry.EntryQueueType = CONFIG_FSM_QUEUE_ENTRY_TYPE_SPIFLASH_READ_COMPLETED;
  
  xQueueSend(configRequestQueueHandle, &QueueEntry, 0);
}

/******************************************************************************
* @brief ReturnCode_T configSpiCompletionWriteCallBack()
* @param  
* @retval 
******************************************************************************/
void configSpiCompletionWriteCallBack()
{
   configFsmQueueEntry_T QueueEntry;
  
   QueueEntry.EntryQueueType = CONFIG_FSM_QUEUE_ENTRY_TYPE_SPIFLASH_WRITE_COMPLETED;
  
  xQueueSend(configRequestQueueHandle, &QueueEntry, 0);
}



/******************************************************************************
* @brief ReturnCode_T configSpiConfigUpdateCompletionWriteCallBack()
* @param  
* @retval 
******************************************************************************/
void configSpiConfigUpdateCompletionWriteCallBack()
{
   configFsmQueueEntry_T QueueEntry;
  
   QueueEntry.EntryQueueType = CONFIG_FSM_QUEUE_ENTRY_TYPE_SPIFLASH_WRITE_COMPLETED;
   xQueueSend(configRequestQueueHandle, &QueueEntry, 0);
}


/******************************************************************************
* @brief ReturnCode_T configSpiConfigUpdateCompletionReadCallBack()
* @param  
* @retval 
******************************************************************************/
void configSpiConfigUpdateCompletionReadCallBack()
{
   configFsmQueueEntry_T QueueEntry;
  
   QueueEntry.EntryQueueType = CONFIG_FSM_QUEUE_ENTRY_TYPE_SPIFLASH_READ_COMPLETED;
   xQueueSend(configRequestQueueHandle, &QueueEntry, 0);
}






/******************************************************************************
*** ReturnCode_T configTaskEventSend(configFsmQueueEntryType_T NotificationToConfigTask )
* @brief  // Sends notifications to the configTask   
* @param  
* @retval 
******************************************************************************/
ReturnCode_T configTaskEventSend(configFsmQueueEntryType_T NotificationToConfigTask )
{
  configFsmQueueEntry_T  ConfigTaskQueueEntry;
  
  // Fill the queue entry
  ConfigTaskQueueEntry.EntryQueueType = NotificationToConfigTask;
  
  // Enqueue the event to the configTask input queue
  xQueueSend(configRequestQueueHandle, &ConfigTaskQueueEntry, 0);
  
  return(RETURNCODE_OK);
}




/******************************************************************************
* ReturnCode_T configDbGet( configConfigurationDb_t* ReturnedConfigDbPtr)
*
* @param  
* @retval 
******************************************************************************/
ReturnCode_T configDbPtrGet( configConfigurationDb_t** ReturnedConfigDbPtr)
{
  *ReturnedConfigDbPtr = &configConfigurationDb;
  
  // Return success
  return(RETURNCODE_OK);
}

  
/******************************************************************************
* uint32_t configVersionIdGet()
*
* @param  
* @retval 
******************************************************************************/
uint32_t configVersionIdGet()
{
   return(configConfigurationDb.ConfigurationVersionId);
}




/******************************************************************************
* ReturnCode_T configProductionDbOpcWrite(uint16_t Offset)
*
* @param  
* @retval 
******************************************************************************/
ReturnCode_T configProductionDbOpcWrite(uint16_t Offset)
{
  configProductionDb.PressureSensorOffsetDelta = sensorPressureOPCSet(Offset, configProductionDb.PressureSensorOffsetDelta);  
  configCopyProductionDbToFlash();
  
  eventsEventWrite(0,0,PROTOCOLAPP_GENERAL_OPC_EVENT,0,0, 0);
  
  // Return success
  return(RETURNCODE_OK);
}

             
/******************************************************************************
* ReturnCode_T configProductionDbKeysWrite()
*
* @param  
* @retval 
******************************************************************************/
ReturnCode_T configProductionDbKeysWrite(uint8_t * buffer)
{
  // Copy new keys
  memcpy(configProductionDb.UpStreamAesCbcKey, buffer, sizeof(configProductionDb.UpStreamAesCbcKey));
  memcpy(configProductionDb.DownStreamAesCbcKey, &buffer[sizeof(configProductionDb.DownStreamAesCbcKey)], sizeof(configProductionDb.DownStreamAesCbcKey));
   
  configCopyProductionDbToFlash();
  
  // Return success
  return(RETURNCODE_OK);
}

/******************************************************************************
* ReturnCode_T configApplicationConfigDbWrite()
*
* @param  
* @retval 
******************************************************************************/
ReturnCode_T configApplicationConfigDbWrite(uint8_t * ConfigPtr, uint32_t ConfigDbLength)
{
  configFsmQueueEntry_T QueueEntry;
  
  if( ConfigDbLength == sizeof(configConfigurationDb_t))
  {
    ConfigConfigTempBuffPtr = pvPortMalloc(sizeof(configConfigurationDb_t));
    ConfigConfigEncryptedTempBuffPtr = pvPortMalloc(sizeof(configNvramConfigurationEncryptedDb_t));
    if(ConfigConfigTempBuffPtr || ConfigConfigEncryptedTempBuffPtr)
    {
      // Copy the applicative default values to the local buffer
      memcpy(&((configNvramConfigurationEncryptedDb_t*)ConfigConfigEncryptedTempBuffPtr)->NvramConfigDb.NvramConfigImage, ConfigPtr, sizeof(configConfigurationDb_t));
      ((configNvramConfigurationEncryptedDb_t*)ConfigConfigEncryptedTempBuffPtr)->NvramConfigDb.Crc32 = crc32BuffCalc(ConfigPtr, 0, sizeof(configConfigurationDb_t));
      // Encrypt the config sturct
      //aescbcEncryptConfig((configNvramConfigurationEncryptedDb_t *)ConfigConfigEncryptedTempBuffPtr);
      
      memcpy(((configConfigurationDb_t*)ConfigConfigTempBuffPtr), ConfigPtr, sizeof(configConfigurationDb_t));

      QueueEntry.EntryQueueType = CONFIG_FSM_QUEUE_ENTRY_TYPE_APPLICATION_CONFIG_UPDATE;
      QueueEntry.ConfigurationDbPtr = ConfigConfigTempBuffPtr;
      QueueEntry.ConfigurationDbLength = sizeof(configNvramConfigurationDb_t);
      
      xQueueSend(configRequestQueueHandle, &QueueEntry, 0);
    }
  }
  
  
  // Return success
  return(RETURNCODE_OK);

}

/******************************************************************************
* ReturnCode_T configCopyProductionDbToFlash()
*
* @param  
* @retval 
******************************************************************************/           
ReturnCode_T configCopyProductionDbToFlash()
{
  uint8_t i = 0;
  uint8_t numWords = (uint8_t) (sizeof(configProductionDb_t) / sizeof(uint32_t));
  uint8_t remainingBytes = sizeof(configProductionDb_t) % sizeof(uint32_t);
    
  
  /* TODO: 
          *Check that clearing the flags is good (bank1 only)
          *Flash programming can only be done in word size
  */ 
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG_BANK1(FLASH_FLAG_EOP_BANK1 | FLASH_FLAG_OPERR_BANK1 | FLASH_FLAG_WRPERR_BANK1 |
	                           FLASH_FLAG_STRBERR_BANK1 | FLASH_FLAG_INCERR_BANK1 | FLASH_FLAG_PGSERR_BANK1);

	configEraseMcuFlashSector(FLASH_SECTOR_2);
//	FLASH_EraseSector(FLASH_Sector_2);

  // Copy procedure
  for(i = 0; i < numWords; i++)
  {
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, (uint32_t)((uint32_t *) &configNvramProductionFlashDb + i), (uint32_t)*((uint32_t *) &configProductionDb + i));
    //FLASH_ProgramWord((uint32_t)((uint32_t *) &configNvramProductionFlashDb + i), (uint32_t)*((uint32_t *) &configProductionDb + i));
  }
  // Check if there is more bytes to copy 
  if(remainingBytes)
  {
    for(i = 0; i < remainingBytes; i++)
    {
      // If so - copy the next word regardless to the redundant last bytes that will be copied
    	HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, (uint32_t)((uint8_t *) &configNvramProductionFlashDb + (numWords * 4) + i), (uint8_t)*((uint8_t *) &configProductionDb + (numWords * 4) + i));
        //FLASH_ProgramByte((uint32_t)((uint8_t *) &configNvramProductionFlashDb + (numWords * 4) + i), (uint8_t)*((uint8_t *) &configProductionDb + (numWords * 4) + i));
    }
  }
  // Calculate the CRC32 of the temp production Db
  uint32_t crc32TempDb = crc32BuffCalc((uint8_t*)&configProductionDb, 0, sizeof(configProductionDb_t));
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, (uint32_t)((uint8_t *) &configNvramProductionFlashDb.Crc32), crc32TempDb & 0xff);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, (uint32_t)((uint8_t *) &configNvramProductionFlashDb.Crc32 + 1), (crc32TempDb >> 8) & 0xff);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, (uint32_t)((uint8_t *) &configNvramProductionFlashDb.Crc32 + 2), (crc32TempDb >> 16) & 0xff);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, (uint32_t)((uint8_t *) &configNvramProductionFlashDb.Crc32 + 3), (crc32TempDb >> 24) & 0xff);
  //FLASH_ProgramWord((uint32_t)&configNvramProductionFlashDb.Crc32, crc32TempDb);
  
  // Lock the flash back
  HAL_FLASH_Lock();
  
  // Return success
  return(RETURNCODE_OK);
}

/******************************************************************************
* ReturnCode_T configCopyProductionDbToFlash()
*
* @param  
* @retval 
******************************************************************************/    
ReturnCode_T configCopyProductionDbFromFlash()
{
	ReturnCode_T MyReturn = RETURNCODE_ERROR;


  // Calculate the CRC32 from the flash production Db
  uint32_t crc32TempDb = crc32BuffCalc((uint8_t*)&configNvramProductionFlashDb.NvramProductionImage, 0, sizeof(configProductionDb_t));

  if(configNvramProductionFlashDb.Crc32 == crc32TempDb)
  {
//    configProductionDb = configNvramProductionFlashDb.NvramProductionImage;
    memcpy(&configProductionDb, &configNvramProductionFlashDb.NvramProductionImage, sizeof(configProductionDb_t) );
    // Return success
    MyReturn = RETURNCODE_OK;
  }
  else
  {
	vlapmainDebugLog("Production Db CRC failed from flash, load defaults");
	// There was a problem with the CRC - return error
	MyReturn = RETURNCODE_ERROR;
  }

  return (MyReturn);
}




ReturnCode_T configEraseMcuFlashSector(uint32_t Sector)
{
	FLASH_EraseInitTypeDef pEraseInit;
	uint32_t SectorError;

	pEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
	pEraseInit.Banks = 0;
	pEraseInit.Sector = Sector;
	pEraseInit.NbSectors = 1;
	pEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	HAL_FLASHEx_Erase(&pEraseInit, &SectorError);

	return(RETURNCODE_OK);
}
