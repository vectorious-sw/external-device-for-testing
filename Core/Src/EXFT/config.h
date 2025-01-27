#pragma once

#include "semphr.h"
#include "common.h"

// G L O B A L   T Y P E S   A N D    D E F I N I T I O N S 

typedef enum {CONFIG_STATE_APP_CONFIG_IDLE, CONFIG_STATE_APP_CONFIG_FIRST_TIME_AFTER_JLINK_PROG, SPI_READ_WAIT, CONFIG_STATE_APP_CONFIG_SPI_READ_WAIT, CONFIG_STATE_NOT_READY, CONFIG_STATE_READY, CONFIG_STATE_APP_CONFIG_UPDATE_SPI_WRITE_WAIT, CONFIG_STATE_APP_CONFIG_UPDATE_SPI_READ_WAIT} configFsmState_T;
typedef enum {CONFIG_FSM_QUEUE_ENTRY_TYPE_TIMEOUT, CONFIG_FSM_QUEUE_ENTRY_TYPE_NORMAL_CONFIG, CONFIG_FSM_QUEUE_ENTRY_TYPE_AFTER_JLINK_PROGRAMMING, CONFIG_FSM_QUEUE_ENTRY_TYPE_APPLICATION_CONFIG_UPDATE, CONFIG_FSM_QUEUE_ENTRY_TYPE_SPIFLASH_READ_COMPLETED, CONFIG_FSM_QUEUE_ENTRY_TYPE_SPIFLASH_WRITE_COMPLETED } configFsmQueueEntryType_T;

typedef struct 
{
  configFsmQueueEntryType_T         EntryQueueType;
  uint32_t ConfigurationDbLength;
  uint8_t* ConfigurationDbPtr;
} configFsmQueueEntry_T;


#define         CONFIG_FSM_STATE_CHANGE(NewState, QueueWairTimeoutIn10mSec)                  \
                {                                                                               \
                configFsmState=NewState;                                                     \
                if(QueueWairTimeoutIn10mSec) xTimerChangePeriod( ConfigFsmTimerHandler, QueueWairTimeoutIn10mSec, 100);      \
                }                                                                               \

#define         CONFIG_FSM_TIMEOUT_SET(QueueWairTimeoutIn10mSec)                  \
                {                                                                               \
                  xTimerChangePeriod( ConfigFsmTimerHandler, QueueWairTimeoutIn10mSec, 100);        \
                }                                                                               \


                  
#define         CONFIG_FSM_STATE_GET configFsmState



#pragma pack(1)
typedef struct  {
uint16_t	ConfigurationType;				
uint16_t	ConfigurationVersionId;				
uint16_t	MeasurementModemLockDelay;							
uint16_t	VoltageHysteresisWidth;				
uint16_t	VbattUpperThreshold;					
uint16_t	VbattLowerThreshold;					
uint16_t	SleepModeTimer;						
uint16_t	MeasurementSequence;						
uint16_t	ManualRelayStateForSequence;					
uint32_t	MeasurementValidIcMax;				
uint32_t	MeasurementValidIcMin;				
uint32_t	MeasurementValidReffMax;				
uint32_t	MeasurementValidReffMin;				
uint32_t	MeasurementValidVmicTempMax;			
uint32_t	MeasurementValidVmicTempMin;			
uint32_t	MeasurementValidPressureCapTempMax;	
uint32_t	MeasurementValidPressureCapTempMin;	
uint32_t	MovementThreshold;					
uint16_t	MeasurementManualPslForSequence;				
uint16_t	ProtocolRetryTimeout;				
uint8_t		ProtocolRetryCount;					
uint16_t	MeasurementDelayStartMilliSeconds;						
uint8_t		BuzzerControl;				
uint8_t		vibratorControl;			
uint8_t		AutoResonaceControl;		
uint8_t		AutoPowerControl;			
uint8_t		MeasurementPslControl;				
uint8_t		PushButtonTurnOffTime;				
uint8_t		PatientNurseMode;					
uint8_t		EnableMeasurmentWhileCharging;		
uint8_t		MeasurementMinimalPslMarginPercentage;						
uint8_t		MeasurementMarginPercentage;
uint32_t	MinimalSequencePsl;		
uint32_t	MaximalSequencePsl;						
uint32_t	PslSequenceStepPercentage;
uint8_t     BeltSize;               // 0:Small, 1:Medium, 3:Larg
uint16_t    NurseModePsl;
uint8_t     EnableChargingWhilePolling;
uint8_t     EnableSleepWhilePolling;
uint32_t    MeasurementTimeoutSeconds;
} configConfigurationDb_t;





// This type represents the image in the NVM, with added checksum field
#pragma pack(1)
typedef  struct  {
configConfigurationDb_t  NvramConfigImage;
uint32_t Crc32;
} configNvramConfigurationDb_t;

#pragma pack(1)
typedef  struct  {
configNvramConfigurationDb_t  NvramConfigDb;
// We assume that there is a need for 16 padding bytes in the worst case, so
// instead of allocating it in run time, we always add it
uint8_t PaddingBytes[16];
uint8_t paddingSize;
uint8_t IvArray[16];
uint32_t Crc32;
} configNvramConfigurationEncryptedDb_t;


#pragma pack(1)
typedef  struct  {
float PressureSensorOffsetDelta;
uint8_t UpStreamAesCbcKey[32];
uint8_t DownStreamAesCbcKey[32];
uint32_t CalibrationTimeStamp;
uint16_t CalibrationSiteLocationCode;
uint32_t CalibrationTechnitionIdCode;
uint8_t UserId[70];
uint8_t BoardId[40];
uint8_t  ProductionBarCode[50];
} configProductionDb_t;


// This type represents the image in the NVM, with added checksum field
#pragma pack(1)
typedef  struct  {
configProductionDb_t  NvramProductionImage;
uint32_t Crc32;
} configNvramProductionDb_t;




extern configConfigurationDb_t configConfigurationDb;
extern configProductionDb_t configProductionDb;
extern SemaphoreHandle_t configConfigurationValidSemaphoreHandle;;









ReturnCode_T configInit(uint8_t FirstTimeAfterJlinkProgramming);
ReturnCode_T configDbGet( configConfigurationDb_t* ReturnedConfigDbPtr);
uint32_t configVersionIdGet();
ReturnCode_T configTaskEventSend(configFsmQueueEntryType_T NotificationToConfigTask);
ReturnCode_T configApplicationConfigDbWrite(uint8_t * ConfigPtr, uint32_t ConfigDbLength);
ReturnCode_T configProductionDbWrite(uint16_t Ofeset);
ReturnCode_T configCopyProductionDbToFlash();
ReturnCode_T configCopyProductionDbFromFlash();




  




    



