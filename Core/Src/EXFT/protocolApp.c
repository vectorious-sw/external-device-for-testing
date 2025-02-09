
#include <hwdrivers.h>
#include "protocolApp.h"
//#include "main.h"
#include "inet.h"
#include "rtc.h"
#include "vmicapplayer.h"
//#include "vlapConfig.h"
#include "common.h"
#include "crc32.h"


// L O C A L    D E F I N I T I O N S
//#define TELIT_SPRINTF "AT#DWSEND=0,property.publish,key,Numerator,value,%d,key,Event,value,%d\r", NTOHL(((ProtocolappHeader_t*)(ProtocoloappMsgPtr+sizeof(DllHeader_t)))->MessageNumerator), NTOHS(((ProtocolappHeader_t*)(ProtocoloappMsgPtr+sizeof(DllHeader_t)))->MessageOpCode)  
#define TELIT_SPRINTF "AT#DWSEND=0,property.publish,key,Numerator,value,%d\r", NTOHL(((ProtocolappHeader_t*)(ProtocoloappMsgPtr+sizeof(DllHeader_t)))->MessageNumerator)  
#if 0
#define TELIT_SPRINTF_BATCH "{ \"cmd\"{                 \
    \"command\": \"property.batch\",                    \
    \"params\": {                                       \
      \"thingKey\": \"HoYjGL7ysexvYeI0\",                \
      \"ts\": \"2017-02-10T19:15:05.322Z\",             \
      \"data\": [                                       \
        {                                               \
          \"key\": \"Numerator\",                        \
          \"value\": %d                                 \
        }                                        \
      ]                                                 \
    }                                                   \
}                                                       \
\r}", NTOHL(((ProtocolappHeader_t*)(ProtocoloappMsgPtr+sizeof(DllHeader_t)))->MessageNumerator)                                                    \

#endif


//#define TELIT_SPRINTF_BATCH "{ \"cmd\":{ \"command\": \"property.batch\",\"params\": { \"thingKey\": \"HoYjGL7ysexvYeI0\",\"ts\": \"2017-02-10T19:18:07.322Z\",\"data\": [{\"key\": \"Test\",\"value\": %d}  ]  } } }" , NTOHL(((ProtocolappHeader_t*)(ProtocoloappMsgPtr+sizeof(DllHeader_t)))->MessageNumerator)
#define TELIT_SPRINTF_BATCH "{ \"cmd\":{ \"command\": \"property.batch\",\"params\": { \"thingKey\": \"89972080091104723172\",\"ts\": \"2017-02-10T19:18:07.322Z\",\"data\": [{\"key\": \"Test\",\"value\": %d}  ]  } } }" , NTOHL(((ProtocolappHeader_t*)(ProtocoloappMsgPtr+sizeof(DllHeader_t)))->MessageNumerator)

// L O C A L    P R O T O T Y P E S                     
ReturnCode_T ConfigNetworkOrderFix(configConfigurationDb_t* MessagePtr, configConfigurationDb_t* MyAppHeader);
ReturnCode_T protocolAppVerifyReceivedProtocolHeader(ProtocolappHeader_t* sentProtocolHeader, ProtocolappHeader_t* receivedProtocolHeader);
// G L O B A L S
uint32_t MessageNumerator;
uint32_t UpStreamSessionID;


volatile int APCounter;
/**************************************************************************
* >>> void protocolappPowerUpInit()
*
*
*
*
***************************************************************************/
void protocolappPowerUpInit()
{
  MessageNumerator = 0;
  UpStreamSessionID = 0;
}

char protocolAppPrintBuff[55];
uint32_t testCalculatedCRC = 0;
uint32_t testReversedCRC32 = 0;
/**************************************************************************
* ReturnCode_T protocolAppVerifyReceivedProtocolHeader(ProtocolappHeader_t* sentProtocolHeader, ProtocolappHeader_t* receivedProtocolHeader)
*
*
*
***************************************************************************/
ReturnCode_T protocolAppVerifyReceivedProtocolHeader(ProtocolappHeader_t* sentProtocolHeader, ProtocolappHeader_t* receivedProtocolHeader)
{  
  // If there are events stored with previous versions which not support CRC check - skip this test
#if 1
	if(sentProtocolHeader->ProtocolVersion > 1)
  {
    // Check if sent protocol header CRC is valid
    if(crc32BuffCalc((uint8_t*) sentProtocolHeader, 0, sizeof(ProtocolappHeader_t) - 4) != NTOHL(sentProtocolHeader->CRC32))
    {
      sprintf(protocolAppPrintBuff, "Sent protocol header CRC invalid");
      vlapmainDebugLog(protocolAppPrintBuff);
      return RETURNCODE_ERROR;
    } 
  }
#endif
  // The protocol header which is sent calculates the CRC with the network order
  protocolappHeadersNetworkOrderFix(sentProtocolHeader, sentProtocolHeader);
  
  // Check if received protocol header CRC is valid, CRC is in network order
  if(crc32BuffCalc((uint8_t*) receivedProtocolHeader, 0, sizeof(ProtocolappHeader_t) - 4) != NTOHL(receivedProtocolHeader->CRC32))
  {
    sprintf(protocolAppPrintBuff, "Received protocol header CRC invalid");
    vlapmainDebugLog(protocolAppPrintBuff);
    return RETURNCODE_ERROR;
  }
  
  // Fix received protocol header network order
  protocolappHeadersNetworkOrderFix(receivedProtocolHeader, receivedProtocolHeader);
  
  // If there are events stored with previous versions which not support CRC check - skip this test
  if(sentProtocolHeader->ProtocolVersion > 1)
  {
    // Verify equal parameters
    if((sentProtocolHeader->EventTimeStamp != receivedProtocolHeader->EventTimeStamp) ||
       (sentProtocolHeader->MessageNumerator != receivedProtocolHeader->MessageNumerator))
    {
      sprintf(protocolAppPrintBuff, "Received ACK is invalid");
      vlapmainDebugLog(protocolAppPrintBuff);
      return RETURNCODE_ERROR;
    }
  }

  return RETURNCODE_OK;
}

/**************************************************************************
***
*
*
*
*
***************************************************************************/
ReturnCode_T protocolappHandlePacket(uartdllChannel_t Channel, uint8_t * MessagePtr, uint32_t MessageLength)
{
    return(RETURNCODE_OK);
}

/**************************************************************************
* >>> 
*
***************************************************************************/
ReturnCode_T SetMode(uartdllChannel_t Channel, uint8_t  data)
{
  return(RETURNCODE_ERROR);
}


/**************************************************************************
* >>> 
*
***************************************************************************/
ReturnCode_T  protocolappHeadersNetworkOrderFix(ProtocolappHeader_t* MessagePtr, ProtocolappHeader_t* MyAppHeader)
{
  MyAppHeader->TransmissionTimeStamp            = NTOHL(MessagePtr->TransmissionTimeStamp);                  
  MyAppHeader->TransmissionTimeStamp10mSec      = MessagePtr->TransmissionTimeStamp10mSec;
  MyAppHeader->EventTimeStamp                   = NTOHL(MessagePtr->EventTimeStamp);                  // Transmission TimeStamp (Number of seconds since Epoch, 1970-01-01 00:00:00 +0000 (UTC))
  MyAppHeader->EventTimeStamp10mSec             = MessagePtr->EventTimeStamp10mSec;
  MyAppHeader->MessageNumerator                 = NTOHL(MessagePtr->MessageNumerator);
  MyAppHeader->SourceAddress                    = NTOHS(MessagePtr->SourceAddress);
  MyAppHeader->DestinationAddress               = NTOHS(MessagePtr->DestinationAddress);
  MyAppHeader->ProtocolVersion                  = MessagePtr->ProtocolVersion;
  MyAppHeader->MessageOpCode                    = NTOHS(MessagePtr->MessageOpCode);
  MyAppHeader->SessionID                        = NTOHL(MessagePtr->SessionID);
  MyAppHeader->HwVersion                        = NTOHL(MessagePtr->HwVersion);
  MyAppHeader->FwVersion                        = (FirmwareVersion_T)NTOHL(MessagePtr->FwVersion.FirmwareVersionValue);
  MyAppHeader->BleFwVersion                     = NTOHL(MessagePtr->BleFwVersion);
  MyAppHeader->BootloaderFwVersion              = NTOHL(MessagePtr->BootloaderFwVersion);
  MyAppHeader->CRC32                            = NTOHL(MessagePtr->CRC32);
  return(RETURNCODE_OK);
} 

ReturnCode_T ConfigNetworkOrderFix(configConfigurationDb_t* MessagePtr, configConfigurationDb_t* MyAppHeader)
{
  MyAppHeader->ConfigurationType = NTOHS(MessagePtr->ConfigurationType);
  MyAppHeader->ConfigurationVersionId = NTOHS(MessagePtr->ConfigurationVersionId);
  MyAppHeader->MeasurementModemLockDelay = NTOHS(MessagePtr->MeasurementModemLockDelay);
  MyAppHeader->VoltageHysteresisWidth = NTOHS(MessagePtr->VoltageHysteresisWidth);
  MyAppHeader->VbattUpperThreshold = NTOHS(MessagePtr->VbattUpperThreshold);
  MyAppHeader->VbattLowerThreshold = NTOHS(MessagePtr->VbattLowerThreshold);
  MyAppHeader->SleepModeTimer = NTOHS(MessagePtr->SleepModeTimer);
  MyAppHeader->MeasurementSequence = NTOHS(MessagePtr->MeasurementSequence);
  MyAppHeader->ManualRelayStateForSequence = NTOHS(MessagePtr->ManualRelayStateForSequence);
  MyAppHeader->MeasurementValidIcMax = NTOHL(MessagePtr->MeasurementValidIcMax);
  MyAppHeader->MeasurementValidIcMin = NTOHL(MessagePtr->MeasurementValidIcMin);
  MyAppHeader->MeasurementValidReffMax = NTOHL(MessagePtr->MeasurementValidReffMax);
  MyAppHeader->MeasurementValidReffMin = NTOHL(MessagePtr->MeasurementValidReffMin);
  MyAppHeader->MeasurementValidVmicTempMax = NTOHL(MessagePtr->MeasurementValidVmicTempMax);
  MyAppHeader->MeasurementValidVmicTempMin = NTOHL(MessagePtr->MeasurementValidVmicTempMin);
  MyAppHeader->MeasurementValidPressureCapTempMax = NTOHL(MessagePtr->MeasurementValidPressureCapTempMax);
  MyAppHeader->MeasurementValidPressureCapTempMin = NTOHL(MessagePtr->MeasurementValidPressureCapTempMin);
  MyAppHeader->MovementThreshold = NTOHL(MessagePtr->MovementThreshold);
  MyAppHeader->MeasurementManualPslForSequence = NTOHS(MessagePtr->MeasurementManualPslForSequence);
  MyAppHeader->ProtocolRetryTimeout = NTOHS(MessagePtr->ProtocolRetryTimeout);
  MyAppHeader->ProtocolRetryCount = MessagePtr->ProtocolRetryCount;
  MyAppHeader->MeasurementDelayStartMilliSeconds = NTOHS(MessagePtr->MeasurementDelayStartMilliSeconds);
  MyAppHeader->BuzzerControl = MessagePtr->BuzzerControl;
  MyAppHeader->vibratorControl = MessagePtr->vibratorControl;
  MyAppHeader->AutoResonaceControl = MessagePtr->AutoResonaceControl;
  MyAppHeader->AutoPowerControl = MessagePtr->AutoPowerControl;
  MyAppHeader->MeasurementPslControl = MessagePtr->MeasurementPslControl;
  MyAppHeader->PushButtonTurnOffTime = MessagePtr->PushButtonTurnOffTime;
  MyAppHeader->PatientNurseMode = MessagePtr->PatientNurseMode;
  MyAppHeader->EnableMeasurmentWhileCharging = MessagePtr->EnableMeasurmentWhileCharging;
  MyAppHeader->MeasurementMinimalPslMarginPercentage = MessagePtr->MeasurementMinimalPslMarginPercentage;
  MyAppHeader->MeasurementMarginPercentage = MessagePtr->MeasurementMarginPercentage;
  MyAppHeader->MinimalSequencePsl = NTOHL(MessagePtr->MinimalSequencePsl);
  MyAppHeader->MaximalSequencePsl = NTOHL(MessagePtr->MaximalSequencePsl);
  MyAppHeader->PslSequenceStepPercentage = NTOHL(MessagePtr->PslSequenceStepPercentage);
  MyAppHeader->BeltSize = MessagePtr->BeltSize;
  MyAppHeader->NurseModePsl = NTOHS(MessagePtr->NurseModePsl);
  MyAppHeader->EnableChargingWhilePolling = MessagePtr->EnableChargingWhilePolling;
  MyAppHeader->EnableSleepWhilePolling = MessagePtr->EnableSleepWhilePolling;
  MyAppHeader->MeasurementTimeoutSeconds = NTOHL(MessagePtr->MeasurementTimeoutSeconds); 
   
  return(RETURNCODE_OK);
} 


uint8_t allocationErrorBuild = 0;
/**************************************************************************
*** ReturnCode_T protocolappMessageBuild(uartdllChannel_t Channel, uint32_t RequrstOrCommandSessionId, PROTOCOLAPP_COMMANDS_T messageOpCode, char * PayloadPtr, uint16_t PayloadLength, uint8_t PayloadMemFreeRequired, uint8_t **ReturnedMessagePtr, uint16_t *ReturnedMessageLengthPtr, uint8_t *ReturnedAddedPad)
* The  points to data which is MessageType depended, See protocolapp.h
*
*
*
***************************************************************************/
ReturnCode_T protocolappMessageBuild(uartdllChannel_t Channel, uint32_t RequrstOrCommandSessionId, PROTOCOLAPP_COMMANDS_T messageOpCode, char * PayloadPtr, uint16_t PayloadLength, uint8_t PayloadMemFreeRequired, uint8_t **ReturnedMessagePtr, uint16_t *ReturnedMessageLengthPtr, uint8_t *ReturnedAddedPad)
{
  uint8_t * cPtr;
  uint16_t	MessageLength;
  ProtocolappHeader_t myProtocolHeader;
  ProtocolappUpStreamHeader_t myUpStreamHeader;
  uint32_t EpochTime=0;
  uint8_t  Current10mSecCount;
  hwdriverPowerMeasurments_t myMeasurments;
  uint8_t* OpCodeRelatedStructurePtr;
  uint16_t OpCodeRelatedStructureLength = 0;
  uint16_t transmitterTemperaturePtr;
  hwdriversNtcTemperatureGet(&transmitterTemperaturePtr,HWDRIVERS_TX_NTC_P);
  //ProtocolappGeneralResp_t myAck;
  
  // The basic message length is the sum of the mandatory headers (SPI Protocol and UpStream) 
  // We also add the DLL headers and (DllHeader_t,  DllEndOfMessage_t) at this stage to avoid another malloc and memcpy in the DLL layer
  // While adding encryption I've removed the DllHeader from the SPI record, The Encryption layer will add this header (To ease pointers complexity)
  // Initialize the MessageLength to the headers + payload(later) to check the total messaage size without the spi and encryption header
  MessageLength =  sizeof(ProtocolappHeader_t) + sizeof(ProtocolappUpStreamHeader_t);
  // Fill the local generic protocol header structures
  if(rtcEpochTimeStampGet( &EpochTime, &Current10mSecCount) != RETURNCODE_OK)
    return(RETURNCODE_ERROR);
  
  // We assume we have the Epoch time stamp as the function returned OK, update the Event time
  myProtocolHeader.EventTimeStamp = HTONL(rtcEpochGet());            
  myProtocolHeader.EventTimeStamp10mSec = 0;//Current10mSecCount;      
  // There is no need to update those value - check the comment for those fields in the sturct definition
  myProtocolHeader.TransmissionTimeStamp = 0;     
  myProtocolHeader.TransmissionTimeStamp10mSec = 0; 
  myProtocolHeader.MessageNumerator            = HTONL(MessageNumerator);          
  myProtocolHeader.SourceAddress               = HTONS(PROTOCOLAPP_VLAPEX_ADDRESS);             
  myProtocolHeader.DestinationAddress         = HTONS(PROTOCOLAPP_SERVER_ADDRESS);
   // fill the protocol version  
  myProtocolHeader.ProtocolVersion                 = PROTOCOLAPP_PROTOCOL_VERSION;
  // Fill the Message Op Code
  myProtocolHeader.MessageOpCode                 = HTONS(messageOpCode);                    
  // Fill the returned sessionId field: If the Message Op Code is Event than use the localy generated session ID, If the Message Op Code is Command/Request, use the Command/Request SessionId
  if(messageOpCode >= PROTOCOLAPP_COMMANDS_AND_REQUESTS_BASE)
    myProtocolHeader.SessionID                    = HTONL(RequrstOrCommandSessionId);
  else
  {
    UpStreamSessionID++;
    myProtocolHeader.SessionID                    = HTONL(UpStreamSessionID);
  }

  myProtocolHeader.HwVersion = HTONL(HW_VERSION);

  myProtocolHeader.FwVersion.FirmwareVersion.BuildVersion = PCCOMMAPPLAYER_FW_VERSION_BUILD;
  myProtocolHeader.FwVersion.FirmwareVersion.MinorVersion = PCCOMMAPPLAYER_FW_VERSION_MINOR;
  myProtocolHeader.FwVersion.FirmwareVersion.MajorVersion = PCCOMMAPPLAYER_FW_VERSION_MAJOR;
  myProtocolHeader.FwVersion = (FirmwareVersion_T)HTONL(myProtocolHeader.FwVersion.FirmwareVersionValue);

  myProtocolHeader.BleFwVersion = HTONL(bleProcessorVersionGet());  
  myProtocolHeader.BootloaderFwVersion = HTONL(fwupgradeBootloaderVersionGet());
  myProtocolHeader.CRC32 = HTONL(crc32BuffCalc((uint8_t *) &myProtocolHeader, 0, sizeof(ProtocolappHeader_t) - 4));  

  
  // Fill the Up Stream header
  // Get the STM32 unique ID and copy to the local structure
  if(cPtr = protocolappUniqueIdPtrGet())
    memcpy(myUpStreamHeader.VlapExUniqId, cPtr, sizeof(myUpStreamHeader.VlapExUniqId));
  myUpStreamHeader.TBD0							= HTONS(0);
  myUpStreamHeader.TransmitterTemperature       = HTONS(transmitterTemperaturePtr);
  myUpStreamHeader.Longitude                    = HTONL(0);	
  //myUpStreamHeader.AltitudeFromMsl              = HTONL(0);
  myUpStreamHeader.AbsolutePressure             = HTONL((uint32_t)sensorAbsolutePressureGet());
  myUpStreamHeader.VmicSerialNumber             = HTONL(measurementSerialIdGet());
  myUpStreamHeader.ConfigurationVersionId       = HTONL(configVersionIdGet());

  
  
  if(hwdriversPowerMeasurmentsGet( &myMeasurments ) != RETURNCODE_OK)
    return(RETURNCODE_ERROR);
  
  // Fill the power measurements and statuses
//  myUpStreamHeader.BattVoltage                  = HTONS(myMeasurments.BattVoltage);
//  myUpStreamHeader.DcVoltage                    = HTONS(myMeasurments.DcInputVoltage);
//  myUpStreamHeader.ExternalTemperature          = HTONS(sensorTemperatureGet());
//  myUpStreamHeader.accX                         = HTONS(sensorAccGet(SENSOR_AXIS_X));
//  myUpStreamHeader.accY                         = HTONS(sensorAccGet(SENSOR_AXIS_Y));
//  myUpStreamHeader.accZ                         = HTONS(sensorAccGet(SENSOR_AXIS_Z));
  // Clear the staus word
  myUpStreamHeader.StatusUnion.StatusWord       = 0;
  // Update bits
  myUpStreamHeader.StatusUnion.Status.DcPlugConnected       = HTONS(myMeasurments.DcPlugConnectionState);
  myUpStreamHeader.StatusUnion.Status.ExternalPowerDetected = (myMeasurments.DcPlugVoltageExist);
  // Fix network order
  myUpStreamHeader.StatusUnion.StatusWord       = HTONL(myUpStreamHeader.StatusUnion.StatusWord);
    
  // Add relevant fields based on the message type
  switch (messageOpCode)
  {
  case PROTOCOLAPP_GENERAL_ACK_EVENT:
  case PROTOCOLAPP_GENERAL_NACK_EVENT:                   
    // The ack response answers with the originating OpCode
  //  myAck.OpCode = messageOpCode;
    // Update the Message Length with the opCode related data structure
  //  MessageLength += sizeof(myAck);
    // update vars used later for copying opcode related data structure while forming the message tx buffer 
  //  OpCodeRelatedStructurePtr = (uint8_t*)&myAck;
  //  OpCodeRelatedStructureLength = sizeof(myAck);
    break;
    
  case PROTOCOLAPP_POWER_BATT_HIGH_EVENT:
  case PROTOCOLAPP_POWER_BATT_LOW_EVENT:                 
  case PROTOCOLAPP_POWER_BATT_NORMAL_EVENT:              
  case PROTOCOLAPP_POWER_DC_PLUG_CONNECTED_EVENT:        
  case PROTOCOLAPP_POWER_DC_PLUG_DISCONECTED_EVENT:      
  case PROTOCOLAPP_POWER_DC_FAULT_EVENT:        
  case PROTOCOLAPP_POWER_DC_POWER_OFF_EVENT:
  case PROTOCOLAPP_CHARGING_STARTED_EVENT:
  case PROTOCOLAPP_CHARGING_ENDED_EVENT:
  case PROTOCOLAPP_GENERAL_KEEP_ALIVE_EVENT:
  case PROTOCOLAPP_GENERAL_OPC_EVENT:
  case PROTOCOLAPP_CONFIGURATION_DOWNLOAD_REQ:
  case PROTOCOLAPP_CONFIGURATION_CHANGE_EVENT:
  case PROTOCOLAPP_MEASUREMENT_ABORTED_SHORT_PUSH_BUTTON_EVENT:
  case PROTOCOLAPP_MEASUREMENT_ABORTED_BELT_OPEN_EVENT:
  case PROTOCOLAPP_MEASUREMENT_ABORTED_BATTERY_LEVEL_EVENT:
  case PROTOCOLAPP_MEASUREMENT_ABORTED_CHARGING_EVENT:
  case PROTOCOLAPP_MEASUREMENT_ABORTED_TEMPERATURE_LEVEL_EVENT:
  case PROTOCOLAPP_MEASUREMENT_ABORTED_TIMEOUT_EVENT:
    // Simple Events has no accompanied data structure
    break;
  case PROTOCOLAPP_GENERAL_NURSE_MODE_END_EVENT:
  case PROTOCOLAPP_MEASUREMENT_ENDED_EVENT:
  case PROTOCOLAPP_MEASUREMENT_FAILED_EVENT:
  case PROTOCOLAPP_GENERAL_SYSTEM_UP_EVENT:
  case PROTOCOLAPP_FW_UPGRADE_CHUNK_SEND_RESP:
  case PROTOCOLAPP_GENERAL_CORRUPTED_EVENT:
    // Update the Message Length with the opCode related data structure
    MessageLength += PayloadLength;
    // update vars used later for copying opcode related data structure while forming the message tx buffer 
    OpCodeRelatedStructurePtr = PayloadPtr;
    OpCodeRelatedStructureLength = PayloadLength;
    break;
    
    // update vars used later for copying opcode related data structure while forming the message tx buffer 
        // Update the Message Length with the opCode related data structure
//    MessageLength += PayloadLength;
//    OpCodeRelatedStructurePtr = PayloadPtr;
//    OpCodeRelatedStructureLength = PayloadLength;
//    break;
  default:
      // Event not supported  
      return(RETURNCODE_WRONG_INPUT_EVENT);
  }

  *ReturnedAddedPad = 0;
  
  // Pad message to %16 to allocate it now before the encryption and not to preform realloc
  if(MessageLength % 16)
  {
    *ReturnedAddedPad = (((MessageLength/16) + 1)* 16) - MessageLength;
    MessageLength = ((MessageLength/16) + 1)* 16;
  }
  else
    MessageLength = MessageLength;
  
  // Add all other headers
//  MessageLength += (sizeof(aescbcProtcolHeader_t) + sizeof(spiflashLogMemoryHeaderT));
  
//  cPtr = (uint8_t*)pvPortMalloc(MessageLength);
//  if(cPtr)
//  {
//    /// Copy the headers and the Message type related structure to the allocated memory, Note, the ProtocolHeader starts after the DLL header
//    // Copy the protocol header to the destination pointer
//    memcpy((uint8_t*)(cPtr+sizeof(spiflashLogMemoryHeaderT) + sizeof(aescbcProtcolHeader_t)), (uint8_t*)&myProtocolHeader, sizeof(myProtocolHeader));
//    // Copy the upstream header to the destination pointer
//    memcpy((uint8_t*)(cPtr+sizeof(spiflashLogMemoryHeaderT)+sizeof(myProtocolHeader) + sizeof(aescbcProtcolHeader_t)), (uint8_t*)&myUpStreamHeader, sizeof(myUpStreamHeader));
//    if(OpCodeRelatedStructureLength && OpCodeRelatedStructurePtr)
//      memcpy((uint8_t*)(cPtr+sizeof(spiflashLogMemoryHeaderT)+sizeof(myProtocolHeader) + sizeof(myUpStreamHeader) + sizeof(aescbcProtcolHeader_t)), (uint8_t*)(OpCodeRelatedStructurePtr), OpCodeRelatedStructureLength);
//
//    // Free the OpCodeRelated allocated memory
//    if(PayloadPtr && PayloadMemFreeRequired)
//      vPortFree(PayloadPtr);
//
//    // Update the calling function with the message ptr and length
////    if(ReturnedMessagePtr && ReturnedMessagePtr)
////    {
////      // Add the Sync pattern
////      ((spiflashLogMemoryHeaderT*)cPtr)->SyncPattern = SPIFLASH_SYNC_PATTERN;
////      // Update the spiflash payload header, exclude the spiflashHeaderSizeT size
//////      ((spiflashLogMemoryHeaderT*)cPtr)->EntrySize = (MessageLength-sizeof(spiflashLogMemoryHeaderT));
////      // Update the spiflash Rev. entry index
////      ((spiflashLogMemoryHeaderT*)cPtr)->PrevEntryAddress = eventsQueuePrevHeadIndexGet();
////      // This is the full msg including the nvm header
////      *ReturnedMessageLengthPtr = MessageLength;
////      *ReturnedMessagePtr = cPtr;
////      // Increment the numerator
////      MessageNumerator++;
////      return(RETURNCODE_OK);
////    }
////    else
////    {
////      // Free the allocated memory
////      vPortFree(cPtr);
////      // Return Error
////      return(RETURNCODE_ERROR);
////    }
//  }
//  else
//  {
//    allocationErrorBuild++;
//      // Return Error
//      return(RETURNCODE_ERROR);
//  }
  
}


/**************************************************************************
* ReturnCode_T protocolappMessageBuildnNotViaFlash(uartdllChannel_t Channel, uint32_t RequrstOrCommandSessionId, PROTOCOLAPP_COMMANDS_T messageOpCode, char * PayloadRelatedDataPtr, uint16_t PayloadRelatedDataLength, uint8_t *OutputBufferPtr, uint16_t *ReturnedMessageLengthPtr, uint8_t *ReturnedAddedPad)
*
*
*
*
***************************************************************************/
ReturnCode_T protocolappMessageBuildnNotViaFlash(uartdllChannel_t Channel, uint32_t RequrstOrCommandSessionId, PROTOCOLAPP_COMMANDS_T messageOpCode, char * PayloadRelatedDataPtr, uint16_t PayloadRelatedDataLength, uint8_t *OutputBufferPtr, uint16_t *ReturnedMessageLengthPtr, uint8_t *ReturnedAddedPad)
{

	return(RETURNCODE_OK);
}


/**************************************************************************
* >>> uint8_t* protocolappUniqueIdPtrGet()
* The payloadPtr points to data which is MessageType depended, See protocolapp.h
*
*
*
***************************************************************************/
uint8_t* protocolappUniqueIdPtrGet()
{
  return( (uint8_t*) 0x1FF1E800);
}



