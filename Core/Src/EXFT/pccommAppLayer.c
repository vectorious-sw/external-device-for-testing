#include <hwdrivers.h>
#include "pccommAppLayer.h"
#include "vmicapplayer.h"
#include "vmicmodem.h"
#include "vlapMain.h"
#include "autoresonance.h"
#include "measure.h"
#include "autopower.h"
#include "config.h"

#define pccommapplayerConnecTimeout 500*portTICK_PERIOD_MS

// P R O T O T Y P E S 
uint8_t *  RegisterRead(uint16_t Address, uint8_t Length, uint8_t *freeMemoryFlag); 
uint8_t *  RegisterWrite(uint16_t Address, uint8_t Length, uint8_t* PayloadPtr); 
AddrRange_T AddressRangeResolve(uint16_t Address, uint16_t* ReturnedOffsetInRange, uint8_t** ReturnedPtr);
void   pcccommAppLayerBuildResponseMsg(uint8_t * payloadPtr, uint8_t MsgLength, uint8_t freeMemoryFlag);
void pccommnapplayerConnectivityTimerCallback();
void pccommnapplayerConnectivityTimerRetrigger(uint16_t timeout);
void pcccommAppLayerSpiFlashReadResponseMsgBuildCallBack();
void pcccommAppLayerSpiFlashWriteResponseMsgBuildCallBack();
void pcccommAppLayerBleBootloaderWriteResponseMsgBuildCallBack(char* data, uint8_t length);
void pcccommAppLayerSpiFlashWriteResponseCRC32CallBack(SPIMemoryCommand_T memoryType, uint8_t result);

// V A R S 
pccpmmAppLayerStruct_T  pccpmmAppLayerStruct;
pccommAppLayerSampleBufferNumber_T pccommAppLayerSampleBufferNumber;
uint8_t pccommAppLayerIndex;
pccpmmAppLayerPendingTx_T pccpmmAppLayerPendingTx;

vmicapplayerVmicTxSourceSelectCommandT pccpmmAppLayerNewVmicSourceAfterTx;

uint8_t ResponseBuffer[300];
uint8_t SamplesIndex;

TimerHandle_t x30Timer = NULL;

TimerHandle_t pccommnapplayerConnectivityTimerHandler;

pccommnapplayerConnectivityState_T pccommnapplayerConnectivityState;

uint8_t * GenericBufferPtr;
uint32_t  GenericBufferPayLoadLength;
/******************************************************************************
 * @brief  void pcccommAppLayerInit()
 * @param  
 * @retval 
 ******************************************************************************/
void pcccommAppLayerInit()
{
  pccommAppLayerSampleBufferNumber=PCCOMMAPPLAYER_SAMPLE_BUFFER_1;
  pccommAppLayerIndex = 0;
  pcccommAppLayerParametersInit();
  pccpmmAppLayerPendingTx = PCCOMMAPPLAYER_VMIC_TX_REQUEST_IDLE;
  SamplesIndex = 0;
  GenericBufferPtr = 0;
  GenericBufferPayLoadLength = 0;
  
  // Initial PC connectivity state is disconnected
  pccommnapplayerConnectivityState = PCCOMMAPPLAYER_CONNECTIVITY_STATE_DISCONNECTED;
  // Create onShot timer to define connectivit mode with PC via USE
  pccommnapplayerConnectivityTimerHandler = xTimerCreate("pccommapplayerConnec",  pccommapplayerConnecTimeout, pdFALSE, (void *)0, pccommnapplayerConnectivityTimerCallback);
  xTimerStart(pccommnapplayerConnectivityTimerHandler, 100);
 }


uint8_t * SpiFlashPageReadAllocatedMemoryPtr;
uint8_t pcccommAppLayerAfterUsbDownloadDelay = 5;
/******************************************************************************
 * @brief  void pcccommAppLayerProcessing( uint8_t * ptr, uint16_t BufferLength)
 * @param  
 * @retval 
 ******************************************************************************/
void pcccommAppLayerProcessing( uint8_t * ptr, uint16_t BufferLength)
{
  uint16_t Address = ptr[MSG_OFFSET_ADDRESS_MSB]*256 + ptr[MSG_OFFSET_ADDRESS_LSB];
  uint8_t  MsgLength = ptr[MSG_OFFSET_LENGTH];
  uint8_t  Command = ptr[MSG_OFFSET_CMD];
  uint8_t*  PayloadPtr = ptr+MSG_OFFSET_PAYLOAD;
  uint8_t* ReturnedPayLoadPtr = NULL;
  uint8_t FreeMemoryFlag = 0;
  uint16_t length;
  uint16_t messageLength;

  
  // Retrigger PC connectivity state onShot timer 
  if((Command == 'M') || (Command == 'L'))
  {
    // If we are during download from the USB - increase the timeout of the polling so it won't go to sleep immediately
    pccommnapplayerConnectivityTimerRetrigger(2000);
    pcccommAppLayerAfterUsbDownloadDelay = 5;
  }
  else
  {
    // If first command after usb download - still use bigger timeout for polling to prevent sleep
    if(pcccommAppLayerAfterUsbDownloadDelay)
    {
      pccommnapplayerConnectivityTimerRetrigger(2000);
      pcccommAppLayerAfterUsbDownloadDelay--;
    }
    else
    {
      pccommnapplayerConnectivityTimerRetrigger(pccommapplayerConnecTimeout);
    }     
  }
    
  
  //printf("Command: %d, Address: %d, MsgLength: %d\n", Command, Address, MsgLength);
  switch(Command)
  {
  case 'R':
    ReturnedPayLoadPtr = RegisterRead(Address, MsgLength, &FreeMemoryFlag); 
    break;
  case 'L':
    // Message length
    length = (ptr[MSG_OFFSET_LENGTH] * 256) + ptr[4];
//    bleProgramFsm(BLE_PROGRAM_STIMULI_SEND_DATA, (uint8_t*)&ptr[5], length, &pcccommAppLayerBleBootloaderWriteResponseMsgBuildCallBack);
    return;
  case 'M':
    // Message length payload without the payload length
    messageLength = (ptr[MSG_OFFSET_LENGTH] * 256) + ptr[4] - 4;
    // Write to SPI memory
    uint32_t baseAddress;
    switch(Address)
    {
    case PCCOMMAPPLAYER_BACKUP_ADDRESS:
      break;
    case PCCOMMAPPLAYER_UPGRADE_ADDRESS:
      break;
    default:
      // Not valid address request - return
      return;
    }
    
    // Check if currently there is a fw download in progress
//    if(fwupgradeStateGet() != FWUPGRADE_STATE_IDLE)
//    {
//      // Notify the FWUPGRADE FSM to stop the download due to version download from the VTS
//      fwupgradeEventSend(FWUPGRADE_OPCODE_STOP_DOWNLOAD, 0, 0);
//    }

    uint32_t flashAddress = baseAddress + ((uint32_t)ptr[5] << 24 | (uint32_t)ptr[6] << 16
                                    | (uint32_t)ptr[7] << 8 | (uint32_t)ptr[8]);

    //hwdriversGpioBitWrite(HWDRIVERS_PB14_USB_TEST, 0);

//    spiflashdiskReqEnqueue(SPIFLASHDISK_FSM_STIMULI_WRITE, flashAddress, 0, (uint8_t*)&ptr[9], messageLength, &pcccommAppLayerSpiFlashWriteResponseMsgBuildCallBack, 0);
    return;
  case 'S':
    // Read the requested spiFlash page
    // Allocated memory buffer
    return;
    break;
  case 'F':
    ReturnedPayLoadPtr = RegisterRead(Address, MsgLength, &FreeMemoryFlag); 
    break;
  case 'W':
    // special case
    PayloadPtr = ptr+MSG_OFFSET_LENGTH;
    ReturnedPayLoadPtr = RegisterWrite(Address, 1, PayloadPtr);
    break;
  case 'B':
    ReturnedPayLoadPtr = RegisterWrite(Address, MsgLength, PayloadPtr);
    break;
  } 
  
  pcccommAppLayerBuildResponseMsg(ReturnedPayLoadPtr, MsgLength, FreeMemoryFlag);
  
}


// TODO: Remove later
volatile uint16_t debugPrint[100];
volatile uint8_t  debugPrintIndex =0;


/******************************************************************************
 * @brief  uint8_t *  RegisterRead(uint16_t Address, uint8_t Length)
 * @param  
 * @retval 
 ******************************************************************************/
uint8_t *  RegisterRead(uint16_t Address, uint8_t Length, uint8_t *freeMemoryFlag) 
{
  uint8_t * Ptr = NULL;
  uint16_t* SrcPtr;
  uint8_t* ReturnedPtr;
  uint16_t i;
  int32_t* Int32Ptr;
  int32_t TempInt32;
  AddrRange_T AddressRangeType;
  uint32_t  ReturnedCount;
  uint32_t ReturnedHeaderPosition;
  uint16_t ReturnedOffsetInRange;
  uint16_t HeaderPosition = (pccpmmAppLayerStruct.DspRegisters.GraphDispOffsetMsb*256)+pccpmmAppLayerStruct.DspRegisters.GraphDispOffsetLsb;
  
  AddressRangeType = AddressRangeResolve(Address, &ReturnedOffsetInRange,  &ReturnedPtr);
  
  // TODO: 
  if((debugPrintIndex <(sizeof(debugPrint)/sizeof(uint16_t))) )
  {
    debugPrint[debugPrintIndex] = Address;
    debugPrintIndex++;
    debugPrint[debugPrintIndex] = Length;
    debugPrintIndex++;
  }
  
  *freeMemoryFlag = 0;
  switch(AddressRangeType)
  {
  case PCCOMMAPPLAYER_ADDR_RANGE_VMIC_DSP_SAMPLES_1:
    switch((DspSamplesBufferSourceSelect_T) pccpmmAppLayerStruct.DspRegisters.SignalMonitorMode)
    {
    case DSP_SAMPLES_SOURCE_NONE:
      break;
    case DSP_SAMPLES_SOURCE_DSP0:
    case DSP_SAMPLES_SOURCE_DSP1:
      Ptr = ReturnedPtr;
      SrcPtr = (uint16_t*)(adc1DmaBuffer[0] + HeaderPosition  + ReturnedOffsetInRange);
      for(i=0; i<Length; i++)
        *(Ptr+i) = *(SrcPtr+i)>>4;
      break;
    case DSP_SAMPLES_SOURCE_AVERAGE:
      if(VmicGetDiagInfo(0,  VMIC_AVERAGE_BUFFER,  (void**)&Int32Ptr, &ReturnedCount) == STATUS_SUCCESSFUL)
      {
        Ptr = ReturnedPtr;
        Int32Ptr = (int32_t*)(Int32Ptr + HeaderPosition + ReturnedOffsetInRange);
        for(i=0; i<Length; i++)
        {
          TempInt32 = *(Int32Ptr+i);
          if(TempInt32 > 0)
            *(Ptr+i) = (TempInt32)>>7;
          else
            *(Ptr+i) = 0;
        }
      }
      break;
    case DSP_SAMPLES_SOURCE_HEADER:
      if(VmicGetDiagInfo(0,  VMIC_DIAG_CORRELATION_RESULTS, (void**) &Int32Ptr, &ReturnedCount) == STATUS_SUCCESSFUL)
      {
        Ptr = ReturnedPtr;
        Int32Ptr = (int32_t*)(Int32Ptr + HeaderPosition+ ReturnedOffsetInRange - Length);
        for(i=0; i<Length; i++)
        {
          TempInt32 = *(Int32Ptr+i);
          if(TempInt32 > 0)
            *(Ptr+i) = (TempInt32)>>7;
          else
            *(Ptr+i) = 0;

        }
      }
      break;
    case DSP_SAMPLES_SOURCE_NOISE:
#if 0
      if(VmicGetDiagInfo(0,  VMIC_DIAG_NOISE_RESULTS, (void**) &Int32Ptr, &ReturnedCount) == STATUS_SUCCESSFUL)
      {
        Ptr = ReturnedPtr;
        SrcPtr = (uint16_t*)(SrcPtr + HeaderPosition);
        for(i=0; i<Length; i++)
          *(Ptr+i) = *(SrcPtr+i)>>4;
      }
#endif
      break;
    default:
      break;
    }
    break;
  case PCCOMMAPPLAYER_ADDR_RANGE_VMIC_MEASURMENT_1:
    // Mark the buffer as Not ready, as it was already read
    pccpmmAppLayerStruct.VmicRegisters3.VmicMeasurementBufferReady = 0;
    break;
  case PCCOMMAPPLAYER_ADDR_RANGE_VMIC_MEASURMENT_2:
    // Mark the buffer as Not ready, as it was already read
    pccpmmAppLayerStruct.VmicRegisters3.VmicMeasurementBufferReady = 0;
    break;
  case PCCOMMAPPLAYER_ADDR_RANGE_BOARD:
#if 1
    if(VmicModemHeaderPositionGet(&ReturnedHeaderPosition) == STATUS_SUCCESSFUL)
    {
      pccpmmAppLayerStruct.DspRegisters.HeaderPositionMsb = (uint8_t)(ReturnedHeaderPosition >> 8);
      pccpmmAppLayerStruct.DspRegisters.HeaderPositionLsb = (uint8_t)ReturnedHeaderPosition;
    }
#endif
    switch(ReturnedOffsetInRange)
    {
    case 0x2a:  // 0x20ba  NTC Pos 
      *Ptr = pccpmmAppLayerStruct.Board1SystemRegisters.NtcPosL;
      break;
    case 0x2b:  // 0x20ba  NTC Pos
      *(Ptr) = pccpmmAppLayerStruct.Board1SystemRegisters.NtcPosM;
      break;
    case 0x2c:  // 0x20bc  NTC Neg 
      *Ptr = pccpmmAppLayerStruct.Board1SystemRegisters.NtcNegL;
      break;
    case 0x2d:  // 0x20bd  NTC Neg
      *(Ptr) = pccpmmAppLayerStruct.Board1SystemRegisters.NtcNegM;
      break;
    case 0x2e:  // 0x20be  Modem Rx PtP Voltage mV 
      *Ptr = pccpmmAppLayerStruct.Board1SystemRegisters.ModemRxPtpL;
      break;
    case 0x2f:  // 0x20bf  Modem Rx PtP Voltage mV 
      *(Ptr) = pccpmmAppLayerStruct.Board1SystemRegisters.ModemRxPtpM;
      break;
    }
    break;
    
  case PCCOMMAPPLAYER_ADDR_RANGE_GENERIC_BUFFER:
    ReturnedPtr = GenericBufferPtr;
    *freeMemoryFlag = 1;
    break;
    
  case PCCOMMAPPLAYER_ADDR_RANGE_VMIC1:
  case PCCOMMAPPLAYER_ADDR_RANGE_VMIC2:
  case PCCOMMAPPLAYER_ADDR_RANGE_VMIC3:
  case PCCOMMAPPLAYER_ADDR_RANGE_BELT_TRANSMITTER:
  case PCCOMMAPPLAYER_ADDR_RANGE_DSP_REGISTERS:
  case PCCOMMAPPLAYER_ADDR_RANGE_ERROR:
    break;
  default:
    break;
  }
  
  return(ReturnedPtr);
}



/******************************************************************************
 * @brief  uint8_t *  RegisterWrite(uint16_t Address, uint8_t Length, uint8_t* PayloadPtr)
 * @param  
 * @retval 
 ******************************************************************************/
uint8_t *  RegisterWrite(uint16_t Address, uint8_t Length, uint8_t* PayloadPtr)
{
  uint8_t * Ptr=0;
  uint8_t* ReturnedPtr;
  uint8_t i;
  AddrRange_T AddressRangeType;
  uint16_t ReturnedOffsetInRange;
  uint16_t Temp;
  MemoryCommand_T commandType;
  uint32_t configurationSize;
  uint32_t bufferSize;
  uint32_t logMemoryPointersSize;
  uint8_t *p;
  uint32_t len;
  uint32_t externalIdSize;
  uint8_t * cPtr;
  uint8_t response = 0;
  uint32_t responseSize;
  uint32_t baseAddress;
  ProtocolappUpStreamHeader_t myUpStreamHeader;


  // Resolve the address range, Return the address type enum
  AddressRangeType = AddressRangeResolve(Address, &ReturnedOffsetInRange, &ReturnedPtr);
  // Copy the payload buffer into the destination buffer 
  for(i=0; i<Length; i++)
    *(ReturnedPtr+i) = *(PayloadPtr+i);
  
  switch(AddressRangeType)
  {
  case PCCOMMAPPLAYER_ADDR_RANGE_VMIC1:
    // VMIC measurment source select
    vmicmodemSelect( pccpmmAppLayerStruct.VmicRegisters1.C2fMeasurementSelect, 0);
    break;
  case PCCOMMAPPLAYER_ADDR_RANGE_VMIC2:
  case PCCOMMAPPLAYER_ADDR_RANGE_VMIC3:
    // Maps between the internal EXT device parameter and the VMIC register to be written
    ExtDeviceToVmicDataUnion3.Data3 = 0;
    ExtDeviceToVmicDataUnion3.DataBits3.StatusMeasure = 1;
    ExtDeviceToVmicDataUnion3.DataBits3.Dst0 = 1;
    ExtDeviceToVmicDataUnion3.DataBits3.Dst1 = 1;
    ExtDeviceToVmicDataUnion3.DataBits3.C2fNotInUse1 = 0;
    ExtDeviceToVmicDataUnion3.DataBits3.AnalogVoltageSelect = pccpmmAppLayerStruct.VmicRegisters2.AnalogSensorVoltage;
    ExtDeviceToVmicDataUnion3.DataBits3.RFsyncEnable = pccpmmAppLayerStruct.VmicRegisters2.ExternalRfClockSource;
    ExtDeviceToVmicDataUnion3.DataBits3.ResonanceCapacitor = pccpmmAppLayerStruct.VmicRegisters2.ResonanceCapacitor;
    ExtDeviceToVmicDataUnion3.DataBits3.ModulationMode = pccpmmAppLayerStruct.VmicRegisters2.ModulationMode;
    // Mark the session as pending VMIC Tx session
    pccpmmAppLayerPendingTx = PCCOMMAPPLAYER_VMIC_TX_REQUEST_PENDING;
    // Update the modulator
    vlapmodemConfigWordSet(VMICMODEM_VMIC_CONFIG_WORD_3, ExtDeviceToVmicDataUnion3.Data3);
    vlapDemodulatorTxReq(VMICMODEM_VMIC_CONFIG_WORD_3, 0);
    break;
    
  case PCCOMMAPPLAYER_ADDR_RANGE_VMIC_DSP_SAMPLES_1:
  case PCCOMMAPPLAYER_ADDR_RANGE_BOARD:
    switch(ReturnedOffsetInRange)
    {
    case 15:
      pccpmmAppLayerStruct.BoardSystemRegisters.OnBoardPushButton = (uint8_t)(*PayloadPtr);
       measureTaskEventSend(MEASURE_PB_START, MEASURE_TASK_QUEUE_SEND_SOURCE_NORMAL);  
      break;
      // Debug0
    case 73:  
        // Update the GUI data structure
        pccpmmAppLayerStruct.BeltTransmitterRegisters.RelayStateMsb = (uint8_t)(*PayloadPtr);
        pccpmmAppLayerStruct.BeltTransmitterRegisters.RelayStateLsb = (uint8_t)*(PayloadPtr+1);
          // Update the relays 
        autoresonanceManaualRelaysSet((uint16_t)*(PayloadPtr+1) + ((uint16_t)*PayloadPtr)*256 );
        break;
      // Debug 1
    case 0x56:
      autopowerManaualPwmSet( (uint8_t)*PayloadPtr);
      break;
    default:
      break;
    }
    break;
 
  case PCCOMMAPPLAYER_ADDR_RANGE_BOARD1:
    switch(ReturnedOffsetInRange)
    {
    case 0x24:  // 0x20b4  DAC ACR 
      pccpmmAppLayerStruct.Board1SystemRegisters.AcrDacValueM = *(PayloadPtr);
      pccpmmAppLayerStruct.Board1SystemRegisters.AcrDacValueL = *(PayloadPtr+1);
      Temp = (uint16_t) + ((uint16_t)*PayloadPtr)*256;
      dacWrite(Temp);
      break;
    case 0x26:  // 0x20b6  OPC One Point Calibration for on board pressure sensor 
        pccpmmAppLayerStruct.Board1SystemRegisters.PressureSensorOffsetM = *(PayloadPtr);
        pccpmmAppLayerStruct.Board1SystemRegisters.PressureSensorOffsetL = *(PayloadPtr+1);
        uint16_t Offset = pccpmmAppLayerStruct.Board1SystemRegisters.PressureSensorOffsetM * 256 + pccpmmAppLayerStruct.Board1SystemRegisters.PressureSensorOffsetL;
        configProductionDbOpcWrite(Offset);
      break;

    case 0x28:  // 0x20b8  Production Configuration Block
//      if(GenericBufferPtr && GenericBufferPayLoadLength)
//      {
//        configApplicationConfigDbWrite(GenericBufferPtr, GenericBufferPayLoadLength);
//        vPortFree(GenericBufferPtr);
//        GenericBufferPtr = 0;
//        GenericBufferPayLoadLength = 0;
//      }
      break;

      
    case 0x29:  // 0x20b9  
      // Free register
      break;
      
    case 0xb:  // 0x209b  Application Ram block request
      commandType = (MemoryCommand_T) *(PayloadPtr);
      
      switch(commandType)
      {
      case PCCOMMAPPLAYER_WRITE_CONFIG:
        configApplicationConfigDbWrite(GenericBufferPtr, GenericBufferPayLoadLength);
        vPortFree(GenericBufferPtr);
        GenericBufferPtr = 0;
        GenericBufferPayLoadLength = 0;
        break;
      case PCCOMMAPPLAYER_READ_CONFIG:
        configurationSize = sizeof(configConfigurationDb_t);
        // Create response buffer with size of configConfigurationDb_t and length of the message.
        bufferSize = configurationSize + sizeof(configurationSize);
        
        GenericBufferPtr = pvPortMalloc(bufferSize);
        GenericBufferPayLoadLength = bufferSize;
        
        memcpy(GenericBufferPtr, &configurationSize, sizeof(configurationSize));
        memcpy(GenericBufferPtr + sizeof(configurationSize), &configConfigurationDb, configurationSize);
        
        break;
      case PCCOMMAPPLAYER_WRITE_PRODUCTION:
        break;
      case PCCOMMAPPLAYER_READ_PRODUCTION:
        break;
      case PCCOMMAPPLAYER_READ_EVENTS_LOG_MEMORY:
//        logMemoryPointersSize = sizeof(eventsLogMemoryPointers_T);
        // Create response buffer with size of eventsLogMemoryPointers_T and length of the message.
        bufferSize = logMemoryPointersSize + sizeof(logMemoryPointersSize);

        GenericBufferPtr = pvPortMalloc(bufferSize);
        GenericBufferPayLoadLength = bufferSize;
        
        memcpy(GenericBufferPtr, &logMemoryPointersSize, sizeof(logMemoryPointersSize));
//        memcpy(GenericBufferPtr + sizeof(logMemoryPointersSize), &eventsLogMemoryPointersStructure, logMemoryPointersSize);
        
        break;
      case PCCOMMAPPLAYER_READ_RAM_MEMORY:
        if(GenericBufferPtr && GenericBufferPayLoadLength)
        {
          p = (uint8_t *) ((uint32_t)GenericBufferPtr[0] << 24 | (uint32_t)GenericBufferPtr[1] << 16
                                    | (uint32_t)GenericBufferPtr[2] << 8 | (uint32_t)GenericBufferPtr[3]);
          
          len = (uint32_t)GenericBufferPtr[4] << 24 | (uint32_t)GenericBufferPtr[5] << 16
            | (uint32_t)GenericBufferPtr[6] << 8 | (uint32_t)GenericBufferPtr[7];
          
          vPortFree(GenericBufferPtr);
          
          GenericBufferPayLoadLength = len + 4;
          GenericBufferPtr = pvPortMalloc(GenericBufferPayLoadLength);
          memcpy(GenericBufferPtr, &len, 4);
          memcpy(GenericBufferPtr + 4, (uint8_t*) (p), len);
        }
        break;
      case PCCOMMAPPLAYER_READ_EXTERNAL_ID:

        externalIdSize = sizeof(myUpStreamHeader.VlapExUniqId);
        // Create response buffer with size of VlapExUniqId and length of the message.
        bufferSize = externalIdSize + sizeof(externalIdSize);
        
        GenericBufferPtr = pvPortMalloc(bufferSize);
        GenericBufferPayLoadLength = bufferSize;
        
        cPtr = protocolappUniqueIdPtrGet();
        
        memcpy(GenericBufferPtr, &externalIdSize, sizeof(externalIdSize));
        memcpy(GenericBufferPtr + sizeof(externalIdSize), cPtr, externalIdSize);
        break;
      case PCCOMMAPPLAYER_SET_KEY:
        if(GenericBufferPtr && GenericBufferPayLoadLength)
        {
          // TODO: Do something about received generic buffer.
          
          // Send one byte as response to VTS.          
          response = 0;

          // Check if received payload is 64 key bytes
          if(GenericBufferPayLoadLength == 64)
          {
            configProductionDbKeysWrite(GenericBufferPtr);
            response = 1;
          }
          vPortFree(GenericBufferPtr);
          
          responseSize = sizeof(response);
          // Create response buffer with size of configConfigurationDb_t and length of the message.
          bufferSize = responseSize + sizeof(responseSize);
          
          GenericBufferPtr = pvPortMalloc(bufferSize);
          GenericBufferPayLoadLength = bufferSize;
          
          memcpy(GenericBufferPtr, &responseSize, sizeof(responseSize));
          memcpy(GenericBufferPtr + sizeof(responseSize), &response, sizeof(response));
        }
        break;
      case PCCOMMAPPLAYER_READ_SPI_FUG_SET_RESULT:
        if(GenericBufferPtr && GenericBufferPayLoadLength)
        {
          // Get the selected memory section to check
          SPIMemoryCommand_T memorySection = (SPIMemoryCommand_T) GenericBufferPtr[0];
//          FwupgradeSpiFugCrcResetT resetRequest = (FwupgradeSpiFugCrcResetT) GenericBufferPtr[1];
          // Free the generic buffer now that we don't need it
          vPortFree(GenericBufferPtr);
          GenericBufferPtr = 0;
          GenericBufferPayLoadLength = 0;
          
          // Write to SPI memory

          switch(memorySection)
          {
          case PCCOMMAPPLAYER_BACKUP_ADDRESS:
//            baseAddress = SPIFLASH_VANILLA_START;
            break;
          case PCCOMMAPPLAYER_UPGRADE_ADDRESS:
//            baseAddress = SPIFLASH_UPGRADE_START;
            break;
          default:
            break;
          }
          
          // Place the check functions 
//          fwupgradeSpiFugCrcCheckStart(baseAddress, resetRequest, pcccommAppLayerSpiFugCrcCheckCompletionCallBack);
        }
        break;
      default:
        break;
      }
      
    default:
      break;
    }
    break;
    
  case PCCOMMAPPLAYER_ADDR_RANGE_BELT_TRANSMITTER:
    switch(ReturnedOffsetInRange)
    {
      case 0x00:
      // Transmitter Enable 
      if(*PayloadPtr)
          pccommapplayerTransmitterControl(TYPES_ENABLE);
        else
          pccommapplayerTransmitterControl(TYPES_DISABLE);
        break;

    case 0x01:
      // Enable Auto Resonance 
      if(*PayloadPtr)
          pccommapplayerAutoResonanceControl(TYPES_ENABLE);
        else
          pccommapplayerAutoResonanceControl(TYPES_DISABLE);
        break;
      case 0x02: 
      // Enable Auto Current control
      if(*PayloadPtr)
         pccommapplayerAutoCurrentControl(TYPES_ENABLE);
        else
          pccommapplayerAutoCurrentControl(TYPES_DISABLE);
      break;
      case 0x03: 
       // Sets the current goal for the auto current control
       pccommapplayerAutoCurrentSet((uint16_t)*(PayloadPtr+1) + ((uint16_t)*PayloadPtr)*256);
        break;
      case 0x09: 
        // Sets the relays state
        autoresonanceManaualRelaysSet((uint16_t)*(PayloadPtr+1) + ((uint16_t)*PayloadPtr)*256);
        break;
      case 0x15: 
        // Sets the Belt size 
        break;
      case 0x16:
        // Sets the Pulse Skip Level
        autopowerPulseSkipLevelSet(*PayloadPtr);
        break;
    }
    break;
  
  case  PCCOMMAPPLAYER_ADDR_RANGE_GENERIC_BUFFER:
    // Check if previous usage of GenericBufferPtr is not freed
    if(GenericBufferPtr)
    {
      vPortFree(GenericBufferPtr);
    }
    
    GenericBufferPayLoadLength = (uint32_t)(*(PayloadPtr+3));
    if( GenericBufferPayLoadLength < 1024)
    {
      GenericBufferPtr = pvPortMalloc(GenericBufferPayLoadLength);
      if(GenericBufferPtr)
        memcpy(GenericBufferPtr, (uint8_t*) (PayloadPtr+4), GenericBufferPayLoadLength);
    }
    else
     GenericBufferPayLoadLength = 0; 
    
    break;

  case PCCOMMAPPLAYER_ADDR_RANGE_DSP_REGISTERS:
  case PCCOMMAPPLAYER_ADDR_RANGE_VMIC_MEASURMENT_1:
  case PCCOMMAPPLAYER_ADDR_RANGE_VMIC_MEASURMENT_2:
  case PCCOMMAPPLAYER_ADDR_RANGE_ERROR:
    break;
default:
	break;
  }
  
  
  
  
 
  return(Ptr);
}


/******************************************************************************
* @brief  AddrRange_T AddressRangeResolve(uint16_t Address, uint8_t** ReturnedPtr)
* @param  
* @retval 
******************************************************************************/
AddrRange_T AddressRangeResolve(uint16_t Address, uint16_t* ReturnedOffsetInRange, uint8_t** ReturnedPtr)
{
  uint16_t Offset;
  AddrRange_T ReturnedAddressRange = PCCOMMAPPLAYER_ADDR_RANGE_ERROR;
  
  // Board Address Range
  if((Address>=0x2000) && (Address<=0x201f))
  {
    // vdbg(">>> 0x2000-0x201f\n");
    Offset = (Address-0x2000);
    *ReturnedPtr = (uint8_t *)&pccpmmAppLayerStruct.BoardSystemRegisters + Offset;
    *ReturnedOffsetInRange = Offset;
    ReturnedAddressRange = PCCOMMAPPLAYER_ADDR_RANGE_BOARD;
  }
  else
    // VMIC 1 Address Range
    if((Address>=0x2020) && (Address<=0x2023))
    {
      // vdbg(">>> 0x2020-0x2023\n");
      Offset = (Address-0x2020);
      *ReturnedPtr = (uint8_t *)&pccpmmAppLayerStruct.VmicRegisters1 + Offset;
      *ReturnedOffsetInRange = Offset;
      ReturnedAddressRange = PCCOMMAPPLAYER_ADDR_RANGE_VMIC1;
    }
    else
      // VMIC 3 Address Range
      if((Address>=0x2028) && (Address<=0x203f))
      {
        // vdbg(">>> 0x2028-0x203f\n");
        Offset = (Address-0x2028);
        *ReturnedPtr = (uint8_t *)&pccpmmAppLayerStruct.VmicRegisters3 + Offset;
        *ReturnedOffsetInRange = Offset;
        ReturnedAddressRange = PCCOMMAPPLAYER_ADDR_RANGE_VMIC3;
      }
      else
        // BeltTransmitter Address Range
        if((Address>=0x2040) && (Address<=0x205f))
        {
          // vdbg(">>> 0x2040-0x205f\n");
          Offset = (Address-0x2040);
          *ReturnedPtr = (uint8_t *)&pccpmmAppLayerStruct.BeltTransmitterRegisters + Offset;
          *ReturnedOffsetInRange = Offset;
          ReturnedAddressRange = PCCOMMAPPLAYER_ADDR_RANGE_BELT_TRANSMITTER;
        }
        else
          // DspRegister Address Range 1
          if((Address>=0x2060) && (Address<=0x206f))
          {
            // vdbg(">>> 0x2060-0x206f\n");
            Offset = (Address-0x2060);
            *ReturnedPtr = (uint8_t *)&pccpmmAppLayerStruct.DspRegisters + Offset;
            *ReturnedOffsetInRange = Offset;
            ReturnedAddressRange = PCCOMMAPPLAYER_ADDR_RANGE_DSP_REGISTERS;
          }
        else
          // DspRegister Address Range 2
          if((Address>=0x2080) && (Address<=0x208f))
          {
            Offset = (Address-0x2080);
            *ReturnedPtr = (uint8_t *)&pccpmmAppLayerStruct.DspRegisters + Offset;
            *ReturnedOffsetInRange = Offset;
            ReturnedAddressRange = PCCOMMAPPLAYER_ADDR_RANGE_DSP2_REGISTERS;
          }

          else
            // VMIC 2 Address Range
            if((Address>=0x2024) && (Address<=0x2027))
            {
              // vdbg(">>> 0x2024-0x2027\n");
              Offset = (Address-0x2024);
              *ReturnedPtr = (uint8_t *)&pccpmmAppLayerStruct.VmicRegisters2 + Offset;
              *ReturnedOffsetInRange = Offset;
              ReturnedAddressRange = PCCOMMAPPLAYER_ADDR_RANGE_VMIC2;
            }
            else
            // Board Address Range
            if((Address>=0x2090) && (Address<=0x20bf))
            {
              // vdbg(">>> 0x2090-0x209e\n");
              Offset = (Address-0x2090);
              *ReturnedPtr = NULL; // The pointer is overridden outside for board params
              *ReturnedOffsetInRange = Offset;
              ReturnedAddressRange = PCCOMMAPPLAYER_ADDR_RANGE_BOARD1;
            }
            else
              // VMIC Measurment 1 Buffer
              if((Address>=0x2100) && (Address<=0x21f9))
              {
                // vdbg(">>> 0x2100-0x21f9\n");
                Offset = (Address-0x2100);
                *ReturnedPtr = (uint8_t *)&pccpmmAppLayerStruct.VmicMeasurementBuffer1 + Offset;
                *ReturnedOffsetInRange = Offset;
                ReturnedAddressRange = PCCOMMAPPLAYER_ADDR_RANGE_VMIC_MEASURMENT_1;
              }
              else
                // VMIC Measurment 2 Buffer
                if((Address>=0x2200) && (Address<=0x22f9))
                {
                  // vdbg(">>> 0x2200-0x22f9\n");
                  Offset = (Address-0x2200);
                  *ReturnedPtr = (uint8_t *)&pccpmmAppLayerStruct.VmicMeasurementBuffer2 + Offset;
                  *ReturnedOffsetInRange = Offset;
                  ReturnedAddressRange = PCCOMMAPPLAYER_ADDR_RANGE_VMIC_MEASURMENT_2;
                }
                else 
                  // DSP Samples 1 Address Range
                  if((Address>=0x2300) && (Address<=0x24fe))
                  {
                    // vdbg(">>> 0x2300-0x24fe\n");
                    Offset = (Address-0x2300);
                    *ReturnedOffsetInRange = Offset;
                    *ReturnedPtr = (uint8_t *)&pccpmmAppLayerStruct.DspSampleBuffer1 + (Address-0x2300);
                    ReturnedAddressRange = PCCOMMAPPLAYER_ADDR_RANGE_VMIC_DSP_SAMPLES_1;
                  }
                  else
                    // Generic Buffer
                    if((Address>=0x3000) && (Address<=0x3800))
                    {
                      // vdbg(">>> 0x3000-0x38000\n");
                      Offset = (Address-0x3000);
                      *ReturnedOffsetInRange = Offset;
                      *ReturnedPtr = NULL; // The pointer is overridden outside for generic buffer
                      ReturnedAddressRange = PCCOMMAPPLAYER_ADDR_RANGE_GENERIC_BUFFER;
                    }
                    else
                    {
                      // vdbg(">>> other. addr = 0x%08x\n", Address);
                      ReturnedAddressRange = PCCOMMAPPLAYER_ADDR_RANGE_ERROR;
                    }
  
  return(ReturnedAddressRange);
}


/******************************************************************************
 * @brief  void   pcccommAppLayerBuildResponseMsg()
 * @param  
 * @retval 
 ******************************************************************************/
void   pcccommAppLayerBuildResponseMsg(uint8_t * payloadPtr, uint8_t MsgLength, uint8_t freeMemoryFlag)
{
  uint16_t i;
  uint8_t CheckSum;
  //Check if payload ptr is 0
  if(!payloadPtr)
  {
    if(freeMemoryFlag && (GenericBufferPtr))
    {
      GenericBufferPtr = 0;
      GenericBufferPayLoadLength = 0;
      vPortFree(payloadPtr);
    }
    return;
  }
  
  CheckSum = ResponseBuffer[0] = 'R';
  for(i=0; i<MsgLength; i++)
  {
    ResponseBuffer[1+i] = *(payloadPtr+i);
    CheckSum += *(payloadPtr+i);
  }
  ResponseBuffer[1+MsgLength]= CheckSum;
  // Send the buffer
  
  if(freeMemoryFlag && (GenericBufferPtr))
  {
    GenericBufferPtr = 0;
    GenericBufferPayLoadLength = 0;
    vPortFree(payloadPtr);
  }
  
  
  // Send the buffer to the GUI 
  //hwdriversUart1DmaTx(ResponseBuffer, MsgLength+2);
  CDC_Transmit_HS(ResponseBuffer, MsgLength+2);
  //TM_USB_VCP_Send(ResponseBuffer, MsgLength+2);
  //uartdllTxQueueEnqueue(UARTDLL_UART_1_VTASGUI, ResponseBuffer, MsgLength+2, false);
}


/******************************************************************************
 * @brief  void pcccommAppLayerBleBootloaderWriteResponseMsgBuildCallBack(char* data, uint8_t length)
 * @param  
 * @retval 
 ******************************************************************************/
void pcccommAppLayerBleBootloaderWriteResponseMsgBuildCallBack(char* receivedData, uint8_t receivedlength)
{
  uint8_t CheckSum;
  
  CheckSum = ResponseBuffer[0] = 'L';
  // For now, don't use the receivedLength parameter and assume that the received data is just ACK or NACK
  ResponseBuffer[1] = (uint8_t) receivedData;
  CheckSum += ResponseBuffer[1];
  ResponseBuffer[2] = CheckSum;
  
  // Send the buffer
  // Send the buffer to the GUI 
//  CDC_Transmit_HS(ResponseBuffer, 3);
  //TM_USB_VCP_Send(ResponseBuffer, 3);
  //uartdllTxQueueEnqueue(UARTDLL_UART_1_VTASGUI, ResponseBuffer, 3, false);
  // There is no memory to free because we are using the cPtr which is a const buffer
}

/******************************************************************************
 * @brief  void   pcccommAppLayerSpiFlashWriteResponseMsgBuildCallBack()
 * @param  
 * @retval 
 ******************************************************************************/
void pcccommAppLayerSpiFlashWriteResponseMsgBuildCallBack()
{
  uint8_t CheckSum;
   
  CheckSum = ResponseBuffer[0] = 'M';
  // TODO: add here the result
  ResponseBuffer[1] = 1;
  CheckSum += ResponseBuffer[1];
  ResponseBuffer[2] = CheckSum;
  
  // Send the buffer
  // Send the buffer to the GUI
//  CDC_Transmit_HS(ResponseBuffer, 3);
  //TM_USB_VCP_Send(ResponseBuffer, 3);
  //uartdllTxQueueEnqueue(UARTDLL_UART_1_VTASGUI, ResponseBuffer, 3, false);
  // There is no memory to free because we are using the cPtr which is a const buffer
}

/******************************************************************************
 * @brief  void   pcccommAppLayerSpiFlashReadResponseMsgBuildCallBack()
 * @param  
 * @retval 
 ******************************************************************************/
void   pcccommAppLayerSpiFlashReadResponseMsgBuildCallBack()
{
  uint16_t i;
  uint8_t CheckSum;
  
  uint16_t MsgLength = 0;
  
  CheckSum = ResponseBuffer[0] = 'S';
  for(i=0; i<MsgLength; i++)
  {
    ResponseBuffer[1+i] = *(SpiFlashPageReadAllocatedMemoryPtr+i);
    CheckSum += *(SpiFlashPageReadAllocatedMemoryPtr+i);
  }
  ResponseBuffer[1+MsgLength]= CheckSum;
  // Send the buffer
  // Send the buffer to the GUI 
  //hwdriversUart1DmaTx(ResponseBuffer, MsgLength+2);
//  CDC_Transmit_HS(ResponseBuffer, MsgLength+2);
  //TM_USB_VCP_Send(ResponseBuffer, MsgLength+2);
  //uartdllTxQueueEnqueue(UARTDLL_UART_1_VTASGUI, ResponseBuffer, MsgLength+2, false);
    // Free the buffer allocated for the spiRead request
  if(SpiFlashPageReadAllocatedMemoryPtr)
    vPortFree(SpiFlashPageReadAllocatedMemoryPtr);
}



/******************************************************************************
 * @brief  void pcccommAppLayerInit()
 * @param  
 * @retval 
 ******************************************************************************/
void pcccommAppLayerParametersInit()
{
  pccpmmAppLayerStruct.BoardSystemRegisters.FirmwareVersion.FirmwareVersion.BuildVersion = PCCOMMAPPLAYER_FW_VERSION_BUILD;
  pccpmmAppLayerStruct.BoardSystemRegisters.FirmwareVersion.FirmwareVersion.MinorVersion = PCCOMMAPPLAYER_FW_VERSION_MINOR;
  pccpmmAppLayerStruct.BoardSystemRegisters.FirmwareVersion.FirmwareVersion.MajorVersion = PCCOMMAPPLAYER_FW_VERSION_MAJOR;
  pccpmmAppLayerStruct.BoardSystemRegisters.FirmwareVersion = (FirmwareVersion_T) TYPES_ENDIAN32_CHANGE(pccpmmAppLayerStruct.BoardSystemRegisters.FirmwareVersion.FirmwareVersionValue);
  pccpmmAppLayerStruct.BoardSystemRegisters.BleFirmwareVersion       = TYPES_ENDIAN16_CHANGE(0);
  pccpmmAppLayerStruct.BoardSystemRegisters.BoardHardwareType        = HW_VERSION;
  pccpmmAppLayerStruct.BoardSystemRegisters.ProtocolVersion          = PROTOCOLAPP_PROTOCOL_VERSION;
  pccpmmAppLayerStruct.BoardSystemRegisters.RtcSeconds               = 0;
  pccpmmAppLayerStruct.BoardSystemRegisters.RtcMinutes               = 1;
  pccpmmAppLayerStruct.BoardSystemRegisters.RtcHours                 = 2;
  pccpmmAppLayerStruct.BoardSystemRegisters.RtcDayInWeek             = 3;
  pccpmmAppLayerStruct.BoardSystemRegisters.RtcDay                   = 28;
  pccpmmAppLayerStruct.BoardSystemRegisters.RtcMonth                 = 12;
  pccpmmAppLayerStruct.BoardSystemRegisters.RtcYear                  = 15;
  pccpmmAppLayerStruct.BoardSystemRegisters.RtcValidSignature        = 10;
  pccpmmAppLayerStruct.BoardSystemRegisters.BatteryVoltage           = 11;
  pccpmmAppLayerStruct.BoardSystemRegisters.OnBoardPushButton        = 1;
  pccpmmAppLayerStruct.BoardSystemRegisters.PositionVerification     = 0xff;
  pccpmmAppLayerStruct.BoardSystemRegisters.SupplyVoltage            = 40;
  pccpmmAppLayerStruct.BoardSystemRegisters.SupplyStatus             = 1;
  pccpmmAppLayerStruct.BoardSystemRegisters.BatteryStatus            = 1;
  pccpmmAppLayerStruct.BoardSystemRegisters.ChargingStatus           = 1;
  pccpmmAppLayerStruct.BoardSystemRegisters.BootVersionRam           = 1; //fwupgradeBootloaderVersionGet();
  pccpmmAppLayerStruct.BoardSystemRegisters.RegisterCorruption       = 0;
  
#if 1
  
  pccpmmAppLayerStruct.VmicRegisters1.C2fMeasurementSelect            = 1;
  pccpmmAppLayerStruct.VmicRegisters1.C2fFrequencySelect             = 0;
  pccpmmAppLayerStruct.VmicRegisters1.C2fBinarySwitchArray           = 64;
  pccpmmAppLayerStruct.VmicRegisters1.LowNoiseMeasurement            = 0;
  
  pccpmmAppLayerStruct.VmicRegisters2.AnalogSensorVoltage            = 0;
  pccpmmAppLayerStruct.VmicRegisters2.ExternalRfClockSource          = 1;
  pccpmmAppLayerStruct.VmicRegisters2.ResonanceCapacitor             = 32;
  pccpmmAppLayerStruct.VmicRegisters2.ModulationMode                 = 0;
  
  pccpmmAppLayerStruct.VmicRegisters3.UniqueIdYear                   = 0;
  pccpmmAppLayerStruct.VmicRegisters3.UniqueIdMonth                  = 0;
  pccpmmAppLayerStruct.VmicRegisters3.UniqueIdDay                    = 0;
  pccpmmAppLayerStruct.VmicRegisters3.UniqueIdSerialNumber           = 0;
  pccpmmAppLayerStruct.VmicRegisters3.PowerGood                      = 12;
  pccpmmAppLayerStruct.VmicRegisters3.Status                         = 1;
  pccpmmAppLayerStruct.VmicRegisters3.VmicMeasurementBufferReady     = 0;
  pccpmmAppLayerStruct.VmicRegisters3.RtMeasurement0                  =0;
  pccpmmAppLayerStruct.VmicRegisters3.RtMeasurement1                  =0;
  pccpmmAppLayerStruct.VmicRegisters3.RtMeasurement2                  =0;
  
  pccpmmAppLayerStruct.BeltTransmitterRegisters.TransmitterEnable       = 0;
  pccpmmAppLayerStruct.BeltTransmitterRegisters.Reserved0     = 0;
  pccpmmAppLayerStruct.BeltTransmitterRegisters.BeltStatus         = 28;
  pccpmmAppLayerStruct.BeltTransmitterRegisters.TransmissionCurrentMonitor  = 10;
  
  
  
  pccpmmAppLayerStruct.DspRegisters.AmplifierGain       = 1;
  pccpmmAppLayerStruct.DspRegisters.AutoAmplifierGainSelect                 = 0;
  pccpmmAppLayerStruct.DspRegisters.FeedbackRmsLevel                        = 28;
  pccpmmAppLayerStruct.DspRegisters.FeedbackCarrierFrequency                = 12;
  pccpmmAppLayerStruct.DspRegisters.SignalMonitorMode                       = 0;
  pccpmmAppLayerStruct.DspRegisters.HeaderPositionMsb                          = 0;
  pccpmmAppLayerStruct.DspRegisters.HeaderPositionLsb                          = 0;
  
  pccpmmAppLayerStruct.DspRegisters.CrcErrorCounter                             = 0;
  pccpmmAppLayerStruct.DspRegisters.SampleAverage                              = 0;
  
  pccpmmAppLayerStruct.DspRegisters2.Snr                                        = 0;
  pccpmmAppLayerStruct.DspRegisters2.Link                                       = 0;
  
  
#endif                                               
}                                                       

/******************************************************************************
 * @brief  void pcccommAppLayerSamplesBufferInsert(uint32_t FullMessage)
 * @param  
 * @retval 
 ******************************************************************************/
void pcccommAppLayerSamplesBufferInsert(uint32_t Sample)
{ 
  // Update the samples buffer, MSB First 

////	if(Sample > 35000)
//		Sample = 13*log(Sample-35000);
	switch(pccommAppLayerSampleBufferNumber)
  {
  case PCCOMMAPPLAYER_SAMPLE_BUFFER_1:
    pccpmmAppLayerStruct.VmicMeasurementBuffer1[SamplesIndex*3] = Sample>>16;
    pccpmmAppLayerStruct.VmicMeasurementBuffer1[SamplesIndex*3+1] = Sample>>8;
    pccpmmAppLayerStruct.VmicMeasurementBuffer1[SamplesIndex*3+2] = Sample>>0;
    SamplesIndex++;
    if(SamplesIndex >= 50)
    {
      SamplesIndex=0;
      // Mark the buffer as ready
      pccpmmAppLayerStruct.VmicRegisters3.VmicMeasurementBufferReady = (uint8_t)PCCOMMAPPLAYER_SAMPLE_BUFFER_1;
      // Switch to the alternate buffer
      pccommAppLayerSampleBufferNumber = PCCOMMAPPLAYER_SAMPLE_BUFFER_2;
    }
    break;                                                             
  case PCCOMMAPPLAYER_SAMPLE_BUFFER_2:
    pccpmmAppLayerStruct.VmicMeasurementBuffer2[SamplesIndex*3] = Sample>>16;
    pccpmmAppLayerStruct.VmicMeasurementBuffer2[SamplesIndex*3+1]= Sample>>8;
    pccpmmAppLayerStruct.VmicMeasurementBuffer2[SamplesIndex*3+2]= Sample>>0;
    SamplesIndex++;
    if(SamplesIndex >= 50)
    {
      SamplesIndex=0;
      // Mark the buffer as ready
      pccpmmAppLayerStruct.VmicRegisters3.VmicMeasurementBufferReady = (uint8_t)PCCOMMAPPLAYER_SAMPLE_BUFFER_2;
      // Switch to the alternat buffer
      pccommAppLayerSampleBufferNumber = PCCOMMAPPLAYER_SAMPLE_BUFFER_1;
    }
    break;
  }
  
  //mainCyclesMeasure(pccommAppLayerSampleBufferNumber);  
}

/******************************************************************************
 * @brief  void pcccommAppLayerSamplesIndexReset()
 * @param  
 * @retval 
 ******************************************************************************/
void pcccommAppLayerSamplesIndexReset()
{
  SamplesIndex = 0;
}
/******************************************************************************
 * @brief  void SamplesBufferIndexManagement(uint8_t SamplesBufferIndex)
 * @param  
 * @retval 
 ******************************************************************************/
void SamplesBufferIndexManagement(uint8_t SamplesBufferIndex)
{
  
  switch(pccommAppLayerSampleBufferNumber)
  {
  case PCCOMMAPPLAYER_SAMPLE_BUFFER_1:
    if(pccommAppLayerIndex>= (uint8_t)150) 
    {
      pccommAppLayerIndex=0;
      // Mark the buffer as ready
      pccpmmAppLayerStruct.VmicRegisters3.VmicMeasurementBufferReady = (uint8_t)PCCOMMAPPLAYER_SAMPLE_BUFFER_1;
      // Switch to the alternat buffer
      pccommAppLayerSampleBufferNumber = PCCOMMAPPLAYER_SAMPLE_BUFFER_2;
    }
    break;
  case PCCOMMAPPLAYER_SAMPLE_BUFFER_2:
    if(pccommAppLayerIndex>= (uint8_t)150) 
    {
      pccommAppLayerIndex=0;
      // Mark the buffer as ready
      pccpmmAppLayerStruct.VmicRegisters3.VmicMeasurementBufferReady = (uint8_t)PCCOMMAPPLAYER_SAMPLE_BUFFER_2;
      // Switch to the alternat buffer
      pccommAppLayerSampleBufferNumber = PCCOMMAPPLAYER_SAMPLE_BUFFER_1;
    }
    break;
  default:
    break;
  }
}










/******************************************************************************
 *** @brief  
 * @param  
 * @retval 
 ******************************************************************************/
ReturnCode_T pccommapplayerTransmitterControl(typesControl_T Control)
{
  // Reflect the new state to the GUI
  pccpmmAppLayerStruct.BeltTransmitterRegisters.TransmitterEnable = Control;

  // Charging is not allowed when transmitter is enabled
  if(Control == TYPES_ENABLE)
  {
//    chargerControl( CHARGER_CONTROL_DISABLE);
    hwdriversGpioBitWrite(HWDRIVERS_TX_SUP_ENABLE, TYPES_ENABLE);
    vTaskDelay(15); // Allow VCC_3.0 to rise before enabling clock
    autopowerPwmTemporaryControl(TYPES_ENABLE);
  }
  else
  {
    autopowerPwmTemporaryControl(TYPES_DISABLE);
    hwdriversGpioBitWrite(HWDRIVERS_TX_SUP_ENABLE, TYPES_DISABLE);
//    chargerControl( CHARGER_CONTROL_ENABLE);
  }
  // Clear the CRC counter when the transmitter is turned on
  if(Control)
  {
//    vmicmodemInit();
 //  autopowerInit();
//   autoresonanceInit(AUTORESONANCE_INIT_REQ_AFTER_TX_ON);
//    pccommapplayerAutoResonanceControl(TYPES_ENABLE);
  }
  
  // Reflect the new state to the GUI
  pccpmmAppLayerStruct.BeltTransmitterRegisters.TransmitterEnable = Control;

  if(Control)
  {
    vlapmainDemodulatorTaskControl( VLAPMAIN_TASK_START);
  }
  else
  {
    vlapmainDemodulatorTaskControl( VLAPMAIN_TASK_STOP);
  }

  return(RETURNCODE_ERROR);
}



/******************************************************************************
 *** @brief  
 * @param  
 * @retval 
 ******************************************************************************/
ReturnCode_T pccommapplayerAutoCurrentControl(typesControl_T Control)
{
  // Enable the auto current control
  autopowerControl(Control?TYPES_ENABLE:TYPES_DISABLE);
  // Reflect the new auto current state to the GUI
  pccpmmAppLayerStruct.BeltTransmitterRegisters.AutoCurrentControl = Control;
  return(RETURNCODE_ERROR);
}


/******************************************************************************
 *** @brief  
 * @param  
 * @retval 
 ******************************************************************************/
ReturnCode_T pccommapplayerAutoResonanceControl(typesControl_T Control)
{
  // Enable the autoresonance mechanism   
  autoresonanceControl(Control?AUTORESONANCE_CONTROL_DELAYED_ON:AUTORESONANCE_CONTROL_OFF);
  return(RETURNCODE_ERROR);
}


/******************************************************************************
 *** @brief  
 * @param  
 * @retval 
 ******************************************************************************/
ReturnCode_T pccommapplayerAutoCurrentSet(uint16_t  NewTransmitterCurrentSet)
{
  // Sets the new required transmitter current goal, The automatic current mechanism will try to reach this current goal 
  autopowerTransmitterCurrentGoalSet(NewTransmitterCurrentSet);
  // Infor the GUI about the new currnet goal
  pccpmmAppLayerStruct.BeltTransmitterRegisters.TransmitterCurrentSet = TYPES_ENDIAN16_CHANGE(NewTransmitterCurrentSet);
  // Return 
  return(RETURNCODE_ERROR);
}

/******************************************************************************
 *** @brief  
 * @param  
 * @retval 
 ******************************************************************************/
ReturnCode_T pccommapplayerAutoCurrentGet(uint16_t  *ReturnedTransmitterCurrentSetPtr)
{
  // Sets the new required transmitter current goal, The automatic current mechanism will try to reach this current goal 
  autopowerTransmitterCurrentGoalGet(ReturnedTransmitterCurrentSetPtr);
  // Return 
  return(RETURNCODE_ERROR);
}



/******************************************************************************
 *** @brief  
 * @param  
 * @retval 
 ******************************************************************************/
void pccommnapplayerConnectivityTimerCallback()
{
  pccommnapplayerConnectivityState = PCCOMMAPPLAYER_CONNECTIVITY_STATE_DISCONNECTED;
}


/******************************************************************************
 *** @brief  
 * @param  
 * @retval 
 ******************************************************************************/
void pccommnapplayerConnectivityTimerRetrigger(uint16_t timeout)
{
  // State is connected
  pccommnapplayerConnectivityState = PCCOMMAPPLAYER_CONNECTIVITY_STATE_CONNECTED;
  xTimerChangePeriod( pccommnapplayerConnectivityTimerHandler, timeout, 100);
  // Retrigger the PC connectivity timer
  //xTimerReset( pccommnapplayerConnectivityTimerHandler, 100);
}


/******************************************************************************
 *** @brief  
 * @param  
 * @retval 
 ******************************************************************************/
pccommnapplayerConnectivityState_T pccommnapplayerConnectivityStateGet()
{
   return(pccommnapplayerConnectivityState);
}


void pcccommAppLayerSpiFlashWriteResponseCRC32CallBack(SPIMemoryCommand_T memoryType, uint8_t result)
{
  GenericBufferPtr = pvPortMalloc(2);
  GenericBufferPayLoadLength = 2;
  
  GenericBufferPtr[0] = (uint8_t) memoryType;
  GenericBufferPtr[1] = result;
}

