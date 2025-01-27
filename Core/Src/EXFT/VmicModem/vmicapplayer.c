#include "vlapConfig.h"
#include "vmicapplayer.h"
#include "pccommapplayer.h"
#include "dac.h"
#include "measure.h"
uint32_t BadFramesCounter  		__attribute__( ( section( ".noinit") ) ) ;
uint32_t SamplesFramesCounter  	__attribute__( ( section( ".noinit") ) );
uint32_t StatusFramesCounter  	__attribute__( ( section( ".noinit") ) );
uint32_t TotalFramesCounter 	 __attribute__( ( section( ".noinit") ) );

#define VMICAPPLAYER_PEAK_TO_PEAK_ARRAY_LENGTH       20

VmicToExtDeviceStatusUnion0_T VmicToExtDeviceStatusUnion0;
VmicToExtDeviceStatusUnion1_T VmicToExtDeviceStatusUnion1;
VmicToExtDeviceStatusUnion2_T VmicToExtDeviceStatusUnion2;
VmicToExtDeviceStatusUnion3_T VmicToExtDeviceStatusUnion3;

ExtDeviceToVmicDataUnion2_T ExtDeviceToVmicDataUnion2;
ExtDeviceToVmicDataUnion3_T ExtDeviceToVmicDataUnion3;

uint32_t FilteredSamples;
uint32_t UnfilteredSample;
uint32_t vmicapplayerSignalPeakToPeak;

uint32_t PtpCircularBuffer[VMICAPPLAYER_PEAK_TO_PEAK_ARRAY_LENGTH];
uint8_t PtpArrayHeadIndex;


/******************************************************************************
 * @brief  void vmicapplayerRx(uint8_t MessageFailed, uint32_t FullMessage)
 * @param  
 * @retval 
 ******************************************************************************/
void vmicapplayerRx(uint8_t MessageFailed, uint32_t FullMessage)
{
  volatile uint32_t PtpArray[VMICAPPLAYER_PEAK_TO_PEAK_ARRAY_LENGTH];
  uint8_t i, j;
  uint32_t MaxLocalSample;
  uint32_t MinLocalSample;
  uint32_t Temp;
  
  TotalFramesCounter++;
  

  if(MessageFailed)
  {
    BadFramesCounter ++;
    // Reset the Head and Tail index
    PtpArrayHeadIndex = 0;
  }
  else
  {
    if(!(FullMessage & VMICAPPLAYER_MEASURE_STATUS_INDICATION_MASK))
    {
      SamplesFramesCounter++;
      // Update the RtMeasurement
      pccpmmAppLayerStruct.VmicRegisters3.RtMeasurement0  = ((FullMessage & VMICAPPLAYER_MEASUREMENT_MASK)>> VMICAPPLAYER_MEASUREMENT_RIGHT_SHIFT)>>0;
      pccpmmAppLayerStruct.VmicRegisters3.RtMeasurement1  = ((FullMessage & VMICAPPLAYER_MEASUREMENT_MASK)>> VMICAPPLAYER_MEASUREMENT_RIGHT_SHIFT)>>8;
      pccpmmAppLayerStruct.VmicRegisters3.RtMeasurement2  = ((FullMessage & VMICAPPLAYER_MEASUREMENT_MASK)>> VMICAPPLAYER_MEASUREMENT_RIGHT_SHIFT)>>16;
      // Insert the new sample into the relevant buffer (Dual buffer)
      pcccommAppLayerSamplesBufferInsert(((FullMessage & VMICAPPLAYER_MEASUREMENT_MASK)>> VMICAPPLAYER_MEASUREMENT_RIGHT_SHIFT));
      UnfilteredSample = ((FullMessage & VMICAPPLAYER_MEASUREMENT_MASK)>> VMICAPPLAYER_MEASUREMENT_RIGHT_SHIFT);
      FilteredSamples = (((FilteredSamples*30)/100) + (UnfilteredSample*70)/100);
      // Inser sample to the Peak to Peak circular buffer
      PtpCircularBuffer[PtpArrayHeadIndex] = UnfilteredSample;
      // Handle circular head index edge circulation
      if(PtpArrayHeadIndex < VMICAPPLAYER_PEAK_TO_PEAK_ARRAY_LENGTH - 1)
        PtpArrayHeadIndex++;
      else
        PtpArrayHeadIndex = 0;
      /// Build local linear buffer from the circular buffer 
      // Copy from head index till circular buffer begining 
      for(i=PtpArrayHeadIndex, j=0; i; i--, j++)
        PtpArray[j] = PtpCircularBuffer[i];
      // Copy from end of circular index till the head index
      for(i=(VMICAPPLAYER_PEAK_TO_PEAK_ARRAY_LENGTH-1); i > PtpArrayHeadIndex; i--, j++)
        PtpArray[j] = PtpCircularBuffer[i];
      // Find max and min in the local buffer
      arm_max_q31(PtpArray, VMICAPPLAYER_PEAK_TO_PEAK_ARRAY_LENGTH-1, &MaxLocalSample, &Temp);
      arm_min_q31(PtpArray, VMICAPPLAYER_PEAK_TO_PEAK_ARRAY_LENGTH-1, &MinLocalSample, &Temp);
      // Calculate Peak to Peak value 
      vmicapplayerSignalPeakToPeak = MaxLocalSample - MinLocalSample;
      measureTaskEventSend(MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_SAMPLE, MEASURE_TASK_QUEUE_SEND_SOURCE_NORMAL); 
    }
    else
    {
      StatusFramesCounter++;
      switch((FullMessage & VMICAPPLAYER_STATUS_SOURCE_MASK)>>VMICAPPLAYER_STATUS_SOURCE_RIGHT_SHIFT)
      {
      case VMICAPPLAYER_STATUS_SOURCE_0:
        VmicToExtDeviceStatusUnion0.Status0 =(FullMessage & VMICAPPLAYER_STATUS_PAYLOAD_MASK)>>VMICAPPLAYER_STATUS_PAYLOAD_RIGHT_SHIFT;
        pccpmmAppLayerStruct.VmicRegisters3.UniqueIdSerialNumber = TYPES_ENDIAN16_CHANGE(VmicToExtDeviceStatusUnion0.Status0 /*+ 256*VmicToExtDeviceStatusUnion1.Status1*/);
        measureTaskEventSend(MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_STATUS_SOURCE_0, MEASURE_TASK_QUEUE_SEND_SOURCE_NORMAL); 
        break;
      case VMICAPPLAYER_STATUS_SOURCE_1:
        VmicToExtDeviceStatusUnion1.Status1 =(FullMessage & VMICAPPLAYER_STATUS_PAYLOAD_MASK)>>VMICAPPLAYER_STATUS_PAYLOAD_RIGHT_SHIFT;
        //pccpmmAppLayerStruct.VmicRegisters3.UniqueIdSerialNumber = TYPES_ENDIAN32_CHANGE(VmicToExtDeviceStatusUnion0.Status0 + 256*VmicToExtDeviceStatusUnion1.Status1);
        measureTaskEventSend(MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_STATUS_SOURCE_1, MEASURE_TASK_QUEUE_SEND_SOURCE_NORMAL); 
        break;
      case VMICAPPLAYER_STATUS_SOURCE_2:
        VmicToExtDeviceStatusUnion2.Status2 =(FullMessage & VMICAPPLAYER_STATUS_PAYLOAD_MASK)>>VMICAPPLAYER_STATUS_PAYLOAD_RIGHT_SHIFT;
        break;
      case VMICAPPLAYER_STATUS_SOURCE_3:
        VmicToExtDeviceStatusUnion3.Status3 =(FullMessage & VMICAPPLAYER_STATUS_PAYLOAD_MASK)>>VMICAPPLAYER_STATUS_PAYLOAD_RIGHT_SHIFT;
        pccpmmAppLayerStruct.VmicRegisters3.PowerGood = VmicToExtDeviceStatusUnion3.StatusBits3.PowerGood;
        break; 
      default:
        break;
      }
    }
  }
  
  
}

void vmicapplayerInitializeBadFramesCounter()
{
  BadFramesCounter = 0;
}
uint32_t vmicapplayerGetBadFramesCounter()
{
  return BadFramesCounter;
}

uint32_t vmicapplayerFilteredSamplesGet()
{
  return(FilteredSamples);
}

uint32_t vmicapplayerUnFilteredSamplesGet()
{
  return(UnfilteredSample);
}

uint32_t vmicapplayerSignalPeakToPeakValueGet()
{
  return(vmicapplayerSignalPeakToPeak);
}


uint32_t vmicapplayerIdGet()
{
  return(VmicToExtDeviceStatusUnion0.Status0 /*+ 256*VmicToExtDeviceStatusUnion1.Status1*/);
}
 
