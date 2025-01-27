#ifndef PCCOMMAPPLAYER_H
#define PCCOMMAPPLAYER_H


#include <stdint.h>
#include "vlapConfig.h"
#include "timers.h"
#include "vmicapplayer.h"




// D E F I N I T I O N S 
// Application Layer message definitions
#define MSG_OFFSET_CMD          0
#define MSG_OFFSET_ADDRESS_MSB  1
#define MSG_OFFSET_ADDRESS_LSB  2
#define MSG_OFFSET_LENGTH       3
#define MSG_OFFSET_PAYLOAD      4

// T Y P E S
// Address Range 
typedef  enum {PCCOMMAPPLAYER_ADDR_RANGE_ERROR, PCCOMMAPPLAYER_ADDR_RANGE_BOARD, PCCOMMAPPLAYER_ADDR_RANGE_BOARD1, PCCOMMAPPLAYER_ADDR_RANGE_VMIC1, PCCOMMAPPLAYER_ADDR_RANGE_VMIC2, PCCOMMAPPLAYER_ADDR_RANGE_VMIC3,
               PCCOMMAPPLAYER_ADDR_RANGE_BELT_TRANSMITTER,PCCOMMAPPLAYER_ADDR_RANGE_DSP_REGISTERS, PCCOMMAPPLAYER_ADDR_RANGE_DSP2_REGISTERS,
               PCCOMMAPPLAYER_ADDR_RANGE_VMIC_MEASURMENT_1,PCCOMMAPPLAYER_ADDR_RANGE_VMIC_MEASURMENT_2,
               PCCOMMAPPLAYER_ADDR_RANGE_VMIC_DSP_SAMPLES_1 , PCCOMMAPPLAYER_ADDR_RANGE_GENERIC_BUFFER} AddrRange_T;

typedef  enum {PCCOMMAPPLAYER_TXREG_TRANS_ENABLE, PCCOMMAPPLAYER_TXREG_AUTO_RESONANCE_CNTL,  PCCOMMAPPLAYER_TXREG_AUTO_CURRENT_CNTL, PCCOMMAPPLAYER_TXREG_TRANS_CURRENT_SET, PCCOMMAPPLAYER_TXREG_TRANS_OUTPUT_SET  } TransmitterRegisters_T;

typedef enum {PCCOMMAPPLAYER_WRITE_CONFIG = 0, PCCOMMAPPLAYER_READ_CONFIG = 1, PCCOMMAPPLAYER_WRITE_PRODUCTION = 2, PCCOMMAPPLAYER_READ_PRODUCTION = 3, PCCOMMAPPLAYER_READ_EVENTS_LOG_MEMORY = 4, PCCOMMAPPLAYER_READ_RAM_MEMORY = 5, PCCOMMAPPLAYER_READ_EXTERNAL_ID = 6, PCCOMMAPPLAYER_SET_KEY = 7, PCCOMMAPPLAYER_READ_SPI_FUG_SET_RESULT = 8  } MemoryCommand_T;

typedef enum {PCCOMMAPPLAYER_BACKUP_ADDRESS = 0, PCCOMMAPPLAYER_UPGRADE_ADDRESS = 1 } SPIMemoryCommand_T;

typedef enum{ 
  MEASSLCT_C2F_POWER_DOWN=0, 
  MEASSLCT_REAL_MEMS_WITH_NOISE_COMPENSATION=1, 
  MEASSLCT_REAL_MEMS_WITHOUT_NOISE_COMPENSATION=2, 
  MEASSLCT_REF_MEMS_WITH_NOISE_COMPENSATION=3, 
  MEASSLCT_REF_MEMS_WITHOUT_NOISE_COMPENSATION=4, 
  MEASSLCT_INTERNAL_REFERENCE_CAPACITOR=5,
  MEASSLCT_TEMPERATURE=6, 
  MEASSLCT_CALIBRATED_MEAS=7, 
  MEASSLCT_ID_LSB=8, 
  MEASSLCT_ID_MSB=9,
  MEASSLCT_CONFIGURATION_REGISTER10=10,
  MEASSLCT_CONFIGURATION_REGISTER11=11
    
} MeasurmentSelect_T;

typedef enum{ DSP_SAMPLES_SOURCE_NONE=0, DSP_SAMPLES_SOURCE_DSP0=1, DSP_SAMPLES_SOURCE_DSP1=2, DSP_SAMPLES_SOURCE_AVERAGE=3, DSP_SAMPLES_SOURCE_HEADER=4, DSP_SAMPLES_SOURCE_NOISE=5
} DspSamplesBufferSourceSelect_T;



typedef enum {
  PCCOMMAPPLAYER_VMIC_TX_REQUEST_IDLE, 
  PCCOMMAPPLAYER_VMIC_TX_REQUEST_PENDING,
  PCCOMMAPPLAYER_VMIC_TX_REQUEST_DELAY_WAIT, 
  PCCOMMAPPLAYER_VMIC_SECOND_TX_REQUEST_WAIT 
} pccpmmAppLayerPendingTx_T;


typedef  enum  {PCCOMMAPPLAYER_HW_TYPE_NOT_DEFINED, PCCOMMAPPLAYER_HW_TYPE_EVB, PCCOMMAPPLAYER_HW_TYPE_EXT_DEVICE} HwType_T;
#pragma pack(1)
typedef  struct
{
#pragma pack(1)
	struct
  {
    FirmwareVersion_T  FirmwareVersion;
    uint8_t   BoardHardwareType;
    uint8_t   RtcSeconds;
    uint8_t   RtcMinutes;
    uint8_t   RtcHours;
    uint8_t   RtcDayInWeek;
    uint8_t   RtcDay;
    uint8_t   RtcMonth;
    uint8_t   RtcYear;
    uint8_t   RtcValidSignature;
    uint16_t  BatteryVoltage;
    uint8_t   OnBoardPushButton;
    uint8_t   PositionVerification;
    uint8_t   ProtocolVersion;
    uint16_t BleFirmwareVersion;
    uint8_t Reserved[3];
    uint16_t  SupplyVoltage;
    uint8_t   SupplyStatus;
    uint8_t   BatteryStatus;
    uint8_t   MeasurmentStage;
    uint8_t   MeasurmentTimer;
    uint8_t   ChargingStatus;
    uint8_t   BootVersionRam;
    uint8_t   RegisterCorruption;
  } BoardSystemRegisters;

 
  
  // To be sent as "In Data destination selector 2" message type with ExtDeviceToVmicStatusUnion2_T structure
#pragma pack(1)
  struct
  {
    uint8_t   C2fMeasurementSelect;
    uint8_t   C2fFrequencySelect;
    uint8_t   C2fBinarySwitchArray;
    uint8_t   LowNoiseMeasurement;
  } VmicRegisters1;

  // To be sent as "In Data destination selector 3" message type with ExtDeviceToVmicStatusUnion3_T structure
#pragma pack(1)
  struct
  {
    uint8_t   AnalogSensorVoltage;
    uint8_t   ExternalRfClockSource;
    uint8_t   ResonanceCapacitor;
    uint8_t   ModulationMode;
  } VmicRegisters2;

  // To be filled when data is received from the VMIC
#pragma pack(1)
  struct
  {
    uint8_t   UniqueIdYear;
    uint8_t   UniqueIdMonth;
    uint8_t   UniqueIdDay;
    uint16_t  UniqueIdSerialNumber;
    uint8_t   PowerGood;
    uint8_t   Status;
    uint8_t   VmicMeasurementBufferReady; 
    uint8_t  RtMeasurement2;
    uint8_t  RtMeasurement1;
    uint8_t  RtMeasurement0;
    uint8_t  SeqTemperatureMeasurment2;
    uint8_t  SeqTemperatureMeasurment1;
    uint8_t  SeqTemperatureMeasurment0;
    uint8_t  SeqInternalCapMeasurment2;
    uint8_t  SeqInternalCapMeasurment1;
    uint8_t  SeqInternalCapMeasurment0;
    uint8_t  SeqReferenceMeasurment2;
    uint8_t  SeqReferenceMeasurment1;
    uint8_t  SeqReferenceMeasurment0;
    uint8_t  SeqMemsMeasurment2;
    uint8_t  SeqMemsMeasurment1;
    uint8_t  SeqMemsMeasurment0;
    uint8_t  Reserved[1];
  } VmicRegisters3;

#pragma pack(1)
  struct
  {
    uint8_t   TransmitterEnable;
    uint8_t   AutoResonanceCtrl;
    uint8_t   AutoCurrentControl;
    uint16_t   TransmitterCurrentSet;
    uint8_t   Reserved0;
    uint8_t   BeltStatus;
    uint16_t  TransmissionCurrentMonitor;
    uint8_t       RelayStateMsb;
    uint8_t       RelayStateLsb;
    uint16_t      CapacitorBank;
    uint16_t  BeltInductance;
    uint16_t  EffectiveBeltResistance;
    uint8_t       LoadDefault;
    uint8_t       Spare;
    uint8_t       BeltStatusForImage;
    uint8_t       BeltInteriorColor;
    uint8_t       BeltSize;
    uint8_t     PulseSkipLevel;
    uint8_t       Reserved1[9];
  } BeltTransmitterRegisters;

#pragma pack(1)
  struct
  {
    uint8_t   AmplifierGain;
    uint8_t   AutoAmplifierGainSelect;
    uint16_t  FeedbackRmsLevel;
    uint16_t  FeedbackCarrierFrequency;
    uint8_t   Reserved1;
    uint8_t   SignalMonitorMode;
    uint8_t   GraphDispOffsetMsb;
    uint8_t   GraphDispOffsetLsb;
    uint8_t   HeaderPositionMsb;
    uint8_t   HeaderPositionLsb;
    uint16_t      CrcErrorCounter;
    uint8_t       SampleAverage;
    uint8_t       Reserved2[1];
  } DspRegisters;
  
#pragma pack(1)
  struct
  {
    uint8_t   Spare[16];  
  } BootloaderRegisters;
  
   
#pragma pack(1)
  struct
  {
    uint16_t  Snr;
    uint8_t   Link;
    uint8_t   Spare[13];
  } DspRegisters2;

  
#pragma pack(1)
  struct
  {
    uint8_t  Debug0M;
    uint8_t  Debug0L;
    uint8_t  Debug1M;
    uint8_t  Debug1L;
    uint8_t  Debug2M;
    uint8_t  Debug2L;
    uint16_t  ImplantPressure;
    uint16_t  ImplantPrFactor;
    uint8_t   ImplantPrNull;
    uint8_t   Spare;
    uint16_t  CalibMeasOffset;
    uint16_t  ExternalPressure;
    uint16_t  ExternalTemperature;
    uint16_t  ImplantTemperature;
    uint8_t   PationId[16];
    uint8_t  AcrDacValueM;
    uint8_t  AcrDacValueL;
    uint8_t  PressureSensorOffsetM;
    uint8_t  PressureSensorOffsetL;
    uint8_t  ProductionConfigSave;  // Command
    uint8_t  ApplicationConfigSave; // Command
    uint8_t  NtcPosM;
    uint8_t  NtcPosL;
    uint8_t  NtcNegM;
    uint8_t  NtcNegL;
    uint8_t  ModemRxPtpM;       // mV
    uint8_t  ModemRxPtpL;
    int16_t  AccX;
    int16_t  AccY;
    int16_t  AccZ;
   } Board1SystemRegisters;
   
  
  uint8_t VmicMeasurementBuffer1[250];
  uint8_t VmicMeasurementBuffer2[250];
  uint8_t DspSampleBuffer1[512];
} pccpmmAppLayerStruct_T;

typedef enum{
  PCCOMMAPPLAYER_CONNECTIVITY_STATE_CONNECTED=0, 
  PCCOMMAPPLAYER_CONNECTIVITY_STATE_DISCONNECTED=1
} pccommnapplayerConnectivityState_T;

// Sample Buffers Management
typedef  enum  {PCCOMMAPPLAYER_SAMPLE_BUFFER_1=1, PCCOMMAPPLAYER_SAMPLE_BUFFER_2=2
} pccommAppLayerSampleBufferNumber_T;


// P R O T O T Y P E S 
void    pcccommAppLayerProcessing( uint8_t * ptr, uint16_t Length);
uint32_t  vmicapplayerStructToVmicConvert();
void    pcccommAppLayerInit();
void SamplesBufferIndexManagement(uint8_t SamplesBufferIndex);
void pcccommAppLayerParametersInit();
void pcccommAppLayerSamplesBufferInsert(uint32_t FullMessage);
void  pccpmmAppLayerEndOfTxCallBack(TimerHandle_t pxExpiredTimer);
void pcccommAppLayerInit();
ReturnCode_T pccommapplayerTransmitterControl(typesControl_T Control);
ReturnCode_T pccommapplayerAutoCurrentControl(typesControl_T Control);
ReturnCode_T pccommapplayerAutoResonanceControl(typesControl_T Control);
ReturnCode_T pccommapplayerAutoCurrentSet(uint16_t  NewTransmitterCurrentSet);
uint32_t vmicapplayerUnFilteredSamplesGet();
pccommnapplayerConnectivityState_T pccommnapplayerConnectivityStateGet();
ReturnCode_T pcccommAppLayerCopyMemoryFromRam(uint8_t* sourceAddr, uint8_t *destinationAddr, uint32_t len);
void pcccommAppLayerSamplesIndexReset();




// E X T E R N A L S 

// Global GUI application layer structure, represents the External Device regiates and buffers
// as union of structures
extern pccpmmAppLayerStruct_T  pccpmmAppLayerStruct;
// Sample bufer Management
extern pccommAppLayerSampleBufferNumber_T pccommAppLayerSampleBufferNumber;
extern uint8_t pccommAppLayerIndex;
extern pccpmmAppLayerPendingTx_T pccpmmAppLayerPendingTx;
extern vmicapplayerVmicTxSourceSelectCommandT pccpmmAppLayerNewVmicSourceAfterTx;

#endif
