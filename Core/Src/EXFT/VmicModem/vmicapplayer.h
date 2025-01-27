#ifndef VMICAPPLAYER_H
#define VMICAPPLAYER_H



// D E F I N I T I O N S 



// From VMIC to External Device Payload data structure

#pragma pack(1)
typedef  union
{
  struct {
    uint16_t  UniqueIdSnBit0:1;
    uint16_t  UniqueIdSnBit1:1;
    uint16_t  UniqueIdSnBit2:1;
    uint16_t  UniqueIdSnBit3:1;
    uint16_t  UniqueIdSnBit4:1;
    uint16_t  UniqueIdSnBit5:1;
    uint16_t  UniqueIdSnBit6:1;
    uint16_t  UniqueIdSnBit7:1;
    uint16_t  UniqueIdSnBit8:1;
    uint16_t  UniqueIdSnBit9:1;
    uint16_t  UniqueIdSnBit10:1;
    uint16_t  UniqueIdSnBit11:1;
    uint16_t  UniqueIdSnBit12:1;
    uint16_t  UniqueIdSnBit13:1;
    uint16_t  UniqueIdSnBit14:1;
    uint16_t  UniqueIdSnBit15:1;
  } StatusBits0;
  uint16_t Status0;
} VmicToExtDeviceStatusUnion0_T;

#pragma pack(1)
  typedef  union
{
  struct {
    uint16_t  UniqueIdSnBit16:1;
    uint16_t  UniqueIdSnBit17:1;
    uint16_t  UniqueIdSnBit18:1;
    uint16_t  UniqueIdSnBit19:1;
    uint16_t  UniqueIdSnBit20:1;
    uint16_t  UniqueIdSnBit21:1;
    uint16_t  UniqueIdSnBit22:1;
    uint16_t  UniqueIdSnBit23:1;
    uint16_t  UniqueIdSnBit24:1;
    uint16_t  UniqueIdSnBit25:1;
    uint16_t  UniqueIdSnBit26:1;
    uint16_t  UniqueIdSnBit27:1;
    uint16_t  UniqueIdSnBit28:1;
    uint16_t  UniqueIdSnBit29:1;
    uint16_t  UniqueIdSnBit30:1;
    uint16_t  UniqueIdSnBit31:1;
  } StatusBits0;
  uint16_t Status1;
} VmicToExtDeviceStatusUnion1_T;


#pragma pack(1)
typedef  union
{
  struct {
    uint16_t  C2fLowNoiseMode:1;
    uint16_t  C2fGeneralC2fPower:1;
    uint16_t  C2fBankSelect:7;
    uint16_t  C2fREFMemsSelect:1;
    uint16_t  C2fRealMemsSelect:1;
    uint16_t  C2fNoiseCompByRefMems:1;
    uint16_t  C2fNoiseCompByRealMEMS:1;
    uint16_t  C2fTemperatureSelect:1;
    uint16_t  C2fFrequencySelect:1;
    uint16_t  C2fNotInUse:1;
   } StatusBits2;
  uint16_t Status2;
} VmicToExtDeviceStatusUnion2_T;


#pragma pack(1)
typedef  union
{
  struct {
    uint16_t  NotInUse1:1;
    uint16_t  ModulationMode:1;
    uint16_t  ResonanceCapacitorBit:6;
    uint16_t  RFsyncEnable:1;
    uint16_t  AnalogVoltageSelectBit0:1;
    uint16_t  AnalogVoltageSelectBit1:1;
    uint16_t  AnalogVoltageSelectBit2:1;
    uint16_t  NotInUse2:3;
    uint16_t  PowerGood:1;
  } StatusBits3;
  uint16_t Status3;
} VmicToExtDeviceStatusUnion3_T;


// From External Device to VMIC Payload data structure
#pragma pack(1)
typedef  union
{
  struct {
    uint32_t  C2fLowNoiseMode:1;
    uint32_t  C2fGeneralC2fPower:1;
    uint32_t  C2fBankSelect:7;
    uint32_t  C2fREFMemsSelect:1;
    uint32_t  C2fRealMemsSelect:1;
    uint32_t  C2fNoiseCompByRefMems:1;
    uint32_t  C2fNoiseCompByRealMEMS:1;
    uint32_t  C2fTemperatureSelect:1;
    uint32_t  C2fFrequencySelect:1;
    uint32_t  C2fNotInUse:1;
    uint32_t  Dst0:1;
    uint32_t  Dst1:1;
    uint32_t  StatusMeasure:1;
  } DataBits2;
  uint32_t Data2;
} ExtDeviceToVmicDataUnion2_T;


//#pragma pack(1)
//typedef  union
//{
//  struct {
//    uint32_t  NotInUse5:1;
//    uint32_t  ModulationMode:1;
//    uint32_t  ResonanceCapacitor:6;
//    uint32_t  RFsyncEnable:1;
//    uint32_t  C2fNotInUse4:4;
//    uint32_t  C2fNotInUse3:3;
//    uint32_t  AnalogVoltageSelect:3;
//    uint32_t  C2fNotInUse2:2;
//    uint32_t  C2fNotInUse1:1;
//    uint32_t  Dst0:1;
//    uint32_t  Dst1:1;
//    uint32_t  StatusMeasure:1;
//  } DataBits3;
//  uint32_t Data3;
//} ExtDeviceToVmicDataUnion3_T;

#pragma pack(1)
typedef  union
{
  struct {
    uint32_t  NotInUse5:1;
    uint32_t  ModulationMode:1;
    uint32_t  ResonanceCapacitor:6;
    uint32_t  RFsyncEnable:1;
    uint32_t  AnalogVoltageSelect:3;
    uint32_t  C2fNotInUse1:4;
    uint32_t  Dst0:1;
    uint32_t  Dst1:1;
    uint32_t  StatusMeasure:1;
  } DataBits3;
  uint32_t Data3;
} ExtDeviceToVmicDataUnion3_T;



#define         VMICAPPLAYER_MEASURE_STATUS_INDICATION_MASK     (uint32_t)0x02000000
#define         VMICAPPLAYER_MEASURE_STATUS_INDICATION_SHIFT    (uint8_t)7

#define         VMICAPPLAYER_STATUS_SOURCE_MASK                  (uint32_t)0x03800000
#define         VMICAPPLAYER_STATUS_SOURCE_RIGHT_SHIFT           (uint8_t)23
#define         VMICAPPLAYER_STATUS_SOURCE_0                     (uint8_t)4
#define         VMICAPPLAYER_STATUS_SOURCE_1                     (uint8_t)5
#define         VMICAPPLAYER_STATUS_SOURCE_2                     (uint8_t)6
#define         VMICAPPLAYER_STATUS_SOURCE_3                     (uint8_t)7
// For VMICAPPLAYER_MEASURE_STATUS_INDICATION_MASK == 0 (Measurment)
#define         VMICAPPLAYER_MEASUREMENT_MASK                   (uint32_t)0x01ffff80
#define         VMICAPPLAYER_MEASUREMENT_RIGHT_SHIFT            (uint8_t)7
// For VMICAPPLAYER_MEASURE_STATUS_INDICATION_MASK == 1
// For VMICAPPLAYER_STATU_SOURCE_0, 1, 2, 3
#define         VMICAPPLAYER_STATUS_PAYLOAD_MASK                (uint32_t)0x007fff80
#define         VMICAPPLAYER_STATUS_PAYLOAD_RIGHT_SHIFT           (uint8_t)7


typedef enum { VMICAPPLAYER_VMIC_SELECT_C2F_MEASUREMENT_COMMAND=(uint32_t)0x00000, 
                VMICAPPLAYER_VMIC_SELECT_ID_LSB_COMMAND=(uint32_t)0x40000,                 
                VMICAPPLAYER_VMIC_SELECT_ID_MSB_COMMAND=(uint32_t)0x50000,                 
                VMICAPPLAYER_VMIC_SELECT_SOURCE_1_COMMAND=(uint32_t)0x60000,
                VMICAPPLAYER_VMIC_SELECT_SOURCE_2_COMMAND=(uint32_t)0x70000,
                VMICAPPLAYER_VMIC_SELECT_INTERNAL_CAP_MEASURMENT_SELECT_COMMAND=(uint32_t)0x40102 ,
                VMICAPPLAYER_VMIC_SELECT_TEMPERATURE_MEASURMENT_SELECT_COMMAND=(uint32_t)0x62102, 
                VMICAPPLAYER_VMIC_SELECT_REAL_MEMS_MEASURMENT_SELECT_COMMAND=(uint32_t)0x60302,
                VMICAPPLAYER_VMIC_DO_NOT_SELECT_NEW_SOURCE_AFTER_TX=(uint32_t)0xfffff,
                } vmicapplayerVmicTxSourceSelectCommandT;                 


extern VmicToExtDeviceStatusUnion0_T VmicToExtDeviceStatusUnion0;
extern VmicToExtDeviceStatusUnion1_T VmicToExtDeviceStatusUnion1;
extern VmicToExtDeviceStatusUnion2_T VmicToExtDeviceStatusUnion2;
extern VmicToExtDeviceStatusUnion3_T VmicToExtDeviceStatusUnion3;

extern ExtDeviceToVmicDataUnion2_T ExtDeviceToVmicDataUnion2;
extern ExtDeviceToVmicDataUnion3_T ExtDeviceToVmicDataUnion3;

void vmicapplayerRx(uint8_t MessageStatus, uint32_t FullMessage);
uint32_t vmicapplayerFilteredSamplesGet();
uint32_t vmicapplayerIdGet();
void vmicapplayerInitializeBadFramesCounter();
uint32_t vmicapplayerGetBadFramesCounter();


#endif
