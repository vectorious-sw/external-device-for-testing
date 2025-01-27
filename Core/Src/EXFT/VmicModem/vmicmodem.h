#pragma once

#include "General.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "pccommAppLayer.h"

#define NET_TRANSACTION(BITS_32_OPERAND) ((BITS_32_OPERAND & 0x03FFFFFF) >> CRC_LEN)//Remove header, shift out CRC bits


typedef WORD_16_BITS  SampleElement;
typedef INT_16_BITS NoiseElement;
typedef int16_t  CorrelationElement;     
typedef unsigned  VmicModemId;
typedef WORD_32_BITS  ModemPayload;   
typedef WORD_32_BITS  ModemTotalTransaction;
extern bool vmicmodemBeltMasking;

typedef enum {STATUS_SUCCESSFUL, STATUS_FAILURE_MODEM_ACTIVITY, STATUS_FAILURE_RESOURCES, STATUS_FAILURE_PARAM, STATUS_FAILURE_WRONG_CRC, STATUS_BUSY,  STATUS_ERROR_TX_CONTEXT, STATUS_PREMATURE}ModemActivityStatus;
typedef enum {VMIC_TRANSACTION_ACCEPTED, VMIC_COMMUNICATION_FAILURE}VmicEvent;
typedef enum {VMIC_DIAG_RAW_SAMPLES, VMIC_AVERAGE_BUFFER, VMIC_DIAG_CORRELATION_RESULTS, VMIC_DIAG_NOISE_RESULTS}VmicDiagIndicator;

typedef enum { VMICMODEM_VMIC_CONFIG_WORD_0, VMICMODEM_VMIC_CONFIG_WORD_1, VMICMODEM_VMIC_CONFIG_WORD_2, VMICMODEM_VMIC_CONFIG_WORD_3} vmicmodemConfigWordSelectT;

typedef enum {  
                VMICMODEM_SELECT_POWER_DOWN_SELECT=0,
                VMICMODEM_SELECT_NOISE_COMP_BY_REAL_MEMS_SELECT=1,
                VMICMODEM_SELECT_REAL_MEMS_SELECT=2,
                VMICMODEM_SELECT_NOISE_COMP_BY_REF_MEMS_SELECT=3,
                VMICMODEM_SELECT_REF_MEMS_SELECT=4, 
                VMICMODEM_SELECT_INTERNAL_CAP_SELECT=5, 
                VMICMODEM_SELECT_TEMPRATURE_SELECT=6,
                VMICMODEM_SELECT_FREQUENCY_SELECT=7
              } vmicmodemVmicTxSourceSelectCommandT;                 



VmicModemId VmicModemInit(void); 
ModemActivityStatus VmicModemDemodulator(VmicModemId vmicModemId, SampleElement samplesBuffer[], unsigned missedBuffers,  ModemTotalTransaction* pDecodedStream, unsigned* pHeaderPosition);  // activate VMIC modem demodulator with fresh buffer
void VmicRxEventsCallback(VmicModemId modemId, VmicEvent eventType, ModemTotalTransaction vmicTransaction, ModemPayload vmicNetTransaction);
void vlapDemodulatorTxReq( vmicmodemConfigWordSelectT ConfigWordSelect , void (*AppEndOfSourceChangeFunctionPtr)());
ModemActivityStatus VmicModemTx(VmicModemId vmicModemId);        
ModemActivityStatus VmicGetDiagInfo(VmicModemId vmicModemId, VmicDiagIndicator   diagIndicator, void** pOutputBuffer, unsigned*   pNofElementsInBuffer);
ModemActivityStatus  VmicModemHeaderPositionGet(unsigned * ReturnedHeaderPosition);
void VmicModemRxModeSet(void);
void vmicmodemDefaultVmicConfigSet();
void vlapmodemConfigWordSet( vmicmodemConfigWordSelectT ConfigWordSelect , uint32_t NewConfigWord);
void  vmicmodemC2fSwitchArraySet(uint8_t C2fBankSelect);
void  vmicmodemSelect(MeasurmentSelect_T  SourceSelect, uint8_t C2fSwitchArray);
uint32_t  vmicmodemCrcCounterGet();
uint32_t vmicapplayerSignalPeakToPeakValueGet();
uint32_t  vmicmodemDemodulationPtpSignalGet();

void  vmicmodemMeasurementSourceSelect(vmicmodemVmicTxSourceSelectCommandT SourceSelect);
void vmicmodemTim6Isr(TIM_HandleTypeDef *htim);
uint8_t vmicmodemSendingDataToVmicInProgress();


#define SUPPORTED_VMIC_MODEMS 1

// Total bits & space computation:
#define BITS_NO_PER_TRANSACTION         32
#define CYCLES_PER_BIT                  4
#define FULL_CYCLE_SAMPLES_NO           16
#define HALF_CYCLE_SAMPLES_NO       (FULL_CYCLE_SAMPLES_NO/2)
#define SAMPLES_PER_BIT           (CYCLES_PER_BIT * FULL_CYCLE_SAMPLES_NO)
#define DMA_BUFFER_SIZE           (BITS_NO_PER_TRANSACTION * SAMPLES_PER_BIT)
#define SYMBOLE_LENGTH          64
// Header bits & space computation:
#define HEADER_BITS_NO            6
#define HEADER_SIZE             (HEADER_BITS_NO * CYCLES_PER_BIT * FULL_CYCLE_SAMPLES_NO)
// Header pattern (0b111010):
#define HEADER_PATTERN            0x3A
#define HEADER_PATTERN_IN_FINAL_PLACE       (WORD_32_BITS)(HEADER_PATTERN << (NET_PAYLOAD + CRC_LEN))     // 0xE8

#// CRC related :
#define TOTAL_DATA_BITS           25
#define BASIC_POLYNOM           0xB7  // dividor polynom: X^7+X^5+x^4+x^2+x^1+x^0
#define CRC_MASK            0x7F  // crc mask: significant msbit removed
#define CRC_LEN             7 // crc bits
#define NET_PAYLOAD                                             (BITS_NO_PER_TRANSACTION -  HEADER_BITS_NO - CRC_LEN)
#define MAX_HEADER_CANDIDATES      20   // maximal #candidates for header position
#define VMICMODEM_RECEPTION_QUALITY_MAX         5


// Tx related:
#define BIT_FRAME_TRANSMISSION_PERIOD_IN_MICROS         1000//1000//600
#define CPU_PROCESSING_DELAY_IN_USECS                   0 //12+3
#define BIT_0_TRANSMISSION_PERIOD_IN_MICROS             (140 - CPU_PROCESSING_DELAY_IN_USECS) //200//160//140
#define BIT_1_TRANSMISSION_PERIOD_IN_MICROS             (28 - CPU_PROCESSING_DELAY_IN_USECS) //50//35//27//28
#define BIT_0_FRAME_TRANSMISSION_PERIOD_IN_MICROS       (BIT_FRAME_TRANSMISSION_PERIOD_IN_MICROS - BIT_0_TRANSMISSION_PERIOD_IN_MICROS - 2 * CPU_PROCESSING_DELAY_IN_USECS) //200
#define BIT_1_FRAME_TRANSMISSION_PERIOD_IN_MICROS       (BIT_FRAME_TRANSMISSION_PERIOD_IN_MICROS - BIT_1_TRANSMISSION_PERIOD_IN_MICROS - 2 * CPU_PROCESSING_DELAY_IN_USECS) //50

typedef enum {
  VMICMODULATOR_VMIC_TX_REQUEST_IDLE, 
  VMICMODULATOR_VMIC_TX_REQUEST_PENDING,
  VMICMODULATOR_VMIC_TX_REQUEST_DELAY_WAIT, 
  VMICMODULATOR_VMIC_SECOND_TX_REQUEST_WAIT 
} vmicmodemPendingTx_T;



typedef enum 
{
  VMICMODEM_MODEM_STATE_IDLE,
  VMICMODEM_MODEM_STATE_RMS_WAIT,
  VMICMODEM_MODEM_STATE_SECOND_FRAME_WAIT,
  VMICMODEM_MODEM_STATE_LONG_CORRELATION,
  VMICMODEM_MODEM_STATE_HEADER_SCAN,
  VMICMODEM_MODEM_STATE_PAYLOAD_DETECT,
  VMICMODEM_MODEM_STATE_TX
}VmicmodemModemStateT;

typedef enum { VMICMODEM_TX_STATE_IDLE, VMICMODEM_TX_STATE_RF_OFF_WAIT, VMICMODEM_TX_STATE_RF_ON_WAIT, VMICMODEM_TX_STATE_COMPLETED} vmicdemudulatorTxStateT; 

typedef enum { VMICMODEM_TX_STATUS_INPROGRESS, VMICMODEM_TX_STATUS_COMPLETED} TxFsmReturnedStatusT;


typedef struct
{
  uint32_t DataToBeTransmitted;
  uint8_t NumberOfBitsToTransmit;
  uint8_t NumberOfTxRepetitions;
  uint8_t TxRepetitionCounter;
  uint8_t BitCounter;
  void (*AppEndOfSourceChangeFunctionPtr)();
} TxRequestEntryT;


#pragma pack(1)
typedef
  union{
  struct {                                             
    uint32_t MeasureStatusBit:1;                        // Measure=0, Status=1
    uint32_t InputDataDestinationOrStatusSource1:1;
    uint32_t InputDataDestinationOrStatusSource0:1;
    uint32_t ConfigWord:16;
  } VmicConfigFields0;
  uint32_t VmicConfigWord0;
  } VmicConfig0_T;


#pragma pack(1)
  typedef
  union{
  struct {                                             
    uint32_t AnalogVoltageSelect:3;
    uint32_t RfSyncEnable:1;
    uint32_t ResonanceCapacitor:6;
    uint32_t ModuloationMode:1;
    uint32_t NotInUse:1;
  } VmicConfigFields1;
  uint32_t VmicConfigWord1;
  } VmicConfig1_T;




//



#define OFFSETS_TABLE_LENGTH    50
