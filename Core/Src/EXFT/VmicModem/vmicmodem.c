
#include <hwdrivers.h>
#include <stdlib.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "vlapConfig.h"
#include "vmicapplayer.h"
#include "vlapMain.h"
#include "vmicapplayer.h"
#include "vmicmodem.h"
#include "pccommApplayer.h"
#include "semphr.h"
#include "audit.h"

#include "common.h"
#include "arm_math.h"
#include "autoresonance.h"
#include "inet.h"
#include "autopower.h"


// L O C A L    D E F I N I T I O N S
#define VMICMODEM_MAX_CONSECUTIVE_GOOD_FRAMES          5 // 100
#define VMICMODEM_IIR_CYCLE_COUNT                       30
#define VMICMODEM_AGC_DELAY                             6
#define VMICMODEM_BELT_MASKING_DELAY                    150 //1.5s
// L O C A L    P R O T O T Y P E S
void modulatorTxTimerInit(void);
void timerSchedule( uint16_t Time);
void ScheduleTimer6Int(unsigned delayInMicros);
void MarkInterrupt(PendingInterruptType interruptType);
void StopPWM(void);
void ResumePWM(void);
void StopRX(void);
void ResumeRX(void);
WORD_32_BITS VmicCRCCalculation(WORD_32_BITS dynamicValue);
uint16_t   ShortListCreate( uint16_t* CandidatesArray);
void vlapDemodulatorTx( uint32_t TxData);
TxFsmReturnedStatusT  TxFsm();
void ResyncFrame();
vmicmodemPendingTx_T  VmicdemodulatorTxCallBack();
uint16_t vmicmodemIrr( uint16_t y, uint16_t x);


// M O D U L E   G L O B A L S
#define DC_BIAS_AMPREF  0 
#define CORR_NOM (6*64)


#define BAIS   0
#define FACTOR (5)


const int16_t HeaderPattern[] = 
{
#include "Samples.h"
};



#define DC_BIAS_AMPREF_BIT 0
#define CORR_NOM_BIT    64
const int16_t HeaderPatternOne[] = {
		( DC_BIAS_AMPREF_BIT+ 2427)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 3334)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 3727)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 3743)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 3743)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 3546)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 2859)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 2041)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 1155)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 270)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 0)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 0)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 14)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 125)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 772)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 1552)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 2442)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 3362)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 3753)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 3755)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 3748)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 3740)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 3049)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 2238)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 1319)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 373)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 4)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 5)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 0)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 8)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 619)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 1396)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 2303)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 3233)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 3752)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 3754)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 3735)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 3577)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 2941)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 2177)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 1325)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 441)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 13)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 11)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 0)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 181)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 852)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 1639)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 2532)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 3441)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 3741)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 3736)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 3737)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 3630)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 2940)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 2126)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 1250)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 402)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 6)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 0)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 0)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 116)/CORR_NOM_BIT, ( DC_BIAS_AMPREF_BIT+ 789)/CORR_NOM_BIT,
};

#define CORR_NOM_BIT_DOWN    70
const int16_t HeaderPatternOneScaledDown[] = {
  
  ( DC_BIAS_AMPREF_BIT+ 2147)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 3404)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 3727)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 3725)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 3725)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 3723)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 3535)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 2617)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 1463)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 290)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 5)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 4)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 4)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 0)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 235)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 1141)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 2300)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 3503)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 3721)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 3725)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 3723)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 3724)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 3531)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 2670)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 1541)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 311)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 4)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 3)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 11)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 1)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 165)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 1060)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 2223)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 3449)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 3726)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 3727)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 3732)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 3725)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 3517)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 2672)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 1550)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 369)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 6)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 0)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 5)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 3)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 211)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 1033)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 2121)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 3330)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 3739)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 3724)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 3727)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 3725)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 3471)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 2643)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 1556)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 390)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 0)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 4)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 12)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 16)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 171)/CORR_NOM_BIT_DOWN, ( DC_BIAS_AMPREF_BIT+ 999)/CORR_NOM_BIT_DOWN, 
};

#define MICRO_SEC_CTR 72 //     cycles/micro (timer6)


VmicmodemModemStateT ModemState = VMICMODEM_MODEM_STATE_IDLE;
SampleElement rawDataContent[2*DMA_BUFFER_SIZE];       
volatile CorrelationElement  correlationBuffer[2*DMA_BUFFER_SIZE+6*SYMBOLE_LENGTH];      
volatile CorrelationElement  IrrBuffer[2*DMA_BUFFER_SIZE];
uint8_t CurrentDmaBufferFlag;


volatile SemaphoreHandle_t AdcBufferReadySemaphore = NULL; 

// Correlation buffer absolute Max and Min values 
int16_t   absMax;
uint32_t  absMaxOffset;
int16_t   absMin;
uint32_t  absMinOffset;

q15_t RecivedSignalRms; 
q31_t RecivedSignalVariance;

uint16_t CyclesCounter;
uint16_t LastGlobalCorrelationThreshold;
uint16_t OffsetsInRawDataBuffTable[OFFSETS_TABLE_LENGTH];
uint8_t  OffsetsInRawDataCounter;
volatile WORD_32_BITS  GoodFrames[OFFSETS_TABLE_LENGTH];       
volatile uint8_t GoodFramesCounter;

volatile WORD_32_BITS  GoodFramesDotProd[OFFSETS_TABLE_LENGTH];       
volatile uint8_t GoodFramesCounterDotProd;

WORD_32_BITS PayLoadAfterDotProduct[OFFSETS_TABLE_LENGTH];
WORD_32_BITS PayLoadAfterDotProductTest[2*DMA_BUFFER_SIZE];

uint16_t BitLocationInCorrBuffTable[OFFSETS_TABLE_LENGTH];
uint16_t BitLocationInCorrBuffTableDotProduct[OFFSETS_TABLE_LENGTH];

uint16_t ModemCrcErrorCounter;

uint32_t Payload;

volatile uint16_t  BadFrames;

q63_t     DotProduct;

volatile uint8_t OptionalBitOffsetsCounter;
volatile WORD_32_BITS CrcValue;
volatile uint16_t SelectedOffset = 0;

TxRequestEntryT TxRequestEntry;
vmicdemudulatorTxStateT TxState = VMICMODEM_TX_STATE_IDLE;

uint8_t ResyncDelayCounter;
uint32_t recentWordEncountered;
vmicmodemPendingTx_T vmicmodemPendingTx;

uint8_t ReceptionQualityCounter;

uint32_t ModulatedSignalPeakToPeak;
uint32_t ModulatedSignalPeakToPeakFilter;

#define OFFSETS_TABLE_LENGTH    50

TimerHandle_t vmicmodemTxTimerHandler;

VmicConfig0_T LastConfig0SentToVmic;
VmicConfig1_T LastConfig1SentToVmic;

uint16_t IrrCycleCounter;

MeasurmentSelect_T  SourceSelected;

enum {VMICMODEM_AGC_STATE_HIGH_GAIN, VMICMODEM_AGC_STATE_HIGH_TO_LOW_CHECK, VMICMODEM_AGC_STATE_LOW_GAIN, VMICMODEM_AGC_STATE_LOW_TO_HIGH_CHECK}  AgcState;
uint8_t AgcStateCounter;

uint32_t SemaphoreTimeOut;

/******************************************************************************
*
*
*
******************************************************************************/
void vmicmodemInit()
{
  
  ResyncDelayCounter = 0;
  recentWordEncountered = 0;
  ReceptionQualityCounter = 0;

  // Sets the VMIC configuration words RAM image to VMIC default values
  vmicmodemDefaultVmicConfigSet();
  
  vmicmodemPendingTx = VMICMODULATOR_VMIC_TX_REQUEST_IDLE;
  // Init TIM6, the modulator Tx bit duration timer
  modulatorTxTimerInit(); 
  // Create modem FSM semaphore, Will be used to activate the modem FSM task as follows: From ADC DMA competion ISR during DeModulator (Rx) mode, From TIM6 during Modulator (Tx) mode
  if( !AdcBufferReadySemaphore)
    AdcBufferReadySemaphore = xSemaphoreCreateBinary();
  // Init FSM state to IDLE
  ModemState = VMICMODEM_MODEM_STATE_IDLE;
  // No pending Transmission requests
  memset(&TxRequestEntry, 0, sizeof(TxRequestEntry));
  vlapmainDemodulatorTaskControl( VLAPMAIN_TASK_STOP);
  ModulatedSignalPeakToPeak = 0;
  ModulatedSignalPeakToPeakFilter = 0;
  // initial AGC state
  AgcState = VMICMODEM_AGC_STATE_HIGH_GAIN;
  
   // Create timer.
  vmicmodemTxTimerHandler =  xTimerCreate("modemTxTmr",  portTICK_PERIOD_MS, pdFALSE, (void *)0, (void (*)())VmicdemodulatorTxCallBack);

  SemaphoreTimeOut = 0;
  
  IrrCycleCounter = VMICMODEM_IIR_CYCLE_COUNT;
}

/******************************************************************************
*
*
*
******************************************************************************/
portTASK_FUNCTION(vlapDemodulator, pvParameters ) 
{
  /* The parameters are not used. */
  volatile uint16_t i,j;
  uint8_t BitVlaue;
  volatile uint32_t PayLoadTemp;
  q15_t PeakToPeakMax;
  q15_t PeakToPeakMin;
  uint32_t MinMaxLocation;
  

  
  while (1)
  {
    
    BaseType_t SemaphoreTakeResult = xSemaphoreTake(AdcBufferReadySemaphore, portMAX_DELAY);
    
    if(SemaphoreTakeResult == pdTRUE) 
    {
      volatile uint8_t HeaderSearchState;
      
      // Calculate modulation signal Peak to Peak value
      arm_max_q15((q15_t*)rawDataContent, 100, &PeakToPeakMax, &MinMaxLocation);
      arm_min_q15((q15_t*)rawDataContent, 100, &PeakToPeakMin, &MinMaxLocation);
      ModulatedSignalPeakToPeak = PeakToPeakMax - PeakToPeakMin;
      ModulatedSignalPeakToPeakFilter = (ModulatedSignalPeakToPeak * 8 +  ModulatedSignalPeakToPeakFilter * 2)/10; 
      uint16_t  VptpTemp = ModulatedSignalPeakToPeakFilter * 3000 / 16384;
      pccpmmAppLayerStruct.Board1SystemRegisters.ModemRxPtpL = VptpTemp;
      pccpmmAppLayerStruct.Board1SystemRegisters.ModemRxPtpM = VptpTemp>>8;
            
      
      // Handle the AGC
      // vmicmodemAgc(ModulatedSignalPeakToPeak);
      
      pccpmmAppLayerStruct.DspRegisters.CrcErrorCounter = HTONS(ModemCrcErrorCounter);

      
      switch(ModemState)
      {
      case VMICMODEM_MODEM_STATE_IDLE:
        CyclesCounter = 20;
        // Sets the default VMIC RAM image to VMIC defaults
        vmicmodemDefaultVmicConfigSet();
        ModemCrcErrorCounter = 10;
        // Indication to PC, No Feedback
        // TODO: Change to enum: See VMIC registers $202e Status
        pccpmmAppLayerStruct.VmicRegisters3.Status = 1;

        //pccpmmAppLayerStruct.DspRegisters.CrcErrorCounter = 10;
        ModemState = VMICMODEM_MODEM_STATE_RMS_WAIT;
        break;
        
      case VMICMODEM_MODEM_STATE_RMS_WAIT:
        // Fill the lowest memory region as this is the first buffer
        memcpy(rawDataContent, adc1DmaBuffer[CurrentDmaBufferFlag], sizeof(rawDataContent)/2);
        if(ModulatedSignalPeakToPeakFilter > 600)
        	ModemState = VMICMODEM_MODEM_STATE_SECOND_FRAME_WAIT;
        break;
        
      case VMICMODEM_MODEM_STATE_SECOND_FRAME_WAIT:
        // Fill the highest part of the double frame buffer, we have now full buffer with at least one full message packet
        memcpy(rawDataContent+DMA_BUFFER_SIZE, adc1DmaBuffer[CurrentDmaBufferFlag], sizeof(rawDataContent)/2 );



#if 0
        // Perform IIR filter to 
        if(IrrCycleCounter)
        {
          IrrCycleCounter--;
          for(i=0; i<NOF_ELEMENTS(IrrBuffer); i++)
            IrrBuffer[i] = vmicmodemIrr( IrrBuffer[i], rawDataContent[i]);
          ModemState = VMICMODEM_MODEM_STATE_RMS_WAIT;
          break;
        }
        else
        {
          IrrCycleCounter = VMICMODEM_IIR_CYCLE_COUNT;
          ModemState = VMICMODEM_MODEM_STATE_LONG_CORRELATION;
        }
#else
        ModemState = VMICMODEM_MODEM_STATE_LONG_CORRELATION;
#endif
        
      case VMICMODEM_MODEM_STATE_LONG_CORRELATION:

        // Zero the correlation buffer as explained in the CMSIS documentation
        arm_fill_q15 (0, (q15_t*)correlationBuffer, NOF_ELEMENTS(correlationBuffer));
        // Zero OffsetsInRawDataBuffTable
        arm_fill_q15 (0, (q15_t*)OffsetsInRawDataBuffTable, NOF_ELEMENTS(OffsetsInRawDataBuffTable));
        // Correlate the double buffer with "1"s pattern 
        arm_correlate_fast_q15((q15_t*)HeaderPattern, NOF_ELEMENTS(HeaderPattern),(q15_t*)rawDataContent, NOF_ELEMENTS(rawDataContent),(q15_t*)correlationBuffer );
//        arm_correlate_fast_q15((q15_t*)HeaderPattern, NOF_ELEMENTS(HeaderPattern),(q15_t*)IrrBuffer, NOF_ELEMENTS(IrrBuffer),(q15_t*)correlationBuffer );
        //       arm_correlate_fast_q15( (q15_t*)HeaderPatternOne, NOF_ELEMENTS(HeaderPatternOne), (q15_t*)HeaderPatternOne, NOF_ELEMENTS(HeaderPatternOne), (q15_t*)correlationBuffer );
        arm_max_q15((q15_t*)correlationBuffer, NOF_ELEMENTS(correlationBuffer), &absMax, &absMaxOffset);
        // When searching for minimum value we have to cancel out the edges as they include zeros
        arm_min_q15((q15_t*)correlationBuffer+100, NOF_ELEMENTS(correlationBuffer)-200, &absMin, &absMinOffset);
        // Calculate the signal threshold as 
        LastGlobalCorrelationThreshold = (uint16_t) (((uint32_t)absMax * (uint32_t)98)/(uint32_t)100);
        OffsetsInRawDataCounter=0;
        for(i=NOF_ELEMENTS(correlationBuffer); i>0; i--)
        {
          if(correlationBuffer[i] > LastGlobalCorrelationThreshold)
          {
            if( (NOF_ELEMENTS(correlationBuffer) - i) >  NOF_ELEMENTS(HeaderPattern))
            {
              OffsetsInRawDataBuffTable[OffsetsInRawDataCounter] = (NOF_ELEMENTS(correlationBuffer) - i - NOF_ELEMENTS(HeaderPattern));
              if(OffsetsInRawDataCounter < (OFFSETS_TABLE_LENGTH-1) )
            	  OffsetsInRawDataCounter++;
            }
          }
        }
        ModemState = VMICMODEM_MODEM_STATE_HEADER_SCAN;
        break;
      case VMICMODEM_MODEM_STATE_HEADER_SCAN:
        // Clear the optional frame begin counter counter
        OptionalBitOffsetsCounter = 0;
        // Clear the GoodFrames counter;
        GoodFramesCounter = 0;
        OptionalBitOffsetsCounter = 0;
        //Check all possible offsets 
        for(i=0; i<OffsetsInRawDataCounter; i++)
        {
          // Set the header search state to idle
          HeaderSearchState = 0;
          // Iterate over the correlation buffer every SYMBOLE_LENGTH, we start from SYMBOLE_LENGTH as the first meaningfull bit in correlation buffer starts at SYMBOLE_LENGTH 
          for(j=0; j< 6; j++)
          {
            arm_correlate_fast_q15((q15_t*)HeaderPatternOne, NOF_ELEMENTS(HeaderPatternOne),(q15_t*)&rawDataContent[OffsetsInRawDataBuffTable[i] +j*SYMBOLE_LENGTH], NOF_ELEMENTS(HeaderPatternOne),(q15_t*)correlationBuffer );
            if(correlationBuffer[64] > 100)
              BitVlaue = 1;
            else
              BitVlaue = 0;
            
            // search for header pattern in the correlation buffer with nth offset
            switch(HeaderSearchState)
            {
            case 0:
              // "1"
              if(BitVlaue)
                HeaderSearchState=1;
              break;
            case 1:
              // "1"
              if(BitVlaue)
                HeaderSearchState=2;
              else
                HeaderSearchState=0;
              break;
              // "1"
            case 2:
              if(BitVlaue)
                HeaderSearchState=3;
              else
                HeaderSearchState=0;
              break;
              // "0"
            case 3:
              if(!BitVlaue)
                HeaderSearchState=4;
              break;
              // "1"
            case 4:
              if(BitVlaue)
                HeaderSearchState=5;
              else
                HeaderSearchState=0;
              break;
              // "0" (reversed this time as we want to process the header is successful
            case 5:
              if(BitVlaue)
                HeaderSearchState=2;
              else
              {
                if(OffsetsInRawDataBuffTable[i] < (DMA_BUFFER_SIZE + 6*64))
                {
                  // Points to the original buffer at the header start.
                  BitLocationInCorrBuffTable[OptionalBitOffsetsCounter] = OffsetsInRawDataBuffTable[i];
                  if(OptionalBitOffsetsCounter < sizeof(BitLocationInCorrBuffTable))
                    OptionalBitOffsetsCounter++;
                }
                HeaderSearchState = 0;
              }
              break;   
            }
          }
        }
        
        if(!OptionalBitOffsetsCounter)
        {
          // Resync on new frame 
          ResyncFrame();
          // Read fresh buffer  and rerun the correlation
          // Indication to PC, No header detection
          // TODO: Change to enum: See VMIC registes $202e Status
          pccpmmAppLayerStruct.VmicRegisters3.Status = 2;
          ModemState = VMICMODEM_MODEM_STATE_IDLE;
        }
        
        // Check for header candidates in correlation buffer  
        for(i=0; i< OptionalBitOffsetsCounter; i++)
        {
          // Build the payload word
          //PayLoadTemp = 0x0000003a;
          PayLoadTemp = 0;
          for(CyclesCounter=0; CyclesCounter<32; CyclesCounter++)
          {
            PayLoadTemp <<= 1;
            arm_correlate_fast_q15((q15_t*)HeaderPatternOne, NOF_ELEMENTS(HeaderPatternOne),(q15_t*)&rawDataContent[BitLocationInCorrBuffTable[i] + (CyclesCounter*SYMBOLE_LENGTH)], NOF_ELEMENTS(HeaderPatternOne),(q15_t*)correlationBuffer );
            if(correlationBuffer[64] > 100)
              PayLoadTemp |= 1;
            else
              PayLoadTemp &= ~1;
          }
          
          CrcValue = VmicCRCCalculation(PayLoadTemp); 
          if( !CrcValue )
          {
            GoodFrames[i]=PayLoadTemp;
            if (GoodFramesCounter < VMICMODEM_MAX_CONSECUTIVE_GOOD_FRAMES)
                GoodFramesCounter++;
          }
          else
          {
            // Indication to PC, CRC errors
            // TODO: Change to enum: See VMIC registes $202e Status
            pccpmmAppLayerStruct.VmicRegisters3.Status = 3;
            ModemState = VMICMODEM_MODEM_STATE_IDLE;
            GoodFrames[i]=0xff;
          }
        }
        
        if(GoodFramesCounter)
        {
          ModemCrcErrorCounter = 0;
          pcccommAppLayerSamplesIndexReset();
          ModemState = VMICMODEM_MODEM_STATE_PAYLOAD_DETECT;
        }
        else
        {
          ModulatedSignalPeakToPeakFilter = 0;
          ModemState = VMICMODEM_MODEM_STATE_IDLE;
        }
        break;
        
      case        VMICMODEM_MODEM_STATE_PAYLOAD_DETECT:
        if(TxRequestEntry.DataToBeTransmitted)
        {
          // Clear the measurmeent buffers 
          memset(pccpmmAppLayerStruct.VmicMeasurementBuffer1, 0, sizeof(pccpmmAppLayerStruct.VmicMeasurementBuffer1));
          memset(pccpmmAppLayerStruct.VmicMeasurementBuffer2, 0, sizeof(pccpmmAppLayerStruct.VmicMeasurementBuffer2));
          // Measurement buffer not ready
          pccpmmAppLayerStruct.VmicRegisters3.VmicMeasurementBufferReady = 0;
          // Prepare for Tx state
          ModemState = VMICMODEM_MODEM_STATE_TX;
          TxState = VMICMODEM_TX_STATE_IDLE; 
          // Stop the ADC sampling mechanism, The semaphore given by the ADC/DMA interrupt will stop
          HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_1);

          // Scehdule TIM6 to generate interrupt after 100uSec, The ISR will call the TxFsm
          ScheduleTimer6Int(100);
          break;
        }
        
        // Move the upper buffer half to the lower buffer half ( Most recent buffer located at the upper buffer half will be moved to the lower part of the buffer)
        memcpy(rawDataContent, rawDataContent+DMA_BUFFER_SIZE, 2*DMA_BUFFER_SIZE );
        // Fill the upper buffer half with fresh data from the ADC DMA
        memcpy(rawDataContent+DMA_BUFFER_SIZE, adc1DmaBuffer[CurrentDmaBufferFlag], 2*DMA_BUFFER_SIZE);
        
        // Check all optional header locations that passed CRC check
        for(i=0; i<OptionalBitOffsetsCounter; i++)
        {
          if(GoodFrames[i] != 0xff)
          {
            // Iterate over all the message bits starting from the header and perform dot product to detect logic One or Zero
            for(j=0; j<32; j++)
            {
              // Perform dot product between stored "One" image and the current bit image in the rawDatabuffer. Iterate over all 32 bits and build the payload word
              arm_dot_prod_q15((q15_t*)HeaderPatternOne, (q15_t*)&rawDataContent[BitLocationInCorrBuffTable[i] + j*SYMBOLE_LENGTH], NOF_ELEMENTS(HeaderPatternOneScaledDown), &DotProduct); 
              PayLoadAfterDotProduct[i] <<= 1;
              if(DotProduct > 3000000)
                PayLoadAfterDotProduct[i] |= 1;
              else
                PayLoadAfterDotProduct[i] &= ~1;
            }
            // Check the payload header is correct 
            if( (PayLoadAfterDotProduct[i] & HEADER_PATTERN_IN_FINAL_PLACE) == HEADER_PATTERN_IN_FINAL_PLACE)
            {
              // Confirm the  
              CrcValue = VmicCRCCalculation(PayLoadAfterDotProduct[i]); 
              if( !CrcValue )
              {
                 if(ReceptionQualityCounter < VMICMODEM_RECEPTION_QUALITY_MAX)
                    ReceptionQualityCounter++;
                 else
                 {
                   pccpmmAppLayerStruct.VmicRegisters3.Status = 5;
                   // Push the payload data to the application layer
                  recentWordEncountered = PayLoadAfterDotProduct[i];
                  // Handle the new data according to the selected source
                  vmicapplayerRx(0, recentWordEncountered);
                  if (GoodFramesCounter < VMICMODEM_MAX_CONSECUTIVE_GOOD_FRAMES)
                      GoodFramesCounter++;
                  if(ModemCrcErrorCounter)
                    ModemCrcErrorCounter--;
                 }
                // No need to check more optional header locations if the CRC is OK, Break the loop and go wait for the comming frame
                break; //goto SWITCH_END_LABLE;
              }
              else
              {
				vmicapplayerRx(1, recentWordEncountered);
 				if(ReceptionQualityCounter)
                    ReceptionQualityCounter--;
                // Increment the CRC errors counter
                ModemCrcErrorCounter+=2;
                if(ModemCrcErrorCounter > 10)
                {
                  // Too many errored frames, Go hunt for new header location 
                  // Move the frame sampling point to new location (Hopping for better frame lock)
                  ResyncFrame();
                 ModemState = VMICMODEM_MODEM_STATE_IDLE;
                }
              }

            }
            else
            {
              //vlapMainSetupHardware();
              ModemState = VMICMODEM_MODEM_STATE_IDLE;
            }
          } // If good header End
        } // For loop End, Optional locations 

        // TODO: What happens if all optional locations failed, consider reSyn frame sampling
//        if(i == OptionalBitOffsetsCounter)
//        	 ModemState = VMICMODEM_MODEM_STATE_RMS_WAIT;
      SWITCH_END_LABLE:
        break;
      
      case VMICMODEM_MODEM_STATE_TX:
        // Control will be returned to this state after TxFsm completes and re enables ADC2 
       if(TxFsm() == VMICMODEM_TX_STATUS_COMPLETED)
        {
          // Check if commands transmission sequence ended   
          if( VmicdemodulatorTxCallBack() == VMICMODULATOR_VMIC_TX_REQUEST_IDLE)
          {
            // We assume the system is locked 
              ResyncFrame();
//             ModemState = VMICMODEM_MODEM_STATE_IDLE;
            ModemState = VMICMODEM_MODEM_STATE_PAYLOAD_DETECT;
          }
        }
        break;
        
        
      } // Switch End
    } // SemaphoreTakeResult End
    else
    	SemaphoreTimeOut++;
  } // While1(1) End
}

void ResumePWM(void)
{
  hwdriversGpioBitWrite(HWDRIVERS_TX_SUP_ENABLE,1);

  __HAL_TIM_MOE_ENABLE(&htim1);
}

void StopRX(void)
{
}

void ResyncFrame()
{
    HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_1);
	// Delay half frame
	vTaskDelay(ResyncDelayCounter);
    HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1);

	ResyncDelayCounter++;
	if(ResyncDelayCounter >= 9)
		ResyncDelayCounter=0;
}


/******************************************************************************
*
*
*
******************************************************************************/
void  mainAdcDmaTransferCompleteIsr(uint8_t BufferIndicationFlag)
{
  CurrentDmaBufferFlag = BufferIndicationFlag;
  xSemaphoreGiveFromISR(AdcBufferReadySemaphore, NULL);
}


/******************************************************************************
* @brief  void ScheduleTimer6Int(unsigned delayInMicros)
* @param  clientType : currently - MODEM only
* @param  id : of the sub-client customer
* @param  delayInMicros : to accept invocation on
* @retval 
******************************************************************************/

void ScheduleTimer6Int(unsigned delayInMicros)
{ 
  timerSchedule(delayInMicros * MICRO_SEC_CTR);
}

/******************************************************************************
* @brief  timerSchedule( uint16_t Time) .
* @param  Each unit represent 1/27 uSecs: 27=>1uSec, 1350=>50uSec
* @retval 
******************************************************************************/
void timerSchedule( uint16_t Time)
{
  __HAL_TIM_CLEAR_FLAG(&htim6, TIM_IT_UPDATE);
  __HAL_TIM_SET_AUTORELOAD(&htim6, Time);
  HAL_TIM_Base_Start_IT(&htim6); 

/*
	TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
  	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
  	TIM6->ARR = (uint32_t)Time;
	// TIM6 enable counter
	TIM6->CR1 |= TIM_CR1_CEN;
*/
}

/******************************************************************************
* @brief  void VmicRxEventsCallback
* @param  VmicModemId     modemId, 
* @param  VmicEvent eventType,
* @param  ModemTotalTransaction vmicTransaction,
* @param   ModemPayload vmicNetTransaction
* @retval 
******************************************************************************/
void VmicRxEventsCallback(
                          VmicModemId     modemId,                        // optional, meaningless. would be constantly 0
                          VmicEvent eventType,
                          ModemTotalTransaction vmicTransaction,          // entire 32 bits transaction. Valid only for eventType == VMIC_TRANSACTION_ACCEPTED
                          ModemPayload vmicNetTransaction                 // net/'peeled' transaction. Valid only for eventType == VMIC_TRANSACTION_ACCEPTED
                            )
{
  vmicapplayerRx(eventType, vmicTransaction);
}

void modulatorTxTimerInit()
{
  
	// Will be supported by the HAL environment
	// __HAL_TIM_ENABLE(&htim6);
/*
  TIM_InternalClockConfig(TIM6);
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructureTim6);
  TIM_TimeBaseStructureTim6.TIM_Period = 0; 
  TIM_TimeBaseStructureTim6.TIM_Prescaler = 0;
  TIM_TimeBaseStructureTim6.TIM_ClockDivision = 0;
  TIM_TimeBaseStructureTim6.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructureTim6);
  
  TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
  
  TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
  
  TIM_Cmd(TIM6, DISABLE);
*/
}




void vmicmodemTim6Isr(TIM_HandleTypeDef *htim)
{
	if(htim == &htim6)
	{
    HAL_TIM_Base_Stop_IT(&htim6);
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_IT_UPDATE);
    TxFsm();
	}
}


void StopPWM(void)
{
  __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
  hwdriversGpioBitWrite(HWDRIVERS_TX_SUP_ENABLE,0);

}

void ResumeRX(void)
{
}


WORD_32_BITS VmicCRCCalculation(WORD_32_BITS dynamicValue)
{
  WORD_32_BITS polynom;
  
  polynom = (WORD_32_BITS)BASIC_POLYNOM << (TOTAL_DATA_BITS - 1);
  
  //    print ('polynom = 0x%x, no = 0x%x' % (polynom,dynamicValue))
  while (polynom >= BASIC_POLYNOM)
  {
    dynamicValue = dynamicValue ^ polynom;
    //        print ('1. polynom = 0x%x, dynamicValue = 0x%x' % (polynom,dynamicValue))
    while ((dynamicValue ^ polynom) > dynamicValue)
      polynom = polynom >> 1;
  }
  return dynamicValue & CRC_MASK;
}
bool vmicmodemBeltMasking = false;
void vmicmodemTransmitterRetoggle()
{
  vmicmodemBeltMasking = true;
  autopowerPwmTemporaryControl(DISABLE);
  vTaskDelay(1);
  autopowerPwmTemporaryControl(ENABLE);
  vlapmainDebugLog("Retoggle transmitter to test open-belt...");
}

 uint16_t openBeltMasking = 0;
 uint8_t openBeltFSMState = 0;
/// @brief Masking belt open events after vmic channel switch (EX5.6 JKFF bug)
/// @param beltStatus belt status from audit
/// @return Allow belt status change or mask it
bool vmicmodemOpenBeltMaskingFSM(AuditBeltHumanCheck_T beltStatus)
{
  if (openBeltMasking == 0) // Not from TxFsm
    {
      openBeltFSMState = 0;
      return TRUE;
    }
  switch (openBeltFSMState)
  {
  case 0: // Idle
    if (openBeltMasking) openBeltMasking--;
    if (beltStatus == AUDIT_BELT_OPEN)
    {
      openBeltFSMState++;
      vmicmodemTransmitterRetoggle();
      openBeltMasking = VMICMODEM_BELT_MASKING_DELAY;
    }
    return FALSE;
  case 1: // Wait after toggle attempt
    if (beltStatus != AUDIT_BELT_OPEN || openBeltMasking == 0) 
    {
      openBeltMasking = 0;
      openBeltFSMState = 0; // Return to Idle
      return TRUE;
    }
    openBeltMasking--;
    return FALSE;
  default:
    break;
  }
}

#if 0
uint16_t   ShortListCreate( uint16_t* CandidatesArray)
{
  uint16_t i;
  uint16_t j;
  uint16_t Temp;
  uint16_t SelectedLocation;
  
  Temp = CandidatesArray[0];
  for(i=0; i< NO_ELEMENTS(CandidatesArray); i++)
  {
    for(j=i; j< NO_ELEMENTS(CandidatesArray); j++)
      if( (CandidatesArray[j] - Temp) == 2048)
      {
        SelectedLocation = CandidatesArray[i];
        break;
      }
    if(j==NO_ELEMENTS(CandidatesArray))
      break;
  }
  
  return(SelectedLocation);
}
#endif

/*****************************************************************************
* void vlapDemodulatorTxReq( vmicmodemConfigWordSelectT ConfigWordSelect , void (*AppEndOfSourceChangeFunctionPtr)())
*
*
*
*****************************************************************************/
void vlapDemodulatorTxReq( vmicmodemConfigWordSelectT ConfigWordSelect , void (*AppEndOfSourceChangeFunctionPtr)())
{
  ModemTotalTransaction   requestedTransmission;
  int             i;
  uint32_t      TxData = 0;
  ModemPayload    bitMask;
  WORD_32_BITS    crcVal;
  requestedTransmission = HEADER_PATTERN_IN_FINAL_PLACE;
  // Disable autoresonance 
  //autoresonanceControl(AUTORESONANCE_CONTROL_OFF);
  
 switch(ConfigWordSelect)
  {
    case VMICMODEM_VMIC_CONFIG_WORD_0:
      break;
    case VMICMODEM_VMIC_CONFIG_WORD_1:
      break;
    case VMICMODEM_VMIC_CONFIG_WORD_2:
      TxData = ExtDeviceToVmicDataUnion2.Data2;
      break;
    case VMICMODEM_VMIC_CONFIG_WORD_3:
      TxData = ExtDeviceToVmicDataUnion3.Data3;
      break;
  }
  
  for (i = NET_PAYLOAD - 1 , bitMask = 1 << (NET_PAYLOAD + CRC_LEN - 1) , TxData <<= CRC_LEN ;
       i >= 0 ; 
       i--, bitMask >>= 1)
    requestedTransmission |= TxData & bitMask;
  crcVal = VmicCRCCalculation(requestedTransmission);
  requestedTransmission |= crcVal;
  
  
  // Mark the session as pending VMIC Tx session
  vmicmodemPendingTx = VMICMODULATOR_VMIC_TX_REQUEST_PENDING;
  
  
  TxRequestEntry.NumberOfBitsToTransmit = 32-1;
  TxRequestEntry.NumberOfTxRepetitions = 10;
  TxRequestEntry.BitCounter = 0;
  TxRequestEntry.TxRepetitionCounter = 0;
  TxRequestEntry.AppEndOfSourceChangeFunctionPtr = AppEndOfSourceChangeFunctionPtr;
  // The DataToBeTransmitted value is polled in the VMICMODEM_MODEM_STATE_PAYLOAD_DETECT state every frame  
  TxRequestEntry.DataToBeTransmitted = requestedTransmission;
}



/*************************************************************************************************
// TxFsmReturnedStatusT  TxFsm() 
//  
//  
*************************************************************************************************/
TxFsmReturnedStatusT  TxFsm()
{
  uint8_t curBitVal = 0;
  TxFsmReturnedStatusT ReturnedStatus = VMICMODEM_TX_STATUS_INPROGRESS;

  switch(TxState)
    {
    case VMICMODEM_TX_STATE_IDLE:
//      StopPWM();
      autopowerPwmTemporaryControl(DISABLE);
      TxState = VMICMODEM_TX_STATE_RF_OFF_WAIT;
      curBitVal = (TxRequestEntry.DataToBeTransmitted & 0x80000000)?1:0;
      ScheduleTimer6Int(curBitVal ? BIT_1_TRANSMISSION_PERIOD_IN_MICROS : BIT_0_TRANSMISSION_PERIOD_IN_MICROS);
      break;
    case  VMICMODEM_TX_STATE_RF_ON_WAIT:
      if(TxRequestEntry.BitCounter < TxRequestEntry.NumberOfBitsToTransmit)
      {
//        StopPWM();

        autopowerPwmTemporaryControl(DISABLE);
        TxRequestEntry.BitCounter++;
        curBitVal = ((TxRequestEntry.DataToBeTransmitted << TxRequestEntry.BitCounter)  & 0x80000000) ? 1:0;
        TxState = VMICMODEM_TX_STATE_RF_OFF_WAIT;
        ScheduleTimer6Int(curBitVal ? BIT_1_TRANSMISSION_PERIOD_IN_MICROS : BIT_0_TRANSMISSION_PERIOD_IN_MICROS);
      }
      else
      {
        // Anyway resume the PWM
//        ResumePWM();
        autopowerPwmTemporaryControl(ENABLE);
        // Tx Request was sent, clear the entry
//        memset(&TxRequestEntry, 0, sizeof(TxRequestEntry));
         TxRequestEntry.DataToBeTransmitted = 0;
        TxState = VMICMODEM_TX_STATE_COMPLETED;
        // Turn on ADC/DMA timer to retrigger modem FSM with semaphore initiated by the DMA completion itsr 
        HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1);
      }
      break;
    case VMICMODEM_TX_STATE_RF_OFF_WAIT:
 //     ResumePWM();
      autopowerPwmTemporaryControl(ENABLE);
      TxState = VMICMODEM_TX_STATE_RF_ON_WAIT;
      ScheduleTimer6Int(curBitVal ? BIT_1_FRAME_TRANSMISSION_PERIOD_IN_MICROS : BIT_0_FRAME_TRANSMISSION_PERIOD_IN_MICROS);
      break;
    
    case VMICMODEM_TX_STATE_COMPLETED:
      // This state will be called by the VMICMODEM_MODEM FSM from state VMICMODEM_MODEM_STATE_TX as the VMICMODEM_MODEM FSM is now 
      // triggered by the ADC after it had been reanabled from state VLAPMODULATOR_TX_STATE_RF_ON_WAIT
      openBeltMasking = VMICMODEM_BELT_MASKING_DELAY;
      openBeltFSMState = 0; // return openbelt fsm to idle
    	ReturnedStatus = VMICMODEM_TX_STATUS_COMPLETED;
      break;

    }
  return(ReturnedStatus);
}

/*************************************************************************************************
// VmicGetDiagInfo - 
// returns: STATUS_SUCCESSFUL (full response) OR:
//      STATUS_FAILURE_PARAM  (illegal vmicModemId or pOutputBuffer == NULL or pNofElementsInBuffer == NULL) OR:
//      STATUS_PREMATURE      (no valid measurement available to conclude noise from)        
*************************************************************************************************/
ModemActivityStatus VmicGetDiagInfo(// Input params:
                                    VmicModemId vmicModemId,            // optional, meaningless. would be constantly 0
                                    VmicDiagIndicator   diagIndicator,  // diag info required
                                    
                                    // Output params (valid only if STATUS_SUCCESSFUL is returned):
                                    void** pOutputBuffer,               // ptr to the output buffer returned (it doesn't have a definite 
                                    // type - since for VMIC_DIAG_RAW_SAMPLES,VMIC_AVERAGE_BUFFER
                                    // it returns buffer of SampleElement. For VMIC_DIAG_NOISE_RESULTS
                                    // it returns buffer of NoiseElement. For VMIC_DIAG_CORRELATION_RESULTS,
                                    // it returns buffer of CorrelationElement.
                                    unsigned*   pNofElementsInBuffer    // ptr to # elements in returned buffer
                                      )
{ 
  ModemActivityStatus modemActivityStatus;
  
  modemActivityStatus = STATUS_SUCCESSFUL;      // Assume success
  
  {    
    // make sure entry is caught 
    
    switch (diagIndicator)
    {
    case VMIC_DIAG_RAW_SAMPLES:                           // raw samples indicator
      *pOutputBuffer = (void*)rawDataContent;
      *pNofElementsInBuffer = NOF_ELEMENTS(rawDataContent);
      break;
    case VMIC_AVERAGE_BUFFER:                             // average buffer indicator
      break;
    case VMIC_DIAG_CORRELATION_RESULTS:  
      *pOutputBuffer = (void*)correlationBuffer;
      *pNofElementsInBuffer = NOF_ELEMENTS(correlationBuffer);
      break;
    case VMIC_DIAG_NOISE_RESULTS:  
      {
        // Compute characteristic diag params (to-be-used during noise analysis):

        *pOutputBuffer = (void*)correlationBuffer;
        // TODO: 
        *pNofElementsInBuffer = 10;
      }
      break;
    default:  
      // Illegal request
      modemActivityStatus = STATUS_FAILURE_PARAM;
      break;
    }
  }
  return modemActivityStatus;
}



/******************************************************************************
* @brief  odemActivityStatus VmicModemHeaderPositionGet(unsigned * ReturnedHeaderPosition)
* @param  
* @retval 
******************************************************************************/
ModemActivityStatus VmicModemHeaderPositionGet(unsigned * ReturnedHeaderPosition)
{
  *ReturnedHeaderPosition = BitLocationInCorrBuffTable[0];
  return(STATUS_SUCCESSFUL);
}

/******************************************************************************
* @brief  ReturnCode_T vmicmodemReceptionQualityGet(uint8_t *ReturnedReceptionQualityPtr)
* @param  
* @retval 
******************************************************************************/
ReturnCode_T vmicmodemReceptionQualityGet(uint8_t *ReturnedReceptionQualityPtr)
{
  *ReturnedReceptionQualityPtr = ReceptionQualityCounter;
  return(RETURNCODE_OK);
}






/******************************************************************************
* @brief  vmicmodemPendingTx_T  VmicdemodulatorTxCallBack()
* @param  
* @retval 
******************************************************************************/
vmicmodemPendingTx_T  VmicdemodulatorTxCallBack()
{
  
  switch(vmicmodemPendingTx)
  {
  case VMICMODULATOR_VMIC_TX_REQUEST_IDLE:
    break;
  case VMICMODULATOR_VMIC_TX_REQUEST_PENDING:   
    // Delay between VMIC new measurement source selection command and the command designed to send the actual measurments
    vmicmodemPendingTx = VMICMODULATOR_VMIC_TX_REQUEST_DELAY_WAIT;
    // If ID is requested we don't need to switch to measurement, just declare the Tx session is completed
    if( (SourceSelected == MEASSLCT_ID_MSB) || (SourceSelected == MEASSLCT_ID_LSB) || (SourceSelected == MEASSLCT_CONFIGURATION_REGISTER10) || (SourceSelected == MEASSLCT_CONFIGURATION_REGISTER11)) 
      vmicmodemPendingTx = VMICMODULATOR_VMIC_TX_REQUEST_IDLE;
    else
      xTimerStart(vmicmodemTxTimerHandler, 10); 
    break;
  case VMICMODULATOR_VMIC_TX_REQUEST_DELAY_WAIT:
    // Pending VMIC session will be terminated by modem end of Tx callback, The callback will send new VMIC command to 
    // change the VMIC to measurment mode.
    vmicmodemMeasurementSourceSelect(VMICMODEM_SELECT_REAL_MEMS_SELECT);
    vlapDemodulatorTxReq(VMICMODEM_VMIC_CONFIG_WORD_2, 0);
    // Change the state to wait for the end of the next tramnission 
    vmicmodemPendingTx = VMICMODULATOR_VMIC_SECOND_TX_REQUEST_WAIT;
    break;
  case VMICMODULATOR_VMIC_SECOND_TX_REQUEST_WAIT:
    // If the application called the vlapDemodulatorTxReq(,) specified end of transmission call back than call the callback function
    if(TxRequestEntry.AppEndOfSourceChangeFunctionPtr)
      TxRequestEntry.AppEndOfSourceChangeFunctionPtr();
    //VmicModemRxModeSet();
    // Turnon the autoresonance after delay 
    //autoresonanceControl(AUTORESONANCE_CONTROL_DELAYED_ON);
    // Change the session to IDLE so the callback of this VMIC measurement command will do nothing.
    vmicmodemPendingTx = VMICMODULATOR_VMIC_TX_REQUEST_IDLE;
    break;
  }

  return(vmicmodemPendingTx);
}



/******************************************************************************
* @brief  void vlapmodemConfigWordSet( vmicmodemConfigWordSelectT ConfigWordSelect , uint32_t NewConfigWord)
* @param  
* @retval 
******************************************************************************/
void vlapmodemConfigWordSet( vmicmodemConfigWordSelectT ConfigWordSelect , uint32_t NewConfigWord)
{
  
  switch(ConfigWordSelect)
  {
    case VMICMODEM_VMIC_CONFIG_WORD_0:
      break;
    case VMICMODEM_VMIC_CONFIG_WORD_1:
      break;
    case VMICMODEM_VMIC_CONFIG_WORD_2:
      ExtDeviceToVmicDataUnion2.Data2 = NewConfigWord;
      break;
    case VMICMODEM_VMIC_CONFIG_WORD_3:
      ExtDeviceToVmicDataUnion3.Data3 = NewConfigWord;
      break;
  }
}

/******************************************************************************
* @brief  uint32_t  vmicmodemDemodulationPtpSignalGet()
* @param  
* @retval 
******************************************************************************/
uint32_t  vmicmodemDemodulationPtpSignalGet()
{
  return(ModulatedSignalPeakToPeakFilter);
}


/******************************************************************************
* @brief  uint32_t  vmicmodemCrcCounterGet()
* @param  
* @retval 
******************************************************************************/
uint32_t  vmicmodemCrcCounterGet()
{
  return(ModemCrcErrorCounter);
}


/******************************************************************************
* @brief  void  vmicmodemDefaultVmicConfigSet()
* @param  
* @retval 
******************************************************************************/
void  vmicmodemDefaultVmicConfigSet()
{
  LastConfig0SentToVmic.VmicConfigWord0 = 0x60b82;
  LastConfig1SentToVmic.VmicConfigWord1 = 0x70100;
}


/******************************************************************************
* @brief  void  vmicmodemMeasurementSourceSelect(vmicmodemVmicTxSourceSelectCommandT SourceSelect)
* @param  
* @retval 
******************************************************************************/
void  vmicmodemMeasurementSourceSelect(vmicmodemVmicTxSourceSelectCommandT SourceSelect)
{
  
  // TODO: This function should be unifiled with vmicmodemSelect( function
    ExtDeviceToVmicDataUnion2.DataBits2.C2fFrequencySelect      = 0;
    ExtDeviceToVmicDataUnion2.DataBits2.C2fRealMemsSelect       = 0;
    ExtDeviceToVmicDataUnion2.DataBits2.C2fREFMemsSelect        = 0;
    ExtDeviceToVmicDataUnion2.DataBits2.C2fTemperatureSelect    = 0;
    ExtDeviceToVmicDataUnion2.DataBits2.C2fNoiseCompByRealMEMS  = 0;
    ExtDeviceToVmicDataUnion2.DataBits2.C2fNoiseCompByRefMems   = 0;
    ExtDeviceToVmicDataUnion2.DataBits2.Dst0                    = 0;
    ExtDeviceToVmicDataUnion2.DataBits2.Dst1                    = 0;
    ExtDeviceToVmicDataUnion2.DataBits2.StatusMeasure           = 0;

  switch(SourceSelect)
  {
  case VMICMODEM_SELECT_POWER_DOWN_SELECT:
    ExtDeviceToVmicDataUnion2.DataBits2.C2fGeneralC2fPower  = 0;
    break;
  case VMICMODEM_SELECT_NOISE_COMP_BY_REAL_MEMS_SELECT:
    ExtDeviceToVmicDataUnion2.DataBits2.C2fRealMemsSelect       = 1;
    ExtDeviceToVmicDataUnion2.DataBits2.C2fGeneralC2fPower      = 1;
    break;
  case VMICMODEM_SELECT_REAL_MEMS_SELECT:
    ExtDeviceToVmicDataUnion2.DataBits2.C2fNoiseCompByRealMEMS  = 1;
    ExtDeviceToVmicDataUnion2.DataBits2.C2fGeneralC2fPower      = 1;
    break;
  case VMICMODEM_SELECT_NOISE_COMP_BY_REF_MEMS_SELECT:
    ExtDeviceToVmicDataUnion2.DataBits2.C2fNoiseCompByRefMems   = 1;
    ExtDeviceToVmicDataUnion2.DataBits2.C2fGeneralC2fPower      = 1;
    break;
  case VMICMODEM_SELECT_REF_MEMS_SELECT: 
    ExtDeviceToVmicDataUnion2.DataBits2.C2fREFMemsSelect        = 1;
    ExtDeviceToVmicDataUnion2.DataBits2.C2fGeneralC2fPower      = 1;
    break;
  case VMICMODEM_SELECT_INTERNAL_CAP_SELECT:
    ExtDeviceToVmicDataUnion2.DataBits2.C2fREFMemsSelect        = 1;
    ExtDeviceToVmicDataUnion2.DataBits2.C2fGeneralC2fPower      = 1;
    break;
  case VMICMODEM_SELECT_TEMPRATURE_SELECT:
    ExtDeviceToVmicDataUnion2.DataBits2.C2fTemperatureSelect    = 1;
    ExtDeviceToVmicDataUnion2.DataBits2.C2fGeneralC2fPower      = 1;
    break;
  case VMICMODEM_SELECT_FREQUENCY_SELECT:
    ExtDeviceToVmicDataUnion2.DataBits2.C2fFrequencySelect      = 1;
    ExtDeviceToVmicDataUnion2.DataBits2.C2fGeneralC2fPower      = 1;
    break;
  }

}


/******************************************************************************
* @brief  void  vmicmodemC2fSwitchArraySet(uint8_t C2fBankSelect)
* @param  
* @retval 
******************************************************************************/
void  vmicmodemC2fSwitchArraySet(uint8_t C2fBankSelect)
{
  ExtDeviceToVmicDataUnion2.DataBits2.C2fBankSelect = C2fBankSelect;
}


/******************************************************************************
* @brief  void  vmicmodemC2fSwitchArraySet(uint8_t C2fBankSelect)
* @param  
* @retval 
******************************************************************************/
void  vmicmodemSelect(MeasurmentSelect_T  SourceSelect, uint8_t C2fSwitchArray)
{
    // Maps between the internal EXT device parameter and the VMIC register to be written      
    ExtDeviceToVmicDataUnion2.Data2 = 0;
    ExtDeviceToVmicDataUnion2.DataBits2.Dst0 = 0;
    ExtDeviceToVmicDataUnion2.DataBits2.Dst1 = 1;
    ExtDeviceToVmicDataUnion2.DataBits2.StatusMeasure = 1; 
    ExtDeviceToVmicDataUnion2.DataBits2.C2fGeneralC2fPower = 1;
    
    
        
    SourceSelected = SourceSelect; 

    switch(SourceSelect)
    {
    case MEASSLCT_C2F_POWER_DOWN:
      ExtDeviceToVmicDataUnion2.DataBits2.C2fGeneralC2fPower = 0;
      break;
    case MEASSLCT_REAL_MEMS_WITH_NOISE_COMPENSATION:
      ExtDeviceToVmicDataUnion2.DataBits2.C2fNoiseCompByRefMems = 1;
      ExtDeviceToVmicDataUnion2.DataBits2.C2fRealMemsSelect = 1;
      break;
    case MEASSLCT_REAL_MEMS_WITHOUT_NOISE_COMPENSATION:
      ExtDeviceToVmicDataUnion2.DataBits2.C2fRealMemsSelect = 1;
      break;
    case MEASSLCT_REF_MEMS_WITH_NOISE_COMPENSATION:
      ExtDeviceToVmicDataUnion2.DataBits2.C2fNoiseCompByRealMEMS = 1;
      ExtDeviceToVmicDataUnion2.DataBits2.C2fREFMemsSelect =1;
      break;
    case MEASSLCT_REF_MEMS_WITHOUT_NOISE_COMPENSATION:
      ExtDeviceToVmicDataUnion2.DataBits2.C2fREFMemsSelect =1;
      break;
    case MEASSLCT_INTERNAL_REFERENCE_CAPACITOR:
      // Cref is selected when Real MEMS and REF. MEMS are both "0"
      ExtDeviceToVmicDataUnion2.DataBits2.C2fRealMemsSelect = 0;
      ExtDeviceToVmicDataUnion2.DataBits2.C2fREFMemsSelect =  0;
      break;
    case MEASSLCT_TEMPERATURE:
      ExtDeviceToVmicDataUnion2.DataBits2.C2fTemperatureSelect = 1;
      break;
    case MEASSLCT_CALIBRATED_MEAS:  // TODO:   
      break;
    case MEASSLCT_ID_LSB:
      ExtDeviceToVmicDataUnion2.DataBits2.Dst0 = 0;
      ExtDeviceToVmicDataUnion2.DataBits2.Dst1 = 0;
      break;
    case MEASSLCT_ID_MSB: 
      ExtDeviceToVmicDataUnion2.DataBits2.Dst0 = 1;
      ExtDeviceToVmicDataUnion2.DataBits2.Dst1 = 0;
      break;
    case MEASSLCT_CONFIGURATION_REGISTER10:
      ExtDeviceToVmicDataUnion2.DataBits2.Dst0 = 0;
      ExtDeviceToVmicDataUnion2.DataBits2.Dst1 = 1;
      break;
    case MEASSLCT_CONFIGURATION_REGISTER11:
      ExtDeviceToVmicDataUnion2.DataBits2.Dst0 = 1;
      ExtDeviceToVmicDataUnion2.DataBits2.Dst1 = 1;
      ExtDeviceToVmicDataUnion2.DataBits2.C2fGeneralC2fPower = 0;
      break;
    }
    // Reflect back the VMIC source selection to the GUI
    pccpmmAppLayerStruct.VmicRegisters1.C2fMeasurementSelect = SourceSelect;

    ExtDeviceToVmicDataUnion2.DataBits2.C2fFrequencySelect = pccpmmAppLayerStruct.VmicRegisters1.C2fFrequencySelect;
    // If the C2fSwitchArray iz zero, the C2F Switch array is taken form the last value set by the external GUI
    if(C2fSwitchArray)
    {
      ExtDeviceToVmicDataUnion2.DataBits2.C2fBankSelect = C2fSwitchArray;
      pccpmmAppLayerStruct.VmicRegisters1.C2fBinarySwitchArray = C2fSwitchArray;
    }
    else
      ExtDeviceToVmicDataUnion2.DataBits2.C2fBankSelect = pccpmmAppLayerStruct.VmicRegisters1.C2fBinarySwitchArray;
      
    ExtDeviceToVmicDataUnion2.DataBits2.C2fLowNoiseMode = pccpmmAppLayerStruct.VmicRegisters1.LowNoiseMeasurement;
    // Mark the session as pending VMIC Tx session
    pccpmmAppLayerPendingTx = PCCOMMAPPLAYER_VMIC_TX_REQUEST_PENDING;
    // Update the modulator
    vlapmodemConfigWordSet(VMICMODEM_VMIC_CONFIG_WORD_2, ExtDeviceToVmicDataUnion2.Data2);
    vlapDemodulatorTxReq(VMICMODEM_VMIC_CONFIG_WORD_2, 0);
}
                             
/******************************************************************************
* @brief  uint16_t vmicmodemIrr( uint16_t y, uint16_t x)
* @param  
* @retval 
******************************************************************************/
uint16_t vmicmodemIrr( uint16_t y, uint16_t x)
{
  float MyY = y;
  float MyX = x;
  
  // Stored value = [(Old value * 15) + (New sample * 1)] / 16
  MyY =  (MyY * 15 + MyX * 1) / (float)16;
  
  //MyY += (x-MyY)*(float)0.0625;
  
  return( (uint16_t) MyY);
}
  
  
                            
void  vmicmodemAgc(uint32_t PeakToPeakSignal)
{
      switch(AgcState)
      {
      case VMICMODEM_AGC_STATE_HIGH_GAIN:
        if(PeakToPeakSignal > 3750)
        {
          AgcStateCounter = VMICMODEM_AGC_DELAY;
          AgcState = VMICMODEM_AGC_STATE_HIGH_TO_LOW_CHECK;
        }
        break;
      
      case VMICMODEM_AGC_STATE_HIGH_TO_LOW_CHECK:
        if(PeakToPeakSignal > 3750)
        {
          if(IrrCycleCounter)
            IrrCycleCounter--;
          else
          {
            hwdriversModemAdcSignalSourceSelect(ADC_CHANNEL_2);
            vlapmainDebugLog("AGC Low");
            AgcState = VMICMODEM_AGC_STATE_LOW_GAIN;
          }
        }
        else
           AgcState = VMICMODEM_AGC_STATE_HIGH_GAIN;
        break;
      
      case VMICMODEM_AGC_STATE_LOW_GAIN:
        if(PeakToPeakSignal < 980)
        {
          AgcStateCounter = VMICMODEM_AGC_DELAY;
          AgcState = VMICMODEM_AGC_STATE_LOW_TO_HIGH_CHECK;
        }
        break;
      
      case VMICMODEM_AGC_STATE_LOW_TO_HIGH_CHECK:
        if(PeakToPeakSignal < 980)
        {
          if(IrrCycleCounter)
            IrrCycleCounter--;
          else
          {
            hwdriversModemAdcSignalSourceSelect(ADC_CHANNEL_1);
            vlapmainDebugLog("AGC High");
            AgcState = VMICMODEM_AGC_STATE_HIGH_GAIN;
          }
        }
        else
           AgcState = VMICMODEM_AGC_STATE_LOW_GAIN;
        break;
      }
}

uint8_t vmicmodemSendingDataToVmicInProgress()
{
	if(ModemState == VMICMODEM_MODEM_STATE_TX)
		return(1);
	else
		return(0);
}
