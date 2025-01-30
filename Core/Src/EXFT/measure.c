#include "vlapConfig.h"
#include "measure.h"
#include "semphr.h"
#include "generic.h"
#include "vlapMain.h"
#include "pccommAppLayer.h"
#include "vmicmodem.h"
#include "pccommAppLayer.h"
#include "autopower.h"
#include "autoresonance.h"
#include "audit.h"
#include "config.h"


// L O C A L    D E F I N I T I O N S
typedef enum{MEASURE_CHANNEL_SELECT_AVRG_INDEX_IC_63=0, MEASURE_CHANNEL_SELECT_AVRG_INDEX_IC_87=1 , MEASURE_CHANNEL_SELECT_AVRG_INDEX_IC_127=2, MEASURE_CHANNEL_SELECT_AVRG_INDEX_REF_127=3, MEASURE_CHANNEL_SELECT_AVRG_INDEX_TEMP_127=4, MEASURE_CHANNEL_SELECT_AVRG_INDEX_REALMEMS_127=5}  MeasureChennelSelectAvrgIndex_T;


//#define MEASURE_PUSH_BUTTON_SIMULATE

#define MEASURE_BELT_VMIC_DETECT_CURRENT_LEVEL_THRESHOLD_IN_MA    3000          // Should be 3000 for 3A
#define MEASURE_CURRENT_DELTA_THRESHOLD                           6
#define MEASURE_RESONANCE_STEP                                    10
#define MEASURE_RESONANCE_STEPS                                   AUTORESONANCE_MAX_RELAYS_VAL/MEASURE_RESONANCE_STEP
#define MEASUREMENT_STABILITY_MAX_ITERATIONS                      33
#define MEASUREMENT_STABILITY_DELTA                               5
#define MEASAURE_NUMBER_OF_CONSEQUATIVE_SUCCESS_RETRIES           2

#define MEASURE_ID_SOURCE_SELECTION_TIME                          6000
#define MEASURE_NEW_SOURCE_SELECTION_STABILITY_TIME               3000
#define MEASURE_NEW_SOURCE_SELECTION_TIME                         4000
#define MEASURE_MAX_PSK_LEVEL                                     236
#define MEASURE_STABILITY_ITERATION_TIME                          600
#define MEASURE_STABILITY_SLOW_ITERATION_TIME                     2000

#define MEASURE_MINIMAL_MODEM_LOCK_DELAY                          600
#define MEASURE_MINIMAL_ENERGY_P2P_SAMPLE                         40
#define MEASURE_MINIMIAL_ENERGY_ITERATION_TIME_SLOW               1000
#define MEASURE_MINIMIAL_ENERGY_ITERATION_TIME_FAST               600
#define MEASURE_MINIMAL_ENERGY_START_PSL                          10
#define MEASURE_MINIMAL_ENERGY_LOW_STEP                           2
#define MEASURE_MINIMAL_ENERGY_LOW_HIGH_STEP                      5
#define MEASURE_MINIMAL_ENERGY_MID_STEP                           10
#define MEASURE_MINIMAL_ENERGY_HIGH_STEP                          20

#define MEASURE_SELECT_MAX_RETRY                                  5

#define MEASURE_DEBUG_MODE                                        0
#define MANUAL_RESONANCE                               43 // 49: long belt , 55 Medium belt // 45 for Saar with small belt

#define MEASURE_TARGET_PTP_SIGNAL                                3900
#define MEASURE_TARGET_PTP_PID_GAIN                              (float)(0.14)         

typedef struct   
{
  float  MinValue;
  float  MaxValue;
} MeasurementVmicSourceLimitsStructT;

typedef enum {MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_IDLE, MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_WAIT, MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_FAIL, MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_MEASURE } measureVmicSourceSelectMonitorFsmState_T;
typedef struct
{
  measureVmicSourceSelectMonitorFsmState_T State;
  MeasurementVmicSourceLimitsStructT LimitEntry;
  MeasurmentSelect_T VmicSourceSelectForRetry;
  uint8_t  VmicSourceSelectC2fSwitchArrayForRetry;
  uint8_t  RetryCounter;
  uint16_t  SampleCounter;
  uint16_t ValidSamplesCounter;
  uint16_t UnsavedSamples;
  uint8_t  WaitTicksCounter;
  float PslPercentageIncrease;
} measureVmicSourceSelectMonitorFsmDb_T;
#define MEASURE_VMIC_SOURCE_SELECT_STABILITY_COUNT 150
#define MEASURE_VMIC_SOURCE_SELECT_SAMPLE_COUNT    4
#define MEASURE_VMIC_SOURCE_SELECT_MONITOR_AVERAGE_WIDTH        10000




// L O C A L    P R O T O T Y P E S
ReturnCode_T measureIdleStateChange(uint16_t QueueWairTimeoutIn1mSec, uint8_t sendMeasureFailedEvent);
ReturnCode_T measureModemStateTurnOff();
uint32_t MeasurementDeltaGet(uint32_t Measurment1, uint32_t Measurment2);
ReturnCode_T  measurmentNurseModeEventBuildAndCreate();
ReturnCode_T measureTaskEventSend(MeasureReq_T NotificationToCommTask , MeasureTaskQueueSourceT IsrOrNonIsrSource);
void MeasureFsmTimerTimeoutCallback();
uint8_t measurementConditionCheck();
measureVmicSourceSelectMonitorFsmState_T measureVmicSourceSelectMonitorFsmInit(MeasureChennelSelectAvrgIndex_T VmicSourceSelectAverageIndex, MeasurmentSelect_T VmicSource, uint8_t C2fSwitchArray, float PslPercentageIncrease);
measureVmicSourceSelectMonitorFsmState_T measureVmicSourceSelectMonitorFsm(uint32_t Sample, uint16_t SamplesToTest);
uint16_t measureSamplesReducedBySeqNumber();
uint16_t StabilityIterationTimeGet();
uint16_t MinimalEnergyIterationTimeGet();
void measureEndMeasurementSeq();
void measurementEndedEventSimulateCallBAck();


// M O D U L E   G L O B A L S

//                                                                   IC1_63 ,       IC2_87 ,        IC3_127,        REF1_127,       TEMP_127,       REALMEMS, 
const MeasurementVmicSourceLimitsStructT  VmicSourceLimitsArray[] = {{77000,87000}, {55000, 65000}, {38000, 45000}, {72000, 85000}, {90000,150000}, {40000,90000}};
QueueHandle_t measureReqQ;
TimerHandle_t MeasureFsmTimerHandler;

MeasureFsmState_T MeasureFsmState;
uint16_t ReturnedRealTimeCurrent1mA;
uint32_t TimeoutCounter;
TickType_t FsmDelayTicks; 

uint32_t VmicId;
uint32_t LastRecievedPayload;
uint8_t PowerAdjustIterationCounter;

uint32_t ResonanceEnergyLevelArray[MEASURE_RESONANCE_STEPS];
uint16_t ResonanceEnergyLevelArrayIndex;
uint16_t MaxInitialRelayState;
uint32_t CurrentMuxInitialEnergy;

uint8_t MaxResonancePeakState; 
uint8_t MaxResonanceStartIndex;
uint8_t MaxResonanceEndIndex;

uint16_t CurrrentPskLevel;
uint32_t MeasurementIndex;
uint16_t PeakToPeakInBandCounter;

uint16_t NursePskLevel;
uint8_t NurseModeFirstBuzzerSet = 1;
uint32_t NurseModeLastFreq;
uint8_t NurseModeLevelFound = 0;
uint8_t NurseModeIdleCount = 0;
uint8_t NurseModeFreqChange = 3;
uint8_t NurseModeFreqChangeCounterUp = 0;
uint8_t NurseModeFreqChangeCounterDown = 0;
uint8_t NurseModeMaxIndicationLevel = 0;
uint8_t measureNurseModeDone;

uint32_t NurseModeStartTime;
uint32_t NurseModeStopTime;

uint32_t measureStartTime;
uint32_t measureStopTime;

ProtocolappMeasurementEndedEvent_t MeasurementsResults; 

uint16_t measureBeltHumanCheckCounter = 0;

uint16_t measureMaxOperationPsl;

uint8_t measureDelayMinimalPSL = MEASURE_MINIMAL_ENERGY_P2P_SAMPLE;
uint32_t measureAvgP2P = 0;
uint8_t delayCalc;

uint8_t measureIDReadIncreasment = 0;

typedef enum { MEASUREMENT_PRE_CONFIGURED_STATE = 0, MEASUREMENT_CONFIGURED_STATE = 1}  measurementConfigurationState_T;
measurementConfigurationState_T measurementConfigurationState;
measureVmicSourceSelectMonitorFsmDb_T measureVmicSourceSelectMonitorFsmDb;
uint16_t MeasurementCounter;
static  char PrintBuff[150];

// Accel parameters
int16_t measureAccXStart;
int16_t measureAccYStart;
int16_t measureAccZStart;

int16_t measureAccXPreReal;
int16_t measureAccYPreReal;
int16_t measureAccZPreReal;

TimerHandle_t measureTimeoutTimerHandler;

uint8_t measureIdRetry = 0;
uint8_t measureSourceSelectTimeoutRetry = 0;
uint8_t measureStabilitySampleRetry = 0;
uint8_t measureShortRealMemsTimeout = 0;
uint32_t measureRealMemsBadFramesCounter = 0;
uint32_t measureRealMemsBadFramesPerSample = 0;
uint32_t measureRealMemsOutOfRangeFramesPerSample = 0;

uint8_t measurePslStep;
uint16_t measureModemLockDelay;
uint16_t measureNumberOfSamples;


volatile AAA;
void MEASURE_FSM_STATE_CHANGE(MeasureFsmState_T NewState, uint32_t QueueWairTimeoutIn1mSec)
{
  if (NewState != MeasureFsmState)
  {
    MeasureFsmState = NewState;
    measureProgressLedsSequence(NewState);
  }
  if(QueueWairTimeoutIn1mSec)
    xTimerChangePeriod( MeasureFsmTimerHandler, QueueWairTimeoutIn1mSec, 100);
  else
    AAA = 0;
}




/******************************************************************************
* @brief  void measureInit()
* @param  
* @retval 
******************************************************************************/
ReturnCode_T measureInit(void)
{
  // Create the  queue  
  measureReqQ = xQueueCreate(3, sizeof(measureReqQueueEntry_T));
  // FSM initial state
  MeasureFsmState = MEASURE_IDLE;
  FsmDelayTicks = 1000; 
  // CReate the task
  xTaskCreate(measureTask, measureTaskName, measureTaskSTACK_SIZE, NULL, 
              measureTaskPriority, ( TaskHandle_t * ) NULL );
  
  MeasureFsmTimerHandler =  xTimerCreate("MeasureTaskFsmTimer",  portTICK_PERIOD_MS, pdFALSE, (void *)0, MeasureFsmTimerTimeoutCallback);
  xTimerStart(MeasureFsmTimerHandler, 100);
  
  MEASURE_FSM_STATE_CHANGE(MEASURE_IDLE, 200);
  pccpmmAppLayerStruct.BoardSystemRegisters.OnBoardPushButton = 0;
  measurementConfigurationState = MEASUREMENT_PRE_CONFIGURED_STATE;
  
  MeasurementCounter = 0;
  measureTimeoutTimerHandler =  xTimerCreate("measureTimeoutTimerHandler",  portTICK_PERIOD_MS, pdFALSE, (void *)0, measureTimeoutTimerCallback);
  
  // Always return OK
  return(RETURNCODE_OK);
}

ReturnCode_T measureInitializeAccelParams()
{
  measureAccXStart = 0;
  measureAccYStart = 0;
  measureAccZStart = 0;
  
  measureAccXPreReal = 0;
  measureAccYPreReal = 0;
  measureAccZPreReal = 0;    
  // Always return OK
  return(RETURNCODE_OK);
}

/*******************************************************************************
* Function Name  : portTASK_FUNCTION(measureTask, pvParameters )
* Description    : 
* Input          : 
*                  
*                  
* Output         : None
* Return         : None
*******************************************************************************/
portTASK_FUNCTION(measureTask, pvParameters )
{
  measureReqQueueEntry_T QueueEntry;
  typesRcvQueueStatus_T RcvQueueStatus;
  uint16_t i;
   uint16_t MyTransCurrentMonitor;
  uint16_t txMonitor = 0;
  //float PtpSignalError;
  uint8_t energyMargin;
  uint16_t delay;
  

  // Block the task till configuration is ready
  switch(measurementConfigurationState)
  {
  case MEASUREMENT_PRE_CONFIGURED_STATE:
    xSemaphoreTake( configConfigurationValidSemaphoreHandle, portMAX_DELAY );
    xSemaphoreGive( configConfigurationValidSemaphoreHandle);
    measurementConfigurationState = MEASUREMENT_CONFIGURED_STATE;
    break;
  case MEASUREMENT_CONFIGURED_STATE:
  default:
    break;
  }
  
  
  
  while(1)
  {
    // Wait for queue event
    RcvQueueStatus = (typesRcvQueueStatus_T) xQueueReceive(measureReqQ, &QueueEntry, 30000); 
    if(RcvQueueStatus == TYPES_RCVQUEU_RCV)
    {
      if((QueueEntry.MeasureRequest == MEASURE_PB_START) && (MeasureFsmState != MEASURE_IDLE) && (MeasureFsmState != MEASURE_NURSE_MODE_STATE) && (! hwdriversGpioBitRead(HWDRIVERS_PUSH_BUTTON_IN) ) )
      {
        uint8_t sendEndMeasureEvent = 0;
        if(MeasureFsmState > MEASURE_MINIMAL_ENERGY_WAIT)
        {
          sendEndMeasureEvent = 1;
        }
        
        MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_SHORT_PRESS;
        eventsEventWrite(0,0,PROTOCOLAPP_MEASUREMENT_ABORTED_SHORT_PUSH_BUTTON_EVENT,0,0,0);
        measureIdleStateChange(100, sendEndMeasureEvent);
        // Assume unknown UniqueIdSerialNumber
        pccpmmAppLayerStruct.VmicRegisters3.UniqueIdSerialNumber = TYPES_ENDIAN16_CHANGE(0);
      }
      else
        switch(MeasureFsmState)
        {
        case MEASURE_IDLE:
          vmicmodemBeltMasking = false;
          switch(QueueEntry.MeasureRequest)
          {
          case MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_SAMPLE:
            break;
          default:
            if(RcvQueueStatus == TYPES_RCVQUEU_RCV)
            {
            	// Wait for user push button press message to initiate the measurement process and to go out of sleep mode
              if(QueueEntry.MeasureRequest == MEASURE_PB_START) 
              {
                // Initialize all the accelerometer paramters before start of a sequence
                measureInitializeAccelParams();
                MeasurementsResults.MeasurementFailureReason = 0;
                MEASURE_FSM_STATE_CHANGE(MEASURE_PUSH_BUTTON_BOUNCING_DELAY, 2000);
              }
            }
            break;
          }
          break;
          
          //      MEASURE_PUSH_BUTTON_BOUNCING_DELAY:
        case MEASURE_PUSH_BUTTON_BOUNCING_DELAY:
          switch(QueueEntry.MeasureRequest)
          {
          case MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_SAMPLE:
            break;
          case MEASURE_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:
            
//              if( (!hwdriversGpioBitRead(HWDRIVERS_PUSH_BUTTON_IN))  || pccpmmAppLayerStruct.BoardSystemRegisters.OnBoardPushButton)
              if( (!HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14))  || pccpmmAppLayerStruct.BoardSystemRegisters.OnBoardPushButton)
            {
            	  measurePslStep = 0;
                  measureModemLockDelay = configConfigurationDb.MeasurementModemLockDelay;
                  if(measureModemLockDelay < MEASURE_MINIMAL_MODEM_LOCK_DELAY)
                    measureModemLockDelay = MEASURE_MINIMAL_MODEM_LOCK_DELAY;


                  // In sequence 109 there is smaller array size to measure
                  if(configConfigurationDb.MeasurementSequence == 109)
                    measureNumberOfSamples = PROTOCOLAPP_GENERAL_SMALL_ARRAY_SIZE;
                  else
                    measureNumberOfSamples = PROTOCOLAPP_GENERAL_ARRAY_SIZE;

            	  // TurnOn the device out from sleep mode
              //              vlapmainDemodulatorTaskControl( VLAPMAIN_TASK_START);
              pccpmmAppLayerStruct.BoardSystemRegisters.OnBoardPushButton = 1;
              if(configConfigurationDb.MeasurementPslControl == 0)
              {
                CurrrentPskLevel = configConfigurationDb.MeasurementManualPslForSequence;
              } 
              else 
              {
                CurrrentPskLevel = configConfigurationDb.MinimalSequencePsl; //MEASURE_MIN_PSK_LEVEL;
              }
              autopowerManaualPwmSet(CurrrentPskLevel);              
              pccommapplayerTransmitterControl(TYPES_ENABLE);
              //pccommapplayerAutoCurrentControl(TYPES_ENABLE);
              //autoresonanceManaualRelaysSet(MANUAL_RESONANCE);
              // Set the Auto Current to 6A
              //              pccommapplayerAutoCurrentSet(MEASURE_BELT_VMIC_DETECT_CURRENT_LEVEL_THRESHOLD_IN_MA+500);
              // Assume unknown UniqueIdSerialNumber
              pccpmmAppLayerStruct.VmicRegisters3.UniqueIdSerialNumber = TYPES_ENDIAN16_CHANGE(0);
              ResonanceEnergyLevelArrayIndex=0;
              MaxInitialRelayState =0;
              CurrentMuxInitialEnergy = 0;
              vlapmainDebugLog("PushButton");
              // Increment the measurement counter
              MeasurementCounter++;
              // Clear the measurement data structure 
              for(i=0; i<sizeof(ProtocolappMeasurementEndedEvent_t); i++)
                ((uint8_t *)&MeasurementsResults)[i] = 0;
                
              
              //            sprintf(PrintBuff, "PushButton  VLAP_ID = Date= %d, Psl= %d", vmicapplayerFilteredSamplesGet(), CurrrentPskLevel);
              vlapmainDebugLog(PrintBuff);
              
              if(auditAbiantTemperatureStateGet()!=AUDIT_TEMPERATURE_NORMAL)
              {
                MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_OVER_TEMPERATURE;
                eventsEventWrite(0,0,PROTOCOLAPP_MEASUREMENT_ABORTED_TEMPERATURE_LEVEL_EVENT,0,0,0);
                sprintf(PrintBuff, "Tx State temperature Out of range");
                vlapmainDebugLog(PrintBuff);
                
                measureIdleStateChange(10, 0);
                break;
              }
              
              // Stop the FW upgrade in case that it is currently downloading
//              fwupgradeEventSend(FWUPGRADE_OPCODE_STOP_DOWNLOAD, 0, 0);
              
              // Send abort message to the commTask to abort the current communication session
//              commTaskEventSend(COMM_FSM_QUEUE_ENTRY_TYPE_COMM_ABORT);
              
              //MEASURE_FSM_STATE_CHANGE(MEASURE_CHECK_USB_CONNECTED, 10);
              MEASURE_FSM_STATE_CHANGE(MEASURE_CHECK_PLUG_IN_NURSE_MODE, 10);
            }
            else
            {
              MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_SHORT_PRESS;
              eventsEventWrite(0,0,PROTOCOLAPP_MEASUREMENT_ABORTED_SHORT_PUSH_BUTTON_EVENT,0,0,0);
              measureIdleStateChange(1, 0);
            }
            break;
            default:
              break;
          }
          break;
          //      MEASURE_CHECK_PLUG_IN_NURSE_MODE:
        case MEASURE_CHECK_PLUG_IN_NURSE_MODE:
          switch(QueueEntry.MeasureRequest)
          {
          case MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_SAMPLE:
            break;
          default:
            //if(auditNurseModeStatusGet() == AUDIT_NURSE_MODE_ENABLED)
            if((configConfigurationDb.PatientNurseMode == 1) && (!measureNurseModeDone))
            {
              sprintf(PrintBuff, "In nurse mode");
              vlapmainDebugLog(PrintBuff);
#if MEASURE_DEBUG_MODE
              NursePskLevel = 1;
              autopowerManaualPwmSet(NursePskLevel);      
              if(configConfigurationDb.AutoResonaceControl)
              {
                pccommapplayerAutoResonanceControl(TYPES_ENABLE);
              }
              else
              {
                pccommapplayerAutoResonanceControl(TYPES_DISABLE);
                autoresonanceManaualRelaysSet(configConfigurationDb.ManualRelayStateForSequence);               
              }
              MEASURE_FSM_STATE_CHANGE(MEASURE_DEBUG_FOR_NURSE_MODE, 10);   
#else
              NursePskLevel = configConfigurationDb.NurseModePsl;
              autopowerManaualPwmSet(NursePskLevel);
              if(configConfigurationDb.AutoResonaceControl)
              {
//                pccommapplayerAutoResonanceControl(TYPES_ENABLE);
              }
              else
              {
                pccommapplayerAutoResonanceControl(TYPES_DISABLE);
                autoresonanceManaualRelaysSet(configConfigurationDb.ManualRelayStateForSequence);
              }

              NurseModeFirstBuzzerSet = 1;
              MEASURE_FSM_STATE_CHANGE(MEASURE_NURSE_MODE_STATE, 3000);  
#endif
            }
            else
            {
              MEASURE_FSM_STATE_CHANGE(MEASURE_CHECK_USB_CONNECTED, 10);
            }     
            break;
          }
          break;
          
          //      MEASURE_DEBUG_FOR_NURSE_MODE:
        case MEASURE_DEBUG_FOR_NURSE_MODE:
          switch(QueueEntry.MeasureRequest)
          {
          case MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_SAMPLE:
            break;
          default:
            
            if(NursePskLevel < configConfigurationDb.NurseModePsl)
            {
              NursePskLevel++;
              autopowerManaualPwmSet(NursePskLevel); 
              NurseModeStartTime = rtcEpochGet();
              MEASURE_FSM_STATE_CHANGE(MEASURE_DEBUG_FOR_NURSE_MODE, 1000);
            }
            else
            {
              NurseModeFirstBuzzerSet = 1;
              MEASURE_FSM_STATE_CHANGE(MEASURE_NURSE_MODE_STATE, 600);
            }
            break;
          }
          break;
          
          //      MEASURE_NURSE_MODE_STATE:
        case MEASURE_NURSE_MODE_STATE:
          switch(QueueEntry.MeasureRequest)
          {
          case MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_SAMPLE:
            break;
          default:
            //Check if plug plugged out.
            //if (auditNurseModeStatusGet() == AUDIT_NURSE_MODE_DISABLED)
            if(QueueEntry.MeasureRequest == MEASURE_PB_START)
            {
              buzzerOff();
              pccommapplayerTransmitterControl(TYPES_DISABLE);
              NurseModeStopTime = rtcEpochGet();
              MEASURE_FSM_STATE_CHANGE(MEASURE_NURSE_MODE_END_WAIT, 100);
              //TODO : Stop nurse mode and go to publish state.
              break;
            }
            
            uint32_t peakToPeakMeasure = vmicapplayerSignalPeakToPeakValueGet(); 
            uint32_t peakToPeakSignalMeasure = vmicmodemDemodulationPtpSignalGet();
            
            //printf("I'm in nurse mode\n");          
            
            measureNurseModeIndication(peakToPeakSignalMeasure, peakToPeakMeasure);
            //MEASURE_FSM_STATE_CHANGE(MEASURE_NURSE_MODE_STATE, 25);
            // TODO: Perhaps we can do it faster?
            MEASURE_FSM_STATE_CHANGE(MEASURE_NURSE_MODE_STATE, 120);
            break;
          }
          break;
          
          //      MEASURE_NURSE_MODE_END_WAIT:       
        case MEASURE_NURSE_MODE_END_WAIT:       
          switch(QueueEntry.MeasureRequest)
          {
          case MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_SAMPLE:
            break;
          default:
            measurmentNurseModeEventBuildAndCreate();     
            MEASURE_FSM_STATE_CHANGE(MEASURE_IDLE, 1000);
            break;            
          }
          break;
          
          //      MEASURE_CHECK_USB_CONNECTED:          
        case MEASURE_CHECK_USB_CONNECTED:  
          switch(QueueEntry.MeasureRequest)
          {
          case MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_SAMPLE:
            break;
          default:
//            if(chargerDcPlugStatusGet() == CHARGER_DC_PLUG_INSERTED)
//            {
//              buzzerRequestToQueueAdd(BUZZER_BEEPING_OPTION4);
//              vibratorRequestToQueueAdd(VIBRATOR_OPTION4);
//              ledsPatternSet(LEDS_MAIN_PATTERN_MEAS_CHARGING);
//              MEASURE_FSM_STATE_CHANGE(MEASURE_CHECK_USB_CONNECTED, 1000);
//            }
//            else
//            {
//              //Mark that measure starts.
//              ledsPatternSet(LEDS_MAIN_PATTERN_MEAS_START);
//              buzzerRequestToQueueAdd(BUZZER_BEEPING_OPTION1);
//              vibratorRequestToQueueAdd(VIBRATOR_OPTION1);
//#if 1
//              if(configConfigurationDb.AutoResonaceControl)
//              {
//                pccommapplayerAutoResonanceControl(TYPES_ENABLE);
//              }
//              else
//              {
//                pccommapplayerAutoResonanceControl(TYPES_DISABLE);
//                autoresonanceManaualRelaysSet(configConfigurationDb.ManualRelayStateForSequence);
//              }
//#endif
//              // Set psl to 10 by default for belt and human check
//              //MEASURE_FSM_STATE_CHANGE(MEASURE_MINIMAL_ENERGY_WAIT, 10000);
//              measureMaxOperationPsl = configConfigurationDb.MaximalSequencePsl;
//              if(configConfigurationDb.MeasurementPslControl == 0) //TODO: This doesnt exist in VTS config
//              {
//                autopowerManaualPwmSet(configConfigurationDb.MeasurementManualPslForSequence);
//              }
//              else
//              {
//                autopowerManaualPwmSet(10);
//              }
//              measureBeltHumanCheckCounter = 20;
//              // Mark that if the Nurse mode is, the next PB will go to Nurse mode
//              measureNurseModeDone = 0;
// //             MEASURE_FSM_STATE_CHANGE(MEASURE_MINIMAL_ENERGY_WAIT, 10000);
//            MEASURE_FSM_STATE_CHANGE(MEASURE_CHECK_BELT_AND_HUMAN_STATUS, 2000);
//            }
            break;
          }
          break;
          
          //      MEASURE_CHECK_BELT_AND_HUMAN_STATUS:
        case MEASURE_CHECK_BELT_AND_HUMAN_STATUS:
#if 1         
          if(measurementConditionCheck())
          {
            measureIdleStateChange(10, 0);
            break;
          }
          else
          {
            switch(QueueEntry.MeasureRequest)
            {
             default:
              break;
            case MEASURE_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:
              // Check low battery after transmitting 10 PSL and abort measurement
              if((auditBatteryStatusGet() == AUDIT_BV_CROSSED_LB) || (auditBatteryStatusGet() == AUDIT_BV_CROSSED_LB_SENDED))
              {
                sprintf(PrintBuff, "Aborting measurement due to low battery level!");
                vlapmainDebugLog(PrintBuff);
                
                MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_LOW_BATTERY;
                eventsEventWrite(0,0,PROTOCOLAPP_MEASUREMENT_ABORTED_BATTERY_LEVEL_EVENT,0,0,0);
                
                measureIdleStateChange(10, 0);
                break;
              }
              

              AuditBeltHumanCheck_T auditBeltHumanStatus = auditBeltHumanStatusGet();
              sprintf(PrintBuff, "Measured Modulated Peak2Peak= %d", vmicmodemDemodulationPtpSignalGet());
              vlapmainDebugLog(PrintBuff);
  
               switch(auditBeltHumanStatus)
              {
               case AUDIT_BELT_OPEN:
                vlapmainDebugLog("Belt is open, aborting measure");
                sprintf(PrintBuff, "Belt is open, aborting measure");
                // In case that belt is open, go to idle and indicate on it
                eventsEventWrite(0,0,PROTOCOLAPP_MEASUREMENT_ABORTED_BELT_OPEN_EVENT,0,0,0);
                MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_BELT_OPEN;
                measureIdleStateChange(10, 0);
                break;
              case AUDIT_BELT_NO_HUMAN:
                vlapmainDebugLog("There is no human, measuring anyway, starting MEASURE_MINIMAL_ENERGY_WAIT");
                //eventsEventWrite(0,0,PROTOCOLAPP_MEASUREMENT_ABORTED_BELT_CLOSED_NO_HUMAN_EVENT,0,0,0);

                // Clear vmic registers
                VmicId = 0;
                VmicToExtDeviceStatusUnion0.Status0 = 0;
                VmicToExtDeviceStatusUnion1.Status1 = 0;
                VmicToExtDeviceStatusUnion2.Status2 = 0;
                VmicToExtDeviceStatusUnion3.Status3 = 0;
                
                // Save measurement start time
                measureStartTime = rtcEpochGet();
                
                // Get accelerometer parameters for the start of measurement
//                measureAccXStart = sensorAccGet(SENSOR_AXIS_X);
//                measureAccYStart = sensorAccGet(SENSOR_AXIS_Y);
//                measureAccZStart = sensorAccGet(SENSOR_AXIS_Z);
                
                // Start capturing P2P for accelerometer axis
                sensorStartCaptureP2PAcc(measureAccXStart, measureAccYStart, measureAccZStart);
                
                // Start timout timer for measurement sequence.
                xTimerChangePeriod(measureTimeoutTimerHandler, configConfigurationDb.MeasurementTimeoutSeconds * 1000, 100); 
                // If one of the configuration fields are higher then 20 - set it to 20.
                if(configConfigurationDb.PslSequenceStepPercentage > 20) { configConfigurationDb.PslSequenceStepPercentage = 20;}
                if(configConfigurationDb.MeasurementMinimalPslMarginPercentage > 20) { configConfigurationDb.MeasurementMinimalPslMarginPercentage = 20;}
                if(configConfigurationDb.MeasurementMarginPercentage > 20) { configConfigurationDb.MeasurementMarginPercentage = 20;}
                

                MEASURE_FSM_STATE_CHANGE(MEASURE_DELAY_BEFORE_START, 1);
                break;
              case AUDIT_BELT_HUMAN_OK:
                vlapmainDebugLog("Belt is close and there is human, starting MEASURE_MINIMAL_ENERGY_WAIT");           
                // Clear vmic registers
                VmicId = 0;
                VmicToExtDeviceStatusUnion0.Status0 = 0;
                VmicToExtDeviceStatusUnion1.Status1 = 0;
                VmicToExtDeviceStatusUnion2.Status2 = 0;
                VmicToExtDeviceStatusUnion3.Status3 = 0;
                
                                // Save measurement start time
                measureStartTime = rtcEpochGet();
                
                // Get accelerometer parameters for the start of measurement
//                measureAccXStart = sensorAccGet(SENSOR_AXIS_X);
//                measureAccYStart = sensorAccGet(SENSOR_AXIS_Y);
//                measureAccZStart = sensorAccGet(SENSOR_AXIS_Z);
                
                // Start capturing P2P for accelerometer axis
                sensorStartCaptureP2PAcc(measureAccXStart, measureAccYStart, measureAccZStart);
                
                // Start timout timer for measurement sequence.
                xTimerChangePeriod(measureTimeoutTimerHandler, configConfigurationDb.MeasurementTimeoutSeconds * 1000, 100); 
              
                // If one of the configuration fields are higher then 20 - set it to 20.
                if(configConfigurationDb.PslSequenceStepPercentage > 20) { configConfigurationDb.PslSequenceStepPercentage = 20;}
                if(configConfigurationDb.MeasurementMinimalPslMarginPercentage > 20) { configConfigurationDb.MeasurementMinimalPslMarginPercentage = 20;}
                if(configConfigurationDb.MeasurementMarginPercentage > 20) { configConfigurationDb.MeasurementMarginPercentage = 20;}
                                
                measureModemLockDelay = configConfigurationDb.MeasurementModemLockDelay;
                if(measureModemLockDelay < MEASURE_MINIMAL_MODEM_LOCK_DELAY)
                {
                  measureModemLockDelay = MEASURE_MINIMAL_MODEM_LOCK_DELAY;
                }
                     
                MEASURE_FSM_STATE_CHANGE(MEASURE_DELAY_BEFORE_START, 1);
                break;
              default:
            	  break;
              }
              break;
            }
          }
#endif
          break;
          
          
        case MEASURE_DELAY_BEFORE_START:
          if(measurementConditionCheck())
          {
            measureIdleStateChange(10, 0);
            break;
          }
          switch(QueueEntry.MeasureRequest)
          {
          case MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_SAMPLE:
            break;
          default:
            sprintf(PrintBuff, "In MEASURE_DELAY_BEFORE_START state, delay is: %d ms", configConfigurationDb.MeasurementDelayStartMilliSeconds);
            vlapmainDebugLog(PrintBuff);
             
            // Reset the min and max values of autoresonance
            autoresonanceResetP2P();
            

            if(configConfigurationDb.MeasurementDelayStartMilliSeconds)
            {
              delay = configConfigurationDb.MeasurementDelayStartMilliSeconds;
            }
            else
            {
              // If MeasurementDelayStartMilliSeconds is zero, set delay to be 1 milliSecond
              delay = 1;
            }
            if(configConfigurationDb.MeasurementPslControl == 0)
            {
              vlapmainDebugLog("Select Capacitor Source");
              measureVmicSourceSelectMonitorFsmInit(MEASURE_CHANNEL_SELECT_AVRG_INDEX_IC_127, MEASSLCT_INTERNAL_REFERENCE_CAPACITOR, 127, 0.05);
              vmicmodemSelect(MEASSLCT_INTERNAL_REFERENCE_CAPACITOR,127);
              measureStabilityFsmInit();
              measureSourceSelectTimeoutRetry = MEASURE_SELECT_MAX_RETRY;
              MEASURE_FSM_STATE_CHANGE(MEASURE_MINIMAL_ENERGY_IC127_SOURCE_SELECT_SAMPLE_WAIT, MEASURE_NEW_SOURCE_SELECTION_STABILITY_TIME);
            }
            else
            {
              // When MinimalSequencePsl is configured to 0, we skip accelerated power estimation
              if(!configConfigurationDb.MinimalSequencePsl)
              {
                //                  PtpSignalError = ( (float)MEASURE_TARGET_PTP_SIGNAL - (float)vmicmodemDemodulationPtpSignalGet()); 
                //                  if(PtpSignalError > 0)
                //                  {
                //                    CurrrentPskLevel = (uint8_t)(PtpSignalError * MEASURE_TARGET_PTP_PID_GAIN);
                //                    vlapmainDebugLog("Accelerated Power Estimation");
                //                  }
                measureMinimalEnergyFsmInit();
                CurrrentPskLevel = MEASURE_MINIMAL_ENERGY_START_PSL;
                autopowerManaualPwmSet(CurrrentPskLevel);
                measureAvgP2P = 0;
                measureDelayMinimalPSL = MEASURE_MINIMAL_ENERGY_P2P_SAMPLE;
                if(!delay){
                	MEASURE_FSM_STATE_CHANGE(MEASURE_MINIMAL_ENERGY_SEARCH, delay);
                }
                else{
                    MEASURE_FSM_STATE_CHANGE(MEASURE_MINIMAL_ENERGY_SEARCH, 1);
                }

              }
              else
              {
                autopowerManaualPwmSet(CurrrentPskLevel);
                uint8_t energyMargin = (uint8_t)round(CurrrentPskLevel * ((double)configConfigurationDb.MeasurementMinimalPslMarginPercentage / 100));
                if(energyMargin > 0)
                {
                  sprintf(PrintBuff, "Add PSL margin = %d", energyMargin);
                  vlapmainDebugLog(PrintBuff);
                  measureIncreaseAndSetPsl(energyMargin);
                }
                
                measurePslStep = (uint8_t)round(CurrrentPskLevel * ((double)configConfigurationDb.PslSequenceStepPercentage / 100));
                if(measurePslStep < 1)
                {
                  measurePslStep =  1;
                }
                MEASURE_FSM_STATE_CHANGE(MEASURE_MINIMAL_ENERGY_WAIT, delay);
              }
            }

            break;
          }
          break;

          
        case MEASURE_MINIMAL_ENERGY_SEARCH:
          if(measurementConditionCheck())
          {
            measureIdleStateChange(10, 0);
            break;
          }
          switch(QueueEntry.MeasureRequest)
          {
          case MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_SAMPLE:
            break;
          default:
            if(measureDelayMinimalPSL)
            {
              measureAvgP2P += vmicmodemDemodulationPtpSignalGet();
              delayCalc = (uint8_t)(MinimalEnergyIterationTimeGet() / MEASURE_MINIMAL_ENERGY_P2P_SAMPLE);
              MEASURE_FSM_STATE_CHANGE(MEASURE_MINIMAL_ENERGY_SEARCH, delayCalc);
              measureDelayMinimalPSL--;
            }
            else
            {
             measureDelayMinimalPSL = MEASURE_MINIMAL_ENERGY_P2P_SAMPLE;
              measureAvgP2P = measureAvgP2P / MEASURE_MINIMAL_ENERGY_P2P_SAMPLE;
              switch(measureMinimalEnergyFsm(measureAvgP2P))
              {
              case MEASURE_MINIMAL_ENERGY_IDLE:
              case MEASURE_MINIMAL_ENERGY_LOWER_SCALE_INCREASE:
              case MEASURE_MINIMAL_ENERGY_LOWER_SCALE_DECREASE:
              case MEASURE_MINIMAL_ENERGY_HIGHER_SCALE_INCREASE:
              case MEASURE_MINIMAL_ENERGY_HIGHER_SCALE_DECREASE: 
              case MEASURE_MINIMAL_ENERGY_LOW_ENERGY:
                measureAvgP2P = 0;
                MEASURE_FSM_STATE_CHANGE(MEASURE_MINIMAL_ENERGY_SEARCH, 1);
                break;
              case MEASURE_MINIMAL_ENERGY_SEARCH_DONE:    
                energyMargin = (uint8_t)round(CurrrentPskLevel * ((double)configConfigurationDb.MeasurementMinimalPslMarginPercentage / 100));
                if(energyMargin > 0)
                {
                  sprintf(PrintBuff, "Add PSL margin = %d", energyMargin);
                  vlapmainDebugLog(PrintBuff);
                  measureIncreaseAndSetPsl(energyMargin);
                }
                
                measurePslStep = (uint8_t)round(CurrrentPskLevel * ((double)configConfigurationDb.PslSequenceStepPercentage / 100));
                if(measurePslStep < 1)
                {
                  measurePslStep =  1;
                }
                MEASURE_FSM_STATE_CHANGE(MEASURE_MINIMAL_ENERGY_WAIT, 200);
                break;
              case MEASURE_MINIMAL_ENERGY_FAILED:
                MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_STATE_TIMEOUT;
                measureIdleStateChange(10, 1);
                break;
              }
            }
           
            break;
          }
          
          break;
                  
          //      MEASURE_MINIMAL_ENERGY_WAIT:
        case MEASURE_MINIMAL_ENERGY_WAIT:
          // Stop the autoresonance
          if(measurementConditionCheck())
          {
            measureIdleStateChange(10, 0);
            break;
          }
          switch(QueueEntry.MeasureRequest)
          {
          case MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_SAMPLE:
        	  break;
          case MEASURE_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:
        	{
				if(vmicmodemCrcCounterGet())
				{
				  if(measureIncreaseAndSetPsl(measurePslStep) == RETURNCODE_OK)
				  {
					sprintf(PrintBuff, "Minimal Energy retry  PSL  = %d",   CurrrentPskLevel);
					vlapmainDebugLog(PrintBuff);
					MEASURE_FSM_STATE_CHANGE(MEASURE_MINIMAL_ENERGY_WAIT, measureModemLockDelay);
				  }
				  else
				  {
					measureIdleStateChange(10, 1);
				  }
				}
				else
				{
				  vlapmainDebugLog("Select Capacitor Source");
				  measureVmicSourceSelectMonitorFsmInit(MEASURE_CHANNEL_SELECT_AVRG_INDEX_IC_127, MEASSLCT_INTERNAL_REFERENCE_CAPACITOR, 127, 0.05);
				  vmicmodemSelect(MEASSLCT_INTERNAL_REFERENCE_CAPACITOR,127);
				  measureStabilityFsmInit();
				  measureSourceSelectTimeoutRetry = MEASURE_SELECT_MAX_RETRY;
				  MEASURE_FSM_STATE_CHANGE(MEASURE_MINIMAL_ENERGY_IC127_SOURCE_SELECT_SAMPLE_WAIT, MEASURE_NEW_SOURCE_SELECTION_STABILITY_TIME);
				}
        	}
            break;
          default:
            break; 
          }			
         break;
          
          //      MEASURE_MINIMAL_ENERGY_AUTO_RESONANCE_WAIT:
        case MEASURE_MINIMAL_ENERGY_AUTO_RESONANCE_WAIT:
          if(measurementConditionCheck())
          {
            switch(QueueEntry.MeasureRequest)
            {
            case MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_SAMPLE:
              break;
            case MEASURE_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:
              MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_STATE_TIMEOUT;
              measureIdleStateChange(10, 1);
              break;
            default:
              measureIdleStateChange(10, 1);
              break;
            }
          }
          // TODO: When auto resonance will be implemented
          break;
          
          
        case MEASURE_MINIMAL_ENERGY_IC127_SOURCE_SELECT_SAMPLE_WAIT:
          if(measurementConditionCheck())
          {
            measureIdleStateChange(10, 1);
            break;
          }
          switch(QueueEntry.MeasureRequest)
          {
          case MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_SAMPLE:
            if (vmicmodemBeltMasking)
              break;
            switch(measureVmicSourceSelectMonitorFsm(vmicapplayerUnFilteredSamplesGet(), MEASURE_VMIC_SOURCE_SELECT_STABILITY_COUNT))
            {
            case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_IDLE:
            case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_WAIT:
              break;
            case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_FAIL:
              measureIdleStateChange(10, 1);
              break;
            case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_MEASURE:
              MEASURE_FSM_STATE_CHANGE(MEASURE_MINIMAL_ENERGY_IC127_SOURCE_SELECT_COMPLETED_WAIT, 1);
              break;
            }
            break;        
          case MEASURE_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:
            if(measureSourceSelectTimeoutRetry)
            {
              measureSourceSelectTimeoutRetry--;
              if (vmicmodemBeltMasking)
              {
                vlapmainDebugLog("Select Capacitor Source timeout, retry...");
                measureVmicSourceSelectMonitorFsmInit(MEASURE_CHANNEL_SELECT_AVRG_INDEX_IC_127, MEASSLCT_INTERNAL_REFERENCE_CAPACITOR, 127, 0.05);
                vmicmodemSelect(MEASSLCT_INTERNAL_REFERENCE_CAPACITOR,127);
                vmicmodemBeltMasking = false;
                break;
              }
          	  // If there is not enough energy to receive samples - try to increase psl and switch channel
          	  // from here instead of from measureVmicSourceSelectMonitorFsm
              if((measureVmicSourceSelectMonitorFsmDb.RetryCounter == 0 && measureVmicSourceSelectMonitorFsmDb.SampleCounter == 0) || vmicmodemCrcCounterGet())
              {
                  uint8_t addPsl = (uint8_t)round(CurrrentPskLevel * 0.05);
                  if(addPsl == 0 || CurrrentPskLevel > 85)
                  {
                    addPsl = 1;
                  }
                  // Increase by mid level PSL
                  if(measureIncreaseAndSetPsl(addPsl) != RETURNCODE_OK)
                  {
                    measureIdleStateChange(10, 1);
                    break;
                  }
                  vlapmainDebugLog("Select Capacitor Source timeout, retry...");
                  measureVmicSourceSelectMonitorFsmInit(MEASURE_CHANNEL_SELECT_AVRG_INDEX_IC_127, MEASSLCT_INTERNAL_REFERENCE_CAPACITOR, 127, 0.05);
                  vmicmodemSelect(MEASSLCT_INTERNAL_REFERENCE_CAPACITOR,127);
              }
              MEASURE_FSM_STATE_CHANGE(MEASURE_MINIMAL_ENERGY_IC127_SOURCE_SELECT_SAMPLE_WAIT, MEASURE_NEW_SOURCE_SELECTION_STABILITY_TIME);
              break;
            }
            MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_STATE_TIMEOUT;
            measureIdleStateChange(10, 1);
            break;
          default:
            measureIdleStateChange(10, 1);
            break;
          }
          break;
          
          //      MEASURE_MINIMAL_ENERGY_IC127_SOURCE_SELECT_COMPLETED_WAIT:
        case MEASURE_MINIMAL_ENERGY_IC127_SOURCE_SELECT_COMPLETED_WAIT:
          if(measurementConditionCheck())
          {
            measureIdleStateChange(10, 1);
            break;
          }
          switch(QueueEntry.MeasureRequest)
          {
          case MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_SAMPLE:
            break;
          default:
            switch( measureStabilityFsm())
            {
            case MEASURE_STABILITY_FSM_RETURNED_STATUS_IDLE:
            case MEASURE_STABILITY_FSM_RETURNED_STATUS_INPROGRESS:
              MEASURE_FSM_STATE_CHANGE(MEASURE_MINIMAL_ENERGY_IC127_SOURCE_SELECT_COMPLETED_WAIT, StabilityIterationTimeGet());
              break;
            case MEASURE_STABILITY_FSM_RETURNED_STATUS_STABLE:
              vmicmodemSelect(MEASSLCT_TEMPERATURE, 127);
              vlapmainDebugLog("Select Temperature Source");

              if (CurrrentPskLevel < 75)
              {
                  // Add additional margin before trying to switch to temp channel in the first time
                  uint8_t tempMargin = (uint8_t)floor(CurrrentPskLevel * ((double) 0.3));
                  if (tempMargin == 0)
                	  tempMargin = 1;
                  else if (CurrrentPskLevel > 65) // Limit margin to 5 if current psl is higher than 65 to avoid noise
                	  tempMargin = 5;
                  else if (tempMargin > 10)
                	  tempMargin = 10; // Limit margin to 10
                  if(measureIncreaseAndSetPsl(tempMargin) == RETURNCODE_OK)
                  {
                    sprintf(PrintBuff, "Add PSL temperature margin = %d", tempMargin);
                    vlapmainDebugLog(PrintBuff);
                  }
              }

              measureStabilityFsmInit();
              measureVmicSourceSelectMonitorFsmInit(MEASURE_CHANNEL_SELECT_AVRG_INDEX_TEMP_127, MEASSLCT_TEMPERATURE, 127, 0.1);
              measureSourceSelectTimeoutRetry = MEASURE_SELECT_MAX_RETRY;
              MEASURE_FSM_STATE_CHANGE(MEASURE_MINIMAL_ENERGY_T127_SOURCE_SELECT_SAMPLE_WAIT, 7000);
              break;
            case MEASURE_STABILITY_FSM_RETURNED_STATUS_FAILED:
              pccpmmAppLayerStruct.BoardSystemRegisters.OnBoardPushButton = 0;
              // Session failed, Send indications 
              measureIdleStateChange(2000, 1);
              break;
            default:
              pccpmmAppLayerStruct.BoardSystemRegisters.OnBoardPushButton = 0;
              // Session failed, Send indications 
              measureIdleStateChange(2000, 1);
              break;
            }
            break;
          }
          break;
          
          
        case MEASURE_MINIMAL_ENERGY_T127_SOURCE_SELECT_SAMPLE_WAIT:
          if(measurementConditionCheck())
          {
            measureIdleStateChange(10, 1);
            break;
          }
          switch(QueueEntry.MeasureRequest)
          {
          case MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_SAMPLE:
            if (vmicmodemBeltMasking)
              break;
            switch(measureVmicSourceSelectMonitorFsm(vmicapplayerUnFilteredSamplesGet(), MEASURE_VMIC_SOURCE_SELECT_STABILITY_COUNT))
            {
            case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_IDLE:
            case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_WAIT:
              break;
            case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_FAIL:
              measureIdleStateChange(10, 1);
              break;
            case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_MEASURE:
              MEASURE_FSM_STATE_CHANGE(MEASURE_MINIMAL_ENERGY_T127_SOURCE_SELECT_COMPLETED_WAIT, 1);
              break;
            }
            break;        
          case MEASURE_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:
            if(measureSourceSelectTimeoutRetry)
            {
              measureSourceSelectTimeoutRetry--;
              if (vmicmodemBeltMasking)
              {
                vlapmainDebugLog("Select Capacitor Source timeout, retry...");
                measureVmicSourceSelectMonitorFsmInit(MEASURE_CHANNEL_SELECT_AVRG_INDEX_IC_127, MEASSLCT_INTERNAL_REFERENCE_CAPACITOR, 127, 0.05);
                vmicmodemSelect(MEASSLCT_INTERNAL_REFERENCE_CAPACITOR,127);
                vmicmodemBeltMasking = false;
                break;
              }
          	  // If there is not enough energy to receive samples - try to increase psl and switch channel
          	  // from here instead of from measureVmicSourceSelectMonitorFsm
              if (measureVmicSourceSelectMonitorFsmDb.RetryCounter == 0 && measureVmicSourceSelectMonitorFsmDb.SampleCounter == 0 || vmicmodemCrcCounterGet())
              {
                  uint8_t addPsl = (uint8_t)round(CurrrentPskLevel * 0.1);
                  if(addPsl < 2 || CurrrentPskLevel > 85)
                  {
                    addPsl = 1;
                  }
                  // Increase by mid level PSL
                  if(measureIncreaseAndSetPsl(addPsl) != RETURNCODE_OK)
                  {
                    measureIdleStateChange(10, 1);
                    break;
                  }
                  vlapmainDebugLog("Select Temperature Source timeout, retry...");
                  vmicmodemSelect(MEASSLCT_TEMPERATURE, 127);
                  measureVmicSourceSelectMonitorFsmInit(MEASURE_CHANNEL_SELECT_AVRG_INDEX_TEMP_127, MEASSLCT_TEMPERATURE, 127, 0.05);
              }
              MEASURE_FSM_STATE_CHANGE(MEASURE_MINIMAL_ENERGY_T127_SOURCE_SELECT_SAMPLE_WAIT, 7000);
              break;
            }
            MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_STATE_TIMEOUT;
            measureIdleStateChange(10, 1);
            break;
          default:
            measureIdleStateChange(10, 1);
            break;
          }
          break;
          
          
          
          //      MEASURE_MINIMAL_ENERGY_T127_SOURCE_SELECT_COMPLETED_WAIT:
        case MEASURE_MINIMAL_ENERGY_T127_SOURCE_SELECT_COMPLETED_WAIT:
          if(measurementConditionCheck())
          {
            measureIdleStateChange(10, 1);
            break;
          }
          switch(QueueEntry.MeasureRequest)
          {
          case MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_SAMPLE:
            break;
          default:
            switch( measureStabilityFsm())
            {
            case MEASURE_STABILITY_FSM_RETURNED_STATUS_IDLE:
            case MEASURE_STABILITY_FSM_RETURNED_STATUS_INPROGRESS:
              MEASURE_FSM_STATE_CHANGE(MEASURE_MINIMAL_ENERGY_T127_SOURCE_SELECT_COMPLETED_WAIT, StabilityIterationTimeGet());
              break;
            case MEASURE_STABILITY_FSM_RETURNED_STATUS_STABLE:
              energyMargin = (uint8_t)round(CurrrentPskLevel * ((double)configConfigurationDb.MeasurementMarginPercentage / 100));
              if(energyMargin > 0)
              {
                sprintf(PrintBuff, "Add PSL margin = %d", energyMargin);
                vlapmainDebugLog(PrintBuff);
                measureIncreaseAndSetPsl(energyMargin);
              }
              vmicmodemSelect(MEASSLCT_ID_LSB, 127);
              vlapmainDebugLog("Select ID LSB Source");
              measureSourceSelectTimeoutRetry = MEASURE_SELECT_MAX_RETRY;
              measureIDReadIncreasment = 0;
              MEASURE_FSM_STATE_CHANGE(MEASURE_VMIC_ID_LSB_WAIT, MEASURE_ID_SOURCE_SELECTION_TIME);
              break;
            case MEASURE_STABILITY_FSM_RETURNED_STATUS_FAILED:
              pccpmmAppLayerStruct.BoardSystemRegisters.OnBoardPushButton = 0;
              // Session failed, Send indications 
              measureIdleStateChange(2000, 1);
              break;
            default:
              pccpmmAppLayerStruct.BoardSystemRegisters.OnBoardPushButton = 0;
              // Session failed, Send indications 
              measureIdleStateChange(2000, 1);
              break;
            }
            break;
          }
          break;
          
          //    MEASURE_VMIC_ID_LSB_WAIT:
        case 	MEASURE_VMIC_ID_LSB_WAIT:
          if(measurementConditionCheck() )
          {
            measureIdleStateChange(10, 1);
            break;
          }
          switch(QueueEntry.MeasureRequest)
          {
          case MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_SAMPLE:
            // We need to receive status source 0 and not a sample
            break;
          case MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_STATUS_SOURCE_0:
            if (vmicmodemBeltMasking)
              break;
            measureSourceSelectTimeoutRetry = MEASURE_SELECT_MAX_RETRY;
            // Skip MSB as we agreed that we don't need it for now
//            vmicmodemSelect(MEASSLCT_ID_MSB, 127);
//            vlapmainDebugLog("Select ID MSB Source");
//            MEASURE_FSM_STATE_CHANGE(MEASURE_VMIC_ID_MSB_WAIT, MEASURE_ID_SOURCE_SELECTION_TIME);
            
            
            VmicId = vmicapplayerIdGet();
            sprintf(PrintBuff, "VMIC ID %d,   MEASURE_VMIC_INTERNAL_CAP_1_MEASURE", VmicId);
            vlapmainDebugLog(PrintBuff);
            pccpmmAppLayerStruct.VmicRegisters3.UniqueIdSerialNumber = TYPES_ENDIAN16_CHANGE(VmicId);
            if(!VmicId)
            {
              // VMIC Id is Zero, cancel the measurement and indicate wrong measurement
              MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_IMPLANT_ID_ZERO;
              measureIdleStateChange(10, 1);
            }
            else
            {
              // Decrease PSL back to be as before we started the LSB state
              measureDecreaseAndSetPsl(measureIDReadIncreasment);
              measureIDReadIncreasment = 0;
              vmicmodemSelect(MEASSLCT_INTERNAL_REFERENCE_CAPACITOR, 63);
              measureVmicSourceSelectMonitorFsmInit(MEASURE_CHANNEL_SELECT_AVRG_INDEX_IC_63, MEASSLCT_INTERNAL_REFERENCE_CAPACITOR, 63, 0);
              vlapmainDebugLog("IC1 63");
              MeasurementIndex = 0;
              measureSourceSelectTimeoutRetry = MEASURE_SELECT_MAX_RETRY;
              MEASURE_FSM_STATE_CHANGE(MEASURE_VMIC_INTERNAL_CAP_1_MEASURE_WAIT, MEASURE_NEW_SOURCE_SELECTION_TIME);
            }
            break;
          case MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_STATUS_SOURCE_1:
            break;
          case MEASURE_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:
            if(measureSourceSelectTimeoutRetry)
            {
              uint8_t increasePsl = (uint8_t)round(CurrrentPskLevel * 0.05);
              if(increasePsl == 0 || CurrrentPskLevel > 95)
              {
                increasePsl = 1;
              }
              measureIDReadIncreasment += increasePsl;
              
              // Increase by low level PSL
              if(vmicmodemBeltMasking && measureIncreaseAndSetPsl(increasePsl) != RETURNCODE_OK)
              {
                measureIdleStateChange(10, 1);
                break;
              }
              
              measureSourceSelectTimeoutRetry--;
              vmicmodemSelect(MEASSLCT_ID_LSB, 127);
              vlapmainDebugLog("Select ID LSB Source timeout! retrying...");
              MEASURE_FSM_STATE_CHANGE(MEASURE_VMIC_ID_LSB_WAIT, MEASURE_ID_SOURCE_SELECTION_TIME);
              vmicmodemBeltMasking = false;
              break;
            }
            
            MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_STATE_TIMEOUT;
            measureIdleStateChange(10, 1);
            break;
          default:
            measureIdleStateChange(10, 1);
            break;
          }
          break;
          
          //    MEASURE_VMIC_ID_MSB_WAIT:
        case 	MEASURE_VMIC_ID_MSB_WAIT:
          if(measurementConditionCheck())
          {
            measureIdleStateChange(10, 1);
            break;
          }
          switch(QueueEntry.MeasureRequest)
          {
          case MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_SAMPLE:
            // We need to receive status source 1 and not a sample
            break;
          case MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_STATUS_SOURCE_0:
            break;
          case MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_STATUS_SOURCE_1:
            if (vmicmodemBeltMasking)
              break;
            VmicId = vmicapplayerIdGet();
            sprintf(PrintBuff, "VMIC ID %d,   MEASURE_VMIC_INTERNAL_CAP_1_MEASURE", VmicId);
            vlapmainDebugLog(PrintBuff);
            pccpmmAppLayerStruct.VmicRegisters3.UniqueIdSerialNumber = TYPES_ENDIAN16_CHANGE(VmicId);
            if(!VmicId)
            {
              // VMIC Id is Zero, cancel the measurement and indicate wrong measurement
              MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_IMPLANT_ID_ZERO;
              measureIdleStateChange(10, 1);
            }
            else
            {
              // Decrease PSL back to be as before we started the LSB state
              measureDecreaseAndSetPsl(measureIDReadIncreasment);
              measureIDReadIncreasment = 0;
              vmicmodemSelect(MEASSLCT_INTERNAL_REFERENCE_CAPACITOR, 63);
              measureVmicSourceSelectMonitorFsmInit(MEASURE_CHANNEL_SELECT_AVRG_INDEX_IC_63, MEASSLCT_INTERNAL_REFERENCE_CAPACITOR, 63, 0);
              vlapmainDebugLog("IC1 63");
              MeasurementIndex = 0;
              measureSourceSelectTimeoutRetry = MEASURE_SELECT_MAX_RETRY;
              MEASURE_FSM_STATE_CHANGE(MEASURE_VMIC_INTERNAL_CAP_1_MEASURE_WAIT, MEASURE_NEW_SOURCE_SELECTION_TIME);
            }
            break;
          case MEASURE_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:            
            if(measureSourceSelectTimeoutRetry)
            {
              uint8_t increasePsl = (uint8_t)round(CurrrentPskLevel * 0.05);
              if(increasePsl == 0 || CurrrentPskLevel > 95)
              {
                increasePsl = 1;
              }
              measureIDReadIncreasment += increasePsl;
              
              // Increase by low level PSL
              if(vmicmodemBeltMasking && measureIncreaseAndSetPsl(increasePsl) != RETURNCODE_OK)
              {
                measureIdleStateChange(10, 1);
                break;
              }
                 
              measureSourceSelectTimeoutRetry--;
              vmicmodemSelect(MEASSLCT_ID_MSB, 127);
              vlapmainDebugLog("Select ID MSB Source timeout! retrying...");
              MEASURE_FSM_STATE_CHANGE(MEASURE_VMIC_ID_MSB_WAIT, MEASURE_ID_SOURCE_SELECTION_TIME);
              vmicmodemBeltMasking = false;
              break;
            }
            
            MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_STATE_TIMEOUT;
            measureIdleStateChange(10, 1);
            break;
          default:
            measureIdleStateChange(10, 1);
            break;
          }
          break;
          
          
          //  	MEASURE_VMIC_INTERNAL_CAP_1_MEASURE_WAIT:
        case 	MEASURE_VMIC_INTERNAL_CAP_1_MEASURE_WAIT:
          if(measurementConditionCheck())
          {
            measureIdleStateChange(10, 1);
            break;
          }
          switch(QueueEntry.MeasureRequest)
          {
          case MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_SAMPLE:
            if (vmicmodemBeltMasking)
              break;
            switch(measureVmicSourceSelectMonitorFsm(vmicapplayerUnFilteredSamplesGet(), MEASURE_VMIC_SOURCE_SELECT_SAMPLE_COUNT))
            {
            case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_IDLE:
            case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_WAIT:
              break;
            case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_FAIL:
              measureIdleStateChange(10, 1);
              break;
            case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_MEASURE:
              if(MeasurementIndex < measureNumberOfSamples)
              {
                MeasurementsResults.Ic1[MeasurementIndex].MeasureStruct.vmicPressure = vmicapplayerUnFilteredSamplesGet();
                // Ge tx monitor
                hwdriversTxMonitorGet(&txMonitor);
                // Get current.
                MyTransCurrentMonitor = autopowerBeltCurrentMilliAmpGet();
                MeasurementsResults.Ic1[MeasurementIndex].MeasureStruct.TxCurrent = MyTransCurrentMonitor;
                MeasurementsResults.Ic1[MeasurementIndex].MeasureWord = HTONL(MeasurementsResults.Ic1[MeasurementIndex].MeasureWord);
                MeasurementIndex++;
                //              MEASURE_FSM_STATE_CHANGE(MEASURE_VMIC_INTERNAL_CAP_1_MEASURE_WAIT, 10);
              }
              else
              {
                measureVmicSourceSelectMonitorFsmInit(MEASURE_CHANNEL_SELECT_AVRG_INDEX_IC_87, MEASSLCT_INTERNAL_REFERENCE_CAPACITOR, 87, 0);
                vlapmainDebugLog("IC2 87");
                MeasurementIndex = 0;
                vmicmodemSelect(MEASSLCT_INTERNAL_REFERENCE_CAPACITOR, 87);
                measureSourceSelectTimeoutRetry = MEASURE_SELECT_MAX_RETRY;
                MEASURE_FSM_STATE_CHANGE(MEASURE_VMIC_INTERNAL_CAP_2_MEASURE_WAIT, MEASURE_NEW_SOURCE_SELECTION_TIME);
              }
            }
            break;
          case MEASURE_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:
            // Check if not a single measurement sample recevied and we can perform retry
            if(measureSourceSelectTimeoutRetry)
            {
              uint8_t addPsl = (uint8_t)round(CurrrentPskLevel * 0.01);
              if(addPsl == 0)
              {
                addPsl = 1;
              }
              // Increase by mid level PSL
              if(vmicmodemBeltMasking && measureIncreaseAndSetPsl(1) != RETURNCODE_OK)
              {
                //measureIdleStateChange(10, 1);
                //break;
              }
              //measureDecreaseAndSetPsl(addPsl);
              
              MeasurementIndex = 0;
              measureSourceSelectTimeoutRetry--;
              vlapmainDebugLog("Select IC1 63 timeout, retry...");
              vmicmodemSelect(MEASSLCT_INTERNAL_REFERENCE_CAPACITOR, 63);
              measureVmicSourceSelectMonitorFsmInit(MEASURE_CHANNEL_SELECT_AVRG_INDEX_IC_63, MEASSLCT_INTERNAL_REFERENCE_CAPACITOR, 63, 0);
              MEASURE_FSM_STATE_CHANGE(MEASURE_VMIC_INTERNAL_CAP_1_MEASURE_WAIT, MEASURE_NEW_SOURCE_SELECTION_TIME);           
              vmicmodemBeltMasking = false;
              break;
          default:
        	  break;
            }
            
            MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_STATE_TIMEOUT;
            measureIdleStateChange(10, 1);
            break;
          }       
          break;
            
            // 	MEASURE_VMIC_INTERNAL_CAP_2_MEASURE_WAIT:
          case 	MEASURE_VMIC_INTERNAL_CAP_2_MEASURE_WAIT:
            if(measurementConditionCheck())
            {
              measureIdleStateChange(10, 1);
              break;
            }
            switch(QueueEntry.MeasureRequest)
            {
            case MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_SAMPLE:
              if (vmicmodemBeltMasking)
                break;
              switch(measureVmicSourceSelectMonitorFsm(vmicapplayerUnFilteredSamplesGet(), MEASURE_VMIC_SOURCE_SELECT_SAMPLE_COUNT))
              {
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_IDLE:
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_WAIT:
                break;
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_FAIL:
                measureIdleStateChange(10, 1);
                break;
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_MEASURE:
                if(MeasurementIndex < measureNumberOfSamples)
                {
                  MeasurementsResults.Ic2[MeasurementIndex].MeasureStruct.vmicPressure = vmicapplayerUnFilteredSamplesGet();
                  // Ge tx monitor
                  hwdriversTxMonitorGet(&txMonitor);
                  // Get current.
                  MyTransCurrentMonitor = autopowerBeltCurrentMilliAmpGet();
                  MeasurementsResults.Ic2[MeasurementIndex].MeasureStruct.TxCurrent = MyTransCurrentMonitor;
                  MeasurementsResults.Ic2[MeasurementIndex].MeasureWord = HTONL(MeasurementsResults.Ic2[MeasurementIndex].MeasureWord);
                  MeasurementIndex++;
                  //              MEASURE_FSM_STATE_CHANGE(MEASURE_VMIC_INTERNAL_CAP_2_MEASURE_WAIT, 10);
                }
                else
                {
                  measureVmicSourceSelectMonitorFsmInit(MEASURE_CHANNEL_SELECT_AVRG_INDEX_IC_127, MEASSLCT_INTERNAL_REFERENCE_CAPACITOR, 127, 0);
                  vlapmainDebugLog("IC3 127");
                  MeasurementIndex = 0;
                  vmicmodemSelect(MEASSLCT_INTERNAL_REFERENCE_CAPACITOR, 127);
                  measureSourceSelectTimeoutRetry = MEASURE_SELECT_MAX_RETRY;
                  MEASURE_FSM_STATE_CHANGE(MEASURE_VMIC_INTERNAL_CAP_3_MEASURE_WAIT, MEASURE_NEW_SOURCE_SELECTION_TIME);
                }
              }
              break;
            case MEASURE_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:
              // Check if not a single measurement sample recevied and we can perform retry
              if(measureSourceSelectTimeoutRetry)
              {
                uint8_t addPsl = (uint8_t)round(CurrrentPskLevel * 0.01);
                if(addPsl == 0)
                {
                  addPsl = 1;
                }
                // Increase by mid level PSL
                if(vmicmodemBeltMasking && measureIncreaseAndSetPsl(1) != RETURNCODE_OK)
                {
                  //measureIdleStateChange(10, 1);
                  //break;
                }
                //measureDecreaseAndSetPsl(addPsl);
                
                MeasurementIndex = 0;
                measureSourceSelectTimeoutRetry--;
                vlapmainDebugLog("Select IC2 87 timeout, retry...");
                vmicmodemSelect(MEASSLCT_INTERNAL_REFERENCE_CAPACITOR, 87);
                measureVmicSourceSelectMonitorFsmInit(MEASURE_CHANNEL_SELECT_AVRG_INDEX_IC_87, MEASSLCT_INTERNAL_REFERENCE_CAPACITOR, 87, 0);
                MEASURE_FSM_STATE_CHANGE(MEASURE_VMIC_INTERNAL_CAP_2_MEASURE_WAIT, MEASURE_NEW_SOURCE_SELECTION_TIME);
                vmicmodemBeltMasking = false;
                break;
              }
              
              MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_STATE_TIMEOUT;
              measureIdleStateChange(10, 1);
              break;
            default:
            	break;
            }
            break;
            
            //    MEASURE_VMIC_INTERNAL_CAP_3_MEASURE_WAIT:
          case 	MEASURE_VMIC_INTERNAL_CAP_3_MEASURE_WAIT:
            if(measurementConditionCheck())
            {
              measureIdleStateChange(10, 1);
              break;
            }
            switch(QueueEntry.MeasureRequest)
            {
            case MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_SAMPLE:
              if (vmicmodemBeltMasking)
                break;
              switch(measureVmicSourceSelectMonitorFsm(vmicapplayerUnFilteredSamplesGet(), MEASURE_VMIC_SOURCE_SELECT_SAMPLE_COUNT))
              {
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_IDLE:
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_WAIT:
                break;
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_FAIL:
                measureIdleStateChange(10, 1);
                break;
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_MEASURE:
                if(MeasurementIndex < measureNumberOfSamples)
                {
                  MeasurementsResults.Ic3[MeasurementIndex].MeasureStruct.vmicPressure = vmicapplayerUnFilteredSamplesGet();
                  // Ge tx monitor
                  hwdriversTxMonitorGet(&txMonitor);
                  // Get current.
                  MyTransCurrentMonitor = autopowerBeltCurrentMilliAmpGet();
                  MeasurementsResults.Ic3[MeasurementIndex].MeasureStruct.TxCurrent = MyTransCurrentMonitor;
                  MeasurementsResults.Ic3[MeasurementIndex].MeasureWord = HTONL(MeasurementsResults.Ic3[MeasurementIndex].MeasureWord);
                  MeasurementIndex++;
                  //              MEASURE_FSM_STATE_CHANGE(MEASURE_VMIC_INTERNAL_CAP_3_MEASURE_WAIT, 10);
                }
                else
                {
                  measureVmicSourceSelectMonitorFsmInit(MEASURE_CHANNEL_SELECT_AVRG_INDEX_REF_127, MEASSLCT_REF_MEMS_WITHOUT_NOISE_COMPENSATION, 127, 0);
                  vlapmainDebugLog("MEASURE_REFERENCE_1_WAIT");
                  MeasurementIndex = 0;
                  vmicmodemSelect(MEASSLCT_REF_MEMS_WITHOUT_NOISE_COMPENSATION, 127);
                  measureSourceSelectTimeoutRetry = MEASURE_SELECT_MAX_RETRY;
                  MEASURE_FSM_STATE_CHANGE(MEASURE_REFERENCE_1_WAIT, MEASURE_NEW_SOURCE_SELECTION_TIME);
                }
                break;
              default:
            	  break;
              }
              break;
            case MEASURE_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:
              // Check if not a single measurement sample recevied and we can perform retry
              if(measureSourceSelectTimeoutRetry)
              {    
                uint8_t addPsl = (uint8_t)round(CurrrentPskLevel * 0.01);
                if(addPsl == 0)
                {
                  addPsl = 1;
                }
                // Increase by low level PSL
                if(vmicmodemBeltMasking && measureIncreaseAndSetPsl(1) != RETURNCODE_OK)
                {
                  //measureIdleStateChange(10, 1);
                  //break;
                }
                //measureDecreaseAndSetPsl(addPsl);
                
                MeasurementIndex = 0;
                measureSourceSelectTimeoutRetry--;
                vlapmainDebugLog("Select IC3 127 timeout, retry...");
                vmicmodemSelect(MEASSLCT_INTERNAL_REFERENCE_CAPACITOR, 127);
                measureVmicSourceSelectMonitorFsmInit(MEASURE_CHANNEL_SELECT_AVRG_INDEX_IC_127, MEASSLCT_INTERNAL_REFERENCE_CAPACITOR, 127, 0);
                MEASURE_FSM_STATE_CHANGE(MEASURE_VMIC_INTERNAL_CAP_3_MEASURE_WAIT, MEASURE_NEW_SOURCE_SELECTION_TIME);
                vmicmodemBeltMasking = false;
                break;
              }
              
              MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_STATE_TIMEOUT;
              measureIdleStateChange(10, 1);
              break;
            default:
            	break;
            }
            break;
            
            //  	MEASURE_REFERENCE_1_WAIT:
          case 	MEASURE_REFERENCE_1_WAIT:
            if(measurementConditionCheck())
            {
              measureIdleStateChange(10, 1);
              break;
            }
            switch(QueueEntry.MeasureRequest)
            {
            case MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_SAMPLE:
              if (vmicmodemBeltMasking)
                break;
              switch(measureVmicSourceSelectMonitorFsm(vmicapplayerUnFilteredSamplesGet(), MEASURE_VMIC_SOURCE_SELECT_SAMPLE_COUNT))
              {
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_IDLE:
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_WAIT:
                break;
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_FAIL:
                measureIdleStateChange(10, 1);
                break;
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_MEASURE:
                if(MeasurementIndex < measureNumberOfSamples)
                {
                  MeasurementsResults.Ref1[MeasurementIndex].MeasureStruct.vmicPressure = vmicapplayerUnFilteredSamplesGet();
                  // Ge tx monitor
                  hwdriversTxMonitorGet(&txMonitor);
                  // Get current.
                  MyTransCurrentMonitor = autopowerBeltCurrentMilliAmpGet();
                  MeasurementsResults.Ref1[MeasurementIndex].MeasureStruct.TxCurrent = NTOHS(MyTransCurrentMonitor);
                  MeasurementsResults.Ref1[MeasurementIndex].MeasureWord = HTONL(MeasurementsResults.Ref1[MeasurementIndex].MeasureWord);
                  MeasurementIndex++;
                  //              MEASURE_FSM_STATE_CHANGE(MEASURE_REFERENCE_1_WAIT, 10);
                }
                else
                {
                  measureVmicSourceSelectMonitorFsmInit(MEASURE_CHANNEL_SELECT_AVRG_INDEX_TEMP_127, MEASSLCT_TEMPERATURE, 127, 0);
                  measureStabilityFsmInit();
                  vlapmainDebugLog("MEASURE_TEMPERATURE_1_MEASUREMENT_WAIT");
                  MeasurementIndex = 0;
                  vmicmodemSelect(MEASSLCT_TEMPERATURE, 127);
                  measureSourceSelectTimeoutRetry = MEASURE_SELECT_MAX_RETRY;
                  MEASURE_FSM_STATE_CHANGE(MEASURE_TEMPERATURE_1_MEASUREMENT_WAIT, MEASURE_NEW_SOURCE_SELECTION_TIME);
                }
                break;
              }
              break;
            case MEASURE_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:
              // Check if not a single measurement sample recevied and we can perform retry
              if(measureSourceSelectTimeoutRetry)
              {
                uint8_t addPsl = (uint8_t)round(CurrrentPskLevel * 0.01);
                if(addPsl == 0)
                {
                  addPsl = 1;
                }
                // Increase by low level PSL
                if(vmicmodemBeltMasking && measureIncreaseAndSetPsl(1) != RETURNCODE_OK)
                {
                  //measureIdleStateChange(10, 1);
                  //break;
                }
                //measureDecreaseAndSetPsl(addPsl);
                
                MeasurementIndex = 0;
                measureSourceSelectTimeoutRetry--;
                vlapmainDebugLog("Select MEASURE_REFERENCE_1 timeout, retry...");
                vmicmodemSelect(MEASSLCT_REF_MEMS_WITHOUT_NOISE_COMPENSATION, 127);
                measureVmicSourceSelectMonitorFsmInit(MEASURE_CHANNEL_SELECT_AVRG_INDEX_REF_127, MEASSLCT_REF_MEMS_WITHOUT_NOISE_COMPENSATION, 127, 0);
                MEASURE_FSM_STATE_CHANGE(MEASURE_REFERENCE_1_WAIT, MEASURE_NEW_SOURCE_SELECTION_TIME);
                vmicmodemBeltMasking = false;
                break;
              }
                 
              MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_STATE_TIMEOUT;
              measureIdleStateChange(10, 1);
              break;
            default:
            	break;
            }
            break;
            
            //      MEASURE_TEMPERATURE_1_MEASUREMENT_WAIT:
          case 	MEASURE_TEMPERATURE_1_MEASUREMENT_WAIT:
            if(measurementConditionCheck())
            {
              measureIdleStateChange(10, 1);
              break;
            }
            switch(QueueEntry.MeasureRequest)
            {
            case MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_SAMPLE:
              if (vmicmodemBeltMasking)
                break;
              switch(measureVmicSourceSelectMonitorFsm(vmicapplayerUnFilteredSamplesGet(), MEASURE_VMIC_SOURCE_SELECT_SAMPLE_COUNT))
              {
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_IDLE:
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_WAIT:
                break;
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_FAIL:
                measureIdleStateChange(10, 1);
                break;
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_MEASURE:
                if(MeasurementIndex < measureNumberOfSamples)
                {
                  MeasurementsResults.T1[MeasurementIndex].MeasureStruct.vmicPressure = vmicapplayerUnFilteredSamplesGet();
                  // Ge tx monitor
                  hwdriversTxMonitorGet(&txMonitor);
                  // Get current.
                  MyTransCurrentMonitor = autopowerBeltCurrentMilliAmpGet();
                  MeasurementsResults.T1[MeasurementIndex].MeasureStruct.TxCurrent = NTOHS(MyTransCurrentMonitor);
                  MeasurementsResults.T1[MeasurementIndex].MeasureWord = HTONL(MeasurementsResults.T1[MeasurementIndex].MeasureWord);
                  MeasurementIndex++;
                  //              MEASURE_FSM_STATE_CHANGE(MEASURE_TEMPERATURE_1_MEASUREMENT_WAIT, 10);
                }
                else
                {
                  vlapmainDebugLog("MEASURE_VMIC_REAL_MEMS, 1500 samples");
                  MeasurementIndex = 0;
                  vmicmodemSelect(MEASSLCT_REAL_MEMS_WITHOUT_NOISE_COMPENSATION, 127);
                  // Get accelerometer parameters for the pre real
//                  measureAccXPreReal = sensorAccGet(SENSOR_AXIS_X);
//                  measureAccYPreReal = sensorAccGet(SENSOR_AXIS_Y);
//                  measureAccZPreReal = sensorAccGet(SENSOR_AXIS_Z);
                  
                  measureVmicSourceSelectMonitorFsmInit(MEASURE_CHANNEL_SELECT_AVRG_INDEX_REALMEMS_127, MEASSLCT_REAL_MEMS_WITHOUT_NOISE_COMPENSATION, 127, 0);
                  measureSourceSelectTimeoutRetry = MEASURE_SELECT_MAX_RETRY;
                  measureRealMemsOutOfRangeFramesPerSample = 0;
                  //MEASURE_FSM_STATE_CHANGE(MEASURE_VMIC_REAL_MEMS_WAIT, 10*MEASURE_NEW_SOURCE_SELECTION_TIME);
                  MEASURE_FSM_STATE_CHANGE(MEASURE_VMIC_REAL_MEMS_WAIT, MEASURE_NEW_SOURCE_SELECTION_TIME);
                }
                break;
              }
              break;
            case MEASURE_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:
              // Check if not a single measurement sample recevied and we can perform retry
              if(measureSourceSelectTimeoutRetry)
              {
                uint8_t addPsl = (uint8_t)round(CurrrentPskLevel * 0.01);
                if(addPsl == 0 || CurrrentPskLevel > 95)
                {
                  addPsl = 1;
                }
                // Increase by low level PSL
                if(vmicmodemBeltMasking && measureIncreaseAndSetPsl(1) != RETURNCODE_OK)
                {
                  //measureIdleStateChange(10, 1);
                  //break;
                }
                //measureDecreaseAndSetPsl(addPsl);
                
                MeasurementIndex = 0;
                measureSourceSelectTimeoutRetry--;
                vlapmainDebugLog("Select MEASURE_TEMPERATURE_1 timeout, retry...");
                vmicmodemSelect(MEASSLCT_TEMPERATURE, 127);
                measureVmicSourceSelectMonitorFsmInit(MEASURE_CHANNEL_SELECT_AVRG_INDEX_TEMP_127, MEASSLCT_TEMPERATURE, 127, 0);
                MEASURE_FSM_STATE_CHANGE(MEASURE_TEMPERATURE_1_MEASUREMENT_WAIT, MEASURE_NEW_SOURCE_SELECTION_TIME);   
                vmicmodemBeltMasking = false;
                break;
              }
              
              MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_STATE_TIMEOUT;
              measureIdleStateChange(10, 1);
              break;
            default:
            	break;
            }
            break;               
            
            //      MEASURE_VMIC_REAL_MEMS_WAIT:
          case MEASURE_VMIC_REAL_MEMS_WAIT:
            if(measurementConditionCheck())
            {
              measureIdleStateChange(10, 1);
              break;
            }
            switch(QueueEntry.MeasureRequest)
            {
            case MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_SAMPLE:
              if (vmicmodemBeltMasking)
                break;
              switch(measureVmicSourceSelectMonitorFsm(vmicapplayerUnFilteredSamplesGet(), MEASURE_VMIC_SOURCE_SELECT_SAMPLE_COUNT))
              {
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_IDLE:
                break;
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_WAIT:
                // If channel reselected -> restart recording
                MeasurementIndex = 0;
                break;
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_FAIL:
                measureIdleStateChange(10, 1);
                break;
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_MEASURE:
                if(MeasurementIndex == 0)
                {
                  measureRealMemsBadFramesCounter = 0;
                  measureRealMemsBadFramesPerSample = 0;
                  measureRealMemsOutOfRangeFramesPerSample = measureVmicSourceSelectMonitorFsmDb.UnsavedSamples;
                  vmicapplayerInitializeBadFramesCounter();
                }
                if(MeasurementIndex < PROTOCOLAPP_REAL_MEMS_SIZE)
                {
                  uint32_t badCounterRead = vmicapplayerGetBadFramesCounter();
                  
                  // Ge tx monitor
                  hwdriversTxMonitorGet(&txMonitor);
                  // Get current.
                  MyTransCurrentMonitor = autopowerBeltCurrentMilliAmpGet();
                  //MeasurementsResults.RealMems[MeasurementIndex].MeasureStruct.TxCurrent = NTOHS(MyTransCurrentMonitor);
                  uint8_t outOfRangeLostFrames = measureVmicSourceSelectMonitorFsmDb.UnsavedSamples - measureRealMemsOutOfRangeFramesPerSample;
                  uint8_t crcLostFrames = badCounterRead - measureRealMemsBadFramesPerSample;
                  MeasurementsResults.RealMems[MeasurementIndex].MeasureStruct.TxCurrent =  outOfRangeLostFrames + crcLostFrames; 
                  MeasurementsResults.RealMems[MeasurementIndex].MeasureStruct.vmicPressure = vmicapplayerUnFilteredSamplesGet();
                  MeasurementsResults.RealMems[MeasurementIndex].MeasureWord = HTONL(MeasurementsResults.RealMems[MeasurementIndex].MeasureWord);
                  MeasurementIndex++;
                  
                  measureRealMemsBadFramesPerSample = badCounterRead;
                  measureRealMemsOutOfRangeFramesPerSample = measureVmicSourceSelectMonitorFsmDb.UnsavedSamples;
                  MEASURE_FSM_STATE_CHANGE(MEASURE_VMIC_REAL_MEMS_WAIT, 2000);
                  //              MEASURE_FSM_STATE_CHANGE(MEASURE_VMIC_REAL_MEMS_WAIT, 10);
                }
                else
                {
                  measureRealMemsBadFramesCounter = vmicapplayerGetBadFramesCounter();
                  measureVmicSourceSelectMonitorFsmInit(MEASURE_CHANNEL_SELECT_AVRG_INDEX_TEMP_127, MEASSLCT_TEMPERATURE, 127, 0);
                  vlapmainDebugLog("MEASSLCT_TEMPERATURE 2");
                  MeasurementIndex = 0;
                  vmicmodemSelect(MEASSLCT_TEMPERATURE, 127);
                  measureSourceSelectTimeoutRetry = MEASURE_SELECT_MAX_RETRY;
                  MEASURE_FSM_STATE_CHANGE(MEASURE_TEMPERATURE_2_MEASUREMENT_WAIT, MEASURE_NEW_SOURCE_SELECTION_TIME);
                }             
                break;
              }                 
              break;
            case MEASURE_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:
              // Check if not a single sample received in timeout period time
              // Check if not a single measurement sample recevied and we can perform retry
              if(measureSourceSelectTimeoutRetry)
              {
                uint8_t addPsl = (uint8_t)round(CurrrentPskLevel * 0.01);
                if(addPsl == 0 || CurrrentPskLevel > 95)
                {
                  addPsl = 1;
                }
                // Increase by low level PSL
                if(vmicmodemBeltMasking && measureIncreaseAndSetPsl(1) != RETURNCODE_OK)
                {
                  //measureIdleStateChange(10, 1);
                  //break;
                }
                //measureDecreaseAndSetPsl(addPsl);
                
                MeasurementIndex = 0;
                measureSourceSelectTimeoutRetry--;
                vlapmainDebugLog("Select MEASURE_VMIC_REAL_MEMS timeout, retry...");
                vmicmodemSelect(MEASSLCT_REAL_MEMS_WITHOUT_NOISE_COMPENSATION, 127);
                measureVmicSourceSelectMonitorFsmInit(MEASURE_CHANNEL_SELECT_AVRG_INDEX_REALMEMS_127, MEASSLCT_REAL_MEMS_WITHOUT_NOISE_COMPENSATION, 127, 0);
                MEASURE_FSM_STATE_CHANGE(MEASURE_VMIC_REAL_MEMS_WAIT, MEASURE_NEW_SOURCE_SELECTION_TIME); 
                vmicmodemBeltMasking = false;
                break;
              }       
              // Check if not a single measurement sample recevied and we can perform retry
              measureRealMemsBadFramesCounter = vmicapplayerGetBadFramesCounter();
              MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_STATE_TIMEOUT;
              measureIdleStateChange(10, 1);
              break;
            default:
              // TODO: Can't decide which failure reason to set here
              //MeasurementsResults.MeasurementFailureReason = 0;
              //measureIdleStateChange(10, 1);
              break;
            }
            break;
            
            //  	MEASURE_TEMPERATURE_2_MEASUREMENT_WAIT:
          case 	MEASURE_TEMPERATURE_2_MEASUREMENT_WAIT:
            if(measurementConditionCheck() )
            {
              measureIdleStateChange(10, 1);
              break;
            }
            switch(QueueEntry.MeasureRequest)
            {
            case MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_SAMPLE:
              if (vmicmodemBeltMasking)
                break;
              switch(measureVmicSourceSelectMonitorFsm(vmicapplayerUnFilteredSamplesGet(), MEASURE_VMIC_SOURCE_SELECT_SAMPLE_COUNT))
              {
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_IDLE:
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_WAIT:
                break;
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_FAIL:
                measureIdleStateChange(10, 1);
                break;
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_MEASURE:
                if(MeasurementIndex < measureNumberOfSamples)
                {
                  MeasurementsResults.T2[MeasurementIndex].MeasureStruct.vmicPressure = vmicapplayerUnFilteredSamplesGet();
                  // Ge tx monitor
                  hwdriversTxMonitorGet(&txMonitor);
                  // Get current.
                  MyTransCurrentMonitor = autopowerBeltCurrentMilliAmpGet();
                  MeasurementsResults.T2[MeasurementIndex].MeasureStruct.TxCurrent = NTOHS(MyTransCurrentMonitor);
                  MeasurementsResults.T2[MeasurementIndex].MeasureWord = HTONL(MeasurementsResults.T2[MeasurementIndex].MeasureWord);
                  MeasurementIndex++;
                  //              MEASURE_FSM_STATE_CHANGE(MEASURE_TEMPERATURE_2_MEASUREMENT_WAIT, 10);
                }
                else
                {
                  // In sequence 109 skip the last 4 states and go to publish result state
                  if(configConfigurationDb.MeasurementSequence == 109)
                  {
                    measureEndMeasurementSeq();
                  }
                  else
                  {
                    measureVmicSourceSelectMonitorFsmInit(MEASURE_CHANNEL_SELECT_AVRG_INDEX_REF_127, MEASSLCT_REF_MEMS_WITHOUT_NOISE_COMPENSATION, 127, 0);
                    vlapmainDebugLog("MEASURE_REFERENCE_2");
                    MeasurementIndex = 0;
                    vmicmodemSelect(MEASSLCT_REF_MEMS_WITHOUT_NOISE_COMPENSATION, 127);
                    measureSourceSelectTimeoutRetry = MEASURE_SELECT_MAX_RETRY;
                    MEASURE_FSM_STATE_CHANGE(MEASURE_REFERENCE_2_WAIT, MEASURE_NEW_SOURCE_SELECTION_TIME);
                  }         
                }
                break;
              }
              break;
            case MEASURE_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:
              // Check if not a single measurement sample recevied and we can perform retry
              if(measureSourceSelectTimeoutRetry)
              {
                uint8_t addPsl = (uint8_t)round(CurrrentPskLevel * 0.01);
                if(addPsl == 0 || CurrrentPskLevel > 95)
                {
                  addPsl = 1;
                }
                // Increase by low level PSL
                if(vmicmodemBeltMasking && measureIncreaseAndSetPsl(1) != RETURNCODE_OK)
                {
                  //measureIdleStateChange(10, 1);
                  //break;
                }
                //measureDecreaseAndSetPsl(addPsl);
                
                MeasurementIndex = 0;
                measureSourceSelectTimeoutRetry--;
                vlapmainDebugLog("Select MEASSLCT_TEMPERATURE 2 timeout, retry...");
                vmicmodemSelect(MEASSLCT_TEMPERATURE, 127);
                measureVmicSourceSelectMonitorFsmInit(MEASURE_CHANNEL_SELECT_AVRG_INDEX_TEMP_127, MEASSLCT_TEMPERATURE, 127, 0);
                MEASURE_FSM_STATE_CHANGE(MEASURE_TEMPERATURE_2_MEASUREMENT_WAIT, MEASURE_NEW_SOURCE_SELECTION_TIME);     
                vmicmodemBeltMasking = false;
                break;
              }
                   
              MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_STATE_TIMEOUT;
              measureIdleStateChange(10, 1);
              break;
            default:
            	break;
            }
            break;
            
            
            //      MEASURE_REFERENCE_2_WAIT:
          case 	MEASURE_REFERENCE_2_WAIT:
            if(measurementConditionCheck() )
            {
              measureIdleStateChange(10, 1);
              break;
            }
            switch(QueueEntry.MeasureRequest)
            {
            case MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_SAMPLE:
              if (vmicmodemBeltMasking)
                break;
              switch(measureVmicSourceSelectMonitorFsm(vmicapplayerUnFilteredSamplesGet(), MEASURE_VMIC_SOURCE_SELECT_SAMPLE_COUNT))
              {
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_IDLE:
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_WAIT:
                break;
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_FAIL:
                measureIdleStateChange(10, 1);
                break;
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_MEASURE:
                if(MeasurementIndex < measureNumberOfSamples)
                {
                  MeasurementsResults.Ref2[MeasurementIndex].MeasureStruct.vmicPressure = vmicapplayerUnFilteredSamplesGet();
                  // Ge tx monitor
                  hwdriversTxMonitorGet(&txMonitor);
                  // Get current.
                  MyTransCurrentMonitor = autopowerBeltCurrentMilliAmpGet();
                  MeasurementsResults.Ref2[MeasurementIndex].MeasureStruct.TxCurrent = NTOHS(MyTransCurrentMonitor);
                  MeasurementsResults.Ref2[MeasurementIndex].MeasureWord = HTONL(MeasurementsResults.Ref2[MeasurementIndex].MeasureWord);
                  MeasurementIndex++;
                  //              MEASURE_FSM_STATE_CHANGE(MEASURE_REFERENCE_2_WAIT, 10);
                }
                else
                {
                  measureVmicSourceSelectMonitorFsmInit(MEASURE_CHANNEL_SELECT_AVRG_INDEX_IC_127, MEASSLCT_INTERNAL_REFERENCE_CAPACITOR, 127, 0);
                  vlapmainDebugLog("IC4 127");
                  MeasurementIndex = 0;
                  vmicmodemSelect(MEASSLCT_INTERNAL_REFERENCE_CAPACITOR,  127);
                  measureSourceSelectTimeoutRetry = MEASURE_SELECT_MAX_RETRY;
                  MEASURE_FSM_STATE_CHANGE(MEASURE_VMIC_INTERNAL_CAP_4_MEASURE_WAIT, MEASURE_NEW_SOURCE_SELECTION_TIME);
                }
                break;
              }
              break;
            case MEASURE_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:
              // Check if not a single measurement sample recevied and we can perform retry
              if(measureSourceSelectTimeoutRetry)
              {
                uint8_t addPsl = (uint8_t)round(CurrrentPskLevel * 0.01);
                if(addPsl == 0)
                {
                  addPsl = 1;
                }
                // Increase by low level PSL
                if(vmicmodemBeltMasking && measureIncreaseAndSetPsl(1) != RETURNCODE_OK)
                {
                  //measureIdleStateChange(10, 1);
                  //break;
                }
                //measureDecreaseAndSetPsl(addPsl);
                
                MeasurementIndex = 0;
                measureSourceSelectTimeoutRetry--;
                vlapmainDebugLog("Select MEASURE_REFERENCE_2 2 timeout, retry...");
                vmicmodemSelect(MEASSLCT_REF_MEMS_WITHOUT_NOISE_COMPENSATION, 127);
                measureVmicSourceSelectMonitorFsmInit(MEASURE_CHANNEL_SELECT_AVRG_INDEX_REF_127, MEASSLCT_REF_MEMS_WITHOUT_NOISE_COMPENSATION, 127, 0);
                MEASURE_FSM_STATE_CHANGE(MEASURE_REFERENCE_2_WAIT, MEASURE_NEW_SOURCE_SELECTION_TIME);
                vmicmodemBeltMasking = false;
                break;
              }
              
              MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_STATE_TIMEOUT;
              measureIdleStateChange(10, 1);
              break;
            default:
            	break;
            }
            break;
            
            //    MEASURE_VMIC_INTERNAL_CAP_4_MEASURE_WAIT:
          case 	MEASURE_VMIC_INTERNAL_CAP_4_MEASURE_WAIT:
            if(measurementConditionCheck())
            {
              measureIdleStateChange(10, 1);
              break;
            }
            switch(QueueEntry.MeasureRequest)
            {
            case MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_SAMPLE:
              if (vmicmodemBeltMasking)
                break;
              switch(measureVmicSourceSelectMonitorFsm(vmicapplayerUnFilteredSamplesGet(), MEASURE_VMIC_SOURCE_SELECT_SAMPLE_COUNT))
              {
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_IDLE:
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_WAIT:
                break;
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_FAIL:
                measureIdleStateChange(10, 1);
                break;
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_MEASURE:
                if(MeasurementIndex < measureNumberOfSamples)
                {
                  MeasurementsResults.Ic4[MeasurementIndex].MeasureStruct.vmicPressure = vmicapplayerUnFilteredSamplesGet();
                  // Ge tx monitor
                  hwdriversTxMonitorGet(&txMonitor);
                  // Get current.
                  MyTransCurrentMonitor = autopowerBeltCurrentMilliAmpGet();
                  MeasurementsResults.Ic4[MeasurementIndex].MeasureStruct.TxCurrent = NTOHS(MyTransCurrentMonitor);
                  MeasurementsResults.Ic4[MeasurementIndex].MeasureWord = HTONL(MeasurementsResults.Ic4[MeasurementIndex].MeasureWord);
                  MeasurementIndex++;
                  //              MEASURE_FSM_STATE_CHANGE(MEASURE_VMIC_INTERNAL_CAP_4_MEASURE_WAIT, 10);
                }
                else
                {
                  measureVmicSourceSelectMonitorFsmInit(MEASURE_CHANNEL_SELECT_AVRG_INDEX_IC_87, MEASSLCT_INTERNAL_REFERENCE_CAPACITOR, 87, 0);
                  vlapmainDebugLog("IC5 87");
                  MeasurementIndex = 0;
                  vmicmodemSelect(MEASSLCT_INTERNAL_REFERENCE_CAPACITOR, 87);
                  measureSourceSelectTimeoutRetry = MEASURE_SELECT_MAX_RETRY;
                  MEASURE_FSM_STATE_CHANGE(MEASURE_VMIC_INTERNAL_CAP_5_MEASURE_WAIT, MEASURE_NEW_SOURCE_SELECTION_TIME);
                }
                break;
              }
              break;
            case MEASURE_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:
              // Check if not a single measurement sample recevied and we can perform retry
              if(measureSourceSelectTimeoutRetry)
              {
                uint8_t addPsl = (uint8_t)round(CurrrentPskLevel * 0.01);
                if(addPsl == 0)
                {
                  addPsl = 1;
                }
                // Increase by low level PSL
                if(vmicmodemBeltMasking && measureIncreaseAndSetPsl(1) != RETURNCODE_OK)
                {
                  //measureIdleStateChange(10, 1);
                  //break;
                }
                //measureDecreaseAndSetPsl(addPsl);
                
                MeasurementIndex = 0;
                measureSourceSelectTimeoutRetry--;
                vlapmainDebugLog("Select IC4 127 timeout, retry...");
                vmicmodemSelect(MEASSLCT_INTERNAL_REFERENCE_CAPACITOR,  127);
                measureVmicSourceSelectMonitorFsmInit(MEASURE_CHANNEL_SELECT_AVRG_INDEX_IC_127, MEASSLCT_INTERNAL_REFERENCE_CAPACITOR, 127, 0);
                MEASURE_FSM_STATE_CHANGE(MEASURE_VMIC_INTERNAL_CAP_4_MEASURE_WAIT, MEASURE_NEW_SOURCE_SELECTION_TIME);    
                vmicmodemBeltMasking = false;
                break;
              }
                  
              MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_STATE_TIMEOUT;
              measureIdleStateChange(10, 1);
              break;
            default:
              break;
            }
            break;
            
            //      MEASURE_VMIC_INTERNAL_CAP_5_MEASURE_WAIT:
          case 	MEASURE_VMIC_INTERNAL_CAP_5_MEASURE_WAIT:
            if(measurementConditionCheck())
            {
              measureIdleStateChange(10, 1);
              break;
            }
            switch(QueueEntry.MeasureRequest)
            {
            case MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_SAMPLE:
              if (vmicmodemBeltMasking)
                break;
              switch(measureVmicSourceSelectMonitorFsm(vmicapplayerUnFilteredSamplesGet(), MEASURE_VMIC_SOURCE_SELECT_SAMPLE_COUNT))
              {
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_IDLE:
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_WAIT:
                break;
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_FAIL:
                measureIdleStateChange(10, 1);
                break;
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_MEASURE:
                if(MeasurementIndex < measureNumberOfSamples)
                {
                  MeasurementsResults.Ic5[MeasurementIndex].MeasureStruct.vmicPressure = vmicapplayerUnFilteredSamplesGet();
                  // Ge tx monitor
                  hwdriversTxMonitorGet(&txMonitor);
                  // Get current.
                  MyTransCurrentMonitor = autopowerBeltCurrentMilliAmpGet();
                  MeasurementsResults.Ic5[MeasurementIndex].MeasureStruct.TxCurrent = NTOHS(MyTransCurrentMonitor);
                  MeasurementsResults.Ic5[MeasurementIndex].MeasureWord = HTONL(MeasurementsResults.Ic5[MeasurementIndex].MeasureWord);
                  MeasurementIndex++;
                  //              MEASURE_FSM_STATE_CHANGE(MEASURE_VMIC_INTERNAL_CAP_5_MEASURE_WAIT, 10);
                }
                else
                {
                  measureVmicSourceSelectMonitorFsmInit(MEASURE_CHANNEL_SELECT_AVRG_INDEX_IC_63, MEASSLCT_INTERNAL_REFERENCE_CAPACITOR, 63, 0);
                  vlapmainDebugLog("IC6 63");
                  MeasurementIndex = 0;
                  vmicmodemSelect(MEASSLCT_INTERNAL_REFERENCE_CAPACITOR, 63);
                  measureSourceSelectTimeoutRetry = MEASURE_SELECT_MAX_RETRY;
                  MEASURE_FSM_STATE_CHANGE(MEASURE_VMIC_INTERNAL_CAP_6_MEASURE_WAIT, MEASURE_NEW_SOURCE_SELECTION_TIME);
                }
                break;
              }
              break;
            case MEASURE_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:
              // Check if not a single measurement sample recevied and we can perform retry
              if(measureSourceSelectTimeoutRetry)
              {
                uint8_t addPsl = (uint8_t)round(CurrrentPskLevel * 0.01);
                if(addPsl == 0)
                {
                  addPsl = 1;
                }
                // Increase by low level PSL
                if(vmicmodemBeltMasking && measureIncreaseAndSetPsl(1) != RETURNCODE_OK)
                {
                  //measureIdleStateChange(10, 1);
                  //break;
                }
                //measureDecreaseAndSetPsl(addPsl);
                
                MeasurementIndex = 0;
                measureSourceSelectTimeoutRetry--;
                vlapmainDebugLog("Select IC5 87 timeout, retry...");
                vmicmodemSelect(MEASSLCT_INTERNAL_REFERENCE_CAPACITOR, 87);
                measureVmicSourceSelectMonitorFsmInit(MEASURE_CHANNEL_SELECT_AVRG_INDEX_IC_87, MEASSLCT_INTERNAL_REFERENCE_CAPACITOR, 87, 0);
                MEASURE_FSM_STATE_CHANGE(MEASURE_VMIC_INTERNAL_CAP_5_MEASURE_WAIT, MEASURE_NEW_SOURCE_SELECTION_TIME);              
                vmicmodemBeltMasking = false;
                break;
              }
              
              MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_STATE_TIMEOUT;
              measureIdleStateChange(10, 1);
              break;
            default:
            	break;
            }
            break;
            
            
            // 	MEASURE_VMIC_INTERNAL_CAP_6_MEASURE_WAIT:
          case 	MEASURE_VMIC_INTERNAL_CAP_6_MEASURE_WAIT:
            if(measurementConditionCheck())
            {
              measureIdleStateChange(10, 1);
              break;
            }
            switch(QueueEntry.MeasureRequest)
            {
            case MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_SAMPLE:
              if (vmicmodemBeltMasking)
                break;
              switch(measureVmicSourceSelectMonitorFsm(vmicapplayerUnFilteredSamplesGet(), MEASURE_VMIC_SOURCE_SELECT_SAMPLE_COUNT))
              {
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_IDLE:
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_WAIT:
                break;
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_FAIL:
                measureIdleStateChange(10, 1);
                break;
              case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_MEASURE:
                if(MeasurementIndex < measureNumberOfSamples)
                {
                  MeasurementsResults.Ic6[MeasurementIndex].MeasureStruct.vmicPressure = vmicapplayerUnFilteredSamplesGet();
                  // Ge tx monitor
                  hwdriversTxMonitorGet(&txMonitor);
                  // Get current.
                  MyTransCurrentMonitor = autopowerBeltCurrentMilliAmpGet();
                  MeasurementsResults.Ic6[MeasurementIndex].MeasureStruct.TxCurrent = NTOHS(MyTransCurrentMonitor);
                  MeasurementsResults.Ic6[MeasurementIndex].MeasureWord = HTONL(MeasurementsResults.Ic6[MeasurementIndex].MeasureWord);
                  MeasurementIndex++;
                  //              MEASURE_FSM_STATE_CHANGE(MEASURE_VMIC_INTERNAL_CAP_6_MEASURE_WAIT, 10);
                }
                else
                {
                  measureEndMeasurementSeq();
                }
                break;
              }
              break;
            case MEASURE_FSM_QUEUE_ENTRY_TYPE_TIMEOUT:
              // Check if not a single measurement sample recevied and we can perform retry
              if(measureSourceSelectTimeoutRetry)
              {
                uint8_t addPsl = (uint8_t)round(CurrrentPskLevel * 0.01);
                if(addPsl == 0)
                {
                  addPsl = 1;
                }
                // Increase by low level PSL
                if(vmicmodemBeltMasking && measureIncreaseAndSetPsl(1) != RETURNCODE_OK)
                {
                  //measureIdleStateChange(10, 1);
                  //break;
                }
                //measureDecreaseAndSetPsl(addPsl);
                
                MeasurementIndex = 0;
                measureSourceSelectTimeoutRetry--;
                vlapmainDebugLog("Select IC6 63 timeout, retry...");
                vmicmodemSelect(MEASSLCT_INTERNAL_REFERENCE_CAPACITOR, 63);
                measureVmicSourceSelectMonitorFsmInit(MEASURE_CHANNEL_SELECT_AVRG_INDEX_IC_63, MEASSLCT_INTERNAL_REFERENCE_CAPACITOR, 63, 0);
                MEASURE_FSM_STATE_CHANGE(MEASURE_VMIC_INTERNAL_CAP_6_MEASURE_WAIT, MEASURE_NEW_SOURCE_SELECTION_TIME);             
                vmicmodemBeltMasking = false;
                break;
              }
              
              MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_STATE_TIMEOUT;
              measureIdleStateChange(10, 1);
              break;
            default:
            	break;
            }
            break;
            
            
            //      MEASURE_PUBLISH_RESULTS_WAIT:
          case MEASURE_PUBLISH_RESULTS_WAIT:
            switch(QueueEntry.MeasureRequest)
            {
            case MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_SAMPLE:
              break;
            default:
              pccpmmAppLayerStruct.BoardSystemRegisters.OnBoardPushButton = 0;
              MeasurementsResults.MeasurementId = HTONS(MeasurementCounter);
              MeasurementsResults.Psl = HTONS(CurrrentPskLevel);
              MeasurementsResults.MeasurementGeneralStatus = MeasureFsmState;
              MeasurementsResults.MeasurementFailureReason = 0;
              MeasurementsResults.AbsolutePressure = HTONL((uint32_t)sensorAbsolutePressureGet());
              // Ge tx monitor
              hwdriversTxMonitorGet(&txMonitor);
              // Get current.
              MyTransCurrentMonitor = autopowerBeltCurrentMilliAmpGet();
              MeasurementsResults.RelaysState = HTONS(autoresonanceRelaysStateGet());
              MeasurementsResults.RelaysStateP2P = HTONS(autoresonanceP2PGet());
              uint16_t ReturnedEffectiveBeltResistance;
              autopowerEffectiveBeltResistanceGet(&ReturnedEffectiveBeltResistance);
              MeasurementsResults.EffectivBeltResistance = HTONL(ReturnedEffectiveBeltResistance);
              MeasurementsResults.AnalogSensorVoltage = 5;
              MeasurementsResults.PowerGood = 1;
              MeasurementsResults.BeltStatus = 1;
              //MeasurementsResults.ReceptionRms = HTONL(234); 
              MeasurementsResults.ReceptionRms = HTONL(measureRealMemsBadFramesCounter);
              uint16_t ReturnedFequency;
              hwdriversFrequencyMonitoringGet(&ReturnedFequency);
              MeasurementsResults.FeedBackFrequency = HTONL(ReturnedFequency);
              
              MeasurementsResults.MeasurementDurationTime = HTONL(measureStopTime - measureStartTime);
              MeasurementsResults.AccXStart = HTONS(measureAccXStart);
              MeasurementsResults.AccYStart = HTONS(measureAccYStart);
              MeasurementsResults.AccZStart = HTONS(measureAccZStart);
              
              MeasurementsResults.AccXPreReal = HTONS(measureAccXPreReal);
              MeasurementsResults.AccYPreReal = HTONS(measureAccYPreReal);
              MeasurementsResults.AccZPreReal = HTONS(measureAccZPreReal);
              
//              MeasurementsResults.AccXEnd = HTONS(sensorAccGet(SENSOR_AXIS_X));
//              MeasurementsResults.AccYEnd = HTONS(sensorAccGet(SENSOR_AXIS_Y));
//              MeasurementsResults.AccZEnd = HTONS(sensorAccGet(SENSOR_AXIS_Z));
              
//              MeasurementsResults.AccXP2P = HTONS(sensorAccP2PGet(SENSOR_AXIS_X));
//              MeasurementsResults.AccYP2P = HTONS(sensorAccP2PGet(SENSOR_AXIS_Y));
//              MeasurementsResults.AccZP2P = HTONS(sensorAccP2PGet(SENSOR_AXIS_Z));
              
              if(configConfigurationDb.MeasurementSequence != 109)
                MeasurementsResults.SequenceNumber = HTONL(108);
              else 
                MeasurementsResults.SequenceNumber = HTONL(configConfigurationDb.MeasurementSequence);
              
               //              vlapmainDebugLog("Event written to Log memory, turn off the device");
              eventsEventWrite(0, 0, PROTOCOLAPP_MEASUREMENT_ENDED_EVENT, (char*)&MeasurementsResults, sizeof(ProtocolappMeasurementEndedEvent_t) - measureSamplesReducedBySeqNumber(), 0);
              pccommapplayerTransmitterControl(TYPES_DISABLE);
#ifdef 	  MEASURE_PUSH_BUTTON_SIMULATE
              MEASURE_FSM_STATE_CHANGE(MEASURE_END_TEST_WAIT, 120000);
#else
              MEASURE_FSM_STATE_CHANGE(MEASURE_END_TEST_WAIT, 1000);
#endif
              break;
            }
            break;
            
            //      MEASURE_END_TEST_WAIT:
          case MEASURE_END_TEST_WAIT:
            switch(QueueEntry.MeasureRequest)
            {
            case MEASURE_FSM_QUEUE_ENTRY_TYPE_MODEM_SAMPLE:
              break;
            default:
#ifdef 	  MEASURE_PUSH_BUTTON_SIMULATE
              // SIMULATION ONLY 
              // TurnOn the device out from sleep mode
              //              vlapmainDemodulatorTaskControl( VLAPMAIN_TASK_START);
              pccpmmAppLayerStruct.BoardSystemRegisters.OnBoardPushButton = 1;
              autopowerManaualPwmSet(1);              
              pccommapplayerTransmitterControl(TYPES_ENABLE);
              //            pccommapplayerAutoResonanceControl(TYPES_ENABLE);
              //            pccommapplayerAutoCurrentControl(TYPES_ENABLE);
              autoresonanceManaualRelaysSet(MANUAL_RESONANCE);
              // Set the Auto Current to 6A
              //              pccommapplayerAutoCurrentSet(MEASURE_BELT_VMIC_DETECT_CURRENT_LEVEL_THRESHOLD_IN_MA+500);
              // Assume unknown UniqueIdSerialNumber
              pccpmmAppLayerStruct.VmicRegisters3.UniqueIdSerialNumber = TYPES_ENDIAN16_CHANGE(0);
              ResonanceEnergyLevelArrayIndex=0;
              MaxInitialRelayState =0;
              CurrentMuxInitialEnergy = 0;
              CurrrentPskLevel = configConfigurationDb.MinimalSequencePsl;
              vlapmainDebugLog("Automated PushButton");
              if(configConfigurationDb.AutoResonaceControl)
              {
                pccommapplayerAutoResonanceControl(TYPES_ENABLE);
              }
              else
              {
                pccommapplayerAutoResonanceControl(TYPES_DISABLE);
                autoresonanceManaualRelaysSet(configConfigurationDb.ManualRelayStateForSequence);
              }
              MEASURE_FSM_STATE_CHANGE(MEASURE_MINIMAL_ENERGY_WAIT, 1000);
#else
              // Measurement ended 
              MEASURE_FSM_STATE_CHANGE(MEASURE_IDLE, 100);
#endif
              break;
            } 
            break;
          }   // switch(MeasureFsmState)
        }     // if(RcvQueueStatus == TYPES_RCVQUEU_RCV) 
    }       // while(1)
}


ReturnCode_T measureIdleStateChange(uint16_t QueueWairTimeoutIn1mSec, uint8_t sendMeasureFailedEvent)
{
  // Stop measurement timeout timer
  xTimerStop(measureTimeoutTimerHandler, 0);
  
  measureModemStateTurnOff();
  pccpmmAppLayerStruct.BoardSystemRegisters.OnBoardPushButton = 0;
  // QueueWairTimeoutIn1mSec equal to 0 indicates we go to Idle without sending indication to user
  // as it is sporadic button push
  if(QueueWairTimeoutIn1mSec)
  {
    buzzAndVibrateForFail();
//    ledsPatternSet(LEDS_MAIN_PATTERN_FAIL);
  }
  
  sprintf(PrintBuff, "Last measurement phase before going to Idle = %d", MeasureFsmState);
  vlapmainDebugLog(PrintBuff);
  if(sendMeasureFailedEvent)
  {
    sprintf(PrintBuff, "Measurement failure reason = %d", MeasurementsResults.MeasurementFailureReason);
    vlapmainDebugLog(PrintBuff);
    
    // Stop capturing P2P of accelerometer
    sensorStopCaptureP2PAcc();
    // Save measurement end time
    measureStopTime = rtcEpochGet();
                
    MeasurementsResults.MeasurementId = HTONL(MeasurementCounter);
    MeasurementsResults.Psl = HTONS(CurrrentPskLevel);
    MeasurementsResults.MeasurementGeneralStatus = MeasureFsmState;
    //MeasurementsResults.MeasurementFailureReason = 0;
    MeasurementsResults.AbsolutePressure = HTONL((uint32_t)sensorAbsolutePressureGet());
    MeasurementsResults.RelaysState = HTONS(autoresonanceRelaysStateGet());
    MeasurementsResults.RelaysStateP2P = HTONS(autoresonanceP2PGet());
    uint16_t ReturnedEffectiveBeltResistance;
    autopowerEffectiveBeltResistanceGet(&ReturnedEffectiveBeltResistance);
    MeasurementsResults.EffectivBeltResistance = HTONL(ReturnedEffectiveBeltResistance);
    MeasurementsResults.AnalogSensorVoltage = 5;
    MeasurementsResults.PowerGood = 1;
    MeasurementsResults.BeltStatus = 1;
    uint16_t ReturnedFequency;
    hwdriversFrequencyMonitoringGet(&ReturnedFequency);
    MeasurementsResults.FeedBackFrequency = HTONL(ReturnedFequency);
    //MeasurementsResults.ReceptionRms = HTONL(234);
    MeasurementsResults.ReceptionRms = HTONL(measureRealMemsBadFramesCounter);
    MeasurementsResults.MeasurementDurationTime = HTONL(measureStopTime - measureStartTime);
    
    MeasurementsResults.AccXStart = HTONS(measureAccXStart);
    MeasurementsResults.AccYStart = HTONS(measureAccYStart);
    MeasurementsResults.AccZStart = HTONS(measureAccZStart);
    
    MeasurementsResults.AccXPreReal = HTONS(measureAccXPreReal);
    MeasurementsResults.AccYPreReal = HTONS(measureAccYPreReal);
    MeasurementsResults.AccZPreReal = HTONS(measureAccZPreReal);
    
//    MeasurementsResults.AccXEnd = HTONS(sensorAccGet(SENSOR_AXIS_X));
//    MeasurementsResults.AccYEnd = HTONS(sensorAccGet(SENSOR_AXIS_Y));
//    MeasurementsResults.AccZEnd = HTONS(sensorAccGet(SENSOR_AXIS_Z));
    
//    MeasurementsResults.AccXP2P = HTONS(sensorAccP2PGet(SENSOR_AXIS_X));
//    MeasurementsResults.AccYP2P = HTONS(sensorAccP2PGet(SENSOR_AXIS_Y));
//    MeasurementsResults.AccZP2P = HTONS(sensorAccP2PGet(SENSOR_AXIS_Z));
    
    if(configConfigurationDb.MeasurementSequence != 109)
      MeasurementsResults.SequenceNumber = HTONL(108);
    else 
      MeasurementsResults.SequenceNumber = HTONL(configConfigurationDb.MeasurementSequence);
    
    eventsEventWrite(0, 0, PROTOCOLAPP_MEASUREMENT_FAILED_EVENT, (char*)&MeasurementsResults, sizeof(ProtocolappMeasurementEndedEvent_t) - measureSamplesReducedBySeqNumber(), 0);    
  }
  
  pccommapplayerTransmitterControl(TYPES_DISABLE);
  
  // TODO: Failed measurement phase will now emulate pushbutton pressingRemove just for test
#ifdef  MEASURE_PUSH_BUTTON_SIMULATE
  MEASURE_FSM_STATE_CHANGE(MEASURE_END_TEST_WAIT, 1000);
#else
  MEASURE_FSM_STATE_CHANGE(MEASURE_IDLE, 100 /*QueueWairTimeoutIn1mSec*/);
#endif
  return(RETURNCODE_OK);
}





ReturnCode_T measureModemStateTurnOff()
{
  vlapmainDemodulatorTaskControl( VLAPMAIN_TASK_STOP);
  pccpmmAppLayerStruct.BoardSystemRegisters.OnBoardPushButton = 0;
  pccommapplayerTransmitterControl(TYPES_DISABLE);
  pccommapplayerAutoResonanceControl(TYPES_DISABLE);
  pccommapplayerAutoCurrentControl(TYPES_DISABLE);
  hwdriversPwmSet(0, 20, 1);
  // Always return OK
  return(RETURNCODE_OK);
}



uint32_t MeasurementDeltaGet(uint32_t Measurment1, uint32_t Measurment2)
{
  if(Measurment1 != Measurment2)
  {
    if(Measurment1 > Measurment2)
      return(Measurment1-Measurment2);
    else
      return(Measurment2-Measurment1);
  }
  else
    return(0);
}


/******************************************************************************
*** ReturnCode_T measureTaskEventSend(MeasureReq_T NotificationToMeasureTask , MeasureTaskQueueSourceT IsrOrNonIsrSource)
* @brief  // Sends notifications to the measurementTask   
* @param  
* @retval 
******************************************************************************/
ReturnCode_T measureTaskEventSend(MeasureReq_T NotificationToMeasureTask , MeasureTaskQueueSourceT IsrOrNonIsrSource)
{
  measureReqQueueEntry_T  MeasureTaskQueueEntry;
  // Fill the queue entry
  MeasureTaskQueueEntry.MeasureRequest = NotificationToMeasureTask;
  MeasureTaskQueueEntry.FreeTxBufferFlag   = 0;
  
  // Enqueue the event to the commTask input queue
  switch(IsrOrNonIsrSource)
  {
  case MEASURE_TASK_QUEUE_SEND_SOURCE_NORMAL:
    xQueueSend(measureReqQ, &MeasureTaskQueueEntry, 0);
    break;
  case MEASURE_TASK_QUEUE_SEND_SOURCE_ISR:
    xQueueSendFromISR( measureReqQ, &MeasureTaskQueueEntry, NULL );
    break;
  }
  
  return(RETURNCODE_OK);
}



/******************************************************************************
* @brief  void MeasureFsmTimerTimeoutCallback( )
* @param  
* @retval 
******************************************************************************/
void MeasureFsmTimerTimeoutCallback( )
{
  measureTaskEventSend(MEASURE_FSM_QUEUE_ENTRY_TYPE_TIMEOUT, MEASURE_TASK_QUEUE_SEND_SOURCE_NORMAL );
}



MeasureMinimalEnergyFsmStateT MeasureMinimalEnergyState;
uint8_t MeasureMinimalEnergyLowStepCounter;
uint8_t MeasureMinimalEnergyHighStepCounter;
uint8_t MeasureMinimalEnergyMinPSL;

/******************************************************************************
* @brief  void measureMinimalEnergyFsmInit()
* @param  
* @retval 
******************************************************************************/
void measureMinimalEnergyFsmInit()
{
  MeasureMinimalEnergyState = MEASURE_MINIMAL_ENERGY_IDLE;
}

/******************************************************************************
* @brief  MeasureMinimalEnergyFsmStateT measureMinimalEnergyFsm()
* @param  
* @retval 
******************************************************************************/
MeasureMinimalEnergyFsmStateT measureMinimalEnergyFsm(uint32_t peakToPeakSignalMeasure)
{
  uint32_t crcCounterResult = vmicmodemCrcCounterGet();
  MeasurePowerLevelT powerLevel = measureConvertP2PtoLevel(peakToPeakSignalMeasure, 0);
  sprintf(PrintBuff, "Minimal energy state: %d, P2P: %d, Level: %d, CurrentPSL: %d, CRC Counter: %d", MeasureMinimalEnergyState, peakToPeakSignalMeasure, powerLevel, CurrrentPskLevel, crcCounterResult);
  vlapmainDebugLog(PrintBuff);
  
  switch(MeasureMinimalEnergyState)
  {
  case MEASURE_MINIMAL_ENERGY_IDLE:
    MeasureMinimalEnergyLowStepCounter = 0;
    MeasureMinimalEnergyHighStepCounter = 0;
    
    // Check if we have high energy with low PSL - means that we to try low scale of energy
    if(powerLevel >= MEASURE_POWER_LEVEL5)
    {
      MeasureMinimalEnergyState = MEASURE_MINIMAL_ENERGY_LOW_ENERGY;
      // Set to 1 PSL to try and look for minimal energy at lower scale
      autopowerManaualPwmSet(1);
      CurrrentPskLevel = 1;
      MeasureMinimalEnergyMinPSL = 10;
      break;
    }
    else
    {
      // Try to find minimal energy at next level of PSL scale
      MeasureMinimalEnergyState = MEASURE_MINIMAL_ENERGY_LOWER_SCALE_INCREASE;
      MeasureMinimalEnergyMinPSL = CurrrentPskLevel;
      if(minimalEnergyMeasureIncreaseLimit(MEASURE_MINIMAL_ENERGY_MID_STEP) != RETURNCODE_OK)
      {
        MeasureMinimalEnergyState = MEASURE_MINIMAL_ENERGY_SEARCH_DONE;
        break;
      }
      // Increase by mid level PSL
      if(measureIncreaseAndSetPsl(MEASURE_MINIMAL_ENERGY_MID_STEP) != RETURNCODE_OK)
      {
        MeasureMinimalEnergyState = MEASURE_MINIMAL_ENERGY_FAILED;
        break;
      }
      MeasureMinimalEnergyLowStepCounter++;
    }
    
    break;
  case MEASURE_MINIMAL_ENERGY_LOWER_SCALE_INCREASE:
    // Check if we reached to max tries in lower scale increasement
    if(MeasureMinimalEnergyLowStepCounter > 8)
    {
      MeasureMinimalEnergyState = MEASURE_MINIMAL_ENERGY_HIGHER_SCALE_INCREASE;
      MeasureMinimalEnergyMinPSL = CurrrentPskLevel;
      if(minimalEnergyMeasureIncreaseLimit(MEASURE_MINIMAL_ENERGY_HIGH_STEP) != RETURNCODE_OK)
      {
        MeasureMinimalEnergyState = MEASURE_MINIMAL_ENERGY_SEARCH_DONE;
        break;
      }
      // Increase by high level PSL
      if(measureIncreaseAndSetPsl(MEASURE_MINIMAL_ENERGY_HIGH_STEP) != RETURNCODE_OK)
      {
        MeasureMinimalEnergyState = MEASURE_MINIMAL_ENERGY_FAILED;
        break;
      }
      MeasureMinimalEnergyHighStepCounter++;
      break;
    }
    
    // Check if we need more energy
    if(powerLevel <= MEASURE_POWER_LEVEL3)
    {
      // Check if we have enough energy to get 0 error counter
      if((!crcCounterResult) && (powerLevel == MEASURE_POWER_LEVEL3))
      {
          MeasureMinimalEnergyState = MEASURE_MINIMAL_ENERGY_LOWER_SCALE_DECREASE;
          // Decrease PSL 
          if(measureDecreaseAndSetPsl(MEASURE_MINIMAL_ENERGY_LOW_STEP) != RETURNCODE_OK)
          {
            MeasureMinimalEnergyState = MEASURE_MINIMAL_ENERGY_FAILED;
            break;
          }
        break;
      }
      if(minimalEnergyMeasureIncreaseLimit(MEASURE_MINIMAL_ENERGY_MID_STEP) != RETURNCODE_OK)
      {
        MeasureMinimalEnergyState = MEASURE_MINIMAL_ENERGY_SEARCH_DONE;
        break;
      }
      // Increase by mid level PSL
      if(measureIncreaseAndSetPsl(MEASURE_MINIMAL_ENERGY_MID_STEP) != RETURNCODE_OK)
      {
        MeasureMinimalEnergyState = MEASURE_MINIMAL_ENERGY_FAILED;
        break;
      }
      // Remain in current state
      MeasureMinimalEnergyState = MEASURE_MINIMAL_ENERGY_LOWER_SCALE_INCREASE;
      MeasureMinimalEnergyLowStepCounter++;
      break;
    }
    // Check if we have energy overflow
    if(powerLevel > MEASURE_POWER_LEVEL3)
    {
      MeasureMinimalEnergyState = MEASURE_MINIMAL_ENERGY_LOWER_SCALE_DECREASE;
      // Decrease PSL 
      if(measureDecreaseAndSetPsl(MEASURE_MINIMAL_ENERGY_LOW_STEP) != RETURNCODE_OK)
      {
        MeasureMinimalEnergyState = MEASURE_MINIMAL_ENERGY_FAILED;
        break;
      }
    } 
    break;
  case MEASURE_MINIMAL_ENERGY_LOWER_SCALE_DECREASE:
    // Check if we have energy overflow
    if(powerLevel >= MEASURE_POWER_LEVEL5)
    {
      // Check if in the next step we are decreasing to level before the increasement
      if(MeasureMinimalEnergyMinPSL > (CurrrentPskLevel + (MEASURE_MINIMAL_ENERGY_LOW_STEP * -1)))
      {
        // Stop decresing, notify on search done
        MeasureMinimalEnergyState = MEASURE_MINIMAL_ENERGY_SEARCH_DONE;
        break;
      }
      
      MeasureMinimalEnergyState = MEASURE_MINIMAL_ENERGY_LOWER_SCALE_DECREASE;
      // Decrease PSL 
      if(measureDecreaseAndSetPsl(MEASURE_MINIMAL_ENERGY_LOW_STEP) != RETURNCODE_OK)
      {
        MeasureMinimalEnergyState = MEASURE_MINIMAL_ENERGY_FAILED;
        break;
      }
      break;
    }
    else
    {
      // Search is done
      MeasureMinimalEnergyState = MEASURE_MINIMAL_ENERGY_SEARCH_DONE;
    }
    break;
  case MEASURE_MINIMAL_ENERGY_HIGHER_SCALE_INCREASE:
    // Check if we reached to max tries in higher scale increasement
    if(MeasureMinimalEnergyHighStepCounter > 6)
    {
      // Notify a success as we reached to max PSL
      MeasureMinimalEnergyState = MEASURE_MINIMAL_ENERGY_SEARCH_DONE;
    }
    
    // Check if we need more energy
    if(powerLevel <= MEASURE_POWER_LEVEL3)
    {
      // Check if we have enough energy to get 0 error counter
      if((!crcCounterResult) && (powerLevel == MEASURE_POWER_LEVEL3))
      {
          MeasureMinimalEnergyState = MEASURE_MINIMAL_ENERGY_HIGHER_SCALE_DECREASE;
          // Decrease PSL 
          if(measureDecreaseAndSetPsl(MEASURE_MINIMAL_ENERGY_MID_STEP) != RETURNCODE_OK)
          {
            MeasureMinimalEnergyState = MEASURE_MINIMAL_ENERGY_FAILED;
            break;
          }
        break;
      }
      
      if(minimalEnergyMeasureIncreaseLimit(MEASURE_MINIMAL_ENERGY_HIGH_STEP) != RETURNCODE_OK)
      {
        MeasureMinimalEnergyState = MEASURE_MINIMAL_ENERGY_SEARCH_DONE;
        break;
      }
      // Increase by mid level PSL
      if(measureIncreaseAndSetPsl(MEASURE_MINIMAL_ENERGY_HIGH_STEP) != RETURNCODE_OK)
      {
        MeasureMinimalEnergyState = MEASURE_MINIMAL_ENERGY_FAILED;
        break;
      }
      // Remain in current state
      MeasureMinimalEnergyState = MEASURE_MINIMAL_ENERGY_HIGHER_SCALE_INCREASE;
      MeasureMinimalEnergyHighStepCounter++;
      break;
    }
    // Check if we have energy overflow
    if(powerLevel > MEASURE_POWER_LEVEL3)
    {
      MeasureMinimalEnergyState = MEASURE_MINIMAL_ENERGY_HIGHER_SCALE_DECREASE;
      // Decrease PSL 
      if(measureDecreaseAndSetPsl(MEASURE_MINIMAL_ENERGY_MID_STEP) != RETURNCODE_OK)
      {
        MeasureMinimalEnergyState = MEASURE_MINIMAL_ENERGY_FAILED;
        break;
      }
    } 
    break;
  case MEASURE_MINIMAL_ENERGY_HIGHER_SCALE_DECREASE:
    // Check if in the next step we are decreasing to level before the increasement
    if(MeasureMinimalEnergyMinPSL > (CurrrentPskLevel + (MEASURE_MINIMAL_ENERGY_MID_STEP * -1)))
    {
      // Stop decresing, notify on search done
      MeasureMinimalEnergyState = MEASURE_MINIMAL_ENERGY_SEARCH_DONE;
      break;
    }
    
    // Check if we have energy overflow
    if(powerLevel >= MEASURE_POWER_LEVEL4)
    {
      MeasureMinimalEnergyState = MEASURE_MINIMAL_ENERGY_LOWER_SCALE_DECREASE;
      // Decrease PSL 
      if(measureDecreaseAndSetPsl(MEASURE_MINIMAL_ENERGY_MID_STEP) != RETURNCODE_OK)
      {
        MeasureMinimalEnergyState = MEASURE_MINIMAL_ENERGY_FAILED;
        break;
      }
      break;
    }
    else
    {
      // Search is done
      MeasureMinimalEnergyState = MEASURE_MINIMAL_ENERGY_SEARCH_DONE;
    }
    break;
  case MEASURE_MINIMAL_ENERGY_LOW_ENERGY:
    // Reached to wanted level of energy
    if(powerLevel > MEASURE_POWER_LEVEL3)
    {
      MeasureMinimalEnergyState = MEASURE_MINIMAL_ENERGY_SEARCH_DONE;
      break;
    }
    if(minimalEnergyMeasureIncreaseLimit(1) != RETURNCODE_OK)
    {
      MeasureMinimalEnergyState = MEASURE_MINIMAL_ENERGY_SEARCH_DONE;
      break;
    }
    // Try to raise a little bit to reach to higher then MEASURE_POWER_LEVEL3
    if(measureIncreaseAndSetPsl(1) != RETURNCODE_OK)
    {
      MeasureMinimalEnergyState = MEASURE_MINIMAL_ENERGY_FAILED;
      break;
    }
    // Remain in current state    
    MeasureMinimalEnergyState = MEASURE_MINIMAL_ENERGY_LOW_ENERGY;
    break;
  default:
	  break;
  }
  
  return MeasureMinimalEnergyState;
}

uint8_t MeasureStabilityState;
uint32_t MeasureStabilityLastMeasuredValue;
uint8_t MeasureStabilityIterationsCounter;

/******************************************************************************
* @brief  void measureStabilityFsmInit()
* @param  
* @retval 
******************************************************************************/
void measureStabilityFsmInit()
{
  MeasureStabilityState = 0;
}


/******************************************************************************
* @brief  MeasureStabilityFsmReturnedStatusT  measureStabilityFsm()
* @param  
* @retval 
******************************************************************************/
MeasureStabilityFsmReturnedStatusT  measureStabilityFsm()
{
  uint8_t ReturnedValue = MEASURE_STABILITY_FSM_RETURNED_STATUS_IDLE;
  uint32_t averageSample;
  uint32_t peakToPeak;
  uint32_t crcError;
 
  sprintf(PrintBuff, "Stability  %d", CurrrentPskLevel);
  vlapmainDebugLog(PrintBuff);
  
  switch(MeasureStabilityState)
  {
  case 0:
    MeasureStabilityIterationsCounter = 2;
    ReturnedValue = MEASURE_STABILITY_FSM_RETURNED_STATUS_IDLE;
    PeakToPeakInBandCounter = 0;
    MeasureStabilityState = 1;
    break;
  case 1:
    // Delay to let the Peak to Peak buffer be filled
    if(MeasureStabilityIterationsCounter)
      MeasureStabilityIterationsCounter--;
    else
      MeasureStabilityState = 2;
    ReturnedValue = MEASURE_STABILITY_FSM_RETURNED_STATUS_INPROGRESS;
    break;
  case 2:
    // Take first smaple to be used as initial ref for the next state
    MeasureStabilityLastMeasuredValue = vmicapplayerSignalPeakToPeakValueGet();
    MeasureStabilityIterationsCounter = MEASUREMENT_STABILITY_MAX_ITERATIONS;
    ReturnedValue = MEASURE_STABILITY_FSM_RETURNED_STATUS_INPROGRESS;
    MeasureStabilityState = 3;
    break;
  case 3:
	crcError = vmicmodemCrcCounterGet();
    peakToPeak = vmicapplayerSignalPeakToPeakValueGet();
    // Try converging Peak to peak
    sprintf(PrintBuff, "Measured Peak2Peak= %d, Psl= %d, tries left = %d, crc= %d", peakToPeak, CurrrentPskLevel, MeasureStabilityIterationsCounter, crcError);
    vlapmainDebugLog(PrintBuff);
    if(MeasureStabilityIterationsCounter)
    {
      MeasureStabilityIterationsCounter--;
      if( peakToPeak > 10 || crcError)
      {
        if (measureIncreaseAndSetPsl(measurePslStep) != RETURNCODE_OK)
        {
          ReturnedValue = MEASURE_STABILITY_FSM_RETURNED_STATUS_FAILED;
          MeasureStabilityState = MEASURE_STABILITY_FSM_RETURNED_STATUS_IDLE;
          return(ReturnedValue);
        }
        PeakToPeakInBandCounter = 0;
      }
      else
        PeakToPeakInBandCounter++;
      
      
      ReturnedValue = MEASURE_STABILITY_FSM_RETURNED_STATUS_INPROGRESS;
      // Look for consquative sucesses before decalring Peak to Peak convergance 
      if(PeakToPeakInBandCounter >= MEASAURE_NUMBER_OF_CONSEQUATIVE_SUCCESS_RETRIES)
      {
        MeasureStabilityState = 4;
        PeakToPeakInBandCounter = 0;
        MeasureStabilityLastMeasuredValue = vmicapplayerFilteredSamplesGet();
        if (measureIncreaseAndSetPsl(measurePslStep) != RETURNCODE_OK)
        {
          ReturnedValue = MEASURE_STABILITY_FSM_RETURNED_STATUS_FAILED;
          MeasureStabilityState = MEASURE_STABILITY_FSM_RETURNED_STATUS_IDLE;
          return(ReturnedValue);
        }
        MeasureStabilityIterationsCounter = MEASUREMENT_STABILITY_MAX_ITERATIONS;
      }
      
    }
    else
    {
      MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_STABILITY_P2P_MAX_ITERATIONS_REACHED;
      ReturnedValue = MEASURE_STABILITY_FSM_RETURNED_STATUS_FAILED;
      MeasureStabilityState = MEASURE_STABILITY_FSM_RETURNED_STATUS_IDLE;
    }
    break;
  case 4:
    crcError = vmicmodemCrcCounterGet();
    averageSample = vmicapplayerFilteredSamplesGet();
    // Try converging average values
    sprintf(PrintBuff, "New Measured Average= %d, previous= %d, Psl= %d, crc= %d", averageSample, MeasureStabilityLastMeasuredValue, CurrrentPskLevel, crcError);
    vlapmainDebugLog(PrintBuff);
    if(MeasureStabilityIterationsCounter)
    {
      MeasureStabilityIterationsCounter--;

      if (PeakToPeakInBandCounter < 2)
      {
        if (!crcError)
        {
          if((MeasureStabilityLastMeasuredValue + 20) < averageSample  ||  (MeasureStabilityLastMeasuredValue - 20) > averageSample)
            PeakToPeakInBandCounter = 0;
          else
            PeakToPeakInBandCounter++;
        }
        if (measureIncreaseAndSetPsl(measurePslStep) != RETURNCODE_OK)
        {
          ReturnedValue = MEASURE_STABILITY_FSM_RETURNED_STATUS_FAILED;
          MeasureStabilityState = MEASURE_STABILITY_FSM_RETURNED_STATUS_IDLE;
          return(ReturnedValue);
        }
    }
      else
      {
          if((MeasureStabilityLastMeasuredValue + 20) < averageSample  ||  (MeasureStabilityLastMeasuredValue - 20) > averageSample || crcError)
          {
            if (measureIncreaseAndSetPsl(measurePslStep) != RETURNCODE_OK)
            {
              ReturnedValue = MEASURE_STABILITY_FSM_RETURNED_STATUS_FAILED;
              MeasureStabilityState = MEASURE_STABILITY_FSM_RETURNED_STATUS_IDLE;
              return(ReturnedValue);
            }
            if (!crcError)
              PeakToPeakInBandCounter = 0;
          }
          else
            PeakToPeakInBandCounter++;
      }

      
      MeasureStabilityLastMeasuredValue = averageSample;
      ReturnedValue = MEASURE_STABILITY_FSM_RETURNED_STATUS_INPROGRESS;
      // Look for  consquative sucesses before decalring avarage convergance 
      if(PeakToPeakInBandCounter >= MEASAURE_NUMBER_OF_CONSEQUATIVE_SUCCESS_RETRIES)
      {
        ReturnedValue = MEASURE_STABILITY_FSM_RETURNED_STATUS_STABLE;
        MeasureStabilityState = 0;
      }
    }
    else
    {
      MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_STABILITY_AVERAGE_MAX_ITERATIONS_REACHED;
      ReturnedValue = MEASURE_STABILITY_FSM_RETURNED_STATUS_FAILED;
      MeasureStabilityState = MEASURE_STABILITY_FSM_RETURNED_STATUS_IDLE;
    }
  }
  
  return(ReturnedValue);
}



/**************************************************************************
*** uin32_t measurementSerialIdGet()
* 
*
*
*
***************************************************************************/
uint32_t measurementSerialIdGet()
{
  return( VmicId);
  
  //return(151);
}

// TODO: Function name do not accord with coding conventions 
ReturnCode_T buzzAndVibrateForFail()
{
//  buzzerRequestToQueueAdd(BUZZER_BEEPING_OPTION3);
//  vibratorRequestToQueueAdd(VIBRATOR_OPTION3);
  
  return RETURNCODE_OK;
}

ReturnCode_T  measurmentNurseModeEventBuildAndCreate()
{
  pccpmmAppLayerStruct.BoardSystemRegisters.OnBoardPushButton = 0;
  measureNurseModeDone = 1;
  ProtocolappNurseModeEndEvent_t  *Ptr = (ProtocolappNurseModeEndEvent_t*)pvPortMalloc(sizeof(ProtocolappNurseModeEndEvent_t));
  
  Ptr->endNurseModeTime = HTONL(rtcEpochGet());
  Ptr->lastPlugInTime = HTONL(auditNurseModeLastPluggedInTimeGet());
  Ptr->lastPlugOutTime = HTONL(auditNurseModeLastPluggedOutTimeGet());
  Ptr->nurseModeDurationSeconds = HTONL(NurseModeStopTime - NurseModeStartTime);
  Ptr->NurseModeMaxIndicationLevel = NurseModeMaxIndicationLevel;
  Ptr->NurseModeStartTime = HTONL(NurseModeStartTime);
  Ptr->NurseModeStopTime = HTONL(NurseModeStopTime);
  Ptr->EventTimeStamp10mSec = 0;
  
  if(Ptr)
  {
    eventsEventWrite(0, 0, PROTOCOLAPP_GENERAL_NURSE_MODE_END_EVENT, (char*)Ptr, sizeof(ProtocolappNurseModeEndEvent_t), 1);
  }
  
  return(RETURNCODE_OK);
}


/**************************************************************************
*** uin32_t measurementStateGet()
* 
*
*
*
***************************************************************************/
MeasureFsmState_T measurementStateGet()
{
  return(MeasureFsmState);
}


volatile int TestTest = 0;
/**************************************************************************
*** uint8_t measurementConditionCheck()
* 
*
*
*
***************************************************************************/
uint8_t measurementConditionCheck()
{
  uint8_t MyReturn = 0;
   
  char * Ptr = PrintBuff;
  
  strcpy(Ptr, "Measurement Error: ");
  
  TestTest++;
  
  
  // Measurement will not start if device is charging( Lab exception: USB connected and Vectorious GUI or VTS are polling the device)
  //if(chargerChargingStatusGet())
//  if(chargerDcPlugStatusGet() == CHARGER_DC_PLUG_INSERTED)
//  {
//    MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_CHARGING_PLUG_CONNECTED;
//    eventsEventWrite(0,0,PROTOCOLAPP_MEASUREMENT_ABORTED_CHARGING_EVENT,0,0,0);
//    strcat(Ptr, "USB ConnectedError ");
//    MyReturn |= 1;
//  }
  // Battery must be within the allowed valaues
  if(auditBatteryStatusGet()!=AUDIT_BV_OK)
  {
    MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_LOW_BATTERY;
    eventsEventWrite(0,0,PROTOCOLAPP_MEASUREMENT_ABORTED_BATTERY_LEVEL_EVENT,0,0,0);
    strcat(Ptr, "BATT Out of range ");
    MyReturn |= 16;
  }
  // Ambient temperature must be lower that the max allowed temperature 
  if(auditAbiantTemperatureStateGet() != AUDIT_TEMPERATURE_NORMAL)
  {
    MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_OVER_TEMPERATURE;
    eventsEventWrite(0,0,PROTOCOLAPP_MEASUREMENT_ABORTED_TEMPERATURE_LEVEL_EVENT,0,0,0);
    strcat(Ptr, "Ambient temperature Out of range ");
    MyReturn |= 2;
  }

  // Transmitter temperature must be lower that the max allowed temperature 
  if(auditTxTemperatureOverheatCheck() != AUDIT_TEMPERATURE_NORMAL)
  {
    MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_OVER_TEMPERATURE;
    eventsEventWrite(0,0,PROTOCOLAPP_MEASUREMENT_ABORTED_TEMPERATURE_LEVEL_EVENT,0,0,0);
    strcat(Ptr, "Tx temperature Out of range ");
    MyReturn |= 4;
  }
  
#if 0  
  if(auditBeltHumanStatusGet() == AUDIT_BELT_OPEN)
  {
    MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_BELT_OPEN;
    eventsEventWrite(0,0,PROTOCOLAPP_MEASUREMENT_ABORTED_BELT_OPEN_EVENT,0,0,0);
    strcat(Ptr, "Belt is open");
    MyReturn |= 5;
  }
#endif  
  
  AuditBeltHumanCheck_T auditBeltHumanStatus = auditBeltHumanStatusGet();
  switch(auditBeltHumanStatus)
  {
    case AUDIT_BELT_OPEN:
      MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_BELT_OPEN;
      eventsEventWrite(0,0,PROTOCOLAPP_MEASUREMENT_ABORTED_BELT_OPEN_EVENT,0,0,0);
      strcat(Ptr, "Belt open problem");
      MyReturn |= 0x08;
      break;
    case AUDIT_BELT_NO_HUMAN:
    case AUDIT_BELT_HUMAN_OK:
      break;
    default:
    	break;
  }
 
  
  
  if(MyReturn)
    vlapmainDebugLog(PrintBuff);
  
  return(MyReturn);
}

//uint16_t measureCalcBuzzerPrescalerByParam(uint32_t param)
//{
//  //uint16_t prescaler = (uint16_t) ((-0.0000001 * param * param * param) + (0.0008 * param * param) - (2.1334 * param) + 2532.8);
//  //uint16_t prescaler = (uint16_t) 2705.2 * 
//  
//  uint16_t prescaler = (uint16_t) (-732.9*log(param) + 6165.3);
//  return prescaler; 
//}

uint32_t measureCalcBuzzerFreqByParam(uint32_t param)
{
  //uint32_t freq = (uint32_t) ((0.0049 * param - 0.8453)*1000);
  uint32_t freq = (uint32_t) ((0.0041 * param - 0.7404)*1000);
  
  if (freq < 20000)
  {
    if (freq < 5000)
    {
      return 200;
    }
    else
    {
      return freq - 1000;
    }
  }
  else
  {
    return 200;
  }
}

MeasurePowerLevelT measureConvertP2PtoLevel(uint32_t peakToPeakSignalMeasure, uint32_t peakToPeakMeasure)
{
//  if (peakToPeakSignalMeasure < 1000)
//    return MEASURE_POWER_LEVEL0;
//  else if((peakToPeakSignalMeasure >= 1000) && (peakToPeakSignalMeasure < 2000))
//    return MEASURE_POWER_LEVEL1;
//  else if((peakToPeakSignalMeasure >= 2000) && (peakToPeakSignalMeasure < 2600))
//    return MEASURE_POWER_LEVEL2;
//  else if((peakToPeakSignalMeasure >= 2600) && (peakToPeakSignalMeasure < 3200))
//    return MEASURE_POWER_LEVEL3;
//  else if((peakToPeakSignalMeasure >= 3200) && (peakToPeakSignalMeasure < 3600))
//    return MEASURE_POWER_LEVEL4;
//  else if(peakToPeakSignalMeasure >= 3600)
//    return MEASURE_POWER_LEVEL5;
//  else
//    return MEASURE_POWER_LEVEL0;


  if (peakToPeakSignalMeasure < 1000)
    return MEASURE_POWER_LEVEL0;
  else if((peakToPeakSignalMeasure >= 1000) && (peakToPeakSignalMeasure < 1800))
    return MEASURE_POWER_LEVEL1;
  else if((peakToPeakSignalMeasure >= 1800) && (peakToPeakSignalMeasure < 2300))
    return MEASURE_POWER_LEVEL2;
  else if((peakToPeakSignalMeasure >= 2300) && (peakToPeakSignalMeasure < 2700))
    return MEASURE_POWER_LEVEL3;
  else if((peakToPeakSignalMeasure >= 2700) && (peakToPeakSignalMeasure < 3400))
    return MEASURE_POWER_LEVEL4;
  else if(peakToPeakSignalMeasure >= 3400)
    return MEASURE_POWER_LEVEL5;
  else
    return MEASURE_POWER_LEVEL0;
}

ReturnCode_T measureNurseModeIndication(uint32_t peakToPeakSignalMeasure, uint32_t peakToPeakMeasure)
{
  MeasurePowerLevelT nurseModeLevel = MEASURE_POWER_LEVEL0;
  uint32_t freq = measureCalcBuzzerFreqByParam(peakToPeakSignalMeasure);
  //printf("p2pSignal: %d\n", peakToPeakSignalMeasure);
  //printf("freq: %d\n", freq);
  //freq = 200;
  if(NurseModeFirstBuzzerSet)
  {
    freq = 200;
    NurseModeLastFreq = freq;
    buzzerOn(freq);
    NurseModeFirstBuzzerSet = 0;
  }
  else
  {
    if(NurseModeLastFreq < freq)
    {
      NurseModeFreqChangeCounterDown = 0;
      NurseModeFreqChangeCounterUp++;
    }
    else if (NurseModeLastFreq > freq)
    {
      NurseModeFreqChangeCounterUp = 0;
      NurseModeFreqChangeCounterDown++;
    }
    else
    {
      NurseModeFreqChangeCounterUp = 0;
      NurseModeFreqChangeCounterDown = 0;
    }
    
    //     printf("Last freq: %d, freq: %d", NurseModeLastFreq, freq);
    //     printf("Counter up: %d\n", NurseModeFreqChangeCounterUp);
    //     printf("Counter down: %d\n", NurseModeFreqChangeCounterDown);
    
    if((NurseModeFreqChangeCounterUp >= NurseModeFreqChange) || (NurseModeFreqChangeCounterDown >= NurseModeFreqChange))
    {
      if(abs(NurseModeLastFreq - freq) > 1500)
      {
        nurseModeLevel = measureConvertP2PtoLevel(peakToPeakSignalMeasure, peakToPeakMeasure);
        //printf("up is: %d, down is: %d\n", NurseModeFreqChangeCounterUp, NurseModeFreqChangeCounterDown);
        NurseModeFreqChangeCounterUp = 0;
        NurseModeFreqChangeCounterDown = 0;  
        //printf("Changed ! last freq is: %d, new freq is : %d\n", NurseModeLastFreq, freq);
        NurseModeLastFreq = freq; 
        buzzerFreqSet(freq);
        
        if(nurseModeLevel != MEASURE_POWER_LEVEL0)
        {
          NurseModeLevelFound = 1;
          NurseModeIdleCount = 0;
        }
        
        switch(nurseModeLevel)
        {
        case MEASURE_POWER_LEVEL0:
          NurseModeLevelFound = 0;
          //ledsPatternSet(LEDS_PATTERN_IDLE);
//          ledsPatternSetByPriority(LEDS_PATTERN_IDLE, LEDS_PRIORITY_P1_HIGHEST);
          break;
        case MEASURE_POWER_LEVEL1:
          //ledsPatternSet(LEDS_POWER_LEVEL1);
//          ledsPatternSetByPriority(LEDS_POWER_LEVEL1, LEDS_PRIORITY_P1_HIGHEST);
          break;
        case MEASURE_POWER_LEVEL2:
          //ledsPatternSet(LEDS_POWER_LEVEL2);
//          ledsPatternSetByPriority(LEDS_POWER_LEVEL2, LEDS_PRIORITY_P1_HIGHEST);
          break;
        case MEASURE_POWER_LEVEL3:
          //ledsPatternSet(LEDS_POWER_LEVEL3);
//          ledsPatternSetByPriority(LEDS_POWER_LEVEL3, LEDS_PRIORITY_P1_HIGHEST);
          break;
        case MEASURE_POWER_LEVEL4:
          //ledsPatternSet(LEDS_POWER_LEVEL4);
//          ledsPatternSetByPriority(LEDS_POWER_LEVEL4, LEDS_PRIORITY_P1_HIGHEST);
          break;
        case MEASURE_POWER_LEVEL5:
          //ledsPatternSet(LEDS_POWER_LEVEL5);
//          ledsPatternSetByPriority(LEDS_POWER_LEVEL5, LEDS_PRIORITY_P1_HIGHEST);
//          vibratorRequestToQueueAdd(VIBRATOR_NURSE_MODE);
          break;
        default:
          break;
        }
      } 
    } 
  }
  
  // This check supposed to prevent a bug that causing the external to display 1 led when there is no need to display any.
  // This bug is caused when the implant disapear immediately from the external site.
  if(NurseModeLevelFound)
  {
    nurseModeLevel = measureConvertP2PtoLevel(peakToPeakSignalMeasure, peakToPeakMeasure);
    NurseModeIdleCount++;
    if(nurseModeLevel == MEASURE_POWER_LEVEL0 && (NurseModeIdleCount >= NurseModeFreqChange))
    {
      
//      ledsPatternSetByPriority(LEDS_PATTERN_IDLE, LEDS_PRIORITY_P1_HIGHEST);
    }
    
  }
  
  
  if (nurseModeLevel > NurseModeMaxIndicationLevel)
  {
    NurseModeMaxIndicationLevel = nurseModeLevel;
  }
  
  return (RETURNCODE_OK);
}

ReturnCode_T measureDecreaseAndSetPsl(uint8_t decreaseNum)
{
  ReturnCode_T returnCode = RETURNCODE_OK;
  
  if(configConfigurationDb.MeasurementPslControl == 0)
    return (returnCode);
  
  if ((CurrrentPskLevel - decreaseNum) > 0)
  {
    CurrrentPskLevel -= decreaseNum;
    autopowerManaualPwmSet(CurrrentPskLevel);
  }
  else
  {
    MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_MAX_PSL_REACHED;
    returnCode = RETURNCODE_ERROR;
  }
  return (returnCode);
}


ReturnCode_T minimalEnergyMeasureIncreaseLimit(uint8_t increaseNum)
{
  uint8_t pslLimit = 85;
  if (CurrrentPskLevel + increaseNum <= pslLimit)
  {
    return RETURNCODE_OK;
  }
  else
  {
	// Limit PSL increase to 85
	measureIncreaseAndSetPsl(pslLimit - CurrrentPskLevel);
	return RETURNCODE_ERROR;
  }
}

ReturnCode_T measureIncreaseAndSetPsl(uint8_t increaseNum)
{
  ReturnCode_T returnCode = RETURNCODE_OK;
  
  if(configConfigurationDb.MeasurementPslControl == 0)
    return (returnCode);
  
  if ((CurrrentPskLevel + increaseNum) < (measureMaxOperationPsl + 1))
  {
    CurrrentPskLevel += increaseNum;
    autopowerManaualPwmSet(CurrrentPskLevel);
  }
  else
  {
    MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_MAX_PSL_REACHED;
    returnCode = RETURNCODE_ERROR;
  }
  return (returnCode);
}

uint8_t measureMeasurementInProgressGet()
{
  if(MeasureFsmState != MEASURE_IDLE)
    return(1);
  else
    return(0);
}




measureVmicSourceSelectMonitorFsmState_T measureVmicSourceSelectMonitorFsmInit(MeasureChennelSelectAvrgIndex_T VmicSourceSelectAverageIndex, MeasurmentSelect_T VmicSource, uint8_t C2fSwitchArray, float PslPercentageIncrease)
{
  measureVmicSourceSelectMonitorFsmDb.VmicSourceSelectForRetry = VmicSource;
  measureVmicSourceSelectMonitorFsmDb.VmicSourceSelectC2fSwitchArrayForRetry = C2fSwitchArray;
  measureVmicSourceSelectMonitorFsmDb.LimitEntry     = VmicSourceLimitsArray[VmicSourceSelectAverageIndex];
  measureVmicSourceSelectMonitorFsmDb.State = MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_IDLE;
  measureVmicSourceSelectMonitorFsmDb.SampleCounter = 0;
  measureVmicSourceSelectMonitorFsmDb.ValidSamplesCounter = 0;
  measureVmicSourceSelectMonitorFsmDb.UnsavedSamples = 0;
  measureVmicSourceSelectMonitorFsmDb.WaitTicksCounter = 0;
  measureVmicSourceSelectMonitorFsmDb.RetryCounter = 0;
  measureVmicSourceSelectMonitorFsmDb.PslPercentageIncrease = PslPercentageIncrease;
  return(MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_MEASURE);
}

  volatile float SampledAverage = 0;
measureVmicSourceSelectMonitorFsmState_T measureVmicSourceSelectMonitorFsm(uint32_t Sample, uint16_t SamplesToTest)
{
  char PrintBuff[100];
  
  if(!(measureVmicSourceSelectMonitorFsmDb.ValidSamplesCounter % 50))
  {
    sprintf(PrintBuff, " Index %d Sample %d, State %d\n", measureVmicSourceSelectMonitorFsmDb.ValidSamplesCounter, Sample, measureVmicSourceSelectMonitorFsmDb.State);
    vlapmainDebugLog(PrintBuff);
  }
  switch(measureVmicSourceSelectMonitorFsmDb.State)
  {
  case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_IDLE:
    measureVmicSourceSelectMonitorFsmDb.UnsavedSamples++;
    // Check Sample is within required range
    if( (Sample > measureVmicSourceSelectMonitorFsmDb.LimitEntry.MinValue) && (Sample < measureVmicSourceSelectMonitorFsmDb.LimitEntry.MaxValue) )
      measureVmicSourceSelectMonitorFsmDb.SampleCounter++;
    else 
    {
      measureVmicSourceSelectMonitorFsmDb.SampleCounter = 0;
    }

    measureVmicSourceSelectMonitorFsmDb.WaitTicksCounter++;
    // All samples are out of range -> try re-selecting
    if(measureVmicSourceSelectMonitorFsmDb.WaitTicksCounter > 10 && measureVmicSourceSelectMonitorFsmDb.SampleCounter == 0)
    {
        vlapmainDebugLog("Select monitor FSM, Reselect VMIC source\n");
        sprintf(PrintBuff, " Reselect VMIC Source %d , c2F %d\n", measureVmicSourceSelectMonitorFsmDb.VmicSourceSelectForRetry,  measureVmicSourceSelectMonitorFsmDb.VmicSourceSelectC2fSwitchArrayForRetry);
        vlapmainDebugLog(PrintBuff);

        if( measureVmicSourceSelectMonitorFsmDb.RetryCounter < 3)
        {
          // If psl percentage is set - increase psl by given percentage
          if (measureVmicSourceSelectMonitorFsmDb.PslPercentageIncrease > 0)
          {
              uint8_t addPsl = (uint8_t)round(CurrrentPskLevel * measureVmicSourceSelectMonitorFsmDb.PslPercentageIncrease);
              if(addPsl == 0 || CurrrentPskLevel < 85)
              {
                addPsl = 1;
              }
              // Increase by mid level PSL
              if(measureIncreaseAndSetPsl(addPsl) != RETURNCODE_OK)
              {
                measureIdleStateChange(10, 1);
                break;
              }
          }

          vlapmainDebugLog("Select monitor FSM, Retry VMIC source select\n");
          vmicmodemSelect(measureVmicSourceSelectMonitorFsmDb.VmicSourceSelectForRetry,  measureVmicSourceSelectMonitorFsmDb.VmicSourceSelectC2fSwitchArrayForRetry);
          measureVmicSourceSelectMonitorFsmDb.State = MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_WAIT;
          measureVmicSourceSelectMonitorFsmDb.RetryCounter++;
          measureVmicSourceSelectMonitorFsmDb.WaitTicksCounter = 0;
          measureVmicSourceSelectMonitorFsmDb.ValidSamplesCounter = 0;
        }
        else
        {
          vlapmainDebugLog("Select monitor FSM, Fail, Could not switch source\n");
          MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_SWITCH_SOURCE;
          measureVmicSourceSelectMonitorFsmDb.State = MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_FAIL;
        }
    }
    else
    {
        if(measureVmicSourceSelectMonitorFsmDb.SampleCounter >= SamplesToTest)
        {
            measureVmicSourceSelectMonitorFsmDb.SampleCounter = 0;
            // If started recording -> reset retries to clear channel switch failures
            measureVmicSourceSelectMonitorFsmDb.RetryCounter = 0;
            measureVmicSourceSelectMonitorFsmDb.State = MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_MEASURE;
        }
    }
    break;
  case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_WAIT:
    measureVmicSourceSelectMonitorFsmDb.WaitTicksCounter++;
    if(measureVmicSourceSelectMonitorFsmDb.WaitTicksCounter > 100)
    {
      measureVmicSourceSelectMonitorFsmDb.SampleCounter = 0;
      measureVmicSourceSelectMonitorFsmDb.WaitTicksCounter = 0;
      vlapmainDebugLog("Select monitor FSM, Timeout, Retry selection\n");
      measureVmicSourceSelectMonitorFsmDb.State = MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_IDLE;
    }
    break;
  case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_FAIL:
    break;
  case MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_MEASURE:
    // Check all samples are still in range
    if((Sample > measureVmicSourceSelectMonitorFsmDb.LimitEntry.MinValue) && (Sample < measureVmicSourceSelectMonitorFsmDb.LimitEntry.MaxValue))
    {
      measureVmicSourceSelectMonitorFsmDb.ValidSamplesCounter++;
    }
    else
    {
      // Try to solve range issue by waiting at idle
      measureVmicSourceSelectMonitorFsmDb.SampleCounter = 0;
      measureVmicSourceSelectMonitorFsmDb.WaitTicksCounter = 0;
      measureVmicSourceSelectMonitorFsmDb.UnsavedSamples++;
      measureVmicSourceSelectMonitorFsmDb.State = MEASURE_VMIC_SOURCE_SELECT_MONITOR_STATE_IDLE;
      sprintf(PrintBuff, "Out-of-bounds sample - %d", Sample);
      vlapmainDebugLog(PrintBuff);
    }
    break;
  }
  return(measureVmicSourceSelectMonitorFsmDb.State);
}       


void measureTimeoutTimerCallback()
{
  uint8_t sendEndMeasureEvent = 0;
  if(MeasureFsmState > MEASURE_MINIMAL_ENERGY_WAIT)
  {
    sendEndMeasureEvent = 1;
  }
  
  MeasurementsResults.MeasurementFailureReason = MEASURE_FAILURE_TOTAL_TIMEOUT; 
  eventsEventWrite(0,0,PROTOCOLAPP_MEASUREMENT_ABORTED_TIMEOUT_EVENT,0,0,0);
  measureIdleStateChange(100, sendEndMeasureEvent);
}

uint16_t StabilityIterationTimeGet()
{
	if (CurrrentPskLevel < 75)
		return MEASURE_STABILITY_ITERATION_TIME;
	else
		return MEASURE_STABILITY_SLOW_ITERATION_TIME;
}

uint16_t MinimalEnergyIterationTimeGet()
{
	if (CurrrentPskLevel < 75)
		return MEASURE_MINIMIAL_ENERGY_ITERATION_TIME_FAST;
	else
		return MEASURE_MINIMIAL_ENERGY_ITERATION_TIME_SLOW;
}

uint16_t measureSamplesReducedBySeqNumber()
{
  // In seqeunce 109 we remove the last 4 arrays in the seqeunce
  if(configConfigurationDb.MeasurementSequence == 109)
    return (PROTOCOLAPP_GENERAL_ARRAY_SIZE * 4);
  else 
    return 0;
}

void measureEndMeasurementSeq()
{
  // Stop measurement timeout timer
  xTimerStop(measureTimeoutTimerHandler, 0);
  
  // Save measurement end time
  measureStopTime = rtcEpochGet();
  
  vlapmainDebugLog("PUBLISH RESULTS");
  //Mark that measure ends.
//  ledsPatternSet(LEDS_MAIN_PATTERN_SUCCESS);
//  buzzerRequestToQueueAdd(BUZZER_BEEPING_OPTION2);
//  vibratorRequestToQueueAdd(VIBRATOR_OPTION2);
  autoresonanceControl(AUTORESONANCE_CONTROL_OFF);
  // Stop capturing P2P of accelerometer
  sensorStopCaptureP2PAcc();
  MEASURE_FSM_STATE_CHANGE(MEASURE_PUBLISH_RESULTS_WAIT, 100);
}

ReturnCode_T measureProgressLedsSequence(MeasureFsmState_T newState)
{
  switch(newState)
  {
  case MEASURE_DELAY_BEFORE_START:
//    ledsPatternSet(LEDS_MAIN_PATTERN_PROGRESS_SLOW);
    break;
  case MEASURE_MINIMAL_ENERGY_T127_SOURCE_SELECT_SAMPLE_WAIT:
    break;
  case MEASURE_VMIC_INTERNAL_CAP_1_MEASURE_WAIT:
    break;
  case MEASURE_VMIC_REAL_MEMS_WAIT:
//    ledsPatternSet(LEDS_MAIN_PATTERN_PROGRESS_RAPID);
    break;
  case MEASURE_TEMPERATURE_2_MEASUREMENT_WAIT:
    break;
  default:
  	break;

  }
  return RETURNCODE_OK;
}



void measurementEndedEventSimulateCallBAck()
{
	eventsEventWrite(0, 0, PROTOCOLAPP_MEASUREMENT_ENDED_EVENT, (char*)&MeasurementsResults, sizeof(ProtocolappMeasurementEndedEvent_t) - measureSamplesReducedBySeqNumber(), 0);
}
