#include "autopower.h"
//#include "hwdrivers.h "
#include "vlapconfig.h"
#include "timers.h"
#include "autopowerPwmLut.h"
#include "pccommAppLayer.h"
#include "math.h"
#include "stdlib.h"
#include "autoresonance.h"



// D E F I N I T I O N S 
typedef struct
{
  uint16_t      PwmPeriod;
  uint16_t      PwmCapture;
} PwmState_T;

#define AUTOPOWER_MAX_LUT_INDEX (sizeof(autopowerPwdLut) / sizeof(PwmLutEntry_T))-1
#define AUTOPOWER_CURRENT_CHNAGE_DELTA 300
/// For Belt resistance calculation period of 5 Hz (250/5)
#define AUTOPOWER_BELT_RESISTANCE_PERIOD_COUNTS 50
// The Over current protection is called at rate of 62 Hz (250/4)
#define AUTOPOWER_OVER_CURRENT_PROTECTION_PERIOD_COUNTS 4
// The autopower FSM is called at rate of 31 Hz (250/8)
#define AUTOPOWER_FSM_COUNTS  8      
// Max Current -  1mA per unit
#define AUTOPOWER_MAX_CURRENT_SET                       9000
// Transmission_overcurrent_protection - 1mA per unit
#define AUTOPOWER_TRANSMISSION_OVERCURRENT_PROTECTION   10500
// Current_error_threshold - 1mA per unit
#define AUTOPOWER_CURRENT_ERROR_THRESHOLD               200
// Max_Effective_belt_resistance - 0.001ohm per unit
#define MAX_EFFECTIVE_BELT_RESISTANCE		        40000
// Max_Pulse_skip_level - 235 is 100% duty cycle
#define MAX_PULSE_SKIP_LEVEL				220
//
#define MAX_PULSE_SKIP_LEVEL_NO_CONNECTION	        8

#define AUTOPOWER_PSL_STEP      1


// Dc2dc power estimation 
typedef enum {AUTOPOWER_TX_POWER_MONITORING_STIMULI_PROCESS, AUTOPOWER_TX_POWER_MONITORING_STIMULI_PSL_UPDATE} autopowerTxPowerMonitoringStimuli_T;
typedef enum {AUTOPOWER_TX_POWER_MONITORING_STATE_IDLE, AUTOPOWER_TX_POWER_MONITORING_STATE_PROCESS, AUTOPOWER_TX_POWER_MONITORING_STATE_BYPASS_TRIGGERED, AUTOPOWER_TX_POWER_MONITORING_STATE_BYPASS, AUTOPOWER_TX_POWER_MONITORING_STATE_BYPASS_OFF_WAIT} autopowerTxPowerMonitoringState_T;
#define AUTOPOWER_TX_POWER_MONITORING_DROP_ALERT_THRESHOLD  1000              // 1 Watt
#define AUTOPOWER_TX_POWER_MONITORING_POWER_FILTER_BETA  80                  
#define AUTOPOWER_TX_POWER_MONITORING_PRINT_DECIMATION_FACTOR   1                  
#define AUTOPOWER_POWER_ESTIMATION_CYCLES                       20


// L O C A L   P R O T Y P E S 
ReturnCode_T LutBinarySearch(uint16_t PwmKey, PwmState_T* ReturnedPwmStatePtr);
void autopowerGsmGlue(void *v);
ReturnCode_T autopowerEffectiveBeltResistanceCalc();
void autopowerFsmGlue(void *v);
ReturnCode_T autopowerFsm();
void autopowerRealTimeXmtrCurrentMeasure(float TxMonitorNewSmaple, float Adc2CurrentOffset);
ReturnCode_T autopowerOverCurrentProtection();
ReturnCode_T autopowerBeltStatusUpdate();
void autopowerTxPowerMonitoringInit();
uint32_t autopoweTxPowerMonitoringFsm(autopowerTxPowerMonitoringStimuli_T Stimuli, uint32_t NewPsl, uint32_t CurrentPsl);
ReturnCode_T autopowerDc2DcDacControl(uint8_t NewPsl);


// G L O B A L S 
typesControl_T  AutopowerControlState;
uint16_t        AutopowerManualPwmPeriod;
uint16_t        AutopowerManualPwmCapture;
uint8_t         AutopowerLutIndex = 5;
uint16_t        RealTimeXmtrCurrent;
uint16_t        TransmitterCurrentGoal;
uint8_t         PulseSkipLevel;
uint8_t         EffectiveBeltResistancePeriodCounter;
uint8_t         AutoPowerFsmPeriodCounter;
uint8_t         OverCurrentProtectionPeriodCounter;
int             FilteredRealTimeXmtrCurrent;
AutopowerBeltStatusT  autopowerBeltStatus;
uint16_t        autopowerEffectiveBeltResistance;

autopowerTxPowerMonitoringState_T  AutopowerTxPowerMonitoringState;
uint8_t autopowerTxPowerMonitoringPrintDecimator;
uint32_t AutopowerDc2dcPowerDropThreshold;
uint32_t AutopowerFilteredDc2dcPowerMw;
uint16_t EstimationCycleCounter;
uint32_t EstimatedPower;
uint32_t AverageEstimatedPower;
uint32_t LastEstimatedPower;
uint32_t TargetPsl;
uint32_t AverageDc2dcVoltage;
uint32_t AverageDc2dcVoltageAccumulator;

uint8_t RfDc2DcBypassEnable;
uint32_t UpdatePsl;


 
uint8_t AutopowerTxMonitoringPrintBuffer[100];
uint8_t Dc2DcFailure;


uint16_t ReturnedDc2dcCurrentMa;


/******************************************************************************
* @brief  void autopowerInit(void)
* @param  
* @retval 
******************************************************************************/
ReturnCode_T autopowerInit()
{

  PwmLutEntry_T PwmLutEntry;

  AutopowerManualPwmPeriod = 0;
  AutopowerManualPwmCapture = 0;
  RealTimeXmtrCurrent=0;
  EffectiveBeltResistancePeriodCounter          = AUTOPOWER_BELT_RESISTANCE_PERIOD_COUNTS;
  AutoPowerFsmPeriodCounter                     = AUTOPOWER_FSM_COUNTS;
  OverCurrentProtectionPeriodCounter            = AUTOPOWER_OVER_CURRENT_PROTECTION_PERIOD_COUNTS;
  RealTimeXmtrCurrent=0;
  FilteredRealTimeXmtrCurrent=0;


  AverageDc2dcVoltage = 0;
  AverageDc2dcVoltageAccumulator = 0;

  
  AutopowerControlState = TYPES_DISABLE; //TYPES_ENABLE;
  // Default current goal 
  TransmitterCurrentGoal = 1000;
  pccpmmAppLayerStruct.BeltTransmitterRegisters.TransmitterCurrentSet = TYPES_ENDIAN16_CHANGE(TransmitterCurrentGoal);
  // Default Lut
  //AutopowerLutIndex = 5;
  // Get the new PWM parameters from the LUT
  PwmLutEntry = autopowerPwdLut[AutopowerLutIndex];
  // Update the GUI
  pccpmmAppLayerStruct.BeltTransmitterRegisters.PulseSkipLevel = PulseSkipLevel = AutopowerLutIndex;
  // Write the new PWM values to the timer
  hwdriversPwmSet(true ,PwmLutEntry.Period, PwmLutEntry.Capture);
  // Manual control vars init
  AutopowerManualPwmPeriod = PwmLutEntry.Period;
  AutopowerManualPwmCapture = PwmLutEntry.Capture;

  
  
  // 
//  hwdriversPwmControl(TYPES_DISABLE);
  
  // Init global vars
  AutopowerControlState = TYPES_DISABLE;
  RealTimeXmtrCurrent = 0;
  TransmitterCurrentGoal = 0;

  RfDc2DcBypassEnable = 0;

  
  // process the Autopower FSM
  xTimerStart(xTimerCreate("AutoPowerFsm", // Just a text name, not used by the kernel.
                           40,      // 
                           pdTRUE, // The timers will auto-reload themselves when they expire.
                           (void *)0,
                           autopowerFsmGlue
                           ), 0);
  
 ReturnedDc2dcCurrentMa = 0;

  
 
  
  return(RETURNCODE_OK);
}


/******************************************************************************
* @brief  autopowerFsmGlue(), Called 250 times per second
* @param  
* @retval 
******************************************************************************/
void autopowerFsmGlue(void *v)
{
  uint16_t ReturnedTxMonitorLevel;
  if(pccpmmAppLayerStruct.BeltTransmitterRegisters.TransmitterEnable)
  {
    // Get the Tx monitoring value
    hwdriversTxMonitorGet(&ReturnedTxMonitorLevel);
    // Real Time Transmission Current monitor is called at rate of 250 Hz
    autopowerRealTimeXmtrCurrentMeasure((float)ReturnedTxMonitorLevel, HWDRIVERS_TX_MONITORING_INPUT_OFFSET);
    
    // Over Current Protection, called at rate of 62 Hz
//     if(OverCurrentProtectionPeriodCounter)
//       OverCurrentProtectionPeriodCounter--;
//     else
//     {
//       OverCurrentProtectionPeriodCounter = AUTOPOWER_OVER_CURRENT_PROTECTION_PERIOD_COUNTS;
//  //     autopowerOverCurrentProtection();
//     }
   
//     // Auto Current FSM is called at rate of 31 Hz
//     if(AutoPowerFsmPeriodCounter)
//       AutoPowerFsmPeriodCounter--;
//     else
//     {
//       AutoPowerFsmPeriodCounter = AUTOPOWER_FSM_COUNTS;
//       autopowerFsm();
//       autopowerBeltStatusUpdate();
//     }
      
//     // Effective Belt Resistance is called at rate of 5 Hz
//     if(EffectiveBeltResistancePeriodCounter)
//       EffectiveBeltResistancePeriodCounter--;
//     else
//     {
//       EffectiveBeltResistancePeriodCounter = AUTOPOWER_BELT_RESISTANCE_PERIOD_COUNTS;
//       autopowerEffectiveBeltResistanceCalc();
//     }
  }
}


/******************************************************************************
* @brief  ReturnCode_T autopowerManaualPwmSet(uint8_t AutopowerLutNewIndex)
* @param  PwmVlaue: 0 - 1000
* @retval 
******************************************************************************/
ReturnCode_T autopowerManaualPwmSet(uint8_t AutopowerLutNewIndex)
{
  ReturnCode_T MyReturnCode = RETURNCODE_ERROR;

  autopowerDc2DcDacControl(AutopowerLutNewIndex);
  pccpmmAppLayerStruct.BeltTransmitterRegisters.PulseSkipLevel = AutopowerLutNewIndex;

  return(MyReturnCode);
}

/******************************************************************************
* @brief  ReturnCode_T autopowerDc2DcDacControl(uint8_t NewPsl)
* @param  
* @retval 
******************************************************************************/
ReturnCode_T autopowerDc2DcDacControl(uint8_t NewPsl)
{
  ReturnCode_T  MyReturnCode = RETURNCODE_OK;
  uint16_t analogWord;

  if (NewPsl>AUTOPOWER_MAX_PSL_STEPS)
    NewPsl = AUTOPOWER_MAX_PSL_STEPS;

  analogWord = 4095 - ((uint16_t)(NewPsl + AUTOPOWER_PSL_OFFSET) * (uint16_t)4095)/(uint16_t)AUTOPOWER_MAX_PSL_STEPS;
  dacWrite(analogWord);

  return(MyReturnCode);
}




/******************************************************************************
* @brief  uint16_t   autopowerRealTimeXmtrCurrentMeasure(float TxMonitorNewSmaple, float Adc2CurrentOffset)
* @param  
* @retval 
******************************************************************************/
void autopowerRealTimeXmtrCurrentMeasure(float TxMonitorNewSmaple, float Adc2CurrentOffset)
{
   float FixedAdcSmaple;
  
  // Fix the adc offset if relevant
  if(TxMonitorNewSmaple > Adc2CurrentOffset)
    FixedAdcSmaple = TxMonitorNewSmaple - Adc2CurrentOffset;
  else
    FixedAdcSmaple = 0;
  
  // Calculate the current ac( cording to the formula from: External Device Software Document: Section "Auto current control", page 24  Rev. 0.95
  // Update the global var.
  //RealTimeXmtrCurrent = (int)((AUTOPO*2WER_CURRENT_MON_FACTOR * FixedAdcSmaple)* autoresonanceRelaysCapaticanceGet() ) / 996;
  RealTimeXmtrCurrent = FixedAdcSmaple;
  //RealTimeXmtrCurrent = (int)(463400 /*1079617*/ * FixedAdcSmaple * autoresonanceRelaysCapaticanceGet() / 1000000000);
  // Filter the RealTimeXmtrCurrent only for stable display 
  FilteredRealTimeXmtrCurrent = (((int)RealTimeXmtrCurrent *10) + FilteredRealTimeXmtrCurrent *70)/80;
  // Update the GUI with the new transmitter power
  pccpmmAppLayerStruct.BeltTransmitterRegisters.TransmissionCurrentMonitor = TYPES_ENDIAN16_CHANGE(FilteredRealTimeXmtrCurrent);
}



uint16_t autopowerBeltCurrentMilliAmpGet()
{
  return(FilteredRealTimeXmtrCurrent);
}


/******************************************************************************
* @brief  ReturnCode_T autopowerFsm()
* @param  
* @retval 
******************************************************************************/
ReturnCode_T autopowerFsm()
{
  PwmLutEntry_T PwmLutEntry;

  
  // When the FSM is not enabled ( Manual mode) the PWM is written directly from termianl via autopowerManaualPwmSet(uint16_t PwmVlaue)
  if(AutopowerControlState == TYPES_ENABLE)
  {
  //Automatic power adaptation starts after the autoresonance reached stable state
    if(autoresonanceIsIdle() == RETURNCODE_OK)
    {
      // Compare the current current against the required current and decide if we need to increase or decrease the index
      if( (RealTimeXmtrCurrent > (TransmitterCurrentGoal+AUTOPOWER_CURRENT_CHNAGE_DELTA)) )        
      {
        if(AutopowerLutIndex)
          AutopowerLutIndex--;
      }
      else
        if((RealTimeXmtrCurrent <= (TransmitterCurrentGoal-AUTOPOWER_CURRENT_CHNAGE_DELTA)))
        {
          if(AutopowerLutIndex < AUTOPOWER_MAX_LUT_INDEX)
            AutopowerLutIndex++;
        }
        else
          return(RETURNCODE_OK);

      
      // Get the new PWM parameters from the LUT
      PwmLutEntry = autopowerPwdLut[AutopowerLutIndex];
      // Update the GUI
      pccpmmAppLayerStruct.BeltTransmitterRegisters.PulseSkipLevel = PulseSkipLevel = AutopowerLutIndex;
      // Write the new PWM values to the timer
      hwdriversPwmSet(true, PwmLutEntry.Period, PwmLutEntry.Capture);
      
      
      return(RETURNCODE_OK);
    }
  }
  else
    // Manual mode  
    return(RETURNCODE_ERROR);
  return(RETURNCODE_ERROR);

} 

    

/******************************************************************************
* @brief  ReturnCode_T autopowerControl(uint8_t ControlByte)
* @param  
* @retval 
******************************************************************************/
ReturnCode_T autopowerControl(uint8_t ControlByte)
{
  // Switch over the Control byte vlaues;
  switch (ControlByte)
  {
  case 0:
    AutopowerControlState = TYPES_DISABLE;
    break;
  case 1:
    AutopowerControlState = TYPES_ENABLE;
    break;
  }

  // Update the GUI status
  pccpmmAppLayerStruct.BeltTransmitterRegisters.AutoCurrentControl = AutopowerControlState;
  // Manual mode  
  return(RETURNCODE_ERROR);
}

        

/******************************************************************************
* @brief  ReturnCode_T autopowerTransmitterCurrentGoalSet(uint16_t NewTransmitterCurrentValue)
* @param  
* @retval 
******************************************************************************/
ReturnCode_T autopowerTransmitterCurrentGoalSet(uint16_t NewTransmitterCurrentGoalValue)
{
  // The lower current goal can't be less than the minimal current change delta
  if(NewTransmitterCurrentGoalValue < AUTOPOWER_CURRENT_CHNAGE_DELTA)
    NewTransmitterCurrentGoalValue = AUTOPOWER_CURRENT_CHNAGE_DELTA;
  // TODO: Add limit for MAX current
  // Sets the new current goal, The automatic current control will try to adjust the current to this new value
  TransmitterCurrentGoal = NewTransmitterCurrentGoalValue;
  // Update the GUI    
  pccpmmAppLayerStruct.BeltTransmitterRegisters.TransmitterCurrentSet = NewTransmitterCurrentGoalValue;
  // Manual mode  
  return(RETURNCODE_ERROR);
}


/******************************************************************************
* @brief  ReturnCode_T autopowerTransmitterCurrentGoalGet(uint16_t *TransmitterCurrentGoalPtr)
* @param  
* @retval 
******************************************************************************/
ReturnCode_T autopowerTransmitterCurrentGoalGet(uint16_t *TransmitterCurrentGoalPtr)
{
  // Returns the last value of the current goal 
  *TransmitterCurrentGoalPtr = TransmitterCurrentGoal;
  // Manual mode  
  return(RETURNCODE_ERROR);
}





/******************************************************************************
* @brief  ReturnCode_T autopowerPulseSkipLevelSet()
* @param  
* @retval 
******************************************************************************/
ReturnCode_T autopowerPulseSkipLevelSet( uint8_t PulseSkipLevel)
{
  ReturnCode_T MyReturnCode;
  
//  if(PulseSkipLevel <= AUTOPOWER_MAX_LUT_INDEX) 
  {
    // Update the GUI
    pccpmmAppLayerStruct.BeltTransmitterRegisters.PulseSkipLevel = AutopowerLutIndex = PulseSkipLevel;
    // Set the new skip pulse level
    autopowerManaualPwmSet(PulseSkipLevel);
    //
    MyReturnCode = RETURNCODE_OK;
  }
//  else
//       MyReturnCode = RETURNCODE_ERROR;
  return MyReturnCode;
}

/******************************************************************************
* @brief  ReturnCode_T autopowerPulseSkipLevelGet(uint8_t *ReturnedPulseSkipLevelPtr)
* @param  
* @retval 
******************************************************************************/
ReturnCode_T autopowerPulseSkipLevelGet(uint8_t *ReturnedPulseSkipLevelPtr)
{
    *ReturnedPulseSkipLevelPtr = PulseSkipLevel;
    return(RETURNCODE_OK);
}


/******************************************************************************
* @brief  ReturnCode_T autopowerManualModePulseSkipLevelIncrease()
* @param  
* @retval 
******************************************************************************/
ReturnCode_T autopowerManualModePulseSkipLevelIncrease()
{
  ReturnCode_T MyReturnCode;
  
  if(AutopowerLutIndex <= AUTOPOWER_MAX_LUT_INDEX)
  {
    // Increment the 
    AutopowerLutIndex++;
    // Update the GUI
    pccpmmAppLayerStruct.BeltTransmitterRegisters.PulseSkipLevel = PulseSkipLevel = AutopowerLutIndex;
    // Set the new skip pulse level
    autopowerManaualPwmSet(AutopowerLutIndex);
    //
    MyReturnCode = RETURNCODE_OK;
  }
  else
       MyReturnCode = RETURNCODE_ERROR;

  return(MyReturnCode);
}

  



/******************************************************************************
* @brief  uint8_t autopowerMaxLutIndexGet()
* @param  
* @retval 
******************************************************************************/
uint8_t autopowerMaxLutIndexGet()
{
  return (AUTOPOWER_MAX_LUT_INDEX);
}


/******************************************************************************
* @brief  uint8_16 autopowerRealTimeXmtrCurrentGet()
* @param  
* @retval 
******************************************************************************/
uint16_t autopowerRealTimeXmtrCurrentGet()
{
  return (RealTimeXmtrCurrent);
}


/******************************************************************************
* @brief  ReturnCode_T autopowerEffectiveBeltResistanceCalc()
* @param  
* @retval 
******************************************************************************/
ReturnCode_T autopowerEffectiveBeltResistanceCalc()
{
  uint16_t BatteryVoltageLevel;
  PwmLutEntry_T PwmLutEntry;
  float EffectiveBeltResistance;
  
  // Get the new PWM parameters from the LUT based on the last value sent to the GUI
  PwmLutEntry = autopowerPwdLut[pccpmmAppLayerStruct.BeltTransmitterRegisters.PulseSkipLevel];
  // Get the current voltage level (in mV)
  hwdriversVbatVoltageGet(&BatteryVoltageLevel);
  // Calc the Effective belt current into a float var 
  EffectiveBeltResistance = 1000 * BatteryVoltageLevel * PwmLutEntry.Capture / (PwmLutEntry.Period * autopowerRealTimeXmtrCurrentGet());
  // Update the global var
  autopowerEffectiveBeltResistance = (uint16_t) EffectiveBeltResistance;
  // Update the GUI 
  pccpmmAppLayerStruct.BeltTransmitterRegisters.EffectiveBeltResistance = TYPES_ENDIAN16_CHANGE(autopowerEffectiveBeltResistance);
  // Return OK
  return(RETURNCODE_OK);
}
    

/******************************************************************************
* @brief  ReturnCode_T autopowerOverCurrentProtection()
* @param  
* @retval 
******************************************************************************/
ReturnCode_T autopowerOverCurrentProtection()
{
  uint16_t TransmitterCurrentGoal;
  uint8_t ReturnedPulseSkipLevel;
  uint16_t ReturnedEffectiveBeltResistance;
  
  // Get the current Current goal as defined by the GUI
  autopowerTransmitterCurrentGoalGet(&TransmitterCurrentGoal);
  
  // Get the current pulse skip level
  autopowerPulseSkipLevelGet(&ReturnedPulseSkipLevel); 
    
  if(TransmitterCurrentGoal > AUTOPOWER_MAX_CURRENT_SET)
    autopowerTransmitterCurrentGoalSet(AUTOPOWER_MAX_CURRENT_SET);
  
  if(RealTimeXmtrCurrent > AUTOPOWER_TRANSMISSION_OVERCURRENT_PROTECTION)
      autopowerControl(TYPES_DISABLE);
  
  if(autopowerBeltStatus == AUTOPOWER_BELT_STATUS_NOT_CONNECTED)
      autopowerPulseSkipLevelSet(MAX_PULSE_SKIP_LEVEL_NO_CONNECTION);
  
  if(ReturnedPulseSkipLevel > MAX_PULSE_SKIP_LEVEL)
      autopowerPulseSkipLevelSet(MAX_PULSE_SKIP_LEVEL_NO_CONNECTION);
  // Get the effective belt resistance 
  autopowerEffectiveBeltResistanceGet(&ReturnedEffectiveBeltResistance);
  // Check that the effective belt resistance is less than the max
  if( ReturnedEffectiveBeltResistance > MAX_EFFECTIVE_BELT_RESISTANCE)
       autopowerControl(TYPES_DISABLE);
 
  
 
  // Return OK
  return(RETURNCODE_OK);
}


/******************************************************************************
* @brief  ReturnCode_T autopowerBeltStatusUpdate()
* @param  
* @retval 
******************************************************************************/
ReturnCode_T autopowerBeltStatusUpdate()
{
  uint16_t RelaysState;

  // Read the autoresonance relays state
  RelaysState = autoresonanceRelaysStateGet();
  // Calculate the belt circule value  
  float  BeltCirculeTemp = sqrt((RelaysState*128)-40);
  // Calc and convert to uint8_t 
  uint8_t BeltCirculeCode = (uint8_t)abs((int)((100-((abs((int)(BeltCirculeTemp-20))*BeltCirculeTemp/2)+(abs((int)(BeltCirculeTemp-40))*BeltCirculeTemp/2)))));
  // Update the GUI
  pccpmmAppLayerStruct.BeltTransmitterRegisters.BeltStatusForImage = BeltCirculeCode;
  
  
  // Figure out the Belt state
  if(pccpmmAppLayerStruct.BeltTransmitterRegisters.TransmitterEnable)
  {
    if(RelaysState == autoresonanceMinimalRelaysStateGet())
      // Not Connected
      autopowerBeltStatus = AUTOPOWER_BELT_STATUS_NOT_CONNECTED;
    else
    if(RelaysState == 255)
      // Over Folded
      autopowerBeltStatus = AUTOPOWER_BELT_STATUS_OVERFOLDED;
    else
      // Not On body (TBD)
      autopowerBeltStatus = AUTOPOWER_BELT_STATUS_NOT_ON_BODY;
  }
  else
  {
     // Connected   
     autopowerBeltStatus = AUTOPOWER_BELT_STATUS_CONNECTED;
  }
  // Update the GUI
  pccpmmAppLayerStruct.BeltTransmitterRegisters.BeltStatus = autopowerBeltStatus;
  // Return OK
  return(RETURNCODE_OK);
}
/******************************************************************************
* @brief  ReturnCode_T autopowerEffectiveBeltResistanceGet( uint16_t *ReturnedResistance)
* @param  
* @retval 
******************************************************************************/
ReturnCode_T autopowerEffectiveBeltResistanceGet( uint16_t *ReturnedResistance)
{
  *ReturnedResistance = autopowerEffectiveBeltResistance ;
  // Return OK
  return(RETURNCODE_OK);
}



/******************************************************************************
* @brief  ReturnCode_T autopowerPwmTemporaryControl(uint8_t Control)
* @param  
* @retval 
******************************************************************************/
ReturnCode_T autopowerPwmTemporaryControl(uint8_t Control)
{

  hwdriversGpioBitWrite(HWDRIVERS_PD13_EXT_TX_OUT_MODULATOR, Control);
  // Return OK
  return(RETURNCODE_OK);
}


