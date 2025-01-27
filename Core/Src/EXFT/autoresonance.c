#include "autoresonance.h"
#include "hwdrivers.h"
#include "portmacro.h"
#include "vlapconfig.h"
#include "timers.h"
#include "pccommAppLayer.h"
#include "main.h"


// D E F I N I T I O N S 
#define MIN_RECOVERY_TIME_AFTER_TX_RESTART              150
#define AUTO_RESONANCE_STATUS_IDLE_DELAY                5
//#define RELAY_LATCH_TIME                                200
#define AUTORESONANCE_DELAYED_TURNON_1MSEC_CYCLES       2


// P R O T Y P E S 

// The change is anautonomous FSM, the time for he relays to change their state is 2mSec (Relay latch enable)
ReturnCode_T hwdriversCapacitorBankChange(uint16_t RelaysBitMap);
// Sets the Relays bank to 0
ReturnCode_T hwdriversCapacitorBankInit(uint16_t RelaysBitMap);
uint16_t  autoresonanceRelaysCapaticanceGet();
uint8_t  autoresonanceMinimalRelaysStateGet();
uint16_t  autoresonanceRelaysCapaticanceGet();
ReturnCode_T autoresonanceIsIdle();
uint8_t  autoresonanceMinimalRelaysStateGet();
void autoresonanceRelaySet(uint16_t RelayState);
void autoresonanceActivationTimerSchedule( uint16_t Time);
void autoresonanceActivationTimerInterruptCallback();

// Use this function to reflects the belt connection state
ReturnCode_T EvnetCreate();
ReturnCode_T autoresonanceStateChange(AutoresonanceState_T NewState);
ReturnCode_T autoresonanceIsIdle();






// G L O B A L S 
AutoresonanceState_T autoresonanceState;
uint16_t        LastRelayGpioBitMap;
uint8_t         RelaysExcitationCycles;
uint8_t         NumberOfCyclesBeforeGoingBackToIdle;
AutoresonanceControl_T  AutoresonanceControlState = AUTORESONANCE_CONTROL_OFF;
uint16_t        ManualRelaysState = 0;
uint16_t        AutoresonanceDelayCounter;
const  uint16_t  RelayCapacitanceWightArray[]= {17, 35, 70, 133, 253, 500, 1000, 2000};
hardwaredriversPhaseStatus_T  PhaseStatus = HWDRIVERS_PHASE_UNDER;

uint8_t SuccessiveCounter[AUTORESONANCE_STATE_LAST];

uint16_t autoresonanceMinValue;
uint16_t autoresonanceMaxValue;
// Relays bank acceleration process

// The resonance process startes 150mSec after transmission signal starts ( device start, or end of transmission to the VMIC)
// End of Resonance adaptation is sent 5mSec after the resonance process stops.




// Outputs
// Register Bank
// Errors: Belt Disconnected, Belt over falded 

/******************************************************************************
* @brief  ReturnCode_T autoresonanceUpdateP2P(uint16_t newValue)
* @param  
* @retval 
******************************************************************************/
ReturnCode_T autoresonanceUpdateP2P(uint16_t newValue)
{
  if(newValue < autoresonanceMinValue)
  {
    autoresonanceMinValue = newValue;
    return (RETURNCODE_OK);
  }
  
  if(newValue > autoresonanceMaxValue)
  {
    autoresonanceMaxValue = newValue;
    return (RETURNCODE_OK);
  }
  
  return (RETURNCODE_OK);
}

/******************************************************************************
* @brief  void autoresonanceInit(void)
* @param  
* @retval 
******************************************************************************/
ReturnCode_T autoresonanceInit(AutoresonanceInitRequest_T InitRequestType)
{
  autoresonanceState = AUTORESONANCE_STATE_IDLE;
  LastRelayGpioBitMap = 2; 
  /* 
     Setting LastRelayGpioBitMap to 2 prevents an edge case where after a reset where the RS was high and created an over-phase (RS=255 for example).
     After this reset if LastRelayGpioBitMap defaults to 1 - the fsm will sense an over-phase and try decremnting the RS 
     but it will be minimum and the FSM will get stuck.
     Setting LastRelayGpioBitMap to 2 forces the FSM to sync the RS in this case by allowing increment / decrement on the first try. 
  */
  autoresonanceUpdateP2P(LastRelayGpioBitMap);
  // Manual control vars init
  ManualRelaysState = 0;

  AutoresonanceControlState =  AUTORESONANCE_CONTROL_ON;
  
  xTimerStart(xTimerCreate("ares", // Just a text name, not used by the kernel.
                           8,      // The timer period in ticks = 1 ms
                           pdTRUE, // The timers will auto-reload themselves when they expire.
                           (void *)0,
                           autoresonanceFsmGlue
                           ), 0);
  
  // Initial Relays state after powerup only
  if(InitRequestType == AUTORESONANCE_INIT_REQ_AFTER_POWERUP)
    autoresonanceManaualRelaysSet(AUTORESONANCE_INITIAL_RELAY_STATE);
  
  return(RETURNCODE_OK);
}



/******************************************************************************
* @brief  autoresonanceFsm(), Called every 1mSec
* @param  
* @retval 
******************************************************************************/
void autoresonanceFsmGlue(void *v)
{
  autoresonanceFsm();
    
}


/******************************************************************************
* @brief  autoresonanceFsm(), Called every 1mSec
* @param  
* @retval 
******************************************************************************/
ReturnCode_T autoresonanceFsm(void)
{
  ReturnCode_T CompareResult;
  uint16_t RelaysDeltaStep = 0;
  uint16_t Frequency = 0;
  uint16_t MinimalRelaysState;
  // Read the input phase status 
  PhaseStatus = hwdriversPhaseSenseGet(PhaseStatus);
  //Update the phase status  	   
  pccpmmAppLayerStruct.Board1SystemRegisters.Debug0L = PhaseStatus;
  pccpmmAppLayerStruct.Board1SystemRegisters.Debug0M = 0;

  
  // The Minimal Relays state is function of the belt used.
  MinimalRelaysState = autoresonanceMinimalRelaysStateGet();
  
#if 0
  //  TODO: Autoresonance freq. monitoring not used as feedback for now.
      // It is assumed the autoresonance is in its stable state.
      // Look for phase change or frequency offset
  
      hwdriversFrequencyMonitoringGet(&Frequency);

      CompareResult = autoresonanceFrequencyCompareGet();
      switch(CompareResult)
      {
      case RETURNCODE_EQUAL:
        // Frequency ok, We use the phase error
        break;
      case RETURNCODE_GREATER_THAN:
        // Override the phase error as we have frequency error
        PhaseStatus = HWDRIVERS_PHASE_OVER;
        break;
      case RETURNCODE_LESS_THAN:
          // Override the phase error as we have frequency error
          PhaseStatus = HWDRIVERS_PHASE_UNDER;
          break;
      case RETURNCODE_OK:
    	  break;
      default:
    	  break;
      }

#endif
  
  if((AutoresonanceControlState == AUTORESONANCE_CONTROL_ON) && (!vmicmodemSendingDataToVmicInProgress()) )
  {
  // Check the new phase status with respect to the last known autoresonance state, Each state can process
  switch(autoresonanceState)
  {
  case AUTORESONANCE_STATE_DELAYED_ON:
    // This state implements delayed activation of the autoresonance mechanism- this delay is used after we return back to data reception after we transmit towards the VMIC
    if(AutoresonanceDelayCounter)
      AutoresonanceDelayCounter--;
    else
      autoresonanceState = AUTORESONANCE_STATE_IDLE;
    break;
    case AUTORESONANCE_STATE_IDLE:
   		// It is assumed the autoresonance is in its stable state.
      	// Look for phase change or frequency offset
		switch(PhaseStatus)
	      {
	      case HWDRIVERS_PHASE_OVER:
	        // Change state to Incrementing
	    	  AutoresonanceDelayCounter++;
	    	  if(AutoresonanceDelayCounter > 2 )
	    		  StateChange(AUTORESONANCE_STATE_DECREMENTING);
	        break;
	      case HWDRIVERS_PHASE_ZERO:
          AutoresonanceDelayCounter = 0;
	        break;
	      case HWDRIVERS_PHASE_UNDER:
	        // Change state to Decrementing
	    	  AutoresonanceDelayCounter++;
	    	  if(AutoresonanceDelayCounter > 2 )
	    		  StateChange(AUTORESONANCE_STATE_INCREMENTING);
	        break;
	      }
      break;
    case AUTORESONANCE_STATE_INCREMENTING:
      switch(PhaseStatus)
      {
      case HWDRIVERS_PHASE_UNDER:
        if(LastRelayGpioBitMap < AUTORESONANCE_MAX_RELAYS_VAL)
        {
          // Determine the required correction step according to the successive iterations counter
          StepGearShift(autoresonanceState, &RelaysDeltaStep);
          // Saturate the relays values if required
          if((LastRelayGpioBitMap+RelaysDeltaStep) < AUTORESONANCE_MAX_RELAYS_VAL)
          {
            LastRelayGpioBitMap += RelaysDeltaStep;
            autoresonanceUpdateP2P(LastRelayGpioBitMap);
          }          
          else
          {
            LastRelayGpioBitMap = AUTORESONANCE_MAX_RELAYS_VAL;
            autoresonanceUpdateP2P(LastRelayGpioBitMap);
          }
          vdbg("AUTOINC Freq= %d, RelayBitMap=%d\n", Frequency, LastRelayGpioBitMap);
          // Activate the mechanical relays change-over excitation 
          autoresonanceRelaySet(LastRelayGpioBitMap);
          // move to relays activation state and return back to this state afte the action completes 
          StateChange(AUTORESONANCE_STATE_INCREMENTING_RELAYS_ACTIVATION);
        }
  	  else
  		  StateChange(AUTORESONANCE_STATE_IDLE);

        break;
      case HWDRIVERS_PHASE_ZERO:
    	  if(NumberOfCyclesBeforeGoingBackToIdle)
    		  NumberOfCyclesBeforeGoingBackToIdle--;
    	  else
    		  StateChange(AUTORESONANCE_STATE_IDLE);
        break;
      case HWDRIVERS_PHASE_OVER:
         //Move from Incrementing to Decrementing state while clearing the successive counters
         StateChange(AUTORESONANCE_STATE_DECREMENTING);
        break;
      }
      break;
    case AUTORESONANCE_STATE_INCREMENTING_RELAYS_ACTIVATION:
    	StateChange(AUTORESONANCE_STATE_INCREMENTING);
    break;
    
    case AUTORESONANCE_STATE_DECREMENTING:
      switch(PhaseStatus)
      {
      case HWDRIVERS_PHASE_UNDER:
        // Move from Decrementing to Incrementing state while clearing the successive counters
        StateChange(AUTORESONANCE_STATE_INCREMENTING);
        break;
      case HWDRIVERS_PHASE_ZERO:
    	  if(NumberOfCyclesBeforeGoingBackToIdle)
    		  NumberOfCyclesBeforeGoingBackToIdle--;
    	  else
    		  StateChange(AUTORESONANCE_STATE_IDLE);
        break;
      case HWDRIVERS_PHASE_OVER:
        if(LastRelayGpioBitMap > MinimalRelaysState)
        {
          // Determine the required correction step according to the successive iterations counter
          StepGearShift(autoresonanceState, &RelaysDeltaStep);
          // Saturate the relays values if required
          if((LastRelayGpioBitMap-RelaysDeltaStep) > MinimalRelaysState)
          {
            LastRelayGpioBitMap -= RelaysDeltaStep;
            autoresonanceUpdateP2P(LastRelayGpioBitMap);
          }            
          else
          {            
            LastRelayGpioBitMap = MinimalRelaysState;
            autoresonanceUpdateP2P(LastRelayGpioBitMap);
          }
          vdbg("AUTODEC Freq= %d, RelayBitMap=%d\n", Frequency, LastRelayGpioBitMap);
          autoresonanceRelaySet(LastRelayGpioBitMap);
          StateChange(AUTORESONANCE_STATE_DECREMENTING_RELAYS_ACTIVATION);
        }
  	  else
  		  StateChange(AUTORESONANCE_STATE_IDLE);
        break;
      }
      break;
    case AUTORESONANCE_STATE_DECREMENTING_RELAYS_ACTIVATION:
		  StateChange(AUTORESONANCE_STATE_DECREMENTING);
		  break;
    default:
    	break;
    }
//   return(RETURNCODE_OK);
  }
  
  

  
    
#if 0  
  else
  {
  // Manual Resonance Control Mode
  switch(autoresonanceState)
  {
    case AUTORESONANCE_STATE_IDLE:
       if(ManaualRelayWriteRequestFlag)
       {
        // Activate the mechanical relays change-over excitation 
        hwdriversRelaysChangeActivate(ManualRelaysState);
        LastRelayGpioBitMap = ManualRelaysState;
        autoresonanceUpdateP2P(LastRelayGpioBitMap);
        // move to relays activation state and return back to this state afte the action completes 
        // The state change function also sets the relays excitation time cycles
        StateChange(AUTORESONANCE_STATE_INCREMENTING_RELAYS_ACTIVATION);
       }
        break;
     case AUTORESONANCE_STATE_INCREMENTING_RELAYS_ACTIVATION:
      if(!( RelaysExcitationCycles--))
      {
        // Stop relays excitation 
        hwdriversRelaysChangeIdle();
        // Turn of the request flag
        ManaualRelayWriteRequestFlag = FALSE;
        // Move back to the IDLE state
        StateChange(AUTORESONANCE_STATE_IDLE);
      }
    break;
   }
  }
  
#endif   
  return(RETURNCODE_OK);  
} 


/******************************************************************************
* @brief  autoresonanceIsIdle()
* @param  
* @retval 
******************************************************************************/
ReturnCode_T autoresonanceIsIdle()
{
  if(autoresonanceState == AUTORESONANCE_STATE_IDLE)
    return(RETURNCODE_OK);
  else
    return(RETURNCODE_ERROR);
}



/******************************************************************************
* @brief  ReturnCode_T autoresonanceStateChange(AutoresonanceState_T NewState)
* @param  
* @retval 
******************************************************************************/
ReturnCode_T StateChange(AutoresonanceState_T NewState)
{
  switch(NewState)
  {
  case AUTORESONANCE_STATE_IDLE:
    SuccessiveCounter[AUTORESONANCE_STATE_INCREMENTING] = 0;
    SuccessiveCounter[AUTORESONANCE_STATE_DECREMENTING] = 0;
    break;
  case AUTORESONANCE_STATE_INCREMENTING:
    SuccessiveCounter[AUTORESONANCE_STATE_DECREMENTING] = 0;
    NumberOfCyclesBeforeGoingBackToIdle = AUTORESONANCE_BACK_TO_IDLE_CYCLES;
    break;
  case AUTORESONANCE_STATE_DECREMENTING:
    SuccessiveCounter[AUTORESONANCE_STATE_INCREMENTING] = 0;
    NumberOfCyclesBeforeGoingBackToIdle = AUTORESONANCE_BACK_TO_IDLE_CYCLES;
    break;
  case AUTORESONANCE_STATE_INCREMENTING_RELAYS_ACTIVATION:
  case AUTORESONANCE_STATE_DECREMENTING_RELAYS_ACTIVATION:
    // Moving to relays activation state, sets the delay as this is a momentary state
    RelaysExcitationCycles = AUTORESONANCE_RELAYS_EXCITATION_DELAY;
    break;
    
  default:
    // No action taken, returns error
    return(RETURNCODE_ERROR);
  }
  // Change the FSM state to the new state
  autoresonanceState = NewState;
  // 
  return(RETURNCODE_OK);
}


/******************************************************************************
* @brief  ReturnCode_T StepGearShift(AutoresonanceState_T State, uint16_t* ReturnedRelayStepPtr)
* @param  
* @retval 
******************************************************************************/
ReturnCode_T StepGearShift(AutoresonanceState_T State, uint16_t* ReturnedRelayStepPtr)
{
  uint16_t MyReturnedStep = 0;
  ReturnCode_T MyReturnCode;
  // Returns the relays step change according to the state's successive counter value
  // Relevant for both incrmeent and decrement directions
    switch(State)
    {
    case AUTORESONANCE_STATE_INCREMENTING:
    case AUTORESONANCE_STATE_DECREMENTING:
      SuccessiveCounter[State]++;
      if(SuccessiveCounter[State] <= AUTORESONANCE_DUCCESSIVE_CNT_THR_A)
        MyReturnedStep = 1;
      else
      {
        if(SuccessiveCounter[State] <= AUTORESONANCE_DUCCESSIVE_CNT_THR_B)
          MyReturnedStep = 2;
        else
        {
          if(SuccessiveCounter[State] <= AUTORESONANCE_DUCCESSIVE_CNT_THR_C)
            MyReturnedStep = 8;
          else
            MyReturnedStep = 10;
        }
      }
      MyReturnCode = RETURNCODE_OK;
      break;
    case AUTORESONANCE_STATE_IDLE:
      SuccessiveCounter[State] = 0;
      MyReturnCode = RETURNCODE_OK;
      break;
    default:
      MyReturnCode = RETURNCODE_ERROR;
    }
  // Update the Step via Ptr
  if(MyReturnCode == RETURNCODE_OK)
    *ReturnedRelayStepPtr = MyReturnedStep;

  return(MyReturnCode);
}


/******************************************************************************
* @brief  
* @param  
* @retval 
******************************************************************************/
ReturnCode_T autoresonanceControl(AutoresonanceControl_T ControlByte)
{
  
  
  ReturnCode_T MyReturnCode = RETURNCODE_OK;
  // Switch over the Control byte values;
  switch (ControlByte)
  {
  case AUTORESONANCE_CONTROL_OFF:
      // Reflect the new state to the GUI
    pccpmmAppLayerStruct.BeltTransmitterRegisters.AutoResonanceCtrl = FALSE;
    AutoresonanceControlState = AUTORESONANCE_CONTROL_OFF;
    autoresonanceState=AUTORESONANCE_STATE_IDLE;
    MyReturnCode = RETURNCODE_OK;
    break;
  case AUTORESONANCE_CONTROL_ON:
    //TODO:
#if 1
    pccpmmAppLayerStruct.BeltTransmitterRegisters.AutoResonanceCtrl = TRUE;
    AutoresonanceControlState = AUTORESONANCE_CONTROL_ON;
    autoresonanceState=AUTORESONANCE_STATE_DELAYED_ON;
    AutoresonanceDelayCounter = AUTORESONANCE_DELAYED_TURNON_1MSEC_CYCLES;
    MyReturnCode = RETURNCODE_OK;
    autoresonanceManaualRelaysSet(AUTORESONANCE_INITIAL_RELAY_STATE);
    LastRelayGpioBitMap = AUTORESONANCE_INITIAL_RELAY_STATE;

//    LastRelayGpioBitMap = 1;
//    autoresonanceUpdateP2P(LastRelayGpioBitMap);
#endif
    break;
case AUTORESONANCE_CONTROL_DELAYED_ON:
#if 1
    pccpmmAppLayerStruct.BeltTransmitterRegisters.AutoResonanceCtrl = TRUE;
    AutoresonanceControlState = AUTORESONANCE_CONTROL_ON;
    autoresonanceState = AUTORESONANCE_STATE_DELAYED_ON;
    AutoresonanceDelayCounter = AUTORESONANCE_DELAYED_TURNON_1MSEC_CYCLES;
    autoresonanceManaualRelaysSet(AUTORESONANCE_INITIAL_RELAY_STATE);
    LastRelayGpioBitMap = AUTORESONANCE_INITIAL_RELAY_STATE;

#endif
    MyReturnCode = RETURNCODE_OK;
    break;
  default:
    MyReturnCode = RETURNCODE_ERROR;
  }
  return(MyReturnCode);
}


/******************************************************************************
* @brief  
* @param  
* @retval 
******************************************************************************/
ReturnCode_T autoresonanceManaualRelaysSet(uint16_t RelaysState)
{
  // Relevant only when the Autoresonance is disabled
//  if(AutoresonanceControlState == AUTORESONANCE_CONTROL_OFF)
  if((ManualRelaysState != RelaysState) || (LastRelayGpioBitMap != RelaysState))
  {
    // Sets the new Relays combination 
    ManualRelaysState = RelaysState;
    
    autoresonanceRelaySet(ManualRelaysState);
    
    pccpmmAppLayerStruct.BeltTransmitterRegisters.RelayStateMsb = (uint8_t)(RelaysState>>8);
    pccpmmAppLayerStruct.BeltTransmitterRegisters.RelayStateLsb = (uint8_t)RelaysState;

    return(RETURNCODE_OK);
  }
  else
    return(RETURNCODE_ERROR);

}


/******************************************************************************
* @brief  
* @param  
* @retval 
******************************************************************************/
uint16_t  autoresonanceRelaysStateGet()
{
  return(LastRelayGpioBitMap);
}


/******************************************************************************
* @brief  
* @param  
* @retval 
******************************************************************************/
uint16_t  autoresonanceRelaysCapaticanceGet()
{
  uint8_t i;
  //uint16_t Capacitance=1680;
  uint16_t Capacitance=2000; // picoF
  
  // Calculate the relay bank capacitors based on the relative capacitance wight of each relay 
  for(i=0; i<sizeof(RelayCapacitanceWightArray)/sizeof(uint16_t); i++)
          Capacitance += ((LastRelayGpioBitMap>>i) & 0x01) * RelayCapacitanceWightArray[i];
  // Update the GUI  
  pccpmmAppLayerStruct.BeltTransmitterRegisters.CapacitorBank = TYPES_ENDIAN16_CHANGE(Capacitance); 

  // Return the capacitance value  
  return(Capacitance);
}



/******************************************************************************
* @brief  
* @param  
* @retval 
******************************************************************************/
uint8_t  autoresonanceMinimalRelaysStateGet()
{
  uint8_t ReturnedMinimalRelaysState;
  // Returns the minimal relays state according to the configured belt size
//  switch(pccpmmAppLayerStruct.BeltTransmitterRegisters.BeltSize)
  switch(configConfigurationDb.BeltSize)
  {
  case 2:
      ReturnedMinimalRelaysState = AUTORESONANCE_2_SEGNEMTS_BELT_MIN_RELALY;
    break;
  case 3:
      ReturnedMinimalRelaysState = AUTORESONANCE_3_SEGNEMTS_BELT_MIN_RELALY;
    break;
  case 4:
      ReturnedMinimalRelaysState = AUTORESONANCE_4_SEGNEMTS_BELT_MIN_RELALY;
    break;
  default:
      ReturnedMinimalRelaysState = AUTORESONANCE_3_SEGNEMTS_BELT_MIN_RELALY;
 
  }
  pccpmmAppLayerStruct.BeltTransmitterRegisters.BeltSize = configConfigurationDb.BeltSize;
  
  return(ReturnedMinimalRelaysState);
}




/******************************************************************************
* @brief  ReturnCode_T  autoresonanceFrequencyCompareGet()
* @param  
* @retval 
******************************************************************************/
ReturnCode_T  autoresonanceFrequencyCompareGet()
{
  ReturnCode_T MyReturnCode;
  uint16_t ReturnedMeasuredFrequency;
  // Get the measured frequency
  hwdriversFrequencyMonitoringGet(&ReturnedMeasuredFrequency);

  
  // Compare the measured frequency with center frequency +- delta of 2% (all in Khz)
  if(ReturnedMeasuredFrequency >  (HSE_VALUE/(4*1000) + AUTORESONANCE_ALLOWED_FREQUENCY_MEASURED_DELTA) )
      MyReturnCode = RETURNCODE_GREATER_THAN;
  else
    if(ReturnedMeasuredFrequency <  (HSE_VALUE/(4*1000) - AUTORESONANCE_ALLOWED_FREQUENCY_MEASURED_DELTA) )
      MyReturnCode = RETURNCODE_LESS_THAN;
  else
      MyReturnCode = RETURNCODE_EQUAL;
  
  return(MyReturnCode);
}

/******************************************************************************
* @brief  ReturnCode_T autoresonanceResetP2P()
* @param  
* @retval 
******************************************************************************/
ReturnCode_T autoresonanceResetP2P()
{
  autoresonanceMaxValue = LastRelayGpioBitMap;
  autoresonanceMinValue = LastRelayGpioBitMap;
  return (RETURNCODE_OK);
}

/******************************************************************************
* @brief  ReturnCode_T  autoresonanceP2PGet()
* @param  
* @retval 
******************************************************************************/
uint16_t autoresonanceP2PGet()
{
  return (autoresonanceMaxValue - autoresonanceMinValue);
}




void autoresonanceRelaySet(uint16_t RelayState)
{
	  hwdriversRelaysChangeActivate(RelayState);
	  autoresonanceActivationTimerSchedule(36000);
}



void autoresonanceActivationTimerSchedule( uint16_t Time)
{
	__HAL_TIM_ENABLE(&htim7);
	__HAL_TIM_SET_AUTORELOAD(&htim7, Time);
	__HAL_TIM_ENABLE_IT(&htim7, TIM_IT_UPDATE);
}

void autoresonanceActivationTimerInterruptCallback()
{
	__HAL_TIM_DISABLE(&htim7);
	hwdriversRelaysChangeIdle();
//	LastRelayGpioBitMap = ManualRelaysState;
//	autoresonanceUpdateP2P(LastRelayGpioBitMap);
}
