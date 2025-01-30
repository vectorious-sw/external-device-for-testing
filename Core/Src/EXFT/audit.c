//#include <hwdrivers.h>
#include <stdlib.h>
#include <math.h>
//#include <arm_math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "vlapConfig.h"
#include "queue.h"
#include "timers.h"
#include "audit.h"
#include "iir.h"
#include "config.h"
#include "vlapConfig.h"
#include "config.h"
#include "vmicmodem.h"
#include "pccommapplayer.h"
#include "measure.h"
#include "autopower.h"
#include "hwdrivers.h"

void auditInit(void);

static void auditLifeCycle(TimerHandle_t pxTimer);


#define AUDIT_AMBIANT_TEMPERATURE_HIGH_THRESAHOLD  600
#define AUDIT_AMBIANT_TEMPERATURE_CHANGE_TRIES  60
#define AUDIT_BATTERY_EVENT_NUMBERS 15
#define AUDIT_LIFE_CYCLE_TIME 10
//#define TEST_MODE_INJECTION 1

auditTemperatureState_T auditAbiantTemperatureStateGet();
auditTemperatureState_T auditAbiantTemperatureState = AUDIT_TEMPERATURE_NORMAL;
uint8_t auditAmbiantTemperatureCounter = 0;

auditBvState_T bvState = AUDIT_BV_OK;

typedef enum { AUDIT_PRE_CONFIGURED_STATE = 0, AUDIT_CONFIGURED_STATE = 1, AUDIT_LEDS_START_WAIT = 2, AUDIT_LEDS_START = 3, AUDIT_FINISH_CONFIG = 4 }  auditConfigurationState_T;

auditConfigurationState_T auditConfigurationState;

AuditBatteryLedState_T auditBatteryLedState = AUDIT_BATTERY_RED_OFF;

static uint8_t auditNurseModeMessage[2] = "NM";
uint8_t auditNurseModeBuffer[2] = {0};
uint8_t auditNurseModeBufferIndex = 0;
AuditNurseModeBufferState_T auditNurseModeBufferState = AUDIT_NURSE_BUFFER_FIRST_BYTE;
const uint16_t AuditNurseModeReceiveTries = 150;
const uint16_t AuditNurseModeReceiveAckTries = 20;
uint16_t AuditNurseModeReceiveTriesCounter = 0;
uint16_t AuditNurseModeReceiveAckCounter = 0;
uint8_t AuditNurseModeReceiveAck = 0;

uint32_t AuditNurseModePluggedInTime = 0;
uint32_t AuditNurseModePluggedOutTime = 0;

uint8_t AuditNewNurseModePlugIn = 1;

uint8_t AuditLowBatteryEventCounter = 0;
uint8_t AuditHighBatteryEventCounter = 0;
uint8_t AuditNormalBatteryEventCounter = 0;

uint8_t AuditBatteryReturnedNormal = 0;

uint16_t AuditBeltHumanCheckCounter = 0;
AuditBeltHumanCheck_T AuditBeltHumanCheckState = AUDIT_BELT_HUMAN_CHECK_IDLE;
AuditBeltHumanCheck_T TempBeltStatus = AUDIT_BELT_HUMAN_CHECK_IDLE;

AuditNurseModeStatus_T AuditNurseModeState = AUDIT_NURSE_MODE_DISABLED;

#ifdef TEST_MODE_INJECTION
uint32_t counterBla = 0;
uint32_t TestEnable = 0;
#endif

/******************************************************************************
* @brief void auditInit(void) 
* @param  
* @retval 
******************************************************************************/
void auditInit(void)
{

auditAbiantTemperatureState = AUDIT_TEMPERATURE_NORMAL;


 auditConfigurationState = AUDIT_PRE_CONFIGURED_STATE;
 AuditNurseModeReceiveAckCounter = AuditNurseModeReceiveAckTries;
  // Init the events handler
//  eventsInit(); // tbd
  // Start the periodic timer callback every 10 timer ticks. 
  xTimerStart(xTimerCreate("acheck", AUDIT_LIFE_CYCLE_TIME, pdTRUE, (void *)0, auditLifeCycle), 0);
}


/******************************************************************************
* @brief static void auditBatteryLevelCheck()
* @param  
* @retval 
******************************************************************************/
ReturnCode_T auditBatteryLevelCheck()
{
  //uint8_t PrintBuff[100];
  //Get upper and lower threshold from config.
  uint16_t hw = configConfigurationDb.VoltageHysteresisWidth;
  uint16_t ub = configConfigurationDb.VbattUpperThreshold;
  uint16_t lb = configConfigurationDb.VbattLowerThreshold;
  //uint16_t bv;
  AuditBatteryLedState_T newbatteryLedState = auditBatteryLedState;
  //Get battery voltage.
  uint16_t bv = chargerBattVoltageGet();
  
  
 //   sprintf(PrintBuff, "Batt Voltage = %d", bv);
 //   vlapmainDebugLog(PrintBuff);
 
  
  switch (bvState)
  {
  case AUDIT_BV_OK:
    if (bv > ub + hw)
    {
      bvState = AUDIT_BV_CROSSED_UB;
      AuditHighBatteryEventCounter = 1;
    }
    else if (bv < lb - hw)
    {
      bvState = AUDIT_BV_CROSSED_LB;
      AuditLowBatteryEventCounter = 1;
    }
    else
    {    
      newbatteryLedState = AUDIT_BATTERY_RED_OFF;
      
      AuditNormalBatteryEventCounter++;
      //Check that battery returned to ok state and clean noise with AuditNormalBatteryEventCounter.
      if (AuditBatteryReturnedNormal && (AuditNormalBatteryEventCounter >= AUDIT_BATTERY_EVENT_NUMBERS))
      {
        AuditBatteryReturnedNormal = 0;
        eventsEventWrite(0, 0, PROTOCOLAPP_POWER_BATT_NORMAL_EVENT, 0, 0, 0); 
      } 
      else if (!AuditBatteryReturnedNormal)
      {
        AuditNormalBatteryEventCounter = 0;
      }
      // nothing - bv in guarded zone
    }
    break;
    
  case AUDIT_BV_CROSSED_UB:
    if (bv > ub - hw)
    {
      AuditHighBatteryEventCounter++;
      
      if(AuditHighBatteryEventCounter >= AUDIT_BATTERY_EVENT_NUMBERS)
      {
        newbatteryLedState = AUDIT_BATTERY_RED_OFF;
        eventsEventWrite(0,0,PROTOCOLAPP_POWER_BATT_HIGH_EVENT,0,0, 0);
        AuditHighBatteryEventCounter = 0;
        bvState = AUDIT_BV_CROSSED_UB_SENDED;
      }
    }
    else 
    {
      bvState = AUDIT_BV_OK;
    }
    break;
    
  case AUDIT_BV_CROSSED_UB_SENDED:
    if (bv <= ub - hw)
    {
      AuditBatteryReturnedNormal = 1;
      bvState = AUDIT_BV_OK;
    }
    break;
    
  case AUDIT_BV_CROSSED_LB:
    if (bv < lb + hw)
    {
      AuditLowBatteryEventCounter++;
      
      if(AuditLowBatteryEventCounter >= AUDIT_BATTERY_EVENT_NUMBERS)
      {
        newbatteryLedState = AUDIT_BATTERY_RED_ON;
        eventsEventWrite(0,0,PROTOCOLAPP_POWER_BATT_LOW_EVENT,0,0, 0);
        // If the battery is low, start saving the current Lom memory pointers to the system section in the spi
        //eventsLogMemoryPointersToNvmSavePVD();
        AuditLowBatteryEventCounter = 0;
        bvState = AUDIT_BV_CROSSED_LB_SENDED;
      }
    }
    else 
    {
      bvState = AUDIT_BV_OK;
      newbatteryLedState = AUDIT_BATTERY_RED_OFF;
    }
    break;
    
  case AUDIT_BV_CROSSED_LB_SENDED:
    if (bv >= lb + hw)
    {
      AuditBatteryReturnedNormal = 1;
      newbatteryLedState = AUDIT_BATTERY_RED_OFF;
      bvState = AUDIT_BV_OK;
    }
    break;
  }   

  return(RETURNCODE_OK);
}

// Use this for version 1038, increase by 240,000
//const uint32_t auditDebugPlaceHolder[60000] = { 0 };

// Use this for version 1039, increase by 123133
//const uint8_t auditDebugPlaceHolder[123133] = { 0 };


uint32_t parameterWrite = 8;

uint16_t toggle =30000;
//extern ProtocolappMeasurementEndedEvent_t MeasurementsResults; 
/******************************************************************************
* @brief static void auditLifeCycle(TimerHandle_t pxTimer)
* @param  
* @retval 
******************************************************************************/
static void auditLifeCycle(TimerHandle_t pxTimer)
{  
  // Block the timer callback till configuration is ready
  switch(auditConfigurationState)
  {
  case AUDIT_PRE_CONFIGURED_STATE:
    xSemaphoreTake( configConfigurationValidSemaphoreHandle, portMAX_DELAY );
    xSemaphoreGive( configConfigurationValidSemaphoreHandle);
//    ledsInit();
    auditConfigurationState = AUDIT_FINISH_CONFIG;
    break;
  case AUDIT_FINISH_CONFIG:
    break;
  default:
    break;
  }

  auditTxTemperatureOverheatCheck();
  auditPowerTelemetryUpdate();

  hwdriversTxProcessing();
//  if(chargerFinishedUpdating)
//  {
//    auditBatteryLevelCheck();
//	  // Charger processing
//	  chargerProcessing();
//	  chargerDcPlugStateProcessing();
//  }
    
  auditBeltHumanStatusProcessing();
  
  auditAmbiantTemperatueProcessing();
  
  
  // Add this for version 1038 and 1039
//  if(counterBla < 1000)
//  {
//    toggle++;
//    toggle = auditDebugPlaceHolder[counterBla];
//    counterBla++;
//  }
//  else
//  {
//    toggle = 4;
//    counterBla = 0;
//  }
  
#ifdef TEST_MODE_INJECTION

if(TestEnable)
  {
    counterBla++;
    if((counterBla%1000) == 0)
// 	if((counterBla%150) == 0)
    {
      //eventsEventWrite(0, 0, PROTOCOLAPP_MEASUREMENT_ABORTED_SHORT_PUSH_BUTTON_EVENT, (char*)0, 0, 0);
// 		hwdriversGpioBitWrite(HWDRIVERS_PB14_USB_TEST, 1);
 		measurementEndedEventSimulateCallBAck();
// 		hwdriversGpioBitWrite(HWDRIVERS_PB14_USB_TEST, 0);
    }
  }
#endif
  
  
  //gpioTableActivate(ALL_GPIO_PINS);

  
  // ************************** Nurse Mode ********************************
//  if (!AuditNurseModeReceiveTriesCounter)
//  {
//    AuditNurseModeReceiveTriesCounter = AuditNurseModeReceiveTries;
//#if COMM_IS_BLE
//    uartdllTxQueueEnqueue(UARTDLL_TX_QUEUE_UART_2_CELLMODEM, auditNurseModeMessage, sizeof(auditNurseModeMessage), false);  
//#endif 
//    AuditNurseModeReceiveAck = 0;
//  }
//  else
//  {
//    AuditNurseModeReceiveTriesCounter--;
//    if(!AuditNurseModeReceiveAck)
//    {
//      AuditNurseModeReceiveAckCounter--;
//      if(!AuditNurseModeReceiveAckCounter)
//      {
//        if(!AuditNewNurseModePlugIn)
//        {
//          AuditNurseModePluggedOutTime = rtcEpochGet();
//          AuditNewNurseModePlugIn = 1;
//        }
//        
//        AuditNurseModeReceiveAckCounter = AuditNurseModeReceiveAckTries;
//        AuditNurseModeState = AUDIT_NURSE_MODE_DISABLED;
//      }
//    }
//  }
  // ************************** Nurse Mode ********************************
}
uint16_t temp;
ReturnCode_T auditAmbiantTemperatueProcessing()
{
  uint16_t newTemp = 0;
  temp = 0;
  auditTemperatureState_T newAuditAmbiantTemperatureState;
  
  // Check new level
  if( newTemp > AUDIT_AMBIANT_TEMPERATURE_HIGH_THRESAHOLD)
    newAuditAmbiantTemperatureState = AUDIT_TEMPERATURE_HIGH;
  else
    newAuditAmbiantTemperatureState = AUDIT_TEMPERATURE_NORMAL;
  
  
  switch(auditAbiantTemperatureState)
  {
  case AUDIT_TEMPERATURE_NORMAL:
    if(newAuditAmbiantTemperatureState != AUDIT_TEMPERATURE_NORMAL)
    {
      auditAmbiantTemperatureCounter++;
      if(auditAmbiantTemperatureCounter >= AUDIT_AMBIANT_TEMPERATURE_CHANGE_TRIES)
      {
        auditAbiantTemperatureState = AUDIT_TEMPERATURE_HIGH;
        auditAmbiantTemperatureCounter = 0;
      }
    }
    else
    {
      auditAmbiantTemperatureCounter = 0;
    }
    
    break;
    
  case AUDIT_TEMPERATURE_HIGH:
    if(newAuditAmbiantTemperatureState != AUDIT_TEMPERATURE_HIGH)
    {
      auditAmbiantTemperatureCounter++;
      if(auditAmbiantTemperatureCounter >= AUDIT_AMBIANT_TEMPERATURE_CHANGE_TRIES)
      {
        auditAbiantTemperatureState = AUDIT_TEMPERATURE_NORMAL;
        auditAmbiantTemperatureCounter = 0;
      }
    }
    else
    {
      auditAmbiantTemperatureCounter = 0;
    }
     
    break;
  }

  return(RETURNCODE_OK);
}



volatile   AuditBeltHumanCheck_T auditBeltHumanCheckStateNew;

  uint16_t current = 0; 
ReturnCode_T auditBeltHumanStatusProcessing()
{ 
  // Get belt current.
  current = autopowerBeltCurrentMilliAmpGet();
  uint16_t CurrentRS = hwdriversRelayStateGet();
  
  // We assume the current measurement is stable as it is filtered 
  if(current < 1300)  // Power is too low
    if(CurrentRS == 255 || CurrentRS == 1) // Auto resonance didn't work & power is too low = open belt
      TempBeltStatus = AUDIT_BELT_OPEN;
    else
      TempBeltStatus = AUDIT_BELT_HUMAN_OK; // Power is low but in resonance = belt exists.
  else if(current < 1500)   // Belt is closed and there is human.
    TempBeltStatus = AUDIT_BELT_HUMAN_OK;
  else                     // Belt is closed and there is no human.
    TempBeltStatus = AUDIT_BELT_NO_HUMAN;
  
  if (pccpmmAppLayerStruct.BeltTransmitterRegisters.TransmitterEnable && vmicmodemOpenBeltMaskingFSM(TempBeltStatus))
    AuditBeltHumanCheckState = TempBeltStatus;
  
#if 0  
  switch(AuditBeltHumanCheckState)
  {
  case AUDIT_BELT_HUMAN_CHECK_IDLE:
    AuditBeltHumanCheckCounter++;
    AuditBeltHumanCheckState = auditBeltHumanCheckStateNew;    
    break;
  case AUDIT_BELT_OPEN_HYS:
    // Check if new state is also BELT_OPEN_HIS and increase hysteresis
    if (auditBeltHumanCheckStateNew == AUDIT_BELT_OPEN_HYS)
    {
      AuditBeltHumanCheckCounter++;
      
      // Check if counter is up to max tries and change state
      if (AuditBeltHumanCheckCounter == AUDIT_BELT_HUMAN_CHECK_TRIES)
      {
        AuditBeltHumanCheckCounter = 0;
        AuditBeltHumanCheckState = AUDIT_BELT_OPEN;
      }      
    }
    else // If not, go to new state and start to count there
    {
      AuditBeltHumanCheckCounter = 1;
      AuditBeltHumanCheckState = auditBeltHumanCheckStateNew;  
    }

    break;
  case AUDIT_BELT_OPEN:
    // Check if new state indicates on change
    if (auditBeltHumanCheckStateNew != AUDIT_BELT_OPEN_HYS)
    {
      AuditBeltHumanCheckCounter++;
      
      // Check if we can change state
      if (AuditBeltHumanCheckCounter == AUDIT_BELT_HUMAN_CHECK_TRIES)
      {
        AuditBeltHumanCheckCounter = 0;
        AuditBeltHumanCheckState = auditBeltHumanCheckStateNew;  
      }
    }
    else
    {
      AuditBeltHumanCheckCounter = 0;
    }
    break;
  case AUDIT_NO_HUMAN_HYS:
    // Check if new state is also NO_HUMAN_HIS and increase hysteresis
    if (auditBeltHumanCheckStateNew == AUDIT_NO_HUMAN_HYS)
    {
      AuditBeltHumanCheckCounter++;
      
      // Check if counter is up to max tries and change state
      if (AuditBeltHumanCheckCounter == AUDIT_BELT_HUMAN_CHECK_TRIES)
      {
        AuditBeltHumanCheckCounter = 0;
        AuditBeltHumanCheckState = AUDIT_NO_HUMAN;
      }      
    }
    else // If not, go to new state and start to count there
    {
      AuditBeltHumanCheckCounter = 1;
      AuditBeltHumanCheckState = auditBeltHumanCheckStateNew;  
    }
    
    break;
  case AUDIT_NO_HUMAN:
    // Check if new state indicates on change
    if (auditBeltHumanCheckStateNew != AUDIT_NO_HUMAN_HYS)
    {
      AuditBeltHumanCheckCounter++;
      
      // Check if we can change state
      if (AuditBeltHumanCheckCounter == AUDIT_BELT_HUMAN_CHECK_TRIES)
      {
        AuditBeltHumanCheckCounter = 0;
        AuditBeltHumanCheckState = auditBeltHumanCheckStateNew;  
      }
    }
    else
    {
      AuditBeltHumanCheckCounter = 0;
    }
    break;
  case AUDIT_BELT_HUMAN_OK_HYS:
    // Check if new state is also HUMAN_OK_HIS and increase hysteresis
    if (auditBeltHumanCheckStateNew == AUDIT_BELT_HUMAN_OK_HYS)
    {
      AuditBeltHumanCheckCounter++;
      
      // Check if counter is up to max tries and change state
      if (AuditBeltHumanCheckCounter == AUDIT_BELT_HUMAN_CHECK_TRIES)
      {
        AuditBeltHumanCheckCounter = 0;
        AuditBeltHumanCheckState = AUDIT_BELT_HUMAN_OK;
      }      
    }
    else // If not, go to new state and start to count there
    {
      AuditBeltHumanCheckCounter = 1;
      AuditBeltHumanCheckState = auditBeltHumanCheckStateNew;  
    }
   
    break;
  case AUDIT_BELT_HUMAN_OK:
    // Check if new state indicates on change
    if (auditBeltHumanCheckStateNew != AUDIT_BELT_HUMAN_OK_HYS)
    {
      AuditBeltHumanCheckCounter++;
      
      // Check if we can change state
      if (AuditBeltHumanCheckCounter == AUDIT_BELT_HUMAN_CHECK_TRIES)
      {
        AuditBeltHumanCheckCounter = 0;
        AuditBeltHumanCheckState = auditBeltHumanCheckStateNew;  
      }
    }
    else
    {
      AuditBeltHumanCheckCounter = 0;
    }
    break;
  }
 
#endif  
  
  return(RETURNCODE_OK);
}

AuditBeltHumanCheck_T auditBeltHumanStatusGet()
{
  return (AuditBeltHumanCheckState);
}

auditTemperatureState_T auditTxTemperatureOverheatCheck()
{
  int16_t ReturnedNtcTemperature;
 
  hwdriversNtcTemperatureGet(&ReturnedNtcTemperature);
  pccpmmAppLayerStruct.Board1SystemRegisters.NtcPosL = ReturnedNtcTemperature;
  pccpmmAppLayerStruct.Board1SystemRegisters.NtcPosM = ReturnedNtcTemperature>>8;
  
  if (ReturnedNtcTemperature >= MAX_TEMPERATURE)
  {
    return(AUDIT_TEMPERATURE_HIGH);
  }

  return(AUDIT_TEMPERATURE_NORMAL);
}

void auditPowerTelemetryUpdate()
{
  int16_t ReturnedCurrent;
  int16_t ReturnedVoltage;
  hwdriversTxPowerGet(&ReturnedCurrent, &ReturnedVoltage);
  pccpmmAppLayerStruct.Board1SystemRegisters.NtcNegL = ReturnedCurrent;
  pccpmmAppLayerStruct.Board1SystemRegisters.NtcNegM = ReturnedCurrent>>8;

  pccpmmAppLayerStruct.BoardSystemRegisters.SupplyVoltage = ReturnedVoltage;
}


// TODO: Function name not according to coding conventions,  Change
// TODO: Enum not according to coding conventions, Change
//isUSBConnected_t getUSBConnectionStatus()
//{
//  uint8_t numberOfLoops = 5;
//  uint16_t dcinVoltage;
//  isUSBConnected_t MyReturn;
//  
//  // If PC GUI is polling the VLAP we will avoid checking for USB disconnected state for LAB tests, We assume the in the field 
//  // the charger PowerSupply will have only power signals 
//  if(pccommnapplayerConnectivityStateGet() != PCCOMMAPPLAYER_CONNECTIVITY_STATE_CONNECTED)
//  {    
//    // Assume connected
//    MyReturn = USB_CONNECTED;
//    // Confirm voltage is under the minimal threshold for repeated samples
//    for(uint8_t i = 0; i < numberOfLoops; i++)
//    {
//      hwdriversExtDcVoltageGet(&dcinVoltage);
//      if (dcinVoltage < USB_MIN_VOLTAGE)
//      {
//        MyReturn = USB_DISCONNECTED;
//      }  
//    }
//  }
//  else
//    MyReturn = USB_DISCONNECTED;
//      
//  return(MyReturn);
//}
//
//
//
 auditBvState_T auditBatteryStatusGet()
 {
   return(bvState);
 }


auditTemperatureState_T auditAbiantTemperatureStateGet()
{
  return auditAbiantTemperatureState;
}

ReturnCode_T auditManageBufferNurseMode(uint8_t newByte)
{
  // Skip this function to disable nurse mode
  return(RETURNCODE_OK);
  
  uint8_t i;
  switch(auditNurseModeBufferState)
  {
  case AUDIT_NURSE_BUFFER_FIRST_BYTE:
    //if (auditNurseModeMessage[auditNurseModeBufferIndex] != newByte)
    if (auditNurseModeMessage[0] != newByte)
    {
      auditNurseModeBufferState = AUDIT_NURSE_BUFFER_FIRST_BYTE;
    }
    else
    {
      auditNurseModeBuffer[auditNurseModeBufferIndex] = newByte;
      auditNurseModeBufferState = AUDIT_NURSE_BUFFER_REST_BYTES;
      auditNurseModeBufferIndex++;
    }
    break;
  case AUDIT_NURSE_BUFFER_REST_BYTES:
    auditNurseModeBuffer[auditNurseModeBufferIndex] = newByte;
    auditNurseModeBufferIndex++;
    if (auditNurseModeBufferIndex >= sizeof(auditNurseModeMessage))
    {
      auditNurseModeBufferIndex = 0;
      auditNurseModeBufferState = AUDIT_NURSE_BUFFER_FIRST_BYTE;
      for(i = 0; i < sizeof(auditNurseModeMessage); i++)
      {
        if (auditNurseModeBuffer[i] !=  auditNurseModeMessage[i])
        {
          AuditNurseModeState = AUDIT_NURSE_MODE_DISABLED;
          return(RETURNCODE_OK);
        }
        auditNurseModeBuffer[i] = 0;
      }
      
      if(AuditNewNurseModePlugIn)
      {
        AuditNurseModePluggedInTime = rtcEpochGet();
        AuditNewNurseModePlugIn = 0;
      }
      
      AuditNurseModeReceiveAck = 1;
      AuditNurseModeState = AUDIT_NURSE_MODE_ENABLED;
    }
    break;
  }
  return(RETURNCODE_OK);
}

AuditNurseModeStatus_T auditNurseModeStatusGet()
{
  return (AuditNurseModeState);
}

uint32_t auditNurseModeLastPluggedOutTimeGet()
{
  return AuditNurseModePluggedOutTime;
}
uint32_t auditNurseModeLastPluggedInTimeGet()
{
  return AuditNurseModePluggedInTime;
}
