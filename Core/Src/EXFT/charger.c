#include "vlapConfig.h"
#include "charger.h"
#include "vlapmain.h"
#include "../common.h"
#include "i2cwork.h"


// M O D U L E   G L O B A L S
// Charging state filter vars
uint8_t chargerChangeLedIndicationCounter = 0;
LedsPatterns_T chargerLedsIndicationLevel  = LEDS_MAIN_PATTERN_IDLE;
LedsPatterns_T chargerPortLedsIndication = LEDS_PORT_PATTERN_IDLE;
chargerFaultState_T chargerFaultState = CHARGER_FAULT_NONE;
bool chargerFault = FALSE;
uint8_t chargerFaultSended = 0;
bool chargingFinishedSent = FALSE;
// DC_PLUG state vats
chargerPlugState_T chargerPlugState;
chargerIndicationState_T chargerIndicationState;
uint8_t DcPlugFilterCounter = 0;
// Charging Status
ChargerEnableStatus_T chargingEnableStatus;
chargerDcPlugStatus_T chargerPlugStatus;
chargerChargingState_T chargerChargingStatus = CHARGER_NOT_CHARGING;
// Config db
extern configConfigurationDb_t configConfigurationDb;

uint16_t chragerShowIndicationAbortDelay = 0;

uint8_t ReturnedI2cChargerBuffer[40];

QueueHandle_t chargerRequestQueueHandle;

chargerConfigStateT ChargerConfigState;

bool chargerFinishedUpdating;
bool chargerStartedUpdating;

bool USB_Phy_inited = false;


const uint8_t Bq25882RegPhase1Config[] =
{
		0xa0,	//00h R/W REG00 Battery Voltage Limit
		0x6c,	//01h R/W REG01 Charge Current Limit = 2.2A
		0x05,	//02h R/W REG02 Input Voltage Limit = 4.4V (default)
		0x39,	//03h R/W REG03 Input Current Limit = 3A input + ICO enabled.
		0x22,	//04h R/W REG04 Precharge and Termination Control
		0x8d,	//05h R/W REG05 Charger Control 1
		0x7D,	//06h R/W REG06 Charger Control 2 = D+/- detection ON
		0x0a,	//07h R/W REG07 Charger Control 3
		0x0d,	//08h R/W REG08 Charger Control 4
		0xf6	//09h R/W REG09 OTG Control
};

const uint8_t Bq25882RegPhase2Config[] =
{
		0xE8,	//12h R/W REG12 Charger Mask 1 = CHRG_STAT[2:0] trigger
		0x93,	//13h R/W REG13 Charger Mask 2 = TS_STAT[2:0] trigger
		0x00,	//14h R/W REG14 Fault Mask = trigger by all faults
		0xb0,	//15h R/W REG15 ADC Control
		0x00	//16h R/W REG16 ADC Function Disable
};


/******************************************************************************
* @brief  void chargerInit()
* @param  
* @retval 
******************************************************************************/
ReturnCode_T chargerInit(void)
{
  // By default the charger is enabled
  chargerControl(CHARGER_CONTROL_ENABLE);
  
  chargerPlugStatus = CHARGER_DC_PLUG_DETACHED;
  chargerPlugState = CHARGER_PLUG_DISCONNECTED;
  chargerIndicationState = CHARGER_INDICATION_NOT_CHARGING;
  
  chragerShowIndicationAbortDelay = CHARGER_PB_ABORT_SHOW_INDICATION_THRESHOLD;
  chargerRequestQueueHandle = xQueueCreate(5, sizeof(chargerReq_T));

  chargerFinishedUpdating = FALSE;
  chargerStartedUpdating = FALSE;
  
  ChargerConfigState = CHARGER_CONFIG_STATE0;
  // Return success
  return(RETURNCODE_OK);
}

/******************************************************************************
* @brief  ReturnCode_T chargerControl( ChargerControl_T ChargerControl)
* @param  
* @retval 
******************************************************************************/
ReturnCode_T chargerControl( ChargerControl_T ChargerControl)
{
    ReturnCode_T MyReturnCode = RETURNCODE_OK; 
  
    switch(ChargerControl)
    {
    case CHARGER_CONTROL_DISABLE:
      chargingEnableStatus = CHARGER_STATUS_DISABLED;
      hwdriversGpioBitWrite(HWDRIVERS_CHARGER_DISABLE, FALSE);
      break;
    case CHARGER_CONTROL_ENABLE:
      chargingEnableStatus = CHARGER_STATUS_ENABLED;
      hwdriversGpioBitWrite(HWDRIVERS_CHARGER_DISABLE, TRUE);
      break;
    default:
      MyReturnCode = RETURNCODE_UNSUPPORTED;
    }
      
    return(MyReturnCode);
}

/******************************************************************************
* @brief chargerChargingState_T chargerChargingStatusGet()  
* @param  
* @retval 
******************************************************************************/
chargerChargingState_T chargerChargingStatusGet()
{
   // Return Charger status
  return(chargerChargingStatus);
}

/******************************************************************************
* @brief Refresh the USB init state to re-initiated / deinitiate the USB Phy
* @param  None
* @retval None
******************************************************************************/
void chargerRefreshUSBInit()
{
  USB_Phy_inited = false; // The 'processing' method will re-init / de-init
}


/******************************************************************************
* @brief ReturnCode_T chargerProcessing()  
* @param  
* @retval 
******************************************************************************/
bool chargingDuringMeasFlag;
ReturnCode_T chargerProcessing()
{  
  uint8_t enableChargingWhilePolling = 1;
  if ((pccommnapplayerConnectivityStateGet() == PCCOMMAPPLAYER_CONNECTIVITY_STATE_CONNECTED) && !configConfigurationDb.EnableChargingWhilePolling)
  {
    enableChargingWhilePolling = 0;
  }

  chargerReq_T QueueEntry;
  // Get queue entry
  BaseType_t QueueState = xQueueReceive(chargerRequestQueueHandle, &QueueEntry, 0);

  switch(chargerIndicationState)
  {
  case CHARGER_INDICATION_NOT_CHARGING: 
    // Check that external is charging - plug is inserted, and no faults occured
    if((chargerPlugStatus == CHARGER_DC_PLUG_INSERTED) && chargerChargeStatusRegGet() && enableChargingWhilePolling
      && !chargerFaultFlagDetect())
    {
      // Set flag to 0 so in the next fault - message will be sended
      chargerFaultSended = 0; 
      chargerChargingStatus = CHARGER_CHARGING;
      // Report the event
      eventsEventWrite(0,0,PROTOCOLAPP_CHARGING_STARTED_EVENT,0,0,0);
      chargingFinishedSent = FALSE;
      chargerIndicationState = CHARGER_INDICATION_CHARGING;
    } 
    else if(!enableChargingWhilePolling)
    {
      // Disable control charging
      chargerControl(CHARGER_CONTROL_DISABLE);
      chargerChargingStatus = CHARGER_NOT_CHARGING;
      chargerIndicationState = CHARGER_INDICATION_POLLING_DISABLE;
    }
    else if(chargerFaultFlagDetect() && (!chargerFaultSended) && (chargerPlugState == CHARGER_PLUG_CONNECTED_SENT))
    {
      chargerChargingStatus = CHARGER_NOT_CHARGING;
      chargerFaultSended = 1;
      // Report the event
      eventsEventWrite(0,0,PROTOCOLAPP_POWER_DC_FAULT_EVENT,0,0,0);
    }
    break;

  case CHARGER_INDICATION_CHARGING:
    //Check if still charging
    if(((chargerPlugStatus == CHARGER_DC_PLUG_INSERTED) || (chargerPlugStatus == CHARGER_DC_PLUG_INSERTED_WITH_POLLING)) 
        && chargerChargeStatusRegGet() && (enableChargingWhilePolling) && !chargerFaultFlagDetect())
    {
      if(chargerChargeStatusRegGet() == CHARGER_BQ_DONE_CHARGE && !chargingFinishedSent)
      {
        // Report charging finished
        eventsEventWrite(0,0,PROTOCOLAPP_CHARGING_ENDED_EVENT,0,0,0);
        chargingFinishedSent = TRUE;
      }
    }
    else if(!enableChargingWhilePolling)
    {      
      // Disable control charging
      chargerControl(CHARGER_CONTROL_DISABLE);
      chargerChargingStatus = CHARGER_NOT_CHARGING;
      chargerIndicationState = CHARGER_INDICATION_POLLING_DISABLE;
      
      // Change level back to idle for the next time
      //chargerLedsIndicationLevel = LEDS_MAIN_PATTERN_IDLE;
    }
    else if (chargerFaultFlagDetect() && (!chargerFaultSended) && (chargerPlugState == CHARGER_PLUG_CONNECTED_SENT))
    {
      chargerChargingStatus = CHARGER_NOT_CHARGING;
      chargerFaultSended = 1;
      // Report the event
      eventsEventWrite(0,0,PROTOCOLAPP_POWER_DC_FAULT_EVENT,0,0,0);
      chargerIndicationState = CHARGER_INDICATION_NOT_CHARGING;
    }
    else
    {            
      // Report the event
      if (!chargingFinishedSent)
        eventsEventWrite(0,0,PROTOCOLAPP_CHARGING_ENDED_EVENT,0,0,0);  
      chargerChargingStatus = CHARGER_NOT_CHARGING;
      // Go back to idle
      chargerIndicationState = CHARGER_INDICATION_NOT_CHARGING;
    }
    break;
  case CHARGER_INDICATION_POLLING_DISABLE:
    if (enableChargingWhilePolling)
    {
      // Enable control charging back
      chargerControl(CHARGER_CONTROL_ENABLE);
      // Go back to default state
      chargerIndicationState = CHARGER_INDICATION_NOT_CHARGING;
    }
    break;
  }
  
   // Refresh leds level
  chargerLedsLevelRefresh();
  //Check if a fault occured requiring indication - overrides previous color if has fault.
  chargerFaultIndicationCheck();
  //Check if a charger connection state changed to set USB Phy accordingly.
  
  chargerBQPlugStatus_T BQ_Plug_Stat = chargerBQPlugDetectionStatusGet();
  if(BQ_Plug_Stat != CHARGER_BQ_PLUG_NOINPUT && !USB_Phy_inited)
  {
    MX_USB_DEVICE_Init();
    USB_Phy_inited = true;
  }
  else if(BQ_Plug_Stat == CHARGER_BQ_PLUG_NOINPUT && USB_Phy_inited)
  {
    USB_Device_DeInit();
    USB_Phy_inited = false;
  }

  if(QueueState == pdTRUE)
    {
      switch(QueueEntry) // Check for indication request.
      {
      case CHARGER_REQ_INDICATION_OFF:
        // Change show indication to off wait
        break;
      case CHARGER_REQ_INDICATION_ON:
        if (chargingDuringMeasFlag) // If PB during measurment with charging - ignore next PB release to decline unwanted indications during fault indication.
        {
          chargingDuringMeasFlag = FALSE;
          // If still indicating measurement failed, dont show battery indication
          if (ledsIndicationStateGet() == LEDS_ON)
            break;
       }
        if(measurementStateGet() == MEASURE_CHECK_USB_CONNECTED)
        {
          chragerShowIndicationAbortDelay = CHARGER_PB_ABORT_SHOW_INDICATION_THRESHOLD;
          measureTaskEventSend(MEASURE_PB_START, MEASURE_TASK_QUEUE_SEND_SOURCE_NORMAL);
          chargingDuringMeasFlag = TRUE; // This indicates we had PB during measurement is waiting for USB disconnect thus failing the measurement.
        }
        else
        {
          // Check if user released push button
          if( HAL_GPIO_ReadPin(PB_IN_GPIO_Port,  PB_IN_Pin) )
          {
            // Turn leds indication on
            chargerShowIndication();
            chragerShowIndicationAbortDelay = CHARGER_PB_ABORT_SHOW_INDICATION_THRESHOLD;
          }
          else
          {
            chargerReq_T newQueueEntry = CHARGER_REQ_INDICATION_ON;

            if(chragerShowIndicationAbortDelay)
            {
              // If user still didn't release push button, send another request until we reach to timeout
              xQueueSend(chargerRequestQueueHandle, &newQueueEntry, 0);
              chragerShowIndicationAbortDelay--;
            }
            else
            {
              chragerShowIndicationAbortDelay = CHARGER_PB_ABORT_SHOW_INDICATION_THRESHOLD;
              measureTaskEventSend(MEASURE_PB_START, MEASURE_TASK_QUEUE_SEND_SOURCE_NORMAL);
            }
          }
        }
        break;
      }
    }


   // Return success
  return(RETURNCODE_OK);
}

/******************************************************************************
* @brief ReturnCode_T chargerProcessing()  
* @param  
* @retval 
******************************************************************************/
ReturnCode_T chargerDcPlugStateProcessing()
{

  chargerDcPlugStatus_T chargerPlugStatusNew = chargerDcPlugStatusGet();

  switch(chargerPlugState)
  {
  case CHARGER_PLUG_DISCONNECTED:
    if((chargerPlugStatusNew == CHARGER_DC_PLUG_INSERTED) || (chargerPlugStatusNew == CHARGER_DC_PLUG_INSERTED_WITH_POLLING))
    {
      DcPlugFilterCounter = CHARGER_DC_PLUG_DETECTION_FILTER_THRESHOLD;
      chargerPlugState = CHARGER_PLUG_CONNECTED;
    }
    break;
  case CHARGER_PLUG_CONNECTED:
    if((chargerPlugStatusNew == CHARGER_DC_PLUG_INSERTED) || (chargerPlugStatusNew == CHARGER_DC_PLUG_INSERTED_WITH_POLLING))
    {
      if(DcPlugFilterCounter)
        DcPlugFilterCounter--;
      else
      {
    	  // Restart UART1 Rx DMA as the plug was reinserted
//    	  hwdriversReInitUart3PcRxDma(); //TODO: Reset USB module?

    	  // Turn charger indication on when plug connected
        chargerRequestSend(CHARGER_REQ_INDICATION_ON);
        
        // Report the event
        eventsEventWrite(0, 0, PROTOCOLAPP_POWER_DC_PLUG_CONNECTED_EVENT, 0, 0, 0);
        
        // Change global state of plug.
        chargerPlugStatus = chargerPlugStatusNew;

        // Go to sended state
        chargerPlugState = CHARGER_PLUG_CONNECTED_SENT;
      }
    }
    else
    {
      // didn't pass filter criteria  
      chargerPlugState = CHARGER_PLUG_DISCONNECTED;
    }
    break;
  case CHARGER_PLUG_CONNECTED_SENT:
    if((chargerPlugStatusNew == CHARGER_DC_PLUG_DETACHED))
    {
      DcPlugFilterCounter = CHARGER_DC_PLUG_DETECTION_FILTER_THRESHOLD;
      chargerPlugState = CHARGER_PLUG_DISCONNECTED_EVENT;
    }
    break;
  case CHARGER_PLUG_DISCONNECTED_EVENT:
    if((chargerPlugStatusNew == CHARGER_DC_PLUG_DETACHED))
    {
      if(DcPlugFilterCounter)
        DcPlugFilterCounter--;
      else
      {
        // Report the event
        eventsEventWrite(0, 0, PROTOCOLAPP_POWER_DC_PLUG_DISCONECTED_EVENT, 0, 0, 0);  
                
        // Change global state of plug.
        chargerPlugStatus = chargerPlugStatusNew;
        // Turn charger indication on when plug disconnected
        chargerRequestSend(CHARGER_REQ_INDICATION_ON);
        // Go to sended state
        chargerPlugState = CHARGER_PLUG_DISCONNECTED;
      }
    }
    else
    {
      // didn't pass filter criteria 
      chargerPlugState = CHARGER_PLUG_CONNECTED_SENT;
    }
    break;
  }
 
  // Return success
  return(RETURNCODE_OK);
}

/******************************************************************************
* @brief Returns the battery voltage based on the BQ's continous measurement 
* @param  None
* @retval Battery voltage in millivolts
******************************************************************************/
uint16_t chargerBattVoltageGet()
{
  uint8_t VBattMsb = chargerRegGet(CHARGER_REG_VBAT_H);
  uint8_t VBattLsb = chargerRegGet(CHARGER_REG_VBAT_L);

  uint16_t VBatt = VBattLsb * CHARGER_BQ25882_VOLT_LSB_BITVAL + VBattMsb * CHARGER_BQ25882_VOLT_MSB_BITVAL;
  return VBatt;
}

/******************************************************************************
* @brief Returns the USB plug state based on the BQ's continous measurement 
* @param  None
* @retval Battery voltage in millivolts
******************************************************************************/
chargerDcPlugStatus_T chargerDcPlugStatusGet() //TODO: Make this more generic with typedefs (for BQ regs)
{
  chargerDcPlugStatus_T MyReturn = CHARGER_DC_PLUG_INSERTED;


  uint16_t dcinVoltage = chargerUSBVoltageGet();

//
//  if(!VbusStat) //No USB input detected
//	  MyReturn = CHARGER_DC_PLUG_DETACHED;
//  else if((pccommnapplayerConnectivityStateGet() == PCCOMMAPPLAYER_CONNECTIVITY_STATE_CONNECTED)) //Any USB type detected - check if VTS is connected
//	  MyReturn = CHARGER_DC_PLUG_INSERTED_WITH_POLLING;
//  else
//	  MyReturn = CHARGER_DC_PLUG_INSERTED; //Charging from any USB source.
  if (dcinVoltage < (CHARGER_MIN_VOLTAGE - CHARGER_VOLTAGE_HYSTERESIS))
      MyReturn = CHARGER_DC_PLUG_DETACHED;
    else if(dcinVoltage > (CHARGER_MIN_VOLTAGE + CHARGER_VOLTAGE_HYSTERESIS))
      MyReturn = CHARGER_DC_PLUG_INSERTED;
    if((MyReturn == CHARGER_DC_PLUG_INSERTED) && (pccommnapplayerConnectivityStateGet() == PCCOMMAPPLAYER_CONNECTIVITY_STATE_CONNECTED))
      MyReturn = CHARGER_DC_PLUG_INSERTED_WITH_POLLING;
  return(MyReturn);
}

/******************************************************************************
* @brief Returns the BQ's USB plug state detected by the BC1.2 protocol. 
* @param  None
* @retval Enum of bits [6:4] in Charger status 2 register from the BQ 
******************************************************************************/
chargerBQPlugStatus_T chargerBQPlugDetectionStatusGet() //TODO: I would like this to be called from a BQ interrupt later on.
{
    uint8_t Vbus_stat = (chargerRegGet(CHARGER_REG_CHG_STATUS2) & 0x70) >> 4;
    return (chargerBQPlugStatus_T)Vbus_stat;
}

/******************************************************************************
* @brief Returns the USB plug voltage based on the BQ's continous measurement 
* @param  None
* @retval Battery voltage in millivolts
******************************************************************************/
uint16_t chargerUSBVoltageGet()
{
  uint8_t VBusLsb = chargerRegGet(CHARGER_REG_VBUS_L);
  uint8_t VBusMsb = chargerRegGet(CHARGER_REG_VBUS_H);

  uint16_t dcinVoltage = VBusLsb * CHARGER_BQ25882_VOLT_LSB_BITVAL + VBusMsb * CHARGER_BQ25882_VOLT_MSB_BITVAL;
  
  return (dcinVoltage);
}
ReturnCode_T chargerRequestSend(chargerReq_T chragerRequest)
{
  xQueueSendFromISR(chargerRequestQueueHandle, &chragerRequest, 0);
  // Return success
  return(RETURNCODE_OK);
}
chargerBQChargeStatus_T chargerChargeStatusRegGet()
{
#ifdef PS_CONNECTED // If powersupply instead of battery - BQ wont charge
  return (CHARGER_BQ_FAST_CHARGE); 
#else
  uint8_t ChargerStatus1 = chargerRegGet(CHARGER_REG_CHG_STATUS1);
  return ((chargerBQChargeStatus_T) (ChargerStatus1 & 0x7));
#endif
}

/******************************************************************************
* @brief Decides if theres a fault in the BQ - read Flag / Status register accordingly
* @param  None
* @retval Fault present or not
******************************************************************************/
bool chargerFaultFlagDetect()
{
  uint8_t BQFaultStatus;
  uint8_t BQ_TS_Stat;
  bool BQFault = FALSE;

  BQFaultStatus = chargerRegGet(CHARGER_REG_FAULT);
  BQ_TS_Stat = chargerRegGet(CHARGER_REG_NTC) & 0x7;
  if(!BQFaultStatus && BQ_TS_Stat < 0x5) // No faults detected & temps are OK
    BQFault = FALSE;
  else
    BQFault = TRUE; // Faults still present - stay here

  return (BQFault);

}
ReturnCode_T chargerLedsLevelRefresh()
{
  LedsPatterns_T chargerLedsIndicationLevelTry;
  
  // Measure voltage
  uint16_t bv = chargerBattVoltageGet();
  
  //Check level of battery
  if (bv < 7250)
    chargerLedsIndicationLevelTry = LEDS_MAIN_PATTERN_CRITBATT;
  else if ((bv >= 7250) && (bv < 7500))
    chargerLedsIndicationLevelTry = LEDS_MAIN_PATTERN_LOWBATT;
  else if ((bv >= 7500) && (bv < 7750))
    chargerLedsIndicationLevelTry = LEDS_MAIN_PATTERN_LOWBATT;
  else if ((bv >= 7750) && (bv < 8000))
    chargerLedsIndicationLevelTry = LEDS_MAIN_PATTERN_FULLBATT;
  else if (bv >= 8000)
    chargerLedsIndicationLevelTry = LEDS_MAIN_PATTERN_FULLBATT;

  // Update threshold
  if(chargerLedsIndicationLevelTry != chargerLedsIndicationLevel)
    chargerChangeLedIndicationCounter++;
  else
    chargerChangeLedIndicationCounter = 0;
  
  // Change leds if we pass threshold
  if(chargerChangeLedIndicationCounter == CHARGER_CHANGE_LED_INDICATION_THRESHOLD)
  {
    chargerLedsIndicationLevel = chargerLedsIndicationLevelTry;
    chargerChangeLedIndicationCounter = 0;
  }

  // Port Indication led color select
  if(chargerLedsIndicationLevel == LEDS_MAIN_PATTERN_FULLBATT) // Port indication color check
	  chargerPortLedsIndication = LEDS_PORT_PATTERN_FULL;
  else
	  chargerPortLedsIndication = LEDS_PORT_PATTERN_CHARGING;
  
  // Return success
  return(RETURNCODE_OK);
}

/******************************************************************************
* @brief  Turn LED indication on accordingly to global indication variables
* @param  None
* @retval Not Implemented
{
******************************************************************************/
ReturnCode_T chargerShowIndication()
{
  ledsPatternSet(chargerLedsIndicationLevel);
  if(!chargerFault && chargerChargingStatus == CHARGER_CHARGING)
    ledsPatternSet(chargerPortLedsIndication);
  // Return success
  return(RETURNCODE_OK);
}

chargerPlugState_T chargerPlugStateGet()
{
  return(chargerPlugState);
} 
 
/******************************************************************************
* @brief  FSM to control port led fault indication (red led), overrides charge state indication and indicates immediatly
* @param  None
* @retval None
{
******************************************************************************/
void chargerFaultIndicationCheck()
{
  chargerFault = chargerFaultFlagDetect();
  switch(chargerFaultState)
  {
    case CHARGER_FAULT_NONE:
      if(chargerFault)
      {
        chargerFaultState = CHARGER_FAULT_DETECTED;
      }
      break;

    case CHARGER_FAULT_DETECTED:
      if(chargerFault)
      {
        chargerFaultState = CHARGER_FAULT_PRESENT;
        chargerPortLedsIndication = LEDS_PORT_PATTERN_FAULT;
        ledsPatternSet(chargerPortLedsIndication);
      }
      else
        chargerFaultState = CHARGER_FAULT_NONE;
      break;
      
    case CHARGER_FAULT_PRESENT:
      if(chargerFault)
        chargerFaultState = CHARGER_FAULT_PRESENT;
      else
      {
        chargerFaultState = CHARGER_FAULT_CLEARED;
        chargerPortLedsIndication = LEDS_PORT_PATTERN_IDLE;
        ledsPatternSet(chargerPortLedsIndication);
      }
      break;

    case CHARGER_FAULT_CLEARED:
      if(chargerFault)
        chargerFaultState = CHARGER_FAULT_DETECTED;
      else
        chargerFaultState = CHARGER_FAULT_NONE;
      break;
      
  }

  return (chargerFault);

}


/******************************************************************************
* @brief  ReturnCode_T chargerReqSchedule( )
* @param
* @retval
******************************************************************************/
ReturnCode_T chargerReqSchedule( )
{
  ReturnCode_T MyReturnCode;
  i2cworkSchedulerEntry_T MySchedulerEntry;

  ChargerConfigState = CHARGER_CONFIG_STATE0;
  // Register I2C activity
  MySchedulerEntry.Mode = I2CWORK_REQMODE_PERIODIC;
  MySchedulerEntry.workType = I2CWORK_OTHER_REQUEST;
  MySchedulerEntry.LedIndex = 0;
  MySchedulerEntry.Period = 20;
  MySchedulerEntry.I2cWorkerPreTranscationCallBackPtr = &chargerReqCallBack;
  // For write operation we don't need the Post transaction callback
  MySchedulerEntry.I2cWorkerPostTranscationCallBackPtr = &chargerPostCallBack;
  i2cworkSchedulerRegister( &MySchedulerEntry);
  // Success
  MyReturnCode = RETURNCODE_OK;
  // Return code
  return(MyReturnCode);
}

uint8_t chargerRegGet(chargerRegId_t regId)
{
  return ReturnedI2cChargerBuffer[regId]; 
}

ReturnCode_T chargerReqCallBack(uint8_t LedIndex, i2cworkSchedulerFunctionParameter_T* ReturnedCommandPtr)
{
  switch( ChargerConfigState)
  {
  case CHARGER_CONFIG_STATE0:
	  // Phase 1 charger config
      ReturnedCommandPtr->Cmd = I2CWORK_TRANSACTION_CMD_WRITE;
      ReturnedCommandPtr->I2cSlaveAddress          = CHARGER_BQ25882_CHARGER_I2C_ADDRESS_LOW;
      ReturnedCommandPtr->SlaveRegAddress          = 0;
      ReturnedCommandPtr->TransactionDataPtr       = Bq25882RegPhase1Config;
      ReturnedCommandPtr->TranscationLength        = sizeof(Bq25882RegPhase1Config);
      ChargerConfigState = CHARGER_CONFIG_STATE1;
      break;
    case CHARGER_CONFIG_STATE1:
       // Phase 2 charger config
       ReturnedCommandPtr->Cmd = I2CWORK_TRANSACTION_CMD_WRITE;
       ReturnedCommandPtr->I2cSlaveAddress          = CHARGER_BQ25882_CHARGER_I2C_ADDRESS_LOW;
       ReturnedCommandPtr->SlaveRegAddress          = 0x12;
       ReturnedCommandPtr->TransactionDataPtr       = Bq25882RegPhase2Config;
       ReturnedCommandPtr->TranscationLength        = sizeof(Bq25882RegPhase2Config);
       ChargerConfigState = CHARGER_CONFIG_STATE2;
       break;


     case CHARGER_CONFIG_STATE2:
      ReturnedCommandPtr->Cmd = I2CWORK_TRANSACTION_CMD_READ;
       ReturnedCommandPtr->I2cSlaveAddress          = CHARGER_BQ25882_CHARGER_I2C_ADDRESS_LOW;
       ReturnedCommandPtr->SlaveRegAddress          = 0;
       ReturnedCommandPtr->TransactionDataPtr       = ReturnedI2cChargerBuffer;
       ReturnedCommandPtr->TranscationLength        = 0x25 + 2;
       ChargerConfigState = CHARGER_CONFIG_STATE2;
       chargerStartedUpdating = TRUE;
       break;

	  default:
		  break;
  }
  return(RETURNCODE_OK);
}


ReturnCode_T chargerPostCallBack(uint8_t LedIndex, i2cworkSchedulerFunctionParameter_T* PostFunctionPtr)
{
  // Only flag for updated data if started reading & finished reading (count is 2)
  if(!chargerFinishedUpdating && chargerStartedUpdating && ChargerConfigState == CHARGER_CONFIG_STATE2)
  {
    chargerFinishedUpdating = TRUE;
  }

  return(RETURNCODE_OK);
}

void chargerResetFirstMeasureFlag()
{
	chargerFinishedUpdating = FALSE;
	chargerStartedUpdating = FALSE;
}




