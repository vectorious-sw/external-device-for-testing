#include "leds.h"


// M O D U L E   G L O B A L S
uint8_t MainLedsCurrentCycleNumber;
LedsCycle_T runningMainPattern;
uint8_t runningMainPatternName;
TimerHandle_t ledsMainPatternTimerHandler;

uint8_t PortLedsCurrentCycleNumber;
LedsCycle_T runningPortPattern;
uint8_t runningPortPatternName;
TimerHandle_t ledsPortPatternTimerHandler;


/******************************************************************************
* @brief  Init the led pattern
* @param  None
* @retval Not implemented
******************************************************************************/
ReturnCode_T ledsInit(void)
{
  //init global variables and timer for main led
  MainLedsCurrentCycleNumber = 0;
  runningMainPattern = ledsPatternTable[LEDS_MAIN_PATTERN_IDLE];
  runningMainPatternName = LEDS_MAIN_PATTERN_IDLE;
  ledsMainPatternTimerHandler = xTimerCreate("ledsMainPatternTimer",  portTICK_PERIOD_MS, pdFALSE, (void *)0, ledsMainPatternTimerTimeoutCallback);

  //init global variables and timer for port led
  PortLedsCurrentCycleNumber = 0;
  runningPortPattern = ledsPatternTable[LEDS_PORT_PATTERN_IDLE];
  runningPortPatternName = LEDS_PORT_PATTERN_IDLE;
  ledsPortPatternTimerHandler = xTimerCreate("ledsPortPatternTimer",  portTICK_PERIOD_MS, pdFALSE, (void *)0, ledsPortPatternTimerTimeoutCallback);

  //Start first led pattern
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
  ledsPatternSet(LEDS_MAIN_PATTERN_WAKEUP);
  ledsPatternSet(LEDS_PORT_PATTERN_IDLE);

  return(RETURNCODE_OK);
}

/******************************************************************************
* @brief  Start running the requested led pattern
* @param  led pattern
* @retval Not implemented
******************************************************************************/
ReturnCode_T ledsPatternSet(LedsPatterns_T pattern)
{
  //set global variables to the relevant led target
  switch(ledsPatternTable[pattern].LedTarget)
  {
    case LEDS_TARGET_MAIN:
      runningMainPattern = ledsPatternTable[pattern];
      runningMainPatternName = pattern;
      MainLedsCurrentCycleNumber = 0;
      //Goto first timer pattern item immediately
      xTimerChangePeriod(ledsMainPatternTimerHandler, 2, 100);
      break;
    case LEDS_TARGET_PORT:
      runningPortPattern = ledsPatternTable[pattern];
      runningPortPatternName = pattern;
      PortLedsCurrentCycleNumber = 0;
      //Goto first timer pattern item immediately
      xTimerChangePeriod(ledsPortPatternTimerHandler, 2, 100);
      break;
  }
  return RETURNCODE_OK;
}

/******************************************************************************
* @brief Set HW gpio for common anode leds by color
* @param  Color for the leds
* @retval Not implemented
******************************************************************************/
ReturnCode_T ledsColorSet(LedsColor_T color, LedTarget_T target) //TODO: Detect if PWM is needed from a new parameter in the struct for color.
{
  HwdriversGpiosT RedPin = ledsHWTargetTable[target].Red;
  HwdriversGpiosT GreenPin = ledsHWTargetTable[target].Green;
  HwdriversGpiosT BluePin = ledsHWTargetTable[target].Blue;
  
  switch(target)
  {
    case LEDS_TARGET_MAIN: //Supports PWM for green
      //GPIO is reversed (common anode)
      hwdriversGpioBitWrite(RedPin, ((color & 0x1) ? 0 : 1 ));
      hwdriversGpioBitWrite(BluePin, ((color & 0x4) ? 0 : 1 ));
      if(color == LEDS_COLOR_ORANGE) //PWM Green (orange color)
      {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, LEDS_ORANGE_GREEN_PWM);
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
      }
      else if(color & 0x2) // Solid green ON
      {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0); //100% Duty cycle
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
      }
      else //Solid green OFF
      {
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
      }
      break;

    case LEDS_TARGET_PORT:
      hwdriversGpioBitWrite(RedPin, ((color & 0x1) ? 0 : 1 ));
      hwdriversGpioBitWrite(GreenPin, ((color & 0x2) ? 0 : 1 ));
      hwdriversGpioBitWrite(BluePin, ((color & 0x4) ? 0 : 1 ));
      break;
  }

  return(RETURNCODE_OK);
}

/******************************************************************************
* @brief  Triggers on ledsMainPatternTimerHandler timeout, progresses the running led pattern
* @param  None
* @retval None
******************************************************************************/
void ledsMainPatternTimerTimeoutCallback()
{
  if (MainLedsCurrentCycleNumber < (runningMainPattern.NumberOfCycles) * 2) //pattern not done
  {
    if(MainLedsCurrentCycleNumber % 2) //leds off
    {
		ledsColorSet(LEDS_COLOR_OFF, LEDS_TARGET_MAIN);
		xTimerChangePeriod(ledsMainPatternTimerHandler, (runningMainPattern.CycleTime * (100 -  runningMainPattern.DutyCycle) / 100), 100);
    }
    else //leds on
    {
		ledsColorSet(runningMainPattern.Color, LEDS_TARGET_MAIN);
    if(runningMainPattern.DutyCycle < 100) //If 100% duty cycle - get stuck (constant color on)
      xTimerChangePeriod(ledsMainPatternTimerHandler, (runningMainPattern.CycleTime * runningMainPattern.DutyCycle / 100), 100);
    }
    MainLedsCurrentCycleNumber = MainLedsCurrentCycleNumber + 1;
  }
  else //pattern done
  {
    ledsColorSet(LEDS_COLOR_OFF, LEDS_TARGET_MAIN);
    runningMainPatternName = LEDS_MAIN_PATTERN_IDLE;
  }
}

/******************************************************************************
* @brief  Triggers on ledsPortPatternTimerHandler timeout, progresses the running led pattern
* @param  None
* @retval None
******************************************************************************/
void ledsPortPatternTimerTimeoutCallback()
{
  if (PortLedsCurrentCycleNumber < (runningPortPattern.NumberOfCycles) * 2) //pattern not done
  {
    if((PortLedsCurrentCycleNumber % 2) && (runningPortPattern.DutyCycle < 100)) //leds off, supports solid color on if DutyCycle is 100
    {
		ledsColorSet(LEDS_COLOR_OFF, LEDS_TARGET_PORT);
		xTimerChangePeriod(ledsPortPatternTimerHandler, (runningPortPattern.CycleTime * (100 -  runningPortPattern.DutyCycle) / 100), 100);
    }
    else //leds on
    {
		ledsColorSet(runningPortPattern.Color, LEDS_TARGET_PORT);
    if(runningPortPattern.DutyCycle < 100) //If 100% duty cycle - get stuck (constant color on)
		  xTimerChangePeriod(ledsPortPatternTimerHandler, (runningPortPattern.CycleTime * runningPortPattern.DutyCycle / 100), 100);
    }

    PortLedsCurrentCycleNumber = PortLedsCurrentCycleNumber + 1;
  }
  else //pattern done
  {
    ledsColorSet(LEDS_COLOR_OFF, LEDS_TARGET_PORT);
    runningPortPatternName = LEDS_PORT_PATTERN_IDLE;
  }

}

/******************************************************************************
* @brief  Retrieve the LEDs indication state. If charge port is RED - does not count as LEDS_ON
* @param  None
* @retval LEDS_ON / LEDS_OFF
******************************************************************************/
LedsIndicationState_T ledsIndicationStateGet()
{
  if((runningMainPatternName == LEDS_MAIN_PATTERN_IDLE) && (runningPortPatternName == LEDS_PORT_PATTERN_IDLE)) // Leds are off
    return (LEDS_OFF);
  else if((runningMainPatternName == LEDS_MAIN_PATTERN_IDLE) && (runningPortPatternName == LEDS_PORT_PATTERN_FAULT)) // Static charger fault indication
    return (LEDS_FAULT);
  else // Leds are on with some pattern
    return(LEDS_ON);
}