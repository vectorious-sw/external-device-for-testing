#include "common.h"
#include "rtc.h"
#include "pccommAppLayer.h"
#include "vlapConfig.h"
#include <time.h>
#include "stm32h7xx.h"
#include "measure.h"
#include "vlapMain.h"
#include "crc32.h"







/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Uncomment the corresponding line to select the RTC Clock source */
#define RTC_CLOCK_SOURCE_LSE           /* LSE used as RTC source clock */
//#define RTC_CLOCK_SOURCE_LSI             /* LSI used as RTC source clock. The RTC Clock
//   may varies due to LSI frequency dispersion. */
#define RTC_BKP_DR_NUMBER   0x14

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
RTC_InitTypeDef   RTC_InitStructure;
RTC_TimeTypeDef   RTC_TimeStructure;
RTC_DateTypeDef   RTC_DateStructure;
TimerHandle_t rtcPBDebounceTimerHandler;
bool PBDebounceSample;

__IO uint32_t uwAsynchPrediv = 0;
__IO uint32_t uwSynchPrediv = 0;
__IO uint32_t uwTimeDisplay = 0;

uint32_t uwErrorIndex = 0;
uint32_t uwIndex = 0;

uint32_t aBKPDataReg[RTC_BKP_DR_NUMBER] =
  {
    RTC_BKP_DR0, RTC_BKP_DR1, RTC_BKP_DR2, 
    RTC_BKP_DR3, RTC_BKP_DR4, RTC_BKP_DR5,
    RTC_BKP_DR6, RTC_BKP_DR7, RTC_BKP_DR8, 
    RTC_BKP_DR9, RTC_BKP_DR10, RTC_BKP_DR11, 
    RTC_BKP_DR12, RTC_BKP_DR13, RTC_BKP_DR14, 
    RTC_BKP_DR15, RTC_BKP_DR16, RTC_BKP_DR17, 
    RTC_BKP_DR18,  RTC_BKP_DR19
  };
uint8_t rtcUsbPluggedState;

/* Private function prototypes -----------------------------------------------*/
static void     RTC_Config(void);
static void     WriteToBackupReg(uint16_t FirstBackupData);
//static uint32_t CheckBackupReg(uint16_t FirstBackupData);
rtcWakeupReasonT rtcWakeupReason;
void rtcPushButtonIsr( void );
void rtcPBDebounceTimerCallback();

rtcLastExceptionEpoch_T  rtcLastExceptionTimeDataStruct __attribute__( ( section( ".noinit") ) );

int rtcInit(void)
{ 
  uint32_t Crc32;
  
  /* Enable the PWR APB1 Clock Interface */
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

  rtcWakeupReason = RTC_WAKEUP_REASON_IDLE;
  rtcPBDebounceTimerHandler =  xTimerCreate("rtcPBDebounceTimerHandler",  portTICK_PERIOD_MS, pdFALSE, (void *)0, rtcPBDebounceTimerCallback);
  xTimerStart(rtcPBDebounceTimerHandler, 100);
  PBDebounceSample = true;
  
  RTC_Config();

  // If it is a powerup, reset LastExceptionEpochTime to the current epoch time 
  Crc32 =  crc32BuffCalc((uint8_t*)&rtcLastExceptionTimeDataStruct.LastExceptionEpochTime, 0, sizeof(rtcLastExceptionTimeDataStruct.LastExceptionEpochTime));
  if(Crc32 != rtcLastExceptionTimeDataStruct.Crc32)
  {
    rtcLastExceptionTimeDataStruct.LastExceptionEpochTime = rtcEpochGet();
    rtcLastExceptionTimeDataStruct.Crc32 = crc32BuffCalc((uint8_t*)&rtcLastExceptionTimeDataStruct.LastExceptionEpochTime, 0, sizeof(rtcLastExceptionTimeDataStruct.LastExceptionEpochTime));
  }
  
  rtcUsbPluggedState = 0;

  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
    {
      Error_Handler();
    }
  return 0;
}

static void RTC_Config(void)
{
  /* Enable the PWR clock */
  //RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

  /* Allow access to RTC - Important to enable it only one time and not to disable it */
  //PWR_BackupAccessCmd(ENABLE);
//  __HAL_RCC_BKPSRAM_CLK_ENABLE(); // TODO: Check if this is needed
  HAL_PWR_EnableBkUpAccess();
  HAL_PWR_EnableBkUpReg();

  
#if 0  // Taken care of by the HAL
    
#if defined (RTC_CLOCK_SOURCE_LSI)  /* LSI used as RTC source clock*/
  /* The RTC Clock may varies due to LSI frequency dispersion. */
  /* Enable the LSI OSC */ 
  RCC_LSICmd(ENABLE);

  /* Wait till LSI is ready */  
  while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
  {
  }

  /* Select the RTC Clock Source */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
  /* ck_spre(1Hz) = RTCCLK(LSI) /(uwAsynchPrediv + 1)*(uwSynchPrediv + 1)*/
  uwSynchPrediv = 0xFF;
  uwAsynchPrediv = 0x7F;

#elif defined (RTC_CLOCK_SOURCE_LSE) /* LSE used as RTC source clock */
  /* Enable the LSE OSC */
  RCC_LSEConfig(RCC_LSE_ON);

  /* Wait till LSE is ready */  
  while(__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == RESET)
  {
  }

  /* Select the RTC Clock Source */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
  /* ck_spre(1Hz) = RTCCLK(LSE) /(uwAsynchPrediv + 1)*(uwSynchPrediv + 1)*/
  uwSynchPrediv = 0xFF;
  uwAsynchPrediv = 0x7F;

  uwSynchPrediv = 0x200;
  uwAsynchPrediv = 0x40;
#else
#error Please select the RTC Clock source inside the main.c file
#endif /* RTC_CLOCK_SOURCE_LSI */
  
  /* Enable the RTC Clock */
  RCC_RTCCLKCmd(ENABLE);

  /* Wait for RTC APB registers synchronisation */
  RTC_WriteProtectionCmd(DISABLE);
  RTC_WaitForSynchro();
  RTC_WriteProtectionCmd(ENABLE);

  // 1 Sec clock source 
  RTC_WakeUpCmd(DISABLE);
  RTC_WakeUpClockConfig(RTC_WakeUpClock_CK_SPRE_16bits);
  RTC_WakeUpCmd(ENABLE);
#if 0
  /* Display the new RCC BDCR and RTC TAFCR Registers */
  vdbg ("RTC Reconfig \n");
  vdbg ("RCC BDCR = 0x%x\n", RCC->BDCR);
  vdbg ("RTC TAFCR = 0x%x\n", RTC->TAFCR); 
#endif

//  /* Set the Time */
//  RTC_TimeStructure.RTC_Hours   = 00;
//  RTC_TimeStructure.RTC_Minutes = 00;
//  RTC_TimeStructure.RTC_Seconds = 0;
//
//  /* Set the Date */
//  RTC_DateStructure.RTC_Month = 4;
//  RTC_DateStructure.RTC_Date = 12;
//  RTC_DateStructure.RTC_Year = 16;
//  
//  /* Set Current Time and Date */
//  RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure);  
//  RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure); 
  
  /* Calendar Configuration */
  RTC_InitStructure.RTC_AsynchPrediv = uwAsynchPrediv;
  RTC_InitStructure.RTC_SynchPrediv =  uwSynchPrediv;
  RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
  RTC_Init(&RTC_InitStructure);



  /*  Backup SRAM ***************************************************************/
  /* Enable BKPRAM Clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE);


  /* Enable the Backup SRAM low power Regulator to retain it's content in VBAT mode */
  PWR_BackupRegulatorCmd(ENABLE);

  /* Wait until the Backup SRAM low power Regulator is ready */
  while(PWR_GetFlagStatus(PWR_FLAG_BRR) == RESET)
  {
  }

#endif

  /* RTC Backup Data Registers **************************************************/
  /* Write to RTC Backup Data Registers */
  // WriteToBackupReg(FIRST_DATA);


}

static void WriteToBackupReg(uint16_t FirstBackupData)
{
  uint32_t index = 0;

  for (index = 0; index < RTC_BKP_DR_NUMBER; index++)
  {
	  HAL_RTCEx_BKUPWrite(&hrtc, aBKPDataReg[index], FirstBackupData + (index * 0x5A));
  }





}

#if 0
/**
 * @brief  Checks if the Backup data registers values are correct or not.
 * @param  FirstBackupData: data to read from first backup data register
 * @retval - 0: All Backup DRx registers data are correct
 *         - Value different from 0: Number of the first Backup register which 
 *           value is not correct
 */
static uint32_t CheckBackupReg(uint16_t FirstBackupData)
{
  uint32_t index = 0;

  for (index = 0; index < RTC_BKP_DR_NUMBER; index++)
  {
    if (HAL_RTCEx_BKUPRead	(&hrtc, aBKPDataReg[index]) != (FirstBackupData + (index * 0x5A)))
    {
      return (index + 1);
    }
  }

  return 0;
}
#endif
void rtcPBDebounceTimerCallback()
{
  PBDebounceSample = true; 
}
void rtcTimeDateUpdate()
{
	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};

  	HAL_RTC_GetTime (&hrtc, &sTime, RTC_FORMAT_BIN);
  	HAL_RTC_GetDate (&hrtc, &sDate, RTC_FORMAT_BIN);

  	pccpmmAppLayerStruct.BoardSystemRegisters.RtcSeconds         = sTime.Seconds;
  	pccpmmAppLayerStruct.BoardSystemRegisters.RtcMinutes         = sTime.Minutes;
  	pccpmmAppLayerStruct.BoardSystemRegisters.RtcHours           = sTime.Hours;
  	pccpmmAppLayerStruct.BoardSystemRegisters.RtcDayInWeek       = sDate.WeekDay;
  	pccpmmAppLayerStruct.BoardSystemRegisters.RtcDay             = sDate.Date;
  	pccpmmAppLayerStruct.BoardSystemRegisters.RtcMonth           = sDate.Month;
  	pccpmmAppLayerStruct.BoardSystemRegisters.RtcYear            = sDate.Year;
}




ReturnCode_T rtcEpochTimeStampGet( uint32_t* ReturnedEpochPtr, uint8_t* Returned10mSecCountPtr)
{
	ReturnCode_T MyReturnCode;
	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};
	struct tm t;

  
	if(ReturnedEpochPtr)
	{
		HAL_RTC_GetTime (&hrtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate (&hrtc, &sDate, RTC_FORMAT_BIN);

		t.tm_year 	= sDate.Year+100;         	// Years since 1900
		t.tm_mon 	= sDate.Month-1;           	// Month, 0 - jan
		t.tm_mday 	= sDate.Date;          		// Day of the month
		t.tm_hour 	= sTime.Hours;
		t.tm_min 	= sTime.Minutes;
		t.tm_sec 	= sTime.Seconds;
		t.tm_isdst = 0;        // Is DST on? 1 = yes, 0 = no, -1 = unknown
		*ReturnedEpochPtr = (uint32_t) mktime(&t);
		*Returned10mSecCountPtr = 0;
		MyReturnCode = RETURNCODE_OK;
	}
	else
		MyReturnCode = RETURNCODE_ERROR;

	return(MyReturnCode);
}
 
void rtcResetDateTime()
{
	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};
//	struct tm t;


	sTime.Hours   = 13;
	sTime.Minutes = 11;
	sTime.Seconds = 0;

	/* Set the Date */
	sDate.Month = 8;
	sDate.Date = 22;
	sDate.Year = 16;

	HAL_RTC_SetTime (&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_SetDate (&hrtc, &sDate, RTC_FORMAT_BIN);

}



void rtcEpochTimeAndDateSet(time_t now)
{
	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};

	struct tm time_tm;
	time_tm = *(localtime(&now));

	sTime.Hours = (uint8_t)time_tm.tm_hour;
	sTime.Minutes = (uint8_t)time_tm.tm_min;
	sTime.Seconds = (uint8_t)time_tm.tm_sec;
  sTime.TimeFormat = RTC_HOURFORMAT12_AM;
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	HAL_RTC_SetTime (&hrtc, &sTime, RTC_FORMAT_BIN);


	if (time_tm.tm_wday == 0) { time_tm.tm_wday = 7; }  // the chip goes mon tue wed thu fri sat sun
	sDate.WeekDay = (uint8_t)time_tm.tm_wday;
	sDate.Month = (uint8_t)time_tm.tm_mon+1; //momth 1-12. This is why date math is frustrating.
	sDate.Date = (uint8_t)time_tm.tm_mday;
	sDate.Year = (uint16_t)(time_tm.tm_year+1900-2000);  // time.h is years since 1900, chip is years since 2000

	HAL_RTC_SetDate (&hrtc, &sDate, RTC_FORMAT_BIN);

}

uint32_t rtcEpochGet()
{
	uint32_t EpochtimeStamp;
	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};
	struct tm t;

	HAL_RTC_GetTime (&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate (&hrtc, &sDate, RTC_FORMAT_BIN);

	t.tm_year 	= sDate.Year+100;         	// Years since 1900
	t.tm_mon 	= sDate.Month-1;           	// Month, 0 - jan
	t.tm_mday 	= sDate.Date;          		// Day of the month
	t.tm_hour 	= sTime.Hours;
	t.tm_min 	= sTime.Minutes;
	t.tm_sec 	= sTime.Seconds;
	t.tm_isdst = 0;        // Is DST on? 1 = yes, 0 = no, -1 = unknown
	EpochtimeStamp = (uint32_t) mktime(&t);

    return(EpochtimeStamp);
}



void rtcDateTimeConsolePrint()
{
	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};

	HAL_RTC_GetTime (&hrtc, &sTime, RTC_FORMAT_BCD);
	HAL_RTC_GetDate (&hrtc, &sDate, RTC_FORMAT_BCD);

	char *PrntPtr = pvPortMalloc(100);
  
  uint32_t Length = sprintf(PrntPtr, "Date And Time = %02x-%02x-%02x   %02x:%02x:%02x\n\r\n\r", sDate.Date, sDate.Month, sDate.Year, sTime.Hours, sTime.Minutes, sTime.Seconds) ;
  uartdllTxQueueEnqueue(UARTDLL_UART_6_RS485, (uint8_t*)PrntPtr, Length, true);
}



/*******************************************************************************
*** void
*
*
*
******************************************************************************/
rtcWakeupReasonT rtcSleepStart()
{
  volatile uint32_t ImrTemp;

  __HAL_RCC_GPIOD_CLK_ENABLE();
  
  __HAL_RTC_WAKEUPTIMER_EXTI_ENABLE_IT();
  __HAL_RTC_TAMPER_TIMESTAMP_EXTI_ENABLE_IT();
  __HAL_RTC_ALARM_ENABLE_IT(&hrtc, RTC_IT_ALRA);
  __HAL_RTC_ALARM_ENABLE_IT(&hrtc, RTC_IT_ALRB);

#if 0
  RTC_ITConfig(RTC_IT_WUT, ENABLE);
  RTC_ITConfig(RTC_IT_TS, DISABLE);
  RTC_ITConfig(RTC_IT_ALRA, DISABLE);
  RTC_ITConfig(RTC_IT_ALRB, DISABLE);
  RTC_ITConfig(RTC_IT_TAMP, DISABLE);
  RTC_ITConfig(RTC_IT_TAMP1, DISABLE);
#endif

  // Time in seconds = WakeUpCounter
  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 50, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);

  //  RTC_WakeUpCmd(ENABLE);
    // Clear pending RTC interrupt flags 
  __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);


  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
// Add clear ALARM1, ALARM2
  

//  GPIOA->MODER = 0x00000000;
//  GPIOB->MODER = 0x00000000;
//  GPIOC->MODER = 0x00000000;
//  GPIOD->MODER = 0x00000000;
//  GPIOE->MODER = 0x00000000;



  //hwdriversGpioBitWrite(HWDRIVERS_PB15_USB_TEST,1);
  USB_Device_DeInit(); // Stop the USB Phy before sleep
  rtcWakeupReason = RTC_WAKEUP_REASON_IDLE;
  vTaskSuspendAll();
  // Invalidate current charger registers
  chargerResetFirstMeasureFlag();
  // Save the IMR
  // TODO: Check if using IMR1 is ok
  ImrTemp = EXTI->IMR1;
  // Mask all interrupts
  EXTI->IMR1 = 0x0;
  
  // Clear all pending interrupt flags 
  EXTI->PR1 = 0x7fffff;
  // Enable only Push button, RTC and VIN (USB Plug Connect) interrupts 
  EXTI->IMR1 = (EXTI_IMR1_IM4 | EXTI_IMR1_IM14 | EXTI_IMR1_IM19 | EXTI_IMR1_IM16);

	/*
	 Charger Event          PE4            EXTI4
	 Push button            PD14           EXTI14
	 WAKEUP                 Internal       EXI19
	 PVD                    Internal       EXTI16
	*/

  __HAL_RCC_SYSCLK_CONFIG(RCC_CFGR_SW_HSI); // Switch to HSI Clock
  __HAL_RCC_HSE_CONFIG(RCC_HSE_OFF); // Turn off HSE before external clock disable

  hwdriversGpioBitWrite(HWDRIVERS_MAIN_OSC_ENABLE, 0); // Disable external clock
  
  HAL_PWR_EnterSTOPMode (PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
  
  hwdriversGpioBitWrite(HWDRIVERS_MAIN_OSC_ENABLE, 1); // Enable external clock after wake up
  
  SystemClock_Config();

  // Restore interrupt mask register
  EXTI->IMR1 = ImrTemp ;

  chargerRefreshUSBInit(); // Re-trigger the charger-USB init / deinit phase
  xTaskResumeAll();

  // rtcWakeupReasons are set by the RTC and the PushButton ISRs 
  return(rtcWakeupReason);
}
       


void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	 rtcWakeupReason = RTC_WAKEUP_REASON_ALARM_A;
}
void HAL_RTC_AlarmBEventCallback(RTC_HandleTypeDef *hrtc)
{
	rtcWakeupReason = RTC_WAKEUP_REASON_ALARM_B;
}
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
	 rtcWakeupReason = RTC_WAKEUP_REASON_TIMED_WAKEUP;
}

#if 0
/******************************************************************************
*** @brief  void RTC HAndler(void)
*   @param  
*   @retval 
******************************************************************************/
void RTC_WKUP_IRQHandler(void)
{

  if(EXTI_GetITStatus(EXTI_Line22) != RESET)
  {
    
    if(RTC_GetITStatus(RTC_IT_WUT))
    {
      rtcWakeupReason = RTC_WAKEUP_REASON_TIMED_WAKEUP;
      RTC_ClearFlag(RTC_FLAG_WUTF);
    }
    if(RTC_GetITStatus(RTC_IT_ALRA))
    {
      rtcWakeupReason = RTC_WAKEUP_REASON_ALARM_A;
      RTC_ClearFlag(RTC_IT_ALRA);
    }
    if(RTC_GetITStatus(RTC_IT_ALRB))
    {
      rtcWakeupReason = RTC_WAKEUP_REASON_ALARM_B;
      RTC_ClearFlag(RTC_IT_ALRB);
    }
    if(RTC_GetITStatus(RTC_IT_TAMP1))
    {
      rtcWakeupReason = RTC_WAKEUP_REASON_UNKNOWN;
      RTC_ClearFlag(RTC_IT_TAMP1);
    }
    

    EXTI_ClearITPendingBit(EXTI_Line22);
  }
}
#endif




/******************************************************************************
*** @brief  rtcPushButtonIsr( void ) PUSH_BUTTON interrupt
*   @param  
*   @retval 
******************************************************************************/
void rtcPushButtonIsr( void ) // blue user button on new device
{
  measureReqQueueEntry_T MeasureQueueEntry;
  
//  if (EXTI_GetITStatus(EXTI_LINE_BLUE_PB_NEW_DEVICE) != RESET)
  {

      if(!HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14) && PBDebounceSample)
      {
        // Debounce push button
        xTimerChangePeriodFromISR(rtcPBDebounceTimerHandler, 300, 100);
        PBDebounceSample = false;
        
        measureTaskEventSend(MEASURE_PB_START, MEASURE_TASK_QUEUE_SEND_SOURCE_ISR);
      }           

  }    
  // We do not clear the Interrupt flag in sleep mode as we want to identify the sleep wakeup reason, will be done in rtc.c sleep section
  //EXTI_ClearITPendingBit(EXTI_LINE_BLUE_PB_NEW_DEVICE);
}


 



// General EXTI Interrupt Service Routin
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	switch(GPIO_Pin)
	{
		case GPIO_PIN_0 :
			break;
		case GPIO_PIN_2 :
			// ACCELGYRO_SNS_INT2:
			break;
		case GPIO_PIN_6 :
			// ACCELGYRO_SNS_INT2:
			break;
		case GPIO_PIN_14:
			// PB_IN
			rtcPushButtonIsr();
			break;

		case GPIO_PIN_1 :
		case GPIO_PIN_3 :
		case GPIO_PIN_4 :
    		rtcWakeupReason = RTC_WAKEUP_REASON_CHARGER_EVENT;
		  break;
		case GPIO_PIN_5 :
		case GPIO_PIN_7 :
		case GPIO_PIN_8 :
		case GPIO_PIN_9 :
		case GPIO_PIN_10:
		case GPIO_PIN_11:
		case GPIO_PIN_12:
		case GPIO_PIN_13:
		case GPIO_PIN_15:
			break;
	}
}


// VI_MEASAURE
#if 0
void EXTI0_IRQHandler()
{
  /* Make sure that interrupt flag is set */
  if (EXTI_GetITStatus(EXTI_Line0) != RESET) 
  {
    if(GPIO_IDR_IDR0_BB(GPIOC))
      rtcWakeupReason = RTC_WAKEUP_REASON_VIN_PLUG_CONNECTED_DISCONNECTED;
    else
      rtcWakeupReason = RTC_WAKEUP_REASON_VIN_PLUG_CONNECTED_CONNECTED;
    
    
    /* Clear interrupt flag */
    EXTI_ClearITPendingBit(EXTI_Line0);
  } 
}
#endif


uint8_t rtcUsbPluggedStateGet()
{
  // rtcUsbPluggedState = HAL_GPIO_ReadPin(VIN_MEASURE_GPIO_Port, VIN_MEASURE_Pin);
  return(rtcUsbPluggedState);
}




      
      
      






rtcWakeupReasonT rtcWakeupReasonGet()
{
  return(rtcWakeupReason);
}
