
#include <hwdrivers.h>
#include <stdlib.h>
#include <math.h> 
#include <stdarg.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "vlapConfig.h"
#include "pccommAppLayer.h"
#include "timers.h"
//#include "gpioTable.h"
#include "audit.h"
#include "autoresonance.h"
#include "autopower.h"
#include "vlapMain.h"
#include "measure.h"
#include "MeasurementsData.h"

#include "measure.h"
#include "dac.h"
#include "main.h"
#include "usartdll.h"

void vlapmainMeasurementSet( uint32_t * DstPtr, uint32_t *ValuePtr, uint16_t Count);
void blinkGreenLedCallback(TimerHandle_t pxTimer);
void  vlapmainVlapUniqueAddPrint();    


uint16_t adc1DmaBuffer[2][BUFFERSIZE];
void testTaskSwitching(void);
void lateConfig(void);
TimerHandle_t xx300Timer = NULL;
int vlapMainBusy = 1;
TaskHandle_t vlapDemodulatorTaskHandler; 

static void blinkGreenLed(void);

void vlapmainDemodulatorTaskControl( vlapmainDemodulatorTaskControlT Control);

 volatile uint32_t FreeMemory;

 uint8_t ModulatorTaskState=0;
 
 // TODO: Remove
 uint8_t CommandSequence;
 
uint8_t RequestState;

//TODO: Remove, Just for testing
  extern ProtocolappMeasurementEndedEvent_t MeasurementsResults; 


  uint8_t * PrntPtr;
  
//
// main task
//
portTASK_FUNCTION(vlapMain, pvParameters )
{
  /* The parameters are not used. */
  (void) pvParameters;
  


  lateConfig(); 


#if 1

  // create the vlapDemodulator task

  vlapmainDemodulatorTaskControl( VLAPMAIN_TASK_CREATE);
  //
  //ledsSet(0, LEDS_COLOR_MAGENTA, LEDS_STATE_BLINK, 4);
  //  
  
//  char MyTestBuff[] = "vlapMain\n\r";
  
//  PrntPtr = pvPortMalloc(100);
//
//  uint16_t Len = sprintf( (char*)PrntPtr, "vlapMain \r");
//  uartdllTxQueueEnqueue(UARTDLL_UART_6_RS485, PrntPtr, Len, true);
#endif
  vlapmainVlapUniqueAddPrint();  
#if 1
  rtcDateTimeConsolePrint();

  // For debug
  //hwdriversGpioBitWrite(HWDRIVERS_PB15_USB_TEST,1);
  
//  chargerReqSchedule();
//  sensorPressureReqSchedule();
  //
  
  //hwdriversGpioBitWrite(HWDRIVERS_UI_ENABLE,0);

  blinkGreenLed();
   
  measureInit();

  // Task allocated for freeing memory allocated for uart transmission - req. via mail box sent from the UART tx dma completion ISR  
  hwdriverFreeMemFromIsrTaskInit();

  //spiflashdiskInit();
  // Start device with the modem off
  measureModemStateTurnOff();

#endif
  // Endless loop
  while(1)
  {
    
    vTaskDelay( 1 );
    // Handle incoming serial data from the UART DMA and feed the uart DLL.

    hwdriversUart1DmaRxDataProcess();

    hwdriversUart6DmaRxDataProcess();
//    hwdriversUart6DmaRxDataProcess();
#ifdef USE_CELLMODEM
//    hwdriversUart2DmaRxDataProcess();
#endif
//    hwdriversUart3DmaRxDataProcess();
    
#if 0    
    switch(RequestState)
    {
    case 0:
      if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_1) )
      {
        commTaskEventSend(COMM_FSM_QUEUE_ENTRY_TYPE_BLE_READ_REQ);
        RequestState =1;
      }
      break;
    case 1:
      if(!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_1) )
      {
        RequestState =0;
      }
      break;
    }
#endif   

    

  }

    
    


}


void vlapTraceTaskSwitchIn()
{
 getCurrentTaskName();
}





void vlapMainSetupHardware(uint8_t FirstTimeAfterJlinkProgramming)
{
   
  //
  // configure the system
  //

  RequestState=0;
  
  pcccommAppLayerInit();  
  vmicmodemInit();
  autoresonanceInit(AUTORESONANCE_INIT_REQ_AFTER_POWERUP);
  autoresonanceControl(AUTORESONANCE_CONTROL_OFF);
 



  uartdllInit();
  usartInit();

  if(FirstTimeAfterJlinkProgramming)
    vlapmainDebugLog("vlapMain: First time after Jlink");
  else
    vlapmainDebugLog("vlapMain");





  autopowerInit();
//  hwdriversClockInit();
  hwdriversPVDConfig();
  //hwdriversNvicConfig();
  // Init UARTS Async format: UARTS:1,2,3,6 

//  vTaskDelay(100);
  hwdriversUartsConfig();

//  vTaskDelay(100);
  // Init UARTS RX DMA: UARTS:1,2,3,6 
  hwdriversUartsDmaRxConfig();
  // Set all GPIOs configuration from the gpioTable
  //gpioTableActivate(ALL_GPIO_PINS);

  //  
  rtcInit();
  



  hwdriversSpiFlashConfig();
  


  if(FirstTimeAfterJlinkProgramming)
  {
//    eventsJlinkProgrammaingLogMemoryPointersSet();
    //eventsLogMemoryPointersToNvmSave();
  }
  
  //
//  commInit();
  // 
  //hwdriversSpiFlashConfig();


   
  hwdriversAdc1Config((uint8_t*)adc1DmaBuffer[0], (uint8_t*)adc1DmaBuffer[1], 2*BUFFERSIZE);
  hwdriversTimersStart();

  // autopower needs to be initialized before ADC2 as is is called from the ADC2 DMA completion isr
  hwdriversAdc2Config();
  // TODO: I2C Removed 
  hwdriversI2cConfig();
  
//  chargerInit();
  
  dacInit();
  
  hwdriversTimer7Init();
  // Init the frequency monitoring timer 
  // hwdriversTimer3Init();
  



  configInit(FirstTimeAfterJlinkProgramming);


  
  // After burn - write the eventsLogMemoryPointers initialized parameters
//  if (FirstTimeAfterJlinkProgramming)
//  {
//    eventsLogMemoryPointersToNvmSave();
//  }
  
  // Start ADC Software Conversion 
  // the following is redundant, since SWSTART is automaticaly set by the dma.
  // but we still decided to leave it here, for doc sake. gc, nm
//  bleInit();
//  bleControl( BLE_CONTROL_ENABLE);
  // TODO:
  //ADC_SoftwareStartConv(ADCx);
  CommandSequence = 0;
  
  //hwdriversVinInterruptConfig();
  //Configure_PA0();

  


//  fwupgradeInit();


}



static void blinkGreenLed(void)
{
//  xTimerStart(xTimerCreate("gled", // Just a text name, not used by the kernel.
//                           5000*portTICK_PERIOD_MS, //ond
//                           pdTRUE, // The timers will auto-reload themselves when they expire.
//                           (void *)0,
//                           blinkGreenLedCallback
//                           ), 0);
}


    
uint8_t MyTestBuff[300];
uint8_t RedLedState;
 void blinkGreenLedCallback(TimerHandle_t pxTimer)
{

  
  
  uint16_t i;

  
  int16_t ReturnedPositiveNtcTemperature;
  int16_t ReturnedNegativeNtcTemperature;
  uint16_t ReturnedVbatVoltage;
  uint16_t ReturnedVbatCurrent;
  uint16_t ReturnedDcVoltage;
  uint16_t ReturnedRequency;

    

  // Send snap of the 
//  FreeMemory  = xPortGetFreeHeapSize();
//  Len = sprintf( MyTestBuff, "%06d, %x6\n\r", CycleCounter ++, FreeMemory);
// uartdllTxQueueEnqueue(UARTDLL_UART_6_RS485, MyTestBuff, Len, false);
  
  
  hwdriversNtcTemperatureGet(&ReturnedPositiveNtcTemperature,HWDRIVERS_TX_NTC_P);
  ReturnedVbatVoltage = 0;//chargerBattVoltageGet();
  
  ReturnedDcVoltage = 0;//chargerUSBVoltageGet();
  hwdriversFrequencyMonitoringGet(&ReturnedRequency);

  // Report the battery voltage to the GUI ( 1mV units)
  pccpmmAppLayerStruct.BoardSystemRegisters.BatteryVoltage = TYPES_ENDIAN16_CHANGE(ReturnedVbatVoltage);
  
  pccpmmAppLayerStruct.BoardSystemRegisters.SupplyVoltage = TYPES_ENDIAN16_CHANGE(ReturnedDcVoltage);
  
#if 0 
  if(RedLedState)
  {
    hwdriversGpioBitWrite(HWDRIVERS_RED_LED, 1 );
    RedLedState = 0;
  }
  else
  {
    hwdriversGpioBitWrite(HWDRIVERS_RED_LED, 0 );
    RedLedState = 1;
  }

#endif
  
}


void lateConfig(void)
{
  // these inits run after the scheduler is up

  //i2cReqQ = xQueueCreate(10, sizeof(I2CREQ));
  //sfReqQ = xQueueCreate(10, sizeof(SFREQ));
  //sfReqQ1 = xQueueCreate(3, sizeof(SFREQ));

#ifndef BURN_CONFIG
  //ioExpanderConfig();
	auditInit();
#endif



	   __HAL_TIM_MOE_ENABLE(&htim1);
	   __HAL_TIM_MOE_ENABLE(&htim5);
	  //  __HAL_TIM_MOE_ENABLE(&htim8);
	  //  __HAL_TIM_MOE_ENABLE(&htim4);
	   __HAL_TIM_MOE_ENABLE(&htim7);


	   __HAL_TIM_ENABLE(&htim1);
//	   __HAL_TIM_ENABLE(&htim5);
	  //  __HAL_TIM_ENABLE(&htim8);


	   TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_1, TIM_CCx_ENABLE);
	  //  TIM_CCxChannelCmd(TIM8, TIM_CHANNEL_4, TIM_CCx_ENABLE);
	   TIM_CCxChannelCmd(TIM5, TIM_CHANNEL_1, TIM_CCx_ENABLE);



  vlapMainBusy = 0;
}




void vlapmainDemodulatorTaskControl( vlapmainDemodulatorTaskControlT Control)
{
 
  switch(Control)
  {
  case VLAPMAIN_TASK_CREATE:
      xTaskCreate(vlapDemodulator, vlapDemodulatorName, 
      vlapDemodulatorSTACK_SIZE, NULL, 
      vlapDemodulatorPriority, ( TaskHandle_t * ) &vlapDemodulatorTaskHandler );
    break;

  case VLAPMAIN_TASK_STOP:
    if(vlapDemodulatorTaskHandler)
       vTaskSuspend( vlapDemodulatorTaskHandler );
    StopPWM();
    ModulatorTaskState = 0;
    break;
  case VLAPMAIN_TASK_START:
    if(vlapDemodulatorTaskHandler)
    {
      if(!ModulatorTaskState)
      {
        vTaskResume(vlapDemodulatorTaskHandler);
       //VmicModemRxModeSet();
       ResumePWM();
       ModulatorTaskState = 1;
      }
    }
    break;
  }
}




void vlapmainDebugLog(char * debugString)
{
  uint8_t Len;
  uint8_t * Ptr;
  FreeMemory  = xPortGetFreeHeapSize();
  Len = sprintf( (char*)MyTestBuff, "%09d, %x,  %s\n\r", hwdriversCycleCounterGet(), FreeMemory, debugString);
  Ptr = pvPortMalloc(Len);
  memcpy(Ptr, MyTestBuff, Len);
  uartdllTxQueueEnqueue(UARTDLL_UART_6_RS485, Ptr, Len, true);
  // Delay and let the message to be queued out 
//  vTaskDelay(100);
}

void  vlapmainVlapUniqueAddPrint()    
{
  uint32_t i;
  uint8_t VlapExUniqId[12];
  uint32_t Length;
  uint8_t * UniqueueAddressPtr = protocolappUniqueIdPtrGet();
  PrntPtr = pvPortMalloc(100);

  
  for(i=0; i<12; i++)
    VlapExUniqId[i] = *UniqueueAddressPtr++;

  Length = sprintf(PrntPtr, "VLAP ID = %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x     FW Version = %d.%d.%d \n\r", VlapExUniqId[0],VlapExUniqId[1],VlapExUniqId[2],VlapExUniqId[3],VlapExUniqId[4],VlapExUniqId[5],VlapExUniqId[6],VlapExUniqId[7],VlapExUniqId[8],VlapExUniqId[9],VlapExUniqId[10],VlapExUniqId[11], PCCOMMAPPLAYER_FW_VERSION_MAJOR, PCCOMMAPPLAYER_FW_VERSION_MINOR, PCCOMMAPPLAYER_FW_VERSION_BUILD);
  uartdllTxQueueEnqueue(UARTDLL_UART_6_RS485, PrntPtr, Length, true);

}
