#include "vlapConfig.h"
#include "usartdll.h"
#include <math.h>
#include <arm_math.h>
#include <hwdrivers.h>
#include "timers.h"
#include "iir.h"
#include "autoresonance.h"
#include "autopower.h"
#include "protocolapp.h"
#include "transQ.h"
#include "pccommAppLayer.h"
#include  "measure.h"
#include "task.h"
#include "queue.h"
#include "stm32h7xx.h"


// G L O B A L   T Y P E S   A N D    D E F I N I T I O N S 
uint8_t Uart1Dma2Stream7Busy;
uint8_t Uart2Dma1Stream6Busy;
uint8_t Uart3Dma1Stream3Busy;
uint8_t Uart6Dma2Stream6Busy;
uint32_t hwdriversCycleCounter;

// G L O B A L  P R O T O T Y P E S 


// L O C A L    T Y P E S   A N D    D E F I N I T I O N S  
// For Spec. definitions: rate of 211.84Khz sampling rate
#define ADC_SAMPLING_FACTOR                     127
#define UART_RX_DMA_CIRCULAR_BUFFER_LEGTH       100
#define HWDRIVERS_ADC_REF_VOLTAGE               3       // 3 Volts
#define HWDRIVERS_ADC_RESOLUTION                4095
#define HWDRIVERS_FREQUENCY_MONITORING_FILTER   5
#define HWDRIVERS_ISMON_OFFSET                  0.25
#define HWDRIVERS_ISMON_RESISTOR                11.11E-3
#define HWDRIVERS_VDD_PA_RATIO              (11.0 / (20.0 + 11.0))
typedef enum {HWDRIVERS_BATT_MEASURE_IDLE, HWDRIVERS_BATT_MEASURE_START } hwdriversBattMeasureState_T;

#define USE_PVD 1




// L O C A L    P R O T O T Y P E S
void RelaysOutputHandle(uint16_t NewRelaysValue);
static void adc2TimerCallback(TimerHandle_t pxTimer );
void hwdriversTimer3Init();
ReturnCode_T hwdriversTxProcessing();
ReturnCode_T hwdriverFreeMemFromIsrTaskInit();
portTASK_FUNCTION(hwdriverFreeMemFromIsrTask, pvParameters );



// V A R S  
uint8_t Uart1RxDmaCicularBuffer[UART_RX_DMA_CIRCULAR_BUFFER_LEGTH];
uint16_t Uart1CicularBufferIndex=0;

uint8_t Uart2RxDmaCicularBuffer[UART_RX_DMA_CIRCULAR_BUFFER_LEGTH];
uint16_t Uart2CicularBufferIndex=0;

uint8_t Uart3RxDmaCicularBuffer[UART_RX_DMA_CIRCULAR_BUFFER_LEGTH];
uint16_t Uart3CicularBufferIndex=0;

uint8_t Uart6RxDmaCicularBuffer[UART_RX_DMA_CIRCULAR_BUFFER_LEGTH];
uint16_t Uart6CicularBufferIndex=0;

hwdriversBattMeasureState_T hwdriversBattMeasureSeqState;


// Frequency Monitoring
uint16_t FreqMonitoringFrequency;
uint16_t FreqMonitoringCyclesLast;
uint16_t LastFrequency;
uint16_t FilteredFrequecy;
uint8_t FrequencyMonitoringFilter;



uint8_t UartRxBuffer[UART_RX_BUFFER_LENGTH];
//TIM_TimeBaseInitTypeDef TIM_TimeBaseStructureTim7;
//TIM_TimeBaseInitTypeDef TIM_TimeBaseStructureTim3;


// addresses of performance cycles registers
volatile uint32_t *DWT_CONTROL = (uint32_t *)0xE0001000;
volatile uint32_t *DWT_CYCCNT = (uint32_t *)0xE0001004; 
volatile uint32_t *DEMCR = (uint32_t *)0xE000EDFC; 

// Note: hwdriversAdc2InputsT is used as index for ADCConvertedValue
volatile uint16_t ADCConvertedValueUnFiltered[4];
volatile uint16_t ADCConvertedValue[4];


// ree buffers from ISR task
QueueHandle_t hwdriversMemFreeQueueHandle;

static void adc2TimerCallback(TimerHandle_t pxTimer );
void waitHere(uint32_t delay);
void hedriversAdc1TranbsferCompleteIsr(void);
void adc2DmaCompleteIsr(void);


#define HWDRIVERS_USB_RX_CIRCULAR_BUFER_LENGTH	1200
uint16_t usbRxCircularHeadIndex;
uint16_t usbRxCircularTailIndex;
uint8_t usbRxCicularBuffer[HWDRIVERS_USB_RX_CIRCULAR_BUFER_LENGTH];


uint16_t SavedRelayState;

/******************************************************************************
 * @brief  void hwdriversTimersStartr()
 * @param  
 * @retval 
 ******************************************************************************/
void hwdriversTimersStart()
{





#if 0

	NVIC_InitTypeDef      NVIC_InitStructure;
  TIM_OCInitTypeDef     sConfigOC;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  

   
  
  // Set timer2 clock source to be internal
  TIM_InternalClockConfig(TIM2);
  
  // Timer2 Time base configuration
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Period = ADC_SAMPLING_FACTOR; 
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  
  // Define Timer2 trigger out event (To be used by ADC1 as sampling trigger source)
  TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update); // ADC_ExternalTrigConv_T2_TRGO
  // TIM2 enable counter
  __HAL_TIM_ENABLE(&htim2);;
  
 // Not Needed !!! Enable Timer2 frequency GPIO output
 // TIM_CCxCmd(TIM2, TIM_Channel_2, TIM_CCx_Enable);
  
#ifdef USE_TIM1  
  // Timer1 clock source is internal  
  TIM_InternalClockConfig(TIM1);
  // Timer1 clock source is the oscillator's 6.76 Mhz frequency required to drive the VMIC
  // it is 1/4 of the base clock of 27.1 MHZ

  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

  TIM_TimeBaseStructure.TIM_Period = 1; //TIM2CLK (ie APB1 = HCLK/4, TIM2CLK = HCLK/2)
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  // Timer1 Output Compare Toggle Mode configuration: Channel1 
  sConfigOC.TIM_OCMode = TIM_OCMode_Toggle;
  sConfigOC.TIM_OutputState = TIM_OutputState_Enable;
  sConfigOC.TIM_Pulse = 0;

  sConfigOC.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OC1Init(TIM1, &sConfigOC);

  // Default output state
  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

  // see gpioTableActivate() TIM1_CH1 entry: the timer output will be asserted via this GPIO
  
  // Enable the output - MOE is being set
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
  // Enable Timer1 CC output
  TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
  
  // Master Mode selection 
  TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update); // this triggers internal trigger 0. Used by timer8 below

  // Select the Master Slave Mode
  TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);

  // Timer1 enable counter
  __HAL_TIM_ENABLE(&htim1);
  // Without this, Timer1 6.76 Mhz didn't work (PortA, Pin8)
  TIM_CCPreloadControl(TIM1, DISABLE);

#if 1
  //
  // synchronize timer8 on timer1
  //
  // Slave Mode selection: TIM8
  TIM_SelectSlaveMode(TIM8, TIM_SlaveMode_Gated);
  TIM_SelectInputTrigger(TIM8, TIM_TS_ITR0); // timer8 is triggered on internal trigger 0

  // Timer8 frequency  is 1/4 of timer1

  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Period = 3;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

  // Timer8 Output Compare Toggle Mode configuration: Channel4
  sConfigOC.TIM_OCMode = TIM_OCMode_Toggle;
  sConfigOC.TIM_OutputState = TIM_OutputState_Enable;
  sConfigOC.TIM_Pulse = 1;
  sConfigOC.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OC4Init(TIM8, &sConfigOC);
  // Default output state
  TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);
  
  // Enable the output - MOE is being set
  TIM_CtrlPWMOutputs(TIM8, ENABLE);
  // Enable Timer8 CC output
  TIM_CCxCmd(TIM8, TIM_Channel_4, TIM_CCx_Enable);
  
  // Timer8 enable counter
  __HAL_TIM_ENABLE(&htim8);
  TIM_CCPreloadControl(TIM8, DISABLE);  
#endif

#ifdef RF_TEST_11_08_16
  //
  // synchronize timer4 on timer1
  //
  // Slave Mode selection: TIM4
  TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Gated);
  TIM_SelectInputTrigger(TIM4, TIM_TS_ITR0); // timer4 is triggered on internal trigger 0

  hwdriversPwmControl(TYPES_DISABLE);


  // Default output state
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
  
  // Enable the output - MOE is being set
  TIM_CtrlPWMOutputs(TIM4, ENABLE);
  // Enable Timer4 CC output
  TIM_CCxCmd(TIM4, TIM_Channel_2, TIM_CCx_Enable);
  
  // Timer4 enable counter
  __HAL_TIM_ENABLE(&htim4);
  TIM_CCPreloadControl(TIM4, ENABLE);  
  
#if 1 
  //TIM5 disable timer on start.
  __HAL_TIM_DISABLE(&htim5);
  
  // Timer5 set default time base structure.
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Period = 3;
  TIM_TimeBaseStructure.TIM_Prescaler = 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

  // Timer5 config to channel 1. 
  sConfigOC.TIM_OCMode = TIM_OCMode_Toggle;
  sConfigOC.TIM_OutputState = TIM_OutputState_Enable;
  sConfigOC.TIM_Pulse = 1;
  sConfigOC.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OC1Init(TIM5, &sConfigOC);
  
  TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);
  
    // Enable the output - MOE is being set
  TIM_CtrlPWMOutputs(TIM5, ENABLE);
  // Enable Timer5 CC output
  TIM_CCxCmd(TIM5, TIM_Channel_1, TIM_CCx_Disable);
  
  // Timer5 enable counter
  TIM_CCPreloadControl(TIM5, DISABLE);
  __HAL_TIM_ENABLE(&htim5);
#endif      

#if 1 //Init buzzer and vibrator.
  buzzerInit();
  vibratorInit();
#endif  
  

#endif
#endif // USE_TIM1

  // Enable the DMA Stream IRQ Channel 
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = INTERRUPT_PRIORITY_VALID_FROMISR_API;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = INTERRUPT_PRIORITY_VALID_FROMISR_API;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);


#endif

//  buzzerInit();
//  vibratorInit();


}



/******************************************************************************
 * @brief  void hwdriversAdcConfig(uint8_t * BuffPtr1, uint8_t * BuffPtr2, uint16 BuffersLength)
 * @param  
 * @retval 
 ******************************************************************************/
void hwdriversAdc1Config(uint8_t * BuffPtr1, uint8_t * BuffPtr2, uint16_t BuffersLength)
{



//	HAL_DMAEx_MultiBufferStart(&hdma_adc1, (uint32_t)&ADC1->DR, (uint32_t)BuffPtr1, (uint32_t)BuffPtr2, BuffersLength);
	HAL_ADC_Start_DMA(&hadc1, BuffPtr1, BuffersLength);

	//  __HAL_TIM_ENABLE(&htim2);

#if 0





	ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef       DMA_InitStructure;





  // gpio initialization is done in gpioTableActivate()

  // Configure DMA2 Stream4 to read from ADC1 Data register and to transfer data to circular double buffer 
  DMA_DeInit(DMA2_Stream4);
  DMA_StructInit (&DMA_InitStructure);
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)BuffPtr1;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = BuffersLength; // Count of 16-bit words
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream4, &DMA_InitStructure);
  DMA_DoubleBufferModeConfig(DMA2_Stream4, (uint32_t)BuffPtr2, DMA_Memory_0);
  DMA_DoubleBufferModeCmd(DMA2_Stream4, ENABLE);
  
  // Enable DMA transfer complete interrupt
  DMA_ITConfig(DMA2_Stream4, DMA_IT_TC, ENABLE);
  // Enable DMA2_Stream4
  DMA_Cmd(DMA2_Stream4, ENABLE);
  
  // ADC Common part Init 
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);
  
  // Configure ADC1 resolution and define the trigger source to be Timer2 trigger out event
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE; // 1 Channel
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // Conversions Triggered
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC1, &ADC_InitStructure);
  
  // ADC1 regular channel 1 configuration */
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_3Cycles);
  // Enable DMA request after last transfer (Single-ADC mode)
  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
  ADC_ITConfig(ADC1, ADC_IT_OVR, ENABLE);
  // Enable ADC1 DMA 
  ADC_DMACmd(ADC1, ENABLE);
  // Enable ADC1 
  ADC_Cmd(ADC1, ENABLE);

#endif

}


void hwdriversModemAdcSignalSourceSelect(uint8_t AdcSignalSourceSelect)
{
  
#if 0
  // ADC1 regular channel 1 configuration */
  ADC_RegularChannelConfig(ADC1, AdcSignalSourceSelect, 1, ADC_SampleTime_3Cycles);

#endif
}


void hwdriversAdc2Config(void)
{

//	HAL_ADC_Start_DMA(&hadc2, (uint16_t*)&ADCConvertedValueUnFiltered, (sizeof(ADCConvertedValueUnFiltered) / sizeof(uint16_t)) - 1);
  // ADC for TX_MONITOR only TODO: Test this reference
	HAL_ADC_Start_DMA(&hadc3, ((uint16_t*)&ADCConvertedValueUnFiltered) + 3, 1);



#if 0


	ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
  NVIC_InitTypeDef      NVIC_InitStructure;
 
  
  /**
     Configure the DMA
  */
  //==Configure DMA2 - Stream 3
  DMA_DeInit(DMA2_Stream3);  //Set DMA registers to default values
  DMA_InitStructure.DMA_Channel = DMA_Channel_1;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC2->DR; //Source address
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)ADCConvertedValueUnFiltered; //Destination address
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = sizeof(ADCConvertedValueUnFiltered) / sizeof(uint16_t) ; //Buffer size
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //source size - 16bit
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; // destination size = 16b
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream3, &DMA_InitStructure); //Initialize the DMA
  DMA_Cmd(DMA2_Stream3, ENABLE); //Enable the DMA2 - Stream 3
 
  /**
     Config the ADC2
  */
  // ADC_DeInit();                                      // Not allowed, caused the ADC1 to be reset 
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;   
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_NbrOfConversion = sizeof(ADCConvertedValueUnFiltered) / sizeof(uint16_t) ;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;          // 1=scan more that one channel in group
  ADC_Init(ADC2,&ADC_InitStructure);

  
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInit(&ADC_CommonInitStructure);
 
  // Tx_monitor - Autopower mechanism feedback, needs to be sampled in rate of 250 Hz
  ADC_RegularChannelConfig(ADC2,ADC_Channel_7 ,1,ADC_SampleTime_480Cycles);
  // Output stage Upper NTC - Designed to monitor the temperature of the output stage 
  ADC_RegularChannelConfig(ADC2,ADC_Channel_5,2,ADC_SampleTime_480Cycles);
  // Output stage Lower NTC - Designed to monitor the temperature of the output stage
  ADC_RegularChannelConfig(ADC2,ADC_Channel_6,3,ADC_SampleTime_480Cycles);
  // VBAT Voltage  measurment
  ADC_RegularChannelConfig(ADC2,ADC_Channel_9,4,ADC_SampleTime_480Cycles);
  // DC input voltage measurment
  // TODO: Removed since we are testing PC0 Interrupt
  ADC_RegularChannelConfig(ADC2,ADC_Channel_10,5,ADC_SampleTime_480Cycles);
  // VBAT current measurement
  ADC_RegularChannelConfig(ADC2,ADC_Channel_12,6,ADC_SampleTime_480Cycles);

  ADC_DMARequestAfterLastTransferCmd(ADC2, ENABLE);

  // Enable the DMA Stream IRQ Channel 
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = INTERRUPT_PRIORITY_VALID_FROMISR_API;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = INTERRUPT_PRIORITY_VALID_FROMISR_API;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // Enable DMA transfer complete interrupt
  DMA_ITConfig(DMA2_Stream3, DMA_IT_TC, ENABLE);

  ADC_DMACmd(ADC2, ENABLE); //Enable ADC2 DMA 
  ADC_Cmd(ADC2, ENABLE);   // Enable ADC2 
//  ADC_SoftwareStartConv(ADC2); // Start ADC2 conversion for the first time

  /* 
     configure and start the adc timer 
  */

#endif





  hwdriversBattMeasureSeqState = HWDRIVERS_BATT_MEASURE_IDLE;

//  xTimerStart(xTimerCreate("adc2Timer", // Just a text name, not used by the kernel.
//                           4, // The timer period in ticks.
//                           pdTRUE, // The timers will auto-reload themselves when they expire.
//                           (void *)0,
//                           adc2TimerCallback
//                           ), 0);



}

//void hwdriversI2CLinesGpioConfigure(hardwaredriversI2CLinesState_T i2cLinesState)
//{
//	GPIO_InitTypeDef  GPIO_InitStruct;
//
//	return;
//
//  switch(i2cLinesState)
//  {
//  case HWDRIVERS_I2C_LINES_AF:
//	  GPIO_InitStruct.Pin = SNS_I2C_SCL_Pin|SNS_I2C_SDA_Pin;
//	  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
//	  GPIO_InitStruct.Pull = GPIO_PULLUP;
//	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//	  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
//	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//	  HAL_GPIO_WritePin (GPIOB,  SNS_I2C_SDA_Pin, GPIO_PIN_SET);
//	 break;
//  case HWDRIVERS_I2C_OUTPUT_TOGGLE_HIGH:
//	  GPIO_InitStruct.Pin = SNS_I2C_SCL_Pin|SNS_I2C_SDA_Pin;
//	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	  GPIO_InitStruct.Pull = GPIO_PULLUP;
//	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
////	  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
//	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//	  HAL_GPIO_WritePin (GPIOB,  SNS_I2C_SDA_Pin, GPIO_PIN_SET);
//    break;
//  case HWDRIVERS_I2C_OUTPUT_TOGGLE_LOW:
//	GPIO_InitStruct.Pin = SNS_I2C_SCL_Pin|SNS_I2C_SDA_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStruct.Pull = GPIO_PULLUP;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
////	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
//	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//	HAL_GPIO_WritePin (GPIOB,  SNS_I2C_SDA_Pin, GPIO_PIN_RESET);
//    break;
//  default:
//	  break;
//  }
//}


//void hwdriversVinGpioConfigure(commSleepState_T SleepState)
//{
//
//  switch(SleepState)
//  {
//  case COMM_SLEEP_STATE_STOP_MODE:
//	SavedRelayState = GPIOE->ODR;
//
//  HAL_GPIO_WritePin(GPIOE, RELAY_CONTROL_EN_Pin|RELAY_CONTROL_0_Pin|RELAY_CONTROL_1_Pin
//                          |RELAY_CONTROL_2_Pin|RELAY_CONTROL_3_Pin|RELAY_CONTROL_4_Pin|RELAY_CONTROL_5_Pin
//                          |RELAY_CONTROL_6_Pin|RELAY_CONTROL_7_Pin, GPIO_PIN_RESET);
//
//    break;
//  case COMM_SLEEP_STATE_NORMAL:
//
//    HAL_GPIO_WritePin(GPIOE, SavedRelayState, GPIO_PIN_SET);
//    break;
//  }
//}


 

//static void adc2TimerCallback(TimerHandle_t pxTimer )
//{
//  switch(hwdriversBattMeasureSeqState)
//  {
//  case HWDRIVERS_BATT_MEASURE_IDLE:
//    // Delay one timer period before starting ADC2 measurement
//    hwdriversBattMeasureSeqState = HWDRIVERS_BATT_MEASURE_START;
//    break;
//
//  case HWDRIVERS_BATT_MEASURE_START:
//    // Start ADC2 scan mode, the battery voltage measurement switch will be turned off by ADC2 DMA completion interrupt
//    HAL_ADC_Start_DMA(&hadc2, (uint16_t*)&ADCConvertedValueUnFiltered, (sizeof(ADCConvertedValueUnFiltered) / sizeof(uint16_t)) - 1);
//    // ADC for TX_MONITOR only TODO: Test this reference
//    HAL_ADC_Start_DMA(&hadc3, ((uint16_t*)&ADCConvertedValueUnFiltered) + 3, 1);
//    hwdriversBattMeasureSeqState = HWDRIVERS_BATT_MEASURE_IDLE;
//    break;
//  }
//
//}









void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
	if(hadc == &hadc1)
	{
		mainAdcDmaTransferCompleteIsr(1);
	}
//	else
//		if(hadc == &hadc2)
//			adc2DmaCompleteIsr();


  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_ADC_ConvCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Regular conversion half DMA transfer callback in non blocking mode
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
	  /* Prevent unused argument(s) compilation warning */
	if(hadc == &hadc1)
		mainAdcDmaTransferCompleteIsr(1);


  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_ADC_ConvHalfCpltCallback could be implemented in the user file
   */
}



/*********************************************************************
*** void DMA2_Stream3_IRQHandler(void)
*
*  ADC2 DMA Completion Interrupt
*********************************************************************/
void adc2DmaCompleteIsr(void)  // called when dma finishes with the adc
{
  int i;

    //perform simple IIR on the values measured
    for( i=0; i< (sizeof( ADCConvertedValue)/sizeof(uint16_t)); i++)
    {
      ADCConvertedValue[i] = (((ADCConvertedValueUnFiltered[i] * 10)) + ((ADCConvertedValue[i] * 70))) / 80;
    }
    // End of ADC2 scan measurment DMA transfer
    // Turn off the Vbatt measurement control to save energy in the voltage divider
    // process the Autopower FSM
    // autopowerFsm((float)(ADCConvertedValue[HWDRIVERS_TX_MONITOR]) /*HWDRIVERS_ADC_REF_VOLTAGE*ADCConvertedValue[HWDRIVERS_TX_MONITOR])/HWDRIVERS_ADC_RESOLUTION*/, 2 /*(float)(HWDRIVERS_ADC_REF_VOLTAGE*273)/HWDRIVERS_ADC_RESOLUTION*/);

}



void hwdriversI2cConfig(void)
{
#ifdef USE_I2C
  
  //chargerControl( CHARGER_CONTROL_ENABLE);
  
//  i2cworkSchedulerInit();
  //ledsInit();
  //LedsSet(LEDS_INDEX_4, LEDS_COLOR_RED, LEDS_STATE_BLINK, 30, 0, NULL);
  //LedsSet(LEDS_INDEX_1, LEDS_COLOR_RED, LEDS_STATE_ON, 3, 7, NULL);
  
  //LedsSet(0, LEDS_COLOR_GREEN, LEDS_STATE_ON, 1, 10);
  //LedsPatternSet(LEDS_PATTERN5);
  //LedsSet(LEDS_INDEX_4, LEDS_COLOR_RED, LEDS_STATE_ON, 0, 0, NULL);
 // LedsBatteryLowLedState(LEDS_BATTERY_RED_ON, 500, 5000);
  //LedsPatternSet(LEDS_PATTERN1);  
   //LedsPatternSet(LEDS_PATTERN_IDLE); 
  
  //LedsBatteryLedRedState(LEDS_BATTERY_RED_ON, 3000);
  
#endif

}

void hwdriversSpiFlashConfig(void)
{
  
  // Check the spiFlash is connected and verify its page size.
  // Use the client server mechanism but block till the operation completes
//  spiflashInit();
}


void ioExpanderConfig(void)
{
#ifdef USE_I2C
#ifdef USE_IO_EXPANDER
  // init the pca9655e io expander
  pca9655eConfig();
#endif
#endif
}

#ifdef SPI_SAME_BOARD_TEST

/******************************************************************************
 * @brief  void hwdriversSpiConfig()
 ******************************************************************************/
void hwdriversSpiConfig()
#if 0
{
  //
  // experimenting with master and slave on NUCLEO F401RE
  //
  TM_SPI_Init(SPI_MASTER, TM_SPI_PinsPack_2); // master

  TM_SPI_DMA_Init(SPI_MASTER);
#endif
}

void hwdriversSpiFlush(SPI_TypeDef *SPIx)
{
#if 0
	SPIx->CR1 &= ~SPI_CR1_SPE;
  vTaskDelay(1);
  SPIx->CR1 |= SPI_CR1_SPE;
#endif
}

void hwdriversSpiDmaTx(SPI_TypeDef *SPIx, uint8_t *s, uint16_t length)
{
#if 0
  while(TM_SPI_DMA_Working(SPIx));
  TM_SPI_DMA_Send(SPIx, s, length);
  while(TM_SPI_DMA_Working(SPIx));
#endif
}

void hwdriversSpiDmaRx(SPI_TypeDef *SPIx, uint8_t *s, uint16_t length)
{
#if 0
  while(TM_SPI_DMA_Working(SPIx));
  TM_SPI_DMA_Receive(SPIx, s, length);
  while(TM_SPI_DMA_Working(SPIx));
#endif
}

#endif


/******************************************************************************
 * @brief  void hwdriversAdc1IRQHandler(void)
 * @param  
 * @retval 
 ******************************************************************************/
void hwdriversAdc1IRQHandler(void)
{

	// TODO:

#if 0
	if(ADC_GetITStatus(ADC1, ADC_IT_OVR))
  {
    ADC_ClearITPendingBit(ADC1, ADC_IT_OVR);
  }
  if(ADC_GetITStatus(ADC1, ADC_IT_EOC))
  {
  }
#endif
}





/******************************************************************************
 * @brief  void hwdriversClockInit()
 * @param  
 * @retval 
 ******************************************************************************/
void hwdriversClockInit()
{
	// set pc1=0  for the external oscillator to work
	HAL_GPIO_WritePin(GPIOC, MAIN_OSC_ENABLE_Pin, GPIO_PIN_RESET);
	//	gpioTableActivate(PC1);
  
	// Activate second regulator
	//TODO: What is PC13
	// gpioTableActivate(PC13);

#if 0

  // Enable HSE bypass mode as we are using oscilator and not crystal
  RCC_HSEConfig(RCC_HSE_Bypass);
  
  // Enable PLL - Not needed in this example
  //RCC_PLLCmd(DISABLE);
  
  // Configure the HSE's multipliers and dividers
  //RCC_PLLConfig(RCC_PLLSource_HSE, 16, 64, 2, 0);
  
  // Enable PLL - Not needed in this example
  //RCC_PLLCmd(ENABLE);
  // Wait till the PLL is ready, Not needed in this example
  //while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {}
  
  // Select HSE as clock source
  RCC_SYSCLKConfig(RCC_SYSCLKSource_HSE);
  
  // Set HCLK, PCLK1, and PCLK2 dividers 
  RCC_HCLKConfig(RCC_SYSCLK_Div1);
  RCC_PCLK1Config(RCC_HCLK_Div2);
  RCC_PCLK2Config(RCC_HCLK_Div1);
  
  // Set ADC clk to 9MHz (14MHz max, 18MHz default)
  //RCC_ADCCLKConfig(RCC_PCLK2_Div4);

  // Enable relevant peripheral clocks 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
  RCC_AHB1PeriphClockCmd(ADCx_CHANNEL_GPIO_CLK, ENABLE);  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
  // TIM1- 6.72Mhz main Tx clock 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);    
  // TIM2 - ADC1 Sampling timer - slave to TIM1 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  // TIM3 - Frequency Monitoring counter
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  // TIM4 - Buzzer PWM
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  // TIM5 - Modulator 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
  // TIM6 - Modulator 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
  // TIM7 - System Timer
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

  
  
  // Enable UASRT1 clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

  // Enable UASRT2 clock
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  // Enable UASRT3 clock
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  
  // Enable UASRT6 clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

  // Enable NRG Clock
  RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);

#endif
}





/******************************************************************************
*** @brief  hwdriversUartsConfig(void)
* @param  
* @retval 
******************************************************************************/
void hwdriversUartsConfig(void)
{

//	// Flags to mark the Uart Tx DMA is busy
//	Uart1Dma2Stream7Busy = false;
//	Uart2Dma1Stream6Busy = false;
//	Uart3Dma1Stream3Busy = false;
//	Uart6Dma2Stream6Busy = false;
//
//
//#if 0
//	USART_InitTypeDef USART_InitStructure;
//
//
//  // Flags to mark the Uart Tx DMA is busy
//  Uart1Dma2Stream7Busy = false;
//  Uart2Dma1Stream6Busy = false;
//  Uart3Dma1Stream3Busy = false;
//  Uart6Dma2Stream6Busy = false;
//
//
//  /* USART resources configuration (Clock, GPIO pins and USART registers) ----*/
//  /* USART configured as follow:
//     - BaudRate = 115200 baud
//     - Word Length = 8 Bits
//     - One Stop Bit
//     - No parity
//     - Hardware flow control disabled (RTS and CTS signals)
//     - Receive and transmit enabled
//  */
//
//  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//  USART_InitStructure.USART_StopBits = USART_StopBits_1;
//  USART_InitStructure.USART_Parity = USART_Parity_No;
//  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//
//  // USART1 configuration - Modem GUI
//  USART_InitStructure.USART_BaudRate = 250000;
//  USART_Init(USART1, &USART_InitStructure);
//  // Enable the USART1
//  USART_Cmd(USART1, ENABLE);
//
//  // USART2 configuration - Cellular Modem
//  USART_InitStructure.USART_BaudRate = 250000;
//  USART_Init(USART2, &USART_InitStructure);
//  // Enable the USART2
//  USART_Cmd(USART2, ENABLE);
//
//  // USART3 configuration - BLE
//  USART_InitStructure.USART_BaudRate = 115200;
//  USART_Init(USART3, &USART_InitStructure);
//  // Enable the USART3
//  USART_Cmd(USART3, ENABLE);
//
//  // USART6 configuration - RS422
//  USART_InitStructure.USART_BaudRate = 250000;
//  USART_Init(USART6, &USART_InitStructure);
//  // Enable the USART6
//  USART_Cmd(USART6, ENABLE);
//
//#endif
//
//
////  Uart1CicularBufferIndex=0;
//  Uart2CicularBufferIndex=0;
//  Uart3CicularBufferIndex=0;
//  Uart6CicularBufferIndex=0;
//
//
////  HAL_UART_Receive_DMA(&huart1, Uart1RxDmaCicularBuffer, UART_RX_DMA_CIRCULAR_BUFFER_LEGTH);
//#ifdef USE_CELLMODEM
//  HAL_UART_Receive_DMA(&huart2, Uart2RxDmaCicularBuffer, UART_RX_DMA_CIRCULAR_BUFFER_LEGTH);
//#endif
//  HAL_UART_Receive_DMA(&huart3, Uart3RxDmaCicularBuffer, UART_RX_DMA_CIRCULAR_BUFFER_LEGTH);
//  HAL_UART_Receive_DMA(&huart6, Uart6RxDmaCicularBuffer, UART_RX_DMA_CIRCULAR_BUFFER_LEGTH);
}



void hwdriversReInitUartBleRxDma()
{
//	HAL_UART_Receive_DMA(&huart3, Uart3RxDmaCicularBuffer, UART_RX_DMA_CIRCULAR_BUFFER_LEGTH);
//	__HAL_UART_DISABLE_IT(&huart3, UART_IT_ERR);
}


void hwdriversReInitUart1PcRxDma()
{
//	  HAL_UART_Receive_DMA(&huart1, Uart1RxDmaCicularBuffer, UART_RX_DMA_CIRCULAR_BUFFER_LEGTH);
}


/******************************************************************************
*** @brief  void hwdriversUartsDmaRxConfig()
* @param  
* @retval 
******************************************************************************/
void hwdriversUartsDmaRxConfig()
{
//#if 0
//	DMA_InitTypeDef DMA_InitStructure;
//
//  // USART1_RX, DMA2, Stream2,  Channel 4, see DM00031020-RefManual, Rev 1, page 308
//  DMA_DeInit(DMA2_Stream2);
//
//  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
//  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Uart1RxDmaCicularBuffer;
//  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
//  DMA_InitStructure.DMA_BufferSize = UART_RX_DMA_CIRCULAR_BUFFER_LEGTH;
//  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//  DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
//  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
//  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//  // Init DMA2
//  DMA_Init(DMA2_Stream2, &DMA_InitStructure);
//  // USART1 Rx will drive DMA2 dma Request signal
//  USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
//  // Enable DMA Stream Transfer Complete interrupt (When all data has been transfferred)
//  DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, ENABLE);
//  // Start DMA2: Will work in the background filling the buffer in circular mode
//  DMA_Cmd(DMA2_Stream2, ENABLE);
//
//
//  // USART2_RX, DMA1, Stream5,  Channel 4, see DM00031020-RefManual, Rev 1, page 308
//  DMA_DeInit(DMA1_Stream5);
//
//  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
//  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Uart2RxDmaCicularBuffer;
//  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
//  DMA_InitStructure.DMA_BufferSize = UART_RX_DMA_CIRCULAR_BUFFER_LEGTH;
//  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//  DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
//  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
//  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//  // Init DMA1
//  DMA_Init(DMA1_Stream5, &DMA_InitStructure);
//  // USART2 Rx will drive DMA2 dma Request signal
//  USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
//  // Enable DMA Stream Transfer Complete interrupt (When all data has been transfferred)
//  DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, ENABLE);
//  // Start DMA1: Will work in the background filling the buffer in circular mode
//  DMA_Cmd(DMA1_Stream5, ENABLE);
//
//
//  // USART3_RX, DMA1, Stream1,  Channel 4, see DM00031020-RefManual, Rev 1, page 308
//  DMA_DeInit(DMA1_Stream1);
//
//  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
//  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Uart3RxDmaCicularBuffer;
//  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
//  DMA_InitStructure.DMA_BufferSize = UART_RX_DMA_CIRCULAR_BUFFER_LEGTH;
//  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//  DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
//  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
//  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//  // Init DMA1
//  DMA_Init(DMA1_Stream1, &DMA_InitStructure);
//  // USART3 Rx will drive DMA1 dma Request signal
//  USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
//  // Enable DMA Stream Transfer Complete interrupt (When all data has been transfferred)
//  DMA_ITConfig(DMA1_Stream1, DMA_IT_TC, DISABLE);
//  // Start DMA2: Will work in the background filling the buffer in circular mode
//  DMA_Cmd(DMA1_Stream1, ENABLE);
//
//  // USART6_RX, DMA2, Stream1,  Channel 5, see DM00031020-RefManual, Rev 1, page 308
//  DMA_DeInit(DMA2_Stream1);
//
//  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART6->DR;
//  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Uart6RxDmaCicularBuffer;
//  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
//  DMA_InitStructure.DMA_BufferSize = UART_RX_DMA_CIRCULAR_BUFFER_LEGTH;
//  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//  DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
//  DMA_InitStructure.DMA_Channel = DMA_Channel_5;
//  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//  // Init DMA2
//  DMA_Init(DMA2_Stream1, &DMA_InitStructure);
//  // USART6 Rx will drive DMA2 dma Request signal
//  USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
//  // Enable DMA Stream Transfer Complete interrupt (When all data has been transfferred)
//  DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, ENABLE);
//  // Start DMA2: Will work in the background filling the buffer in circular mode
//  DMA_Cmd(DMA2_Stream1, ENABLE);
//
//#endif
//
//
//  //HAL_UART_Receive_DMA(&huart1, Uart1RxDmaCicularBuffer, UART_RX_DMA_CIRCULAR_BUFFER_LEGTH);
//#ifdef USE_CELLMODEM
//  HAL_UART_Receive_DMA(&huart2, Uart2RxDmaCicularBuffer, UART_RX_DMA_CIRCULAR_BUFFER_LEGTH);
//#endif
//  HAL_UART_Receive_DMA(&huart3, Uart3RxDmaCicularBuffer, UART_RX_DMA_CIRCULAR_BUFFER_LEGTH);
//  //HAL_UART_Receive_DMA(&huart6, Uart6RxDmaCicularBuffer, UART_RX_DMA_CIRCULAR_BUFFER_LEGTH);


}


/******************************************************************************
*** @brief  void hwdriversNvicConfig(void)
* @param  
* @retval 
******************************************************************************/
void hwdriversNvicConfig(void)
{
	hwdriversPVDConfig();

#if 0

	NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Configure the Priority Group to 2 bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  
  /* Enable the USART1 TX DMA Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = INTERRUPT_PRIORITY_VALID_FROMISR_API;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  /* Enable the USART1 RX DMA Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = INTERRUPT_PRIORITY_VALID_FROMISR_API;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable the USART2 TX DMA Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = INTERRUPT_PRIORITY_VALID_FROMISR_API;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  /* Enable the USART2 RX DMA Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = INTERRUPT_PRIORITY_VALID_FROMISR_API;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);


  /* Enable the USART3 TX DMA Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = INTERRUPT_PRIORITY_VALID_FROMISR_API;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  /* Enable the USART3 RX DMA Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = INTERRUPT_PRIORITY_VALID_FROMISR_API;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  
 /* Enable the USART6 TX DMA Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = INTERRUPT_PRIORITY_VALID_FROMISR_API;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  /* Enable the USART6 RX DMA Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = INTERRUPT_PRIORITY_VALID_FROMISR_API;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
/* Enable the SPI1 TX DMA Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = INTERRUPT_PRIORITY_VALID_FROMISR_API;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable the SPI1 RX DMA Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = INTERRUPT_PRIORITY_VALID_FROMISR_API;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);


  
  // Timer5 interrupt 
  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = INTERRUPT_PRIORITY_VALID_FROMISR_API;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

    // Timer6 interrupt 
  NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = INTERRUPT_PRIORITY_VALID_FROMISR_API;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  
  // timer7 interrupt 
  NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = INTERRUPT_PRIORITY_VALID_FROMISR_API;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

#ifdef BOARD_4_2
  // external interrupts  - PUSH BUTTON
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = INTERRUPT_PRIORITY_VALID_FROMISR_API;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
#endif
#ifdef BOARD_4_1
  // external interrupts  
  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = INTERRUPT_PRIORITY_VALID_FROMISR_API;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
#endif

  // external interrupts  
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = INTERRUPT_PRIORITY_VALID_FROMISR_API;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Configure the I2C event priority */
  NVIC_InitStructure.NVIC_IRQChannel                   = I2C1_EV_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = INTERRUPT_PRIORITY_VALID_FROMISR_API;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
#if 1
  hwdriversPVDConfig();
#endif


#endif
}

///* Configure pins to be interrupts */
//void Configure_PA0(void) {
//  /* Set variables used */
//  GPIO_InitTypeDef GPIO_InitStruct;
//  EXTI_InitTypeDef EXTI_InitStruct;
//  NVIC_InitTypeDef NVIC_InitStruct;
//  
//  /* Set pin as input */
////  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
////  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
////  GPIO_InitStruct.GPIO_PIN = GPIO_PIN_0;
////  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
////  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
////  GPIO_Init(GPIOA, &GPIO_InitStruct);
//  
//  /* Tell system that you will use PA0 for EXTI_Line0 */
//  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
//  
//  /* PD0 is connected to EXTI_Line0 */
//  EXTI_InitStruct.EXTI_Line = EXTI_Line0;
//  /* Enable interrupt */
//  EXTI_InitStruct.EXTI_LineCmd = ENABLE;
//  /* Interrupt mode */
//  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
//  /* Triggers on rising and falling edge */
//  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
//  /* Add to EXTI */
//  EXTI_Init(&EXTI_InitStruct);
//  
//  /* Add IRQ vector to NVIC */
//  /* PD0 is connected to EXTI_Line0, which has EXTI0_IRQn vector */
//  NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
//  /* Set priority */
//  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = INTERRUPT_PRIORITY_VALID_FROMISR_API;
//  /* Set sub priority */
//  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
//  /* Enable interrupt */
//  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//  /* Add to NVIC */
//  NVIC_Init(&NVIC_InitStruct);
//}




/**************************************************************************
 *** ReturnCode_T hwdriversTxProcessing()
 *
 *
 *
 ***************************************************************************/
ReturnCode_T hwdriversTxProcessing()
{
  transQEntry_t myQueueData;
/*
  // UART1: Check the tx queue only if the last DMA session has been completed
  if(!Uart1Dma2Stream7Busy)
    if(transQTop(UartTransQueueHandle1, &myQueueData) == TRANSQ_OK)
    {
      // Starting the transfer. Using Basic Mode, The DMA completion interrupt will dequeue the element from the queue and free associated payload buffers
      hwdriversUart1DmaTx(myQueueData.UserDataPtr, myQueueData.Length);
    }
*/

  // UART2: Check the tx queue only if the last DMA session has been completed
  if(!Uart2Dma1Stream6Busy)
    if(transQTop(UartTransQueueHandle2, &myQueueData) == TRANSQ_OK)
    {
      // Starting the transfer. Using Basic Mode, The DMA completion interrupt will dequeue the element from the queue and free associated payload buffers
#ifdef USE_CELLMODEM
      hwdriversUart2DmaTx(myQueueData.UserDataPtr, myQueueData.Length);
#endif
    }

  // UART3: Check the tx queue only if the last DMA session has been completed
  if(!Uart3Dma1Stream3Busy)
    if(transQTop(UartTransQueueHandle3, &myQueueData) == TRANSQ_OK)
    {
      // Starting the transfer. Using Basic Mode, The DMA completion interrupt will dequeue the element from the queue and free associated payload buffers
      hwdriversUart3DmaTx(myQueueData.UserDataPtr, myQueueData.Length);
    }
  
  // UART6: Check the tx queue only if the last DMA session has been completed
  if(!Uart6Dma2Stream6Busy)
    if(transQTop(UartTransQueueHandle6, &myQueueData) == TRANSQ_OK)
    {
      // Starting the transfer. Using Basic Mode, The DMA completion interrupt will dequeue the element from the queue and free associated payload buffers
      hwdriversUart6DmaTx(myQueueData.UserDataPtr, myQueueData.Length);
    }
 
  return(RETURNCODE_OK);
}

/******************************************************************************
*** @brief  void hwdriversUart2DmaTx(uint8_t *s, uint16_t length)
*   @param  
*   @retval 
******************************************************************************/
void hwdriversUart2DmaTx(uint8_t *s, uint16_t length)
{

//	HAL_UART_Transmit_DMA(&huart2, s, length);
	  Uart2Dma1Stream6Busy = true;
#if 0

	DMA_InitTypeDef DMA_InitStructure;
 
  Uart2Dma1Stream6Busy = true;

  // USART2_TX, DMA1, Stream6,  Channel 4, see DM00031020-RefManual, Rev 1, page 308
  DMA_DeInit(DMA1_Stream6);
  DMA_StructInit (&DMA_InitStructure);
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)(s);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_BufferSize = length; 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream6, &DMA_InitStructure);
  
  // Enable DMA transfer complete interrupt
  DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, ENABLE);
  USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
  
  // Enable DMA2_Stream6
  DMA_Cmd(DMA1_Stream6, ENABLE);

#endif
}  
  
/******************************************************************************
*** @brief  void hwdriversUart3DmaTx(uint8_t *s, uint16_t length)
*   @param  
*   @retval 
******************************************************************************/
void hwdriversUart3DmaTx(uint8_t *s, uint16_t length)
{

//	HAL_UART_Transmit_DMA(&huart3, s, length);
//	Uart3Dma1Stream3Busy = true;
//#if 0
//
//	MA_InitTypeDef DMA_InitStructure;
//
//  Uart3Dma1Stream3Busy = true;
//
//  // USART3_TX, DMA1, Stream3,  Channel 4, see DM00031020-RefManual, Rev 1, page 308
//  DMA_DeInit(DMA1_Stream3);
//  DMA_StructInit (&DMA_InitStructure);
//  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
//  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)(s);
//  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
//  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
//  DMA_InitStructure.DMA_BufferSize = length;
//  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
//  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
//  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//  DMA_Init(DMA1_Stream3, &DMA_InitStructure);
//
//  // Enable DMA transfer complete interrupt
//  // void DMA1_Stream3_IRQHandler(void) // USART3_TX
//  DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);
//  USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
//
//  // Enable DMA1_Stream3
//  DMA_Cmd(DMA1_Stream3, ENABLE);
//
//#endif
}  
  

/******************************************************************************
*** @brief  void hwdriversUart6DmaTx(uint8_t *s, uint16_t length)
*   @param  
*   @retval 
******************************************************************************/
void hwdriversUart6DmaTx(uint8_t *s, uint16_t length)
{
//	HAL_UART_Transmit_DMA(&huart6, s, length);
	Uart6Dma2Stream6Busy = true;

#if 0
#if 0
	DMA_InitTypeDef DMA_InitStructure;
 
#ifdef USE_USART6_FOR_DEBUGGING
  while(dma2Stream6Active); // wait for this dma transfer to complete.
#endif

  Uart6Dma2Stream6Busy = true;

  // USART6_TX, DMA2, Stream6,  Channel 5, see DM00031020-RefManual, Rev 1, page 308
  DMA_DeInit(DMA2_Stream6);
  DMA_StructInit (&DMA_InitStructure);
  DMA_InitStructure.DMA_Channel = DMA_Channel_5;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)(s);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART6->DR;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_BufferSize = length; 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream6, &DMA_InitStructure);
  
  // Enable UART6 TX DMA transfer complete interrupt
  DMA_ITConfig(DMA2_Stream6, DMA_IT_TC, ENABLE);
  USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE);
  
  // Enable DMA2_Stream6
  DMA_Cmd(DMA2_Stream6, ENABLE);
  #endif
#endif
}  



//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
//	transQEntry_t myQueueDataPtr;
//
//#ifdef USE_CELLMODEM
//	if(huart == &huart2)
//	{
//		// Transmission completed, Dequeue element from queue and free user allocated if neeeded
//		if(transQDequeueFromISR(UartTransQueueHandle2, &myQueueDataPtr) == TRANSQ_OK)
//		{
//		  if(myQueueDataPtr.UserDataPtr && myQueueDataPtr.MemoryFreeNeeded)
//			xQueueSendFromISR(hwdriversMemFreeQueueHandle, &myQueueDataPtr.UserDataPtr, 0);
//		  Uart2Dma1Stream6Busy = false;
//		}
//	}
//#endif
//	if(huart == &huart6)
//	{
//		// Transmission completed, Dequeue element from queue and free user allocated if needed
//		if(transQDequeueFromISR(UartTransQueueHandle6, &myQueueDataPtr) == TRANSQ_OK)
//		{
//		 if(myQueueDataPtr.UserDataPtr && myQueueDataPtr.MemoryFreeNeeded)
//			xQueueSendFromISR(hwdriversMemFreeQueueHandle, &myQueueDataPtr.UserDataPtr, 0);
//		  Uart6Dma2Stream6Busy = false;
//		}
//	}
//}


void UART_DMAReceiveCplt(DMA_HandleTypeDef *hdma)
{

}






#if 0
/******************************************************************************
*** @brief  void DMA2_Stream7_IRQHandler(void) // USART1_TX
*   @param  
*   @retval 
******************************************************************************/
void DMA2_Stream7_IRQHandler(void) // USART1_TX
{
  /* Test on DMA Transfer Complete interrupt */
  if (__HAL_DMA_GET_FLAG(&hdma_usart1_tx, DMA_FLAG_TCIF3_7))
  {
    /* Clear DMA Transfer Complete interrupt pending bit */
    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_FLAG_TCIF3_7);
    
    transQEntry_t myQueueDataPtr;
    // Transmission completed, Dequeue element from queue and free user allocated if neeeded
    if(transQDequeueFromISR(UartTransQueueHandle1, &myQueueDataPtr) == TRANSQ_OK)
    {
      if(myQueueDataPtr.UserDataPtr && myQueueDataPtr.MemoryFreeNeeded)
        xQueueSendFromISR(hwdriversMemFreeQueueHandle, &myQueueDataPtr.UserDataPtr, 0);
      Uart1Dma2Stream7Busy = false;
    }
  }
}


/******************************************************************************
*** @brief  void DMA2_Stream2_IRQHandler(void) // USART1_RX
*   @param  
*   @retval 
******************************************************************************/
void DMA2_Stream2_IRQHandler(void) // USART1_RX
{
  /* Test on DMA Transfer Complete interrupt */
  if (__HAL_DMA_GET_FLAG(&hdma_usart1_rx, DMA_FLAG_TCIF2_6))
  {
    /* Clear DMA Transfer Complete interrupt pending bit */
	  __HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx, DMA_FLAG_TCIF2_6);
    
    
    /* ... */
  }
}
  

/******************************************************************************
*** @brief  void DMA1_Stream6_IRQHandler(void) // USART2_TX
*   @param  
*   @retval 
******************************************************************************/
void DMA1_Stream6_IRQHandler(void) // USART2_TX
{
  /* Test on DMA Transfer Complete interrupt */
  if (__HAL_DMA_GET_FLAG(&hdma_usart2_tx, DMA_FLAG_TCIF2_6))
  {
    /* Clear DMA Transfer Complete interrupt pending bit */
	  __HAL_DMA_CLEAR_FLAG(&hdma_usart2_tx, DMA_FLAG_TCIF2_6);
    
    transQEntry_t myQueueDataPtr;
    // Transmission completed, Dequeue element from queue and free user allocated if neeeded
    if(transQDequeueFromISR(UartTransQueueHandle2, &myQueueDataPtr) == TRANSQ_OK)
    {
      if(myQueueDataPtr.UserDataPtr && myQueueDataPtr.MemoryFreeNeeded)
        xQueueSendFromISR(hwdriversMemFreeQueueHandle, &myQueueDataPtr.UserDataPtr, 0);
      Uart2Dma1Stream6Busy = false;
    }
  }
}


/******************************************************************************
*** @brief  void DMA1_Stream_IRQHandler(void) // USART2_RX
*   @param  
*   @retval 
******************************************************************************/
void DMA1_Stream5_IRQHandler(void) // USART2_RX
{
  /* Test on DMA Transfer Complete interrupt */
  if (__HAL_DMA_GET_FLAG(&hdma_usart2_tx, DMA_FLAG_TCIF1_5))
  {
    /* Clear DMA Transfer Complete interrupt pending bit */
	  __HAL_DMA_CLEAR_FLAG(&hdma_usart2_rx, DMA_FLAG_TCIF1_5);
    
    
    /* ... */
  }
}


/******************************************************************************
*** @brief  void DMA1_Stream1_IRQHandler(void) // USART3_TX
*   @param  
*   @retval 
******************************************************************************/
void DMA1_Stream3_IRQHandler(void) // USART3_TX
{
  /* Test on DMA Transfer Complete interrupt */
  if (__HAL_DMA_GET_FLAG(&hdma_usart3_tx, DMA_FLAG_TCIF3_7))
  {
     
     transQEntry_t myQueueDataPtr;
    // Transmission completed, Dequeue element from queue and free user allocated if neeeded
    if(transQDequeueFromISR(UartTransQueueHandle3, &myQueueDataPtr) == TRANSQ_OK)
    {
     if(myQueueDataPtr.UserDataPtr && myQueueDataPtr.MemoryFreeNeeded)
        xQueueSendFromISR(hwdriversMemFreeQueueHandle, &myQueueDataPtr.UserDataPtr, 0);
     Uart3Dma1Stream3Busy = false;
    }
   /* Clear DMA Transfer Complete interrupt pending bit */
    __HAL_DMA_CLEAR_FLAG(&hdma_usart3_tx, DMA_FLAG_TCIF3_7);
  }
}


/******************************************************************************
*** @brief  void DMA1_Stream_IRQHandler(void) // USART3_RX
*   @param  
*   @retval 
******************************************************************************/
void DMA1_Stream1_IRQHandler(void) // USART3_RX
{
  /* Test on DMA Transfer Complete interrupt */
  if (__HAL_DMA_GET_FLAG(&hdma_usart3_tx, DMA_FLAG_TCIF1_5))
  {
    /* Clear DMA Transfer Complete interrupt pending bit */
	  __HAL_DMA_CLEAR_FLAG(&hdma_usart3_rx, DMA_FLAG_TCIF1_5);
    
    
    /* ... */
  }
}






/******************************************************************************
*** @brief  void DMA2_Stream6_IRQHandler(void) // USART6_TX
*   @param  
*   @retval 
******************************************************************************/
void DMA2_Stream6_IRQHandler(void) // USART6_TX
{
  /* Test on DMA Transfer Complete interrupt */
  if (__HAL_DMA_GET_FLAG(&hdma_usart6_tx, DMA_FLAG_TCIF2_6))
  {
    /* Clear DMA Transfer Complete interrupt pending bit */
	  __HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx, DMA_FLAG_TCIF2_6);
    
    transQEntry_t myQueueDataPtr;
    // Transmission completed, Dequeue element from queue and free user allocated if neeeded
    if(transQDequeueFromISR(UartTransQueueHandle6, &myQueueDataPtr) == TRANSQ_OK)
    {
     if(myQueueDataPtr.UserDataPtr && myQueueDataPtr.MemoryFreeNeeded)
        xQueueSendFromISR(hwdriversMemFreeQueueHandle, &myQueueDataPtr.UserDataPtr, 0);
      Uart6Dma2Stream6Busy = false;
    }
  }
}


/******************************************************************************
*** @brief  void DMA2_Stream_IRQHandler(void) // USART6_RX
*   @param  
*   @retval 
******************************************************************************/
void DMA2_Stream1_IRQHandler(void) // USART6_RX
{
  /* Test on DMA Transfer Complete interrupt */
  if (__HAL_DMA_GET_FLAG(&hdma_usart6_tx, DMA_FLAG_TCIF1_5))
  {
    /* Clear DMA Transfer Complete interrupt pending bit */
    //    DMA_ClearITPendingBit(DMA2_IT_TC2);
	  __HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_FLAG_TCIF1_5);
    
    
    /* ... */
  }
}

#endif


/******************************************************************************
*** @brief  void hwdriversUart2DmaRxDataProcess() - Cellular Modem
*   @param  
*   @retval 
******************************************************************************/
#ifdef USE_CELLMODEM
void hwdriversUart2DmaRxDataProcess()
{
#if 1
  uint16_t DmaBufferHwIndex = UART_RX_DMA_CIRCULAR_BUFFER_LEGTH - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
  uint16_t i;
  if(Uart2CicularBufferIndex < DmaBufferHwIndex)
  {
    for(i=0; i< DmaBufferHwIndex-Uart2CicularBufferIndex; i++) 
    {
#if COMM_IS_BLE
      auditManageBufferNurseMode(Uart2RxDmaCicularBuffer[Uart2CicularBufferIndex+i]);
#else
      uartDllRxFsm(UARTDLL_UART_2_CELLMODEM, UARTDLL_FSM_STIMULI_INCOMMING_CHAR,Uart2RxDmaCicularBuffer[Uart2CicularBufferIndex+i] );
#endif
    }
      
  }
  if(Uart2CicularBufferIndex > DmaBufferHwIndex)
  {
    for(i=0; i< UART_RX_DMA_CIRCULAR_BUFFER_LEGTH-Uart2CicularBufferIndex; i++) 
    {
#if COMM_IS_BLE
      auditManageBufferNurseMode(Uart2RxDmaCicularBuffer[Uart2CicularBufferIndex+i]);
#else
      uartDllRxFsm(UARTDLL_UART_2_CELLMODEM, UARTDLL_FSM_STIMULI_INCOMMING_CHAR,Uart2RxDmaCicularBuffer[Uart2CicularBufferIndex+i]);
#endif
    }
      
    for(i=0; i< DmaBufferHwIndex; i++) 
    {
#if COMM_IS_BLE
      auditManageBufferNurseMode(Uart2RxDmaCicularBuffer[Uart2CicularBufferIndex+i]);
#else
      uartDllRxFsm(UARTDLL_UART_2_CELLMODEM, UARTDLL_FSM_STIMULI_INCOMMING_CHAR,Uart2RxDmaCicularBuffer[i]);
#endif
    }
      
  }
  
  Uart2CicularBufferIndex = DmaBufferHwIndex;
#endif
}
#endif

/******************************************************************************
*** @brief  void hwdriversUart3DmaRxDataProcess() - BLE
*   @param  
*   @retval 
******************************************************************************/
void hwdriversUart3DmaRxDataProcess()
{
//  uint16_t DmaBufferHwIndex = UART_RX_DMA_CIRCULAR_BUFFER_LEGTH - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
//  uint16_t i;
//  //
////  if(Uart3CicularBufferIndex < DmaBufferHwIndex)
////  {
////    for(i=0; i< DmaBufferHwIndex-Uart3CicularBufferIndex; i++)
//////      bleRxFsm(UARTDLL_FSM_STIMULI_INCOMMING_CHAR, Uart3RxDmaCicularBuffer[Uart3CicularBufferIndex+i]);
////  }
////  if(Uart3CicularBufferIndex > DmaBufferHwIndex)
////  {
////    for(i=0; i< UART_RX_DMA_CIRCULAR_BUFFER_LEGTH-Uart3CicularBufferIndex; i++)
//////      bleRxFsm(UARTDLL_FSM_STIMULI_INCOMMING_CHAR, Uart3RxDmaCicularBuffer[Uart3CicularBufferIndex+i]);
////    for(i=0; i< DmaBufferHwIndex; i++)
//////      bleRxFsm(UARTDLL_FSM_STIMULI_INCOMMING_CHAR, Uart3RxDmaCicularBuffer[i]);
////  }
//  Uart3CicularBufferIndex = DmaBufferHwIndex;
}


/******************************************************************************
*** @brief  void hwdriversUart6DmaRxDataProcess() - RS-485
*   @param  
*   @retval 
******************************************************************************/
void hwdriversUart6DmaRxDataProcess()
{
//  uint16_t DmaBufferHwIndex = UART_RX_DMA_CIRCULAR_BUFFER_LEGTH - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);
//  uint16_t i;
//  //
//  if(Uart6CicularBufferIndex < DmaBufferHwIndex)
//  {
//    for(i=0; i< DmaBufferHwIndex-Uart6CicularBufferIndex; i++)
//      uartDllRxFsm(UARTDLL_UART_6_RS485, UARTDLL_FSM_STIMULI_INCOMMING_CHAR,Uart6RxDmaCicularBuffer[Uart6CicularBufferIndex+i] );
//  }
//
//  if(Uart6CicularBufferIndex > DmaBufferHwIndex)
//  {
//    for(i=0; i< UART_RX_DMA_CIRCULAR_BUFFER_LEGTH-Uart6CicularBufferIndex; i++)
//      uartDllRxFsm(UARTDLL_UART_6_RS485, UARTDLL_FSM_STIMULI_INCOMMING_CHAR,Uart6RxDmaCicularBuffer[Uart6CicularBufferIndex+i]);
//    for(i=0; i< DmaBufferHwIndex; i++)
//      uartDllRxFsm(UARTDLL_UART_6_RS485, UARTDLL_FSM_STIMULI_INCOMMING_CHAR,Uart6RxDmaCicularBuffer[i]);
//  }
//  Uart6CicularBufferIndex = DmaBufferHwIndex;
}




/******************************************************************************
*** @brief  void hwdriversTimer3Init()
*   @param  
*   @retval 
****************************************************************/
void hwdriversTimer3Init()
{

	__HAL_TIM_ENABLE(&htim3);


	#if 0
	// Frequency Monotoring timer
  // The ETR input configuration 
  TIM_ETRClockMode2Config(TIM3, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
  
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructureTim3);
  TIM_TimeBaseStructureTim3.TIM_Period = 0; 
  TIM_TimeBaseStructureTim3.TIM_Prescaler = 0;
  TIM_TimeBaseStructureTim3.TIM_ClockDivision = 0;
  TIM_TimeBaseStructureTim3.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructureTim3);
  
  //TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
 
  //TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
   
 TIM3->ARR = (uint32_t)0xffff;
  // TIM7 enable counter
  TIM3->CR1 |= TIM_CR1_CEN;
  
  __HAL_TIM_ENABLE(&htim3);
#endif
}



/******************************************************************************
*** @brief  void hwdriversTimer7Init()
*   @param  
*   @retval 
******************************************************************************/
void hwdriversTimer7Init()
{
  
	// TODO: What is it : Frequency measurement time slicer

	__HAL_TIM_ENABLE(&htim7);
	__HAL_TIM_ENABLE_IT(&htim7, TIM_IT_UPDATE);


#if 0
  FreqMonitoringFrequency=0;
  FreqMonitoringCyclesLast=0;
  FilteredFrequecy = 0;
  LastFrequency = 0;
  FrequencyMonitoringFilter=0;

  
  TIM_InternalClockConfig(TIM7);
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructureTim7);
  TIM_TimeBaseStructureTim7.TIM_Period = 0; 
  TIM_TimeBaseStructureTim7.TIM_Prescaler = 0;
  TIM_TimeBaseStructureTim7.TIM_ClockDivision = 0;
  TIM_TimeBaseStructureTim7.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructureTim7);
  
  TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
 
  TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
  
  // Interrupt every 1mSec, so we devide the 27120000 by 1000
  TIM7->ARR = (uint32_t) (HSE_VALUE/1000)-1;
  // TIM7 enable counter
  TIM7->CR1 |= TIM_CR1_CEN;
  
  __HAL_TIM_ENABLE(&htim7);

#endif

}

/******************************************************************************
 * @brief  void hwdriversFrequencyMonitoringPeriodIsr(TIM_HandleTypeDef *htim)
 * @param  
 * @retval 
 ******************************************************************************/
void hwdriversFrequencyMonitoringPeriodIsr(TIM_HandleTypeDef *htim)
{
  uint16_t TempCnt;

	// Get a sample of the Hw counter
	TempCnt = TIM3->CNT;
	if(TempCnt == FreqMonitoringCyclesLast)
	  FreqMonitoringFrequency = 0;
	else
	{
	if(TempCnt > FreqMonitoringCyclesLast)
	  FreqMonitoringFrequency = TempCnt - FreqMonitoringCyclesLast;
	else
	  FreqMonitoringFrequency = (0xffff - FreqMonitoringCyclesLast) + TempCnt;

	  FreqMonitoringCyclesLast = TempCnt;
	}

	// Measured frequency filter mechanism
	if(FreqMonitoringFrequency == LastFrequency)
	{
	  if(FrequencyMonitoringFilter < HWDRIVERS_FREQUENCY_MONITORING_FILTER)
		FrequencyMonitoringFilter++;
	  else
	  {
		LastFrequency = FreqMonitoringFrequency;
		FilteredFrequecy = FreqMonitoringFrequency;
		// Update debug 2 field
		pccpmmAppLayerStruct.Board1SystemRegisters.Debug1L = (uint8_t)FilteredFrequecy;
		pccpmmAppLayerStruct.Board1SystemRegisters.Debug1M = (uint8_t)(FilteredFrequecy>>8);
	  }
	}
	else
	{
	  FrequencyMonitoringFilter = 0;
	  LastFrequency = FreqMonitoringFrequency;
	}

	hwdriversCycleCounter++;
}


uint32_t hwdriversCycleCounterGet()
{
  return(hwdriversCycleCounter);
}


uint8_t flashBuffer2Required;
#ifdef USE_PVD
void hwdriversPVDConfig(void)
{
	// TODO: Check if we need to enable it not only during power up reset

	PWR_PVDTypeDef PvdStruc;

	PvdStruc.PVDLevel 	= PWR_PVDLEVEL_3;
	PvdStruc.Mode		= PWR_PVD_MODE_IT_RISING;

	HAL_PWR_ConfigPVD(&PvdStruc);
	HAL_PWR_EnablePVD();


	#if 0
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;

  /* Enable PWR clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  
  /* Enable the PVD Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = PVD_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = INTERRUPT_PRIORITY_VALID_FROMISR_API;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
      
  /* Configure EXTI Line16(PVD Output) to generate an interrupt on rising and
     falling edges */
  EXTI_ClearITPendingBit(EXTI_Line16);
  EXTI_InitStructure.EXTI_Line = EXTI_Line16;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Configure the PVD Level to 7 (PVD detection level set to 2.9V, refer to the
     electrical characteristics of you device datasheet for more details) */
  PWR_PVDLevelConfig(PWR_PVDLevel_7); // as it turned out - only level 5 is working on falling (why ?)

  /* Enable the PVD Output */
  PWR_PVDCmd(ENABLE);
  
  //hwdriversGpioBitWrite(HWDRIVERS_PB15_USB_TEST, 1);
#endif
}

/******************************************************************************
*** @brief  void HAL_PWR_PVDCallback(void)
*   @param  
*   @retval 
******************************************************************************/
void HAL_PWR_PVDCallback(void)
{  
//	hwdriversGpioBitWrite(HWDRIVERS_PB14_USB_TEST,1);
	if(__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO) != RESET)
  {
    // Save pointers to the SPI - Flash 
//	hwdriversGpioBitWrite(HWDRIVERS_PB14_USB_TEST,0);
//	hwdriversGpioBitWrite(HWDRIVERS_PB14_USB_TEST,1);

    //    if (flashBuffer2Required)
//    {
//    RCC_AHB1PeriphClockCmd(ADCx_CHANNEL_GPIO_CLK, DISABLE);  
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, DISABLE);
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, DISABLE);
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, DISABLE);    
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, DISABLE);
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, DISABLE);
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, DISABLE);
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, DISABLE);
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, DISABLE);
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, DISABLE);
//    
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, DISABLE);  
//    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SYSCFG, DISABLE); 
//    
//    //sfFlushBuffer2();
//    }
    /* Clear the Key Button EXTI line pending bit */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_PVDO);

    // Brick the device until brown-out so no memory validations will be made
    vTaskSuspendAll();
  }
}
#endif


void hwdriversVinInterruptConfig(void)
{


	// TODO: Implement VIN EXTI interrupt

#if 0

NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;



  /* Enable the EXTI0_IRQn Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = INTERRUPT_PRIORITY_VALID_FROMISR_API;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
#endif      

}







/******************************************************************************
*** @brief  void  hwdriversStartCycleCounter()
*   @param  
*   @retval 
******************************************************************************/
void  hwdriversStartCycleCounter()
{
  // enable the use DWT
  // DWT = data watchdog and trace
  
  *DEMCR |= 0x01000000;
  
  // Reset cycle counter
  *DWT_CYCCNT = 0; 

  // enable cycle counter
  *DWT_CONTROL |= 1 ; 
}

/******************************************************************************
*** @brief  uint32_t  hwdriversCyclesCounterGet()
*   @param  
*   @retval 
******************************************************************************/
uint32_t  hwdriversCyclesCounterGet()
{
  uint32_t CyclesCounter;
  
  CyclesCounter = *DWT_CYCCNT;
  
  *DWT_CYCCNT = 0; 

  return (CyclesCounter); 
}



/******************************************************************************
*** @brief  long hwdriversStopCycleCounter()
*   @param  
*   @retval 
******************************************************************************/
long hwdriversStopCycleCounter()
{
  // disable cycle counter
  *DWT_CONTROL &= ~1 ; 
  return *DWT_CYCCNT;
}


/**
 * @brief  Splits a float into two integer values.
 * @param  in the float value as input
 * @param  out_int the pointer to the integer part as output
 * @param  out_dec the pointer to the decimal part as output
 * @param  dec_prec the decimal precision to be used
 * @retval None
 */
void floatToInt(float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec)
{
  *out_int = (int32_t)in;
  if(in >= 0.0f)
  {
    in = in - (float)(*out_int);
  }
  else
  {
    in = (float)(*out_int) - in;
  } 
  *out_dec = (int32_t)trunc(in * pow(10, dec_prec));
}


/**
 * @brief Creates a delay using predefined variables (which are set according to the system clock
 * @note  See UpdateWaitTimingVariables function
 * @param ps: the delay time
 * @retval None
 */
#pragma optimize=none
void waitHere(uint32_t delay)
{
  while ((uint32_t)delay)
    (uint32_t)delay--;
}


/******************************************************************************
*** @brief  ReturnCode_T hwdriversPowerMeasurmentsGet( hwdriverPowerMeasurments_t* ReturnedPowerMeasurmentsPtr)
*   @param  
*   @retval 
******************************************************************************/
ReturnCode_T hwdriversPowerMeasurmentsGet( hwdriverPowerMeasurments_t* ReturnedPowerMeasurmentsPtr)
{
  hwdriverPowerMeasurments_t myMeasurments;
  
  myMeasurments.BattVoltage = chargerBattVoltageGet();
  
  myMeasurments.DcInputVoltage = chargerUSBVoltageGet();
  
  myMeasurments.ChargingStatus          = TRUE;
  myMeasurments.DcPlugConnectionState   = TRUE;  
  myMeasurments.DcPlugVoltageExist      = TRUE;
  myMeasurments.DcVbatCurrent = 0; // Place holder, unused
  // Update the calling function pointed structure
  *ReturnedPowerMeasurmentsPtr = myMeasurments;
  
  // If successful TODO: 
  return(RETURNCODE_OK);
}








#if defined(USE_NUCLEO_BOARD) && defined(USE_PVD)
/******************************************************************************
*** @brief  void PVD_IRQHandlerTest(void)
*   @param  
*   @retval 
******************************************************************************/
void PVD_IRQHandlerTest(void)
{
  
  RCC_AHB1PeriphClockCmd(ADCx_CHANNEL_GPIO_CLK, DISABLE);  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, DISABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, DISABLE);    
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, DISABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, DISABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, DISABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, DISABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, DISABLE);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, DISABLE);  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, DISABLE);  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, DISABLE); 

  gdbg("P1234567890");
}
#endif


/******************************************************************************
*** @brief  uint32_t hwdriversLinuxTimeStampGet()
*   @param  
*   @retval 
******************************************************************************/
uint32_t hwdriversLinuxTimeStampGet()
{
  return 11; // tbd
}


/******************************************************************************
*** @brief  uint8_t  hwdrivers10mSecGet()
*   @param  
*   @retval 
******************************************************************************/
uint8_t  hwdrivers10mSecGet()
{
  return 12; // tbd
}

/******************************************************************************
* @brief  hardwaredriversPhaseStatus_T hwdriversPhaseSenseGet()
* @param  
* @retval 
******************************************************************************/
hardwaredriversPhaseStatus_T hwdriversPhaseSenseGet(hardwaredriversPhaseStatus_T previousPhase)
{
  uint8_t Temp = 0;
  
  hardwaredriversPhaseStatus_T MyReturn;

  
  if( HAL_GPIO_ReadPin(PHASE_SENSE_ZERO_GPIO_Port, PHASE_SENSE_ZERO_Pin))
    Temp |= 1;
  else
    Temp &= ~1;
  
  if(HAL_GPIO_ReadPin(PHASE_SENSE_UNDER_GPIO_Port, PHASE_SENSE_UNDER_Pin))
    Temp |= 2;
  else
    Temp &= ~2;
  
  if( HAL_GPIO_ReadPin(PHASE_SENSE_OVER_GPIO_Port, PHASE_SENSE_OVER_Pin))
    Temp |= 4;
  else
    Temp &= ~4;
  
  pccpmmAppLayerStruct.Board1SystemRegisters.Debug2L = Temp;
  
  switch(Temp)
    {
    case 0:
       MyReturn = HWDRIVERS_PHASE_OVER;
       break;
    case 4:
      MyReturn = HWDRIVERS_PHASE_ZERO;
      break;
    case 6:
    case 7:
      MyReturn = HWDRIVERS_PHASE_UNDER;
      break;
    case 3:
      if(previousPhase == HWDRIVERS_PHASE_UNDER)
        MyReturn = HWDRIVERS_PHASE_UNDER;
      else
        MyReturn = HWDRIVERS_PHASE_ZERO;      
      break;
    case 2:
      if(previousPhase == HWDRIVERS_PHASE_OVER)
        MyReturn = HWDRIVERS_PHASE_OVER;
      else
        MyReturn = HWDRIVERS_PHASE_ZERO;
      break;
    default:
      MyReturn = HWDRIVERS_PHASE_ZERO;
      break;
    }
  
  
#if 0
 if(Temp == 3 )
    MyReturn = HWDRIVERS_PHASE_ZERO;
  else
    if(Temp == 7)
      MyReturn = HWDRIVERS_PHASE_OVER;
  else
          MyReturn = HWDRIVERS_PHASE_UNDER;
#endif
       

  
  return MyReturn;
}



/******************************************************************************
 * @brief  uint8_t hwdriversPhaseRawDataGet()
 * @param  
 * @retval 
 ******************************************************************************/
uint8_t hwdriversPhaseRawDataGet()
{
 return  ( HAL_GPIO_ReadPin(PHASE_SENSE_OVER_GPIO_Port, PHASE_SENSE_OVER_Pin)*4 +  HAL_GPIO_ReadPin(PHASE_SENSE_UNDER_GPIO_Port, PHASE_SENSE_UNDER_Pin)*2 +  HAL_GPIO_ReadPin(PHASE_SENSE_ZERO_GPIO_Port, PHASE_SENSE_ZERO_Pin)) ;
 }


/******************************************************************************
 * @brief  ReturnCode_T hwdriversRelaysValueSet( uint16_t NewRelaysValue)
 * @param  
 * @retval 
 ******************************************************************************/
ReturnCode_T hwdriversRelaysStateInit()
{
  
  return(RETURNCODE_OK);
}



/******************************************************************************
 * @brief  ReturnCode_T hwdriversRelaysActivate(uint16_t NewRelaysValue)
 * @param  
 * @retval 
 ******************************************************************************/
ReturnCode_T hwdriversRelaysChangeActivate(uint16_t NewRelaysValue)
{
  // Update the Relays output bits
  RelaysOutputHandle(NewRelaysValue);
  // Update the GUI data structure
  pccpmmAppLayerStruct.BeltTransmitterRegisters.RelayStateMsb = (uint8_t)(NewRelaysValue>>8);
  pccpmmAppLayerStruct.BeltTransmitterRegisters.RelayStateLsb = (uint8_t)(NewRelaysValue);

  // Activate the Relays Latch Enable
//  HAL_GPIO_WritePin(RELAY_CONTROL_EN_GPIO_Port, RELAY_CONTROL_EN_Pin, GPIO_PIN_SET);
  //GPIO_WriteBit(GPIOE, GPIO_PIN_7, Bit_SET);
  return RETURNCODE_OK;  
}


/******************************************************************************
 * @brief  ReturnCode_T hwdriversRelaysDeActivate()
 * @param  
 * @retval 
 ******************************************************************************/
ReturnCode_T hwdriversRelaysChangeIdle()
{
  // Deactivate the Relays Latch Enable
//	HAL_GPIO_WritePin(RELAY_CONTROL_EN_GPIO_Port, RELAY_CONTROL_EN_Pin, GPIO_PIN_RESET);
	//GPIO_WriteBit(GPIOE, GPIO_PIN_7, Bit_RESET);
  // Move all the relays states to inactive
  //RelaysOutputHandle(0);
  return RETURNCODE_OK;  
}


/******************************************************************************
 * @brief  void RelaysOutputHandle(uint16_t NewRelaysValue)
 * @param  
 * @retval 
 ******************************************************************************/
void RelaysOutputHandle(uint16_t NewRelaysValue)
{
  // The 9 relays are connected as follows: RELAYS[8:0] =  {GPIOA[10], GPIOE[15:8]} while the Relays latch enable bit is located at GPIOE[7]
  // Read GPIOE output bits state, The relevant bits for the Relays are bits 8 to 15 corresponding to Relay-0 to relay-7
  uint16_t MyTemp = GPIOE->ODR;
  // MAsk the relevant bits
  MyTemp &= ~0xff00;
  // place the new relays value bits into the upper byte of GPIOE, we ignore NewRelaysValue bit 8 in this action, will be handled later
  MyTemp |= (NewRelaysValue<<8);
  // Write to the GPIO
  GPIOE->ODR = MyTemp;
  // Handle NewRelaysValue bit 8 separately as it drives GPIOA[10]
  //HAL_GPIO_WritePin(LATCHED_DATA_8_GPIO_Port, LATCHED_DATA_8_Pin, (NewRelaysValue & 0x0100)?GPIO_PIN_SET:GPIO_PIN_RESET);
}



/******************************************************************************
* @brief  ReturnCode_T hwdriversPwmSet(uint16_t OutputState, uint16_t TimerPeriod, uint16_t TimerCapture)
* @param  
* @retval 
******************************************************************************/
ReturnCode_T hwdriversPwmSet(uint16_t OutputState, uint16_t TimerPeriod, uint16_t TimerCapture)
{
#if 0
	TIM_OnePulse_InitTypeDef sConfig;



  // Timer4 enable counter
  __HAL_TIM_MOE_DISABLE(&htim1);
  __HAL_TIM_DISABLE(&htim4);

	TIM_OC_InitTypeDef MyOcInit;

//	   TIM_CCxChannelCmd(TIM4, TIM_CHANNEL_2, TIM_CCx_DISABLE);


	  htim4.Instance = TIM4;
	  htim4.Init.Prescaler = 3;
	  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim4.Init.Period = TimerPeriod-1;
	  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	  HAL_TIM_Base_Init(&htim4);

	   MyOcInit.OCFastMode = TIM_OCFAST_DISABLE;
	   MyOcInit.OCMode = TIM_OCMODE_PWM1;
	   MyOcInit.OCIdleState = TIM_OCIDLESTATE_RESET;
	   MyOcInit.Pulse = TimerPeriod - TimerCapture;
	   MyOcInit.OCPolarity = TIM_OCPOLARITY_LOW;
	   HAL_TIM_PWM_ConfigChannel(&htim4, &MyOcInit, TIM_CHANNEL_2);

//	   TIM_CCxChannelCmd(TIM4, TIM_CHANNEL_2, OutputState);

	   if(OutputState)
	   {
		   HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
		   __HAL_TIM_MOE_ENABLE(&htim1);
	   }


	   // TODO: Implement PWM timer
#endif
#if 0

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure4;
	TIM_OCInitTypeDef     sConfigOC4;


	 // Timer4 enable counter
	  __HAL_TIM_DISABLE(&htim1);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure4);
	TIM_TimeBaseStructure4.TIM_Period = TimerPeriod-1;
	TIM_TimeBaseStructure4.TIM_Prescaler = 1;
	TIM_TimeBaseStructure4.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure4.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_Base_SetConfig(&htim4, &ITIM_TimeBaseStructure4);

	// Timer4 Output Compare Toggle Mode configuration: Channel2
	sConfigOC4.TIM_OCMode = TIM_OCMode_PWM1;
	sConfigOC4.TIM_OutputState = OutputState;
	sConfigOC4.TIM_Pulse = TimerPeriod - TimerCapture;
	sConfigOC4.TIM_OCPolarity = TIM_OCPolarity_Low;
	HAL_TIM_PWM_ConfigChannel(&htim4, &sConfig, TIM_CHANNEL_2);

	__HAL_TIM_ENABLE(&htim1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);


#if 0
	TIM_OCInitTypeDef     sConfigOC4;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure4;

  
  // Timer4 enable counter
  __HAL_TIM_DISABLE(&htim1);
  __HAL_TIM_DISABLE(&htim4);

  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure4);
  TIM_TimeBaseStructure4.TIM_Period = TimerPeriod-1;
  TIM_TimeBaseStructure4.TIM_Prescaler = 1;
  TIM_TimeBaseStructure4.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure4.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure4);

  // Timer4 Output Compare Toggle Mode configuration: Channel2
  sConfigOC4.TIM_OCMode = TIM_OCMode_PWM1;
  sConfigOC4.TIM_OutputState = OutputState;
  sConfigOC4.TIM_Pulse = TimerPeriod - TimerCapture;
  sConfigOC4.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OC2Init(TIM4, &sConfigOC4);
  
  // Timer4 enable counter
  __HAL_TIM_ENABLE(&htim4);
  __HAL_TIM_ENABLE(&htim1);


#endif

#endif
  return RETURNCODE_OK;  

}


/******************************************************************************
* @brief  
* @param  
* @retval 
******************************************************************************/
ReturnCode_T hwdriversPwmControl(typesControl_T Control)
{
 
  
  if(Control == TYPES_ENABLE)
  {
	  __HAL_TIM_ENABLE(&htim1);
	  TIM_CCxChannelCmd((&htim1)->Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
	  // __HAL_TIM_ENABLE(&htim4);
	  // TIM_CCxChannelCmd((&htim4)->Instance, TIM_CHANNEL_2, TIM_CCx_ENABLE);
  }
  else
  {
	  // Channels must be disabled before timer can be disabled
	  TIM_CCxChannelCmd((&htim1)->Instance, TIM_CHANNEL_1, TIM_CCx_DISABLE);
	  __HAL_TIM_DISABLE(&htim1);
	  // TIM_CCxChannelCmd((&htim4)->Instance, TIM_CHANNEL_2, TIM_CCx_DISABLE);
	  // __HAL_TIM_DISABLE(&htim4);
  }

  return(RETURNCODE_OK);
}






/******************************************************************************
* @brief  ReturnCode_T hwdriversNtcTemperatureGet(int16_t *ReturnedNtcTemperaturePtr)
* @param  
* @retval 
******************************************************************************/
ReturnCode_T hwdriversNtcTemperatureGet(int16_t *ReturnedNtcTemperaturePtr)
{
  ReturnCode_T MyReturnCode;
  float FloatAdcVoltage;
  float FloatNTCCalc;
  float resultTemperature;
  
  // Calculate voltage in mV
  FloatAdcVoltage = (HEDRIVERS_VDD * ADCConvertedValue[HWDRIVERS_TX_NTC])/4096;
  //Calculate NTC resistor by given formula: Vmeasure*10K/(Vcc-Vmeasure)
  FloatNTCCalc = (FloatAdcVoltage * 10000.0)/(HEDRIVERS_VDD - FloatAdcVoltage);
  //Calculate temperature in Kelvin and then change it to celsius
  resultTemperature = (1/(((1.0/3428.0) * log(FloatNTCCalc/10000.0)) +(1.0/298.15))) - 273.15;
  *ReturnedNtcTemperaturePtr = (int16_t) (resultTemperature * 100);
  
  MyReturnCode = RETURNCODE_OK;

  return(MyReturnCode);
}
/******************************************************************************
* @brief Set the pointer's value to the output current ond voltage the VDD_PA converter * 100
* @param 
* @retval
******************************************************************************/
void hwdriversTxPowerGet(int16_t *ReturnedCurrentPtr, int16_t *ReturnedVoltagePtr)
{
  float adcCurrent;
  float adcVoltage;
  float resultCurrent;
  float resultVoltage;
  
  // Calculate voltage in mV
  adcCurrent = (HEDRIVERS_VDD * ADCConvertedValue[HWDRIVERS_TX_ISMON_PA])/4095;
  adcVoltage = (HEDRIVERS_VDD * ADCConvertedValue[HWDRIVERS_TX_VDD_PA])/4095;
  // Calculate output current: ((Vmeasure - 0.25V) / 10) / Rsense
  resultCurrent = ((adcCurrent - HWDRIVERS_ISMON_OFFSET) / 10.0) / HWDRIVERS_ISMON_RESISTOR;
  resultVoltage = (adcVoltage / (float)HWDRIVERS_VDD_PA_RATIO);
  *ReturnedCurrentPtr = (int16_t) (resultCurrent * 100);
  *ReturnedVoltagePtr = (int16_t) (resultVoltage * 100);
}
/******************************************************************************
* @brief  ReturnCode_T hwdriversFrequencyMonitoringGet(uint16_t *ReturnedRequencyPtr)
* @param  
* @retval 
******************************************************************************/
ReturnCode_T hwdriversFrequencyMonitoringGet(uint16_t *ReturnedRequencyPtr)
{
  // Return the last measured frequency
  *ReturnedRequencyPtr = FilteredFrequecy;
  
  return(RETURNCODE_OK);
}

/******************************************************************************
* @brief  ReturnCode_T hwdriversTxMonitorGet(uint16_t *ReturnedTxMonitorLevelPtr)
* @param  
* @retval 
******************************************************************************/
ReturnCode_T hwdriversTxMonitorGet(uint16_t *ReturnedTxMonitorLevelPtr)
{
  // Return the last measured Tx monitor value
  *ReturnedTxMonitorLevelPtr = ADCConvertedValue[HWDRIVERS_TX_MONITOR];

  return(RETURNCODE_OK);
}


/******************************************************************************
* @brief  ReturnCode_T hwdriversGpioSet(HwdriversGpiosT RegID, uint8_t BitState)
* @param  
* @retval 
******************************************************************************/
ReturnCode_T hwdriversGpioBitWrite(HwdriversGpiosT RegID, uint8_t BitState)
{
#if 1
	struct
  {
    GPIO_TypeDef * GpioPtr;
    uint16_t		 PortPin;
  } TempGpio = {GPIOA, GPIO_PIN_0};

  switch(RegID)
  {	
  case HWDRIVERS_PUSH_BUTTON_IN	  :
    TempGpio.GpioPtr = GPIOD;
    TempGpio.PortPin = GPIO_PIN_14;
    break;                                          
  case HWDRIVERS_SENSORS_ENABLE	     : 
    TempGpio.GpioPtr = GPIOA;
    TempGpio.PortPin = GPIO_PIN_9;
    break;                                          
  case HWDRIVERS_BLE_POWER_ENABLE      : 
    TempGpio.GpioPtr = GPIOA;
    TempGpio.PortPin = GPIO_PIN_11;
    break;                                          
  case HWDRIVERS_MAIN_RED_LED          :
    TempGpio.GpioPtr = GPIOA;
    TempGpio.PortPin = GPIO_PIN_2;
    break;                                          
  case HWDRIVERS_MAIN_GREEN_LED          :
    TempGpio.GpioPtr = GPIOA;
    TempGpio.PortPin = GPIO_PIN_3;
    break;          
  case HWDRIVERS_MAIN_BLUE_LED          :
    TempGpio.GpioPtr = GPIOE;
    TempGpio.PortPin = GPIO_PIN_6;
    break;
  case HWDRIVERS_USB_RED_LED          :
    TempGpio.GpioPtr = GPIOC;
    TempGpio.PortPin = GPIO_PIN_12;
    break;
  case HWDRIVERS_USB_GREEN_LED          :
    TempGpio.GpioPtr = GPIOC;
    TempGpio.PortPin = GPIO_PIN_11;
    break;
  case HWDRIVERS_USB_BLUE_LED          :
    TempGpio.GpioPtr = GPIOC;
    TempGpio.PortPin = GPIO_PIN_10;
    break;
  case HWDRIVERS_VIBRATOR		         :  
    TempGpio.GpioPtr = GPIOB;
    TempGpio.PortPin = GPIO_PIN_0;
    break;                                          
  case HWDRIVERS_MAIN_OSC_ENABLE	     :
    TempGpio.GpioPtr = GPIOC;
    TempGpio.PortPin = GPIO_PIN_1;
    break;                                          
  case HWDRIVERS_TX_SUP_ENABLE		 : 
    TempGpio.GpioPtr = GPIOC;
    TempGpio.PortPin = GPIO_PIN_3;
    break;
  case HWDRIVERS_UI_ENABLE	     :
    TempGpio.GpioPtr = GPIOC;
    TempGpio.PortPin = GPIO_PIN_4;
    break;                                          
  case HWDRIVERS_ACCL_GYRO_INT		 : 
    TempGpio.GpioPtr = GPIOD;;
    TempGpio.PortPin = GPIO_PIN_0;
    break;                                          
  case HWDRIVERS_CHRG_CRNT_CNTL	     : 
    TempGpio.GpioPtr = GPIOD;
    TempGpio.PortPin = GPIO_PIN_4;
    break;                                          
  case HWDRIVERS_BLE_RESET		     :  
    TempGpio.GpioPtr = GPIOD;
    TempGpio.PortPin = GPIO_PIN_7;
    break;                                          
  case HWDRIVERS_BLE_BOOT		     : 
    TempGpio.GpioPtr = GPIOD;
    TempGpio.PortPin = GPIO_PIN_8;
    break;                                          
  case HWDRIVERS_PHASE_SENSE_OVER	     :  
    TempGpio.GpioPtr = GPIOD;
    TempGpio.PortPin = GPIO_PIN_9;
    break;                                          
  case HWDRIVERS_PHASE_SENSE_UNDER	 : 
    TempGpio.GpioPtr = GPIOD;
    TempGpio.PortPin = GPIO_PIN_10;
    break;                                          
  case HWDRIVERS_PHASE_SENSE_ZERO	     : 
    TempGpio.GpioPtr = GPIOD;
    TempGpio.PortPin = GPIO_PIN_11;
    break;                                          
  case HWDRIVERS_BUZZER		         : 
    TempGpio.GpioPtr = GPIOD;
    TempGpio.PortPin = GPIO_PIN_12;
    break;                                          
  case HWDRIVERS_CHARGER_DISABLE	     :
    TempGpio.GpioPtr = GPIOD;
    TempGpio.PortPin = GPIO_PIN_15;
    break;
  case HWDRIVERS_RS485_ENB		     :
    TempGpio.GpioPtr = GPIOD;
    TempGpio.PortPin = GPIO_PIN_0;
    break;
  case HWDRIVERS_CHRG_FALT_INDICATION	 :
    TempGpio.GpioPtr = GPIOE;
    TempGpio.PortPin = GPIO_PIN_3;
    break;
  case HWDRIVERS_CHRG_INDICATION	     :
    TempGpio.GpioPtr = GPIOE;
    TempGpio.PortPin = GPIO_PIN_4;
    break;
  case HWDRIVERS_RELAYS_LATCH		     :
    TempGpio.GpioPtr = GPIOE;
    TempGpio.PortPin = GPIO_PIN_7;
    break;
  case HWDRIVERS_LATCH_DATA_0		     :
    TempGpio.GpioPtr = GPIOE;
    TempGpio.PortPin = GPIO_PIN_8;
    break;
  case HWDRIVERS_LATCH_DATA_1		     :
    TempGpio.GpioPtr = GPIOE;
    TempGpio.PortPin = GPIO_PIN_9;
    break;
  case HWDRIVERS_LATCH_DATA_2		     :
    TempGpio.GpioPtr = GPIOE;
    TempGpio.PortPin = GPIO_PIN_10;
    break;
  case HWDRIVERS_LATCH_DATA_3		     :
    TempGpio.GpioPtr = GPIOE;
    TempGpio.PortPin = GPIO_PIN_11;
    break;
  case HWDRIVERS_LATCH_DATA_4		     :
    TempGpio.GpioPtr = GPIOE;
    TempGpio.PortPin = GPIO_PIN_12;
    break;
  case HWDRIVERS_LATCH_DATA_5		     :
    TempGpio.GpioPtr = GPIOE;
    TempGpio.PortPin = GPIO_PIN_13;
    break;
  case HWDRIVERS_LATCH_DATA_6		     :
    TempGpio.GpioPtr = GPIOE;
    TempGpio.PortPin = GPIO_PIN_14;
    break;
  case HWDRIVERS_LATCH_DATA_7		     :
    TempGpio.GpioPtr = GPIOE;
    TempGpio.PortPin = GPIO_PIN_15;
    break;
  case HWDRIVERS_LATCH_DATA_8		     :
    TempGpio.GpioPtr = GPIOA;
    TempGpio.PortPin = GPIO_PIN_10;
    break;
    
  case HWDRIVERS_PC5_TEST:
    TempGpio.GpioPtr = GPIOC;
    TempGpio.PortPin = GPIO_PIN_5;
    break;
    
  case HWDRIVERS_PC10_TEST:
    TempGpio.GpioPtr = GPIOC;
    TempGpio.PortPin = GPIO_PIN_10;
  break;
      
  case HWDRIVERS_PB14_USB_TEST:
    TempGpio.GpioPtr = GPIOB;
    TempGpio.PortPin = GPIO_PIN_14;
  break;
      
  case HWDRIVERS_PB15_USB_TEST:
    TempGpio.GpioPtr = GPIOB;
    TempGpio.PortPin = GPIO_PIN_15;
  break;
  
  case HWDRIVERS_PB8_SCL:
    TempGpio.GpioPtr = GPIOB;
    TempGpio.PortPin = GPIO_PIN_8;
    break;
    
  case HWDRIVERS_PB9_SDA:
    TempGpio.GpioPtr = GPIOB;
    TempGpio.PortPin = GPIO_PIN_9;
    break; 
    
  case HWDRIVERS_PC8_SFLASH_RESET_N:
    TempGpio.GpioPtr = GPIOC;
    TempGpio.PortPin = GPIO_PIN_8;
    break; 
    
  case HWDRIVERS_PE6_MAX_POWER_EN:
	   TempGpio.GpioPtr = GPIOD;
	   TempGpio.PortPin = GPIO_PIN_1;
	  break;

  case HWDRIVERS_PD13_EXT_TX_OUT_MODULATOR:
	   TempGpio.GpioPtr = GPIOD;
	   TempGpio.PortPin = GPIO_PIN_13;
	  break;

  case HWDRIVERS_PC5_MAIN_BUCK_EN:
	   TempGpio.GpioPtr = GPIOC;
	   TempGpio.PortPin = GPIO_PIN_5;
	   break;



  }
  
  if(BitState)
	  HAL_GPIO_WritePin (TempGpio.GpioPtr, TempGpio.PortPin, GPIO_PIN_SET);
//    GPIO_BSRR_SET_PIN(TempGpio.GpioPtr, TempGpio.PortPin);
  else
	  HAL_GPIO_WritePin (TempGpio.GpioPtr, TempGpio.PortPin, GPIO_PIN_RESET);

#endif

  return(RETURNCODE_OK);
}




/******************************************************************************
* @brief  ReturnCode_T hwdriversGpioSet(HwdriversGpiosT RegID, uint8_t BitState)
* @param
* @retval
******************************************************************************/
uint8_t hwdriversGpioBitRead(HwdriversGpiosT RegID)
{

	uint8_t ReturnedValue;

	struct
  {
    GPIO_TypeDef * GpioPtr;
    uint16_t	 PortPin;
  } TempGpio = {GPIOA, GPIO_PIN_0};

  switch(RegID)
  {
  case HWDRIVERS_PUSH_BUTTON_IN	  :
    TempGpio.GpioPtr = GPIOD;
    TempGpio.PortPin = (GPIO_PIN_14);
    break;
  case HWDRIVERS_SENSORS_ENABLE	     :
    TempGpio.GpioPtr = GPIOA;
    TempGpio.PortPin = (GPIO_PIN_9);
    break;
  case HWDRIVERS_BLE_POWER_ENABLE      :
    TempGpio.GpioPtr = GPIOA;
    TempGpio.PortPin = (GPIO_PIN_11);
    break;
  case HWDRIVERS_MAIN_RED_LED          :
    TempGpio.GpioPtr = GPIOA;
    TempGpio.PortPin = GPIO_PIN_2;
    break;
  case HWDRIVERS_MAIN_GREEN_LED          :
    TempGpio.GpioPtr = GPIOA;
    TempGpio.PortPin = GPIO_PIN_3;
    break;
  case HWDRIVERS_MAIN_BLUE_LED          :
    TempGpio.GpioPtr = GPIOE;
    TempGpio.PortPin = GPIO_PIN_6;
    break;
  case HWDRIVERS_USB_RED_LED          :
    TempGpio.GpioPtr = GPIOC;
    TempGpio.PortPin = GPIO_PIN_12;
    break;
  case HWDRIVERS_USB_GREEN_LED          :
    TempGpio.GpioPtr = GPIOC;
    TempGpio.PortPin = GPIO_PIN_11;
    break;
  case HWDRIVERS_USB_BLUE_LED          :
    TempGpio.GpioPtr = GPIOC;
    TempGpio.PortPin = GPIO_PIN_10;
    break;
  case HWDRIVERS_VIBRATOR		         :
    TempGpio.GpioPtr = GPIOB;
    TempGpio.PortPin = (GPIO_PIN_0);
    break;
  case HWDRIVERS_MAIN_OSC_ENABLE	     :
    TempGpio.GpioPtr = GPIOC;
    TempGpio.PortPin = (GPIO_PIN_1);
    break;
  case HWDRIVERS_TX_SUP_ENABLE		 :
    TempGpio.GpioPtr = GPIOC;
    TempGpio.PortPin = (GPIO_PIN_3);
    break;
  case HWDRIVERS_UI_ENABLE	     :
    TempGpio.GpioPtr = GPIOC;
    TempGpio.PortPin = (GPIO_PIN_4);
    break;
  case HWDRIVERS_ACCL_GYRO_INT		 :
    TempGpio.GpioPtr = GPIOD;;
    TempGpio.PortPin = (GPIO_PIN_0);
    break;
  case HWDRIVERS_CHRG_CRNT_CNTL	     :
    TempGpio.GpioPtr = GPIOD;
    TempGpio.PortPin = (GPIO_PIN_4);
    break;
  case HWDRIVERS_BLE_RESET		     :
    TempGpio.GpioPtr = GPIOD;
    TempGpio.PortPin = (GPIO_PIN_7);
    break;
  case HWDRIVERS_BLE_BOOT		     :
    TempGpio.GpioPtr = GPIOD;
    TempGpio.PortPin = (GPIO_PIN_8);
    break;
  case HWDRIVERS_PHASE_SENSE_OVER	     :
    TempGpio.GpioPtr = GPIOD;
    TempGpio.PortPin = (GPIO_PIN_9);
    break;
  case HWDRIVERS_PHASE_SENSE_UNDER	 :
    TempGpio.GpioPtr = GPIOD;
    TempGpio.PortPin = (GPIO_PIN_10);
    break;
  case HWDRIVERS_PHASE_SENSE_ZERO	     :
    TempGpio.GpioPtr = GPIOD;
    TempGpio.PortPin = (GPIO_PIN_11);
    break;
  case HWDRIVERS_BUZZER		         :
    TempGpio.GpioPtr = GPIOD;
    TempGpio.PortPin = (GPIO_PIN_12);
    break;
  case HWDRIVERS_CHARGER_DISABLE	     :
    TempGpio.GpioPtr = GPIOD;
    TempGpio.PortPin = (GPIO_PIN_15);
    break;
  case HWDRIVERS_RS485_ENB		     :
    TempGpio.GpioPtr = GPIOD;
    TempGpio.PortPin = (GPIO_PIN_0);
    break;
  case HWDRIVERS_CHRG_FALT_INDICATION	 :
    TempGpio.GpioPtr = GPIOE;
    TempGpio.PortPin = (GPIO_PIN_3);
    break;
  case HWDRIVERS_CHRG_INDICATION	     :
    TempGpio.GpioPtr = GPIOE;
    TempGpio.PortPin = (GPIO_PIN_4);
    break;
  case HWDRIVERS_RELAYS_LATCH		     :
    TempGpio.GpioPtr = GPIOE;
    TempGpio.PortPin = (GPIO_PIN_7);
    break;
  case HWDRIVERS_LATCH_DATA_0		     :
    TempGpio.GpioPtr = GPIOE;
    TempGpio.PortPin = (GPIO_PIN_8);
    break;
  case HWDRIVERS_LATCH_DATA_1		     :
    TempGpio.GpioPtr = GPIOE;
    TempGpio.PortPin = (GPIO_PIN_9);
    break;
  case HWDRIVERS_LATCH_DATA_2		     :
    TempGpio.GpioPtr = GPIOE;
    TempGpio.PortPin = (GPIO_PIN_10);
    break;
  case HWDRIVERS_LATCH_DATA_3		     :
    TempGpio.GpioPtr = GPIOE;
    TempGpio.PortPin = (GPIO_PIN_11);
    break;
  case HWDRIVERS_LATCH_DATA_4		     :
    TempGpio.GpioPtr = GPIOE;
    TempGpio.PortPin = (GPIO_PIN_12);
    break;
  case HWDRIVERS_LATCH_DATA_5		     :
    TempGpio.GpioPtr = GPIOE;
    TempGpio.PortPin = (GPIO_PIN_13);
    break;
  case HWDRIVERS_LATCH_DATA_6		     :
    TempGpio.GpioPtr = GPIOE;
    TempGpio.PortPin = (GPIO_PIN_14);
    break;
  case HWDRIVERS_LATCH_DATA_7		     :
    TempGpio.GpioPtr = GPIOE;
    TempGpio.PortPin = (GPIO_PIN_15);
    break;
  case HWDRIVERS_LATCH_DATA_8		     :
    TempGpio.GpioPtr = GPIOA;
    TempGpio.PortPin = (GPIO_PIN_10);
    break;

  case HWDRIVERS_PC5_TEST:
    TempGpio.GpioPtr = GPIOC;
    TempGpio.PortPin = (GPIO_PIN_5);
    break;

  case HWDRIVERS_PC10_TEST:
    TempGpio.GpioPtr = GPIOC;
    TempGpio.PortPin = (GPIO_PIN_10);
  break;

  case HWDRIVERS_PB14_USB_TEST:
    TempGpio.GpioPtr = GPIOB;
    TempGpio.PortPin = (GPIO_PIN_14);
  break;

  case HWDRIVERS_PB15_USB_TEST:
    TempGpio.GpioPtr = GPIOB;
    TempGpio.PortPin = (GPIO_PIN_15);
  break;

  case HWDRIVERS_PB8_SCL:
    TempGpio.GpioPtr = GPIOB;
    TempGpio.PortPin = (GPIO_PIN_8);
    break;

  case HWDRIVERS_PB9_SDA:
    TempGpio.GpioPtr = GPIOB;
    TempGpio.PortPin = (GPIO_PIN_9);
    break;

  case HWDRIVERS_PC8_SFLASH_RESET_N:
    TempGpio.GpioPtr = GPIOC;
    TempGpio.PortPin = (GPIO_PIN_8);
    break;

  case HWDRIVERS_PE6_MAX_POWER_EN:
 	   TempGpio.GpioPtr = GPIOD;
 	   TempGpio.PortPin = GPIO_PIN_1;
 	  break;


  case HWDRIVERS_PD13_EXT_TX_OUT_MODULATOR:
	   TempGpio.GpioPtr = GPIOD;
	   TempGpio.PortPin = GPIO_PIN_13;
	  break;

  case HWDRIVERS_PC5_MAIN_BUCK_EN:
	   TempGpio.GpioPtr = GPIOC;
	   TempGpio.PortPin = GPIO_PIN_5;
	   break;


  }


  ReturnedValue = HAL_GPIO_ReadPin (TempGpio.GpioPtr, TempGpio.PortPin);

  return(ReturnedValue);
}



/******************************************************************************
* @brief  ReturnCode_T wdriverFreeMemFromIsrInit()
* @param  
* @retval 
******************************************************************************/
ReturnCode_T hwdriverFreeMemFromIsrTaskInit()
{
  // Create input queue, each entry is a uint32_t representing allocated memory to be freed  
  hwdriversMemFreeQueueHandle = xQueueCreate(30, sizeof(uint32_t));
  // Create the task
  xTaskCreate(hwdriverFreeMemFromIsrTask, "hwdriverFreeMemFromIsrTask", 1000, NULL, 1, ( TaskHandle_t * ) NULL );

  return(RETURNCODE_OK);
}


/******************************************************************************
* @brief  portTASK_FUNCTION(hwdriverFreeMemFromIsrTask, pvParameters )
* @param  
* @retval 
******************************************************************************/
portTASK_FUNCTION(hwdriverFreeMemFromIsrTask, pvParameters )
{
  uint32_t PointerToBeFreed;

  while(1)
  {
    // block task till new queue entry is received or timeout 
    BaseType_t  QueueState = xQueueReceive(hwdriversMemFreeQueueHandle, &PointerToBeFreed, 100);

    if(QueueState == pdTRUE)
    {
      if(PointerToBeFreed)
        vPortFree((void*)PointerToBeFreed);
    }
    else
    {
      // Timed out, No issue to handle 
    }
  }
}



/******************************************************************************
* @brief  void hwdriversBleUartTxPinControl(hardwaredriversUart3TxPinState_T Control )
* @param  
* @retval 
******************************************************************************/
//void hwdriversBleUartTxPinControl(hardwaredriversUart3TxPinState_T Control )
//{
//
//  GPIO_InitTypeDef GPIO_InitStruct;
//
//  switch( Control)
//  {
//  case HWDRIVERS_UART3_TX_PIN_GPIO_ZERO:
//	  GPIO_InitStruct.Pin = BLE_UART_TX_Pin;
//	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	  GPIO_InitStruct.Pull = GPIO_NOPULL;
//	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//    GPIO_InitStruct.Alternate = NULL; // Regular Push-pull, alternate is unused
//	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//	  HAL_GPIO_WritePin (GPIOB,  BLE_UART_TX_Pin, GPIO_PIN_RESET);
//    break;
//  case HWDRIVERS_UART3_TX_PIN_UART_MODE:
//	  GPIO_InitStruct.Pin = BLE_UART_TX_Pin;
//	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//	  GPIO_InitStruct.Pull = GPIO_NOPULL;
//	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//	  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
//	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//    break;
//  }
//
//}




void hwdriversUsbCicularBufferInser(uint8_t* Ptr, int Length)
{

	uint16_t i;
	for(i=0; i<Length; i++)
	{
		usbRxCicularBuffer[usbRxCircularHeadIndex] = *(Ptr+i);
		usbRxCircularHeadIndex++;
		if(usbRxCircularHeadIndex >= HWDRIVERS_USB_RX_CIRCULAR_BUFER_LENGTH)
			usbRxCircularHeadIndex = 0;
	}
}

/******************************************************************************
*** @brief  void hwdriversUart1DmaRxDataProcess() - PC
*   @param
*   @retval
******************************************************************************/
void hwdriversUart1DmaRxDataProcess()
{
#if 0
	uint8_t c;
  TM_USB_VCP_Result usb_result = TM_USB_VCP_ERROR;
  ReturnCode_T result = RETURNCODE_OK;

  if(TM_USB_VCP_GetStatus() == TM_USB_VCP_CONNECTED)
  {
    usb_result = TM_USB_VCP_Getc(&c);
    while(usb_result == TM_USB_VCP_DATA_OK)
    {
      result = usartDllRxFsm(c, USARTDLL_FSM_STIMULI_INCOMING_CHAR);
      usb_result = TM_USB_VCP_Getc(&c);
    }
  }

#endif

  	uint16_t i;

  	uint16_t MyLocalHeadIndex = usbRxCircularHeadIndex;

  	// UARTS_UART_CELLULAR
  	if(usbRxCircularTailIndex < MyLocalHeadIndex)
  	{
  		for(i=0; i< MyLocalHeadIndex-usbRxCircularTailIndex; i++)
  			usartDllRxFsm(usbRxCicularBuffer[usbRxCircularTailIndex+i], USARTDLL_FSM_STIMULI_INCOMING_CHAR);
  	}
  	if(usbRxCircularTailIndex > MyLocalHeadIndex)
  	{
  		for(i=0; i< HWDRIVERS_USB_RX_CIRCULAR_BUFER_LENGTH-usbRxCircularTailIndex; i++)
 			usartDllRxFsm(usbRxCicularBuffer[usbRxCircularTailIndex+i], USARTDLL_FSM_STIMULI_INCOMING_CHAR);
  		for(i=0; i< MyLocalHeadIndex; i++)
			usartDllRxFsm(usbRxCicularBuffer[i], USARTDLL_FSM_STIMULI_INCOMING_CHAR);
  	}
  	usbRxCircularTailIndex = MyLocalHeadIndex;

}

/******************************************************************************
*** @brief  Get the relay state number directly from the output port
*   @param none
*   @retval 16bit num of the current relay state
******************************************************************************/
uint16_t hwdriversRelayStateGet()
{
  uint16_t MyTemp = GPIOE->ODR;
  // Mask the relevant bits
  MyTemp &= 0xff00;
  // Move to LSB
  MyTemp = MyTemp >> 8;
  return MyTemp;
}


