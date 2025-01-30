#ifndef HWDRIVERS_H
#define HWDRIVERS_H

/* Includes ------------------------------------------------------------------*/

#include "FreeRTOS.h"
#include "queue.h"
#include "timers.h"

#include "common.h"
#include "uartdll.h"
#include <stdio.h>
#include "common.h"
#include "stm32h7xx.h"

#if defined(USE_STM324xG_EVAL)
#include "stm324xg_eval.h"
#include "stm324xg_eval_lcd.h"

#elif defined(USE_STM324x7I_EVAL)
#include "stm324x7i_eval.h"
#include "stm324x7i_eval_lcd.h"

#elif defined(USE_STM324x9I_EVAL)
#include "stm324x9i_eval.h"
#include "stm324x9i_eval_lcd.h"

#include "audit.h"

#else
// #error "Please select first the Evaluation board used in your application (in Project Options)"
#endif

// G L O B A L   T Y P E S   A N D    D E F I N I T I O N S
/* Private define ------------------------------------------------------------*/
/* used to display the ADC converted value on LCD */
// #define USE_LCD
/* if you are not using the LCD, you can monitor the converted value by adding
the variable "uhADCxConvertedValue" to the debugger watch window */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

#define UART_RX_BUFFER_LENGTH /*256*/ 1100
#define HWDRIVERS_TX_MONITORING_INPUT_OFFSET 200
extern uint8_t UartRxBuffer[UART_RX_BUFFER_LENGTH];
extern uint8_t flashBuffer2Required;
;

// G L O B A L   T Y P E S
typedef struct
{
  uint16_t BattVoltage;          // resolution of 100mV
  uint16_t DcInputVoltage;       // Resolution of 100mV
  uint16_t DcVbatCurrent;        // Vbat Current - Transmitter power current in mA
  uint8_t ChargingStatus;        // TRUE / FALSE
  uint8_t DcPlugConnectionState; // DC plug conencted TRUE/FLASE
  uint8_t DcPlugVoltageExist;    // DC voltage exist
} hwdriverPowerMeasurments_t;

enum WaitDelayEnum
{
  eDELAY404NSEC = 0,
  eDELAY592NSEC = 1,
  eDELAY1USEC = 4,
  eDELAY1_4USEC = 5,
  eDELAY30USEC = 158,
  eDELAY200USEC = 1065,
  eDELAY400USEC = 2135,
  eDELAY500USEC = 2675,
  eDELAY1MSEC = 5320,
  eDELAY2MSEC = 10665,
  eDELAY4MSEC = 21270,
  eDELAY10MSEC = 53500,
  eDELAY100MSEC = 537000,
  eDELAY0_5SEC = 2660000,
  eDELAY1SEC = 5320000
};

typedef enum
{
  HWDRIVERS_PHASE_UNDER,
  HWDRIVERS_PHASE_ZERO,
  HWDRIVERS_PHASE_OVER
} hardwaredriversPhaseStatus_T;

typedef enum
{
  HWDRIVERS_UART3_TX_PIN_GPIO_ZERO,
  HWDRIVERS_UART3_TX_PIN_UART_MODE
} hardwaredriversUart3TxPinState_T;

typedef enum
{
  HWDRIVERS_I2C_LINES_AF,
  HWDRIVERS_I2C_OUTPUT_LOW,
  HWDRIVERS_I2C_OUTPUT_TOGGLE_LOW,
  HWDRIVERS_I2C_OUTPUT_TOGGLE_HIGH
} hardwaredriversI2CLinesState_T;

#define HWDRIVERS_RELAY_0 MY_BITBAND_PERI(GPIOC, HWDRIVERS_ODR_OFFSET, 4)

// L O C A L    T Y P E S   A N D    D E F I N I T I O N S
// Relays

typedef enum
{
  HWDRIVERS_TX_NTC,
  HWDRIVERS_TX_VDD_PA,
  HWDRIVERS_TX_ISMON_PA,
  HWDRIVERS_TX_MONITOR,
} hwdriversAdc2InputsT;

#define UART_RX_BUFFER_LENGTH /*256*/ 1100
extern uint8_t UartRxBuffer[UART_RX_BUFFER_LENGTH];

#// Define base address of bit-band
#define BITBAND_SRAM_BASE 0x20000000
// Define base address of alias band
#define ALIAS_SRAM_BASE 0x22000000
// Convert SRAM address to alias region
#define BITBAND_SRAM(a, b) ((ALIAS_SRAM_BASE + (a - BITBAND_SRAM_BASE) * 32 + (b * 4)))

// Define base address of peripheral bit-band
#define BITBAND_PERI_BASE 0x40000000
// Define base address of peripheral alias band
#define ALIAS_PERI_BASE 0x42000000
// Convert PERI address to alias region
#define BITBAND_PERI(a, b) (volatile uint32_t *)((uint32_t)ALIAS_PERI_BASE + ((uint32_t)a - (uint32_t)BITBAND_PERI_BASE) * 32 + ((uint32_t)b * 4))

#define HWDRIVERS_ODR_OFFSET 0x14

#define MY_BITBAND_PERI(gpioBaseAddress, gpioRegOffset, gpioBitNumber) (volatile uint32_t *)(((uint32_t)PERIPH_BB_BASE + ((uint32_t)gpioBaseAddress - (uint32_t)PERIPH_BASE + (uint32_t)gpioRegOffset) * 32) + ((uint32_t)gpioBitNumber * 4))

#define HEDRIVERS_VDD 3.3
#define HWDRIVERS_DEFAULT_TX_PWM_PERIOD 16
#define HWDRIVERS_DEFAULT_TX_PWM_CAPTURE 14
#define R48 8.06e3
#define R69 (double)499
#define R105 100e3
#define VD16 0.15

#define R62 150e3
#define R47 100e3

// GPIOS
typedef enum
{
  HWDRIVERS_SENSORS_ENABLE,
  HWDRIVERS_BLE_POWER_ENABLE,
  HWDRIVERS_MAIN_RED_LED,
  HWDRIVERS_MAIN_GREEN_LED,
  HWDRIVERS_MAIN_BLUE_LED,
  HWDRIVERS_USB_RED_LED,
  HWDRIVERS_USB_GREEN_LED,
  HWDRIVERS_USB_BLUE_LED,
  HWDRIVERS_VIBRATOR,
  HWDRIVERS_MAIN_OSC_ENABLE,
  HWDRIVERS_TX_SUP_ENABLE,
  HWDRIVERS_UI_ENABLE,
  HWDRIVERS_ACCL_GYRO_INT,
  HWDRIVERS_CHRG_CRNT_CNTL,
  HWDRIVERS_BLE_RESET,
  HWDRIVERS_BLE_BOOT,
  HWDRIVERS_PHASE_SENSE_OVER,
  HWDRIVERS_PHASE_SENSE_UNDER,
  HWDRIVERS_PHASE_SENSE_ZERO,
  HWDRIVERS_BUZZER,
  HWDRIVERS_CHARGER_DISABLE,
  HWDRIVERS_RS485_ENB,
  HWDRIVERS_CHRG_FALT_INDICATION,
  HWDRIVERS_CHRG_INDICATION,
  HWDRIVERS_VIN_MEASURE_CNTL,
  HWDRIVERS_RELAYS_LATCH,
  HWDRIVERS_LATCH_DATA_0,
  HWDRIVERS_LATCH_DATA_1,
  HWDRIVERS_LATCH_DATA_2,
  HWDRIVERS_LATCH_DATA_3,
  HWDRIVERS_LATCH_DATA_4,
  HWDRIVERS_LATCH_DATA_5,
  HWDRIVERS_LATCH_DATA_6,
  HWDRIVERS_LATCH_DATA_7,
  HWDRIVERS_LATCH_DATA_8,
  HWDRIVERS_PC5_TEST,
  HWDRIVERS_PC10_TEST,
  HWDRIVERS_PB14_USB_TEST,
  HWDRIVERS_PB15_USB_TEST,
  HWDRIVERS_PB8_SCL,
  HWDRIVERS_PB9_SDA,
  HWDRIVERS_PC8_SFLASH_RESET_N,
  HWDRIVERS_PE6_MAX_POWER_EN,
  HWDRIVERS_PUSH_BUTTON_IN,
  HWDRIVERS_PD13_EXT_TX_OUT_MODULATOR,
  HHWDRIVERS_PC5_MAIN_BUCK_ENWDRIVERS_PC3_TX_SUP_ENABLE,
  HWDRIVERS_PC5_MAIN_BUCK_EN
} HwdriversGpiosT;

// G L O B A L  P R O T O T Y P E S
void hwdriversAdc1Config(uint8_t *BuffPtr1, uint8_t *BuffPtr2, uint16_t BuffersLength);
void hwdriversAdc1IRQHandler(void);
void hwdriversAdc2Config(void);
uint32_t hwdriversAdc2VbatVoltageGet(void);
void hwdriversDma2Stream4_IRQHandler(void);
void hwdriversClockInit(void);

void hwdriversUartsDmaRxConfig(void);

void hwdriversUartsConfig(void);

void hwdriversUart1DmaRxDataProcess();
void hwdriversUart2DmaRxDataProcess();
void hwdriversUart3DmaRxDataProcess();
void hwdriversUart6DmaRxDataProcess();

void hwdriversUartDmaTx(uint8_t *s, uint16_t length);
void hwdriversuartDmaRxDataGet();

void hwdriversUart1DmaTx(uint8_t *s, uint16_t length);
void hwdriversUart2DmaTx(uint8_t *s, uint16_t length);
void hwdriversUart3DmaTx(uint8_t *s, uint16_t length);
void hwdriversUart6DmaTx(uint8_t *s, uint16_t length);

void hwdriversDmaConfig(void);
void hwdriversDma2Stream7_IRQHandler(void); // USART1_TX
void hwdriversDma2Stream2_IRQHandler(void); // USART1_RX
void hwdriversNvicConfig(void);
void hwdriversSpiConfig(void);
void hwdriversTimer4Init(void);
ReturnCode_T hwdriversPowerMeasurmentsGet(hwdriverPowerMeasurments_t *ReturnedPowerMeasurmentsPtr);

void hwdriversStartCycleCounter();
long hwdriversStopCycleCounter();
void hwdriversSpiFlush(SPI_TypeDef *SPIx);
void hwdriversSpiDmaTx(SPI_TypeDef *SPIx, uint8_t *s, uint16_t length);
void hwdriversSpiDmaRx(SPI_TypeDef *SPIx, uint8_t *s, uint16_t length);

void hwdriversI2cConfig(void);
void hwdriversSpiFlashConfig(void);

void hwdriversSensorsConfig(void);
// giorauint8_t hwdriverPsGetDeviceId(void);
void hwdriversTimer7Init(void);
void hwdriversTimer3Init();

void floatToInt(float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec);
void hwdriversSetLed(uint8_t onOff);
void hwdriversPVDConfig(void);
void hwdriverAutoResonanceConfig(void);
uint32_t hwdriversLinuxTimeStampGet(void);
uint8_t hwdrivers10mSecGet(void);
hardwaredriversPhaseStatus_T hwdriversPhaseSenseGet(hardwaredriversPhaseStatus_T previousPhase);
ReturnCode_T hwdriversRelaysChangeActivate(uint16_t NewRelaysValue);
uint16_t hwdriversRelayStateGet();
ReturnCode_T hwdriversRelaysChangeIdle();
ReturnCode_T hwdriversPwmSet(uint16_t OutputState, uint16_t TimerPeriod, uint16_t TimerCapture);
ReturnCode_T hwdriversPwmControl(typesControl_T Control);
uint16_t autoresonanceRelaysStateGet();
uint8_t hwdriversPhaseRawDataGet();
ReturnCode_T hwdriversGpioSet(HwdriversGpiosT RegID, uint8_t BitState);

uint32_t hwdriversCyclesCounterGet();
ReturnCode_T hwdriversNtcTemperatureGet(int16_t *ReturnedNtcTemperaturePtr);
ReturnCode_T hwdriversVbatVoltageGet(uint16_t *ReturnedVbatVoltagePtr);
ReturnCode_T hwdriversFrequencyMonitoringGet(uint16_t *ReturnedRequencyPtr);
void hwdriversTxPowerGet(int16_t *ReturnedCurrentPtr, int16_t *ReturnedVoltagePtr);
ReturnCode_T hwdriversTxMonitorGet(uint16_t *ReturnedTxMonitorLevelPtr);
ReturnCode_T hwdriversGpioBitWrite(HwdriversGpiosT RegID, uint8_t BitState);
ReturnCode_T hwdriversTxProcessing();
ReturnCode_T hwdriverFreeMemFromIsrTaskInit();
void hwdriversTimersStart();
void hwdriversModemAdcSignalSourceSelect(uint8_t AdcSignalSourceSelect);
void vmicmodemAgc(uint32_t PeakToPeakSignal);
void hwdriversBleUartTxPinControl(hardwaredriversUart3TxPinState_T Control);
//void hwdriversVinGpioConfigure(commSleepState_T SleepState);
void hwdriversI2CLinesGpioConfigure(hardwaredriversI2CLinesState_T i2cLinesState);
void hwdriversUart3DmaTx(uint8_t *s, uint16_t length);

uint32_t hwdriversCycleCounterGet();
void hwdriversFrequencyMonitoringPeriodIsr(TIM_HandleTypeDef *htim);

uint8_t hwdriversGpioBitRead(HwdriversGpiosT RegID);

void hwdriversReInitUart3RxDma();
void hwdriversReInitUart1PcRxDma();
void hwdriversReInitUartBleRxDma();

void hwdriversUsbCicularBufferInser(uint8_t *Ptr, int Length);

// void hwdriversVinInterruptConfig(void);
// void Configure_PA0(void);

#endif
