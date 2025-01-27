
#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "rtc.h"
#include "queue.h"
#include "uartDll.h"
#include "events.h"

// unhide the following line to enable debugging, If enabled during
// Runtime execution (without debugger), the program will stop after 4 seconds !!!
#define VLAP_DEBUG

// use the following define in various places
#define RELEASE_VER
#define BLINK_GREEN_LED
#define USE_BITBANDING
//#define USE_PVD
#define RF_TEST_11_08_16
//#define USE_BLUE_PUSHBUTTON_EXT_INT
//#define USE_RTC_BKP_FOR_POINTERS
#define USE_SPI_FLASH
#define USE_AUTORESONANCE
//#define HEAP_2_WA


//#define USE_USART1_FOR_DEBUGGING
#define USE_TIM1
#define USE_I2C
#define USE_PRESSURE_SENSOR
//#define USE_ACCELEROMETER
//#define USE_IO_EXPANDER
//#define USE_NUCLEO_BOARD


//#define includeVlapTest1
//#define includeVlapTest2
//#define includeVlapTest3

#ifdef SPI_SAME_BOARD_TEST
#define SPI_MASTER SPI1
#define SPI_SLAVE  SPI3
#endif
#define PRIORITY_CHECKS_OK

#ifdef VLAP_DEBUG
#include <stdio.h>
#define DEBUG_REGISTER_DHCSR (* ((volatile uint32_t *) 0xe000edf0)) // see cortex technical ref manual  core debug register:
// http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.ddi0337e/BHCFCFDB.html
#define DEBUG_ENABLED 0
#define vdbg(...) {if (DEBUG_REGISTER_DHCSR & DEBUG_ENABLED) printf(__VA_ARGS__);}
#else
#define vdbg(...)
#endif

//
// use the following defines for task traces
//
//#define vlapTraceTASK_SWITCHED_IN() vlapTraceTaskSwitchIn()

//
// tasks
//

// main
#define vlapMainName "VMAIN"
#define vlapMainSTACK_SIZE configMINIMAL_STACK_SIZE
#define vlapMainPriority (tskIDLE_PRIORITY + 1UL)
extern void vlapMain(void *pvParameters);

// clientsPresTemp
#define clientsPresTempName "CLI1"
#define clientsPresTempSTACK_SIZE configMINIMAL_STACK_SIZE
#define clientsPresTempPriority (tskIDLE_PRIORITY + 1UL)
extern void clientsPresTemp(void *pvParameters);


// i2cworkSchedulerTask
#define i2cserverName "I2C_SCHEDULER_TASK"
#define i2cserverSTACK_SIZE configMINIMAL_STACK_SIZE
#define i2cserverPriority (tskIDLE_PRIORITY + 1UL)
extern void i2cworkSchedulerTask(void *pvParameters);

// eventsTask
#define eventsTaskName "EVENTS_TASK"
#define eventsTaskSTACK_SIZE 400  // configMINIMAL_STACK_SIZE //configMINIMAL_STACK_SIZE_ENC
#define eventsPriority (tskIDLE_PRIORITY + 1UL)
extern void eventsTask(void *pvParameters);

// spiflashTask
#define spiflashTaskName "SPIFLASH_TASK"
#define spiflashTaskSTACK_SIZE configMINIMAL_STACK_SIZE
#define spiflashTaskPriority (tskIDLE_PRIORITY + 1UL)
extern void spiflashTask(void *pvParameters);

// spiflashdiskTask
#define spiflashdiskTaskName "SFLSHDSK_TASK"
#define spiflashdiskTaskSTACK_SIZE configMINIMAL_STACK_SIZE
#define spiflashdiskTaskPriority (tskIDLE_PRIORITY + 1UL)
extern void spiflashdiskTask(void *pvParameters);



// measureTask
#define measureTaskName "MEASURE_TASK"
#define measureTaskSTACK_SIZE configMINIMAL_STACK_SIZE
#define measureTaskPriority (tskIDLE_PRIORITY + 1UL)
extern void measureTask(void *pvParameters);

// commTask
#define commTaskName "COMM_TASK"
#define commTaskSTACK_SIZE 400
#define commTaskPriority (tskIDLE_PRIORITY + 1UL)
extern void commTask(void *pvParameters);

// commTask
#define uartdllTaskName "UARTDLL_TASK"
#define uartdllTaskSTACK_SIZE configMINIMAL_STACK_SIZE  //configMINIMAL_STACK_SIZE_ENC
#define uartdllTaskPriority (tskIDLE_PRIORITY + 1UL)
extern void uartdllTask(void *pvParameters);

// buzzerTask
#define buzzerTaskName "BUZZER_TASK"
#define buzzerTaskSTACK_SIZE configMINIMAL_STACK_SIZE
#define buzzerTaskPriority (tskIDLE_PRIORITY + 1UL)
extern void buzzerTask(void *pvParameters);

// vibratorTask
#define vibratorTaskName "VIBRATOR_TASK"
#define vibratorTaskSTACK_SIZE configMINIMAL_STACK_SIZE
#define vibratorTaskPriority (tskIDLE_PRIORITY + 1UL)
extern void vibratorTask(void *pvParameters);

// ledsApplicationTask
#define ledsApplicationTaskName "LEDS_APPLICATION_TASK"
#define ledsApplicationTaskSTACK_SIZE configMINIMAL_STACK_SIZE
#define ledsApplicationTaskPriority (tskIDLE_PRIORITY + 1UL)
extern void ledsApplicationTask(void *pvParameters);

// sfServer
#define sfserverName "SFSER"
#define sfserverSTACK_SIZE configMINIMAL_STACK_SIZE
#define sfserverPriority (tskIDLE_PRIORITY + 1UL)
extern void sfserver(void *pvParameters);

// demodulator
#define vlapDemodulatorName "VDEMOD" // reserved name. dont change. see task.c
#define vlapDemodulatorSTACK_SIZE configMINIMAL_STACK_SIZE
#define vlapDemodulatorPriority (tskIDLE_PRIORITY + 1UL)
extern void vlapDemodulator(void *pvParameters);


// configTask
#define configTaskName "CONFIG_TASK" 
#define configTaskSTACK_SIZE /*configMINIMAL_STACK_SIZE*/ 400
#define configTaskPriority (tskIDLE_PRIORITY + 1UL)
extern void configTask(void *pvParameters);


// upgradeTask
#define fwupgradeTaskName "UPGRD_TASK" 
#define fwupgradeTaskSTACK_SIZE 500 //configMINIMAL_STACK_SIZE
#define fwupgradeTaskPriority (tskIDLE_PRIORITY + 1UL)
extern void fwupgradeTask(void *pvParameters);



#define GENERAL_PURPOSE_CTR_RANGE                       0xEFFFF99A//0x100000//0x1000//0xC000=>0.73 u//200000
#define DIVIDOR_50_USECS                                1706
#define UNINITIALIZED_CTR_VALUE                         (GENERAL_PURPOSE_CTR_RANGE + 1)
#define USECS_GRANULARITY                               50

#define WRAP_AROUND_50_USECS_QUANTITY                   (GENERAL_PURPOSE_CTR_RANGE/DIVIDOR_50_USECS)    

extern int vlapMainBusy;
   
typedef enum
  {
    VMIC_MODEM
  }TimerClientType;

typedef enum
  {       
    NULL_INTERRUPT,
    FIRST_DMA_BUFFER_INTERRUPT,
    SECOND_DMA_BUFFER_INTERRUPT,
    SPURIOUS_INTERRUPT
  }PendingInterruptType;

typedef struct qelt_t
{
  int number;
  char message[120];
}QELT;


typedef enum
  {
    PRESSURE_READ,
    TEMPERATURE_READ,
    PRES_TEMP_READ,
    AXES_READ,
    ANGULAR_VELOCITY_READ,
    BUFFER_READ,
    BUFFER_WRITE,
  }I2CREQ_OPCODE;



typedef struct i2creq_t
{
  I2CREQ_OPCODE opcode; 
  uint32_t      period; // sampling rate x0.01 sec
  QueueHandle_t *i2cRespQ;  
  

  // the following variables are used by BUFFER_READ and BUFFER_WRITE
  uint8_t        deviceAddr;
  uint8_t        regAddr;
  uint16_t       numBytes;
  uint8_t       *pBuffer;
  QueueHandle_t InitiatingQueueHandle;
} I2CREQ;

#define VALID_STRUCT 97965944

typedef struct i2cresp_t
{
  volatile struct i2cresp_t *this;
  volatile uint32_t validStruct;
  volatile uint16_t size;
  volatile uint8_t *data;
} I2CRESP;

//extern QueueHandle_t i2cReqQ;


typedef struct sfresp_t
{
  volatile struct sfresp_t *this;
  volatile uint32_t validStruct;
  volatile int size;
  volatile char *data;
} SFRESP;

extern QueueHandle_t sfReqQ;
extern QueueHandle_t sfReqQ1;

#define BUFFERSIZE 2048
extern uint16_t buffer[2][BUFFERSIZE];

void mainAdcDmaTransferCompleteIsr(uint8_t BufferIndicationFlag);

void mainCyclesMeasure(uint32_t Param);
void mainCyclesClear();
void gdbg(const char* format, ...);

typedef enum
  {
    SF_256,
    SF_264,
  }SF_PAGE_SIZE;

typedef enum
  {
    ELOG_NOT_FULL,
    ELOG_FULL,
  }ELOG_STATE;

typedef struct sf_elog_ptr_typ
{
  // persistent elog pointers struct
  uint32_t validStruct;
  uint32_t elogRptr;
  uint32_t elogWptr;
  uint32_t elogEntSize;
#ifdef USE_PVD
  uint8_t pad[256-16];
#endif
} SF_ELOG_PTRS;



#define BITNR(x) (x&0x1?0:x&0x2?1:x&0x4?2:x&0x8?3:                      \
                  x&0x10?4:x&0x20?5:x&0x40?6:x&0x80?7:                  \
                  x&0x100?8:x&0x200?9:x&0x400?10:x&0x800?11:            \
                  x&0x1000?12:x&0x2000?13:x&0x4000?14:x&0x8000?15:      \
                  x&0x10000?16:x&0x20000?17:x&0x40000?18:x&0x80000?19:  \
                  x&0x100000?20:x&0x200000?21:x&0x400000?22:x&0x800000?23: \
                  x&0x1000000?24:x&0x2000000?25:x&0x4000000?26:x&0x8000000?27: \
                  x&0x10000000?28:x&0x20000000?29:x&0x40000000?30:x&0x80000000?31:32)


