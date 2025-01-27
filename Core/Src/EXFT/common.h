/******************************************************************************
 * File name	:  	types.h
 *
 *
 ********************************************************************************/

#ifndef TYPES_H_
#define TYPES_H_

#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stm32h7xx.h"


//#include <intrinsics.h>
//#include <malloc.h>

//#include "em_device.h"
//#include "em_usart.h"
//#include "em_cmu.h"
//#include "em_rtc.h"
//#include "em_gpio.h"
//#include "em_int.h"






// E X T E R N A L    H A L    H A N D L E R S

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern DAC_HandleTypeDef hdac1;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern RTC_HandleTypeDef hrtc;
extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim10;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;





#ifndef TRUE
#define TRUE         true
#endif
#ifndef FALSE
#define FALSE        false
#endif

#ifndef NULL
#define NULL 		(	(void *)0	)
#endif

#ifndef IN
#define IN 			const
#endif

#ifndef OUT
#define OUT
#endif

#ifndef INOUT
#define INOUT
#endif

#ifndef __INLINE
#define __INLINE			inline
#endif

#ifndef __STATIC_INLINE
#define __STATIC_INLINE		static inline
#endif

#define PA_HANDLE						uint32_t	// Peripheral Access Handle


#ifndef ReturnCode_T
typedef enum  { RETURNCODE_OK=0, RETURNCODE_LESS_THAN=1,  RETURNCODE_GREATER_THAN=2,  RETURNCODE_EQUAL=3, RETURNCODE_ERROR=-1, RETURNCODE_NOT_COMPLETED=-2, RETURNCODE_UNSUPPORTED=-3, NO_RESPONSE_NEEDED=-4, RETURNCODE_ADDRESS_NOT_MATCH=-5, RETURNCODE_WRONG_HEADER_LENGTH=-6, RETURNCODE_WRONG_SRC_ADDRESS=-7, RETURNCODE_WRONG_DST_ADDRESS=-8, RETURNCODE_WRONG_INPUT_EVENT=-9, RETURNCODE_NO_EMPTY_ENTRIES=-10, RETURNCODE_OUT_OF_RANGE=-11, RETURNCODE_LOGMEMORY_FULL= -12, RETURNCODE_LOGMEMORY_NOTFULL= -13 , RETURNCODE_TX_QUEUE_FULL= -14, RETURNCODE_NTC_OVER_HEAT= -15} ReturnCode_T;
#endif

typedef enum  { TYPES_DISABLE=0, TYPES_ENABLE=1} typesControl_T;

typedef enum { TYPES_RCVQUEU_TIMEOUT, TYPES_RCVQUEU_RCV} typesRcvQueueStatus_T;


/******************************************************************************
 * 		ERROR OPCODES
 *****************************************************************************/
typedef enum
{
	DRV_RETVAL_OK						= 0		,

	DRV_RETVAL_UART_OFFSET				= 10 	,
	DRV_UART_RETVAL_INVALID_PARAM
}DRV_RETVAL_OPCODES_e ;

// TODO (PHASE2): types.h - Temp Placement
/******************************************************************************
* 	ENUM		: uart_baud_e
*	Description	: UART baud rate options enumerator
*****************************************************************************/
typedef enum
{
	UART_BAUDRATE_4800		= 4800		,
	UART_BAUDRATE_9600		= 9600		,
	UART_BAUDRATE_14400		= 14400		,
	UART_BAUDRATE_19200		= 19200		,
	UART_BAUDRATE_38400		= 38400		,
	UART_BAUDRATE_57600		= 57600		,
	UART_BAUDRATE_115200	= 115200	,
	UART_BAUDRATE_AUTO					,
	//----------------------------------//
	UART_BAUDRATE_NUM					// Number of BAUDRATE options, must be the last!
}uart_baud_e;


#define TYPES_ENDIAN16_CHANGE(A) ((((uint16_t)(A) & 0xff00) >> 8) | \
(((uint16_t)(A) & 0x00ff) << 8))

#define TYPES_ENDIAN32_CHANGE(A) ( __REV(A)  )


#endif /* TYPES_H_ */
