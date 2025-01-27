
#pragma once

#include "General.h"
#include "stm32h7xx.h"

typedef enum _EventType
  {
    EMPTY_ENTRY,

    // Various logging events:
    DMA_BUFFER_CALLBACK,
    DMA_BUFFER_HANDLED,
    DMA_BUFFER_STATISTICS,
    
    DEMODULATOR_TIMING,
    
    POLLINGS_IN_50_KHZ_CLOCK,
    
    TRANSMISSION,
    PWM_SETTING,
    
    CANDIDATES_REPORT,

    CANDIDATE_TESTING,
    
    RX_MODE,
    
    NOISE_RMS_REPORT,
    // Exceptions
    UNPLANNED_TRANSITION_PATTERN,
    
    DMA_BUFFER_HANDLING_OVERRUN,
    // Errors
    DMA_BUFFER_DEMODULATION_FAILURE
  }EventType;

// Release ver - exclude for debugging:
//#define RELEASE_VER
#ifdef RELEASE_VER
#define InitLog()
#define Log(evType, val, param1, param2)
#else
void InitLog(void);
void Log(EventType evType,WORD_16_BITS val,WORD_16_BITS param1,WORD_16_BITS param2);
#endif

/**
 * @brief  Private Service: Get the Dedicated-timer timestamp Sub seconds value.
 * @param  None
 * @retval Dedicated-timer current timestamp Sub seconds value.
 */
uint32_t LocalApp_GetTimeStampSubSecond(void);
uint32_t LocalApp_GetTimeStamp50uSUnits(void);
