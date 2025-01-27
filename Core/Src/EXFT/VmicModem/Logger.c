#include <string.h>
#include "Logger.h"
#include "vlapConfig.h"

#define LOG_ENTRIES 512

#define US_50_DIVIDOR   1706

typedef struct _LogEntry
{
  EventType evType;
  WORD_16_BITS val;
  WORD_16_BITS param1;
  WORD_16_BITS param2;
}LogEntry;





/**
 * @brief  Private Service: Get the Dedicated-timer timestamp Sub seconds value.
 * @param  None
 * @retval Dedicated-timer current timestamp Sub seconds value.
 */
uint32_t LocalApp_GetTimeStampSubSecond(void)
{
  /* Get timestamp sub seconds values from the correspondent registers */
  return (uint32_t)(TIM6->CNT);
}

/**
 * @brief  Private Service: Get the Dedicated-timer timestamp in 50uSecs units:
 * @param  None
 * @retval Dedicated-timer current timestamp Sub seconds value.
 */
uint32_t LocalApp_GetTimeStamp50uSUnits(void)
{
  /* Get timestamp sub seconds values from the correspondent registers */
  return (LocalApp_GetTimeStampSubSecond()/US_50_DIVIDOR);
}

#ifndef RELEASE_VER

void InitLog(void)
{
  memset(m_LogBuf,EMPTY_ENTRY,sizeof(m_LogBuf));
}

void Log(EventType evType,WORD_16_BITS val,WORD_16_BITS param1,WORD_16_BITS param2)
{

  //   if ((evType == PWM_SETTING) || (evType == TRANSMISSION))
  //  if (evType != DEMODULATOR_TIMING)
  if ((evType == DMA_BUFFER_HANDLED) || (evType == RX_MODE) || (evType == CANDIDATE_TESTING) || (evType == NOISE_RMS_REPORT))
  {
    m_LogBuf[m_LogBufEntry].evType = evType;
    m_LogBuf[m_LogBufEntry].val = val;
    m_LogBuf[m_LogBufEntry].param1 = param1;
    m_LogBuf[m_LogBufEntry].param2 = param2;

    m_LogBufEntry = SUCCESSOR(m_LogBufEntry,LOG_ENTRIES);

    if (!m_LogBufEntry)
      m_LogBufEntry = 1;
    memset(&m_LogBuf[m_LogBufEntry],EMPTY_ENTRY,sizeof(m_LogBuf[m_LogBufEntry]));
  }
}
#endif
