
#include <hwdrivers.h>
#include <stdlib.h>
#include <math.h> 
#include <stdarg.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "vlapConfig.h"
#include "pccommAppLayer.h"
#include "timers.h"
//#include "bitband.h"
#include "audit.h"
#include "autoresonance.h"
#include "transQ.h"


transQStatus transQInit(uint8_t NumberOfElements, QueueHandle_t *QueuePtr)
{
  
  *QueuePtr = xQueueCreate(NumberOfElements, sizeof(transQEntry_t));
  return TRANSQ_OK;
}

transQStatus transQEnqueue(QueueHandle_t MyQueue, void * DataPtr, uint32_t Length, uint8_t MemoryFreeNeeded)
{
  BaseType_t rc;
  transQEntry_t transQElt;
  transQElt.MemoryFreeNeeded = MemoryFreeNeeded;
  transQElt.UserDataPtr = DataPtr;
  transQElt.Length = Length;

  rc = xQueueSend(MyQueue, &transQElt, 0);
  if (rc != pdPASS)
    return  TRANSQ_FULL;
  else
    return TRANSQ_OK;
}

transQStatus transQDequeue(QueueHandle_t MyQueue, transQEntry_t* ReturnedQueueEntryPtr)
{
  BaseType_t rc;
  rc = xQueueReceive(MyQueue, ReturnedQueueEntryPtr, 0);  

  if (rc != pdPASS)
    return TRANSQ_EMPTY;
  else
    return TRANSQ_OK;
}

transQStatus transQDequeueFromISR(QueueHandle_t MyQueue, transQEntry_t* ReturnedQueueEntryPtr)
{
  xQueueReceiveFromISR(MyQueue, ReturnedQueueEntryPtr, NULL); 
  return TRANSQ_OK;
}

transQStatus transQTop(QueueHandle_t MyQueue, transQEntry_t* ReturnedQueueEntryPtr)
{
  BaseType_t rc;
  rc = xQueuePeek(MyQueue, ReturnedQueueEntryPtr, 0); 
  if (rc == pdPASS)
    return TRANSQ_OK;
  else
    return TRANSQ_EMPTY;
}

