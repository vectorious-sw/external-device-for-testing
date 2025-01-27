
#pragma once

#include "vlapConfig.h"

typedef  enum  {TRANSQ_OK, TRANSQ_QCB_PNTR_ERR, TRANSQ_QELEMENT_MALLOC_ERROR, TRANSQ_FULL, TRANSQ_EMPTY} transQStatus;

void transQFirst();

typedef struct
{
  uint8_t       MemoryFreeNeeded;
  void *        UserDataPtr;
  uint16_t      Length; 
} transQEntry_t ;

typedef struct
{
  uint16_t      NumberOfElements;
  transQEntry_t *   HeadPtr;
  transQEntry_t *   TailPtr;
  transQEntry_t *   FirstElementPtr;
  uint16_t      QueuedElements;
}transQControlBlock;

typedef struct
{
  uint8_t       QueueLevel;
  transQStatus    LastQueueState;
}transQState;


transQStatus transQInit(uint8_t NumberOfElements, QueueHandle_t *QueuePtr);
transQStatus transQEnqueue(QueueHandle_t MyQueue, void * DataPtr, uint32_t Length, uint8_t MemoryFreeNeeded);
transQStatus transQDequeue(QueueHandle_t MyQueue, transQEntry_t* ReturnedQueueEntryPtr);
transQStatus transQDequeueFromISR(QueueHandle_t MyQueue, transQEntry_t* ReturnedQueueEntryPtr);
transQStatus transQTop(QueueHandle_t MyQueue, transQEntry_t* ReturnedQueueEntryPtr);
transQStatus transQGetStatus(QueueHandle_t MyQueue, transQState * ReturnedQStatusPtr);
transQStatus transQTopRemove(QueueHandle_t MyQueue);

