#ifndef NISSQUEUE_H
#define NISSQUEUE_H

#include "types.h"

typedef  enum  {NISSQUEUE_OK, NISSQUEUE_QCB_PNTR_ERR, NISSQUEUE_QELEMENT_MALLOC_ERROR, NISSQUEUE_FULL, NISSQUEUE_EMPTY} nissQueueStatus;


void nissqueueFirst();

#define packedStruct  typedef struct

packedStruct
{
  void *   UserDataPtr;
  uint16_t Length;
} nissqueueEntry_t ;

packedStruct
{
  uint16_t            NumberOfElements;
  nissqueueEntry_t *  HeadPtr;
  nissqueueEntry_t *  TailPtr;
  nissqueueEntry_t *  FirstElementPtr;
  uint16_t      QueuedElements;
}nissqueueControlBlock;

packedStruct
{
  uint8_t         QueueLevel;
  nissQueueStatus LastQueueState;
}nissqueueState;


nissQueueStatus nissqueueInit(uint8_t NumberOfElements, nissqueueControlBlock * QueueControlBlockPtr);
nissQueueStatus nissqueueEnqueue(nissqueueControlBlock * MyQueueCBPtr, void * DataPtr, uint32_t Length);
nissQueueStatus nissqueueDequeue(nissqueueControlBlock * MyQueueCBPtr, nissqueueEntry_t* ReturnedQueueEntryPtr);
nissQueueStatus nissqueueTop(nissqueueControlBlock * MyQueueCBPtr, nissqueueEntry_t* ReturnedQueueEntryPtr);
nissQueueStatus nissqueueStatus(nissqueueControlBlock * MyQueueCBPtr, nissqueueState * ReturnedQStatusPtr);
nissQueueStatus nissqueueTopRemove(nissqueueControlBlock * MyQueueCBPtr);

#endif
