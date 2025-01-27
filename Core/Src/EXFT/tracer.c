
#include <hwdrivers.h>
#include "protocolapp.h"
#include "inet.h"
#include "comm.h"
#include "events.h"
#include "charger.h"
#include "tracer.h"
#include "common.h"




 ProtocolappPowerUpEventResetInfo_T ProtocolappPowerUpEventResetInfo __attribute__( ( section( ".noinit") ) );



uint32_t TracerArray[2][200]; 
uint16_t TracerIndex;



inline char * get_SP();

void tracerWrite(uint32_t Event)
{
  TracerArray[0][TracerIndex] = hwdriversCyclesCounterGet();
  TracerArray[1][TracerIndex] = Event;
  
  TracerIndex++;
  if(TracerIndex >= sizeof(TracerArray)/(2*sizeof(uint32_t)) )
     TracerIndex = 0;
}

void tracerInit()
{
  TracerIndex = 0;
  hwdriversStartCycleCounter();
}

__attribute__((always_inline)) static inline char* get_SP(void) // TODO: New code, test
{
    register char* result asm("r0");
    asm volatile("MOV %0, SP" : "=r"(result));
    return result;
}


void tracerHardFaultExceptionImageSave(tracerGeneratedResetSourceCause_T ResetCause)
{

#if 1

	// TODO: Commented out since I have to find the corresponding intrinsic for get_SP
	uint32_t i;
  uint32_t * Ptr =  (uint32_t*)((uint32_t)get_SP());


 
   for(i=0; i< sizeof(((ProtocolappPowerUpEventResetInfo_T*)0)->ExceptionImage)/sizeof(uint32_t); i++)
   {
     ((uint32_t*)&ProtocolappPowerUpEventResetInfo.ExceptionImage)[i] = HTONL(*Ptr);
     Ptr-=1;
   }

   ProtocolappPowerUpEventResetInfo.RCC_CSR = RCC->CSR;
   ProtocolappPowerUpEventResetInfo.commState = (uint16_t)commStateGet();
   ProtocolappPowerUpEventResetInfo.eventsState = (uint16_t)eventsStateGet();
   ProtocolappPowerUpEventResetInfo.chargerStatus = (uint16_t)chargerChargingStatusGet();
   ProtocolappPowerUpEventResetInfo.measurementState = (uint16_t)measurementStateGet();
   ProtocolappPowerUpEventResetInfo.ApplicationResetCause = (uint16_t)ResetCause;
   if(ResetCause != TRACER_GENERATE_SW_HW_RESET_POWERUP_NOT_EXCEPTION)
	   strcpy(ProtocolappPowerUpEventResetInfo.TaskName, pcTaskGetName(NULL));
 //  memcpy(&ProtocolappPowerUpEventResetInfo_T.PointersStruct, &eventsLogMemoryPointersStructure, sizeof(eventsLogMemoryPointers_T));

#endif
}


void tracerHardFaultExceptionImageClear()
{
  memset((void*)&ProtocolappPowerUpEventResetInfo, 0, sizeof(ProtocolappPowerUpEventResetInfo_T));
}

 void tracerSwHwResetGenerate()
 {
  // Reset the device
  NVIC_SystemReset();  
}

ProtocolappPowerUpEventResetInfo_T* tracerExceptionDataPtrGet()
{
  return(&ProtocolappPowerUpEventResetInfo);
}
