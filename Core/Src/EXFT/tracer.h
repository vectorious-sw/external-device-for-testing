#ifndef TRACER_H
#define TRACER_H

typedef enum  { TRACER_GENERATE_SW_HW_RESET_JLINK, TRACER_GENERATE_SW_HW_RESET_POWERUP_NOT_EXCEPTION, TRACER_GENERATE_SW_HW_RESET_HARD_FAULT, TRACER_GENERATE_SW_HW_RESET_I2C_TIMEOUT, TRACER_GENERATE_SW_HW_RESET_STACK_OVERFLOW} tracerGeneratedResetSourceCause_T;


#include "protocolApp.h"



void tracerWrite(uint32_t Event);
void tracerInit();

ProtocolappPowerUpEventResetInfo_T* tracerExceptionDataPtrGet();
void tracerHardFaultExceptionImageSave(tracerGeneratedResetSourceCause_T ResetCause);
void tracerSwHwResetGenerate();


#endif 