#ifndef VLAPCONFIG_H
#define VLAPCONFIG_H


#include "vlapConfig.h"
typedef enum { VLAPMAIN_TASK_CREATE, VLAPMAIN_TASK_STOP, VLAPMAIN_TASK_START} vlapmainDemodulatorTaskControlT;

#define INTERRUPT_PRIORITY_VALID_FROMISR_API (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1)

extern uint16_t adc1DmaBuffer[2][BUFFERSIZE];
void vlapmainDemodulatorTaskControl( vlapmainDemodulatorTaskControlT Control);
ReturnCode_T vmicmodemReceptionQualityGet(uint8_t *ReturnedReceptionQualityPtr);
void vlapmainDebugLog(char * debugString);



#endif 