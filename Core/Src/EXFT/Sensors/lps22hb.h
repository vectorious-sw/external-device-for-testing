
#pragma once

#include "sensor.h"
#include "lps22hbRegs.h"
#include "vlapConfig.h"

// the device is LPS22HB_P_0, located on the x-nucleo-iks01a1 expansion board

void lps22hbConfig(void);
uint8_t lps22hbGetDeviceId(void);
void lps22hbGetPressure(I2CREQ *i2cReq);
void lps22hbGetTemp(I2CREQ *i2cReq);
void lps22hbGetPresTemp(I2CREQ *i2cReq);
